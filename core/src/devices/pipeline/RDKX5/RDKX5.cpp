/**
 * @file RDKX5.cpp
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/pipeline/RDKX5/RDKX5.h"

#include "devices/pipeline/pipeline.hpp"

#include "base/i2c/i2c.hpp"
#include "base/rectify/rectify.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace dsc {
namespace pipeline {

namespace {
// SC132GS chip ID: keep the register and value in sync with dsc-detect-camera.
// SC132GS 芯片 ID：寄存器和值须与 dsc-detect-camera 一致。
constexpr uint16_t kChipIdReg = 0x3107;
constexpr uint16_t kChipId    = 0x0132;

int64_t trigger_ts_ns(const hbn_frame_info_t &info)
{
    return static_cast<int64_t>(info.trig_tv.tv_sec) * 1000000000LL +
           static_cast<int64_t>(info.trig_tv.tv_usec) * 1000LL;
}

// Approximate mid-exposure from the trigger edge and frame start; fall back to the trigger.
// 用触发沿与帧起点估算曝光中点；无法估算时退回触发时间。
uint64_t frame_ts_ns(const hbn_frame_info_t &info)
{
    const int64_t trigger_ns = trigger_ts_ns(info);
    const int64_t frame_start_ns = static_cast<int64_t>(info.tv.tv_sec) * 1000000000LL +
                                   static_cast<int64_t>(info.tv.tv_usec) * 1000LL;
    if(trigger_ns <= 0 || frame_start_ns <= trigger_ns)return static_cast<uint64_t>(trigger_ns > 0 ? trigger_ns : 0);
    return static_cast<uint64_t>(trigger_ns + (frame_start_ns - trigger_ns) / 2);
}

}

struct Pipeline::Impl {
    bool probed     = false;
    bool inited = false;

    struct CamHw {
        camera_handle_t    cam_fd  = 0;
        hbn_vnode_handle_t vin = 0, isp = 0, vse = 0, gdc = 0;
        hbn_vflow_handle_t vflow    = 0;
        hbn_vnode_handle_t output_node = 0;
        uint32_t           output_chn  = 0;
        hb_mem_common_buf_t gdc_bin{};
        int  mipi_rx = -1, reset_gpio = -1, i2c_bus = -1;
        uint8_t i2c_addr = 0;   // 0 means not found. 0 表示未探测到。
    } cam[static_cast<std::size_t>(CamIndex::Num)];

    // mid is the VSE input: GDC output when present, otherwise sensor size (pixels).
    // mid 是 VSE 输入，单位为像素：有 GDC 时取其输出尺寸，否则取传感器尺寸。
    uint32_t input_w = 0, input_h = 0, output_w = 0, output_h = 0;
    uint32_t mid_w = 0, mid_h = 0, vse_chn = 0;
    int install_angle = 0;
};

Pipeline::Pipeline(
    uint8_t left_addr, uint8_t right_addr,
    const uint8_t *bus_list, std::size_t bus_num)
    : impl_(std::make_unique<Impl>())
{
    for(std::size_t i = 0; i < bus_num; ++i){
        const uint8_t bus = bus_list[i];

        if(impl_->cam[static_cast<std::size_t>(CamIndex::Right)].i2c_addr == 0){
            base::I2cDevice dev(bus, right_addr);
            uint16_t id = 0;
            if(dev && dev.read16(kChipIdReg, &id) == Status::Ok && id == kChipId){
                impl_->cam[static_cast<std::size_t>(CamIndex::Right)].i2c_bus  = bus;
                impl_->cam[static_cast<std::size_t>(CamIndex::Right)].i2c_addr = right_addr;
            }
            dev.close();
        }
        if(impl_->cam[static_cast<std::size_t>(CamIndex::Left)].i2c_addr == 0){
            base::I2cDevice dev(bus, left_addr);
            uint16_t id = 0;
            if(dev && dev.read16(kChipIdReg, &id) == Status::Ok && id == kChipId){
                impl_->cam[static_cast<std::size_t>(CamIndex::Left)].i2c_bus  = bus;
                impl_->cam[static_cast<std::size_t>(CamIndex::Left)].i2c_addr = left_addr;
            }
            dev.close();
        }
    }

    impl_->probed = (impl_->cam[static_cast<std::size_t>(CamIndex::Right)].i2c_addr != 0 &&
                     impl_->cam[static_cast<std::size_t>(CamIndex::Left)].i2c_addr != 0);
}

Pipeline::~Pipeline()
{
    deinit();
}

Pipeline::operator bool() const { return impl_ && impl_->probed; }

Status Pipeline::init(const PipelineConfig &cfg, StereoImuModel *cal)
{
    if(!impl_->probed)return Status::NotFound;
    if(impl_->inited)return Status::ParamError;

    hb_mem_module_open();

    impl_->input_w = cfg.sensor_width;
    impl_->input_h = cfg.sensor_height;
    impl_->output_w = cfg.output_width;
    impl_->output_h = cfg.output_height;

    // Raw bypasses GDC and VSE, so the size stays native despite ISP conversion to NV12.
    // Raw 跳过 GDC 和 VSE；ISP 转换为 NV12，但保留传感器原尺寸。
    if(cfg.mode == OutputMode::Raw)
        if(cfg.sensor_width != cfg.output_width || cfg.sensor_height != cfg.output_height)
            return Status::ParamError;

    // The probed I2C bus determines the wired MIPI receiver and reset GPIO.
    // 根据探测到的 I2C 总线确定 MIPI 接收端和复位 GPIO。
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        const int bus = impl_->cam[i].i2c_bus;
        if(bus < 0 || bus >= 32 || cfg.bus_mipi_rx[bus] == 0xFF)return Status::ParamError;

        impl_->cam[i].mipi_rx    = cfg.bus_mipi_rx[bus];
        impl_->cam[i].reset_gpio = cfg.bus_reset_gpio[bus];
    }

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        // Fixed SC132GS link clock, settle and MCLK, independent of frame size.
        // SC132GS 的链路时钟、settle 和 MCLK 固定，不随帧尺寸变化。
        int ret = camera_open(
            &impl_->cam[i].cam_fd,
            impl_->cam[i].i2c_addr,
            impl_->input_w, impl_->input_h, cfg.fps,
            cfg.line_length, cfg.frame_length,
            1200, 20, 1, cfg.tuning_file);

        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        int ret = vin_open(
            &impl_->cam[i].vin,
            impl_->cam[i].mipi_rx,
            impl_->input_w, impl_->input_h, cfg.fps);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        int ret = isp_open(
            &impl_->cam[i].isp,
            impl_->input_w, impl_->input_h);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    if(cfg.mode != OutputMode::Raw){
        if(cal == nullptr)return Status::ParamError;

        // The GDC map supports only multiples of 90°.
        // GDC 映射只支持 90 度的整数倍。
        impl_->install_angle = ((cal->install_angle % 360) + 360) % 360;
        if(impl_->install_angle % 90 != 0) return Status::ParamError;

        if(cfg.mode == OutputMode::Rect || impl_->install_angle != 0){
            // Build maps in the installed orientation: 90° and 270° swap the sensor axes.
            // 按安装朝向生成映射；90 度和 270 度时交换传感器的宽高。
            const bool swap = (impl_->install_angle == 90 || impl_->install_angle == 270);
            const uint32_t src_w = swap ? impl_->input_h : impl_->input_w;
            const uint32_t src_h = swap ? impl_->input_w : impl_->input_h;

            std::vector<RemapPoint> map[static_cast<std::size_t>(CamIndex::Num)];
            if(cfg.mode == OutputMode::Rect){
                Status st = base::stereo_rectify(cal, src_w, src_h,
                                                 &impl_->mid_w, &impl_->mid_h,
                                                 &map[static_cast<std::size_t>(CamIndex::Left)],
                                                 &map[static_cast<std::size_t>(CamIndex::Right)]);
                if(st != Status::Ok){
                    deinit();
                    return Status::Unsupported;
                }
            }
            // An identity map lets GDC apply rotation only, with no rectification.
            // 使用恒等映射，让 GDC 只做旋转，不做校正。
            else{
                impl_->mid_w = src_w;
                impl_->mid_h = src_h;

                const uint32_t n = impl_->mid_w * impl_->mid_h;
                for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
                    map[i].resize(n);
                    for(uint32_t p = 0; p < n; p++){
                        map[i][p].x = p % impl_->mid_w;
                        map[i][p].y = p / impl_->mid_w;
                    }
                }
            }

            for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
                int ret = gdc_open(
                    &impl_->cam[i].gdc, &impl_->cam[i].gdc_bin,
                    map[i].data(), impl_->input_w, impl_->input_h,
                    impl_->mid_w, impl_->mid_h, impl_->install_angle);
                if(ret){
                    deinit();
                    return Status::HwError;
                }
            }
        }
        else{
            impl_->mid_w = impl_->input_w;
            impl_->mid_h = impl_->input_h;
        }
    }

    if(cfg.mode != OutputMode::Raw){
        if(roi_ratio_exact(impl_->mid_w, impl_->mid_h,
                           impl_->output_w, impl_->output_h) != 0){
            deinit();
            return Status::Unsupported;
        }

        // On X5, VSE channel 5 upsizes; channel 0 keeps size or downsizes.
        // X5 的 VSE 通道 5 用于放大，通道 0 用于原尺寸或缩小。
        const common_rect_t roi = aspect_roi(impl_->mid_w, impl_->mid_h,
                                             impl_->output_w, impl_->output_h);
        impl_->vse_chn = (impl_->output_w > roi.w || impl_->output_h > roi.h) ? 5 : 0;

        for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
            int ret = vse_open(
                &impl_->cam[i].vse, impl_->mid_w, impl_->mid_h,
                impl_->output_w, impl_->output_h, impl_->vse_chn);
            if(ret){
                deinit();
                return Status::HwError;
            }
        }

        // Update both cameras' intrinsics from VSE input coordinates to output coordinates.
        // 将双目内参从 VSE 输入坐标系换算到输出坐标系。
        const double sfx = static_cast<double>(impl_->output_w) / roi.w;
        const double sfy = static_cast<double>(impl_->output_h) / roi.h;
        for(CameraIntrinsics *k : {&cal->cam_left, &cal->cam_right}){
            k->fx = k->fx * sfx;
            k->fy = k->fy * sfy;
            k->cx = (k->cx - roi.x) * sfx;
            k->cy = (k->cy - roi.y) * sfy;
            k->K[0] = k->fx; k->K[2] = k->cx;
            k->K[4] = k->fy; k->K[5] = k->cy;
        }
    }

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        int ret = vflow_build(
            &impl_->cam[i].vflow, impl_->cam[i].cam_fd,
            impl_->cam[i].vin, impl_->cam[i].isp,
            impl_->cam[i].gdc, impl_->cam[i].vse,
            impl_->vse_chn,
            &impl_->cam[i].output_node, &impl_->cam[i].output_chn);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    impl_->inited = true;
    return Status::Ok;
}

void Pipeline::deinit()
{
    if(!impl_->inited)
        return;

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        teardown_cam(impl_->cam[i].vflow, impl_->cam[i].cam_fd,
                     impl_->cam[i].vin, impl_->cam[i].isp,
                     impl_->cam[i].vse, impl_->cam[i].gdc,
                     &impl_->cam[i].gdc_bin, impl_->cam[i].reset_gpio);
        // The handles passed to teardown_cam() are by value; clear the stored copies here.
        // teardown_cam() 按值接收句柄；这里清空保存的句柄。
        impl_->cam[i].cam_fd = 0;
        impl_->cam[i].vin = impl_->cam[i].isp = impl_->cam[i].vse = impl_->cam[i].gdc = 0;
        impl_->cam[i].vflow = 0;
        impl_->cam[i].output_node = 0;
        impl_->cam[i].output_chn  = 0;
    }

    // Reset after both flows are gone so controlled sensors can be probed again.
    // 两路采集流销毁后再复位，让受控传感器可再次被探测。
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        if(impl_->cam[i].reset_gpio >= 0)
            sensor_power(impl_->cam[i].reset_gpio, 1);
    }

    impl_->inited = false;
    hb_mem_module_close();
}

Status Pipeline::start(CamIndex first)
{
    if(!impl_->inited)
        return Status::ParamError;

    const std::size_t a = static_cast<std::size_t>(first);
    const std::size_t b = static_cast<std::size_t>(
        first == CamIndex::Right ? CamIndex::Left : CamIndex::Right);

    if(hbn_vflow_start(impl_->cam[a].vflow) != 0){
        deinit();
        return Status::HwError;
    }
    if(hbn_vflow_start(impl_->cam[b].vflow) != 0){
        deinit();
        return Status::HwError;
    }
    return Status::Ok;
}

void Pipeline::stop()
{
    if(!impl_->inited)
        return;

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++)
        if(impl_->cam[i].vflow != 0)
            hbn_vflow_stop(impl_->cam[i].vflow);
}

Status Pipeline::get_frame(CamIndex idx,
                           uint8_t *y, uint8_t *uv,
                           uint32_t width, uint32_t height,
                           uint32_t y_stride, uint32_t uv_stride,
                           uint64_t *timestamp_ns,
                           uint64_t *trigger_ns,
                           uint32_t timeout_ms)
{
    if(y == nullptr || uv == nullptr || timestamp_ns == nullptr || trigger_ns == nullptr ||
       !impl_->inited)
        return Status::ParamError;

    const std::size_t i = static_cast<std::size_t>(idx);
    auto &c = impl_->cam[i];

    hbn_vnode_image_t img;
    memset(&img, 0, sizeof(img));
    if(hbn_vnode_getframe(c.output_node, c.output_chn, timeout_ms, &img) != 0)
        return Status::Timeout;

    if((uint32_t)img.buffer.width != width || (uint32_t)img.buffer.height != height){
        hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
        return Status::ParamError;
    }

    const int64_t trigger_value_ns = trigger_ts_ns(img.info);
    const uint64_t ts_ns = frame_ts_ns(img.info);
    if(ts_ns == 0){
        hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
        return Status::HwError;
    }

    for(uint32_t r = 0; r < height; ++r)
        memcpy(y + (std::size_t)r * y_stride,
               img.buffer.virt_addr[0] + (std::size_t)r * img.buffer.stride, width);
    for(uint32_t r = 0; r < height / 2; ++r)
        memcpy(uv + (std::size_t)r * uv_stride,
               img.buffer.virt_addr[1] + (std::size_t)r * img.buffer.stride, width);

    *timestamp_ns = ts_ns;
    *trigger_ns = static_cast<uint64_t>(trigger_value_ns > 0 ? trigger_value_ns : 0);

    hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
    return Status::Ok;
}

}
}
