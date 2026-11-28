/**
 * @file RDKS600.cpp
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/pipeline/RDKS600/RDKS600.h"

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
// Use the same SC132GS ID register and value as dsc-detect-camera.
// SC132GS 的 ID 寄存器和值与 dsc-detect-camera 一致。
constexpr uint16_t kChipIdReg = 0x3107;
constexpr uint16_t kChipId    = 0x0132;

// The BSP uses ISP/PYM hw_id 3 for direct sensors, 0 for GMSL.
// BSP 为直连传感器分配 ISP/PYM hw_id 3，为 GMSL 传感器分配 0。
constexpr uint32_t kIspHwId = 3;
constexpr uint32_t kPymHwId = 3;

// Start at slot 4 (BSP isp0_next_slot_id); starting at 0 left the nodes unbound.
// 从槽位 4（BSP 的 isp0_next_slot_id 起始值）分配；从 0 开始会导致节点绑定不上。
constexpr uint32_t kIspSlotBase = 4;

int64_t trigger_ts_ns(const hbn_frame_info_t &info)
{
    return static_cast<int64_t>(info.trig_tv.tv_sec) * 1000000000LL +
           static_cast<int64_t>(info.trig_tv.tv_usec) * 1000LL;
}

// Estimate the exposure midpoint from the trigger and frame-start times.
// The IMU tracker uses the trigger edge itself.
// 用触发时间和帧开始时间估算曝光中点；IMU 跟踪器使用触发沿本身。
uint64_t frame_ts_ns(const hbn_frame_info_t &info)
{
    const int64_t trigger_ns = trigger_ts_ns(info);
    const int64_t frame_start_ns = static_cast<int64_t>(info.tv.tv_sec) * 1000000000LL +
                                   static_cast<int64_t>(info.tv.tv_usec) * 1000LL;
    if(trigger_ns <= 0 || frame_start_ns <= trigger_ns)return static_cast<uint64_t>(trigger_ns > 0 ? trigger_ns : 0);
    return static_cast<uint64_t>(trigger_ns + (frame_start_ns - trigger_ns) / 2);
}


} // namespace

struct Pipeline::Impl {
    bool probed     = false;
    bool inited = false;
    bool hb_mem_opened = false;

    struct CamHw {
        camera_handle_t    cam_fd  = 0;
        hbn_vnode_handle_t vin = 0, isp = 0, pym = 0, gdc = 0;
        hbn_vflow_handle_t vflow    = 0;
        hbn_vnode_handle_t output_node = 0;
        uint32_t           output_chn  = 0;
        uint32_t           isp_slot    = 0;
        hb_mem_common_buf_t gdc_bin{};
        int  mipi_rx = -1, reset_gpio = -1, i2c_bus = -1;
        uint8_t i2c_addr = 0;
    } cam[static_cast<std::size_t>(CamIndex::Num)];

    // output_roi is in PYM input coordinates; it is used to update the intrinsics.
    // output_roi 以 PYM 输入帧为基准，用于更新内参。
    uint32_t input_w = 0, input_h = 0, output_w = 0, output_h = 0;
    uint32_t mid_w = 0, mid_h = 0;
    uint32_t pym_in_w = 0, pym_in_h = 0;
    dsc_rect_t output_roi{};
    int install_angle = 0;   // Degrees / 单位：度
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
    impl_->hb_mem_opened = true;

    // Reject zero dimensions before the ROI calculations, which divide by the output dimensions.
    // ROI 计算会除以输出宽高，因此先拒绝尺寸为 0 的配置。
    if(cfg.sensor_width == 0 || cfg.sensor_height == 0 ||
       cfg.output_width == 0 || cfg.output_height == 0){
        deinit();
        return Status::ParamError;
    }

    impl_->input_w = cfg.sensor_width;
    impl_->input_h = cfg.sensor_height;
    impl_->output_w = cfg.output_width;
    impl_->output_h = cfg.output_height;

    // Raw uses ISP NV12 output at sensor size, without scaling or rectification.
    // Raw 直接取传感器尺寸的 ISP NV12 输出，不缩放、不校正。
    if(cfg.mode == OutputMode::Raw)
        if(cfg.sensor_width != cfg.output_width || cfg.sensor_height != cfg.output_height){
            deinit();
            return Status::ParamError;
        }

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        const int bus = impl_->cam[i].i2c_bus;
        if(bus < 0 || bus >= 32 || cfg.bus_mipi_rx[bus] == 0xFF){
            deinit();
            return Status::ParamError;
        }

        impl_->cam[i].mipi_rx    = cfg.bus_mipi_rx[bus];
        impl_->cam[i].reset_gpio = cfg.bus_reset_gpio[bus];
        impl_->cam[i].isp_slot   = kIspSlotBase + static_cast<uint32_t>(i);
    }

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        // Use the S600 dual-sensor preset for MIPI timing; X5 uses a link clock of 1200.
        // MIPI 时序沿用 S600 双目配置；X5 的链路时钟为 1200。
        int ret = camera_open(
            &impl_->cam[i].cam_fd,
            impl_->cam[i].i2c_addr,
            impl_->input_w, impl_->input_h, cfg.fps,
            cfg.line_length, cfg.frame_length,
            2400, 20, 1, cfg.tuning_file);

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
            impl_->input_w, impl_->input_h,
            kIspHwId, impl_->cam[i].isp_slot, cfg.fps);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    impl_->pym_in_w = impl_->input_w;
    impl_->pym_in_h = impl_->input_h;

    if(cfg.mode == OutputMode::Raw){
        impl_->mid_w = impl_->input_w;
        impl_->mid_h = impl_->input_h;
    }
    else{
        if(cal == nullptr)return Status::ParamError;

        impl_->install_angle = ((cal->install_angle % 360) + 360) % 360;
        if(impl_->install_angle % 90 != 0) return Status::ParamError;

        // Build maps in the installed orientation: 90° and 270° swap the axes.
        // 映射按安装朝向生成；90° 和 270° 时交换宽高。
        const bool swap = (impl_->install_angle == 90 || impl_->install_angle == 270);

        if(cfg.mode == OutputMode::Rect){
            const uint32_t src_w = swap ? impl_->input_h : impl_->input_w;
            const uint32_t src_h = swap ? impl_->input_w : impl_->input_h;
            uint32_t rect_w = 0, rect_h = 0;

            std::vector<RemapPoint> map[static_cast<std::size_t>(CamIndex::Num)];
            Status st = base::stereo_rectify(cal, src_w, src_h, &rect_w, &rect_h,
                                             &map[static_cast<std::size_t>(CamIndex::Left)],
                                             &map[static_cast<std::size_t>(CamIndex::Right)]);
            if(st != Status::Ok){
                deinit();
                return Status::Unsupported;
            }

            // Rect feeds PYM from the full rectification grid, which can exceed the
            // sensor size (e.g. 1088x1280 -> 1088x1536).
            // Rect 将完整校正网格交给 PYM，网格可能大于传感器画幅（如 1088x1280 -> 1088x1536）。
            for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
                int ret = gdc_open(
                    &impl_->cam[i].gdc, &impl_->cam[i].gdc_bin,
                    map[i].data(), impl_->input_w, impl_->input_h,
                    rect_w, rect_h, impl_->install_angle);
                if(ret){
                    deinit();
                    return Status::HwError;
                }
            }

            impl_->pym_in_w = rect_w;
            impl_->pym_in_h = rect_h;
            impl_->mid_w    = impl_->output_w;
            impl_->mid_h    = impl_->output_h;
            impl_->output_roi = aspect_roi(rect_w, rect_h,
                                           impl_->output_w, impl_->output_h);
        }
        else if(impl_->install_angle != 0){
            impl_->mid_w = swap ? impl_->output_h : impl_->output_w;
            impl_->mid_h = swap ? impl_->output_w : impl_->output_h;

            // Use an identity map for rotation without rectification.
            // 用恒等映射只做旋转，不做校正。
            std::vector<RemapPoint> map[static_cast<std::size_t>(CamIndex::Num)];
            const uint32_t n = impl_->output_w * impl_->output_h;
            for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
                map[i].resize(n);
                for(uint32_t p = 0; p < n; p++){
                    map[i][p].x = p % impl_->output_w;
                    map[i][p].y = p / impl_->output_w;
                }
            }

            for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
                int ret = gdc_open(
                    &impl_->cam[i].gdc, &impl_->cam[i].gdc_bin,
                    map[i].data(), impl_->mid_w, impl_->mid_h,
                    impl_->output_w, impl_->output_h, impl_->install_angle);
                if(ret){
                    deinit();
                    return Status::HwError;
                }
            }
        }
        else{
            impl_->mid_w = impl_->output_w;
            impl_->mid_h = impl_->output_h;
        }

        if(cfg.mode != OutputMode::Rect)
            impl_->output_roi = aspect_roi(impl_->input_w, impl_->input_h,
                                           impl_->mid_w, impl_->mid_h);
    }

    if(cfg.mode != OutputMode::Raw){
        // Check the crop ratio here; the PYM driver validates the requested size.
        // 这里只检查裁剪比例，输出尺寸是否支持由 PYM 驱动校验。
        if(roi_ratio_exact(impl_->pym_in_w, impl_->pym_in_h,
                           impl_->mid_w, impl_->mid_h) != 0){
            deinit();
            return Status::Unsupported;
        }

        for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
            int ret = pym_open(
                &impl_->cam[i].pym, impl_->pym_in_w, impl_->pym_in_h,
                impl_->mid_w, impl_->mid_h,
                kPymHwId, impl_->cam[i].isp_slot);
            if(ret){
                deinit();
                return Status::HwError;
            }
        }

        const dsc_rect_t roi = impl_->output_roi;
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

    // After VIN -> ISP: Raw reads ISP; Resize reads PYM (GDC if rotated);
    // Rect reads PYM after GDC.
    // VIN -> ISP 之后：Raw 从 ISP 取帧；Resize 从 PYM 取帧，旋转时改从 GDC 取帧；
    // Rect 先经 GDC，再从 PYM 取帧。
    const int gdc_before_pym = (cfg.mode == OutputMode::Rect) ? 1 : 0;
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        int ret = vflow_build(
            &impl_->cam[i].vflow, impl_->cam[i].cam_fd,
            impl_->cam[i].vin, impl_->cam[i].isp,
            impl_->cam[i].gdc, impl_->cam[i].pym,
            0, gdc_before_pym,
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
    // Partial init can leave live handles even when inited is false.
    // 初始化未完成时，即使 inited 为 false，也可能已有有效句柄。

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        // Without a flow, close the nodes here. A flow only owns nodes already added to it.
        // 尚无 flow 时在此关闭节点。flow 只接管已加入的节点。
        if(impl_->cam[i].vflow == 0){
            if(impl_->cam[i].vin)hbn_vnode_close(impl_->cam[i].vin);
            if(impl_->cam[i].isp)hbn_vnode_close(impl_->cam[i].isp);
            if(impl_->cam[i].pym)hbn_vnode_close(impl_->cam[i].pym);
            if(impl_->cam[i].gdc)hbn_vnode_close(impl_->cam[i].gdc);
        }
        teardown_cam(impl_->cam[i].vflow, impl_->cam[i].cam_fd,
                     impl_->cam[i].vin, impl_->cam[i].isp,
                     impl_->cam[i].pym, impl_->cam[i].gdc,
                     &impl_->cam[i].gdc_bin, impl_->cam[i].reset_gpio);
        impl_->cam[i].cam_fd = 0;
        impl_->cam[i].vin = impl_->cam[i].isp = impl_->cam[i].pym = impl_->cam[i].gdc = 0;
        impl_->cam[i].vflow = 0;
        impl_->cam[i].output_node = 0;
        impl_->cam[i].output_chn  = 0;
    }

    // Reset after teardown so the sensors can be probed again.
    // 销毁采集流后复位传感器，使其可以再次被探测。
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        if(impl_->cam[i].reset_gpio >= 0)
            sensor_power(impl_->cam[i].reset_gpio, 1);
    }

    impl_->inited = false;
    if(impl_->hb_mem_opened){
        hb_mem_module_close();
        impl_->hb_mem_opened = false;
    }
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

    // On S600, ISP (stream_group=1) and PYM require group fetches; single-image
    // fetches time out. GDC delivers single images.
    // S600 的 ISP（stream_group=1）和 PYM 必须按组取帧，单帧接口会超时。GDC 按单帧输出。
    hbn_vnode_image_t img;
    hbn_vnode_image_group_t grp;
    memset(&img, 0, sizeof(img));
    memset(&grp, 0, sizeof(grp));

    const bool grouped = (c.output_node == c.pym || c.output_node == c.isp);
    if(grouped){
        if(hbn_vnode_getframe_group(c.output_node, c.output_chn, timeout_ms, &grp) != 0)
            return Status::Timeout;
    }
    else if(hbn_vnode_getframe(c.output_node, c.output_chn, timeout_ms, &img) != 0){
        return Status::Timeout;
    }

    const hb_mem_graphic_buf_t &buf =
        grouped ? grp.buf_group.graph_group[c.output_chn] : img.buffer;

    // Invalidate CPU cache lines before reading buffers written by hardware.
    // 读取硬件写入的缓冲区前，先使对应的 CPU 缓存行失效。
    if(buf.virt_addr[0] != nullptr)
        hb_mem_invalidate_buf_with_vaddr((uint64_t)buf.virt_addr[0], buf.size[0]);
    if(buf.plane_cnt > 1 && buf.virt_addr[1] != nullptr)
        hb_mem_invalidate_buf_with_vaddr((uint64_t)buf.virt_addr[1], buf.size[1]);

    if((uint32_t)buf.width != width || (uint32_t)buf.height != height){
        if(grouped)hbn_vnode_releaseframe_group(c.output_node, c.output_chn, &grp);
        else       hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
        return Status::ParamError;
    }

    // Without an LPWM edge, the frame cannot be paired with IMU samples. Do not substitute another clock.
    // 缺少 LPWM 触发沿就无法与 IMU 采样对齐，不能用其他时钟补时间戳。
    const int64_t trigger_value_ns = trigger_ts_ns(grouped ? grp.info : img.info);
    const uint64_t ts_ns = frame_ts_ns(grouped ? grp.info : img.info);
    if(ts_ns == 0){
        if(grouped)hbn_vnode_releaseframe_group(c.output_node, c.output_chn, &grp);
        else       hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
        return Status::HwError;
    }

    for(uint32_t r = 0; r < height; ++r)
        memcpy(y + (std::size_t)r * y_stride,
               buf.virt_addr[0] + (std::size_t)r * buf.stride, width);
    if(buf.plane_cnt > 1 && buf.virt_addr[1] != nullptr)
        for(uint32_t r = 0; r < height / 2; ++r)
            memcpy(uv + (std::size_t)r * uv_stride,
                   buf.virt_addr[1] + (std::size_t)r * buf.stride, width);

    *timestamp_ns = ts_ns;
    *trigger_ns = static_cast<uint64_t>(trigger_value_ns > 0 ? trigger_value_ns : 0);

    if(grouped)hbn_vnode_releaseframe_group(c.output_node, c.output_chn, &grp);
    else       hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
    return Status::Ok;
}

} // namespace pipeline
} // namespace dsc
