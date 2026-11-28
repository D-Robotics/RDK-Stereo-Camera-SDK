/**
 * @file dsc.cpp
 * @brief C API for capture and calibration.
 * @brief 采集与标定的 C 接口。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "dstereo_cam.h"

#include "types.hpp"

#include "base/fifo/fifo.hpp"
#include "base/tracker/tracker.hpp"

#include "devices/eeprom/eeprom.hpp"
#include "devices/imu/imu.hpp"
#include "devices/pipeline/pipeline.hpp"

#include <array>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <utility>

using namespace dsc;

static dsc_err_t to_err(Status st)
{
    switch (st) {
    case Status::Ok:          return DSC_OK;
    case Status::ParamError:  return DSC_PARAM_ERROR;
    case Status::Unsupported: return DSC_UNSUPPORTED;
    case Status::NotFound:    return DSC_NOT_FOUND;
    case Status::HwError:     return DSC_HW_ERROR;
    case Status::Timeout:     return DSC_TIMEOUT;
    case Status::ThreadClosed:return DSC_THREAD_CLOSED;
    }
    return DSC_HW_ERROR;
}

struct dsc_device_s {
    std::mutex              mtx;   // Serializes public API operations; worker threads do not acquire it. 串行化公共 API 操作；工作线程不获取该锁。

    std::unique_ptr<eeprom::Eeprom>     eeprom;
    std::unique_ptr<imu::Imu>           imu;
    std::unique_ptr<pipeline::Pipeline> pipeline;

    std::unique_ptr<
        base::Fifo<
            std::array<
                dsc_image_nv12_t, static_cast<std::size_t>(CamIndex::Num)
            >>> camera_fifo;
    std::unique_ptr<base::Fifo<dsc_imu_packet_t>> imu_fifo;

    StereoImuModel       cal_internal;

    std::thread              camera_thread;
    std::thread              imu_thread;
    // With an IMU, wait for a batch ending in FSYNC before starting camera capture.
    // 有 IMU 时，等到某批数据的末包带 FSYNC 标记后再启动相机采集。
    std::atomic<bool>        camera_on{false};
    // Runtime faults stay latched until deinit().
    // 运行中的故障状态保留到 deinit()。
    std::atomic<Status>      state{Status::ThreadClosed};

    // The camera and IMU threads share the timestamp tracker.
    // 相机和 IMU 线程共用时间戳跟踪器。
    std::mutex tracker_mtx;
    std::unique_ptr<base::TimestampTracker> tracker;

    uint64_t imu_group_delay_ns = 0;
    uint32_t output_w = 0, output_h = 0;
    dsc_stereo_layout_t stereo_layout = DSC_STEREO_LAYOUT_NONE;
    uint32_t fps = 0;
    uint16_t accel_fsr_g = 0, gyro_fsr_dps = 0;
    CamIndex fsync_camera = CamIndex::Right;
};

/* ---- Calibration helpers (3x3 row-major) 标定辅助函数（3x3 行主序） ---- */

static void mat33_mul(const double *A, const double *B, double *C)   // C = A * B（矩阵相乘）
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            C[i*3+j] = A[i*3+0]*B[0*3+j] + A[i*3+1]*B[1*3+j] + A[i*3+2]*B[2*3+j];
}

static void mat33_t_mul(const double *A, const double *B, double *C)   // C = A^T * B（A 的转置乘以 B）
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            C[i*3+j] = A[0*3+i]*B[0*3+j] + A[1*3+i]*B[1*3+j] + A[2*3+i]*B[2*3+j];
}

static void mat33_mul_t(const double *A, const double *B, double *C)   // C = A * B^T（A 乘以 B 的转置）
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            C[i*3+j] = A[i*3+0]*B[j*3+0] + A[i*3+1]*B[j*3+1] + A[i*3+2]*B[j*3+2];
}

static void mat33_mul_vec(const double *A, const double *v, double *out)   // out = A * v（矩阵乘以向量）
{
    for(int i = 0; i < 3; i++)
        out[i] = A[i*3+0]*v[0] + A[i*3+1]*v[1] + A[i*3+2]*v[2];
}

static void mat33_t_mul_vec(const double *A, const double *v, double *out)   // out = A^T * v（A 的转置乘以向量）
{
    for(int i = 0; i < 3; i++)
        out[i] = A[0*3+i]*v[0] + A[1*3+i]*v[1] + A[2*3+i]*v[2];
}

static std::pair<double*, double*> frame_extrinsics(dsc_device_t *dev, dsc_reference_frame_t frame)
{
    switch(frame){
        case DSC_REF_CAMERA_RIGHT: return {dev->cal_internal.cam_right_R, dev->cal_internal.cam_right_T};
        case DSC_REF_CAMERA_LEFT:  return {dev->cal_internal.cam_left_R,  dev->cal_internal.cam_left_T};
        case DSC_REF_IMU:          return {dev->cal_internal.imu_R,       dev->cal_internal.imu_T};
        default:                     return {nullptr, nullptr};
    }
}

/* ---- Background threads 后台线程 ---- */

static void imu_thread_func(dsc_device_t *dev)
{
    const double accel_scale = (double)dev->accel_fsr_g * 9.80665 / 32768.0;
    const double gyro_scale  = (double)dev->gyro_fsr_dps * 3.14159265358979323846 / 180.0 / 32768.0;

    ImuHwFifo16Packet raw[128]; // hardware FIFO holds up to 128 packets 硬件 FIFO 最多可容纳 128 个数据包
    bool camera_released = false;

    while(dev->state.load() == Status::Ok) {
        if(dev->imu->full()){dev->state.store(Status::HwError);return ;}

        std::size_t n = 0;
        Status st = dev->imu->read(raw, 128, &n);
        if(st != Status::Ok || n == 0) { usleep(1000); continue; }

        if(!camera_released){
            if(raw[n - 1].is_fsync){
                dev->camera_on.store(true);
                camera_released = true;
            }
            continue;
        }

        for(std::size_t i = 0; i < n; ++i){
            const auto &p = raw[i];
            uint64_t delta_ns = static_cast<uint64_t>(p.delta_time_us) * 1000;
            uint64_t timestamp_ns;
            {
                std::lock_guard<std::mutex> lock(dev->tracker_mtx);
                timestamp_ns = dev->tracker->get_slave_timestamp(p.is_fsync, p.is_fsync ? &delta_ns : nullptr);
            }
            if(timestamp_ns <= dev->imu_group_delay_ns)continue;

            dsc_imu_packet_t pkt{};
            for(std::size_t axis = 0; axis < 3; ++axis){
                pkt.accel[axis] = static_cast<float>(p.accel[axis] * accel_scale);
                pkt.gyro[axis] = static_cast<float>(p.gyro[axis] * gyro_scale);
            }
            pkt.temp = static_cast<float>(p.temp) / 2.07f + 25.0f;
            pkt.is_fsync = p.is_fsync;
            // Compensate the published sample, not the tracker's FSYNC timebase.
            // 补偿对外发布的采样，而不是跟踪器的 FSYNC 时基。
            pkt.timestamp_ns = timestamp_ns - dev->imu_group_delay_ns;
            dev->imu_fifo->push(pkt);
        }
        if(dev->state.load() != Status::Ok)break;
    }
}

static void free_frame_pair(std::array<dsc_image_nv12_t, static_cast<std::size_t>(CamIndex::Num)> &pair)
{
    for(auto &image : pair) {
        free(image.data);
        image.data = nullptr;
    }
}

static void camera_thread_func(dsc_device_t *dev)
{
    while(dev->state.load() == Status::Ok && !dev->camera_on.load())usleep(1000);
    if(dev->state.load() != Status::Ok)return ;

    const Status st_start = dev->pipeline->start(dev->fsync_camera);
    if(st_start != Status::Ok){dev->state.store(st_start);return ;}

    const uint64_t cycle_ns = 1000000000ULL / dev->fps / 2;
    const size_t R = static_cast<std::size_t>(CamIndex::Right);
    const size_t L = static_cast<std::size_t>(CamIndex::Left);

    while(dev->state.load() == Status::Ok){
        std::array<dsc_image_nv12_t, static_cast<std::size_t>(CamIndex::Num)> frame{};
        Status st;
        uint64_t ts[static_cast<std::size_t>(CamIndex::Num)] = {0, 0};
        uint64_t trig[static_cast<std::size_t>(CamIndex::Num)] = {0, 0};
        uint8_t *y[static_cast<std::size_t>(CamIndex::Num)]  = {nullptr, nullptr};
        uint8_t *uv[static_cast<std::size_t>(CamIndex::Num)] = {nullptr, nullptr};
        uint32_t stride;
        const uint32_t width  = dev->output_w;
        const uint32_t height = dev->output_h;

        // Timestamp frames at the estimated exposure midpoint; anchor the tracker on the FSYNC trigger.
        // 帧时间戳取估算的曝光中点，跟踪器仍以 FSYNC 触发沿为锚。
        const CamIndex F = dev->fsync_camera;
        const CamIndex O = (F == CamIndex::Right) ? CamIndex::Left : CamIndex::Right;
        const size_t fi = static_cast<std::size_t>(F);
        const size_t oi = static_cast<std::size_t>(O);

        switch(dev->stereo_layout){
            case DSC_STEREO_LAYOUT_NONE:
                stride = width;
                y[R] = (uint8_t *)malloc((std::size_t)width * height * 3 / 2);
                y[L] = (uint8_t *)malloc((std::size_t)width * height * 3 / 2);
                frame[R].data = y[R];
                frame[L].data = y[L];
                if(!y[R] || !y[L])goto free_frame;
                uv[R] = y[R] + (std::size_t)width * height;
                uv[L] = y[L] + (std::size_t)width * height;
                frame[R].width = width; frame[R].height = height;
                frame[L].width = width; frame[L].height = height;
                break;
            case DSC_STEREO_LAYOUT_LEFT_RIGHT:
                stride = width * 2;
                y[L] = (uint8_t *)malloc((std::size_t)width * height * 3);
                frame[0].data = y[L];
                if(!y[L])goto free_frame;
                y[R]  = y[L] + width;
                uv[L] = y[L] + (std::size_t)width * height * 2;
                uv[R] = uv[L] + width;
                frame[0].width = width * 2; frame[0].height = height;
                break;
            case DSC_STEREO_LAYOUT_RIGHT_LEFT:
                stride = width * 2;
                y[R] = (uint8_t *)malloc((std::size_t)width * height * 3);
                frame[0].data = y[R];
                if(!y[R])goto free_frame;
                y[L]  = y[R] + width;
                uv[R] = y[R] + (std::size_t)width * height * 2;
                uv[L] = uv[R] + width;
                frame[0].width = width * 2; frame[0].height = height;
                break;
            case DSC_STEREO_LAYOUT_TOP_BOTTOM:
                stride = width;
                y[L] = (uint8_t *)malloc((std::size_t)width * height * 3);
                frame[0].data = y[L];
                if(!y[L])goto free_frame;
                y[R]  = y[L] + (std::size_t)width * height;
                uv[L] = y[L] + (std::size_t)width * height * 2;
                uv[R] = uv[L] + (std::size_t)width * height / 2;
                frame[0].width = width; frame[0].height = height * 2;
                break;
            case DSC_STEREO_LAYOUT_BOTTOM_TOP:
                stride = width;
                y[R] = (uint8_t *)malloc((std::size_t)width * height * 3);
                frame[0].data = y[R];
                if(!y[R])goto free_frame;
                y[L]  = y[R] + (std::size_t)width * height;
                uv[R] = y[R] + (std::size_t)width * height * 2;
                uv[L] = uv[R] + (std::size_t)width * height / 2;
                frame[0].width = width; frame[0].height = height * 2;
                break;
            default:
                goto free_frame;
        }

        st = dev->pipeline->get_frame(F, y[fi], uv[fi],
                                      width, height, stride, stride, &ts[fi], &trig[fi], 100);
        if(st == Status::HwError){
            dev->state.store(Status::HwError);
            goto free_frame;
        }
        if(st != Status::Ok)goto free_frame;
        if(dev->tracker){
            std::lock_guard<std::mutex> lock(dev->tracker_mtx);
            dev->tracker->update_master_timestamp_ns(trig[fi]);
        }

        st = dev->pipeline->get_frame(O, y[oi], uv[oi],
                                      width, height, stride, stride, &ts[oi], &trig[oi], 100);
        if(st == Status::HwError){
            dev->state.store(Status::HwError);
            goto free_frame;
        }
        if(st != Status::Ok)goto free_frame;

        // Refetch the older eye until the pair is within half a frame period.
        // 重取较旧的一目，直到两帧时间差不超过半个帧周期。
        for(;;) {
            uint64_t d = (ts[R] > ts[L]) ? (ts[R] - ts[L]) : (ts[L] - ts[R]);
            if(d <= cycle_ns)break;
            if(dev->state.load() != Status::Ok)goto free_frame;

            const size_t lag = (ts[R] < ts[L]) ? R : L;
            const CamIndex lag_idx = (lag == R) ? CamIndex::Right : CamIndex::Left;
            st = dev->pipeline->get_frame(lag_idx, y[lag], uv[lag],
                                          width, height, stride, stride, &ts[lag], &trig[lag], 100);
            if(st == Status::HwError){
            dev->state.store(Status::HwError);
            goto free_frame;
        }
        if(st != Status::Ok)goto free_frame;
            if(dev->tracker && lag == fi){
                std::lock_guard<std::mutex> lock(dev->tracker_mtx);
                dev->tracker->update_master_timestamp_ns(trig[fi]);
            }
        }

        if(dev->stereo_layout == DSC_STEREO_LAYOUT_NONE){
            frame[R].timestamp_ns = ts[R];
            frame[L].timestamp_ns = ts[L];
        } else {
            frame[0].timestamp_ns = ts[fi];   // combined frame keeps the FSYNC-bound eye's timestamp 合并帧沿用与 FSYNC 绑定那一目的时间戳
        }

        if(!dev->camera_fifo->push(frame))goto free_frame;
        continue;

free_frame:
        free_frame_pair(frame);
    }
}

// Stop and join the worker threads. The caller must hold dev->mtx.
// 停止并等待工作线程退出。调用方必须持有 dev->mtx。
static void stop_locked(dsc_device_t *dev)
{
    Status expected = Status::Ok;
    dev->state.compare_exchange_strong(expected, Status::ThreadClosed);

    if(!dev->camera_thread.joinable() && !dev->imu_thread.joinable())return;
    if(dev->camera_thread.joinable())dev->camera_thread.join();
    if(dev->imu_thread.joinable())dev->imu_thread.join();
    dev->pipeline->stop();
    if(dev->imu)dev->imu->stop();
}

/* ---- Public C API 公共 C 接口 ---- */

extern "C" {

#ifndef DSC_VERSION
#  define DSC_VERSION "unknown"
#endif

const char *dsc_version(void)
{
    return DSC_VERSION;
}

#ifndef DSC_PLATFORM
#  define DSC_PLATFORM "unknown"
#endif

const char *dsc_platform(void)
{
    return DSC_PLATFORM;
}

dsc_device_t *dsc_create()
{
    return new dsc_device_t;
}

void dsc_destroy(
    dsc_device_t *dev)
{
    delete dev;   // call dsc_deinit() first: destroying with running threads is undefined behavior 请先调用 dsc_deinit()：在线程仍在运行时销毁属于未定义行为
}

dsc_err_t dsc_init(
    dsc_device_t *dev,
    const dsc_config_t *cfg)
{
    if(dev == nullptr || cfg == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(dev->eeprom || dev->pipeline || dev->imu)return DSC_PARAM_ERROR;
    if(cfg->camera_config.stereo_layout > DSC_STEREO_LAYOUT_BOTTOM_TOP)return DSC_PARAM_ERROR;
    if(cfg->camera_fifo.depth < 2)return DSC_PARAM_ERROR;
    if(cfg->imu_config.bus_num > 0 && cfg->imu_fifo.depth < 2)return DSC_PARAM_ERROR;

    for(std::size_t i = 0; i < cfg->eeprom_config.bus_num; i++){
        dev->eeprom.reset(
            new eeprom::Eeprom(cfg->eeprom_config.bus[i], cfg->eeprom_config.addr));
        if(*dev->eeprom)break;
    }
    if(dev->eeprom && !*dev->eeprom)dev->eeprom.reset();

    for(std::size_t i = 0; i < cfg->imu_config.bus_num; i++) {
        dev->imu.reset(
            new imu::Imu(cfg->imu_config.bus[i], cfg->imu_config.addr));
        if(*dev->imu)break;
    }
    if(dev->imu && !*dev->imu)dev->imu.reset();

    dev->pipeline.reset(
        new pipeline::Pipeline(
            cfg->camera_config.left_addr, cfg->camera_config.right_addr,
            cfg->camera_config.bus, cfg->camera_config.bus_num));
    if(!*dev->pipeline)return DSC_NOT_FOUND;

    // The FIFO frees frames it evicts or still owns when destroyed.
    // FIFO 负责释放它淘汰的帧，以及销毁时仍持有的帧。
    dev->camera_fifo.reset(new base::Fifo<std::array<dsc_image_nv12_t, 2>>(
        cfg->camera_fifo.depth, static_cast<FifoMode>(cfg->camera_fifo.mode), free_frame_pair));
    dev->imu_fifo.reset(new base::Fifo<dsc_imu_packet_t>(
        cfg->imu_fifo.depth, static_cast<FifoMode>(cfg->imu_fifo.mode)));

    if(dev->eeprom) {
        Status st = dev->eeprom->read(&dev->cal_internal);
        if(st != Status::Ok)return to_err(st);
    }

    if(dev->imu) {
        ImuConfig ic{};
        ic.odr_hz       = cfg->imu_config.odr_hz;
        ic.accel_fsr_g  = cfg->imu_config.accel_fsr_g;
        ic.gyro_fsr_dps = cfg->imu_config.gyro_fsr_dps;
        ic.bw_sel       = cfg->imu_config.bw_sel;
        Status st = dev->imu->init(ic);
        if(st != Status::Ok)return to_err(st);
        // Read the delay before the IMU thread takes over I2C. Unsupported means no compensation.
        // 在 IMU 线程接管 I2C 前读取延迟。配置不支持时不补偿。
        dev->imu_group_delay_ns = 0;
        st = dev->imu->group_delay_ns(&dev->imu_group_delay_ns);
        if(st != Status::Ok && st != Status::Unsupported)return to_err(st);
    }

    dev->output_w = cfg->camera_config.output_width;
    dev->output_h = cfg->camera_config.output_height;
    dev->stereo_layout = cfg->camera_config.stereo_layout;
    dev->fps      = cfg->camera_config.fps;
    dev->accel_fsr_g  = cfg->imu_config.accel_fsr_g;
    dev->gyro_fsr_dps = cfg->imu_config.gyro_fsr_dps;
    dev->fsync_camera = (cfg->camera_config.fsync_camera == DSC_CAMERA_RIGHT_IDX)
                             ? CamIndex::Right : CamIndex::Left;

    {
        const dsc_camera_config_t &cc = cfg->camera_config;
        PipelineConfig pc{};
        memcpy(pc.bus_mipi_rx,    cc.bus_mipi_rx,    sizeof(pc.bus_mipi_rx));
        memcpy(pc.bus_reset_gpio, cc.bus_reset_gpio, sizeof(pc.bus_reset_gpio));
        pc.sensor_width   = cc.sensor_width;
        pc.sensor_height  = cc.sensor_height;
        pc.fps            = cc.fps;
        pc.line_length    = cc.line_length;
        pc.frame_length   = cc.frame_length;
        pc.tuning_file    = cc.tuning_file;
        pc.mode           = static_cast<OutputMode>(cc.mode);
        pc.output_width   = cc.output_width;
        pc.output_height  = cc.output_height;

        StereoImuModel *cal = dev->eeprom ? &dev->cal_internal : nullptr;
        // RESIZE and RECT update output intrinsics, so both require calibration.
        // RESIZE 和 RECT 会更新输出内参，因此两者都需要标定数据。
        if(cal == nullptr && cc.mode != DSC_CAMERA_MODE_RAW)return DSC_PARAM_ERROR;
        Status st = dev->pipeline->init(pc, cal);
        if(st != Status::Ok)return to_err(st);
    }

    return DSC_OK;
}

dsc_err_t dsc_deinit(
    dsc_device_t *dev)
{
    if(dev == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);

    stop_locked(dev);

    dev->state.store(Status::ThreadClosed);
    dev->pipeline.reset();
    dev->imu.reset();
    dev->eeprom.reset();
    dev->camera_fifo.reset();
    dev->imu_fifo.reset();
    dev->tracker.reset();
    return DSC_OK;
}

dsc_err_t dsc_start(
    dsc_device_t *dev)
{
    if(dev == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);

    const Status st = dev->state.load();
    if(st == Status::Ok)return DSC_PARAM_ERROR;
    if(st != Status::ThreadClosed)return to_err(st);               // faulted: deinit() first 已故障：请先 deinit()
    if(!dev->pipeline)return DSC_PARAM_ERROR;
    if(dev->imu){
        if(dev->fps == 0)return DSC_PARAM_ERROR;
        // Match vin.c: truncate the LPWM period to whole microseconds before converting to ns.
        // 与 vin.c 一致：LPWM 周期先截断到整微秒，再换算为纳秒。
        dev->tracker = std::make_unique<base::TimestampTracker>(
            (1000000U / dev->fps) * 1000U);
    }

    dev->camera_on = false;
    dev->state.store(Status::Ok);

    if(dev->imu) {
        Status s = dev->imu->start();
        if(s != Status::Ok) { dev->state.store(Status::ThreadClosed); return to_err(s); }
        dev->imu_thread = std::thread(imu_thread_func, dev);
    } else {
        dev->camera_on = true;   // no IMU: skip the handshake, start streaming directly 无 IMU：跳过握手，直接开始取流
    }
    dev->camera_thread = std::thread(camera_thread_func, dev);

    return DSC_OK;
}

void dsc_stop(
    dsc_device_t *dev)
{
    if(dev == nullptr)return;
    std::lock_guard<std::mutex> lock(dev->mtx);
    stop_locked(dev);
}

/* ---- Camera data 相机数据 ---- */

size_t dsc_available_camera(
    dsc_device_t *dev)
{
    if(dev == nullptr || !dev->camera_fifo)return 0;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(dev->state.load() != Status::Ok)return 0;
    return dev->camera_fifo->size();
}

dsc_err_t dsc_get_nv12_frame(
    dsc_device_t *dev,
    dsc_image_nv12_t *image_left,
    dsc_image_nv12_t *image_right)
{
    if(dev == nullptr || image_left == nullptr || image_right == nullptr)
        return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(const Status st = dev->state.load(); st != Status::Ok)return to_err(st);
    if(dev->stereo_layout != DSC_STEREO_LAYOUT_NONE)return DSC_UNSUPPORTED;
    if(!dev->camera_fifo)return DSC_PARAM_ERROR;

    std::array<dsc_image_nv12_t, 2> f;
    if(!dev->camera_fifo->pop(f))return DSC_TIMEOUT;

    *image_left  = f[static_cast<std::size_t>(CamIndex::Left)];
    *image_right = f[static_cast<std::size_t>(CamIndex::Right)];
    return DSC_OK;
}

dsc_err_t dsc_get_stereo_nv12_frame(
    dsc_device_t *dev,
    dsc_image_nv12_t *image)
{
    if(dev == nullptr || image == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(const Status st = dev->state.load(); st != Status::Ok)return to_err(st);
    if(dev->stereo_layout == DSC_STEREO_LAYOUT_NONE)return DSC_UNSUPPORTED;
    if(!dev->camera_fifo)return DSC_PARAM_ERROR;

    std::array<dsc_image_nv12_t, 2> f;
    if(!dev->camera_fifo->pop(f))return DSC_TIMEOUT;

    *image = f[0];   // Transfer the assembled buffer without another pixel copy. 直接交出已组装的缓冲区，不再多做一次像素拷贝。
    return DSC_OK;
}

/* ---- IMU data IMU 数据 ---- */

size_t dsc_available_imu(
    dsc_device_t *dev)
{
    if(dev == nullptr || !dev->imu || !dev->imu_fifo)return 0;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(dev->state.load() != Status::Ok)return 0;
    return dev->imu_fifo->size();
}

dsc_err_t dsc_get_imu_packet(
    dsc_device_t *dev,
    dsc_imu_packet_t *out)
{
    if(dev == nullptr || out == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(const Status st = dev->state.load(); st != Status::Ok)return to_err(st);
    if(!dev->imu || !dev->imu_fifo)return DSC_PARAM_ERROR;
    if(!dev->imu_fifo->pop(*out))return DSC_TIMEOUT;
    return DSC_OK;
}

const char *dsc_get_imu_name(
    dsc_device_t *dev)
{
    if(dev == nullptr || !dev->imu)return nullptr;
    std::lock_guard<std::mutex> lock(dev->mtx);
    return dev->imu->name();
}

const char *dsc_get_imu_info(
    dsc_device_t *dev)
{
    if(dev == nullptr || !dev->imu)return nullptr;
    std::lock_guard<std::mutex> lock(dev->mtx);
    return dev->imu->info();
}

/* ---- Calibration 标定 ---- */

dsc_err_t dsc_get_camera_intrinsics(
    dsc_device_t *dev,
    dsc_camera_index_t cam_idx,
    dsc_camera_intrinsics_t *intrinsics)
{
    if(dev == nullptr || intrinsics == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(cam_idx != DSC_CAMERA_LEFT_IDX && cam_idx != DSC_CAMERA_RIGHT_IDX)return DSC_PARAM_ERROR;
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return to_err(st);
    if(!dev->eeprom)return DSC_NOT_FOUND;

    const CameraIntrinsics &src = (cam_idx == DSC_CAMERA_LEFT_IDX) ? dev->cal_internal.cam_left
                                                                     : dev->cal_internal.cam_right;
    intrinsics->fx = src.fx; intrinsics->fy = src.fy;
    intrinsics->cx = src.cx; intrinsics->cy = src.cy;
    memcpy(intrinsics->K, src.K, sizeof(src.K));
    memcpy(intrinsics->dist_coeffs, src.dist_coeffs, sizeof(src.dist_coeffs));
    intrinsics->dist_model = (src.dist_model == DistModel::Fisheye) ? DSC_DIST_FISHEYE
                                                                    : DSC_DIST_PINHOLE;
    return DSC_OK;
}

dsc_err_t dsc_get_imu_intrinsics(
    dsc_device_t *dev,
    dsc_imu_intrinsics_t *intrinsics)
{
    if(dev == nullptr || intrinsics == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return to_err(st);
    if(!dev->eeprom)return DSC_NOT_FOUND;

    const ImuIntrinsics &src = dev->cal_internal.imu;
    memcpy(intrinsics->accel_misalign, src.accel_misalign, sizeof(src.accel_misalign));
    memcpy(intrinsics->accel_scale,    src.accel_scale,    sizeof(src.accel_scale));
    memcpy(intrinsics->accel_bias,     src.accel_bias,     sizeof(src.accel_bias));
    intrinsics->accel_noise       = src.accel_noise;
    intrinsics->accel_random_walk = src.accel_random_walk;
    memcpy(intrinsics->gyro_misalign, src.gyro_misalign, sizeof(src.gyro_misalign));
    memcpy(intrinsics->gyro_scale,    src.gyro_scale,    sizeof(src.gyro_scale));
    memcpy(intrinsics->gyro_bias,     src.gyro_bias,     sizeof(src.gyro_bias));
    intrinsics->gyro_noise       = src.gyro_noise;
    intrinsics->gyro_random_walk = src.gyro_random_walk;
    return DSC_OK;
}

dsc_err_t dsc_get_relative_R(
    dsc_device_t *dev,
    dsc_reference_frame_t from_frame,
    dsc_reference_frame_t to_frame,
    double *R)
{
    if(dev == nullptr || R == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return to_err(st);
    if(!dev->eeprom)return DSC_NOT_FOUND;

    const auto from = frame_extrinsics(dev, from_frame);
    const auto to   = frame_extrinsics(dev, to_frame);
    if(from.first == nullptr || to.first == nullptr)return DSC_PARAM_ERROR;

    mat33_t_mul(to.first, from.first, R);   // R = R_to^T * R_from（相对旋转矩阵）
    return DSC_OK;
}

dsc_err_t dsc_get_relative_T(
    dsc_device_t *dev,
    dsc_reference_frame_t from_frame,
    dsc_reference_frame_t to_frame,
    double *T)
{
    if(dev == nullptr || T == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return to_err(st);
    if(!dev->eeprom)return DSC_NOT_FOUND;

    const auto from = frame_extrinsics(dev, from_frame);
    const auto to   = frame_extrinsics(dev, to_frame);
    if(from.first == nullptr || to.first == nullptr)return DSC_PARAM_ERROR;

    // T = R_to^T * (T_from - T_to)
    // 相对平移向量
    const double d[3] = {from.second[0] - to.second[0],
                         from.second[1] - to.second[1],
                         from.second[2] - to.second[2]};
    mat33_t_mul_vec(to.first, d, T);
    return DSC_OK;
}

dsc_err_t dsc_get_calibration(
    dsc_device_t *dev,
    dsc_calibration_t *calibration)
{
    if(dev == nullptr || calibration == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return to_err(st);
    if(!dev->eeprom)return DSC_NOT_FOUND;

    static_assert(sizeof(dsc_calibration_t) == sizeof(StereoImuModel),
                  "C calibration struct and internal StereoImuModel must be layout-compatible");
    memcpy(calibration, &dev->cal_internal, sizeof(*calibration));
    return DSC_OK;
}

dsc_err_t dsc_convert_calibration(
    dsc_device_t *dev,
    dsc_reference_frame_t ref_frame,
    const double ref_R[9],
    const double ref_T[3])
{
    if(dev == nullptr || ref_R == nullptr || ref_T == nullptr)return DSC_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return to_err(st);
    if(!dev->eeprom)return DSC_NOT_FOUND;

    const auto anchor = frame_extrinsics(dev, ref_frame);
    if(anchor.first == nullptr)return DSC_PARAM_ERROR;

    // Map the old reference frame to the new one so the anchor takes (ref_R, ref_T).
    // Apply the same transform to every sensor to preserve their relative poses.
    // 将旧参考系变换到新参考系，使锚点外参等于 (ref_R, ref_T)。
    // 所有传感器应用同一变换，彼此的相对位姿不变。
    double GR[9], GT[3], tmp[3];
    mat33_mul_t(ref_R, anchor.first, GR);
    mat33_mul_vec(GR, anchor.second, tmp);
    for(int i = 0; i < 3; i++)GT[i] = ref_T[i] - tmp[i];

    double *all_R[3] = {dev->cal_internal.cam_right_R, dev->cal_internal.cam_left_R, dev->cal_internal.imu_R};
    double *all_T[3] = {dev->cal_internal.cam_right_T, dev->cal_internal.cam_left_T, dev->cal_internal.imu_T};
    for(int k = 0; k < 3; k++){
        double R2[9], T2[3];
        mat33_mul(GR, all_R[k], R2);
        mat33_mul_vec(GR, all_T[k], T2);
        for(int i = 0; i < 3; i++)T2[i] += GT[i];
        memcpy(all_R[k], R2, sizeof(R2));
        memcpy(all_T[k], T2, sizeof(T2));
    }
    return DSC_OK;
}

const char *dsc_get_eeprom_name(
    dsc_device_t *dev)
{
    if(dev == nullptr || !dev->eeprom)return nullptr;
    std::lock_guard<std::mutex> lock(dev->mtx);
    return dev->eeprom->name();
}

const char *dsc_get_eeprom_info(
    dsc_device_t *dev)
{
    if(dev == nullptr || !dev->eeprom)return nullptr;
    std::lock_guard<std::mutex> lock(dev->mtx);
    return dev->eeprom->info();
}

} // extern "C"
