/**
 * @file pipeline.hpp
 * @brief Platform-independent stereo camera pipeline interface.
 * @brief 与平台无关的双目相机流水线接口。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSC_DEVICES_PIPELINE_PIPELINE_HPP
#define DSC_DEVICES_PIPELINE_PIPELINE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>

#include "types.hpp"

namespace dsc {
namespace pipeline {

// Serialize calls on the same instance; Pipeline is not thread-safe.
// Pipeline 不是线程安全的；同一实例的调用必须串行。
class Pipeline {
public:
    // The first chip-ID match in bus_list wins for each eye. Probe only; no stream is allocated.
    // 每目采用 bus_list 中首个芯片 ID 匹配的总线。这里仅探测，不创建采集流。
    Pipeline(uint8_t left_addr, uint8_t right_addr,
             const uint8_t *bus_list, std::size_t bus_num);
    ~Pipeline();

    Pipeline(const Pipeline &)            = delete;
    Pipeline &operator=(const Pipeline &) = delete;

    // True only if both sensors were found.
    // 仅在探测到双目传感器时为 true。
    explicit operator bool() const;

    /**
     * Builds the pipeline without starting capture. Raw can run without calibration.
     * Resize and Rect require calibration and update it in place for the output.
     * The caller owns cal; the pipeline does not retain it.
     * 构建流水线，不启动采集。Raw 可以不传标定数据；Resize 和 Rect 必须提供标定，
     * 并原地更新为输出图像对应的参数。cal 归调用方所有，流水线不保留其指针。
     */
    Status init(const PipelineConfig &cfg, StereoImuModel *cal);

    // The destructor also calls deinit(); cleanup after a failed init() is backend-specific.
    // 析构时也调用 deinit()；初始化失败时能否清理取决于后端。
    void   deinit();

    // Start the FSYNC-bound eye first; its trigger anchors the IMU clock.
    // 先启动接有 FSYNC 的那一目，其触发沿是 IMU 校时的锚点。
    Status start(CamIndex first);

    // Keeps the pipeline initialized so capture can restart.
    // 保留已初始化的流水线，可以重新启动采集。
    void   stop();

    /**
     * Caller-owned NV12 buffers: y has height rows, uv has height / 2 rows;
     * strides are in bytes and may exceed width. Dimensions must match the frame.
     * NV12 缓冲区由调用方分配：y 有 height 行，uv 有 height / 2 行；
     * 步长以字节为单位，可大于 width。宽高必须与帧尺寸一致。
     *
     * timestamp_ns approximates the exposure midpoint; trigger_ns is the LPWM
     * edge used as the IMU FSYNC anchor. Both are in nanoseconds.
     * timestamp_ns 近似表示曝光中点；trigger_ns 是用于 IMU FSYNC 校时的 LPWM
     * 触发沿。两者单位均为纳秒。
     */
    Status get_frame(CamIndex idx,
                     uint8_t *y, uint8_t *uv,
                     uint32_t width, uint32_t height,
                     uint32_t y_stride, uint32_t uv_stride,
                     uint64_t *timestamp_ns,
                     uint64_t *trigger_ns,
                     uint32_t timeout_ms);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace pipeline
} // namespace dsc

#endif // DSC_DEVICES_PIPELINE_PIPELINE_HPP
