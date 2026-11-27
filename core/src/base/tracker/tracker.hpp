/**
 * @file tracker.hpp
 * @brief Master-slave timestamp tracker; callers serialize concurrent access.
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSC_BASE_TRACKER_HPP
#define DSC_BASE_TRACKER_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace dsc {
namespace base {

// 离散 PID：dt 由调用方给出，首拍不计算微分项
class PIDController {
public:
    PIDController() = default;
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd) {}

    double update(double error, double dt)
    {
        if(!(dt > 0.0))return 0.0;
        integral_ += error * dt;
        // 首拍没有历史误差，导数项按 0 处理，避免起步冲击
        const double derivative = has_last_ ? (error - last_error_) / dt : 0.0;
        last_error_ = error;
        has_last_ = true;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;

    double integral_ = 0.0;
    double last_error_ = 0.0;
    bool has_last_ = false;
};

class TimestampTracker {
public:
    explicit TimestampTracker(uint32_t master_cycle_ns);

    // copying disabled
    TimestampTracker(const TimestampTracker &) = delete;
    TimestampTracker &operator=(const TimestampTracker &) = delete;

    TimestampTracker(TimestampTracker &&) noexcept = default;
    TimestampTracker &operator=(TimestampTracker &&) noexcept = default;

    void update_master_timestamp_ns(uint64_t timestamp_ns);

    uint64_t get_slave_timestamp(bool is_fsync, uint64_t *trig_fsync_dt_ns);

private:
    struct MasterState {
        uint64_t first_ts_ns = 0; // the first master timestamp seen (phase reference)
        uint64_t last_ts_ns  = 0; // the last master timestamp seen
    };

    struct SlaveState {
        uint64_t last_ts_ns = 0;                      // the last slave timestamp produced
        uint64_t last_fsync_ts_ns = 0;                // 上一帧 FSYNC 帧的 IMU 时间戳
        uint64_t last_trig_fsync_dt_ns = 0;           // 上一帧 FSYNC 帧的触发时间间隔
        uint64_t first_trig_fsync_dt_ns = UINT64_MAX; // 第一个 dt

        uint32_t sample_count = 0;                    // slave 时钟数量统计，用于 PTP 计算
        uint32_t fsync_count = 0;                     // slave 时钟数量统计，用于 PTP 计算

        uint32_t period_ns = 0;                       // slave 时钟周期，实时更新
    };

    uint32_t master_cycle_ns_;
    MasterState master_;
    SlaveState slave_;

    PIDController pi_controller_;   // 周期环：误差 = 实测周期 - 当前周期
};

} // namespace base
} // namespace dsc

#endif // DSC_BASE_TRACKER_HPP