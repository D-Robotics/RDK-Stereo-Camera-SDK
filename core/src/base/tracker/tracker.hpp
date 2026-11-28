/**
 * @file tracker.hpp
 * @brief Master-slave timestamp tracker; callers serialize concurrent access.
 * @brief 主从时间戳跟踪器；并发访问由调用方串行化。
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

// Discrete PID: dt is supplied by the caller; the derivative term is not computed
// on the first tick
// 离散 PID：dt 由调用方给出，首拍不计算微分项。
class PIDController {
public:
    PIDController() = default;
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd) {}

    double update(double error, double dt)
    {
        if(!(dt > 0.0))return 0.0;
        integral_ += error * dt;
        // There is no history error on the first tick, so the derivative term is treated as 0
        // to avoid a start-up kick
        // 首拍没有历史误差，导数项按 0 处理，避免起步冲击。
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
    // 禁止拷贝。
    TimestampTracker(const TimestampTracker &) = delete;
    TimestampTracker &operator=(const TimestampTracker &) = delete;

    TimestampTracker(TimestampTracker &&) noexcept = default;
    TimestampTracker &operator=(TimestampTracker &&) noexcept = default;

    void update_master_timestamp_ns(uint64_t timestamp_ns);

    uint64_t get_slave_timestamp(bool is_fsync, uint64_t *trig_fsync_dt_ns);

private:
    struct MasterState {
        uint64_t first_ts_ns = 0; // the first master timestamp seen (phase reference) 看到的第一个主时间戳（相位参考）
        uint64_t last_ts_ns  = 0; // the last master timestamp seen 最后看到的主时间戳。
    };

    struct SlaveState {
        uint64_t last_ts_ns = 0;                      // the last slave timestamp produced 最后产出的从时间戳
        uint64_t last_fsync_ts_ns = 0;                // the IMU timestamp of the previous FSYNC frame 上一帧 FSYNC 帧的 IMU 时间戳
        uint64_t last_trig_fsync_dt_ns = 0;           // the trigger-time delta of the previous FSYNC frame 上一帧 FSYNC 帧的触发时间间隔
        uint64_t first_trig_fsync_dt_ns = UINT64_MAX; // the first dt 第一个 dt。

        uint32_t sample_count = 0;                    // slave clock sample count, used for the PTP computation slave 时钟数量统计，用于 PTP 计算
        uint32_t fsync_count = 0;                     // slave clock FSYNC count, used for the PTP computation slave 时钟数量统计，用于 PTP 计算。

        uint32_t period_ns = 0;                       // slave clock period, updated in real time slave 时钟周期，实时更新。
    };

    uint32_t master_cycle_ns_;
    MasterState master_;
    SlaveState slave_;

    PIDController pi_controller_;   // period loop: error = measured period - current period 周期环：误差 = 实测周期 - 当前周期。
};

} // namespace base
} // namespace dsc

#endif // DSC_BASE_TRACKER_HPP