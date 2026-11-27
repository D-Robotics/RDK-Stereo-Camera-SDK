/**
 * @file tracker.cpp
 * @brief TimestampTracker implementation: handshake state machine and phase alignment.
 *
 * Output timestamps use the master's absolute epoch while interpolating slave-sample
 * spacing between aligned FSYNC anchors. The tracker starts from the master edge of
 * the previous anchor, predicts the
 * current edge by counting anchors on the nominal master period, snaps that prediction
 * to the newest master timestamp, and interpolates the corrections for the samples
 * taken since the previous anchor.
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/tracker/tracker.hpp"
#include <cstdio>

namespace dsc {
namespace base {

TimestampTracker::TimestampTracker(uint32_t master_cycle_ns)
    : master_cycle_ns_(master_cycle_ns),
      pi_controller_(0.2, 0.0, 0.0)
{}

// The first master timestamp doubles as the phase reference: it is kept as first_ns and
// stays the "previous anchor edge" until a real anchor arrives.
void TimestampTracker::update_master_timestamp_ns(
    uint64_t timestamp_ns)
{
    if(master_.first_ts_ns == 0)master_.first_ts_ns = timestamp_ns;
    master_.last_ts_ns = timestamp_ns;
}

uint64_t TimestampTracker::get_slave_timestamp(bool is_fsync, uint64_t *trig_fsync_dt_ns)
{
    /////////////////////////////////////////////////////////////////
    // 首个 fsync 出现前丢弃
    // 首个 fsync 出现时记录 dt
    /////////////////////////////////////////////////////////////////
    if(slave_.first_trig_fsync_dt_ns == UINT64_MAX && is_fsync)
        slave_.first_trig_fsync_dt_ns = *trig_fsync_dt_ns;
    if(slave_.first_trig_fsync_dt_ns == UINT64_MAX)return 0;
    /////////////////////////////////////////////////////////////////

    ++slave_.sample_count;

    if(is_fsync)++slave_.fsync_count;

    /////////////////////////////////////////////////////////////////
    // 主时钟有效后的第一个 fsync 开始才有正常的计算逻辑
    /////////////////////////////////////////////////////////////////
    if(master_.first_ts_ns == 0)return 0;
    /////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////
    // fsync 帧：先计算本帧正确时间戳，再更新 dt
    /////////////////////////////////////////////////////////////////
    if(is_fsync){
        // 首个 fsync 下的逻辑
        if(slave_.last_fsync_ts_ns == 0){
            --slave_.fsync_count; // 首个 fsync 不计入周期计算
            --slave_.sample_count; // 首个 sample 不计入周期计算
            /////////////////////////////////////////////////////////////////
            // 计算当前时间戳
            /////////////////////////////////////////////////////////////////
            uint64_t trigger_ts_ns = master_.first_ts_ns + slave_.fsync_count * master_cycle_ns_;
            // 周期内矫正到最新主时钟相位
            uint64_t tmp_trigger_ts_ns = master_.last_ts_ns;
            while(tmp_trigger_ts_ns < trigger_ts_ns - master_cycle_ns_ / 2)tmp_trigger_ts_ns += master_cycle_ns_;
            while(tmp_trigger_ts_ns > trigger_ts_ns + master_cycle_ns_ / 2)tmp_trigger_ts_ns -= master_cycle_ns_;
            uint64_t ts_ns = tmp_trigger_ts_ns + *trig_fsync_dt_ns;

            /////////////////////////////////////////////////////////////////
            // 计算 slave 周期
            /////////////////////////////////////////////////////////////////
            slave_.period_ns = (ts_ns - master_.first_ts_ns - slave_.first_trig_fsync_dt_ns) / slave_.sample_count;

            // 更新状态
            slave_.last_ts_ns = ts_ns;
            slave_.last_fsync_ts_ns = ts_ns;
            slave_.last_trig_fsync_dt_ns = *trig_fsync_dt_ns;
            slave_.sample_count = 0;

            return ts_ns;
        }
        // 后续 fsync 下的逻辑
        else{
            /////////////////////////////////////////////////////////////////
            // 计算当前时间戳
            /////////////////////////////////////////////////////////////////
            uint64_t trigger_ts_ns = slave_.last_fsync_ts_ns - slave_.last_trig_fsync_dt_ns + master_cycle_ns_;
            // 周期内矫正到最新主时钟相位
            uint64_t tmp_trigger_ts_ns = master_.last_ts_ns;
            while(tmp_trigger_ts_ns < trigger_ts_ns - master_cycle_ns_ / 2)tmp_trigger_ts_ns += master_cycle_ns_;
            while(tmp_trigger_ts_ns > trigger_ts_ns + master_cycle_ns_ / 2)tmp_trigger_ts_ns -= master_cycle_ns_;
            uint64_t ts_ns = tmp_trigger_ts_ns + *trig_fsync_dt_ns;

            /////////////////////////////////////////////////////////////////
            // 周期误差 = 实测周期 - 当前周期；每个 FSYNC 算一拍
            /////////////////////////////////////////////////////////////////
            if(slave_.sample_count > 0){
                const int64_t anchor_ns = ts_ns;
                const int64_t previous_anchor_ns = slave_.last_fsync_ts_ns;
                const int64_t period_ns = slave_.period_ns;
                const int64_t measured_ns = (anchor_ns - previous_anchor_ns) / slave_.sample_count;
                const auto correction_ns = static_cast<int64_t>(
                    pi_controller_.update(measured_ns - period_ns, 1.0));
                slave_.period_ns = static_cast<uint32_t>(period_ns + correction_ns);
            }

            // 更新状态
            slave_.last_ts_ns += slave_.period_ns;
            slave_.last_fsync_ts_ns = slave_.last_ts_ns;
            slave_.last_trig_fsync_dt_ns = *trig_fsync_dt_ns;
            slave_.sample_count = 0;

            return slave_.last_ts_ns;
        }
    }
    // 普通帧
    else{
        slave_.last_ts_ns += slave_.period_ns;
        return slave_.last_ts_ns;
    }

    return 0;
}

} // namespace base
} // namespace dsc
