/**
 * @file vin.c
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKX5.h"

#include <string.h>

int vin_open(hbn_vnode_handle_t *vin, int mipi_rx,
             uint32_t width, uint32_t height, uint32_t fps)
{
    /* Use whole-microsecond periods to match the IMU FSYNC tracker. The phase centers
       the 1 ms pulse and accounts for rounding, so the 1 Hz retrigger falls between
       pulses and preserves the requested count. Encode period and pulse width
       minus one, and phase plus one.
       周期取整微秒，与 IMU FSYNC 跟踪器一致。相位将 1 ms 脉冲居中并补偿取整余量，
       让每秒一次的重触发落在脉冲之间，保持设定的脉冲数。驱动用周期和脉宽减一、
       相位加一的值配置 period、duty 和 offset。 */
    const uint32_t pulse_period_us = 1000000U / fps;
    const uint32_t high_us = 1000U;
    const uint32_t remainder_us = 1000000U - pulse_period_us * fps;
    const uint32_t phase_us = remainder_us + (pulse_period_us - high_us) / 2U;
    const uint32_t period_us = pulse_period_us - 1U;
    const uint32_t offset_us = phase_us + 1U;

    vin_node_attr_t node = {
        .cim_attr = {
            .mipi_rx = (uint32_t)mipi_rx,
            .vc_index = 0,
            .ipi_channel = 1,
            .cim_isp_flyby = 0,
            .func = {
                .enable_frame_id = 1,
                .set_init_frame_id = 0,
                .hdr_mode = NOT_HDR,
                .time_stamp_en = 1,
                .time_stamp_mode = TS_IPI_VSYNC | TS_IPI_TRIGGER,
                .ts_src = (uint32_t)((mipi_rx == 0) ? 5 : 6),   /* RX timestamp source: 0 -> 5, otherwise 6. 接收端时间戳源：RX 0 用 5，其余用 6。 */
            },
        },
        /* All four LPWM channels share timing; board routing selects camera and IMU FSYNC outputs.
           四路 LPWM 时序相同，由板级布线决定哪些输出接相机和 IMU FSYNC。 */
        .lpwm_attr = {
            .enable = 1,
            .lpwm_chn_attr = {
                [0] = { .trigger_source = 0, .trigger_mode = 0, .period = period_us,
                        .offset = offset_us, .duty_time = 999, .threshold = 0, .adjust_step = 0 },
                [1] = { .trigger_source = 0, .trigger_mode = 0, .period = period_us,
                        .offset = offset_us, .duty_time = 999, .threshold = 0, .adjust_step = 0 },
                [2] = { .trigger_source = 0, .trigger_mode = 0, .period = period_us,
                        .offset = offset_us, .duty_time = 999, .threshold = 0, .adjust_step = 0 },
                [3] = { .trigger_source = 0, .trigger_mode = 0, .period = period_us,
                        .offset = offset_us, .duty_time = 999, .threshold = 0, .adjust_step = 0 },
            },
        },
    };

    vin_ichn_attr_t ichn = {
        .width = width,
        .height = height,
        .format = 0x2B,
    };

    vin_ochn_attr_t ochn = {
        .ddr_en = 1,
        .ochn_attr_type = VIN_BASIC_ATTR,
        .vin_basic_attr = {
            .format = 0x2B,
            .wstride = width * 2,   /* DDR RAW10 uses 2 bytes per pixel. DDR 中的 RAW10 每像素占 2 字节。 */
        },
    };

    if (hbn_vnode_open(HB_VIN, (uint32_t)mipi_rx, AUTO_ALLOC_ID, vin) != 0) {
        *vin = 0;
        return -1;
    }
    if (hbn_vnode_set_attr(*vin, &node) != 0 ||
        hbn_vnode_set_ichn_attr(*vin, 0, &ichn) != 0 ||
        hbn_vnode_set_ochn_attr(*vin, 0, &ochn) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = 3,        /* Driver-owned capture buffers. 采集缓冲区归驱动管理。 */
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED,
    };
    if (hbn_vnode_set_ochn_buf_attr(*vin, 0, &alloc) != 0)
        return -1;
    return 0;
}
