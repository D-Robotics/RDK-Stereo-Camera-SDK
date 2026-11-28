/**
 * @file vin.c
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKS600.h"

#include <string.h>

#define DSC_VIN_RAW10 0x2B

/* VCON bus IDs come from the BSP sensor table, not the device tree.
 * VCON 总线编号取自 BSP 传感器表，不从设备树读取。 */
#define DSC_VIN_VCON_BUS_MAIN   2
#define DSC_VIN_VCON_BUS_SECOND 2

/* The S600 BSP uses six driver-owned capture buffers for this sensor.
 * 此传感器沿用 BSP 的六个采集缓冲区，由驱动持有。 */
#define DSC_VIN_BUF_NUM 6

/* VIN and ISP must agree on the packed RAW10 stride: round up to 32 bytes
 * (e.g. 1088 pixels -> 1360 packed bytes -> 1376 bytes per row).
 * VIN 和 ISP 的 RAW10 行跨度必须一致：按 32 字节向上对齐
 * （例如 1088 像素打包后为 1360 字节，对齐后为 1376 字节）。 */
#define DSC_VIN_WSTRIDE_ALIGN 32U

int vin_open(hbn_vnode_handle_t *vin, int mipi_rx,
             uint32_t width, uint32_t height, uint32_t fps)
{
    /* Center the 1 ms pulse in the frame window and account for the division
     * remainder so the one-second retrigger falls between pulses. The driver
     * encodes the period as N - 1 and the phase as N + 1, in microseconds.
     * 将 1 ms 脉冲放在帧窗口中央，并补上整除余数，让每秒一次的重触发落在脉冲之间。
     * 驱动的周期填 N - 1，相位填 N + 1，单位为微秒。 */
    const uint32_t pulse_period_us = 1000000U / fps;
    const uint32_t high_us = 1000U;
    const uint32_t remainder_us = 1000000U - pulse_period_us * fps;
    const uint32_t phase_us = remainder_us + (pulse_period_us - high_us) / 2U;
    const uint32_t period_us = pulse_period_us - 1U;
    const uint32_t offset_us = phase_us + 1U;

    const uint32_t ochn_wstride =
        (((width * 10U / 8U) + DSC_VIN_WSTRIDE_ALIGN - 1U) / DSC_VIN_WSTRIDE_ALIGN)
        * DSC_VIN_WSTRIDE_ALIGN;

    vin_attr_t attr = { 0 };

    attr.vin_node_attr.vcon_attr.bus_main   = DSC_VIN_VCON_BUS_MAIN;
    attr.vin_node_attr.vcon_attr.bus_second = DSC_VIN_VCON_BUS_SECOND;

    /* Use DDR for VIN -> ISP; do not enable flyby.
     * VIN 经 DDR 向 ISP 送帧，不走 flyby。 */
    attr.vin_node_attr.cim_attr.mipi_en       = 1;
    attr.vin_node_attr.cim_attr.cim_isp_flyby = 0;
    attr.vin_node_attr.cim_attr.cim_pym_flyby = 0;
    attr.vin_node_attr.cim_attr.mipi_rx       = (uint32_t)mipi_rx;
    attr.vin_node_attr.cim_attr.vc_index      = 0;   /* Must match camera.c.
                                                      * 须与 camera.c 一致。 */
    attr.vin_node_attr.cim_attr.ipi_channels  = 1;
    attr.vin_node_attr.cim_attr.y_uv_swap     = 0;

    attr.vin_node_attr.cim_attr.func.enable_frame_id   = 1;
    attr.vin_node_attr.cim_attr.func.set_init_frame_id = 1;
    attr.vin_node_attr.cim_attr.func.enable_pattern    = 0;
    /* LPWM drives exposure; no LPWM output is fed back into CIM for frame IDs.
     * LPWM 触发曝光，不回送 CIM 作为帧 ID 触发源。 */
    attr.vin_node_attr.cim_attr.func.lpwm_trig_sel = (int32_t)LPWM_CHN_INVALID;

    attr.vin_node_attr.cim_attr.rdma_input.rdma_en = 0;

    attr.vin_node_attr.magicNumber = DSC_S600_MAGIC_NUMBER;

    /* Use identical LPWM timing for both eyes; this period also drives IMU FSYNC.
     * 双目共用 LPWM 时序；IMU FSYNC 也以该周期为基准。 */
    for (uint32_t ch = 0; ch < 4; ch++) {
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].enable        = 1;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].trigger_source = 0;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].trigger_mode   = 0;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].period         = period_us;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].offset         = offset_us;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].duty_time      = 999;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].threshold      = 0;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].adjust_step    = 0;
    }

    attr.vin_ichn_attr.width  = width;
    attr.vin_ichn_attr.height = height;
    attr.vin_ichn_attr.format = DSC_VIN_RAW10;

    attr.vin_ochn_attr[VIN_MAIN_FRAME].ddr_en = 1;
    attr.vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.format    = DSC_VIN_RAW10;
    attr.vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.wstride   = ochn_wstride;
    attr.vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.pack_mode = 1;
    attr.vin_ochn_attr[VIN_MAIN_FRAME].pingpong_ring = 1;
    attr.vin_ochn_attr[VIN_MAIN_FRAME].magicNumber   = DSC_S600_MAGIC_NUMBER;

    attr.vin_ochn_buff_attr[VIN_MAIN_FRAME].buffers_num = DSC_VIN_BUF_NUM;

    attr.magicNumber = DSC_S600_MAGIC_NUMBER;

    /* VIN uses the MIPI receiver index as its hardware ID.
     * VIN 使用 MIPI 接收器索引作为硬件 ID。 */
    if (hbn_vnode_open(HB_VIN, (uint32_t)mipi_rx, AUTO_ALLOC_ID, vin) != 0) {
        *vin = 0;
        return -1;
    }
    if (hbn_vnode_set_attr(*vin, &attr) != 0 ||
        hbn_vnode_set_ichn_attr(*vin, 0, &attr.vin_ichn_attr) != 0 ||
        hbn_vnode_set_ochn_attr(*vin, 0, &attr.vin_ochn_attr[VIN_MAIN_FRAME]) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = DSC_VIN_BUF_NUM,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED,
    };
    if (hbn_vnode_set_ochn_buf_attr(*vin, 0, &alloc) != 0)
        return -1;
    return 0;
}
