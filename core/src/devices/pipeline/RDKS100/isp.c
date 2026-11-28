/**
 * @file isp.c
 * @brief Offline ISP converts RAW10 from VIN in DDR to NV12 on channel 0.
 * 离线 ISP 从 DDR 读取 VIN 的 RAW10，在通道 0 输出 NV12。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKS100.h"

#include <string.h>

/* The BSP uses three driver-owned buffers for offline ISP output.
 * BSP 为离线 ISP 输出配置三个缓冲区，由驱动持有。 */
#define DSC_ISP_BUF_NUM 3

int isp_open(hbn_vnode_handle_t *isp, uint32_t width, uint32_t height,
             uint32_t hw_id, uint32_t slot_id, uint32_t fps)
{
    isp_cfg_t cfg = { 0 };

    cfg.isp_attr.channel.hw_id   = hw_id;
    cfg.isp_attr.channel.slot_id = slot_id;
    cfg.isp_attr.channel.ctx_id  = AUTO_ALLOC_ID;

    cfg.isp_attr.work_mode  = 0;
    /* The BSP encodes linear (non-HDR) mode as 1.
     * BSP 用 1 表示线性（非 HDR）模式。 */
    cfg.isp_attr.hdr_mode   = 1;
    cfg.isp_attr.size.width  = width;
    cfg.isp_attr.size.height = height;
    cfg.isp_attr.frame_rate  = fps;
    cfg.isp_attr.sched_mode  = SCHED_MODE_MANUAL;
    cfg.isp_attr.algo_state  = 1;

    cfg.isp_attr.isp_combine.isp_channel_mode = ISP_CHANNEL_MODE_NORMAL;

    /* Match the BSP's 3A statistics settings for this sensor.
     * 3A 统计配置沿用 BSP 对此传感器的设置。 */
    cfg.isp_attr.isp_sw_ctrl.ae_stat_buf_en    = 1;
    cfg.isp_attr.isp_sw_ctrl.awb_stat_buf_en   = 1;
    cfg.isp_attr.isp_sw_ctrl.ae5bin_stat_buf_en = 1;
    cfg.isp_attr.isp_sw_ctrl.ctx_buf_en        = 0;
    cfg.isp_attr.isp_sw_ctrl.pixel_consistency_en = 0;

    /* Keep the full frame for downstream cropping in PYM.
     * 保留整帧，交给下游 PYM 裁剪。 */
    cfg.ichn_attr.input_crop_cfg.enable  = 0;
    cfg.ichn_attr.in_buf_noclean         = 1;
    cfg.ichn_attr.in_buf_noncached       = 0;

    cfg.ochn_attr.output_crop_cfg.enable = 0;
    cfg.ochn_attr.out_buf_noinvalid      = 1;
    cfg.ochn_attr.out_buf_noncached      = 0;
    cfg.ochn_attr.output_raw_level       = 0;   /* ISP_OUTPUT_RAW_LEVEL_SENSOR_DATA (sensor data).
                                               * ISP_OUTPUT_RAW_LEVEL_SENSOR_DATA（传感器数据）。 */
    cfg.ochn_attr.buf_num                = DSC_ISP_BUF_NUM;

    cfg.ochn_attr.stream_output_mode = STREAM_OUTPUT_MODE_DISABLE;
    cfg.ochn_attr.axi_output_mode    = AXI_OUTPUT_MODE_YUV420;

    if (hbn_vnode_open(HB_ISP, hw_id, AUTO_ALLOC_ID, isp) != 0) {
        *isp = 0;
        return -1;
    }
    if (hbn_vnode_set_attr(*isp, &cfg) != 0 ||
        hbn_vnode_set_ochn_attr(*isp, 0, &cfg.ochn_attr) != 0 ||
        hbn_vnode_set_ichn_attr(*isp, 0, &cfg.ichn_attr) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc_attr = { 0 };
    alloc_attr.buffers_num = DSC_ISP_BUF_NUM;
    alloc_attr.is_contig = 1;
    alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                       HB_MEM_USAGE_CPU_WRITE_OFTEN |
                       HB_MEM_USAGE_CACHED;
    if (hbn_vnode_set_ochn_buf_attr(*isp, 0, &alloc_attr) != 0)
        return -1;

    return 0;
}
