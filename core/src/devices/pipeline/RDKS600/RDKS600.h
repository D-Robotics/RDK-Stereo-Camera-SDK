/**
 * @file RDKS600.h
 * Return handles and buffers to the caller even if a later setup step fails.
 * In that case, the caller must still clean up any acquired resources.
 * 后续配置失败时，已取得的句柄和缓冲区仍会返回，由调用方清理。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSC_PIPELINE_RDKS600_H
#define DSC_PIPELINE_RDKS600_H

#include <stdint.h>
#include <stddef.h>

#include <hb_mem_mgr.h>
#include <hb_camera_interface.h>
#include <hbn_vpf_interface.h>
#include <hbn_vin_cfg.h>
#include <hbn_isp_cfg.h>
#include <hb_comm_isp.h>
#include <hbn_pym_cfg.h>
#include <hb_gdc_cfg.h>
/* gdc_bin_cfg.h and hb_gdc_data_info.h share guard HB_GDC_DATA_INFO_H_
 * and define the same types; include only one.
 * gdc_bin_cfg.h 和 hb_gdc_data_info.h 共用 HB_GDC_DATA_INFO_H_，
 * 类型定义也相同，只需引入其中一个。 */
#include <gdc_bin_cfg.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Driver magicNumber comes from hobot_mipi_cam (vp_sensors.h), not the driver headers.
 * 驱动要求的 magicNumber 取自 hobot_mipi_cam 的 vp_sensors.h，而非驱动头文件。
 */
#define DSC_S600_MAGIC_NUMBER 0x12345678u

/**
 * A negative GPIO is a no-op. Otherwise, on pulses reset 1 -> 0 -> 1 with
 * 30 ms per step; off holds reset low without cutting power.
 * GPIO 为负时不操作。否则，on 以 1 -> 0 -> 1 复位，每步 30 ms；
 * off 将复位引脚拉低，不切断供电。
 */
int sensor_power(int gpio, int on);

/**
 * tuning_file is borrowed for this call only; NULL selects "disable".
 * tuning_file 仅在本次调用中借用；传 NULL 则使用 "disable"。
 */
int camera_open(
    camera_handle_t *cam_fd,
    uint8_t i2c_addr,
    uint32_t width, uint32_t height, uint32_t fps,
    uint32_t line_length, uint32_t frame_length,
    uint16_t mipiclk, uint16_t settle, uint16_t mclk,
    const char *tuning_file);

int vin_open(
    hbn_vnode_handle_t *vin,
    int mipi_rx,
    uint32_t width, uint32_t height, uint32_t fps);

int isp_open(
    hbn_vnode_handle_t *isp,
    uint32_t width, uint32_t height,
    uint32_t hw_id, uint32_t slot_id, uint32_t fps);

/**
 * Return 0 if the aspect-ratio crop divides exactly, -1 if it would truncate.
 * 按宽高比裁剪时，尺寸能整除则返回 0，否则返回 -1。
 */
int roi_ratio_exact(uint32_t in_w, uint32_t in_h,
                    uint32_t out_w, uint32_t out_h);

/**
 * Rectangle in input pixel coordinates.
 * 输入图像像素坐标系中的矩形。
 */
typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t w;
    uint32_t h;
} dsc_rect_t;

/**
 * Centered crop with the output aspect ratio. Check roi_ratio_exact() before use.
 * 按输出宽高比居中裁剪。使用前先用 roi_ratio_exact() 检查能否整除。
 */
dsc_rect_t aspect_roi(uint32_t in_w, uint32_t in_h,
                        uint32_t out_w, uint32_t out_h);

int pym_open(
    hbn_vnode_handle_t *pym,
    uint32_t in_w, uint32_t in_h,
    uint32_t out_w, uint32_t out_h,
    uint32_t hw_id, uint32_t slot_id);

/**
 * map must have grid_w * grid_h points with the driver point_t layout (double x, y).
 * RemapPoint is cast to point_t without a compile-time layout check.
 * map 必须包含 grid_w * grid_h 个点，布局须与驱动的 point_t（double x, y）一致。
 * RemapPoint 直接转为 point_t，编译时不检查布局。
 */
int gdc_open(
    hbn_vnode_handle_t *gdc,
    hb_mem_common_buf_t *gdc_bin,
    const void *map, uint32_t in_w, uint32_t in_h,
    uint32_t grid_w, uint32_t grid_h, int install_angle);

/**
 * VIN is required. Zero skips an optional node; out_node and out_chn identify the output.
 * VIN 必须提供。可选节点的句柄为 0 时跳过；out_node 和 out_chn 指明取帧位置。
 */
int vflow_build(hbn_vflow_handle_t *vflow, camera_handle_t cam_fd,
                hbn_vnode_handle_t vin, hbn_vnode_handle_t isp,
                hbn_vnode_handle_t gdc, hbn_vnode_handle_t pym,
                uint32_t pym_chn, int gdc_before_pym,
                hbn_vnode_handle_t *out_node, uint32_t *out_chn);

/**
 * The flow owns registered vnodes. Keep gdc_bin alive until the flow is destroyed.
 * 已注册的 vnode 归 flow 所有；gdc_bin 须保留到 flow 销毁后。
 */
void teardown_cam(
    hbn_vflow_handle_t vflow,
    camera_handle_t cam_fd,
    hbn_vnode_handle_t vin, hbn_vnode_handle_t isp,
    hbn_vnode_handle_t pym,
    hbn_vnode_handle_t gdc,
    hb_mem_common_buf_t *gdc_bin, int reset_gpio);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DSC_PIPELINE_RDKS600_H */
