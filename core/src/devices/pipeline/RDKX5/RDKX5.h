/**
 * @file RDKX5.h
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSC_PIPELINE_RDKX5_H
#define DSC_PIPELINE_RDKX5_H

#include <stdint.h>
#include <stddef.h>

#include <hbn_api.h>
#include <hb_mem_mgr.h>
#include <cam_def.h>
#include <vin_cfg.h>
#include <isp_cfg.h>
#include <vse_cfg.h>
#include <gdc_cfg.h>
#include <hb_camera_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reset via sysfs without switching power. A negative GPIO is a no-op.
 * Nonzero on pulses 1 -> 0 -> 1 with 30 ms after each level; zero pulls low without waiting.
 * Returns -1 if the GPIO value file cannot be opened; otherwise 0.
 * 通过 sysfs 复位，不切断供电。GPIO 编号为负时不操作。
 * on 非零时按 1 -> 0 -> 1 输出，每级保持 30 ms；为零时拉低，不等待。
 * GPIO 的 value 文件打不开时返回 -1，否则返回 0。
 */
int sensor_power(int gpio, int on);

/* Use the probed I2C address. Timing is in pixel clocks per line and lines per frame.
 * Link parameters pass through unchanged. The borrowed tuning path is copied; NULL passes "disable".
 * 使用探测到的 I2C 地址。行长以像素时钟计，帧长以行计。
 * 链路参数原样传给驱动。调校路径只借用并复制；NULL 时传入 "disable"。
 */
int camera_open(
    camera_handle_t *cam_fd,
    uint8_t i2c_addr,
    uint32_t width, uint32_t height, uint32_t fps,
    uint32_t line_length, uint32_t frame_length,
    uint16_t mipiclk, uint16_t settle, uint16_t mclk,
    const char *tuning_file);

/* Use the wired MIPI RX. VIN writes to DDR (no ISP fly-by) and timestamps IPI VSYNC and triggers.
 * For VIN, ISP, VSE and GDC, an open failure clears the handle; a later failure can leave it set.
 * 使用传感器接入的 MIPI RX。VIN 写入 DDR（不走 ISP fly-by），并记录 IPI VSYNC 和触发时间戳。
 * VIN、ISP、VSE 和 GDC 打开失败时将句柄清零；后续配置失败时句柄可能仍非零。
 */
int vin_open(
    hbn_vnode_handle_t *vin,
    int mipi_rx,
    uint32_t width, uint32_t height, uint32_t fps);

/* Offline ISP: read RAW10 from DDR and write main frames as 8-bit NV12.
 * ISP 离线处理：从 DDR 读取 RAW10，主帧以 8 位 NV12 输出。
 */
int isp_open(
    hbn_vnode_handle_t *isp,
    uint32_t width, uint32_t height);

/* Return -1 if the aspect-preserving crop would require a fractional pixel; otherwise 0.
 * 等比裁剪所需尺寸不是整像素时返回 -1，否则返回 0。
 */
int roi_ratio_exact(uint32_t in_w, uint32_t in_h,
                    uint32_t out_w, uint32_t out_h);

/* Center the crop in input coordinates, keeping one axis at full size. Check roi_ratio_exact() first.
 * 在输入坐标系中居中裁剪，保留一轴的完整尺寸。先用 roi_ratio_exact() 检查。
 */
common_rect_t aspect_roi(uint32_t in_w, uint32_t in_h,
                         uint32_t out_w, uint32_t out_h);

/* The caller selects the hardware scaling channel and reads frames from that same channel.
 * 调用方选择硬件缩放通道，并从同一通道取帧。
 */
int vse_open(
    hbn_vnode_handle_t *vse,
    uint32_t in_w, uint32_t in_h,
    uint32_t out_w, uint32_t out_h,
    uint32_t vse_chn);

/* map is borrowed, not retained: grid_w * grid_h source sampling points (one per output pixel).
 * RemapPoint must match point_t's two doubles; there is no layout check.
 * Angles 90, 180 and 270 rotate the map; 0 and other values leave it unrotated.
 * Once allocated, gdc_bin belongs to the caller even on error; keep it until teardown_cam().
 * map 仅借用，不保留：共 grid_w * grid_h 个源采样点，每个输出像素对应一点。
 * RemapPoint 必须与 point_t 的两个 double 布局一致，代码不做检查。
 * 90、180 和 270 度会旋转映射；0 度及其他值不旋转。
 * gdc_bin 分配后归调用方所有，即使后续失败也要保留到 teardown_cam() 释放。
 */
int gdc_open(
    hbn_vnode_handle_t *gdc,
    hb_mem_common_buf_t *gdc_bin,
    const void *map, uint32_t in_w, uint32_t in_h,
    uint32_t grid_w, uint32_t grid_h, int install_angle);

/* VIN -> ISP -> [GDC] -> [VSE]; zero skips only GDC or VSE. Attach the camera last.
 * Read the last node on vse_chn, GDC channel 0, or ISP_MAIN_FRAME; use the outputs only on success.
 * VIN -> ISP -> [GDC] -> [VSE]；仅 GDC 和 VSE 可用零句柄跳过。最后挂接相机。
 * 从末级节点取帧：VSE 用 vse_chn，GDC 用通道 0，ISP 用 ISP_MAIN_FRAME；成功后才使用输出参数。
 */
int vflow_build(
    hbn_vflow_handle_t *vflow,
    camera_handle_t cam_fd,
    hbn_vnode_handle_t vin,
    hbn_vnode_handle_t isp,
    hbn_vnode_handle_t gdc,
    hbn_vnode_handle_t vse, uint32_t vse_chn,
    hbn_vnode_handle_t *out_node, uint32_t *out_chn);

/* Destroying the flow releases its added nodes. The caller handles GPIO reset afterward.
 * A nonzero gdc_bin fd is freed and the buffer descriptor cleared.
 * 销毁 flow 时释放已加入的节点，随后由调用方复位 GPIO。
 * gdc_bin 的 fd 非零时释放缓冲区，并清空描述符。
 */
void teardown_cam(
    hbn_vflow_handle_t vflow,
    camera_handle_t cam_fd,
    hbn_vnode_handle_t vin, hbn_vnode_handle_t isp,
    hbn_vnode_handle_t vse,
    hbn_vnode_handle_t gdc,
    hb_mem_common_buf_t *gdc_bin, int reset_gpio);

#ifdef __cplusplus
}
#endif

#endif
