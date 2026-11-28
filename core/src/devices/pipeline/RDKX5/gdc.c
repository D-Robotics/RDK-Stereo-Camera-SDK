/**
 * @file gdc.c
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKX5.h"

#include <stdlib.h>
#include <string.h>

int gdc_open(hbn_vnode_handle_t *gdc, hb_mem_common_buf_t *gdc_bin,
             const void *map, uint32_t in_w, uint32_t in_h,
             uint32_t grid_w, uint32_t grid_h, int install_angle)
{
    const uint32_t npts = grid_w * grid_h;
    const point_t *m = (const point_t *)map;

    const double xmax = (double)in_w - 1.0;
    const double ymax = (double)in_h - 1.0;
    // The map uses rotated axes; swap the clamp bounds for 90° and 270°.
    // 映射表采用旋转后的坐标轴；90° 和 270° 时交换钳位上界。
    const int    swap     = (install_angle == 90 || install_angle == 270);
    const double src_xmax = swap ? ymax : xmax;
    const double src_ymax = swap ? xmax : ymax;

    point_t *pts = (point_t *)malloc((size_t)npts * sizeof(point_t));
    if (!pts)
        return -1;

    for (uint32_t i = 0; i < npts; ++i) {
        /* Shift by +0.5 px for bilinear sampling; cap at the upper edge to avoid black borders.
           双线性采样补偿 +0.5 像素；钳位上界，避免黑边。 */
        double x = m[i].x + 0.5;
        double y = m[i].y + 0.5;
        if (x > src_xmax) x = src_xmax;
        if (y > src_ymax) y = src_ymax;
        /* Map the sampling point back into sensor coordinates.
           将采样点转换回传感器坐标。 */
        switch (install_angle) {
        case 90:  pts[i].x = y;        pts[i].y = ymax - x; break;
        case 180: pts[i].x = xmax - x; pts[i].y = ymax - y; break;
        case 270: pts[i].x = xmax - y; pts[i].y = x;        break;
        default:  pts[i].x = x;        pts[i].y = y;        break;
        }
    }

    param_t param = {
        .format = FMT_SEMIPLANAR_420,   /* NV12, as output by ISP. 与 ISP 输出的 NV12 一致。 */
        .in = { .w = in_w, .h = in_h },
        .out = { .w = grid_w, .h = grid_h },
        /* Custom generator: FOV in degrees, diameter in input pixels.
           自定义映射生成参数：FOV 用度，直径用输入像素。 */
        .fov = 180.0,
        .diameter = in_h,
    };

    window_t win = {
        .transform = CUSTOM,
        .strength = 1.0,
        .strengthY = 1.0,
        .keep_ratio = 1,
        /* Generator angles for CUSTOM mode.
           CUSTOM 模式的生成器角度参数。 */
        .FOV_h = 90.0,
        .FOV_w = 90.0,
        .trapezoid_left_angle = 90.0,
        .trapezoid_right_angle = 90.0,
        .out_r = { .w = grid_w, .h = grid_h },
        .input_roi_r = { .w = in_w, .h = in_h },
        .zoom = 1.0,
        .custom = {
            .full_tile_calc = 1,
            .tile_incr_x = 50,
            .tile_incr_y = 50,
            .w = (int32_t)grid_w - 1,       /* Last zero-based grid index. 从 0 起算的网格末索引。 */
            .h = (int32_t)grid_h - 1,
            .centerx = (double)(grid_w / 2),
            .centery = (double)(grid_h / 2),
            .points = pts,
        },
    };

    uint32_t *cfg_buf  = NULL;
    uint64_t  cfg_size = 0;
    if (hbn_gen_gdc_bin(&param, &win, 1, &cfg_buf, &cfg_size) != 0 ||
        cfg_buf == NULL || cfg_size == 0) {
        free(cfg_buf);
        free(pts);
        return -1;
    }

    const int64_t flags = HB_MEM_USAGE_MAP_INITIALIZED |
                          HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
                          HB_MEM_USAGE_CPU_READ_OFTEN |
                          HB_MEM_USAGE_CPU_WRITE_OFTEN |
                          HB_MEM_USAGE_CACHED;
    memset(gdc_bin, 0, sizeof(*gdc_bin));
    if (hb_mem_alloc_com_buf(cfg_size, flags, gdc_bin) != 0 ||
        gdc_bin->virt_addr == NULL) {
        free(cfg_buf);
        free(pts);
        return -1;
    }
    memcpy(gdc_bin->virt_addr, cfg_buf, (size_t)cfg_size);
    free(cfg_buf);
    free(pts);

    /* Make the cached binary visible to the GDC before attaching it.
       挂接配置二进制前先刷缓存，使 GDC 能读到配置。 */
    if (hb_mem_flush_buf(gdc_bin->fd, 0, cfg_size) != 0)
        return -1;

    if (hbn_vnode_open(HB_GDC, 0, AUTO_ALLOC_ID, gdc) != 0) {
        *gdc = 0;
        return -1;
    }

    gdc_attr_t attr = {
        .config_addr   = gdc_bin->phys_addr,
        .config_size   = (uint32_t)gdc_bin->size,
        .binary_ion_id = gdc_bin->share_id,
        .binary_offset = gdc_bin->offset,
        .total_planes  = 2,
    };
    if (hbn_vnode_set_attr(*gdc, &attr) != 0)
        return -1;

    gdc_ichn_attr_t ichn = {
        .input_width  = in_w,
        .input_height = in_h,
        .input_stride = in_w,
    };
    if (hbn_vnode_set_ichn_attr(*gdc, 0, &ichn) != 0)
        return -1;

    gdc_ochn_attr_t ochn = {
        .output_width  = grid_w,
        .output_height = grid_h,
        .output_stride = grid_w,
    };
    if (hbn_vnode_set_ochn_attr(*gdc, 0, &ochn) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = 5,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED,
    };
    if (hbn_vnode_set_ochn_buf_attr(*gdc, 0, &alloc) != 0)
        return -1;
    return 0;
}
