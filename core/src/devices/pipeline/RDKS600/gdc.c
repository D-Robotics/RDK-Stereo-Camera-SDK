/**
 * @file gdc.c
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKS600.h"

#include <stdlib.h>
#include <string.h>

/* S600 GDC uses 16-byte strides (BSP ALIGN_16 in create_gdc_node()).
 * S600 的 GDC 行跨度按 16 字节对齐，沿用 BSP create_gdc_node() 的 ALIGN_16。 */
#define DSC_ALIGN_16(v) (((v) + 15u) & ~15u)

#define DSC_GDC_BUF_NUM 3

int gdc_open(hbn_vnode_handle_t *gdc, hb_mem_common_buf_t *gdc_bin,
             const void *map, uint32_t in_w, uint32_t in_h,
             uint32_t grid_w, uint32_t grid_h, int install_angle)
{
    const uint32_t npts = grid_w * grid_h;
    const point_t *m = (const point_t *)map;

    const double xmax = (double)in_w - 1.0;
    const double ymax = (double)in_h - 1.0;
    /* Clamp in map coordinates: 90° and 270° swap the source bounds.
     * 按映射坐标限幅；安装角为 90° 或 270° 时交换源图的宽高边界。 */
    const int    swap     = (install_angle == 90 || install_angle == 270);
    const double src_xmax = swap ? ymax : xmax;
    const double src_ymax = swap ? xmax : ymax;

    point_t *pts = (point_t *)malloc((size_t)npts * sizeof(point_t));
    if (!pts)
        return -1;

    for (uint32_t i = 0; i < npts; ++i) {
        /* Compensate for GDC bilinear sampling by +0.5 px; cap at the upper bounds to avoid black edges.
         * 为 GDC 双线性采样补偿 +0.5 像素，并限制上界以避免黑边。 */
        double x = m[i].x + 0.5;
        double y = m[i].y + 0.5;
        if (x > src_xmax) x = src_xmax;
        if (y > src_ymax) y = src_ymax;
        /* Map the rotated sampling coordinates back into the GDC input frame.
         * 把旋转后的采样坐标转回 GDC 输入帧的坐标系。 */
        switch (install_angle) {
        case 90:  pts[i].x = y;        pts[i].y = ymax - x; break;
        case 180: pts[i].x = xmax - x; pts[i].y = ymax - y; break;
        case 270: pts[i].x = xmax - y; pts[i].y = x;        break;
        default:  pts[i].x = x;        pts[i].y = y;        break;
        }
    }

    param_t param = {
        .format = FMT_SEMIPLANAR_420,   /* NV12, matching the ISP output.
                                           * NV12，与 ISP 输出一致。 */
        .in = { .w = in_w, .h = in_h },
        .out = { .w = grid_w, .h = grid_h },
        .fov = 180.0,
        .diameter = in_h,
    };

    window_t win = {
        .transform = CUSTOM,
        .strength = 1.0,
        .strengthY = 1.0,
        .keep_ratio = 1,
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
            .w = (int32_t)grid_w - 1,       /* Last index, not point count.
                                               * 填末尾索引，不是点数。 */
            .h = (int32_t)grid_h - 1,
            .centerx = (double)(grid_w / 2),
            .centery = (double)(grid_h / 2),
            .points = pts,
        },
    };

    /* The S-series generator returns void * (X5: uint32_t *). Release it with
     * hbn_free_gdc_cfg(), not free().
     * S 系列生成器返回 void *，X5 返回 uint32_t *。必须用 hbn_free_gdc_cfg() 释放。 */
    void    *cfg_buf  = NULL;
    uint64_t cfg_size = 0;
    if (hbn_gen_gdc_cfg(&param, &win, 1, &cfg_buf, &cfg_size) != 0 ||
        cfg_buf == NULL || cfg_size == 0) {
        if (cfg_buf != NULL)
            hbn_free_gdc_cfg((uint32_t *)cfg_buf);
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
        hbn_free_gdc_cfg((uint32_t *)cfg_buf);
        free(pts);
        return -1;
    }
    memcpy(gdc_bin->virt_addr, cfg_buf, (size_t)cfg_size);
    hbn_free_gdc_cfg((uint32_t *)cfg_buf);
    free(pts);

    if (hb_mem_flush_buf(gdc_bin->fd, 0, cfg_size) != 0)
        return -1;

    gdc_settings_t settings = { 0 };
    settings.gdc_config.config_addr   = gdc_bin->phys_addr;
    settings.gdc_config.config_size   = (uint32_t)gdc_bin->size;
    settings.gdc_config.input_width   = in_w;
    settings.gdc_config.input_height  = in_h;
    settings.gdc_config.input_stride  = DSC_ALIGN_16(in_w);
    settings.gdc_config.output_width  = grid_w;
    settings.gdc_config.output_height = grid_h;
    settings.gdc_config.output_stride = DSC_ALIGN_16(grid_w);
    settings.gdc_config.total_planes  = 2;
    settings.binary_ion_id            = gdc_bin->share_id;
    settings.binary_offset            = gdc_bin->offset;
    settings.magicNumber              = DSC_S600_MAGIC_NUMBER;

    if (hbn_vnode_open(HB_GDC, 0, AUTO_ALLOC_ID, gdc) != 0) {
        *gdc = 0;
        return -1;
    }

    if (hbn_vnode_set_attr(*gdc, &settings) != 0 ||
        hbn_vnode_set_ichn_attr(*gdc, 0, &settings) != 0 ||
        hbn_vnode_set_ochn_attr(*gdc, 0, &settings) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = DSC_GDC_BUF_NUM,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED,
    };
    if (hbn_vnode_set_ochn_buf_attr(*gdc, 0, &alloc) != 0)
        return -1;
    return 0;
}
