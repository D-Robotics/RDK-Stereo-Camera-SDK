/**
 * @file pym.c
 * Buffer counts, handshaking and blanking follow the SC132GS BSP preset:
 * hobot_mipi_cam: src/s100/sensor/sc132gs_linear_1088x1280_raw10_30fps_1lane.c.
 * 缓冲数量、握手和消隐参数沿用上述 SC132GS BSP 配置。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKS100.h"

#include <string.h>

/* PYM requires 16-byte strides, as in the BSP.
 * PYM 的行跨度按 BSP 要求对齐到 16 字节。 */
#define DSC_ALIGN_16(v) (((v) + 15u) & ~15u)

#define DSC_PYM_BL_MAX_EN      5
#define DSC_PYM_SUFFIX_HB      68
#define DSC_PYM_PREFIX_HB      2
#define DSC_PYM_SUFFIX_VB      20
#define DSC_PYM_PREFIX_VB      2
#define DSC_PYM_OUTPUT_BUF_NUM 6
#define DSC_PYM_FB_BUF_NUM     2
#define DSC_PYM_BUF_NUM        3

/* PYM rejects a region-to-output ratio of 2x or more. Try up to four
 * halvings, rounding each layer down to even dimensions. Selector 0 reads
 * full resolution; selector 1 starts at layer 0 (half resolution).
 * 裁剪区域与输出的尺寸比达到 2 倍时，PYM 会拒绝配置。最多逐级减半四次，
 * 每层尺寸向下取偶数。选择器 0 读全分辨率，选择器 1 从第 0 层（半分辨率）起。 */
static void pym_layer_pick(uint32_t in_w, uint32_t in_h,
                           uint32_t out_w, uint32_t out_h,
                           uint32_t *bl_w, uint32_t *bl_h,
                           uint32_t *sel, uint32_t *layer)
{
    uint32_t depth = 0;
    uint32_t w = in_w, h = in_h;

    while (depth < 4 && (out_w <= (w >> 1) && out_h <= (h >> 1))) {
        w >>= 1; h >>= 1;
        w &= ~1u; h &= ~1u;
        depth++;
    }

    *bl_w  = w;
    *bl_h  = h;
    *sel   = (depth == 0) ? 0u : 1u;
    *layer = (depth == 0) ? 0u : depth - 1u;
}

typedef struct {
    uint32_t sel, layer;
    uint32_t bl_w, bl_h;
    uint32_t start_left, start_top;
    uint32_t region_w, region_h;
    uint32_t out_w, out_h;
} pym_plan_t;

static void pym_plan(uint32_t in_w, uint32_t in_h,
                     uint32_t out_w, uint32_t out_h, pym_plan_t *p)
{
    const dsc_rect_t roi = aspect_roi(in_w, in_h, out_w, out_h);

    pym_layer_pick(in_w, in_h, out_w, out_h,
                   &p->bl_w, &p->bl_h, &p->sel, &p->layer);
    p->out_w = out_w;
    p->out_h = out_h;
    p->start_left = (roi.x * p->bl_w) / in_w;
    p->start_top  = (roi.y * p->bl_h) / in_h;
    p->region_w = ((roi.w * p->bl_w) / in_w) & ~1u;
    p->region_h = ((roi.h * p->bl_h) / in_h) & ~1u;
}

int roi_ratio_exact(uint32_t in_w, uint32_t in_h,
                    uint32_t out_w, uint32_t out_h)
{
    if (in_w * out_h > out_w * in_h) {
        return (in_h * out_w % out_h == 0) ? 0 : -1;
    }
    return (in_w * out_h % out_w == 0) ? 0 : -1;
}

dsc_rect_t aspect_roi(uint32_t in_w, uint32_t in_h,
                        uint32_t out_w, uint32_t out_h)
{
    dsc_rect_t roi;
    if (in_w * out_h > out_w * in_h) {
        roi.h = in_h;
        roi.w = in_h * out_w / out_h;
        roi.x = (in_w - roi.w) / 2;
        roi.y = 0;
    } else {
        roi.w = in_w;
        roi.h = in_w * out_h / out_w;
        roi.x = 0;
        roi.y = (in_h - roi.h) / 2;
    }
    return roi;
}

int pym_open(hbn_vnode_handle_t *pym, uint32_t in_w, uint32_t in_h,
             uint32_t out_w, uint32_t out_h, uint32_t hw_id, uint32_t slot_id)
{
    pym_cfg_t cfg = { 0 };

    /* M2M accepts ISP or GDC DDR output through a direct bind, without a CPU bridge.
     * M2M 可以直接绑定 ISP 或 GDC 的 DDR 输出，无需 CPU 搬运。 */
    cfg.hw_id    = hw_id;
    cfg.slot_id  = slot_id;
    cfg.pym_mode = PYM_M2M_MODE;

    cfg.pingpong_ring        = 0;
    cfg.output_buf_num       = DSC_PYM_OUTPUT_BUF_NUM;
    cfg.fb_buf_num           = DSC_PYM_FB_BUF_NUM;
    cfg.timeout              = 0;
    cfg.threshold_time       = 0;
    cfg.layer_num_trans_next = 0;
    cfg.layer_num_share_prev = -1;
    cfg.out_buf_noinvalid    = 1;
    cfg.out_buf_noncached    = 0;
    cfg.in_buf_noclean       = 1;
    cfg.in_buf_noncached     = 0;

    cfg.chn_ctrl.invalid_head_lines  = 0;
    cfg.chn_ctrl.src_in_width        = in_w;
    cfg.chn_ctrl.src_in_height       = in_h;
    cfg.chn_ctrl.src_in_stride_y     = DSC_ALIGN_16(in_w);
    cfg.chn_ctrl.src_in_stride_uv    = DSC_ALIGN_16(in_w);
    cfg.chn_ctrl.suffix_hb_val       = DSC_PYM_SUFFIX_HB;
    cfg.chn_ctrl.prefix_hb_val       = DSC_PYM_PREFIX_HB;
    cfg.chn_ctrl.suffix_vb_val       = DSC_PYM_SUFFIX_VB;
    cfg.chn_ctrl.prefix_vb_val       = DSC_PYM_PREFIX_VB;
    cfg.chn_ctrl.bl_max_layer_en     = DSC_PYM_BL_MAX_EN;
    cfg.chn_ctrl.ds_roi_uv_bypass    = 0;

    pym_plan_t plan;
    pym_plan(in_w, in_h, out_w, out_h, &plan);
    cfg.chn_ctrl.ds_roi_sel[0]   = (uint8_t)plan.sel;
    cfg.chn_ctrl.ds_roi_layer[0] = (uint8_t)plan.layer;
    cfg.chn_ctrl.ds_roi_en       = (uint8_t)(1u << 0);

    roi_box_t *box = &cfg.chn_ctrl.ds_roi_info[0];
    /* ROI offsets and sizes are in layer coordinates. The driver checks
     * start + region <= layer size, so a full-layer region cannot also have an offset.
     * ROI 的偏移和尺寸均以选中层为基准。驱动检查 start + region <= 层尺寸，
     * 所以裁剪区域若占满整层，偏移必须为 0。 */
    box->start_left    = (uint16_t)plan.start_left;
    box->start_top     = (uint16_t)plan.start_top;
    box->region_width  = (uint16_t)plan.region_w;
    box->region_height = (uint16_t)plan.region_h;
    box->out_width     = out_w;
    box->out_height    = out_h;
    box->wstride_y     = DSC_ALIGN_16(out_w);
    box->wstride_uv    = DSC_ALIGN_16(out_w);
    box->vstride       = out_h;

    /* The driver validates only enabled slots, but copies the whole config.
     * Preset the unused slots to reduced layers.
     * 驱动只校验已启用的槽位，但会复制整份配置。未用槽位预设为降采样层。 */
    for (uint32_t i = 1; i < MAX_DS_NUM; i++) {
        cfg.chn_ctrl.ds_roi_sel[i]   = 1;
        cfg.chn_ctrl.ds_roi_layer[i] = (uint8_t)(i - 1);
    }

    cfg.magicNumber = DSC_S100_MAGIC_NUMBER;

    if (hbn_vnode_open(HB_PYM, cfg.hw_id, AUTO_ALLOC_ID, pym) != 0) {
        *pym = 0;
        return -1;
    }
    if (hbn_vnode_set_attr(*pym, &cfg) != 0 ||
        hbn_vnode_set_ichn_attr(*pym, 0, &cfg) != 0 ||
        hbn_vnode_set_ochn_attr(*pym, 0, &cfg) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = DSC_PYM_BUF_NUM,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED |
                 HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF,
    };
    if (hbn_vnode_set_ochn_buf_attr(*pym, 0, &alloc) != 0)
        return -1;
    return 0;
}
