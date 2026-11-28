/**
 * @file rectify.hpp
 * @brief Binocular stereo rectification: max-frame map search + virtual calibration write-back.
 * @brief 双目立体校正：最大取景框的 map 搜索 + 虚拟标定回写。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSC_BASE_RECTIFY_HPP
#define DSC_BASE_RECTIFY_HPP

#include <cstdint>
#include <vector>

#include "types.hpp"

namespace dsc {
namespace base {

/**
 * Binocular stereo rectification: automatically find the maximum Map + scale factor.
 * 双目立体校正：自动求出最大的 Map 与缩放系数。
 *
 * Behavior:
 * 行为：
 * - Output grid size:
 *   - 输出网格尺寸：
 *     - Undistort using the source image size (e.g. 1088x1280);
 *       - 使用源图像尺寸（如 1088x1280）做去畸变；
 *     - Use the source size as the frame, align output grid centers to the undistorted
 *       optical center, and auto-search the maximum frame;
 *       - 以源尺寸作为取景框，把输出网格中心对齐到去畸变后的光心，
 *         并自动搜索最大的取景框；
 *     - Pinhole model: output a maximum-frame grid Map at the source size (e.g. 1088x1280);
 *       - 针孔模型：在源尺寸（如 1088x1280）下输出最大取景框的网格 Map；
 *     - Fisheye model: once one side (e.g. width) is tangent to the undistorted
 *       butterfly-region boundary, grow the other side in 32-pixel steps until that side
 *       also touches the border (e.g. h); output the expanded maximum grid (e.g. 1088x1312).
 *       - 鱼眼模型：当一侧（如宽度）贴到去畸变后的蝴蝶形区域边界后，
 *         以 32 像素步长增长另一侧，直到该侧也触及边界（如 h）；
 *         输出扩展后的最大网格（如 1088x1312）。
 * - Distortion correction:
 *   - 畸变校正：
 *     - Automatically align the left/right images' fx, fy
 *       - 自动对齐左右图像的 fx、fy。
 *     - After applying the rectification Map, image optical center = image center
 *       - 应用校正 Map 后，图像光心 = 图像中心。
 *     - Correct the post-rectification intrinsics/extrinsics and automatically write the
 *       virtual intrinsics/extrinsics back to *cal
 *       - 修正校正后的内参/外参，并把虚拟内参/外参自动写回 *cal。
 *
 * @param[in,out] cal      Calibration of the stereo pair, see the parameter notes below.
 * @param[in,out] cal      双目相机的标定，参见下面的参数说明。
 * @param[in]     src_w    Width of one source image in pixels.
 * @param[in]     src_w    单幅源图像的宽度，单位像素。
 * @param[in]     src_h    Height of one source image in pixels.
 * @param[in]     src_h    单幅源图像的高度，单位像素。
 * @param[out]    grid_w   Width of the rectified grid in pixels.
 * @param[out]    grid_w   校正后网格的宽度，单位像素。
 * @param[out]    grid_h   Height of the rectified grid in pixels.
 * @param[out]    grid_h   校正后网格的高度，单位像素。
 * @param[out]    left_map Left remap table, overwritten; resized to grid_w * grid_h entries
 *                         in row-major order (row 0 = top row), each entry sampling the
 *                         source image as described by RemapPoint.
 * @param[out]    left_map 左目重映射表，会被覆盖；调整为 grid_w * grid_h 个条目，
 *                         按行主序排列（第 0 行 = 顶行），每个条目按 RemapPoint 的描述对源图像采样。
 * @param[out]    right_map Right remap table, filled like left_map.
 * @param[out]    right_map 右目重映射表，填充方式与 left_map 相同。
 * @return Status::Ok on success, Status::ParamError when a pointer is null, the source
 *         image is smaller than 32x32 pixels, a focal length is not positive, or the two
 *         cameras use different distortion models.
 * @return 成功时返回 Status::Ok；指针为空、源图像小于 32x32 像素、焦距不为正，
 *         或两个相机使用不同的畸变模型时返回 Status::ParamError。
 *
 * @note The caller owns every output: grid_w/grid_h are written but not read, the two map
 *       vectors are resized and filled, and their previous contents are discarded.
 * @note 所有输出都归调用方所有：grid_w/grid_h 只写不读，两个 map 向量会被调整大小并填充，
 *       其原有内容被丢弃。
 * @note The calibration is modified in place and only on Status::Ok: the camera intrinsics
 *       of both cameras are replaced by the virtual (rectified) ones -- zero distortion,
 *       shared focal length, principal point at the grid center -- and the extrinsic
 *       rotations are multiplied by the rectification rotation, while the translations and
 *       the IMU intrinsics are left as they were.  Pass a copy if the original calibration
 *       is still needed.
 * @note 标定只在返回 Status::Ok 时就地修改：两个相机的内参被替换为虚拟（校正后）内参——
 *       零畸变、共享焦距、主点位于网格中心——外参旋转乘以校正旋转，
 *       而平移和 IMU 内参保持原样。如果仍然需要原始标定，请传入一份拷贝。
 */
Status stereo_rectify(StereoImuModel *cal,
                      uint32_t src_w, uint32_t src_h,
                      uint32_t *grid_w, uint32_t *grid_h,
                      std::vector<RemapPoint> *left_map,
                      std::vector<RemapPoint> *right_map);

} // namespace base
} // namespace dsc

#endif // DSC_BASE_RECTIFY_HPP
