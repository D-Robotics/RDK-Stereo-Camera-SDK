/**
 * @file rectify.cpp
 * @brief stereo_rectify implementation: fov_scale coarse+fine search, 32-px frame expansion, map generation.
 * @brief stereo_rectify 实现：fov_scale 粗+细搜索、32 像素取景框扩展、map 生成。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/rectify/rectify.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cstring>

namespace dsc {
namespace base {
namespace {

constexpr int    kProbeWidth   = 128;    // Fixed proxy width for the approximate search. 近似搜索使用的固定代理宽度
constexpr int    kGridStep     = 32;     // Fisheye frame-expansion step. 鱼眼取景框扩展步长
constexpr double kCoarseBegin  = 0.10;   // First fov_scale value in the bounded search. 有界搜索中的第一个 fov_scale 取值
constexpr double kCoarseEnd    = 2.00;   // last fov_scale tried by the coarse sweep 粗扫尝试的最后一个 fov_scale
constexpr double kCoarseStep   = 0.05;   // coarse sweep step 粗扫步长
constexpr double kFineStep     = 0.005;  // fine sweep step, over one coarse step around the coarse result 细扫步长，覆盖粗结果附近一个粗步长的范围。

int dist_count(DistModel m)
{
    return (m == DistModel::Fisheye) ? 4 : 8;
}

// Check that the sampled coordinates along all four mapped borders fall inside the source image.
// Upper bound uses src-1: bilinear interpolation needs the +1 neighbor pixel.
// Checking the borders alone suffices: with center-symmetric monotone distortion the sample
// range is enclosed by the borders, so interior points never go out of bounds first.
// 检查沿四条映射边界的采样坐标是否都落在源图像之内。
// 上界使用 src-1：双线性插值需要 +1 的相邻像素。
// 只检查边界即可：在中心对称的单调畸变下，采样范围被边界包围，
// 因此内部点不会先越界。
bool border_in_source(const cv::Mat &mx, const cv::Mat &my, int src_w, int src_h)
{
    const int   w = mx.cols, h = mx.rows;
    const float x_max = static_cast<float>(src_w - 1);
    const float y_max = static_cast<float>(src_h - 1);

    auto bad = [&](float x, float y){
        return x < 0.0F || x >= x_max || y < 0.0F || y >= y_max;
    };

    for(int x = 0; x < w; ++x){
        if(bad(mx.at<float>(0, x), my.at<float>(0, x)))
            return false;
        if(bad(mx.at<float>(h - 1, x), my.at<float>(h - 1, x)))
            return false;
    }
    for(int y = 0; y < h; ++y){
        if(bad(mx.at<float>(y, 0), my.at<float>(y, 0)))
            return false;
        if(bad(mx.at<float>(y, w - 1), my.at<float>(y, w - 1)))
            return false;
    }
    return true;
}

// Stereo rectification + principal-point centering. Fisheye uses fov_scale; pinhole uses alpha=0 followed by manual focal scaling.
// 立体校正 + 主点居中。鱼眼使用 fov_scale；针孔使用 alpha=0 之后再手动缩放焦距。
//
// new_image_size must be passed separately from out_size: probing always uses src_size and only the
// final generation uses out_size. Otherwise, when expanding the frame, the focal length would scale
// with the size, angular coverage would stay unchanged, the black-border check would always pass,
// and the expansion loop would never terminate.
// new_image_size 必须与 out_size 分开传入：探测始终使用 src_size，只有
// 最终生成才使用 out_size。否则在扩展取景框时，焦距会随尺寸一起缩放，
// 角度覆盖范围保持不变，黑边检查会一直通过，扩展循环永远不会结束。
//
// Outputs are the per-camera rectification rotations rect_lR/rect_rR, the 3x4 projection
// matrices proj_lP/proj_rP (principal point placed at the center of out_size), and the
// internally computed disparity-to-depth matrix Q, which is discarded by every caller.
// 输出为每个相机的校正旋转 rect_lR/rect_rR、3x4 投影矩阵 proj_lP/proj_rP
// （主点置于 out_size 的中心），以及内部计算的视差-深度矩阵 Q，
// 它会被每个调用方丢弃。
void stereo_rectify_impl(DistModel model,
                    const cv::Mat &lK, const cv::Mat &lD,
                    const cv::Mat &rK, const cv::Mat &rD,
                    const cv::Mat &R_r2l, const cv::Mat &t_r2l,
                    cv::Size src_size, cv::Size out_size,
                    cv::Size new_image_size, double fov_scale,
                    cv::Mat &rect_lR, cv::Mat &rect_rR,
                    cv::Mat &proj_lP, cv::Mat &proj_rP)
{
    cv::Mat Q;
    if(model == DistModel::Fisheye){
        cv::fisheye::stereoRectify(lK, lD, rK, rD, src_size, R_r2l, t_r2l,
                                   rect_lR, rect_rR, proj_lP, proj_rP, Q,
                                   cv::fisheye::CALIB_ZERO_DISPARITY,
                                   new_image_size, 0.0, fov_scale);
    } else{
        cv::stereoRectify(lK, lD, rK, rD, src_size, R_r2l, t_r2l,
                          rect_lR, rect_rR, proj_lP, proj_rP, Q,
                          cv::CALIB_ZERO_DISPARITY, 0.0, new_image_size);
        // pinhole has no fov_scale parameter; equivalently scale the focal length in P
        // 针孔模型没有 fov_scale 参数；等价地缩放 P 中的焦距。
        proj_lP.at<double>(0, 0) /= fov_scale;
        proj_lP.at<double>(1, 1) /= fov_scale;
        proj_rP.at<double>(0, 0) /= fov_scale;
        proj_rP.at<double>(1, 1) /= fov_scale;
    }
    proj_lP.at<double>(0, 2) = out_size.width * 0.5;
    proj_lP.at<double>(1, 2) = out_size.height * 0.5;
    proj_rP.at<double>(0, 2) = out_size.width * 0.5;
    proj_rP.at<double>(1, 2) = out_size.height * 0.5;
}

// Floating-point maps in source-image coordinates: CV_32FC1 is what the GDC remap consumes.
// 源图像坐标系下的浮点 map：GDC remap 使用的正是 CV_32FC1。
void undistort_rectify_map(DistModel model,
                           const cv::Mat &K, const cv::Mat &D,
                           const cv::Mat &rect_R, const cv::Mat &P,
                           cv::Size size, cv::Mat &mx, cv::Mat &my)
{
    if(model == DistModel::Fisheye)
        cv::fisheye::initUndistortRectifyMap(K, D, rect_R, P, size, CV_32FC1, mx, my);
    else
        cv::initUndistortRectifyMap(K, D, rect_R, P, size, CV_32FC1, mx, my);
}

// Single fov_scale attempt: rectify -> proxy map -> black-border check
// The returned bool is the acceptance criterion of the search: true when every sampled
// border pixel of both proxy maps stays inside the source image.
// 单次 fov_scale 尝试：校正 -> 代理 map -> 黑边检查
// 返回的 bool 是搜索的接受判据：当两个代理 map 的每个采样边界像素
// 都仍落在源图像内时为 true。
bool try_fov_scale(DistModel model,
                   const cv::Mat &lK, const cv::Mat &lD,
                   const cv::Mat &rK, const cv::Mat &rD,
                   const cv::Mat &R_r2l, const cv::Mat &t_r2l,
                   cv::Size src_size, cv::Size out_size, cv::Size probe_size,
                   double fov_scale)
{
    cv::Mat rect_lR, rect_rR, proj_lP, proj_rP;
    stereo_rectify_impl(model, lK, lD, rK, rD, R_r2l, t_r2l,
                        src_size, out_size, src_size, fov_scale,
                        rect_lR, rect_rR, proj_lP, proj_rP);

    // Scale P proportionally with the proxy size
    // 按代理尺寸成比例缩放 P。
    const double sx = static_cast<double>(probe_size.width) / out_size.width;
    const double sy = static_cast<double>(probe_size.height) / out_size.height;
    cv::Mat Pl = proj_lP.clone(), Pr = proj_rP.clone();
    for(cv::Mat *P : {&Pl, &Pr}){
        P->at<double>(0, 0) *= sx;
        P->at<double>(0, 2) *= sx;
        P->at<double>(1, 1) *= sy;
        P->at<double>(1, 2) *= sy;
    }

    cv::Mat lmx, lmy, rmx, rmy;
    undistort_rectify_map(model, lK, lD, rect_lR, Pl, probe_size, lmx, lmy);
    undistort_rectify_map(model, rK, rD, rect_rR, Pr, probe_size, rmx, rmy);

    return border_in_source(lmx, lmy, src_size.width, src_size.height) &&
           border_in_source(rmx, rmy, src_size.width, src_size.height);
}

// Proxy grid of a given output size: fixed width (kProbeWidth), height rounded to keep
// the output aspect ratio, so the criterion is evaluated on the same shape at a lower cost.
// 给定输出尺寸的代理网格：宽度固定（kProbeWidth），高度取整以保持
// 输出的长宽比，从而以更低的代价在相同形状上评估判据。
cv::Size probe_of(cv::Size out_size)
{
    int h = static_cast<int>(kProbeWidth * static_cast<double>(out_size.height) /
                                 out_size.width + 0.5);
    if(h < 1)
        h = 1;
    return cv::Size(kProbeWidth, h);
}

// Find the largest black-border-free fov_scale: coarse search first, then fine search in its neighborhood
// Both sweeps stop at the first rejected scale, which relies on the criterion being
// monotone in fov_scale for a fixed output size.  The fine sweep then walks up to one
// coarse step past the coarse result, so it can only refine it by at most that step.
// 找出无黑边的最大 fov_scale：先粗搜索，再在其邻域内细搜索
// 两次扫描都在遇到第一个被拒绝的取值时停止，这依赖于在固定输出尺寸下
// 判据对 fov_scale 是单调的。细扫随后最多走到粗结果之后一个粗步长处，
// 因此它最多只能在该步长范围内细化结果。
double find_best_fov_scale(DistModel model,
                           const cv::Mat &lK, const cv::Mat &lD,
                           const cv::Mat &rK, const cv::Mat &rD,
                           const cv::Mat &R_r2l, const cv::Mat &t_r2l,
                           cv::Size src_size, cv::Size out_size)
{
    const cv::Size probe = probe_of(out_size);

    double coarse = kCoarseBegin;
    for(double s = kCoarseBegin; s <= kCoarseEnd + 1e-4; s += kCoarseStep){
        if(!try_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                           src_size, out_size, probe, s))
            break;
        coarse = s;
    }

    double best = coarse;
    const double hi = coarse + kCoarseStep;
    for(double s = coarse + kFineStep; s <= hi + 1e-9; s += kFineStep){
        if(!try_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                           src_size, out_size, probe, s))
            break;
        best = s;
    }
    return best;
}

// Point transform from the left camera frame to the right camera frame, as expected by
// OpenCV stereoRectify(camera1=left, camera2=right): p_right = R_r2l*p_left + t_r2l.
// 从左相机坐标系到右相机坐标系的点变换，与
// OpenCV stereoRectify(camera1=left, camera2=right) 的期望一致：p_right = R_r2l*p_left + t_r2l。
void relative_pose(const double *lR, const double *lT,
                   const double *rR, const double *rT,
                   cv::Mat &R_r2l, cv::Mat &t_r2l)
{
    const cv::Mat lRm(3, 3, CV_64F, const_cast<double *>(lR));
    const cv::Mat rRm(3, 3, CV_64F, const_cast<double *>(rR));
    const cv::Mat lTm(3, 1, CV_64F, const_cast<double *>(lT));
    const cv::Mat rTm(3, 1, CV_64F, const_cast<double *>(rT));
    // The inputs are sensor-to-reference transforms:
    // p_ref = lR*p_left + lT = rR*p_right + rT.
    // 输入是从传感器到参考系的变换：p_ref = lR*p_left + lT = rR*p_right + rT。
    R_r2l = rRm.t() * lRm;
    t_r2l = rRm.t() * (lTm - rTm);
}

// OpenCV K from the calibration fields; the skew is 0 and the last row is (0,0,1).
// The returned matrix owns its data, so it does not alias the calibration.
// 由标定字段构造 OpenCV 的 K；skew 为 0，最后一行为 (0,0,1)。
// 返回的矩阵拥有自己的数据，因此不与标定共享内存。
cv::Mat k_mat(const CameraIntrinsics &c)
{
    return (cv::Mat_<double>(3, 3) << c.fx, 0.0, c.cx,
                                      0.0, c.fy, c.cy,
                                      0.0, 0.0, 1.0);
}

// Distortion vector in the length OpenCV expects for the model: 4 for fisheye, 8 for pinhole.
// 按该模型下 OpenCV 期望的长度构造畸变向量：鱼眼为 4，针孔为 8。
cv::Mat d_mat(const CameraIntrinsics &c, DistModel m)
{
    return cv::Mat(dist_count(m), 1, CV_64F,
                   const_cast<double *>(c.dist_coeffs)).clone();
}

// Unify the left/right focal lengths to min(fx,fy) and move the principal point to the output center
// 把左右焦距统一为 min(fx,fy)，并把主点移到输出中心。
void align_focal_and_center(cv::Mat &proj_lP, cv::Mat &proj_rP, int w, int h)
{
    const double f = std::min(proj_lP.at<double>(0, 0), proj_lP.at<double>(1, 1));
    for(cv::Mat *P : {&proj_lP, &proj_rP}){
        P->at<double>(0, 0) = f;
        P->at<double>(1, 1) = f;
        P->at<double>(0, 2) = w * 0.5;
        P->at<double>(1, 2) = h * 0.5;
    }
}

// fill_map flattens both OpenCV maps into one vector: output pixel (x,y) lands at
// index y*w + x, so the table is row-major with the top-left output pixel first.
// fill_map 把两个 OpenCV map 展平到一个向量中：输出像素 (x,y) 落在
// 索引 y*w + x 处，因此该表是行主序的，最左上角的输出像素排在最前。
void fill_map(std::vector<RemapPoint> *dst,
              const cv::Mat &mx, const cv::Mat &my, int w, int h)
{
    dst->resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
    for(int y = 0; y < h; ++y){
        for(int x = 0; x < w; ++x){
            RemapPoint &p = (*dst)[static_cast<std::size_t>(y) * w + x];
            p.x = mx.at<float>(y, x);
            p.y = my.at<float>(y, x);
        }
    }
}

// Virtual calibration write-back: zero distortion, shared focal, centered principal point, R = original R * rect_R^T, T kept unchanged.
// orig_R and the target R may alias the same memory, so clone the original R before writing back.
// Only fx/fy/cx/cy and the four matching K entries are written: the rest of K (skew and
// the last row) is left as the caller had it.
// 虚拟标定回写：零畸变、共享焦距、主点居中，R = 原始 R * rect_R^T，T 保持不变。
// orig_R 与目标 R 可能指向同一块内存，因此回写前先克隆原始 R。
// 只写入 fx/fy/cx/cy 以及 K 中对应的四个元素：K 的其余部分（skew 和
// 最后一行）保持调用方传入时的样子。
void write_virtual(CameraIntrinsics *k, double *R,
                   const double *orig_R,
                   const cv::Mat &P, const cv::Mat &rect_R)
{
    k->fx = P.at<double>(0, 0);
    k->fy = P.at<double>(1, 1);
    k->cx = P.at<double>(0, 2);
    k->cy = P.at<double>(1, 2);
    memset(k->dist_coeffs, 0, sizeof(k->dist_coeffs));
    k->K[0] = k->fx; k->K[2] = k->cx;
    k->K[4] = k->fy; k->K[5] = k->cy;

    const cv::Mat oR = cv::Mat(3, 3, CV_64F,
                               const_cast<double *>(orig_R)).clone();
    const cv::Mat virtR = oR * rect_R.t();
    memcpy(R, virtR.ptr<double>(), 9 * sizeof(double));
}

} // namespace

// Reject the configurations the search cannot handle before touching any output; nothing
// is written back on these paths, so a rejected call leaves *cal and the maps unchanged.
// 在触碰任何输出之前先拒绝搜索无法处理的配置；这些路径上不会回写任何内容，
// 因此被拒绝的调用会让 *cal 和各个 map 保持原样。
Status stereo_rectify(StereoImuModel *cal,
                      uint32_t src_w, uint32_t src_h,
                      uint32_t *grid_w, uint32_t *grid_h,
                      std::vector<RemapPoint> *left_map,
                      std::vector<RemapPoint> *right_map)
{
    if(cal == nullptr || grid_w == nullptr || grid_h == nullptr ||
        left_map == nullptr || right_map == nullptr)
        return Status::ParamError;
    if(src_w < 32 || src_h < 32)
        return Status::ParamError;
    if(cal->cam_left.fx <= 0.0 || cal->cam_left.fy <= 0.0 ||
        cal->cam_right.fx <= 0.0 || cal->cam_right.fy <= 0.0)
        return Status::ParamError;

    const DistModel model = cal->cam_left.dist_model;
    if(cal->cam_right.dist_model != model)
        return Status::ParamError;   // left/right distortion models differ 左右畸变模型不同。

    const cv::Mat lK = k_mat(cal->cam_left),  lD = d_mat(cal->cam_left, model);
    const cv::Mat rK = k_mat(cal->cam_right), rD = d_mat(cal->cam_right, model);

    cv::Mat R_r2l, t_r2l;
    relative_pose(cal->cam_left_R, cal->cam_left_T,
                  cal->cam_right_R, cal->cam_right_T, R_r2l, t_r2l);

    const cv::Size src_size(static_cast<int>(src_w), static_cast<int>(src_h));
    cv::Size out_size(static_cast<int>(src_w), static_cast<int>(src_h));

    // Pinhole alpha=0 already applies zoom+shift for maximum framing; no search or frame expansion needed
    // The grid starts at the source size and only grows for fisheye.
    // 针孔的 alpha=0 已经应用 zoom+shift 以获得最大的取景范围；无需搜索或扩展取景框
    // 网格从源尺寸开始，只有鱼眼才会增长。
    double fov_scale = 1.0;
    if(model == DistModel::Fisheye){
        fov_scale = find_best_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                                        src_size, out_size);

        // After touching the border, expand in 32-pixel steps to the maximum frame: grow only in the direction that stays black-border-free
        // Both directions are probed first, so the axis that is already blocked is not
        // grown; a single direction is then extended until that probe fails, and if
        // neither direction is free the grid keeps the source size.
        // 触及边界后，以 32 像素步长扩展到最大取景框：只朝仍无黑边的方向增长
        // 先探测两个方向，因此已经受阻的轴不会被增长；
        // 随后沿单个方向持续扩展直到该探测失败；如果两个方向都不可用，
        // 网格保持源尺寸。
        const cv::Size grow_w(out_size.width + kGridStep, out_size.height);
        const cv::Size grow_h(out_size.width, out_size.height + kGridStep);
        const bool w_ok = try_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                                        src_size, grow_w, probe_of(grow_w), fov_scale);
        const bool h_ok = try_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                                        src_size, grow_h, probe_of(grow_h), fov_scale);

        if(w_ok != h_ok){
            const bool grow_width = w_ok;
            for(;;){
                cv::Size next = out_size;
                if(grow_width)
                    next.width += kGridStep;
                else
                    next.height += kGridStep;
                if(!try_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                                   src_size, next, probe_of(next), fov_scale))
                    break;
                out_size = next;
            }
        }
    }

    // Final generation at the accepted output size; from here on the calibration is
    // modified, which is why every rejection above happens earlier.
    // 在接受的输出尺寸下做最终生成；从此处开始标定会被修改，
    // 这也是上面所有拒绝都提前发生的原因。
    cv::Mat rect_lR, rect_rR, proj_lP, proj_rP;
    stereo_rectify_impl(model, lK, lD, rK, rD, R_r2l, t_r2l,
                        src_size, out_size, out_size, fov_scale,
                        rect_lR, rect_rR, proj_lP, proj_rP);

    align_focal_and_center(proj_lP, proj_rP, out_size.width, out_size.height);

    cv::Mat lmx, lmy, rmx, rmy;
    undistort_rectify_map(model, lK, lD, rect_lR, proj_lP, out_size, lmx, lmy);
    undistort_rectify_map(model, rK, rD, rect_rR, proj_rP, out_size, rmx, rmy);

    *grid_w = static_cast<uint32_t>(out_size.width);
    *grid_h = static_cast<uint32_t>(out_size.height);
    fill_map(left_map,  lmx, lmy, out_size.width, out_size.height);
    fill_map(right_map, rmx, rmy, out_size.width, out_size.height);

    // In-place write-back of virtual intrinsics/extrinsics (zero distortion, shared focal, centered principal point; T unchanged)
    // Both cameras keep their translation, so the stereo baseline survives rectification.
    // 就地回写虚拟内参/外参（零畸变、共享焦距、主点居中；T 不变）
    // 两个相机都保留各自的平移，因此立体基线在校正后依然存在。
    write_virtual(&cal->cam_left,  cal->cam_left_R,  cal->cam_left_R,  proj_lP, rect_lR);
    write_virtual(&cal->cam_right, cal->cam_right_R, cal->cam_right_R, proj_rP, rect_rR);
    return Status::Ok;
}

} // namespace base
} // namespace dsc
