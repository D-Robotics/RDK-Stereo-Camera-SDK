/**
 * @file szygsjkj_stereo_fisheye_v1p2r0n4.cpp
 * @brief SZYGSJKJ stereo (no IMU), fisheye, v1.2, rotation 0 deg, 4 distortion coefficients.
 * @brief SZYGSJKJ 双目（无 IMU），鱼眼，v1.2，旋转 0 度，4 个畸变系数。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/eeprom/eeprom.hpp"

#include <cstring>

namespace dsc {
namespace eeprom {
namespace {

// ============================== Identification 识别 ==============================
constexpr uint16_t kHeaderSize = 15;
constexpr uint16_t kChecksumOff = kHeaderSize;
constexpr uint8_t  kChecksumLen = 14;

constexpr uint8_t kHeader[kHeaderSize] = {
    0x53, 0x5a, 0x59, 0x47, 0x53, 0x4a, 0x4b, 0x4a,  // "SZYGSJKJ"
    0x01, 0x01,   // stereo, no IMU 双目，无 IMU
    0x01, 0x02,   // v1.2
    0x00,         // rotation index 0 (0 deg) 旋转索引 0（0 度）
    0x04,         // 4 distortion coefficients 4 个畸变系数
    0x00,
};

// ============================== Layout offsets 布局偏移 ==============================
constexpr uint16_t kLBlock   = 0x0010;
constexpr uint16_t kRBlock   = 0x0048;
constexpr uint16_t kStereoR  = 0x008C;
constexpr uint16_t kStereoT  = 0x00B0;
constexpr uint16_t kSize     = 0x00C0;   // camera calibration only; there is no IMU area 仅相机标定；没有 IMU 区

constexpr uint16_t kDistOff   = 0x0018;
constexpr uint8_t  kDistCount = 4;

double read_float(const uint8_t *buf, uint16_t off)
{
    float v;
    memcpy(&v, &buf[off], sizeof(v));
    return v;
}

void read_intrinsics(const uint8_t *buf, CameraIntrinsics *cam, uint16_t off)
{
    cam->fx = read_float(buf, off + 0x08);
    cam->fy = read_float(buf, off + 0x0C);
    cam->cx = read_float(buf, off + 0x10);
    cam->cy = read_float(buf, off + 0x14);

    cam->K[0] = cam->fx;
    cam->K[2] = cam->cx;
    cam->K[4] = cam->fy;
    cam->K[5] = cam->cy;
    cam->K[8] = 1.0f;

    cam->dist_model = DistModel::Fisheye;
    for(uint8_t i = 0; i < kDistCount; i++)
        cam->dist_coeffs[i] = read_float(buf, off + kDistOff + i * 4);
}

bool probe(base::I2cDevice &bus)
{
    uint8_t buf[kHeaderSize + 1];
    if(bus.readBurst16(0x0000, buf, sizeof(buf)) != Status::Ok)
        return false;

    uint16_t sum = 0;
    for(uint8_t i = 0; i < kChecksumLen; i++)
        sum = static_cast<uint16_t>(sum + buf[i]);
    if(static_cast<uint8_t>((sum % 255) + 1) != buf[kChecksumOff])
        return false;

    return memcmp(buf, kHeader, kHeaderSize) == 0;
}

Status read(base::I2cDevice &bus, StereoImuModel *out)
{
    if(!out)
        return Status::ParamError;

    uint8_t buf[kSize];
    if(bus.readBurst16(0x0000, buf, sizeof(buf)) != Status::Ok)
        return Status::HwError;
    if(memcmp(buf, kHeader, kHeaderSize) != 0)
        return Status::HwError;

    *out = StereoImuModel{};

    // The EEPROM L/R blocks map directly to the module's left and right cameras.
    // EEPROM 的 L/R 块直接对应模组的左目与右目相机。
    read_intrinsics(buf, &out->cam_left,  kLBlock);
    read_intrinsics(buf, &out->cam_right, kRBlock);

    out->install_angle = 0;

    // The right camera is the common reference; the stored pose belongs to the left camera.
    // 右目相机是公共参考；存储的位姿属于左目相机。
    double r_stored[9];
    double t_stored[3];
    for(uint8_t i = 0; i < 9; i++)
        r_stored[i] = read_float(buf, kStereoR + i * 4);
    for(uint8_t i = 0; i < 3; i++)
        t_stored[i] = read_float(buf, kStereoT + i * 4);

    for(uint8_t i = 0; i < 9; i++){
        out->cam_right_R[i] = (i % 4 == 0) ? 1.0 : 0.0;
        out->cam_left_R[i]  = r_stored[i];
    }
    for(uint8_t i = 0; i < 3; i++){
        out->cam_right_T[i] = 0.0;
        out->cam_left_T[i]  = t_stored[i];
    }

    // No IMU is present; all IMU fields stay zero-initialised.
    // 没有 IMU；所有 IMU 字段保持零初始化。
    return Status::Ok;
}

} // namespace

extern const ModelDesc kSZYGSJKJStereoFisheyeV1P2R0N4 = {
    "SZYGSJKJ Stereo Fisheye V1.2 Rotate-0-deg 4-Distortion-parameters",
    "- Calibration manufacturer:    SZYGSJKJ\n"
    "- Distortion model:            Fisheyes-4-parameters,\n"
    "- Calibration version:         V1.2\n"
    "- Device Type:                 Stereo Camera\n"
    "- Rotating installation angle: 0 degrees\n",
    probe,
    read,
};

DSC_EEPROM_REGISTER_MODEL(kSZYGSJKJStereoFisheyeV1P2R0N4);

} // namespace eeprom
} // namespace dsc
