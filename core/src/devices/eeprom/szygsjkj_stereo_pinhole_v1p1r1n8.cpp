/**
 * @file szygsjkj_stereo_pinhole_v1p1r1n8.cpp
 * @brief SZYGSJKJ stereo (no IMU), pinhole, v1.1, rotation index 1 (=90 deg), 8 distortion coefficients.
 * @brief SZYGSJKJ 双目（无 IMU），针孔，v1.1，旋转索引 1（=90 度），8 个畸变系数。
 *
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
    0x01, 0x00,   // stereo, no IMU 双目，无 IMU
    0x01, 0x01,   // v1.1
    0x01,         // rotation index 1 (90 deg) 旋转索引 1（90 度）
    0x08,         // 8 distortion coefficients 8 个畸变系数
    0x00,
};

constexpr uint16_t kRotationOff = 0x000C;
constexpr int kQuarterTurnDeg = 90;

// ============================== Layout offsets 布局偏移 ==============================
constexpr uint16_t kLBlock   = 0x0010;
constexpr uint16_t kRBlock   = 0x0048;
constexpr uint16_t kStereoR  = 0x008C;
constexpr uint16_t kStereoT  = 0x00B0;
constexpr uint16_t kSize     = 0x00C0;

constexpr uint16_t kDistOff    = 0x0018;
constexpr uint8_t  kDistCount  = 8;

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

    cam->dist_model = DistModel::Pinhole;
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

    read_intrinsics(buf, &out->cam_left,  kLBlock);
    read_intrinsics(buf, &out->cam_right, kRBlock);

    out->install_angle = static_cast<int>(buf[kRotationOff]) * kQuarterTurnDeg;

    for(uint8_t i = 0; i < 9; i++){
        out->cam_right_R[i] = (i % 4 == 0) ? 1.0f : 0.0f;
        out->cam_left_R[i]  = read_float(buf, kStereoR + i * 4);
    }
    for(uint8_t i = 0; i < 3; i++){
        out->cam_right_T[i] = 0.0f;
        out->cam_left_T[i]  = read_float(buf, kStereoT + i * 4);
    }

    return Status::Ok;
}

} // namespace

extern const ModelDesc kSZYGSJKJStereoPinholeV1P1R1N8 = {
    "SZYGSJKJ Stereo Pinhole V1.1 Rotate-90-deg 8-Distortion-parameters",
    "- Calibration manufacturer:    SZYGSJKJ\n"
    "- Distortion model:            Pinhole-8-parameters [k1,k2,p1,p2,k3,k4,k5,k6],\n"
    "- Calibration version:         V1.1\n"
    "- Device Type:                 Stereo Camera\n"
    "- Rotating installation angle: 90 degrees\n",
    probe,
    read,
};

DSC_EEPROM_REGISTER_MODEL(kSZYGSJKJStereoPinholeV1P1R1N8);

} // namespace eeprom
} // namespace dsc
