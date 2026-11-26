/**
 * @file union_stereo_fisheye_v1p3r0n4.cpp
 * @brief UNION stereo (no IMU), fisheye, v1.3, rotation 0 deg, 4 distortion coefficients.
 *
 * The camera calibration uses the same byte layout as UNION V1.2: double-precision
 * intrinsics, distortion coefficients and stereo extrinsics. The identification header
 * selects V1.3 without an IMU, so no IMU area is required or interpreted.
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

constexpr uint16_t kHeaderSize = 15;
constexpr uint16_t kChecksumOff = kHeaderSize;
constexpr uint8_t  kChecksumLen = 14;

constexpr uint8_t kHeader[kHeaderSize] = {
    0x55, 0x4e, 0x49, 0x4f, 0x4e, 0x00, 0x00, 0x00,  // "UNION"
    0x01, 0x01,   // stereo, no IMU
    0x01, 0x03,   // v1.3
    0x00,         // rotation 0 deg
    0x04,         // 4 distortion coefficients
    0x00,
};

// Camera calibration offsets, shared with UNION V1.2 (all fields are double).
constexpr uint16_t kLFx     = 0x0018;
constexpr uint16_t kRFx     = 0x0081;
constexpr uint16_t kStereoR = 0x00EA;
constexpr uint16_t kStereoT = 0x0132;
constexpr uint16_t kSize    = 0x014A;   // camera calibration only

double read_double(const uint8_t *buf, uint16_t off)
{
    double v;
    memcpy(&v, &buf[off], sizeof(v));
    return v;
}

void read_intrinsics(const uint8_t *buf, CameraIntrinsics *cam, uint16_t off)
{
    cam->fx = read_double(buf, off + 0x00);
    cam->fy = read_double(buf, off + 0x08);
    cam->cx = read_double(buf, off + 0x10);
    cam->cy = read_double(buf, off + 0x18);

    cam->K[0] = cam->fx;
    cam->K[2] = cam->cx;
    cam->K[4] = cam->fy;
    cam->K[5] = cam->cy;
    cam->K[8] = 1.0f;

    cam->dist_model = DistModel::Fisheye;
    cam->dist_coeffs[0] = read_double(buf, off + 0x20);
    cam->dist_coeffs[1] = read_double(buf, off + 0x28);
    cam->dist_coeffs[2] = read_double(buf, off + 0x30);
    cam->dist_coeffs[3] = read_double(buf, off + 0x38);
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

    read_intrinsics(buf, &out->cam_left,  kLFx);
    read_intrinsics(buf, &out->cam_right, kRFx);

    out->install_angle = 0;

    // The right camera is the common reference; the stored left-camera pose is direct.
    for(uint8_t i = 0; i < 9; i++){
        out->cam_right_R[i] = (i % 4 == 0) ? 1.0 : 0.0;
        out->cam_left_R[i]  = read_double(buf, kStereoR + i * 8);
    }
    for(uint8_t i = 0; i < 3; i++){
        out->cam_right_T[i] = 0.0;
        out->cam_left_T[i]  = read_double(buf, kStereoT + i * 8);
    }

    // No IMU is present; all IMU fields stay zero-initialised.
    return Status::Ok;
}

} // namespace

extern const ModelDesc kUnionStereoFisheyeV1p3R0N4 = {
    "UNION Stereo Fisheye V1.3 Rotate-0-deg 4-Distortion-parameters",
    "- Calibration manufacturer:    UNION IMAGE\n"
    "- Distortion model:            Fisheyes-4-parameters,\n"
    "- Calibration version:         V1.3\n"
    "- Device Type:                 Stereo Camera\n"
    "- Rotating installation angle: 0 degrees\n",
    probe,
    read,
};

DSC_EEPROM_REGISTER_MODEL(kUnionStereoFisheyeV1p3R0N4);

} // namespace eeprom
} // namespace dsc
