/**
 * @file szygsjkj_stereo_imu_fisheye_v1p2r0n4.cpp
 * @brief SZYGSJKJ stereo + IMU, fisheye, v1.2, rotation 0 deg, 4 distortion coefficients.
 * @brief SZYGSJKJ 双目 + IMU，鱼眼，v1.2，旋转 0 度，4 个畸变系数。
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
    0x11, 0x01,   // stereo + IMU 双目 + IMU
    0x01, 0x02,   // v1.2
    0x00,         // rotation 0 deg 旋转 0 度
    0x04,         // 4 distortion coefficients 4 个畸变系数
    0x00,
};

constexpr uint8_t  kCamTypeStereoImu = 0x11;
constexpr uint16_t kCamTypeOff       = 0x0008;   // module type (char) 模组类型（char）

// ============================== Layout offsets 布局偏移 ==============================
constexpr uint16_t kLBlock        = 0x0010;
constexpr uint16_t kRBlock        = 0x0048;
constexpr uint16_t kStereoR       = 0x008C;
constexpr uint16_t kStereoT       = 0x00B0;
constexpr uint16_t kAccelMisalign = 0x0200;
constexpr uint16_t kAccelNoise    = 0x0224;
constexpr uint16_t kAccelWalk     = 0x0228;
constexpr uint16_t kGyroMisalign  = 0x022C;
constexpr uint16_t kGyroNoise     = 0x0250;
constexpr uint16_t kGyroWalk      = 0x0254;
constexpr uint16_t kImuR          = 0x025C;
constexpr uint16_t kImuT          = 0x0280;
constexpr uint16_t kSize          = 0x02D8;


constexpr uint16_t kImuCheckOff = 0x02D8;
constexpr uint16_t kImuSumLen   = kImuCheckOff - kAccelMisalign;

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
    cam->dist_coeffs[0] = read_float(buf, off + 0x18);
    cam->dist_coeffs[1] = read_float(buf, off + 0x1C);
    cam->dist_coeffs[2] = read_float(buf, off + 0x20);
    cam->dist_coeffs[3] = read_float(buf, off + 0x24);
}


void compose_imu_to_ref(StereoImuModel *c, const double *imu_to_left_R,
                        const double *imu_to_left_T)
{
    for(uint8_t r = 0; r < 3; r++){
        for(uint8_t col = 0; col < 3; col++){
            c->imu_R[r * 3 + col] = 0.0;
            for(uint8_t k = 0; k < 3; k++)
                c->imu_R[r * 3 + col] += c->cam_left_R[r * 3 + k] * imu_to_left_R[k * 3 + col];
        }
        c->imu_T[r] = c->cam_left_T[r];
        for(uint8_t k = 0; k < 3; k++)
            c->imu_T[r] += c->cam_left_R[r * 3 + k] * imu_to_left_T[k];
    }
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

    if(buf[kCamTypeOff] != kCamTypeStereoImu)
        return Status::HwError;

    uint32_t imu_sum = 0;
    for(uint16_t i = 0; i < kImuSumLen; i++)
        imu_sum += buf[kAccelMisalign + i];
    uint8_t imu_checksum = 0;
    if(bus.readBurst16(kImuCheckOff, &imu_checksum, 1) != Status::Ok)
        return Status::HwError;
    if(static_cast<uint8_t>((imu_sum % 255) + 1) != imu_checksum)
        return Status::HwError;

    *out = StereoImuModel{};

    // The EEPROM L/R blocks map directly to the module's left/right cameras
    // EEPROM 的 L/R 块直接对应模组的左目/右目相机
    read_intrinsics(buf, &out->cam_left,  kLBlock);
    read_intrinsics(buf, &out->cam_right, kRBlock);

    out->install_angle = static_cast<int8_t>(buf[0x0C]);

    // This layout uses the right camera as the common reference frame, so cam_right is the identity transform
    // 本布局以右目相机作为公共参考系，因此 cam_right 是单位变换
    for(uint8_t i = 0; i < 9; i++){
        out->cam_right_R[i] = (i % 4 == 0) ? 1.0f : 0.0f;
        out->cam_left_R[i]  = read_float(buf, kStereoR + i * 4);
    }
    for(uint8_t i = 0; i < 3; i++){
        out->cam_right_T[i] = 0.0f;
        out->cam_left_T[i]  = read_float(buf, kStereoT + i * 4);
    }

    // Each sensor carries scale and misalignment combined in one matrix, with no separate
    // scale or bias field; the matrix is kept whole and no missing term is invented.
    // 每个传感器把刻度与失准合并存放在一个矩阵里，没有单独的刻度字段或零偏字段；
    // 该矩阵整体保留，不虚构任何缺失项。
    for(uint8_t i = 0; i < 9; i++){
        out->imu.accel_misalign[i] = read_float(buf, kAccelMisalign + i * 4);
        out->imu.gyro_misalign[i]  = read_float(buf, kGyroMisalign + i * 4);
    }
    out->imu.accel_noise       = read_float(buf, kAccelNoise);
    out->imu.accel_random_walk = read_float(buf, kAccelWalk);
    out->imu.gyro_noise        = read_float(buf, kGyroNoise);
    out->imu.gyro_random_walk  = read_float(buf, kGyroWalk);

    double imu_to_left_R[9];
    double imu_to_left_T[3];
    for(uint8_t i = 0; i < 9; i++)
        imu_to_left_R[i] = read_float(buf, kImuR + i * 4);
    for(uint8_t i = 0; i < 3; i++)
        imu_to_left_T[i] = read_float(buf, kImuT + i * 4);
    compose_imu_to_ref(out, imu_to_left_R, imu_to_left_T);

    return Status::Ok;
}

} // namespace

extern const ModelDesc kSZYGSJKJStereoImuFisheyeV1P2R0N4 = {
    "SZYGSJKJ Stereo-IMU Fisheye V1.2 Rotate-0-deg 4-Distortion-parameters",
    "- Calibration manufacturer:    SZYGSJKJ\n"
    "- Distortion model:            Fisheyes-4-parameters,\n"
    "- Calibration version:         V1.2\n"
    "- Device Type:                 Stereo Camera and IMU\n"
    "- Rotating installation angle: 0 degrees\n",
    probe,
    read,
};

DSC_EEPROM_REGISTER_MODEL(kSZYGSJKJStereoImuFisheyeV1P2R0N4);

} // namespace eeprom
} // namespace dsc
