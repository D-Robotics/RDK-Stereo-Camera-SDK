/**
 * @file types.hpp
 * @brief Internal C++ shared types: status codes, configuration, calibration, and
 *        FIFO policy.
 * @brief 内部 C++ 共享类型：状态码、配置、标定以及 FIFO 策略。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSC_TYPES_HPP
#define DSC_TYPES_HPP

#include <cstdint>

namespace dsc {

// Status maps 1:1 to dsc_err_t; test for Status::Ok to detect success.
// Status 与 dsc_err_t 一一对应；用 Status::Ok 判断是否成功。
enum class Status {
    Ok,
    ParamError,
    Unsupported,    // No fallback, rounding, or substitution. 不降级、不取整、不替换。
    NotFound,
    HwError,        // Transport or driver failure. 通信或驱动故障。
    Timeout,
    ThreadClosed,   // Thread stopped or never started. 线程已停止或从未启动。
};

// Per-pipeline array indices follow the logical eye, not the I2C probe order.
// 流水线数组下标按左右目区分，与 I2C 探测顺序无关。
enum class CamIndex {
    Right = 0,
    Left  = 1,
    Num   = 2,
};

enum class OutputMode {
    Raw,      // NV12 at sensor dimensions. 传感器原生尺寸的 NV12。
    Resize,
    Rect,
};

// The caller resolves the configuration for the target board.
// 调用方按目标板确定配置。
struct PipelineConfig {
    // Key by detected I2C bus, not CamIndex: buses can be swapped.
    // bus_mipi_rx: 0xFF = unconfigured; bus_reset_gpio: -1 = do not control.
    // 总线可能互换；以实际探测到的 I2C 总线号为索引，不用 CamIndex。
    // bus_mipi_rx：0xFF 表示未配置；bus_reset_gpio：-1 表示不控制。
    uint8_t bus_mipi_rx[32];
    int     bus_reset_gpio[32];

    // VIN/ISP input size in pixels. VIN/ISP 输入尺寸，单位为像素。
    uint32_t sensor_width, sensor_height;

    // Frames per second; must match sensor timing. 单位为帧/秒，须与传感器时序一致。
    uint32_t fps;
    OutputMode mode;

    // Pixels; required for Resize/Rect, equal to sensor size in Raw.
    // 单位为像素；Resize/Rect 模式必填，Raw 模式下等于传感器尺寸。
    uint32_t output_width;
    uint32_t output_height;

    // Sensor timings passed unchanged to the driver. 传感器时序值原样传给驱动。
    uint32_t line_length;
    uint32_t frame_length;

    // ISP tuning path borrowed during Pipeline::init(); nullptr skips loading.
    // ISP 调优路径仅在 Pipeline::init() 期间借用；nullptr 表示不加载。
    const char *tuning_file;
};

/**
 * Decoded 16-byte ICM FIFO packet. Wire layout: header at byte 0, big-endian
 * accel/gyro words at bytes 1–12, temperature at 13, timestamp/delta at 14–15.
 * Samples are in host byte order and unscaled; use the configured range and
 * model-specific sensitivity. Accel and gyro use the sensor package axes.
 * 解码后的 ICM 16 字节 FIFO 包。传输布局：第 0 字节为包头，1–12 为大端加速度和
 * 角速度字，13 为温度，14–15 为时间戳/增量。采样值已转为主机字节序，未做换算；
 * 换算需使用配置的量程和对应型号的灵敏度。加速度和角速度沿用传感器封装的轴向。
 */
struct ImuHwFifo16Packet {
    int16_t  accel[3];
    int16_t  gyro[3];
    int8_t   temp;
    // is_fsync marks an FSYNC-stamped packet. Only then is delta_time_us
    // valid: the offset from the FSYNC edge to the sample, in microseconds.
    // is_fsync 表示该包带有 FSYNC 标记；仅此时 delta_time_us 有效，
    // 表示从 FSYNC 边沿到采样的时间差，单位为微秒。
    bool     is_fsync;
    uint32_t delta_time_us;
};

// Supported values are model-specific (ModelDesc::info); others return Status::Unsupported.
// 支持的取值因型号而异（见 ModelDesc::info）；其余取值返回 Status::Unsupported。
struct ImuConfig {
    uint32_t odr_hz;
    uint16_t accel_fsr_g;   // 1 g = 9.80665 m/s^2（标准重力加速度）
    uint16_t gyro_fsr_dps;  // Degrees per second. 度/秒。
    uint8_t  bw_sel;   // Shared accel/gyro UI filter selector. 加速度计和陀螺仪共用的 UI 滤波器选择值。
};

// OpenCV dist_coeffs order; unused entries are zero.
// dist_coeffs 按 OpenCV 顺序排列，未用的项为 0。
enum class DistModel {
    Pinhole,        // [k1,k2,p1,p2,k3,k4,k5,k6] 针孔模型
    Fisheye,        // [k1,k2,k3,k4,0,0,0,0] 鱼眼模型
};

/**
 * fx, fy, cx, cy are in pixels; the origin is the upper-left, x right, y down.
 * base::stereo_rectify() updates the intrinsics in place.
 * fx、fy、cx、cy 的单位为像素；原点在左上角，x 向右，y 向下。
 * base::stereo_rectify() 就地更新内参。
 */
struct CameraIntrinsics {
    double fx, fy, cx, cy;
    // K stores the same intrinsics in row-major order, with last row (0,0,1).
    // K 按行主序存储同一份内参，最后一行为 (0,0,1)。
    double K[9], dist_coeffs[8];
    // Both cameras must use the same distortion model. 左右相机的畸变模型必须一致。
    DistModel dist_model = DistModel::Fisheye;
};

// EEPROM calibration: vectors follow sensor axis order; 3x3 matrices are row-major.
// EEPROM 标定数据：向量遵循传感器轴序，3x3 矩阵按行主序存储。
struct ImuIntrinsics {
    double accel_misalign[9];    // Cross-axis coupling. 轴间耦合。
    double accel_scale[3];
    double accel_bias[3];
    double accel_noise;          // m/s^2/sqrt(Hz) 加速度噪声密度
    double accel_random_walk;    // m/s^3/sqrt(Hz) 加速度随机游走
    double gyro_misalign[9];
    double gyro_scale[3];
    double gyro_bias[3];
    double gyro_noise;           // rad/s/sqrt(Hz) 角速度噪声密度
    double gyro_random_walk;     // rad/s^2/sqrt(Hz) 角速度随机游走
};

/**
 * Each R/T maps its sensor frame into a common reference frame chosen by the
 * EEPROM driver: p_ref = R * p_sensor + T. R is row-major.
 * dsc_convert_calibration() changes the reference without re-rectifying.
 * 每组 R/T 将各自的传感器坐标系变换到 EEPROM 驱动选定的公共参考系：
 * p_ref = R * p_sensor + T。R 按行主序存储。
 * dsc_convert_calibration() 可切换参考系，不重新做立体校正。
 */
struct StereoImuModel {
    ImuIntrinsics    imu;
    CameraIntrinsics cam_right;
    CameraIntrinsics cam_left;

    double cam_right_R[9];
    double cam_right_T[3];
    double cam_left_R[9];
    double cam_left_T[3];
    double imu_R[9];
    double imu_T[3];
    // Mounting rotation in degrees, a multiple of 90; 0 means no rotation.
    // 安装旋转角，单位为度，取 90 的整数倍；0 表示不旋转。
    int    install_angle;
};

/**
 * For each output pixel, (x, y) is the source sampling position in pixels;
 * fractional positions are allowed. Keep two consecutive doubles in x, y order:
 * the RDK X5 backend reinterprets this layout as Horizon point_t.
 * 对每个输出像素，(x, y) 是源图像的采样位置，单位为像素，可取小数。
 * ABI 须保持按 x、y 排列的两个连续 double；RDK X5 后端将此布局当作 Horizon point_t 使用。
 */
struct RemapPoint {
    double x, y;
};

// Policy when base::Fifo is full. base::Fifo 队列满时的策略。
enum class FifoMode {
    DropNew,
    DropOld,   // Overwrite the oldest entry. 覆盖最旧的数据。
};

} // namespace dsc

#endif // DSC_TYPES_HPP
