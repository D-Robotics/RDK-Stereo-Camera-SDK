/**
 * @file dstereo_cam.h
 * @brief RDK Stereo Camera SDK C API.
 * @brief RDK 双目相机 SDK C API。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSTEREO_CAM_H
#define DSTEREO_CAM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum dsc_err_e{
    DSC_OK = 0,          /**< Operation completed successfully. 操作成功完成。 */
    DSC_PARAM_ERROR,     /**< Invalid argument, configuration, or call order. 参数、配置或调用顺序无效。 */
    DSC_UNSUPPORTED,     /**< The selected hardware or mode is not supported. 所选的硬件或模式不受支持。 */
    DSC_NOT_FOUND,       /**< A required device or calibration record was not detected. 未检测到必需的设备或标定记录。 */
    DSC_HW_ERROR,        /**< Low-level communication or driver failure. 底层通信或驱动故障。 */
    DSC_TIMEOUT,         /**< A non-blocking data query found no item available. 非阻塞数据查询未发现可用数据项。 */
    DSC_THREAD_CLOSED,   /**< Capture threads are stopped or have not been started. 采集线程已停止或尚未启动。 */
}dsc_err_t;

/**
 * Policy applied when the data buffer is full.
 * 数据缓冲区写满后的策略。
 */
typedef enum dsc_fifo_mode_e{
    DSC_FIFO_DROP_NEW,   /**< Reject the new item without waiting for space. 拒绝新数据，不等待队列腾出空间。 */
    DSC_FIFO_DROP_OLD,   /**< Discard the oldest item and enqueue the new item. 丢弃最旧的数据项并让新数据入队。 */
}dsc_fifo_mode_t;

/**
 * Data buffer configuration.
 * 数据缓冲区配置。
 */
typedef struct dsc_fifo_config_s{
    size_t depth;
    dsc_fifo_mode_t mode;
}dsc_fifo_config_t;

/* ==================== Device Config 设备配置 ==================== */

/**
 * Camera output mode. RAW outputs the original NV12 image; RESIZE scales on top
 * of the RAW image; RECT reads the data in the camera EEPROM and performs
 * stereo rectification.
 * 相机出图模式，RAW 表示输出原始 NV12 图像，RESIZE 支持在 RAW 图的基础上进行缩放，RECT 将读取相机的 EEPROM 中的数据并进行双目立体矫正。
 */
typedef enum dsc_camera_mode_e{
    DSC_CAMERA_MODE_RAW,     /**< CAM -> VIN -> ISP -> OUT */
    DSC_CAMERA_MODE_RESIZE,  /**< CAM -> VIN -> ISP -> VSE -> OUT */
    DSC_CAMERA_MODE_RECT,    /**< CAM -> VIN -> ISP -> GDC -> VSE -> OUT */
}dsc_camera_mode_t;

/**
 * Left and right camera indices.
 * 左右目索引。
 */
typedef enum dsc_camera_index_e{
    DSC_CAMERA_RIGHT_IDX = 0,
    DSC_CAMERA_LEFT_IDX  = 1,
}dsc_camera_index_t;

/**
 * Stereo stitched output mode.
 * 双目拼接出图模式。
 */
typedef enum dsc_stereo_layout_e{
    DSC_STEREO_LAYOUT_NONE,          /**< No stitching; return separate frames. 不拼接；返回各自独立的帧。 */
    DSC_STEREO_LAYOUT_LEFT_RIGHT,    /**< Left image on the left, right image on the right. 左图在左，右图在右。 */
    DSC_STEREO_LAYOUT_RIGHT_LEFT,    /**< Left image on the right, right image on the left. 左图在右，右图在左。 */
    DSC_STEREO_LAYOUT_TOP_BOTTOM,    /**< Left image on top, right image at the bottom. 左图在上，右图在下。 */
    DSC_STEREO_LAYOUT_BOTTOM_TOP,    /**< Left image at the bottom, right image on top. 左图在下，右图在上。 */
}dsc_stereo_layout_t;

/**
 * Stereo camera configuration.
 * 双目相机配置。
 */
typedef struct dsc_camera_config_s{
    uint8_t bus[32]; /**< I2C bus the camera is on; only the first bus_num entries are valid. 相机所在的 I2C 总线号，前 bus_num 个值有效。 */
    size_t bus_num; /**< Number of valid entries in bus; maximum 32. bus 中有效条目的数量；最大为 32。 */
    uint8_t  left_addr; /**< Left camera I2C address. 左目摄像头的 I2C 地址。 */
    uint8_t  right_addr; /**< Right camera I2C address. 右目摄像头的 I2C 地址。 */

    uint32_t sensor_width, sensor_height; /**< Native sensor dimensions in pixels. 传感器的原生尺寸，单位为像素。 */
    uint32_t fps; /**< Output frame rate. 出图帧率。 */
    uint32_t line_length, frame_length; /**< Sensor timing values. 传感器时序值。 */
    const char *tuning_file; /**< ISP file path; NULL means do not load. ISP 文件路径；为 NULL 表示不加载。 */

    uint32_t output_width, output_height; /**< Output dimensions; see the README for the sizes each mode supports. 出图尺寸，各模式支持尺寸见 README。 */
    dsc_camera_mode_t mode; /**< Output mode. 出图模式。 */

    /**
     * NONE returns separate frames; Other produces one stitched image.
     * NONE 返回各自独立的帧；Other 拼接出图。
     */
    dsc_stereo_layout_t stereo_layout;

    uint8_t bus_mipi_rx[32]; /**< MIPI RX number for the I2C bus of the same MIPI port; 0xFF means unconfigured. 同一 MIPI 端口 I2C 总线对应的 MIPI RX 号；0xFF 表示未配置。 */
    int bus_reset_gpio[32]; /**< Reset-GPIO map for the I2C bus of the same MIPI port; -1 disables GPIO control. 同一 MIPI 端口 I2C 总线到复位 GPIO 的映射；-1 表示禁用 GPIO 控制。 */

    dsc_camera_index_t fsync_camera; /**< IMU FSYNC signal is attached to the left or right LPWM. IMU FSYNC 信号挂载于左目或右目 LPWM。 */
}dsc_camera_config_t;

/**
 * IMU configuration.
 * IMU 配置。
 */
typedef struct dsc_imu_config_s{
    uint8_t bus[32]; /**< I2C bus the IMU is on; only the first bus_num entries are valid. IMU 所在的 I2C 总线号，前 bus_num 个值有效。 */
    size_t bus_num; /**< Number of valid entries in bus; 0 disables IMU probing. bus 中有效条目的数量；0 表示禁用 IMU 探测。 */
    uint8_t addr; /**< IMU I2C address. IMU 的 I2C 地址。 */
    uint32_t odr_hz; /**< Output data rate in Hz. 输出数据速率，单位为 Hz。 */
    uint16_t accel_fsr_g; /**< Accelerometer full-scale range in g. 加速度计量程，单位为 g。 */
    uint16_t gyro_fsr_dps; /**< Gyroscope full-scale range in degrees per second. 陀螺仪量程，单位为度每秒。 */
    uint8_t bw_sel; /**< Shared accel/gyro UI-filter bandwidth selector; see dsc_get_imu_info(). 加速度计/陀螺仪共用的 UI 滤波器带宽选择；详见 dsc_get_imu_info()。 */
}dsc_imu_config_t;

/**
 * EEPROM configuration.
 * EEPROM 配置。
 */
typedef struct dsc_eeprom_config_s{
    uint8_t bus[32]; /**< I2C bus of the EEPROM that stores the calibration parameters; only the first bus_num entries are valid. 存储标定参数的 EEPROM 所在的 I2C 总线号，前 bus_num 个值有效。 */
    size_t bus_num; /**< Number of valid entries in bus; 0 disables EEPROM probing. bus 中有效条目的数量；0 表示禁用 EEPROM 探测。 */
    uint8_t addr; /**< EEPROM I2C address. EEPROM 的 I2C 地址。 */
}dsc_eeprom_config_t;

/**
 * Overall configuration for the stereo camera and the stereo IMU device.
 * 双目、双目 IMU 设备总配置。
 *
 * After instantiation, the whole structure must be zero-initialized first. The
 * SDK provides no defaults; alternatively, modify one of the presets in dsc_define.h.
 * 实例化后必须先将整个结构体清零初始化。SDK 不会提供默认值；
 * 也可以基于 dsc_define.h 中的某个预设做改动。
 */
typedef struct dsc_config_s{
    dsc_camera_config_t camera_config; /**< Camera settings. 相机设置。 */
    dsc_imu_config_t imu_config; /**< IMU settings. IMU 设置。 */
    dsc_eeprom_config_t eeprom_config; /**< EEPROM settings. EEPROM 设置。 */
    dsc_fifo_config_t camera_fifo; /**< Camera data buffer. 相机数据缓冲区。 */
    dsc_fifo_config_t imu_fifo; /**< IMU data buffer. IMU 数据缓冲区。 */
}dsc_config_t;

/**
 * Opaque device handle.
 * 不透明设备句柄。
 */
typedef struct dsc_device_s dsc_device_t;

/**
 * @brief SDK version string.
 * @brief SDK 版本字符串。
 *
 * @return Version string such as 1.0.0; statically allocated, do not free.
 * @return 形如 1.0.0 的版本字符串；静态分配，请勿释放。
 */
const char *dsc_version(void);

/**
 * @brief Compiled-in platform string.
 * @brief 编译时内置的平台字符串。
 *
 * @return Platform name (e.g. "RDKX5"), taken from the platform directory name at
 *         build time; statically allocated, do not free.
 * @return 平台名称（例如 "RDKX5"），取自构建时的平台目录名；静态分配，请勿释放。
 */
const char *dsc_platform(void);

/**
 * @brief Create a device handle (an empty shell; call dsc_init to initialize it).
 * @brief 创建设备句柄（一个空壳；调用 dsc_init 对其进行初始化）。
 *
 * The normal lifecycle is create -> init -> start -> stop -> deinit -> destroy.
 * Capture may be restarted after stop, and a deinitialized handle may be
 * initialized again.
 * 正常的生命周期为 create -> init -> start -> stop -> deinit -> destroy。
 * stop 之后可以重新启动采集，已反初始化的句柄可以再次初始化。
 *
 * @return A new device handle. Release it with dsc_destroy().
 * @return 新的设备句柄。请用 dsc_destroy() 释放。
 */
dsc_device_t *dsc_create();

/**
 * @brief Release a device handle.
 * @brief 释放设备句柄。
 *
 * dsc_deinit() (or at least dsc_stop()) must be called first; destroying the
 * handle while background threads are still running is undefined behaviour.
 * 必须先调用 dsc_deinit()（或至少 dsc_stop()）；在后台线程仍在运行时销毁句柄属于未定义行为。
 *
 * @param[in] dev Device handle returned by dsc_create(); NULL is accepted.
 * @param[in] dev 由 dsc_create() 返回的设备句柄；允许为 NULL。
 */
void dsc_destroy(
    dsc_device_t *dev);

/**
 * @brief Initialize the device: probe EEPROM/IMU along the candidate buses in order,
 *        load the calibration, configure the camera and the IMU (streaming stays off).
 * @brief 初始化设备：按顺序沿候选总线探测 EEPROM/IMU，加载标定数据，
 *        配置相机和 IMU（此时不启动数据流）。
 *
 * EEPROM and IMU probing are optional. RAW mode can operate without EEPROM
 * calibration; RESIZE and RECT require it because initialization updates the
 * output calibration. A handle can be initialized only once until deinitialized.
 * EEPROM 和 IMU 探测是可选的。RAW 模式可以在没有 EEPROM 标定数据的情况下工作；
 * RESIZE 和 RECT 模式需要标定数据，因为初始化会更新输出标定。句柄在反初始化之前只能初始化一次。
 *
 * @param[in] dev Device handle returned by dsc_create().
 * @param[in] dev 由 dsc_create() 返回的设备句柄。
 * @param[in] cfg Fully initialized device configuration.
 * @param[in] cfg 已完全初始化的设备配置。
 * @retval DSC_OK Initialization succeeded.
 * @retval DSC_OK 初始化成功。
 * @retval DSC_PARAM_ERROR An argument, configuration field, or call order is invalid.
 * @retval DSC_PARAM_ERROR 参数、配置字段或调用顺序无效。
 * @retval DSC_UNSUPPORTED The requested hardware configuration is unsupported.
 * @retval DSC_UNSUPPORTED 请求的硬件配置不受支持。
 * @retval DSC_NOT_FOUND The stereo camera could not be detected.
 * @retval DSC_NOT_FOUND 无法检测到立体相机。
 * @retval DSC_HW_ERROR Device communication or pipeline setup failed.
 * @retval DSC_HW_ERROR 设备通信或流水线设置失败。
 */
dsc_err_t dsc_init(
    dsc_device_t *dev,
    const dsc_config_t *cfg);

/**
 * @brief Deinitialize: release all device resources (calls dsc_stop() first if still running).
 * @brief 反初始化：释放所有设备资源（若仍在运行，会先调用 dsc_stop()）。
 *
 * The handle itself stays valid afterwards and must be released by dsc_destroy().
 * 此后句柄本身仍然有效，必须通过 dsc_destroy() 释放。
 *
 * @param[in] dev Device handle returned by dsc_create().
 * @param[in] dev 由 dsc_create() 返回的设备句柄。
 * @retval DSC_OK Resources were released, or the handle was already idle.
 * @retval DSC_OK 资源已释放，或句柄本来就处于空闲状态。
 * @retval DSC_PARAM_ERROR dev is NULL.
 * @retval DSC_PARAM_ERROR dev 为 NULL。
 */
dsc_err_t dsc_deinit(
    dsc_device_t *dev);

/**
 * @brief Start capture.
 * @brief 启动采集。
 *
 * When an IMU was detected, the camera waits for the IMU FSYNC handshake to complete
 * before streaming; without an IMU it streams immediately.  A repeated call returns
 * DSC_PARAM_ERROR.
 * 当检测到 IMU 时，相机会先等待 IMU FSYNC 握手完成后才开始输出数据流；没有 IMU 时则立即输出。
 * 重复调用会返回 DSC_PARAM_ERROR。
 *
 * @param[in] dev Initialized device handle.
 * @param[in] dev 已初始化的设备句柄。
 * @retval DSC_OK Capture threads were started.
 * @retval DSC_OK 采集线程已启动。
 * @retval DSC_PARAM_ERROR The handle is NULL, uninitialized, or already running.
 * @retval DSC_PARAM_ERROR 句柄为 NULL、未初始化或已在运行。
 * @retval DSC_HW_ERROR The IMU or camera pipeline failed to start.
 * @retval DSC_HW_ERROR IMU 或相机流水线启动失败。
 */
dsc_err_t dsc_start(
    dsc_device_t *dev);

/**
 * @brief Stop capture and wait for the background threads to exit.
 * @brief 停止采集并等待后台线程退出。
 *
 * A no-op when not running; dsc_start() may be called again afterwards.
 * 未运行时为空操作；之后可以再次调用 dsc_start()。
 *
 * @param[in] dev Device handle; NULL is accepted and ignored.
 * @param[in] dev 设备句柄；允许为 NULL 并被忽略。
 */
void dsc_stop(
    dsc_device_t *dev);

/* ==================== Camera Data 相机数据 ==================== */

/**
 * NV12 image structure.
 * NV12 图像结构。
 */
typedef struct dsc_image_nv12_s{
    uint8_t *data; /**< SDK-allocated buffer: Y plane followed by interleaved UV; caller must free(). SDK 分配的缓冲区：Y 平面后跟交错的 UV 平面；调用方必须调用 free()。 */
    uint32_t width, height; /**< Image dimensions in pixels. 图像尺寸，单位为像素。 */
    uint64_t timestamp_ns; /**< Capture timestamp in nanoseconds, aligned to the camera clock. 采集时间戳，单位为纳秒，对齐到相机时钟。 */
}dsc_image_nv12_t;

/**
 * @brief Query the number of image buffers.
 * @brief 查询图片缓冲区数量。
 *
 * The count is a snapshot and may change before the next call. The frame getters
 * are non-blocking and return DSC_TIMEOUT if another consumer empties the FIFO.
 * 该计数是一个快照，在下一次调用之前可能发生变化。取帧接口都是非阻塞的，
 * 如果另一个消费者清空了 FIFO，则返回 DSC_TIMEOUT。
 *
 * @param[in] dev Running device handle.
 * @param[in] dev 正在运行的设备句柄。
 * @return Number of frame pairs currently queued, or 0 for an invalid, idle, or
 *         faulted device.
 * @return 当前排队的帧对数量；对于无效、空闲或发生故障的设备，返回 0。
 */
size_t dsc_available_camera(
    dsc_device_t *dev);

/**
 * @brief Fetch the left and right NV12 data separately.
 * @brief 分别获取左右两路 NV12 数据。
 *
 * This function is non-blocking. On success, each returned data pointer is owned
 * by the caller and must be released with free().
 * 此函数是非阻塞的。成功时，返回的每个数据指针都归调用方所有，必须用 free() 释放。
 *
 * @param[in]  dev Running device handle.
 * @param[in]  dev 正在运行的设备句柄。
 * @param[out] image_left Left-camera frame.
 * @param[out] image_left 左相机帧。
 * @param[out] image_right Right-camera frame.
 * @param[out] image_right 右相机帧。
 * @retval DSC_OK A synchronized frame pair was returned.
 * @retval DSC_OK 已返回一对同步帧。
 * @retval DSC_PARAM_ERROR An argument is NULL or the camera FIFO is unavailable.
 * @retval DSC_PARAM_ERROR 某个参数为 NULL，或相机 FIFO 不可用。
 * @retval DSC_UNSUPPORTED A stitched output layout is configured.
 * @retval DSC_UNSUPPORTED 配置了拼接输出布局。
 * @retval DSC_TIMEOUT The FIFO is currently empty.
 * @retval DSC_TIMEOUT FIFO 当前为空。
 * @retval DSC_THREAD_CLOSED Capture is not running.
 * @retval DSC_THREAD_CLOSED 采集未在运行。
 * @retval DSC_HW_ERROR A capture thread terminated after a hardware failure.
 * @retval DSC_HW_ERROR 某个采集线程因硬件故障而终止。
 */
dsc_err_t dsc_get_nv12_frame(
    dsc_device_t *dev,
    dsc_image_nv12_t *image_left,
    dsc_image_nv12_t *image_right);

/**
 * @brief Fetch one stitched NV12 image.
 * @brief 获取一帧拼接 NV12 图像。
 *
 * The layout is fixed by dsc_camera_config_t.stereo_layout during
 * dsc_init(). This function is non-blocking and transfers ownership of the
 * already assembled pixel buffer to the caller; release image->data with free().
 * 布局在 dsc_init() 期间由 dsc_camera_config_t.stereo_layout 固定。此函数是非阻塞的，
 * 并将已组装好的像素缓冲区的所有权转移给调用方；请用 free() 释放 image->data。
 *
 * @param[in]  dev Running device handle.
 * @param[in]  dev 正在运行的设备句柄。
 * @param[out] image Stitched NV12 frame. Horizontal layouts double width; vertical
 *                   layouts double height.
 * @param[out] image 拼接后的 NV12 帧。水平布局使宽度加倍；垂直布局使高度加倍。
 * @retval DSC_OK A stitched frame was returned.
 * @retval DSC_OK 已返回一帧拼接图像。
 * @retval DSC_PARAM_ERROR An argument is NULL or the camera FIFO is unavailable.
 * @retval DSC_PARAM_ERROR 某个参数为 NULL，或相机 FIFO 不可用。
 * @retval DSC_UNSUPPORTED DSC_STEREO_LAYOUT_NONE is configured.
 * @retval DSC_UNSUPPORTED 配置了 DSC_STEREO_LAYOUT_NONE。
 * @retval DSC_TIMEOUT The FIFO is currently empty.
 * @retval DSC_TIMEOUT FIFO 当前为空。
 * @retval DSC_THREAD_CLOSED Capture is not running.
 * @retval DSC_THREAD_CLOSED 采集未在运行。
 * @retval DSC_HW_ERROR A capture thread terminated after a hardware failure.
 * @retval DSC_HW_ERROR 某个采集线程因硬件故障而终止。
 */
dsc_err_t dsc_get_stereo_nv12_frame(
    dsc_device_t *dev,
    dsc_image_nv12_t *image);

/* ==================== IMU Data IMU 数据 ==================== */

/**
 * IMU data packet with a hardware timestamp.
 * 带硬件时间戳的 IMU 数据包。
 */
typedef struct dsc_imu_packet_s{
    float accel[3]; /**< Acceleration in m/s^2. 加速度，单位为 m/s^2。 */
    float gyro[3]; /**< Angular velocity in rad/s. 角速度，单位为 rad/s。 */
    float temp; /**< Temperature in degrees Celsius. 温度，单位为摄氏度。 */
    bool  is_fsync; /**< True when the sample carries an IMU FSYNC anchor. 当该采样携带 IMU FSYNC 锚点时为 true。 */
    uint64_t timestamp_ns; /**< Timestamp in ns on the camera clock, with known IMU filter delay removed. 相机时钟上的时间戳，单位为 ns，已扣除已知的 IMU 滤波延迟。 */
}dsc_imu_packet_t;

/**
 * @brief Query the number of IMU data buffers.
 * @brief 查询 IMU 数据缓冲区数量。
 *
 * The count is a snapshot and may change before the next call. Returns 0 when
 * no IMU is available, capture is idle, or the device has faulted.
 * 该计数是一个快照，在下一次调用之前可能发生变化。当没有可用的 IMU、采集空闲，
 * 或设备已发生故障时，返回 0。
 *
 * @param[in] dev Running device handle.
 * @param[in] dev 正在运行的设备句柄。
 * @return Number of packets currently queued.
 * @return 当前排队的数据包数量。
 */
size_t dsc_available_imu(
    dsc_device_t *dev);

/**
 * @brief Fetch one IMU packet.
 * @brief 获取一个 IMU 数据包。
 *
 * This function is non-blocking.
 * 此函数是非阻塞的。
 *
 * @param[in]  dev Running device handle.
 * @param[in]  dev 正在运行的设备句柄。
 * @param[out] out Destination for one IMU packet.
 * @param[out] out 一个 IMU 数据包的目标存放位置。
 * @retval DSC_OK One packet was returned.
 * @retval DSC_OK 已返回一个数据包。
 * @retval DSC_PARAM_ERROR An argument is NULL or no IMU is available.
 * @retval DSC_PARAM_ERROR 某个参数为 NULL，或没有可用的 IMU。
 * @retval DSC_TIMEOUT The FIFO is currently empty.
 * @retval DSC_TIMEOUT FIFO 当前为空。
 * @retval DSC_THREAD_CLOSED Capture is not running.
 * @retval DSC_THREAD_CLOSED 采集未在运行。
 * @retval DSC_HW_ERROR The IMU thread terminated after a hardware failure.
 * @retval DSC_HW_ERROR IMU 线程因硬件故障而终止。
 */
dsc_err_t dsc_get_imu_packet(
    dsc_device_t *dev,
    dsc_imu_packet_t *out);

/**
 * @brief Get the model name of the detected IMU.
 * @brief 获取检测到的 IMU 的型号名称。
 *
 * @param[in] dev Device handle from dsc_create().
 * @param[in] dev 来自 dsc_create() 的设备句柄。
 * @return Statically allocated IMU model string, or NULL if no IMU was detected.
 *         The caller must not modify or free the returned string.
 * @return 静态分配的 IMU 型号字符串；如果未检测到 IMU，则为 NULL。
 *         调用方不得修改或释放返回的字符串。
 */
const char *dsc_get_imu_name(
    dsc_device_t *dev);

/**
 * @brief Get detailed information about the detected IMU (setting/bandwidth tables, etc.).
 * @brief 获取检测到的 IMU 的详细信息（设置/带宽表等）。
 *
 * @param[in] dev Device handle from dsc_create().
 * @param[in] dev 来自 dsc_create() 的设备句柄。
 * @return Statically allocated information string, or NULL if no IMU was
 *         detected. The caller must not modify or free the returned string.
 * @return 静态分配的信息字符串；如果未检测到 IMU，则为 NULL。
 *         调用方不得修改或释放返回的字符串。
 */
const char *dsc_get_imu_info(
    dsc_device_t *dev);

/* ==================== EEPROM Data EEPROM 数据 ==================== */

/**
 * Camera distortion model used by dist_coeffs.
 * dist_coeffs 使用的相机畸变模型。
 */
typedef enum dsc_dist_model_e{
    DSC_DIST_PINHOLE,    /**< OpenCV/radtan order: [k1,k2,p1,p2,k3,k4,k5,k6]. OpenCV/radtan 顺序：[k1,k2,p1,p2,k3,k4,k5,k6]。 */
    DSC_DIST_FISHEYE,    /**< Equidistant order: [k1,k2,k3,k4,0,0,0,0]. 等距模型顺序：[k1,k2,k3,k4,0,0,0,0]。 */
}dsc_dist_model_t;

/**
 * Camera intrinsic parameters. Matrices are stored row-major.
 * 相机内参。矩阵按行主序存储。
 */
typedef struct dsc_camera_intrinsics_s{
    double fx, fy, cx, cy; /**< Focal lengths and principal point in pixels. 焦距和主点，单位为像素。 */
    /**
     * K is a row-major 3x3 intrinsic matrix; dist_coeffs uses the model order above.
     * K 是行主序的 3x3 内参矩阵；dist_coeffs 使用上面的模型顺序。
     */
    double K[9], dist_coeffs[8];
    dsc_dist_model_t dist_model; /**< Model used to interpret dist_coeffs. 用于解释 dist_coeffs 的模型。 */
}dsc_camera_intrinsics_t;

/**
 * IMU intrinsics.
 * IMU 内参。
 */
typedef struct dsc_imu_intrinsics_s{
    double accel_misalign[9]; /**< Cross-axis coupling matrix. 交轴耦合矩阵。 */
    double accel_scale[3]; /**< Scale factors. 缩放因子。 */
    double accel_bias[3]; /**< Bias. 偏置。 */
    double accel_noise;          /**< Accelerometer noise density, m/s^2/sqrt(Hz). 加速度计噪声密度，m/s^2/sqrt(Hz)。 */
    double accel_random_walk;    /**< Accelerometer random walk, m/s^3/sqrt(Hz). 加速度计随机游走，m/s^3/sqrt(Hz)。 */
    double gyro_misalign[9]; /**< Cross-axis coupling matrix. 交轴耦合矩阵。 */
    double gyro_scale[3]; /**< Scale factors. 缩放因子。 */
    double gyro_bias[3]; /**< Bias. 偏置。 */
    double gyro_noise;           /**< Gyroscope noise density, rad/s/sqrt(Hz). 陀螺仪噪声密度，rad/s/sqrt(Hz)。 */
    double gyro_random_walk;     /**< Gyroscope random walk, rad/s^2/sqrt(Hz). 陀螺仪随机游走，rad/s^2/sqrt(Hz)。 */
}dsc_imu_intrinsics_t;

/**
 * Stereo camera and IMU intrinsics/extrinsics.
 * 双目 IMU 内外参。
 */
typedef struct dsc_calibration_s{
    dsc_imu_intrinsics_t imu;
    dsc_camera_intrinsics_t camera_right;
    dsc_camera_intrinsics_t camera_left;

    /**
     * Extrinsics are absolute poses expressed as sensor-to-reference transforms:
     * p_reference = R * p_sensor + T. The EEPROM driver selects the initial
     * reference frame. Rectification preserves that reference but replaces the
     * camera poses with virtual parallel-stereo poses; consumers must account for
     * that change when using rectified calibration.
     * 外参是以传感器到参考系变换表示的绝对位姿：p_reference = R * p_sensor + T。
     * EEPROM 驱动会选择初始参考系。校正（rectification）会保留该参考系，
     * 但会用虚拟的平行立体位姿替换相机位姿；使用校正后标定的调用方必须考虑这一变化。
     */
    double camera_right_R[9], camera_right_T[3];
    double camera_left_R[9], camera_left_T[3];
    double imu_R[9], imu_T[3];

    int camera_install_angle; /**< Installation rotation in degrees; normalized modulo 360 and required to be a multiple of 90. 安装旋转角度，单位为度；对 360 取模归一化，且必须是 90 的倍数。 */
}dsc_calibration_t;

/**
 * Reference coordinate objects.
 * 参考坐标对象。
 */
typedef enum dsc_reference_frame_e{
    DSC_REF_CAMERA_RIGHT, /**< Right camera frame. 右相机坐标系。 */
    DSC_REF_CAMERA_LEFT,  /**< Left camera frame. 左相机坐标系。 */
    DSC_REF_IMU,          /**< IMU frame. IMU 坐标系。 */
}dsc_reference_frame_t;

/**
 * @brief Get the camera intrinsics.
 * @brief 获取相机内参。
 *
 * This query is valid after initialization, whether capture is running or idle.
 * 此查询在初始化之后有效，无论采集正在运行还是空闲。
 *
 * @param[in]  dev        Initialized device handle.
 * @param[in]  dev        已初始化的设备句柄。
 * @param[in]  cam_idx    DSC_CAMERA_RIGHT_IDX or DSC_CAMERA_LEFT_IDX.
 * @param[in]  cam_idx    DSC_CAMERA_RIGHT_IDX 或 DSC_CAMERA_LEFT_IDX。
 * @param[out] intrinsics Destination for the camera intrinsics.
 * @param[out] intrinsics 相机内参的目标存放位置。
 * @retval DSC_OK Intrinsics were returned.
 * @retval DSC_OK 已返回内参。
 * @retval DSC_PARAM_ERROR An argument or camera index is invalid.
 * @retval DSC_PARAM_ERROR 某个参数或相机索引无效。
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_NOT_FOUND 未检测到标定 EEPROM。
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 * @retval DSC_HW_ERROR 设备存在已锁存的硬件故障。
 */
dsc_err_t dsc_get_camera_intrinsics(
    dsc_device_t *dev,
    dsc_camera_index_t cam_idx,
    dsc_camera_intrinsics_t *intrinsics);

/**
 * @brief Get the IMU intrinsics.
 * @brief 获取 IMU 内参。
 *
 * This query is valid after initialization, whether capture is running or idle.
 * 此查询在初始化之后有效，无论采集正在运行还是空闲。
 *
 * @param[in]  dev        Initialized device handle.
 * @param[in]  dev        已初始化的设备句柄。
 * @param[out] intrinsics Destination for the IMU intrinsics.
 * @param[out] intrinsics IMU 内参的目标存放位置。
 * @retval DSC_OK Intrinsics were returned.
 * @retval DSC_OK 已返回内参。
 * @retval DSC_PARAM_ERROR An argument is invalid.
 * @retval DSC_PARAM_ERROR 某个参数无效。
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_NOT_FOUND 未检测到标定 EEPROM。
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 * @retval DSC_HW_ERROR 设备存在已锁存的硬件故障。
 */
dsc_err_t dsc_get_imu_intrinsics(
    dsc_device_t *dev,
    dsc_imu_intrinsics_t *intrinsics);

/**
 * @brief Get the relative rotation matrix between two devices.
 * @brief 获取两个设备之间的相对旋转矩阵。
 *
 * @param[in]  dev        Initialized device handle.
 * @param[in]  dev        已初始化的设备句柄。
 * @param[in]  from_frame Frame in which the input point is expressed.
 * @param[in]  from_frame 输入点所在的坐标系。
 * @param[in]  to_frame   Frame in which the output point is expressed.
 * @param[in]  to_frame   输出点所在的坐标系。
 * @param[out] R          Row-major 3x3 rotation satisfying p_to = R * p_from.
 * @param[out] R          满足 p_to = R * p_from 的行主序 3x3 旋转矩阵。
 * @retval DSC_OK The rotation was returned.
 * @retval DSC_OK 已返回旋转矩阵。
 * @retval DSC_PARAM_ERROR An argument or frame identifier is invalid.
 * @retval DSC_PARAM_ERROR 某个参数或坐标系标识无效。
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_NOT_FOUND 未检测到标定 EEPROM。
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 * @retval DSC_HW_ERROR 设备存在已锁存的硬件故障。
 *
 * @note For identical frames, the result is numerically close to the identity
 *       matrix; it may contain floating-point roundoff.
 * @note 对于相同的坐标系，结果在数值上接近单位矩阵；其中可能包含浮点舍入误差。
 */
dsc_err_t dsc_get_relative_R(
    dsc_device_t *dev,
    dsc_reference_frame_t from_frame,
    dsc_reference_frame_t to_frame,
    double *R);

/**
 * @brief Get the relative translation vector between two devices.
 * @brief 获取两个设备之间的相对平移向量。
 *
 * Use this vector with dsc_get_relative_R() as p_to = R * p_from + T.
 * 将此向量与 dsc_get_relative_R() 配合使用，满足 p_to = R * p_from + T。
 *
 * @param[in]  dev        Initialized device handle.
 * @param[in]  dev        已初始化的设备句柄。
 * @param[in]  from_frame Frame in which the input point is expressed.
 * @param[in]  from_frame 输入点所在的坐标系。
 * @param[in]  to_frame   Frame in which the output point is expressed.
 * @param[in]  to_frame   输出点所在的坐标系。
 * @param[out] T          Three-element translation vector expressed in to_frame.
 * @param[out] T          在 to_frame 中表示的三元素平移向量。
 * @retval DSC_OK The translation was returned.
 * @retval DSC_OK 已返回平移向量。
 * @retval DSC_PARAM_ERROR An argument or frame identifier is invalid.
 * @retval DSC_PARAM_ERROR 某个参数或坐标系标识无效。
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_NOT_FOUND 未检测到标定 EEPROM。
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 * @retval DSC_HW_ERROR 设备存在已锁存的硬件故障。
 *
 * @note For identical frames, the result is numerically close to zero; it may
 *       contain floating-point roundoff.
 * @note 对于相同的坐标系，结果在数值上接近零；其中可能包含浮点舍入误差。
 */
dsc_err_t dsc_get_relative_T(
    dsc_device_t *dev,
    dsc_reference_frame_t from_frame,
    dsc_reference_frame_t to_frame,
    double *T);

/**
 * @brief Get the complete stereo (and IMU) calibration.
 * @brief 获取完整的立体（以及 IMU）标定。
 *
 * The result includes any virtual intrinsics/extrinsics produced by the selected
 * pipeline mode and any reference-frame conversion applied since initialization.
 * 结果包含所选流水线模式产生的任何虚拟内参/外参，以及自初始化以来应用过的任何参考系转换。
 *
 * @param[in]  dev         Initialized device handle.
 * @param[in]  dev         已初始化的设备句柄。
 * @param[out] calibration Destination for the complete calibration.
 * @param[out] calibration 完整标定的目标存放位置。
 * @retval DSC_OK Calibration was returned.
 * @retval DSC_OK 已返回标定。
 * @retval DSC_PARAM_ERROR An argument is invalid.
 * @retval DSC_PARAM_ERROR 某个参数无效。
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_NOT_FOUND 未检测到标定 EEPROM。
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 * @retval DSC_HW_ERROR 设备存在已锁存的硬件故障。
 */
dsc_err_t dsc_get_calibration(
    dsc_device_t *dev,
    dsc_calibration_t *calibration);

/**
 * @brief Change the reference frame of the devices.
 * @brief 更改设备的参考坐标系。
 *
 * Sets the selected device pose to (ref_R, ref_T) and transforms every other
 * device pose by the same rigid transform, preserving all relative poses.
 * 将选定设备的位姿设置为 (ref_R, ref_T)，并用同一刚体变换变换其他所有设备的位姿，
 * 从而保持所有相对位姿不变。
 *
 * @param[in] dev       Initialized device handle.
 * @param[in] dev       已初始化的设备句柄。
 * @param[in] ref_frame Device whose pose is being assigned.
 * @param[in] ref_frame 被赋予位姿的设备。
 * @param[in] ref_R     Row-major 3x3 sensor-to-new-reference rotation matrix.
 * @param[in] ref_R     行主序的 3x3 传感器到新参考系的旋转矩阵。
 * @param[in] ref_T     Three-element sensor-to-new-reference translation vector.
 * @param[in] ref_T     三元素传感器到新参考系的平移向量。
 * @retval DSC_OK The in-memory calibration was converted.
 * @retval DSC_OK 内存中的标定已转换。
 * @retval DSC_PARAM_ERROR An argument or frame identifier is invalid.
 * @retval DSC_PARAM_ERROR 某个参数或坐标系标识无效。
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_NOT_FOUND 未检测到标定 EEPROM。
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 * @retval DSC_HW_ERROR 设备存在已锁存的硬件故障。
 *
 * @note This function modifies the calibration stored inside the device, so later
 *       dsc_get_calibration / dsc_get_relative_R / dsc_get_relative_T results
 *       change accordingly.
 * @note 此函数会修改设备内部存储的标定，因此后续 dsc_get_calibration /
 *       dsc_get_relative_R / dsc_get_relative_T 的结果会相应改变。
 * @note Only valid after dsc_init(): init reloads the calibration from the EEPROM and
 *       overwrites any earlier change.  This function only changes the calibration data;
 *       it does not recompute the stereo rectification.
 * @note 仅在 dsc_init() 之后有效：init 会从 EEPROM 重新加载标定并覆盖之前的任何更改。
 *       此函数只修改标定数据，不会重新计算立体校正。
 */
dsc_err_t dsc_convert_calibration(
    dsc_device_t *dev,
    dsc_reference_frame_t ref_frame,
    const double ref_R[9],
    const double ref_T[3]);

/**
 * @brief Get the model name of the detected calibration EEPROM.
 * @brief 获取检测到的标定 EEPROM 的型号名称。
 *
 * @param[in] dev Initialized device handle.
 * @param[in] dev 已初始化的设备句柄。
 * @return Statically allocated EEPROM model string, or NULL if no supported
 *         EEPROM was detected. The caller must not modify or free it.
 * @return 静态分配的 EEPROM 型号字符串；如果未检测到受支持的 EEPROM，则为 NULL。
 *         调用方不得修改或释放它。
 */
const char *dsc_get_eeprom_name(
    dsc_device_t *dev);

/**
 * @brief Get details about the detected calibration EEPROM.
 * @brief 获取检测到的标定 EEPROM 的详细信息。
 *
 * The returned text may include the vendor, format version, device type, and
 * distortion model.
 * 返回的文本可能包含厂商、格式版本、设备类型和畸变模型。
 *
 * @param[in] dev Initialized device handle.
 * @param[in] dev 已初始化的设备句柄。
 * @return Statically allocated information string, or NULL if no supported
 *         EEPROM was detected. The caller must not modify or free it.
 * @return 静态分配的信息字符串；如果未检测到受支持的 EEPROM，则为 NULL。
 *         调用方不得修改或释放它。
 */
const char *dsc_get_eeprom_info(
    dsc_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* DSC_H */
