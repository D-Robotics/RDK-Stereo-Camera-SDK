/**
 * @file dstereo_cam.h
 * @brief RDK Stereo Camera SDK C API.
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

/** Result codes returned by the public SDK functions. */
typedef enum dsc_err_e{
    DSC_OK = 0,          /**< Operation completed successfully. */
    DSC_PARAM_ERROR,     /**< Invalid argument, configuration, or call order. */
    DSC_UNSUPPORTED,     /**< The selected hardware or mode is not supported. */
    DSC_NOT_FOUND,       /**< A required device or calibration record was not detected. */
    DSC_HW_ERROR,        /**< Low-level communication or driver failure. */
    DSC_TIMEOUT,         /**< A non-blocking data query found no item available. */
    DSC_THREAD_CLOSED,   /**< Capture threads are stopped or have not been started. */
}dsc_err_t;

/** Policy applied when a bounded FIFO is full. */
typedef enum dsc_fifo_mode_e{
    DSC_FIFO_DROP_NEW,   /**< Reject the newly produced item. */
    DSC_FIFO_DROP_OLD,   /**< Discard the oldest item and enqueue the new item. */
}dsc_fifo_mode_t;

/** Configuration for one bounded SDK FIFO. */
typedef struct dsc_fifo_config_s{
    size_t depth;              /**< Capacity; must be at least 2. */
    dsc_fifo_mode_t mode;   /**< Policy applied when the FIFO is full. */
}dsc_fifo_config_t;

/* ==================== Device Config ==================== */

/** Camera processing pipeline selected at initialization. */
typedef enum dsc_camera_mode_e{
    DSC_CAMERA_MODE_RAW,     /**< CAM -> VIN -> ISP -> OUT; no calibration required. */
    DSC_CAMERA_MODE_RESIZE,  /**< CAM -> VIN -> ISP -> VSE -> OUT; requires calibration. */
    DSC_CAMERA_MODE_RECT,    /**< CAM -> VIN -> ISP -> GDC -> VSE -> OUT; requires calibration. */
}dsc_camera_mode_t;

/** Logical camera identifiers used by the SDK. */
typedef enum dsc_camera_index_e{
    DSC_CAMERA_RIGHT_IDX = 0, /**< Right camera; also the default FSYNC camera. */
    DSC_CAMERA_LEFT_IDX  = 1, /**< Left camera. */
}dsc_camera_index_t;

/** Arrangement of the two camera images in a stitched output frame. */
typedef enum dsc_stereo_layout_e{
    DSC_STEREO_LAYOUT_NONE,          /**< No stitching; return separate frames. */
    DSC_STEREO_LAYOUT_LEFT_RIGHT,    /**< Horizontal: left image, then right image. */
    DSC_STEREO_LAYOUT_RIGHT_LEFT,    /**< Horizontal: right image, then left image. */
    DSC_STEREO_LAYOUT_TOP_BOTTOM,    /**< Vertical: left image, then right image. */
    DSC_STEREO_LAYOUT_BOTTOM_TOP,    /**< Vertical: right image, then left image. */
}dsc_stereo_layout_t;

/** Stereo-camera and platform pipeline configuration. */
typedef struct dsc_camera_config_s{
    uint8_t bus[32]; /**< Candidate I2C bus numbers, probed in array order. */
    size_t bus_num; /**< Number of valid entries in bus; maximum 32. */
    uint8_t  left_addr; /**< Left sensor I2C address. */
    uint8_t  right_addr; /**< Right sensor I2C address. */

    uint32_t sensor_width, sensor_height; /**< Native sensor dimensions in pixels. */
    uint32_t fps; /**< Camera frame rate in frames per second. */
    uint32_t line_length, frame_length; /**< Sensor timing values in pixel clocks and lines. */
    const char *tuning_file; /**< ISP tuning file path, or NULL to disable loading. */

    uint32_t output_width, output_height; /**< Output dimensions; RAW requires the native size. */
    dsc_camera_mode_t mode; /**< Processing pipeline to construct. */

    /** NONE returns separate frames; other values produce one prearranged frame. */
    dsc_stereo_layout_t stereo_layout;

    uint8_t bus_mipi_rx[32]; /**< I2C-bus-to-MIPI-RX map; 0xFF means unconfigured. */
    int bus_reset_gpio[32]; /**< I2C-bus-to-reset-GPIO map; -1 disables GPIO control. */

    dsc_camera_index_t fsync_camera; /**< Camera whose trigger is wired to IMU FSYNC. */
}dsc_camera_config_t;

/** IMU discovery and sampling configuration. */
typedef struct dsc_imu_config_s{
    uint8_t bus[32]; /**< Candidate I2C bus numbers, probed in array order. */
    size_t bus_num; /**< Number of valid entries in bus; 0 disables IMU probing. */
    uint8_t addr; /**< IMU I2C address. */
    uint32_t odr_hz; /**< Output data rate in Hz. */
    uint16_t accel_fsr_g; /**< Accelerometer full-scale range in g. */
    uint16_t gyro_fsr_dps; /**< Gyroscope full-scale range in degrees per second. */
    uint8_t accel_bw_sel; /**< Accelerometer UI-filter selector, 0x0..0xF; see dsc_get_imu_info(). */
    uint8_t gyro_bw_sel; /**< Gyroscope UI-filter selector, 0x0..0xF; see dsc_get_imu_info(). */
}dsc_imu_config_t;

/** EEPROM discovery configuration for calibration data. */
typedef struct dsc_eeprom_config_s{
    uint8_t bus[32]; /**< Candidate I2C bus numbers, probed in array order. */
    size_t bus_num; /**< Number of valid entries in bus; 0 disables EEPROM probing. */
    uint8_t addr; /**< EEPROM I2C address. */
}dsc_eeprom_config_t;

/**
 * Top-level configuration passed to dsc_init().
 *
 * Zero-initialize the complete structure before assigning fields. The SDK does
 * not supply implicit defaults for fields left at zero; alternatively, start
 * from one of the presets in dsc_define.h.
 */
typedef struct dsc_config_s{
    dsc_camera_config_t camera_config; /**< Camera and platform pipeline settings. */
    dsc_imu_config_t imu_config; /**< IMU settings. */
    dsc_eeprom_config_t eeprom_config; /**< Calibration EEPROM settings. */
    dsc_fifo_config_t camera_fifo; /**< Synchronized camera-frame FIFO. */
    dsc_fifo_config_t imu_fifo; /**< IMU-packet FIFO; required when IMU probing is enabled. */
}dsc_config_t;

/** Opaque device handle. */
typedef struct dsc_device_s dsc_device_t;

/**
 * @brief SDK version string.
 *
 * @return Version string in major.minor.patch form; statically allocated, do
 *         not free.
 */
const char *dsc_version(void);

/**
 * @brief Compiled-in platform string.
 *
 * @return Platform name (e.g. "RDKX5"), taken from the platform directory name at
 *         build time; statically allocated, do not free.
 */
const char *dsc_platform(void);

/**
 * @brief Create a device handle (an empty shell; call dsc_init to initialize it).
 *
 * The normal lifecycle is create -> init -> start -> stop -> deinit -> destroy.
 * Capture may be restarted after stop, and a deinitialized handle may be
 * initialized again.
 *
 * @return A new device handle. Release it with dsc_destroy().
 */
dsc_device_t *dsc_create();

/**
 * @brief Release a device handle.
 *
 * dsc_deinit() (or at least dsc_stop()) must be called first; destroying the
 * handle while background threads are still running is undefined behaviour.
 *
 * @param[in] dev Device handle returned by dsc_create(); NULL is accepted.
 */
void dsc_destroy(
    dsc_device_t *dev);

/**
 * @brief Initialize the device: probe EEPROM/IMU along the candidate buses in order,
 *        load the calibration, configure the camera and the IMU (streaming stays off).
 *
 * EEPROM and IMU probing are optional. RAW mode can operate without EEPROM
 * calibration; RESIZE and RECT require it because initialization updates the
 * output calibration. A handle can be initialized only once until deinitialized.
 *
 * @param[in] dev Device handle returned by dsc_create().
 * @param[in] cfg Fully initialized device configuration.
 * @retval DSC_OK Initialization succeeded.
 * @retval DSC_PARAM_ERROR An argument, configuration field, or call order is invalid.
 * @retval DSC_UNSUPPORTED The requested hardware configuration is unsupported.
 * @retval DSC_NOT_FOUND The stereo camera could not be detected.
 * @retval DSC_HW_ERROR Device communication or pipeline setup failed.
 */
dsc_err_t dsc_init(
    dsc_device_t *dev,
    const dsc_config_t *cfg);

/**
 * @brief Deinitialize: release all device resources (calls dsc_stop() first if still running).
 *
 * The handle itself stays valid afterwards and must be released by dsc_destroy().
 *
 * @param[in] dev Device handle returned by dsc_create().
 * @retval DSC_OK Resources were released, or the handle was already idle.
 * @retval DSC_PARAM_ERROR dev is NULL.
 */
dsc_err_t dsc_deinit(
    dsc_device_t *dev);

/**
 * @brief Start capture.
 *
 * When an IMU was detected, the camera waits for the IMU FSYNC handshake to complete
 * before streaming; without an IMU it streams immediately.  A repeated call returns
 * DSC_PARAM_ERROR.
 *
 * @param[in] dev Initialized device handle.
 * @retval DSC_OK Capture threads were started.
 * @retval DSC_PARAM_ERROR The handle is NULL, uninitialized, or already running.
 * @retval DSC_HW_ERROR The IMU or camera pipeline failed to start.
 */
dsc_err_t dsc_start(
    dsc_device_t *dev);

/**
 * @brief Stop capture and wait for the background threads to exit.
 *
 * A no-op when not running; dsc_start() may be called again afterwards.
 *
 * @param[in] dev Device handle; NULL is accepted and ignored.
 */
void dsc_stop(
    dsc_device_t *dev);

/* ==================== Camera Data ==================== */

/** One tightly packed NV12 image returned by the SDK. */
typedef struct dsc_image_nv12_s{
    uint8_t *data; /**< SDK-allocated buffer: Y plane followed by interleaved UV; caller must free(). */
    uint32_t width, height; /**< Image dimensions in pixels. */
    uint64_t timestamp_ns; /**< Capture timestamp in nanoseconds, aligned to the camera clock. */
}dsc_image_nv12_t;

/**
 * @brief Query how many synchronized stereo frame pairs the queue currently holds.
 *
 * The count is a snapshot and may change before the next call. The frame getters
 * are non-blocking and return DSC_TIMEOUT if another consumer empties the FIFO.
 *
 * @param[in] dev Running device handle.
 * @return Number of frame pairs currently queued, or 0 for an invalid, idle, or
 *         faulted device.
 */
size_t dsc_available_camera(
    dsc_device_t *dev);

/**
 * @brief Fetch the left and right NV12 frames separately.
 *
 * This function is non-blocking. On success, each returned data pointer is owned
 * by the caller and must be released with free().
 *
 * @param[in]  dev Running device handle.
 * @param[out] image_left Left-camera frame.
 * @param[out] image_right Right-camera frame.
 * @retval DSC_OK A synchronized frame pair was returned.
 * @retval DSC_PARAM_ERROR An argument is NULL or the camera FIFO is unavailable.
 * @retval DSC_UNSUPPORTED A stitched output layout is configured.
 * @retval DSC_TIMEOUT The FIFO is currently empty.
 * @retval DSC_THREAD_CLOSED Capture is not running.
 * @retval DSC_HW_ERROR A capture thread terminated after a hardware failure.
 */
dsc_err_t dsc_get_nv12_frame(
    dsc_device_t *dev,
    dsc_image_nv12_t *image_left,
    dsc_image_nv12_t *image_right);

/**
 * @brief Fetch a stitched left+right NV12 frame.
 *
 * The layout is fixed by dsc_camera_config_t.stereo_layout during
 * dsc_init(). This function is non-blocking and transfers ownership of the
 * already assembled pixel buffer to the caller; release image->data with free().
 *
 * @param[in]  dev Running device handle.
 * @param[out] image Stitched NV12 frame. Horizontal layouts double width; vertical
 *                   layouts double height.
 * @retval DSC_OK A stitched frame was returned.
 * @retval DSC_PARAM_ERROR An argument is NULL or the camera FIFO is unavailable.
 * @retval DSC_UNSUPPORTED DSC_STEREO_LAYOUT_NONE is configured.
 * @retval DSC_TIMEOUT The FIFO is currently empty.
 * @retval DSC_THREAD_CLOSED Capture is not running.
 * @retval DSC_HW_ERROR A capture thread terminated after a hardware failure.
 */
dsc_err_t dsc_get_stereo_nv12_frame(
    dsc_device_t *dev,
    dsc_image_nv12_t *image);

/* ==================== IMU Data ==================== */

/** One timestamp-corrected IMU sample. */
typedef struct dsc_imu_packet_s{
    float accel[3]; /**< Acceleration in m/s^2. */
    float gyro[3]; /**< Angular velocity in rad/s. */
    float temp; /**< Temperature in degrees Celsius. */
    bool  is_fsync; /**< True when the sample carries an IMU FSYNC anchor. */
    uint64_t timestamp_ns; /**< Absolute timestamp in ns, aligned to the camera clock. */
}dsc_imu_packet_t;

/**
 * @brief Query how many IMU packets the queue currently holds.
 *
 * The count is a snapshot and may change before the next call. Returns 0 when
 * no IMU is available, capture is idle, or the device has faulted.
 *
 * @param[in] dev Running device handle.
 * @return Number of packets currently queued.
 */
size_t dsc_available_imu(
    dsc_device_t *dev);

/**
 * @brief Fetch one IMU packet.
 *
 * This function is non-blocking.
 *
 * @param[in]  dev Running device handle.
 * @param[out] out Destination for one IMU packet.
 * @retval DSC_OK One packet was returned.
 * @retval DSC_PARAM_ERROR An argument is NULL or no IMU is available.
 * @retval DSC_TIMEOUT The FIFO is currently empty.
 * @retval DSC_THREAD_CLOSED Capture is not running.
 * @retval DSC_HW_ERROR The IMU thread terminated after a hardware failure.
 */
dsc_err_t dsc_get_imu_packet(
    dsc_device_t *dev,
    dsc_imu_packet_t *out);

/**
 * @brief Get the model name of the detected IMU.
 *
 * @param[in] dev Device handle from dsc_create().
 * @return Statically allocated IMU model string, or NULL if no IMU was detected.
 *         The caller must not modify or free the returned string.
 */
const char *dsc_get_imu_name(
    dsc_device_t *dev);

/**
 * @brief Get detailed information about the detected IMU (setting/bandwidth tables, etc.).
 *
 * @param[in] dev Device handle from dsc_create().
 * @return Statically allocated information string, or NULL if no IMU was
 *         detected. The caller must not modify or free the returned string.
 */
const char *dsc_get_imu_info(
    dsc_device_t *dev);

/* ==================== EEPROM Data ==================== */

/** Camera distortion model used by dist_coeffs. */
typedef enum dsc_dist_model_e{
    DSC_DIST_PINHOLE,    /**< OpenCV/radtan order: [k1,k2,p1,p2,k3,k4,k5,k6]. */
    DSC_DIST_FISHEYE,    /**< Equidistant order: [k1,k2,k3,k4,0,0,0,0]. */
}dsc_dist_model_t;

/** Camera intrinsic parameters. Matrices are stored row-major. */
typedef struct dsc_camera_intrinsics_s{
    double fx, fy, cx, cy; /**< Focal lengths and principal point in pixels. */
    /** K is a row-major 3x3 intrinsic matrix; dist_coeffs uses the model order above. */
    double K[9], dist_coeffs[8];
    dsc_dist_model_t dist_model; /**< Model used to interpret dist_coeffs. */
}dsc_camera_intrinsics_t;

/** IMU calibration parameters and noise characteristics. */
typedef struct dsc_imu_intrinsics_s{
    double accel_misalign[9]; /**< 3x3 row-major cross-axis coupling matrix. */
    double accel_scale[3]; /**< Per-axis scale factors. */
    double accel_bias[3]; /**< Per-axis bias in the calibrated sensor units. */
    double accel_noise;          /**< Accelerometer noise density, m/s^2/sqrt(Hz). */
    double accel_random_walk;    /**< Accelerometer random walk, m/s^3/sqrt(Hz). */
    double gyro_misalign[9]; /**< 3x3 row-major cross-axis coupling matrix. */
    double gyro_scale[3]; /**< Per-axis scale factors. */
    double gyro_bias[3]; /**< Per-axis bias in rad/s. */
    double gyro_noise;           /**< Gyroscope noise density, rad/s/sqrt(Hz). */
    double gyro_random_walk;     /**< Gyroscope random walk, rad/s^2/sqrt(Hz). */
}dsc_imu_intrinsics_t;

/** Complete camera, IMU, and extrinsic calibration. */
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
     */
    double camera_right_R[9], camera_right_T[3];
    double camera_left_R[9], camera_left_T[3];
    double imu_R[9], imu_T[3];

    int camera_install_angle; /**< Installation rotation in degrees; normalized modulo 360 and required to be a multiple of 90. */
}dsc_calibration_t;

/** Frames that may be used as calibration references. */
typedef enum dsc_reference_frame_e{
    DSC_REF_CAMERA_RIGHT, /**< Right camera frame. */
    DSC_REF_CAMERA_LEFT,  /**< Left camera frame. */
    DSC_REF_IMU,          /**< IMU frame. */
}dsc_reference_frame_t;

/**
 * @brief Get the camera intrinsics.
 *
 * This query is valid after initialization, whether capture is running or idle.
 *
 * @param[in]  dev        Initialized device handle.
 * @param[in]  cam_idx    DSC_CAMERA_RIGHT_IDX or DSC_CAMERA_LEFT_IDX.
 * @param[out] intrinsics Destination for the camera intrinsics.
 * @retval DSC_OK Intrinsics were returned.
 * @retval DSC_PARAM_ERROR An argument or camera index is invalid.
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 */
dsc_err_t dsc_get_camera_intrinsics(
    dsc_device_t *dev,
    dsc_camera_index_t cam_idx,
    dsc_camera_intrinsics_t *intrinsics);

/**
 * @brief Get the IMU intrinsics.
 *
 * This query is valid after initialization, whether capture is running or idle.
 *
 * @param[in]  dev        Initialized device handle.
 * @param[out] intrinsics Destination for the IMU intrinsics.
 * @retval DSC_OK Intrinsics were returned.
 * @retval DSC_PARAM_ERROR An argument is invalid.
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 */
dsc_err_t dsc_get_imu_intrinsics(
    dsc_device_t *dev,
    dsc_imu_intrinsics_t *intrinsics);

/**
 * @brief Get the relative rotation matrix between two devices.
 *
 * @param[in]  dev        Initialized device handle.
 * @param[in]  from_frame Frame in which the input point is expressed.
 * @param[in]  to_frame   Frame in which the output point is expressed.
 * @param[out] R          Row-major 3x3 rotation satisfying p_to = R * p_from.
 * @retval DSC_OK The rotation was returned.
 * @retval DSC_PARAM_ERROR An argument or frame identifier is invalid.
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 *
 * @note For identical frames, the result is numerically close to the identity
 *       matrix; it may contain floating-point roundoff.
 */
dsc_err_t dsc_get_relative_R(
    dsc_device_t *dev,
    dsc_reference_frame_t from_frame,
    dsc_reference_frame_t to_frame,
    double *R);

/**
 * @brief Get the relative translation vector between two devices.
 *
 * Use this vector with dsc_get_relative_R() as p_to = R * p_from + T.
 *
 * @param[in]  dev        Initialized device handle.
 * @param[in]  from_frame Frame in which the input point is expressed.
 * @param[in]  to_frame   Frame in which the output point is expressed.
 * @param[out] T          Three-element translation vector expressed in to_frame.
 * @retval DSC_OK The translation was returned.
 * @retval DSC_PARAM_ERROR An argument or frame identifier is invalid.
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 *
 * @note For identical frames, the result is numerically close to zero; it may
 *       contain floating-point roundoff.
 */
dsc_err_t dsc_get_relative_T(
    dsc_device_t *dev,
    dsc_reference_frame_t from_frame,
    dsc_reference_frame_t to_frame,
    double *T);

/**
 * @brief Get the complete stereo (and IMU) calibration.
 *
 * The result includes any virtual intrinsics/extrinsics produced by the selected
 * pipeline mode and any reference-frame conversion applied since initialization.
 *
 * @param[in]  dev         Initialized device handle.
 * @param[out] calibration Destination for the complete calibration.
 * @retval DSC_OK Calibration was returned.
 * @retval DSC_PARAM_ERROR An argument is invalid.
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 */
dsc_err_t dsc_get_calibration(
    dsc_device_t *dev,
    dsc_calibration_t *calibration);

/**
 * @brief Change the reference frame of the devices.
 *
 * Sets the selected device pose to (ref_R, ref_T) and transforms every other
 * device pose by the same rigid transform, preserving all relative poses.
 *
 * @param[in] dev       Initialized device handle.
 * @param[in] ref_frame Device whose pose is being assigned.
 * @param[in] ref_R     Row-major 3x3 sensor-to-new-reference rotation matrix.
 * @param[in] ref_T     Three-element sensor-to-new-reference translation vector.
 * @retval DSC_OK The in-memory calibration was converted.
 * @retval DSC_PARAM_ERROR An argument or frame identifier is invalid.
 * @retval DSC_NOT_FOUND No calibration EEPROM was detected.
 * @retval DSC_HW_ERROR The device has a latched hardware failure.
 *
 * @note This function modifies the calibration stored inside the device, so later
 *       dsc_get_calibration / dsc_get_relative_R / dsc_get_relative_T results
 *       change accordingly.
 * @note Only valid after dsc_init(): init reloads the calibration from the EEPROM and
 *       overwrites any earlier change.  This function only changes the calibration data;
 *       it does not recompute the stereo rectification.
 */
dsc_err_t dsc_convert_calibration(
    dsc_device_t *dev,
    dsc_reference_frame_t ref_frame,
    const double ref_R[9],
    const double ref_T[3]);

/**
 * @brief Get the model name of the detected calibration EEPROM.
 *
 * @param[in] dev Initialized device handle.
 * @return Statically allocated EEPROM model string, or NULL if no supported
 *         EEPROM was detected. The caller must not modify or free it.
 */
const char *dsc_get_eeprom_name(
    dsc_device_t *dev);

/**
 * @brief Get details about the detected calibration EEPROM.
 *
 * The returned text may include the vendor, format version, device type, and
 * distortion model.
 *
 * @param[in] dev Initialized device handle.
 * @return Statically allocated information string, or NULL if no supported
 *         EEPROM was detected. The caller must not modify or free it.
 */
const char *dsc_get_eeprom_info(
    dsc_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* DSC_H */
