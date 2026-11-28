/**
 * @file dsc_define.h
 * @brief Optional preset configurations for supported DSC modules.
 * @brief 受支持 DSC 模组的可选预置配置。
 *
 * Include dstereo_cam.h, string.h, stdio.h, and stdlib.h before this header.
 * 使用本头文件前请先包含 dstereo_cam.h、string.h、stdio.h 和 stdlib.h。
 *
 * @warning Preset values may change between SDK releases. This header does not
 *          provide a backward-compatible configuration ABI; use the presets flexibly.
 * @warning 预置值可能随 SDK 版本变化。本头文件不提供向后兼容的配置 ABI，建议开发者灵活使用。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSTEREO_CAM_DEFINE_H
#define DSTEREO_CAM_DEFINE_H

#define DSC_CONFIG(device, mode, w, h, fps, odr) DSC_CONFIG_PLATFORM(dsc_platform(), device, mode, w, h, fps, odr)

#define DSC_CONFIG_PLATFORM(platform, device, mode, w, h, fps, odr) \
    strcmp((platform), "RDKX5") == 0 && strcmp((device), "GS130W")             == 0 ? (dsc_config_t) DSC_CONFIG_RDKX5_GS130W((mode), (w), (h), (fps), (odr))  : \
    strcmp((platform), "RDKX5") == 0 && strcmp((device), "GS130WI")            == 0 ? (dsc_config_t) DSC_CONFIG_RDKX5_GS130WI((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKX5") == 0 && strcmp((device), "A1")                 == 0 ? (dsc_config_t) DSC_CONFIG_A1((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKX5") == 0 && strcmp((device), "A1P")                == 0 ? (dsc_config_t) DSC_CONFIG_A1P((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKS100") == 0 && strcmp((device), "GS130W")           == 0 ? (dsc_config_t) DSC_CONFIG_RDKS100_GS130W((mode), (w), (h), (fps), (odr))  : \
    strcmp((platform), "RDKS100") == 0 && strcmp((device), "GS130WI")          == 0 ? (dsc_config_t) DSC_CONFIG_RDKS100_GS130WI((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKS100") == 0 && strcmp((device), "A1")               == 0 ? (dsc_config_t) DSC_CONFIG_RDKS100_A1((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKS100") == 0 && strcmp((device), "A1P")              == 0 ? (dsc_config_t) DSC_CONFIG_RDKS100_A1P((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKS600") == 0 && strcmp((device), "GS130W")           == 0 ? (dsc_config_t) DSC_CONFIG_RDKS600_GS130W((mode), (w), (h), (fps), (odr))  : \
    strcmp((platform), "RDKS600") == 0 && strcmp((device), "GS130WI")          == 0 ? (dsc_config_t) DSC_CONFIG_RDKS600_GS130WI((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKS600") == 0 && strcmp((device), "A1")               == 0 ? (dsc_config_t) DSC_CONFIG_RDKS600_A1((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKS600") == 0 && strcmp((device), "A1P")              == 0 ? (dsc_config_t) DSC_CONFIG_RDKS600_A1P((mode), (w), (h), (fps), (odr)) : \
    (fprintf(stderr, "unsupported platform/device: %s %s\n", (platform), (device)), exit(1), (dsc_config_t){0})

#define DSC_CONFIG_RDKX5_GS130WI(mode_, width_, height_, fps_, odr_) {    \
    .camera_config = {                                                      \
        .bus = {4, 6}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = NULL,                                                \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 2, [5] = 0xFF, [6] = 0, [7 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 351, [5] = -1, [6] = 353, [7 ... 31] = -1 },   \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus = {4, 6}, .bus_num = 2, .addr = 0x68,                          \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {4, 6}, .bus_num = 2, .addr = 0x50 },         \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
    .imu_fifo    = { .depth = 1024, .mode = DSC_FIFO_DROP_OLD },          \
}

#define DSC_CONFIG_RDKX5_GS130W(mode_, width_, height_, fps_, odr_) {    \
    .camera_config = {                                                      \
        .bus = {4, 6}, .bus_num = 2,                                        \
        .left_addr = 0x33, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = NULL,                                                \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 2, [5] = 0xFF, [6] = 0, [7 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 351, [5] = -1, [6] = 353, [7 ... 31] = -1 },   \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus_num = 0, .addr = 0x68,                                         \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {4, 6}, .bus_num = 2, .addr = 0x50 },         \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
}
/* A1: the same SC132GS stereo pair as the A1P without the IMU, so the pipeline
   starts streaming directly instead of waiting for the F-SYNC handshake.
   A1：与 A1P 相同的 SC132GS 双目模组，只是不含 IMU，因此管线直接开始出流；
   而不必等待 F-SYNC 握手。 */
#define DSC_CONFIG_A1(mode_, width_, height_, fps_, odr_) {                 \
    .camera_config = {                                                      \
        .bus = {4, 6}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = NULL,                                                \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 2, [5] = 0xFF, [6] = 0, [7 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 351, [5] = -1, [6] = 353, [7 ... 31] = -1 },   \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus_num = 0, .addr = 0x68,                                         \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {4, 6}, .bus_num = 2, .addr = 0x50 },         \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
}

/* A1P: SC132GS stereo pair, 70 mm baseline, IMU (ICM-42607) with its
   F-SYNC input wired to the right eye.
   A1P：SC132GS 双目模组，70 mm 基线，IMU（ICM-42607）的 F-SYNC 输入接到右目。 */
#define DSC_CONFIG_A1P(mode_, width_, height_, fps_, odr_) {         \
    .camera_config = {                                                      \
        .bus = {4, 6}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = NULL,                                                \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 2, [5] = 0xFF, [6] = 0, [7 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 351, [5] = -1, [6] = 353, [7 ... 31] = -1 },   \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus = {4, 6}, .bus_num = 2, .addr = 0x68,                          \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {4, 6}, .bus_num = 2, .addr = 0x50 },         \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
    .imu_fifo    = { .depth = 1024, .mode = DSC_FIFO_DROP_OLD },          \
}

#define DSC_CONFIG_RDKS100_GS130WI(mode_, width_, height_, fps_, odr_) {  \
    .camera_config = {                                                      \
        .bus = {1, 2}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = "lib_sc132gs_linear.so",                             \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0] = 0xFF, [1] = 0, [2] = 1, [3 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 31] = -1 },                              \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus = {2}, .bus_num = 1, .addr = 0x68,                             \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {2}, .bus_num = 1, .addr = 0x50 },            \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
    .imu_fifo    = { .depth = 1024, .mode = DSC_FIFO_DROP_OLD },          \
}

#define DSC_CONFIG_RDKS100_GS130W(mode_, width_, height_, fps_, odr_) {   \
    .camera_config = {                                                      \
        .bus = {1, 2}, .bus_num = 2,                                        \
        .left_addr = 0x33, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = "lib_sc132gs_linear.so",                             \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0] = 0xFF, [1] = 0, [2] = 1, [3 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 31] = -1 },                              \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus_num = 0, .addr = 0x68,                                         \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {1, 2}, .bus_num = 2, .addr = 0x50 },         \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
}

#define DSC_CONFIG_RDKS600_GS130WI(mode_, width_, height_, fps_, odr_) {  \
    .camera_config = {                                                      \
        .bus = {4, 5}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = "lib_sc132gs_linear.so",                             \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 4, [5] = 5, [6 ... 31] = 0xFF },  \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 411, [5] = 412, [6 ... 31] = -1 },  \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus = {5}, .bus_num = 1, .addr = 0x68,                             \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {5}, .bus_num = 1, .addr = 0x50 },            \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
    .imu_fifo    = { .depth = 1024, .mode = DSC_FIFO_DROP_OLD },          \
}

#define DSC_CONFIG_RDKS600_GS130W(mode_, width_, height_, fps_, odr_) {   \
    .camera_config = {                                                      \
        .bus = {4, 5}, .bus_num = 2,                                        \
        .left_addr = 0x33, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = "lib_sc132gs_linear.so",                             \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 4, [5] = 5, [6 ... 31] = 0xFF },  \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 411, [5] = 412, [6 ... 31] = -1 },  \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus_num = 0, .addr = 0x68,                                         \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {4, 5}, .bus_num = 2, .addr = 0x50 },         \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
}

/* A1P on RDKS100: SC132GS stereo pair with the ICM-42607 IMU on bus 2.
   RDKS100 上的 A1P：SC132GS 双目模组，ICM-42607 IMU 接在总线 2 上。 */
#define DSC_CONFIG_RDKS100_A1P(mode_, width_, height_, fps_, odr_) {      \
    .camera_config = {                                                      \
        .bus = {1, 2}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = "lib_sc132gs_linear.so",                             \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0] = 0xFF, [1] = 0, [2] = 1, [3 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 31] = -1 },                              \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus = {2}, .bus_num = 1, .addr = 0x68,                             \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {2}, .bus_num = 1, .addr = 0x50 },            \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
    .imu_fifo    = { .depth = 1024, .mode = DSC_FIFO_DROP_OLD },          \
}

/* A1 on RDKS100: A1P without the IMU.
   RDKS100 上的 A1：A1P 去掉 IMU。 */
#define DSC_CONFIG_RDKS100_A1(mode_, width_, height_, fps_, odr_) {       \
    .camera_config = {                                                      \
        .bus = {1, 2}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = "lib_sc132gs_linear.so",                             \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0] = 0xFF, [1] = 0, [2] = 1, [3 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 31] = -1 },                              \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus_num = 0, .addr = 0x68,                                         \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {2}, .bus_num = 1, .addr = 0x50 },            \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
}

/* A1P on RDKS600: SC132GS stereo pair with the ICM-42607 IMU on bus 5.
   RDKS600 上的 A1P：SC132GS 双目模组，ICM-42607 IMU 接在总线 5 上。 */
#define DSC_CONFIG_RDKS600_A1P(mode_, width_, height_, fps_, odr_) {      \
    .camera_config = {                                                      \
        .bus = {4, 5}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = "lib_sc132gs_linear.so",                             \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 4, [5] = 5, [6 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 411, [5] = 412, [6 ... 31] = -1 }, \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus = {5}, .bus_num = 1, .addr = 0x68,                             \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {5}, .bus_num = 1, .addr = 0x50 },            \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
    .imu_fifo    = { .depth = 1024, .mode = DSC_FIFO_DROP_OLD },          \
}

/* A1 on RDKS600: A1P without the IMU.
   RDKS600 上的 A1：A1P 去掉 IMU。 */
#define DSC_CONFIG_RDKS600_A1(mode_, width_, height_, fps_, odr_) {       \
    .camera_config = {                                                      \
        .bus = {4, 5}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = "lib_sc132gs_linear.so",                             \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = DSC_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 4, [5] = 5, [6 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 411, [5] = 412, [6 ... 31] = -1 }, \
        .fsync_camera = DSC_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus_num = 0, .addr = 0x68,                                         \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .bw_sel = 0,                                                        \
    },                                                                      \
    .eeprom_config = { .bus = {5}, .bus_num = 1, .addr = 0x50 },            \
    .camera_fifo = { .depth = 4,    .mode = DSC_FIFO_DROP_OLD },          \
}

#endif /* DSTEREO_CAM_DEFINE_H */
