/**
 * @file camera.c
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#define _DEFAULT_SOURCE
#include "RDKS600.h"

#include <stdio.h>
#include <unistd.h>

#define DSC_CAMERA_RAW10 0x2B

/* The BSP slave init table fixes two lanes (0x3018 = 0x32, bits [7:5] = 001);
 * the driver on this board does not receive this field, and there is no one-lane table.
 * BSP 从模式初始化表固定使用两条数据通道（0x3018 = 0x32，位 [7:5] = 001）；
 * 本板卡的驱动不接收此字段，BSP 也没有单通道初始化表。 */
#define DSC_CAMERA_LANES 2

/* BSP mode 6 uses the VIN LPWM trigger for stereo exposure.
 * BSP 模式 6 使用 VIN 的 LPWM 信号触发双目曝光。 */
#define DSC_CAMERA_SENSOR_MODE 6

int sensor_power(int gpio, int on)
{
    if(gpio < 0)return 0;

    char path[64];
    FILE *fp;

    snprintf(path, sizeof(path), "%d", gpio);
    fp = fopen("/sys/class/gpio/export", "w");
    if (fp) { fputs(path, fp); fclose(fp); }

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    fp = fopen(path, "w");
    if (fp) { fputs("out", fp); fclose(fp); }

    for (const char *p = on ? "101" : "0"; *p; ++p) {
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
        fp = fopen(path, "w");
        if (!fp)
            return -1;
        fputc(*p, fp);
        fclose(fp);
        if (on)
            usleep(30 * 1000);
    }
    return 0;
}

int camera_open(camera_handle_t *cam_fd, uint8_t i2c_addr,
                uint32_t width, uint32_t height, uint32_t fps,
                uint32_t line_length, uint32_t frame_length,
                uint16_t mipiclk, uint16_t settle, uint16_t mclk,
                const char *tuning_file)
{
    /* The BSP sets nocheck for a receiver started by the sensor stream.
     * 接收端由传感器码流启动，沿用 BSP 的 nocheck 设置。 */
    mipi_config_t mipi_cfg = {
        .rx_enable = 1,
        .rx_attr = {
            .phy = 0,
            .lane = DSC_CAMERA_LANES,
            .datatype = DSC_CAMERA_RAW10,
            .fps = (uint16_t)fps,
            .mclk = mclk,
            .mipiclk = mipiclk,
            .width = (uint16_t)width,
            .height = (uint16_t)height,
            .linelenth = (uint16_t)line_length,
            .framelenth = (uint16_t)frame_length,
            .settle = settle,
            .channel_num = 1,
            .channel_sel = {0},
        },
        .rx_ex_mask = 0x1,
        .rx_attr_ex = {
            .nocheck = 1,
        },
        .end_flag = MIPI_CONFIG_END_FLAG,
    };

    /* EEPROM and serializer addresses are BSP defaults. sensor_power() owns reset;
     * the camera driver must not toggle the GPIO.
     * EEPROM 和串行器地址沿用 BSP 默认值。复位由 sensor_power() 控制，不交给相机驱动。 */
    camera_config_t cam_cfg = {
        .name = "sc132gs",
        .addr = i2c_addr,
        .eeprom_addr = 0x51,
        .serial_addr = 0x40,
        .sensor_mode = DSC_CAMERA_SENSOR_MODE,
        .fps = fps,
        .format = DSC_CAMERA_RAW10,
        .width = width,
        .height = height,
        .extra_mode = 0,
        .config_index = 0,
        .mipi_cfg = &mipi_cfg,
        .gpio_enable = 0x00,
        .gpio_level = 0x00,
        .end_flag = CAMERA_CONFIG_END_FLAG,
    };
    snprintf(cam_cfg.calib_lname, sizeof(cam_cfg.calib_lname),
             "%s", tuning_file ? tuning_file : "disable");

    if (hbn_camera_create(&cam_cfg, cam_fd) != 0) {
        *cam_fd = 0;
        return -1;
    }
    return 0;
}
