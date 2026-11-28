/**
 * @file dsc-eeprom-info.c
 * @brief Probe the EEPROM and print its model and details
 * @brief 探测 EEPROM 并打印其型号与详细信息
 *
 * usage: dsc-eeprom-info <device> <mode> <width> <height> <fps> <odr>
 *
 *   Positional arguments, all required, no flags:
 *
 *   <device>  device model: GS130W | GS130WI | A1 | A1P
 *   <mode>    pipeline mode: raw | resize | rect (anything else means raw)
 *   <width>   output width in pixels
 *   <height>  output height in pixels
 *   <fps>     camera frame rate
 *   <odr>     IMU output data rate in Hz
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "dstereo_cam.h"
#include "dstereo_cam_define.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    if(argc < 7)return 1;
    const char *device   = argv[1];
    const char *mode_s   = argv[2];
    int w   = atoi(argv[3]);
    int h   = atoi(argv[4]);
    int fps = atoi(argv[5]);
    int odr = atoi(argv[6]);

    dsc_camera_mode_t mode = DSC_CAMERA_MODE_RAW;
    if(!strcmp(mode_s, "resize"))     mode = DSC_CAMERA_MODE_RESIZE;
    else if(!strcmp(mode_s, "rect"))  mode = DSC_CAMERA_MODE_RECT;

    dsc_config_t cfg = DSC_CONFIG(device, mode, w, h, fps, odr);
    dsc_device_t *dev = dsc_create();
    if(dsc_init(dev, &cfg) != DSC_OK){ fprintf(stderr, "init failed\n"); dsc_destroy(dev); return 1; }

    const char *name = dsc_get_eeprom_name(dev);
    if(!name){
        printf("no EEPROM detected\n");
    } else {
        const char *info = dsc_get_eeprom_info(dev);
        printf("eeprom name: %s\n%s\n", name, info ? info : "");
    }

    dsc_deinit(dev);
    dsc_destroy(dev);
    return 0;
}
