/**
 * @file dsc-eeprom-info.c
 * @brief Probe the EEPROM and print its model and details
 *
 * usage: dsc-eeprom-info <device> <mode> <width> <height> <fps> <odr>
 *
 *   Positional arguments, all required, no flags:
 *
 *   <device>  device model: GS130WI | GS130W | GS130W_NO_EEPROM | GS130WI_20260924
 *   <mode>    pipeline mode: raw | resize | rect (anything else means raw)
 *   <width>   output width in pixels
 *   <height>  output height in pixels
 *   <fps>     camera frame rate
 *   <odr>     IMU output data rate in Hz
 *
 *   Normally not called directly: 'eeprom-info' inside `dstereo-cam shell` passes the
 *   device config locked by the shell in exactly this order. 'eeprom-info
 *   --help' is answered by the shell, which prints its usage and never starts
 *   this program; outside the shell "--help" would be read as a device name.
 *
 * output: on stdout, the model line followed by the detail block
 *
 *   eeprom name: <model>
 *   <info>
 *
 *   or a single line when no EEPROM was detected:
 *
 *   no EEPROM detected
 *
 *   <info> is whatever dsc_get_eeprom_info() returns; a NULL detail block
 *   prints as an empty line, and a model name without details prints the name
 *   alone.
 *
 * exit status:
 *   0  a model name was reported, or no EEPROM was detected
 *   1  fewer than six arguments, or dsc_init() failed
 *
 * error reporting: this program prints plain, unprefixed diagnostics on stderr
 * ("init failed"), so a failure that happens in here does not carry the
 * 'dsc-...: <reason>' / try '--help' contract the shell script follows.
 * The same is true of the other sample programs, and of '--help' and argument
 * validation: those live in the dstereo-cam script, not in the samples.
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "dstereo_cam.h"
#include "dsc_define.h"
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
