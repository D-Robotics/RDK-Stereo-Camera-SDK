/**
 * @file dsc-version.c
 * @brief Print the SDK version and the build platform
 * @brief 打印 SDK 版本与构建平台
 *
 * usage: dsc-version
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "dstereo_cam.h"
#include <stdio.h>

int main(void)
{
    printf("dstereo_cam %s (platform: %s)\n", dsc_version(), dsc_platform());
    return 0;
}
