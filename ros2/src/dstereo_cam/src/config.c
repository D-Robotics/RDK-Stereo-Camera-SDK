/**
 * @file config.c
 * @brief C implementation of the SDK preset bridge used by the ROS 2 node.
 *
 * This file is compiled as GNU C because the SDK presets use designated-range
 * initializers and compound literals that are not a portable C++ interface.
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "dstereo_cam_define.h"

dsc_config_t dstereo_cam_config_from_define(
  const char * device, dstereo_cam_mode_t mode,
  uint32_t width, uint32_t height, uint32_t fps, uint32_t odr,
  dsc_stereo_layout_t layout)
{
  dsc_config_t config = DSC_CONFIG(device, mode, width, height, fps, odr);
  /* SDK presets select separate frames; the ROS parameter overrides that field. */
  config.camera_config.stereo_layout = layout;
  return config;
}
