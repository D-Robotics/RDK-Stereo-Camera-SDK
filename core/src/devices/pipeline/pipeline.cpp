/**
 * @file pipeline.cpp
 * @brief Build the backend selected by DSC_PLATFORM_IMPL. Do not compile its .cpp separately.
 * @brief 编译 DSC_PLATFORM_IMPL 选定的后端。不要单独编译后端的 .cpp 文件。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/pipeline/pipeline.hpp"

#ifndef DSC_PLATFORM_IMPL
#  error "No platform selected: the Makefile defines DSC_PLATFORM_IMPL from the platform directory name"
#endif

#include DSC_PLATFORM_IMPL
