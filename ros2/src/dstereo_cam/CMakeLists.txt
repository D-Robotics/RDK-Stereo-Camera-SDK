# RDK Stereo Camera ROS 2 package.
#
# This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
# Copyright (c) 2026 D-Robotics.
# SPDX-License-Identifier: MIT
# See the LICENSE file in the project root for the full license text.

cmake_minimum_required(VERSION 3.8)
project(dstereo_cam)

# config.c expands the SDK's GNU C preset macros; dstereo_cam_node.cpp uses C++17.
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ROS 2 build system and message/TF interfaces used by dstereo_cam_node.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

# libdstereocam does not export a CMake package configuration, so locate its library
# and public header independently. A checkout build finds the sibling core header.
# Standalone builds can use DSC_ROOT/include for headers, while the library must
# be in a default linker search path or supplied through the DSC_LIBRARY cache
# variable; DSC_INCLUDE_DIR may likewise override header discovery.
find_library(DSC_LIBRARY NAMES dstereocam)
find_path(DSC_INCLUDE_DIR NAMES dstereo_cam.h
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../core/include"
        "$ENV{DSC_ROOT}/include")

if(NOT DSC_LIBRARY)
  message(FATAL_ERROR "libdstereocam not found; configure with -DDSC_LIBRARY=/path/to/libdstereocam.so")
endif()
if(NOT DSC_INCLUDE_DIR)
  message(FATAL_ERROR "dstereo_cam.h not found; build from the SDK checkout or pass -DDSC_INCLUDE_DIR=")
endif()
if(NOT EXISTS "${DSC_INCLUDE_DIR}/dstereo_cam_define.h")
  message(FATAL_ERROR "dstereo_cam_define.h not found next to dstereo_cam.h")
endif()

# Build the C++ node together with the small GNU C preset bridge.
add_executable(dstereo_cam_node src/dstereo_cam_node.cpp src/config.c)
target_include_directories(dstereo_cam_node PRIVATE ${DSC_INCLUDE_DIR})
target_link_libraries(dstereo_cam_node ${DSC_LIBRARY})
ament_target_dependencies(dstereo_cam_node
  rclcpp sensor_msgs geometry_msgs builtin_interfaces tf2_ros)

# Install the executable where ros2 run expects package executables and copy all
# launch files into the package share directory.
install(TARGETS dstereo_cam_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
