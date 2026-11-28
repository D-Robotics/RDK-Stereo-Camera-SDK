/**
 * @file dsc-calib-export.c
 * @brief Export the EEPROM calibration as Kalibr YAML
 * @brief 将 EEPROM 标定导出为 Kalibr YAML
 *
 * usage: dsc-calib-export <dir> <device> <mode> <width> <height> <fps> <odr>
 *
 *   Positional arguments, all required, no flags:
 *
 *   <dir>     output directory, created when missing
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static void save_yaml(const char *dir, const char *name, char *yaml);
static char *camchain_yaml(dsc_device_t *dev, const dsc_calibration_t *cal,
                           int w, int h, bool has_imu, bool imu_missing);
static char *imu_yaml(const dsc_calibration_t *cal, int odr);

int main(int argc, char **argv)
{
    if(argc < 8)return 1;

    dsc_camera_mode_t mode = !strcmp(argv[3], "rect")   ? DSC_CAMERA_MODE_RECT :
                               !strcmp(argv[3], "resize") ? DSC_CAMERA_MODE_RESIZE :
                               DSC_CAMERA_MODE_RAW;

    dsc_config_t cfg = DSC_CONFIG(
        argv[2], mode, atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7]));

    /* Create Device Handle */
    /* 创建设备句柄 */
    dsc_device_t *dev = dsc_create();

    /* Init Device */
    /* 初始化设备 */
    int ret = 0;
    if(dsc_init(dev, &cfg) != DSC_OK){
        fprintf(stderr, "init failed\n");
        ret = 1; goto out;
    }

    dsc_calibration_t cal;
    if(dsc_get_calibration(dev, &cal) != DSC_OK){
        fprintf(stderr, "get_calibration failed\n");
        ret = 1; goto out;
    }

    /* Get stereo (and IMU) calibration data */
    /* 获取立体（以及 IMU）标定数据 */
    if(strcmp(argv[3], "raw") != 0){
        fprintf(stderr, "\033[33mwarning: mode is \"%s\", not \"raw\" -- the exported calibration is the "
            "rectified/resized one, not the EEPROM's raw calibration\033[0m\n", argv[3]);
    }
    /* Probe the IMU once and reuse the answer for both documents. Two probes can disagree
       while another process holds the device, and that dropped T_cam_imu or imu.yaml
       without a word. */
    /* IMU 只探测一次，两份文档复用同一结果。在另一个进程占用设备时，两次探测
       可能给出不同结果，而那样会悄无声息地丢掉 T_cam_imu 或 imu.yaml。 */
    const bool has_imu = dsc_get_imu_name(dev) != NULL;
    /* The caller's configuration says whether this module is supposed to have one, which is
       what separates "no IMU on this module" from "the probe failed". */
    /* 调用方的配置说明本模块是否应当带有 IMU，这正是区分"该模块没有 IMU"与
       "探测失败"的依据。 */
    const bool imu_missing = !has_imu && cfg.imu_config.bus_num > 0;
    if(imu_missing){
        fprintf(stderr, "warning: %s configures an IMU but the device reported none -- "
            "T_cam_imu and imu.yaml are omitted\n", argv[2]);
    }

    char *camchain = camchain_yaml(
        dev, &cal, 
        cfg.camera_config.output_width,
        cfg.camera_config.output_height,
        has_imu, imu_missing);
    if(camchain == NULL){
        fprintf(stderr, "out of memory building camchain.yaml\n");
        ret = 1; goto out;
    }
    printf("%s", camchain);
    save_yaml(argv[1], "camchain.yaml", camchain);

    /* Get IMU intrinsics and save */
    /* 获取 IMU 内参并保存 */
    if(has_imu){
        char *imu = imu_yaml(&cal, atoi(argv[7]));
        if(imu == NULL){
            fprintf(stderr, "out of memory building imu.yaml\n");
            ret = 1; goto out;
        }
        printf("%s", imu);
        save_yaml(argv[1], "imu.yaml", imu);
    }


out:
    dsc_deinit(dev);
    dsc_destroy(dev);
    return ret;
}

static char *camchain_yaml(dsc_device_t *dev, const dsc_calibration_t *cal,
                           int w, int h, bool has_imu, bool imu_missing)
{
    char *yaml = NULL;
    size_t len = 0;
    FILE *f = open_memstream(&yaml, &len);
    if(f == NULL)return NULL;

    fprintf(f, "# dstereo-cam EEPROM calibration, Kalibr camchain format\n");
    fprintf(f, "# p_to = T * p_from (T_cam_imu: IMU -> camera, T_cn_cnm1: cam0 -> cam1)\n");
    fprintf(f, "# cam0 is the left camera and cam1 is the right camera.\n");
    if(imu_missing)
        fprintf(f, "# warning: the configuration expects an IMU but the device reported "
                   "none; T_cam_imu is omitted, so this file is camera-only\n");
    fprintf(f, "# T_cam_imu takes a point in the IMU frame to the camera frame. A consumer that\n"
               "# expects T_imu_cam -- the other direction -- has to invert it:\n"
               "# T_imu_cam = inv(T_cam_imu).\n");

    for(int i = 0; i < 2; i++){
        /* Kalibr's usual convention: cam0 is the left camera and cam1 the right. */
        /* Kalibr 的常见约定：cam0 是左相机，cam1 是右相机。 */
        const dsc_camera_intrinsics_t *ci = i ? &cal->camera_right : &cal->camera_left;
        const dsc_reference_frame_t ref = i ? DSC_REF_CAMERA_RIGHT : DSC_REF_CAMERA_LEFT;
        double R[9], T[3];

        fprintf(f, "cam%d:\n", i);
        fprintf(f, "  camera_model: pinhole\n");
        fprintf(f, "  distortion_model: %s\n",
                (ci->dist_model == DSC_DIST_FISHEYE) ? "equidistant" : "radtan");
        /* Kalibr takes 4 coefficients: radtan=[k1,k2,p1,p2], equidistant=[k1,k2,k3,k4] */
        /* Kalibr 取 4 个系数：radtan=[k1,k2,p1,p2]，equidistant=[k1,k2,k3,k4] */
        fprintf(f, "  distortion_coeffs: [%.12g, %.12g, %.12g, %.12g]\n",
                ci->dist_coeffs[0], ci->dist_coeffs[1], ci->dist_coeffs[2], ci->dist_coeffs[3]);
        fprintf(f, "  intrinsics: [%.12g, %.12g, %.12g, %.12g]\n", ci->fx, ci->fy, ci->cx, ci->cy);
        fprintf(f, "  resolution: [%d, %d]\n", w, h);
        fprintf(f, "  rostopic: /image_%s/gray\n", i ? "right" : "left");

        if(has_imu){
            dsc_get_relative_R(dev, DSC_REF_IMU, ref, R);
            dsc_get_relative_T(dev, DSC_REF_IMU, ref, T);
            fprintf(f, "  T_cam_imu:\n");
            for(int r = 0; r < 3; r++)
                fprintf(f, "  - [%.12g, %.12g, %.12g, %.12g]\n",
                        R[r * 3], R[r * 3 + 1], R[r * 3 + 2], T[r]);
            fprintf(f, "  - [0.0, 0.0, 0.0, 1.0]\n");
        }
        if(i){
            /* cam1 (right) from cam0 (left): the usual Kalibr left-to-right chain. */
            /* cam1（右）相对 cam0（左）：Kalibr 常见的从左到右链。 */
            dsc_get_relative_R(dev, DSC_REF_CAMERA_LEFT, DSC_REF_CAMERA_RIGHT, R);
            dsc_get_relative_T(dev, DSC_REF_CAMERA_LEFT, DSC_REF_CAMERA_RIGHT, T);
            fprintf(f, "  T_cn_cnm1:\n");
            for(int r = 0; r < 3; r++)
                fprintf(f, "  - [%.12g, %.12g, %.12g, %.12g]\n",
                        R[r * 3], R[r * 3 + 1], R[r * 3 + 2], T[r]);
            fprintf(f, "  - [0.0, 0.0, 0.0, 1.0]\n");
        }
    }

    fclose(f);
    return yaml;
}

static char *imu_yaml(const dsc_calibration_t *cal, int odr)
{
    char *yaml = NULL;
    size_t len = 0;
    FILE *f = open_memstream(&yaml, &len);
    if(f == NULL)return NULL;

    const dsc_imu_intrinsics_t *imu = &cal->imu;

    fprintf(f, "# dstereo-cam EEPROM calibration, IMU parameters\n");
    fprintf(f, "accelerometer_noise_density: %.12g\n", imu->accel_noise);
    fprintf(f, "accelerometer_random_walk:   %.12g\n", imu->accel_random_walk);
    fprintf(f, "gyroscope_noise_density:     %.12g\n", imu->gyro_noise);
    fprintf(f, "gyroscope_random_walk:       %.12g\n", imu->gyro_random_walk);
    fprintf(f, "rostopic: /imu_data\n");
    fprintf(f, "update_rate: %d.0\n", odr);

    fprintf(f, "\n# --- dstereo-cam extension: IMU intrinsics (not part of Kalibr's schema) ---\n");

    for(int i = 0; i < 2; i++){
        const double *mis   = i ? imu->gyro_misalign : imu->accel_misalign;
        const double *scale = i ? imu->gyro_scale    : imu->accel_scale;
        const double *bias  = i ? imu->gyro_bias     : imu->accel_bias;

        fprintf(f, "%s:\n", i ? "gyroscope" : "accelerometer");
        fprintf(f, "  misalignment: [%.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g]\n",
                mis[0], mis[1], mis[2], mis[3], mis[4], mis[5], mis[6], mis[7], mis[8]);
        fprintf(f, "  scale: [%.12g, %.12g, %.12g]\n", scale[0], scale[1], scale[2]);
        fprintf(f, "  bias: [%.12g, %.12g, %.12g]\n", bias[0], bias[1], bias[2]);
    }

    fclose(f);
    return yaml;
}

static void save_yaml(const char *dir, const char *name, char *yaml)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", dir, name);

    for(char *p = path + 1; *p != '\0'; p++){
        if(*p != '/')continue;
        *p = '\0';
        mkdir(path, 0777);
        *p = '/';
    }

    FILE *f = fopen(path, "w");
    if(f != NULL){
        fputs(yaml, f);
        fclose(f);
    }
    free(yaml);
}
