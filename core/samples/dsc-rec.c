/**
 * @file dsc-rec.c
 * @brief Record camera frames and IMU packets into index ranges
 * @brief 把相机帧与 IMU 数据包按索引区间录制下来
 *
 * usage: dsc-rec <cam_from> <cam_to> <imu_from> <imu_to> <out_dir> <stitch>
 *                   <device> <mode> <width> <height> <fps> <odr>
 *
 *   Positional arguments, all required, no flags:
 *
 *   <cam_from>, <cam_to>  half-open camera index range [from, to) to keep,
 *                         from = -1 records no camera frames
 *   <imu_from>, <imu_to>  the same for IMU packets
 *   <out_dir>             output directory, created when missing
 *   <stitch>              non-zero records one stitched left-right frame per
 *                         camera index, 0 records one file per eye
 *   <device>              device model: GS130W | GS130WI | A1 | A1P
 *   <mode>                pipeline mode: raw | resize | rect (anything else is raw)
 *   <width>, <height>     output size in pixels
 *   <fps>                 camera frame rate
 *   <odr>                 IMU output data rate in Hz
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "dstereo_cam.h"
#include "dstereo_cam_define.h"

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define MAX_FRAMES 512   /* the kept range lives in RAM: ~2 GiB at 1088x1280 stereo 保留的区间常驻内存：1088x1280 双目约 2 GiB */
#define REPORT_US  1000000

static volatile sig_atomic_t stop = 0;
static void on_sigint(int sig){(void)sig; stop = 1;}

static uint64_t now_us(void);
static void mkdir_p(const char *path);
static void report(uint64_t t0, bool keep_cam, size_t ncam, size_t cam_cap,
                   bool keep_imu, size_t nimu, size_t imu_cap);
static bool get_frame(dsc_device_t *dev, bool stitch, dsc_image_nv12_t f[2]);
static void save_frame(const char *dir, long index, const char *suffix,
                       const dsc_image_nv12_t *image);
static void save_imu(const char *dir, long from, const dsc_imu_packet_t *imu, size_t n);

int main(int argc, char **argv)
{
    if(argc < 13)return 1;
    signal(SIGINT, on_sigint);

    const long cam_from = atol(argv[1]), cam_to = atol(argv[2]);
    const long imu_from = atol(argv[3]), imu_to = atol(argv[4]);
    const char *dir = argv[5];
    const bool stitch = (atoi(argv[6]) != 0);
    const bool keep_cam = (cam_from >= 0), keep_imu = (imu_from >= 0);

    if(!keep_cam && !keep_imu){
        fprintf(stderr, "nothing to record\n");
        return 1;
    }
    if(keep_cam && cam_to - cam_from > MAX_FRAMES){
        fprintf(stderr, "camera range is %ld frames, the limit is %d\n", cam_to - cam_from, MAX_FRAMES);
        return 1;
    }

    const size_t cam_cap = keep_cam ? (size_t)(cam_to - cam_from) : 0;
    const size_t imu_cap = keep_imu ? (size_t)(imu_to - imu_from) : 0;
    dsc_image_nv12_t (*cam)[2] = cam_cap ? calloc(cam_cap, sizeof(*cam)) : NULL;
    dsc_imu_packet_t *imu = imu_cap ? calloc(imu_cap, sizeof(*imu)) : NULL;
    if((cam_cap && cam == NULL) || (imu_cap && imu == NULL)){
        fprintf(stderr, "out of memory\n");
        return 1;
    }
    size_t ncam = 0, nimu = 0;

    mkdir_p(dir);
    char cam_dir[512];
    snprintf(cam_dir, sizeof(cam_dir), "%s/cam", dir);
    if(keep_cam)mkdir_p(cam_dir);

    dsc_camera_mode_t mode = !strcmp(argv[8], "rect")   ? DSC_CAMERA_MODE_RECT :
                               !strcmp(argv[8], "resize") ? DSC_CAMERA_MODE_RESIZE :
                               DSC_CAMERA_MODE_RAW;

    dsc_config_t cfg = DSC_CONFIG(
        argv[7], mode, atoi(argv[9]), atoi(argv[10]), atoi(argv[11]), atoi(argv[12]));
    if(stitch)cfg.camera_config.stereo_layout = DSC_STEREO_LAYOUT_LEFT_RIGHT;

    /* Create Device Handle */
    /* 创建设备句柄 */
    dsc_device_t *dev = dsc_create();

    /* Init Device and Start DataFlow */
    /* 初始化设备并启动数据流 */
    int ret = 0;
    if(dsc_init(dev, &cfg) != DSC_OK || dsc_start(dev) != DSC_OK){
        fprintf(stderr, "init or start failed\n");
        ret = 1; goto out;
    }

    printf("recording into %s: cam [%ld, %ld)%s, imu [%ld, %ld)\n", dir, cam_from, cam_to,
           stitch ? " stitched" : "", imu_from, imu_to);

    const uint64_t t0 = now_us();
    uint64_t cam_idx = 0, imu_idx = 0;

    /* Get data */
    /* 获取数据 */
    while(!stop){
        dsc_image_nv12_t f[2];
        while(get_frame(dev, stitch, f)){
            const uint64_t i = cam_idx++;
            if(keep_cam && i >= (uint64_t)cam_from && i < (uint64_t)cam_to){
                cam[ncam][0] = f[0];
                cam[ncam][1] = f[1];
                ncam++;
            } else {
                free(f[0].data), free(f[1].data);   /* the buffer is ours to free 缓冲区由我们释放 */
            }
        }
        dsc_imu_packet_t p;
        while(dsc_get_imu_packet(dev, &p) == DSC_OK){
            const uint64_t i = imu_idx++;
            if(keep_imu && i >= (uint64_t)imu_from && i < (uint64_t)imu_to)imu[nimu++] = p;
        }

        report(t0, keep_cam, ncam, cam_cap, keep_imu, nimu, imu_cap);
        usleep(1000);
    }

    /* release the device before the write, so it is not held while the files go out */
    /* 在写文件之前先释放设备，这样写文件期间不会一直占用设备 */
    dsc_stop(dev);
    dsc_deinit(dev);
    dsc_destroy(dev);
    dev = NULL;   /* out: only tears the device down when init/start failed 仅在 init/start 失败时才销毁设备 */
    printf("\nrecorded for %.3fs: kept %zu camera frames and %zu imu packets\n",
           (double)(now_us() - t0) / 1e6, ncam, nimu);

    /* the kept items are a contiguous run, so entry i is <range>_from + i */
    /* 保留的数据项是连续的一段，因此第 i 项对应 <range>_from + i */
    for(size_t i = 0; i < ncam; i++)
        for(int e = 0; e < (stitch ? 1 : 2); e++){
            save_frame(dir, cam_from + (long)i, stitch ? "" : (e ? "_R" : "_L"), &cam[i][e]);
            free(cam[i][e].data);
        }
    if(keep_cam)printf("wrote %s/cam: %zu frames\n", dir, ncam);
    if(keep_imu)save_imu(dir, imu_from, imu, nimu);

out:
    if(dev != NULL){
        dsc_deinit(dev);
        dsc_destroy(dev);
    }
    free(cam);
    free(imu);
    return ret;
}

/* pull one frame pair; when stitching there is one frame in f[0] and f[1] stays empty */
/* 取一帧图像对；拼接时只有 f[0] 中有一帧，f[1] 保持为空 */
static bool get_frame(dsc_device_t *dev, bool stitch, dsc_image_nv12_t f[2])
{
    f[1] = (dsc_image_nv12_t){0};
    return stitch ? dsc_get_stereo_nv12_frame(dev, &f[0]) == DSC_OK
                  : dsc_get_nv12_frame(dev, &f[0], &f[1]) == DSC_OK;
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}

static void mkdir_p(const char *path)
{
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s", path);

    for(char *p = tmp + 1; *p != '\0'; p++){
        if(*p != '/')continue;
        *p = '\0';
        mkdir(tmp, 0777);
        *p = '/';
    }
    mkdir(tmp, 0777);
}

static void report(uint64_t t0, bool keep_cam, size_t ncam, size_t cam_cap,
                   bool keep_imu, size_t nimu, size_t imu_cap)
{
    static uint64_t next_report = 0;
    const uint64_t now = now_us();
    if(now < next_report)return;
    next_report += REPORT_US;
    if(next_report <= now)next_report = now + REPORT_US;

    printf("[%6.1fs] ", (double)(now - t0) / 1e6);
    if(keep_cam)printf("cam %zu/%zu   ", ncam, cam_cap);
    if(keep_imu)printf("imu %zu/%zu", nimu, imu_cap);
    printf("\n");
}

static void save_frame(const char *dir, long index, const char *suffix, const dsc_image_nv12_t *image)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/cam/%06ld%s.nv12", dir, index, suffix);

    if(image->data == NULL)return;

    FILE *f = fopen(path, "wb");
    if(f == NULL){
        fprintf(stderr, "cannot open %s\n", path);
        return;
    }
    fwrite(image->data, 1, (size_t)image->width * image->height * 3 / 2, f);
    fclose(f);
}

static void save_imu(const char *dir, long from, const dsc_imu_packet_t *imu, size_t n)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/imu.csv", dir);

    FILE *f = fopen(path, "w");
    if(f == NULL){
        fprintf(stderr, "cannot open %s\n", path);
        return;
    }

    fprintf(f, "index,timestamp_ns,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,temp,is_fsync\n");
    for(size_t i = 0; i < n; i++)
        fprintf(f, "%ld,%llu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%d\n", from + (long)i,
                (unsigned long long)imu[i].timestamp_ns, imu[i].accel[0], imu[i].accel[1],
                imu[i].accel[2], imu[i].gyro[0], imu[i].gyro[1], imu[i].gyro[2],
                imu[i].temp, imu[i].is_fsync);
    fclose(f);

    printf("wrote %s: %zu packets\n", path, n);
}
