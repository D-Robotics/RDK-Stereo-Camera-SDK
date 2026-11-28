/**
 * @file dsc-detect-imu.cpp
 * @brief Detect registered IMU models at I2C address 0x68.
 * @brief 在 I2C 地址 0x68 探测已注册的 IMU 型号。
 *
 * Usage: dsc detect imu [-b <bus...>]
 * 用法：dsc detect imu [-b <bus...>]
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/i2c/i2c.hpp"
#include "devices/imu/imu.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

using namespace dsc;

static bool parse_int(const char *s, int *out)
{
    char *end = nullptr;
    long v = strtol(s, &end, 0);   // base 0: decimal, 0x-hex, 0-octal 进制 0：十进制、0x 十六进制、0 八进制
    if(end == s || *end != '\0')return false;
    *out = (int)v;
    return true;
}

static void print_block(const char *indent, const char *text)
{
    if(!text)return;
    for(const char *p = text; *p; ){
        const char *nl = strchr(p, '\n');
        int len = nl ? (int)(nl - p) : (int)strlen(p);
        printf("%s%.*s\n", indent, len, p);
        if(!nl)break;
        p = nl + 1;
    }
}

// Exit 0 means the scan completed, not that an IMU was found.
// 返回 0 只表示扫描完成，不代表找到了 IMU。
int main(int argc, char **argv)
{
    std::vector<int> buses;
    const std::vector<int> addrs = {0x68};
    for(int i = 1; i < argc; i++){
        if(strcmp(argv[i], "-b")){
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            return 1;
        }
        while(i + 1 < argc && argv[i+1][0] != '-'){
            int v;
            if(!parse_int(argv[++i], &v)){ fprintf(stderr, "bad number: %s\n", argv[i]); return 1; }
            buses.push_back(v);
        }
    }
    if(buses.empty())buses = {1, 2, 4, 5, 6};

    printf("IMU scan: buses");
    for(int b : buses)printf(" %d", b);
    printf(", addrs");
    for(int a : addrs)printf(" 0x%02x", a);
    printf("\n bus  addr    result\n ---  -----   ----------------\n");

    int n_found = 0, n_unknown = 0, n_empty = 0, n_buserr = 0;
    for(int b : buses){
        base::I2cDevice probe((uint8_t)b, (uint8_t)addrs.front());
        if(!probe){
            printf(" %3d  --      bus open failed\n", b);
            n_buserr++;
            continue;
        }
        for(int a : addrs){
            imu::Imu imu((uint8_t)b, (uint8_t)a);
            if(imu){
                printf(" %3d  0x%02x   %s\n", b, a, imu.name());
                print_block("                ", imu.info());
                n_found++;
            } else {
                // A successful fallback read shows a response, not an IMU identification.
                // 兜底读取成功只说明有应答，不能确认设备是 IMU。
                base::I2cDevice d((uint8_t)b, (uint8_t)a);
                uint8_t v;
                if(d && d.read(0x00, &v) == Status::Ok){
                    printf(" %3d  0x%02x   unknown device\n", b, a);
                    n_unknown++;
                } else {
                    printf(" %3d  0x%02x   no device\n", b, a);
                    n_empty++;
                }
            }
        }
    }
    printf("found %d, unknown %d, empty %d, bus error %d\n", n_found, n_unknown, n_empty, n_buserr);
    return 0;
}
