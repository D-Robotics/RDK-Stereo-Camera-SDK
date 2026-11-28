/**
 * @file dsc-detect-camera.cpp
 * @brief Detect SC132GS sensors on I2C.
 * @brief 在 I2C 总线上探测 SC132GS 传感器。
 *
 * Usage: dsc detect camera [-b <bus...>]
 * 用法：dsc detect camera [-b <bus...>]
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/i2c/i2c.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

using namespace dsc;

// Keep the chip ID register and value in sync with the pipeline probe.
// 芯片 ID 寄存器和期望值须与流水线探测逻辑一致。
static constexpr uint16_t kChipIdReg = 0x3107;
static constexpr uint16_t kChipId    = 0x0132;

static bool parse_int(const char *s, int *out)
{
    char *end = nullptr;
    long v = strtol(s, &end, 0);   // base 0: decimal, 0x-hex, 0-octal 进制 0：十进制、0x 十六进制、0 八进制
    if(end == s || *end != '\0')return false;
    *out = (int)v;
    return true;
}

// Exit 0 means the scan completed, not that a sensor was found.
// 返回 0 只表示扫描完成，不代表找到了传感器。
int main(int argc, char **argv)
{
    std::vector<int> buses;
    const std::vector<int> addrs = {0x30, 0x31, 0x32, 0x33};
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

    printf("CAMERA scan: buses");
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
            base::I2cDevice d((uint8_t)b, (uint8_t)a);
            uint16_t id = 0;
            const bool ok = d && (d.read16(kChipIdReg, &id) == Status::Ok);
            if(ok && id == kChipId){
                printf(" %3d  0x%02x   chip-id 0x%04x\n", b, a, id);
                n_found++;
            } else if(ok){
                printf(" %3d  0x%02x   unknown device (id 0x%04x)\n", b, a, id);
                n_unknown++;
            } else {
                printf(" %3d  0x%02x   no device\n", b, a);
                n_empty++;
            }
        }
    }
    printf("found %d, unknown %d, empty %d, bus error %d\n", n_found, n_unknown, n_empty, n_buserr);
    return 0;
}
