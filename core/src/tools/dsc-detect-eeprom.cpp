/**
 * @file dsc-detect-eeprom.cpp
 * @brief Detect calibration EEPROMs at I2C address 0x50.
 * @brief 在 I2C 地址 0x50 探测标定 EEPROM。
 *
 * Usage: dsc detect eeprom [-b <bus...>]
 * 用法：dsc detect eeprom [-b <bus...>]
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/i2c/i2c.hpp"
#include "devices/eeprom/eeprom.hpp"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

using namespace dsc;

namespace {

// Keep the header layout and checksum in sync with union_stereo_imu_fisheye_v1p2r0n4.cpp.
// 标识区布局和校验算法须与 union_stereo_imu_fisheye_v1p2r0n4.cpp 一致。
constexpr uint16_t kIdAreaSize = 16;
constexpr uint8_t kChecksumOff = 15;
constexpr uint8_t kChecksumLen = 14;   // Byte 14 is excluded. 校验不包含第 14 号字节。
constexpr uint8_t kVendorLen   = 8;

static uint8_t id_area_checksum(const uint8_t *raw)
{
    uint16_t sum = 0;
    for(uint8_t i = 0; i < kChecksumLen; i++)sum = (uint16_t)(sum + raw[i]);
    return (uint8_t)((sum % 255) + 1);
}


static bool id_area_ok(const uint8_t *raw)
{
    return id_area_checksum(raw) == raw[kChecksumOff];
}

// The vendor field may fill all 8 bytes without a NUL terminator.
// 厂商字段可能占满 8 字节，不带 NUL 结束符。
static void id_area_vendor(const uint8_t *raw, char *out /* [kVendorLen + 1] */)
{
    for(uint8_t i = 0; i < kVendorLen; i++)out[i] = (char)raw[i];
    out[kVendorLen] = '\0';
}

static void miss_text(const uint8_t *raw, char *out, size_t cap)
{
    char vendor[kVendorLen + 1];
    id_area_vendor(raw, vendor);
    snprintf(out, cap, "%.8s unknown(0x%02x,0x%02x) V%u.%u Rotate-%u-deg %u-Distortion-parameters",
             vendor, raw[8], raw[9], raw[10], raw[11], raw[12], raw[13]);
}

} // namespace

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

// Exit 0 means the scan completed, not that an EEPROM was found.
// 返回 0 只表示扫描完成，不代表找到了 EEPROM。
int main(int argc, char **argv)
{
    std::vector<int> buses;
    const std::vector<int> addrs = {0x50};
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

    printf("EEPROM scan: buses");
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
            eeprom::Eeprom ep((uint8_t)b, (uint8_t)a);
            if(ep){
                printf(" %3d  0x%02x   %s\n", b, a, ep.name());
                print_block("                ", ep.info());
                n_found++;
            } else {
                // After the model probe misses, an ACK alone is not enough: require a valid header checksum.
                // 未匹配到已注册型号时，仅有应答还不够，标识区的校验和也必须通过。
                base::I2cDevice d((uint8_t)b, (uint8_t)a);
                uint8_t v;
                if(!d) {
                    printf(" %3d  0x%02x   no device\n", b, a);
                    n_empty++;
                } else if(d.read(0x00, &v) != Status::Ok) {
                    printf(" %3d  0x%02x   no device\n", b, a);
                    n_empty++;
                } else {
                    uint8_t raw[kIdAreaSize];
                    char text[128];
                    if(d.readBurst16(0x0000, raw, kIdAreaSize) == Status::Ok
                       && id_area_ok(raw)) {
                        miss_text(raw, text, sizeof(text));
                        printf(" %3d  0x%02x   %s\n", b, a, text);
                        n_unknown++;
                    } else {
                        printf(" %3d  0x%02x   no device\n", b, a);
                        n_empty++;
                    }
                }
            }
        }
    }
    printf("found %d, unknown %d, empty %d, bus error %d\n", n_found, n_unknown, n_empty, n_buserr);
    return 0;
}
