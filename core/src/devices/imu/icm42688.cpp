/**
 * @file icm42688.cpp
 * @brief ICM-42688-P implementation.
 * @brief ICM-42688-P 的实现。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/imu/imu.hpp"

#include <unistd.h>

#include <vector>

namespace dsc {
namespace imu {
namespace {

// ---------------- bank 0 寄存器组 0 ----------------
constexpr uint8_t kBankSel      = 0x76;
constexpr uint8_t kDeviceConfig = 0x11;
constexpr uint8_t kFifoConfig   = 0x16;
constexpr uint8_t kFifoCount    = 0x2E;
constexpr uint8_t kIntStatus    = 0x2D;
constexpr uint8_t kFifoData     = 0x30;
constexpr uint8_t kIntfConfig1  = 0x4D;
constexpr uint8_t kPwrMgmt0     = 0x4E;
constexpr uint8_t kGyroConfig0  = 0x4F;
constexpr uint8_t kAccelConfig0 = 0x50;
constexpr uint8_t kGyroConfig1  = 0x51;
constexpr uint8_t kGyroAccelCfg = 0x52;
constexpr uint8_t kAccelConfig1 = 0x53;
constexpr uint8_t kGyroUiOrderShift  = 2;  // GYRO_CONFIG1[3:2] GYRO_CONFIG1 的 [3:2] 位
constexpr uint8_t kAccelUiOrderShift = 3;  // ACCEL_CONFIG1[4:3] ACCEL_CONFIG1 的 [4:3] 位
constexpr uint8_t kUiOrderMask       = 0x03;
constexpr uint8_t kUiOrderSecond    = 0x01;
constexpr uint8_t kTmstConfig   = 0x54;
constexpr uint8_t kFifoConfig1  = 0x5F;
constexpr uint8_t kFsyncConfig  = 0x62;
constexpr uint8_t kIntConfig1   = 0x64;
constexpr uint8_t kWhoAmI       = 0x75;

// ---------------- bank 1 寄存器组 1 ----------------
constexpr uint8_t kIntfConfig5  = 0x7B;   // PIN9 muxed as FSYNC PIN9 复用为 FSYNC

constexpr uint8_t kPacketSize   = 16;
constexpr uint8_t kFsyncMask    = 0x0C;   // packet header bits[3:2] 数据包头 bits[3:2]
constexpr uint8_t kFsyncVal     = 0x0C;   // 11 = FSYNC delta; 10 = normal timestamp 11 = FSYNC 增量；10 = 普通时间戳

// delta_time tick conversion: us = raw * 32 / 30
// delta_time 的 tick 换算：us = raw * 32 / 30
constexpr uint32_t kTickNum = 32;
constexpr uint32_t kTickDen = 30;

// {physical value, register bits}. Keyed by physical value for easy line-by-line checking against the datasheet.
// {物理值, 寄存器位}。以物理值为键，便于逐行与数据手册核对。
struct Code {
    uint16_t value;
    uint8_t  bits;
};

// GYRO/ACCEL_CONFIG0 low 4 bits
// GYRO/ACCEL_CONFIG0 的低 4 位
constexpr Code kOdr[] = {
    {200, 0x07},
    {500, 0x0F},
    {0, 0},
};

// Full-scale encoding is descending: 000 = +/-16g, 011 = +/-2g
// 满量程编码是递减的：000 = +/-16g，011 = +/-2g
constexpr Code kAccelFsr[] = {
    {16, 0x00},
    {8,  0x20},
    {4,  0x40},
    {2,  0x60},
    {0, 0},
};

// Also descending: 000 = +/-2000dps, 011 = +/-250dps
// 同样是递减的：000 = +/-2000dps，011 = +/-250dps
constexpr Code kGyroFsr[] = {
    {2000, 0x00},
    {1000, 0x20},
    {500,  0x40},
    {250,  0x60},
    {0, 0},
};

// Number of UI filter settings; see ModelDesc::info for setting-to-bandwidth mapping
// UI 滤波器设置的数量；设置值到带宽的映射见 ModelDesc::info
constexpr uint8_t kBwSelCount = 16;

bool lookup(const Code *tab, uint32_t value, uint8_t *bits)
{
    for(const Code *c = tab; c->value != 0; c++){
        if(c->value == value){
            *bits = c->bits;
            return true;
        }
    }
    return false;
}

bool select_bank(base::I2cDevice &bus, uint8_t bank)
{
    return bus.update(kBankSel, 0x07, bank) == Status::Ok;
}

// UI-filter group delay at DC in 0.1 ms units (DS-000347 rev. 1.2, pp. 31–33).
// Each row uses the raw order and ODR codes. Columns use the raw bandwidth code;
// 8–13 are reserved. At 2 kHz and above, the datasheet gives one delay per ODR.
// UI 滤波器在直流处的群延迟，单位为 0.1 ms（DS-000347 rev. 1.2，第 31–33 页）。
// 每行使用原始的阶数与 ODR 编码。各列使用原始带宽编码；8–13 为保留值。在 2 kHz 及以上，
// 数据手册对每个 ODR 只给出一个延迟值。
struct DelayRow {
    uint8_t order;
    uint8_t odr;
    uint16_t tenths_ms[kBwSelCount];
};

constexpr DelayRow kGroupDelay[] = {
    //          BW:  0   1   2   3    4    5    6    7    8–13              14  15  带宽编码（列）
    // First order
    // 一阶
    {0,  1, {       1,  1,  1,  1,   1,   1,   1,   1,   0, 0, 0, 0, 0, 0,  1,  1}},
    {0,  2, {       1,  1,  1,  1,   1,   1,   1,   1,   0, 0, 0, 0, 0, 0,  1,  1}},
    {0,  3, {       2,  2,  2,  2,   2,   2,   2,   2,   0, 0, 0, 0, 0, 0,  2,  2}},
    {0,  4, {       4,  4,  4,  4,   4,   4,   4,   4,   0, 0, 0, 0, 0, 0,  4,  4}},
    {0,  5, {       8,  8,  8,  8,   8,   8,   8,   8,   0, 0, 0, 0, 0, 0,  8,  8}},
    {0,  6, {       6, 18, 20, 28,  31,  41,  47,  81,   0, 0, 0, 0, 0, 0, 15,  2}},
    {0, 15, {      11, 36, 40, 55,  61,  81,  93, 162,   0, 0, 0, 0, 0, 0, 30,  4}},
    {0,  7, {      27, 44, 50, 68,  76, 102, 117, 203,   0, 0, 0, 0, 0, 0, 38, 10}},
    {0,  8, {      53, 44, 50, 68,  76, 102, 117, 203,   0, 0, 0, 0, 0, 0, 38, 19}},
    {0,  9, {     105, 44, 50, 68,  76, 102, 117, 203,   0, 0, 0, 0, 0, 0, 38, 38}},
    {0, 10, {     210, 44, 50, 68,  76, 102, 117, 203,   0, 0, 0, 0, 0, 0, 38, 75}},
    {0, 11, {     210, 44, 50, 68,  76, 102, 117, 203,   0, 0, 0, 0, 0, 0, 38, 75}},
    // Second order
    // 二阶
    {1,  1, {       1,  1,  1,  1,   1,   1,   1,   1,   0, 0, 0, 0, 0, 0,  1,  1}},
    {1,  2, {       1,  1,  1,  1,   1,   1,   1,   1,   0, 0, 0, 0, 0, 0,  1,  1}},
    {1,  3, {       2,  2,  2,  2,   2,   2,   2,   2,   0, 0, 0, 0, 0, 0,  2,  2}},
    {1,  4, {       4,  4,  4,  4,   4,   4,   4,   4,   0, 0, 0, 0, 0, 0,  4,  4}},
    {1,  5, {       8,  8,  8,  8,   8,   8,   8,   8,   0, 0, 0, 0, 0, 0,  8,  8}},
    {1,  6, {       7, 21, 24, 32,  37,  52,  61, 120,   0, 0, 0, 0, 0, 0, 15,  2}},
    {1, 15, {      13, 41, 47, 64,  73, 104, 122, 240,   0, 0, 0, 0, 0, 0, 30,  4}},
    {1,  7, {      33, 51, 58, 80,  91, 129, 153, 300,   0, 0, 0, 0, 0, 0, 38, 10}},
    {1,  8, {      65, 51, 58, 80,  91, 129, 153, 300,   0, 0, 0, 0, 0, 0, 38, 19}},
    {1,  9, {     129, 51, 58, 80,  91, 129, 153, 300,   0, 0, 0, 0, 0, 0, 38, 38}},
    {1, 10, {     257, 51, 58, 80,  91, 129, 153, 300,   0, 0, 0, 0, 0, 0, 38, 75}},
    {1, 11, {     257, 51, 58, 80,  91, 129, 153, 300,   0, 0, 0, 0, 0, 0, 38, 75}},
    // Third order
    // 三阶
    {2,  1, {       1,  1,  1,  1,   1,   1,   1,   1,   0, 0, 0, 0, 0, 0,  1,  1}},
    {2,  2, {       1,  1,  1,  1,   1,   1,   1,   1,   0, 0, 0, 0, 0, 0,  1,  1}},
    {2,  3, {       2,  2,  2,  2,   2,   2,   2,   2,   0, 0, 0, 0, 0, 0,  2,  2}},
    {2,  4, {       4,  4,  4,  4,   4,   4,   4,   4,   0, 0, 0, 0, 0, 0,  4,  4}},
    {2,  5, {       8,  8,  8,  8,   8,   8,   8,   8,   0, 0, 0, 0, 0, 0,  8,  8}},
    {2,  6, {       8, 23, 27, 40,  46,  66,  82, 141,   0, 0, 0, 0, 0, 0, 15,  2}},
    {2, 15, {      16, 46, 54, 79,  92, 132, 163, 281,   0, 0, 0, 0, 0, 0, 30,  4}},
    {2,  7, {      40, 58, 68, 98, 114, 165, 204, 352,   0, 0, 0, 0, 0, 0, 38, 10}},
    {2,  8, {      80, 58, 68, 98, 114, 165, 204, 352,   0, 0, 0, 0, 0, 0, 38, 19}},
    {2,  9, {     159, 58, 68, 98, 114, 165, 204, 352,   0, 0, 0, 0, 0, 0, 38, 38}},
    {2, 10, {     318, 58, 68, 98, 114, 165, 204, 352,   0, 0, 0, 0, 0, 0, 38, 75}},
    {2, 11, {     318, 58, 68, 98, 114, 165, 204, 352,   0, 0, 0, 0, 0, 0, 38, 75}},
};

Status group_delay_ns(base::I2cDevice &bus, uint64_t *delay_ns)
{
    if(!delay_ns)return Status::ParamError;
    if(!select_bank(bus, 0))return Status::HwError;

    uint8_t gyro_config0 = 0, accel_config0 = 0, gyro_config1 = 0, accel_config1 = 0;
    uint8_t filt_bw = 0;
    if(bus.read(kGyroConfig0, &gyro_config0) != Status::Ok ||
       bus.read(kAccelConfig0, &accel_config0) != Status::Ok ||
       bus.read(kGyroConfig1, &gyro_config1) != Status::Ok ||
       bus.read(kAccelConfig1, &accel_config1) != Status::Ok ||
       bus.read(kGyroAccelCfg, &filt_bw) != Status::Ok)
        return Status::HwError;

    const uint8_t gyro_order  = (gyro_config1 >> kGyroUiOrderShift) & kUiOrderMask;
    const uint8_t accel_order = (accel_config1 >> kAccelUiOrderShift) & kUiOrderMask;
    const uint8_t gyro_odr    = gyro_config0 & 0x0F;
    const uint8_t accel_odr   = accel_config0 & 0x0F;
    const uint8_t gyro_bw     = filt_bw & 0x0F;
    const uint8_t accel_bw    = filt_bw >> 4;

    uint16_t gyro_delay = 0, accel_delay = 0;
    for(const DelayRow &row : kGroupDelay){
        if(row.order == gyro_order && row.odr == gyro_odr)
            gyro_delay = row.tenths_ms[gyro_bw];
        if(row.order == accel_order && row.odr == accel_odr)
            accel_delay = row.tenths_ms[accel_bw];
    }
    if(gyro_delay == 0 || gyro_delay != accel_delay)return Status::Unsupported;
    *delay_ns = static_cast<uint64_t>(gyro_delay) * 100000ULL;
    return Status::Ok;
}

Status init(base::I2cDevice &bus, const ImuConfig &cfg)
{
    // Strict validation: unsupported values (including unset 0) always error out; no defaults, no rounding to nearest
    // 严格校验：不受支持的值（包括未设置时的 0）一律报错；既不给默认值，也不做就近取整
    uint8_t odr_bits, afs_bits, gfs_bits;
    if(!lookup(kOdr, cfg.odr_hz, &odr_bits))                     return Status::Unsupported;
    if(!lookup(kAccelFsr, cfg.accel_fsr_g, &afs_bits))           return Status::Unsupported;
    if(!lookup(kGyroFsr, cfg.gyro_fsr_dps, &gfs_bits))           return Status::Unsupported;
    if(cfg.bw_sel >= kBwSelCount)                                return Status::Unsupported;

    if(!select_bank(bus, 0))
        return Status::HwError;

    // Soft reset
    // 软复位
    if(bus.update(kDeviceConfig, 0x01, 0x01) != Status::Ok)
        return Status::HwError;
    usleep(1500);

    // PIN9 muxed as FSYNC; this register is in bank 1
    // PIN9 复用为 FSYNC；该寄存器位于 bank 1
    if(!select_bank(bus, 1))
        return Status::HwError;
    if(bus.update(kIntfConfig5, 0x06, 0x02) != Status::Ok)
        return Status::HwError;
    if(!select_bank(bus, 0))
        return Status::HwError;

    if(bus.write(kIntfConfig1, 0x91) != Status::Ok)     // RC oscillator RC 振荡器
        return Status::HwError;
    if(bus.write(kTmstConfig, 0x23) != Status::Ok)      // timestamp + delta mode 时间戳 + delta 模式
        return Status::HwError;
    if(bus.write(kFsyncConfig, 0x10) != Status::Ok)     // FSYNC rising edge FSYNC 上升沿
        return Status::HwError;
    if(bus.write(kFifoConfig1, 0x4F) != Status::Ok)     // TMST_FSYNC+TEMP+GYRO+ACCEL 时间戳+温度+陀螺仪+加速度计
        return Status::HwError;
    if(bus.write(kIntConfig1, 0x00) != Status::Ok)      // async reset off 关闭异步复位
        return Status::HwError;

    // Keep both UI filters at the factory-default second order. The two order fields
    // occupy different bits; preserve the reserved, temperature and DEC2 fields.
    // 两个 UI 滤波器都保持出厂默认的二阶。两个阶数字段占用不同的位；保留位、温度字段
    // 与 DEC2 字段保持不变。
    if(bus.update(kGyroConfig1, kUiOrderMask << kGyroUiOrderShift,
                  kUiOrderSecond << kGyroUiOrderShift) != Status::Ok ||
       bus.update(kAccelConfig1, kUiOrderMask << kAccelUiOrderShift,
                  kUiOrderSecond << kAccelUiOrderShift) != Status::Ok)
        return Status::HwError;

    // Power on
    // 上电
    if(bus.update(kPwrMgmt0, 0x3F, 0x0F) != Status::Ok)
        return Status::HwError;
    usleep(50000);

    // FIFO to bypass first, then configure sampling parameters
    // 先把 FIFO 置为直通（bypass），再配置采样参数
    if(bus.update(kFifoConfig, 0xC0, 0x00) != Status::Ok)
        return Status::HwError;
    if(bus.update(kGyroConfig0, 0xEF, static_cast<uint8_t>(gfs_bits | odr_bits))
            != Status::Ok)
        return Status::HwError;
    if(bus.update(kAccelConfig0, 0xEF, static_cast<uint8_t>(afs_bits | odr_bits))
            != Status::Ok)
        return Status::HwError;
    if(bus.write(kGyroAccelCfg, static_cast<uint8_t>((cfg.bw_sel << 4) | cfg.bw_sel))
            != Status::Ok)
        return Status::HwError;

    return Status::Ok;
}

Status start(base::I2cDevice &bus)
{
    uint8_t int_status;
    if(!select_bank(bus, 0))return Status::HwError;
    (void)bus.readBurst(kIntStatus, &int_status, 1);   // clear stale FIFO_FULL flag 清除残留的 FIFO_FULL 标志
    if(bus.update(kFifoConfig, 0xC0, 0x40) != Status::Ok)   // stream-to-FIFO 开启流向 FIFO
        return Status::HwError;
    return Status::Ok;
}

void stop(base::I2cDevice &bus)
{
    bus.update(kFifoConfig, 0xC0, 0x00);
}

Status read(base::I2cDevice &bus, ImuHwFifo16Packet *out, std::size_t cap, std::size_t *n_out)
{
    if(!out || !cap || !n_out)
        return Status::ParamError;
    *n_out = 0;

    uint8_t cbuf[2];
    if(bus.readBurst(kFifoCount, cbuf, 2) != Status::Ok)
        return Status::HwError;

    const uint16_t bytes = static_cast<uint16_t>((cbuf[0] << 8) | cbuf[1]);
    uint16_t pkts = static_cast<uint16_t>(bytes / kPacketSize);
    if(!pkts)
        return Status::Ok;
    if(pkts > cap)
        pkts = static_cast<uint16_t>(cap);

    std::vector<uint8_t> raw(static_cast<std::size_t>(pkts) * kPacketSize);
    if(bus.readBurst(kFifoData, raw.data(), static_cast<uint32_t>(raw.size()))
            != Status::Ok)
        return Status::HwError;

    for(uint16_t i = 0; i < pkts; i++){
        const uint8_t *p = raw.data() + i * kPacketSize;
        ImuHwFifo16Packet &s = out[i];

        s.is_fsync = (p[0] & kFsyncMask) == kFsyncVal;
        s.accel[0] = static_cast<int16_t>((p[1]  << 8) | p[2]);
        s.accel[1] = static_cast<int16_t>((p[3]  << 8) | p[4]);
        s.accel[2] = static_cast<int16_t>((p[5]  << 8) | p[6]);
        s.gyro[0]  = static_cast<int16_t>((p[7]  << 8) | p[8]);
        s.gyro[1]  = static_cast<int16_t>((p[9]  << 8) | p[10]);
        s.gyro[2]  = static_cast<int16_t>((p[11] << 8) | p[12]);
        s.temp     = static_cast<int8_t>(p[13]);

        // When the header is not FSYNC, this field is a normal timestamp, not an edge offset; set 0 to avoid misuse
        // 当头部不是 FSYNC 时，该字段是普通时间戳，而不是边沿偏移；置 0 以避免误用
        if(s.is_fsync){
            const uint32_t dt = static_cast<uint32_t>((p[14] << 8) | p[15]);
            s.delta_time_us = dt * kTickNum / kTickDen;
        }
        else{
            s.delta_time_us = 0;
        }
    }

    *n_out = pkts;
    return Status::Ok;
}

bool full(base::I2cDevice &bus)
{
    uint8_t val = 0;
    if(!select_bank(bus, 0))return false;
    if(bus.readBurst(kIntStatus, &val, 1) != Status::Ok)return false;
    return (val & 0x02) != 0;   // INT_STATUS bit1 = FIFO_FULL (auto-cleared on read) INT_STATUS bit1 = FIFO_FULL（读取时自动清除）
}

void deinit(base::I2cDevice &bus)
{
    // Graceful shutdown: back to bank0 -> stop FIFO -> power down the sensor
    // 优雅关闭：切回 bank0 -> 停止 FIFO -> 给传感器断电
    select_bank(bus, 0);
    bus.update(kFifoConfig, 0xC0, 0x00);
    bus.update(kPwrMgmt0, 0x3F, 0x00);
}

} // namespace

// The definition must use extern: a namespace-scope const has internal linkage by default in C++
// 定义必须使用 extern：在 C++ 中，命名空间作用域的 const 默认具有内部链接
extern const ModelDesc kIcm42688 = {
    kWhoAmI,
    0x47,
    "ICM-42688-P",
    "- odr:       200 | 500 Hz\n"
    "- accel fsr: 2 | 4 | 8 | 16 g\n"
    "- gyro fsr:  250 | 500 | 1000 | 2000 dps\n"
    "- bw_sel: 0..F\n"
    "  - 0 BW=ODR/2\n"
    "  - 1 BW=max(400Hz, ODR)/4\n"
    "  - 2 BW=max(400Hz, ODR)/5\n"
    "  - 3 BW=max(400Hz, ODR)/8\n"
    "  - 4 BW=max(400Hz, ODR)/10\n"
    "  - 5 BW=max(400Hz, ODR)/16\n"
    "  - 6 BW=max(400Hz, ODR)/20\n"
    "  - 7 BW=max(400Hz, ODR)/40\n"
    "  - 8 to 13: Reserved\n"
    "  - 14 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(400Hz, ODR)\n"
    "  - 15 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)\n"
    "- noise density: accel 70 ug/rtHz, gyro 0.0028 dps/rtHz\n",
    init,
    start,
    stop,
    read,
    full,
    deinit,
    group_delay_ns,
};

DSC_IMU_REGISTER_MODEL(kIcm42688);

} // namespace imu
} // namespace dsc
