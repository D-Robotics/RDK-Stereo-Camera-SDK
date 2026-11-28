/**
 * @file icm42607.cpp
 * @brief ICM-42607-C implementation (the part fitted to A1P).
 * @brief ICM-42607-C 的实现（A1P 上装配的型号）。
 *
 * The register map differs from the ICM-42688-P one in icm42688.cpp, so the two models do
 * not share code.  Two differences matter here:
 * 本型号的寄存器映射与 icm42688.cpp 中 ICM-42688-P 的不同，因此两个型号不共享代码。
 * 这里有两处差异值得注意：
 *
 *   - the configuration registers live at 0x1F..0x28 instead of 0x4E..0x5F, the FIFO
 *     count sits at 0x3D and the data port at 0x3F;
 *   - the FIFO enable bits (accelerometer, gyroscope, F-SYNC timestamp) are not in bank 0
 *     at all but in MREG1, which is reached indirectly through BLK_SEL_W / MADDR_W / M_W
 *     (0x79/0x7A/0x7B).  Without writing MREG1 FIFO_CONFIG5 no packet ever reaches the
 *     FIFO, which reads back as "count 0" while the data registers keep updating.
 *   - 配置寄存器位于 0x1F..0x28，而不是 0x4E..0x5F，FIFO 计数位于 0x3D，数据端口位于 0x3F；
 *   - FIFO 使能位（加速度计、陀螺仪、F-SYNC 时间戳）完全不在 bank 0 中，而在 MREG1 中，
 *     需通过 BLK_SEL_W / MADDR_W / M_W（0x79/0x7A/0x7B）间接访问。如果不写 MREG1 的
 *     FIFO_CONFIG5，任何数据包都不会进入 FIFO，此时读回的计数为 0，而数据寄存器仍在更新。
 *
 * Per MREG1 FDATA, the FIFO record is Packet 3 (datasheet section "13 FIFO"): one header
 * byte, accelerometer XYZ, gyroscope XYZ, a two byte timestamp and one byte of
 * temperature, 16 bytes in total.
 * 按 MREG1 的 FDATA 说明，FIFO 记录为 Packet 3（数据手册“13 FIFO”一节）：一个头部字节、
 * 加速度计 XYZ、陀螺仪 XYZ、两字节时间戳和一个字节温度，共 16 字节。
 */

#include <cstring>
#include <vector>
#include <unistd.h>

#include "base/i2c/i2c.hpp"
#include "devices/imu/imu.hpp"

namespace dsc {
namespace imu {
namespace {

// ---------------- bank 0 寄存器组 0 ----------------
constexpr uint8_t kSignalPathReset = 0x02;   // bit4 = software reset, bit2 = FIFO flush bit4 = 软件复位，bit2 = FIFO 清空
constexpr uint8_t kPwrMgmt0        = 0x1F;
constexpr uint8_t kGyroConfig0     = 0x20;   // full scale bits[6:5], ODR bits[3:0] 满量程 bits[6:5]，ODR bits[3:0]
constexpr uint8_t kAccelConfig0    = 0x21;
constexpr uint8_t kGyroConfig1     = 0x23;   // UI filter bandwidth bits[2:0] UI 滤波器带宽 bits[2:0]
constexpr uint8_t kAccelConfig1    = 0x24;
constexpr uint8_t kFifoConfig1     = 0x28;   // bit1 = mode, bit0 = bypass bit1 = 模式，bit0 = 直通
constexpr uint8_t kIntfConfig0     = 0x35;   // bit6 = FIFO count in records bit6 = FIFO 计数按记录数
constexpr uint8_t kIntfConfig1     = 0x36;   // bits[1:0] = clock select bits[1:0] = 时钟选择
constexpr uint8_t kIntStatus       = 0x3A;   // bit1 = FIFO full bit1 = FIFO 满
constexpr uint8_t kFifoData        = 0x3F;
constexpr uint8_t kWhoAmI          = 0x75;

// Indirect access to the MREG banks
// 对 MREG 寄存器组的间接访问
constexpr uint8_t kBlkSelW = 0x79;
constexpr uint8_t kMAddrW  = 0x7A;
constexpr uint8_t kMW      = 0x7B;

// MREG1 registers, values from the vendor's ICM42607 configuration note
// MREG1 寄存器，取值来自厂商的 ICM42607 配置说明
constexpr uint8_t kSensorConfig3 = 0x06;   // 0x40: APEX off, FIFO extended to 2.25 KiB 0x40：关闭 APEX，FIFO 扩展到 2.25 KiB
constexpr uint8_t kFdrConfig     = 0x66;   // 0x00: no FIFO decimation 0x00：FIFO 不做抽取
constexpr uint8_t kTmstConfig1   = 0x00;   // 0x03: timestamp on, F-SYNC capture on, 1 us 0x03：开启时间戳，开启 F-SYNC 捕获，1 us
constexpr uint8_t kFifoConfig5   = 0x01;   // 0x37: accel+gyro+FSYNC time, partial read resume 0x37：加速度计+陀螺仪+FSYNC 时间，支持部分读取续读
constexpr uint8_t kFifoConfig6   = 0x02;   // 0x01: 0xFF when empty, no watermark RCOSC wake 0x01：空时返回 0xFF，水印不触发 RCOSC 唤醒
constexpr uint8_t kFsyncConfig   = 0x03;   // 0x10: temperature LSB as UI marker, rising edge 0x10：温度最低位作 UI 标记，上升沿

// FIFO count: the low byte latches both halves, so it has to be read first
// FIFO 计数：低字节会把两半一起锁存，因此必须先读低字节
constexpr uint8_t kFifoCountLow  = 0x3E;
constexpr uint8_t kFifoCountHigh = 0x3D;

// Identification: WHO_AM_I reports 0x61 on the ICM-42607-C.
// 识别：ICM-42607-C 的 WHO_AM_I 返回 0x61。
constexpr uint8_t kIdIcm42607C = 0x61;

constexpr uint8_t kPacketSize = 16;
constexpr uint8_t kFsyncMask  = 0x0C;   // packet header bits[3:2] 数据包头 bits[3:2]
constexpr uint8_t kFsyncVal   = 0x0C;   // 11 = F-SYNC delta; 10 = normal timestamp 11 = F-SYNC 增量；10 = 普通时间戳


// {physical value, register bits}. Keyed by physical value for easy line-by-line checking against the datasheet.
// {物理值, 寄存器位}。以物理值为键，便于逐行与数据手册核对。
struct Code {
    uint16_t value;
    uint8_t  bits;
};

// GYRO_CONFIG0 / ACCEL_CONFIG0 bits[3:0]
// GYRO_CONFIG0 / ACCEL_CONFIG0 的 bits[3:0]
constexpr Code kOdr[] = {
    {1600, 0x05},
    {800,  0x06},
    {400,  0x07},
    {200,  0x08},
    {100,  0x09},
    {50,   0x0A},
    {25,   0x0B},
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

// GYRO_CONFIG1 / ACCEL_CONFIG1 filter bandwidth codes: 0 = bypass, 1 = 180Hz ... 7 = 16Hz
// GYRO_CONFIG1 / ACCEL_CONFIG1 的滤波带宽编码：0 = 直通，1 = 180Hz ... 7 = 16Hz
constexpr uint8_t kBwSelCount = 8;

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

// Write one MREG1 register: select the bank, set the address, then the value.  The part
// needs 10 us between the address and the data write, and no other register access in
// between, so the selector is cleared afterwards.
// 写一个 MREG1 寄存器：选择寄存器组、设置地址，然后写值。该器件要求地址与数据写入之间
// 间隔 10 us，且其间不得有其他寄存器访问，因此写完后会清除选择器。
Status mreg1_write(base::I2cDevice &bus, uint8_t addr, uint8_t value)
{
    if(bus.write(kBlkSelW, 0x00) != Status::Ok)return Status::HwError;
    if(bus.write(kMAddrW, addr) != Status::Ok)return Status::HwError;
    usleep(20);
    if(bus.write(kMW, value) != Status::Ok)return Status::HwError;
    usleep(20);
    if(bus.write(kBlkSelW, 0x00) != Status::Ok)return Status::HwError;
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

    // Software reset; the bit clears itself once the reset has completed
    // 软件复位；复位完成后该位会自行清零
    if(bus.update(kSignalPathReset, 0x10, 0x10) != Status::Ok)
        return Status::HwError;
    usleep(2000);

    // Sample data and FIFO counts in big-endian order, FIFO counted in bytes
    // 采样数据与 FIFO 计数均为大端顺序，FIFO 计数以字节为单位
    if(bus.update(kIntfConfig0, 0x70, 0x30) != Status::Ok)
        return Status::HwError;

    // PLL when available, RC oscillator otherwise
    // 可用时使用 PLL，否则使用 RC 振荡器
    if(bus.update(kIntfConfig1, 0x03, 0x01) != Status::Ok)
        return Status::HwError;

    // RC oscillator first, then gyro and accel in low-noise mode with IDLE kept
    // 先开 RC 振荡器，再让陀螺仪与加速度计进入低噪声模式并保留 IDLE 位
    if(bus.write(kPwrMgmt0, 0x10) != Status::Ok)
        return Status::HwError;
    usleep(1000);
    if(bus.write(kPwrMgmt0, 0x1F) != Status::Ok)
        return Status::HwError;
    usleep(200000);

    // FIFO to bypass first, then configure sampling parameters
    // 先把 FIFO 置为直通（bypass），再配置采样参数
    if(bus.update(kFifoConfig1, 0x01, 0x01) != Status::Ok)
        return Status::HwError;
    if(bus.update(kGyroConfig0, 0x6F, static_cast<uint8_t>(gfs_bits | odr_bits))
            != Status::Ok)
        return Status::HwError;
    if(bus.update(kAccelConfig0, 0x6F, static_cast<uint8_t>(afs_bits | odr_bits))
            != Status::Ok)
        return Status::HwError;
    if(bus.update(kGyroConfig1, 0x07, cfg.bw_sel) != Status::Ok)
        return Status::HwError;
    if(bus.update(kAccelConfig1, 0x07, cfg.bw_sel) != Status::Ok)
        return Status::HwError;

    // Watermark unused: the host polls the count instead
    // 不使用水印：改由主机轮询计数
    if(bus.write(0x29, 0x00) != Status::Ok)
        return Status::HwError;
    if(bus.write(0x2A, 0x00) != Status::Ok)
        return Status::HwError;

    // MREG1: APEX off, no FIFO decimation, timestamp and the three FIFO fields on,
    // empty-FIFO marker, F-SYNC on the rising edge of the temperature UI marker
    // MREG1：关闭 APEX，FIFO 不做抽取，开启时间戳与三个 FIFO 字段，设置空 FIFO 标记，
    // F-SYNC 取温度 UI 标记的上升沿
    if(mreg1_write(bus, kSensorConfig3, 0x40) != Status::Ok)   return Status::HwError;
    if(mreg1_write(bus, kFdrConfig,     0x00) != Status::Ok)   return Status::HwError;
    if(mreg1_write(bus, kTmstConfig1,   0x03) != Status::Ok)   return Status::HwError;
    if(mreg1_write(bus, kFifoConfig5,   0x37) != Status::Ok)   return Status::HwError;
    if(mreg1_write(bus, kFifoConfig6,   0x01) != Status::Ok)   return Status::HwError;
    if(mreg1_write(bus, kFsyncConfig,   0x10) != Status::Ok)   return Status::HwError;

    return Status::Ok;
}

Status start(base::I2cDevice &bus)
{
    // Bypass, flush, then stream.  FIFO_FLUSH clears itself and must not be written 0.
    // 先直通、再清空，然后开始出流。FIFO_FLUSH 会自行清零，不得写 0。
    if(bus.update(kFifoConfig1, 0x03, 0x01) != Status::Ok)
        return Status::HwError;
    if(bus.update(kSignalPathReset, 0x04, 0x04) != Status::Ok)
        return Status::HwError;
    usleep(1000);
    if(bus.update(kFifoConfig1, 0x03, 0x00) != Status::Ok)
        return Status::HwError;
    usleep(10000);
    return Status::Ok;
}

void stop(base::I2cDevice &bus)
{
    bus.update(kFifoConfig1, 0x03, 0x01);
}

Status read(base::I2cDevice &bus, ImuHwFifo16Packet *out, std::size_t cap, std::size_t *n_out)
{
    if(!out || !cap || !n_out)
        return Status::ParamError;
    *n_out = 0;

    uint8_t lo = 0, hi = 0;
    if(bus.readBurst(kFifoCountLow, &lo, 1) != Status::Ok)
        return Status::HwError;
    if(bus.readBurst(kFifoCountHigh, &hi, 1) != Status::Ok)
        return Status::HwError;

    const uint16_t bytes = static_cast<uint16_t>((hi << 8) | lo);
    uint16_t pkts = static_cast<uint16_t>(bytes / kPacketSize);
    if(!pkts)
        return Status::Ok;
    if(pkts > cap)
        pkts = static_cast<uint16_t>(cap);

    std::vector<uint8_t> raw(static_cast<std::size_t>(pkts) * kPacketSize);
    if(bus.readBurst(kFifoData, raw.data(), static_cast<uint32_t>(raw.size()))
            != Status::Ok)
        return Status::HwError;

    std::size_t used = 0;
    for(uint16_t i = 0; i < pkts; i++){
        const uint8_t *p = raw.data() + i * kPacketSize;

        // Header bit7 marks an empty record, which the part fills with 0xFF
        // 头部 bit7 标记空记录，该器件会用 0xFF 填充这类记录
        if(p[0] & 0x80)
            continue;

        ImuHwFifo16Packet &s = out[used];
        s.is_fsync = (p[0] & kFsyncMask) == kFsyncVal;
        s.accel[0] = static_cast<int16_t>((p[1]  << 8) | p[2]);
        s.accel[1] = static_cast<int16_t>((p[3]  << 8) | p[4]);
        s.accel[2] = static_cast<int16_t>((p[5]  << 8) | p[6]);
        s.gyro[0]  = static_cast<int16_t>((p[7]  << 8) | p[8]);
        s.gyro[1]  = static_cast<int16_t>((p[9]  << 8) | p[10]);
        s.gyro[2]  = static_cast<int16_t>((p[11] << 8) | p[12]);
        // The SDK converts with the 42688 scale (2.07 LSB/degC); this part uses
        // 1.983, so the stored code is pre-scaled to land on the right value.
        // SDK 按 42688 的刻度（2.07 LSB/degC）换算；本器件的刻度是 1.983，因此先把存储的
        // 原始码预缩放，使其落到正确的数值上。
        s.temp     = static_cast<int8_t>(p[13] * (2.07f / 1.983f));

        // When the header is not F-SYNC, this field is a normal timestamp, not an edge offset; set 0 to avoid misuse
        // 当头部不是 F-SYNC 时，该字段是普通时间戳，而不是边沿偏移；置 0 以避免误用
        if(s.is_fsync){
            const uint32_t dt = static_cast<uint32_t>((p[14] << 8) | p[15]);
            s.delta_time_us = dt;   // 1 us per tick 每个 tick 为 1 us
        }
        else{
            s.delta_time_us = 0;
        }
        used++;
    }

    *n_out = used;
    return Status::Ok;
}

bool full(base::I2cDevice &bus)
{
    uint8_t val = 0;
    if(bus.readBurst(kIntStatus, &val, 1) != Status::Ok)return false;
    return (val & 0x02) != 0;   // INT_STATUS bit1 = FIFO_FULL (auto-cleared on read) INT_STATUS bit1 = FIFO_FULL（读取时自动清除）
}

// No compensation until the filter delay is confirmed for this model.
// 在本型号的滤波延迟得到确认之前，不做补偿。
Status group_delay_ns(base::I2cDevice &, uint64_t *delay_ns)
{
    if(!delay_ns)return Status::ParamError;
    *delay_ns = 0;
    return Status::Ok;
}

void deinit(base::I2cDevice &bus)
{
    // Graceful shutdown: stop FIFO -> power down the sensor
    // 优雅关闭：停止 FIFO -> 给传感器断电
    bus.update(kFifoConfig1, 0x03, 0x01);
    bus.update(kPwrMgmt0, 0x0F, 0x00);
}

} // namespace

// Only the -C part is used on this module: WHO_AM_I 0x61.
// 本模组只使用 -C 型号：WHO_AM_I 为 0x61。
extern const ModelDesc kIcm42607C = {
    kWhoAmI,
    kIdIcm42607C,
    "ICM-42607-C",
    "- odr:       25 | 50 | 100 | 200 | 400 | 800 | 1600 Hz\n"
    "- accel fsr: 2 | 4 | 8 | 16 g\n"
    "- gyro fsr:  250 | 500 | 1000 | 2000 dps\n"
    "- bw_sel: 0..7\n"
    "  - 0 filter bypassed\n"
    "  - 1 180Hz\n"
    "  - 2 121Hz\n"
    "  - 3 73Hz\n"
    "  - 4 53Hz\n"
    "  - 5 34Hz\n"
    "  - 6 25Hz\n"
    "  - 7 16Hz\n",
    init,
    start,
    stop,
    read,
    full,
    deinit,
    group_delay_ns,
};

DSC_IMU_REGISTER_MODEL(kIcm42607C);

} // namespace imu
} // namespace dsc
