/**
 * @file icm42607.cpp
 * @brief ICM-42607-C implementation (the part fitted to A1P).
 *
 * The register map differs from the ICM-42688-P one in icm42688.cpp, so the two models do
 * not share code.  Two differences matter here:
 *
 *   - the configuration registers live at 0x1F..0x28 instead of 0x4E..0x5F, the FIFO
 *     count sits at 0x3D and the data port at 0x3F;
 *   - the FIFO enable bits (accelerometer, gyroscope, F-SYNC timestamp) are not in bank 0
 *     at all but in MREG1, which is reached indirectly through BLK_SEL_W / MADDR_W / M_W
 *     (0x79/0x7A/0x7B).  Without writing MREG1 FIFO_CONFIG5 no packet ever reaches the
 *     FIFO, which reads back as "count 0" while the data registers keep updating.
 *
 * Per MREG1 FDATA, the FIFO record is Packet 3 (datasheet section "13 FIFO"): one header
 * byte, accelerometer XYZ, gyroscope XYZ, a two byte timestamp and one byte of
 * temperature, 16 bytes in total.
 */

#include <cstring>
#include <vector>
#include <unistd.h>

#include "base/i2c/i2c.hpp"
#include "devices/imu/imu.hpp"

namespace dsc {
namespace imu {
namespace {

// ---------------- bank 0 ----------------
constexpr uint8_t kSignalPathReset = 0x02;   // bit4 = software reset, bit2 = FIFO flush
constexpr uint8_t kPwrMgmt0        = 0x1F;
constexpr uint8_t kGyroConfig0     = 0x20;   // full scale bits[6:5], ODR bits[3:0]
constexpr uint8_t kAccelConfig0    = 0x21;
constexpr uint8_t kGyroConfig1     = 0x23;   // UI filter bandwidth bits[2:0]
constexpr uint8_t kAccelConfig1    = 0x24;
constexpr uint8_t kFifoConfig1     = 0x28;   // bit1 = mode, bit0 = bypass
constexpr uint8_t kIntfConfig0     = 0x35;   // bit6 = FIFO count in records
constexpr uint8_t kIntfConfig1     = 0x36;   // bits[1:0] = clock select
constexpr uint8_t kIntStatus       = 0x3A;   // bit1 = FIFO full
constexpr uint8_t kFifoData        = 0x3F;
constexpr uint8_t kWhoAmI          = 0x75;

// Indirect access to the MREG banks
constexpr uint8_t kBlkSelW = 0x79;
constexpr uint8_t kMAddrW  = 0x7A;
constexpr uint8_t kMW      = 0x7B;

// MREG1 registers, values from the vendor's ICM42607 configuration note
constexpr uint8_t kSensorConfig3 = 0x06;   // 0x40: APEX off, FIFO extended to 2.25 KiB
constexpr uint8_t kFdrConfig     = 0x66;   // 0x00: no FIFO decimation
constexpr uint8_t kTmstConfig1   = 0x00;   // 0x03: timestamp on, F-SYNC capture on, 1 us
constexpr uint8_t kFifoConfig5   = 0x01;   // 0x37: accel+gyro+FSYNC time, partial read resume
constexpr uint8_t kFifoConfig6   = 0x02;   // 0x01: 0xFF when empty, no watermark RCOSC wake
constexpr uint8_t kFsyncConfig   = 0x03;   // 0x10: temperature LSB as UI marker, rising edge

// FIFO count: the low byte latches both halves, so it has to be read first
constexpr uint8_t kFifoCountLow  = 0x3E;
constexpr uint8_t kFifoCountHigh = 0x3D;

// Identification: WHO_AM_I reports 0x61 on the ICM-42607-C.
constexpr uint8_t kIdIcm42607C = 0x61;

constexpr uint8_t kPacketSize = 16;
constexpr uint8_t kFsyncMask  = 0x0C;   // packet header bits[3:2]
constexpr uint8_t kFsyncVal   = 0x0C;   // 11 = F-SYNC delta; 10 = normal timestamp


// {physical value, register bits}. Keyed by physical value for easy line-by-line checking against the datasheet.
struct Code {
    uint16_t value;
    uint8_t  bits;
};

// GYRO_CONFIG0 / ACCEL_CONFIG0 bits[3:0]
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
constexpr Code kAccelFsr[] = {
    {16, 0x00},
    {8,  0x20},
    {4,  0x40},
    {2,  0x60},
    {0, 0},
};

// Also descending: 000 = +/-2000dps, 011 = +/-250dps
constexpr Code kGyroFsr[] = {
    {2000, 0x00},
    {1000, 0x20},
    {500,  0x40},
    {250,  0x60},
    {0, 0},
};

// GYRO_CONFIG1 / ACCEL_CONFIG1 filter bandwidth codes: 0 = bypass, 1 = 180Hz ... 7 = 16Hz
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
    uint8_t odr_bits, afs_bits, gfs_bits;
    if(!lookup(kOdr, cfg.odr_hz, &odr_bits))                     return Status::Unsupported;
    if(!lookup(kAccelFsr, cfg.accel_fsr_g, &afs_bits))           return Status::Unsupported;
    if(!lookup(kGyroFsr, cfg.gyro_fsr_dps, &gfs_bits))           return Status::Unsupported;
    if(cfg.accel_bw_sel >= kBwSelCount)                          return Status::Unsupported;
    if(cfg.gyro_bw_sel  >= kBwSelCount)                          return Status::Unsupported;

    // Software reset; the bit clears itself once the reset has completed
    if(bus.update(kSignalPathReset, 0x10, 0x10) != Status::Ok)
        return Status::HwError;
    usleep(2000);

    // Sample data and FIFO counts in big-endian order, FIFO counted in bytes
    if(bus.update(kIntfConfig0, 0x70, 0x30) != Status::Ok)
        return Status::HwError;

    // PLL when available, RC oscillator otherwise
    if(bus.update(kIntfConfig1, 0x03, 0x01) != Status::Ok)
        return Status::HwError;

    // RC oscillator first, then gyro and accel in low-noise mode with IDLE kept
    if(bus.write(kPwrMgmt0, 0x10) != Status::Ok)
        return Status::HwError;
    usleep(1000);
    if(bus.write(kPwrMgmt0, 0x1F) != Status::Ok)
        return Status::HwError;
    usleep(200000);

    // FIFO to bypass first, then configure sampling parameters
    if(bus.update(kFifoConfig1, 0x01, 0x01) != Status::Ok)
        return Status::HwError;
    if(bus.update(kGyroConfig0, 0x6F, static_cast<uint8_t>(gfs_bits | odr_bits))
            != Status::Ok)
        return Status::HwError;
    if(bus.update(kAccelConfig0, 0x6F, static_cast<uint8_t>(afs_bits | odr_bits))
            != Status::Ok)
        return Status::HwError;
    if(bus.update(kGyroConfig1, 0x07, cfg.gyro_bw_sel) != Status::Ok)
        return Status::HwError;
    if(bus.update(kAccelConfig1, 0x07, cfg.accel_bw_sel) != Status::Ok)
        return Status::HwError;

    // Watermark unused: the host polls the count instead
    if(bus.write(0x29, 0x00) != Status::Ok)
        return Status::HwError;
    if(bus.write(0x2A, 0x00) != Status::Ok)
        return Status::HwError;

    // MREG1: APEX off, no FIFO decimation, timestamp and the three FIFO fields on,
    // empty-FIFO marker, F-SYNC on the rising edge of the temperature UI marker
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
        s.temp     = static_cast<int8_t>(p[13] * (2.07f / 1.983f));

        // When the header is not F-SYNC, this field is a normal timestamp, not an edge offset; set 0 to avoid misuse
        if(s.is_fsync){
            const uint32_t dt = static_cast<uint32_t>((p[14] << 8) | p[15]);
            s.delta_time_us = dt;   // 1 us per tick
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
    return (val & 0x02) != 0;   // INT_STATUS bit1 = FIFO_FULL (auto-cleared on read)
}

void deinit(base::I2cDevice &bus)
{
    // Graceful shutdown: stop FIFO -> power down the sensor
    bus.update(kFifoConfig1, 0x03, 0x01);
    bus.update(kPwrMgmt0, 0x0F, 0x00);
}

} // namespace

// Only the -C part is used on this module: WHO_AM_I 0x61.
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
};

DSC_IMU_REGISTER_MODEL(kIcm42607C);

} // namespace imu
} // namespace dsc
