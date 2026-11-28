/**
 * @file i2c.cpp
 * @brief I2cDevice implementation: plain i2c-dev read/write transactions.
 * @brief I2cDevice 实现：基于 i2c-dev 的普通读/写事务。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/i2c/i2c.hpp"

#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

namespace dsc {
namespace base {

I2cDevice::~I2cDevice()
{
    close();
}

// The descriptor is the only resource held; the slave address is bound to the bus
// rather than to the object, so the moved-from object simply ends up closed.
// 描述符是唯一持有的资源；从机地址绑定在总线上而不是绑定在对象上，
// 因此被移动后的对象最终只是处于关闭状态。
I2cDevice::I2cDevice(I2cDevice &&other) noexcept
    : fd_(other.fd_)
{
    other.fd_ = -1;
}

I2cDevice &I2cDevice::operator=(I2cDevice &&other) noexcept
{
    if(this != &other){
        close(); // release this object's old fd first 先释放本对象的旧 fd
        fd_ = other.fd_; // take over the other's fd 接管对方的 fd。
        other.fd_ = -1;
    }
    return *this;
}

// Failure to open the bus or to select the address leaves the object closed rather
// than throwing, so the caller tests operator bool() / is_open().
// 打开总线或选择地址失败时对象保持关闭状态而不是抛异常，
// 因此调用方通过 operator bool() / is_open() 判断是否可用。
I2cDevice::I2cDevice(uint8_t bus, uint8_t addr)
{
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%u", bus);

    fd_ = ::open(path, O_RDWR);
    if(fd_ < 0)return ; // failure: object invalid (!*this) 失败：对象无效（!*this）。

    // I2C_SLAVE_FORCE rather than I2C_SLAVE, so the address can be selected even on a
    // bus that already has a kernel driver bound to it; user-space access then
    // bypasses that driver, which must therefore stay idle.
    // 使用 I2C_SLAVE_FORCE 而不是 I2C_SLAVE，这样即使总线上已经绑定了内核驱动，
    // 也能选中该地址；此后用户态访问会绕过该驱动，因此该驱动必须保持空闲。
    if(::ioctl(fd_, I2C_SLAVE_FORCE, addr) < 0){
        ::close(fd_);
        fd_ = -1;
    }
}

void I2cDevice::close()
{
    if(fd_ >= 0){
        ::close(fd_);
        fd_ = -1;
    }
}

Status I2cDevice::read(uint8_t reg, uint8_t *val) const
{
    if(fd_ < 0 || !val)return Status::ParamError;
    if(::write(fd_, &reg, 1) != 1)return Status::HwError;   // address phase 地址阶段
    if(::read(fd_, val, 1) != 1)return Status::HwError;     // data phase 数据阶段。

    return Status::Ok;
}

// The register address is transmitted big-endian (MSB first), like the value.
// 寄存器地址按大端（MSB 在前）传输，数值同样如此。
Status I2cDevice::read16(uint16_t reg, uint16_t *val) const
{
    if(fd_ < 0 || !val)return Status::ParamError;

    const uint8_t addr[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
    };
    uint8_t rbuf[2];

    if(::write(fd_, addr, 2) != 2)return Status::HwError;
    if(::read(fd_, rbuf, 2) != 2)return Status::HwError;

    *val = static_cast<uint16_t>((rbuf[0] << 8) | rbuf[1]);

    return Status::Ok;
}

Status I2cDevice::write(uint8_t reg, uint8_t val) const
{
    if(fd_ < 0)return Status::ParamError;

    const uint8_t buf[2] = {reg, val};
    if(::write(fd_, buf, 2) != 2)return Status::HwError;
    
    return Status::Ok;
}

// Read-modify-write guarded by the mask; an unchanged value costs one read only.
// 由掩码保护的读-改-写；值未变化时只需付出一次读的代价。
Status I2cDevice::update(uint8_t reg, uint8_t mask, uint8_t val) const
{
    uint8_t old_val = 0;
    Status ret = read(reg, &old_val);
    if(ret != Status::Ok)return ret;

    const uint8_t new_val = static_cast<uint8_t>(
        (old_val & static_cast<uint8_t>(~mask)) |
        (val & mask));

    if(new_val == old_val)return Status::Ok;   // nothing to write 无需写入。

    return write(reg, new_val);
}

// Register address and value are both big-endian: MSB first for each of them.
// 寄存器地址和数值都是大端：两者均 MSB 在前。
Status I2cDevice::write16(uint16_t reg, uint16_t val) const
{
    if(fd_ < 0)return Status::ParamError;

    const uint8_t buf[4] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>(val >> 8),
        static_cast<uint8_t>(val & 0xFF)};

    if(::write(fd_, buf, 4) != 4)return Status::HwError;
    
    return Status::Ok;
}

// len bytes in one data phase; any short transfer is reported as a hardware error,
// because i2c-dev repeats no data on its own.
// 在一个数据阶段读取 len 个字节；任何短传输都会上报为硬件错误，
// 因为 i2c-dev 自身不会重发数据。
Status I2cDevice::readBurst(uint8_t reg, uint8_t *buf, uint32_t len) const
{
    if(fd_ < 0 || !buf || !len)return Status::ParamError;
    if(::write(fd_, &reg, 1) != 1)return Status::HwError;
    if(::read(fd_, buf, len) != static_cast<ssize_t>(len))
        return Status::HwError;
    
    return Status::Ok;
}

// Only the address phase is 16-bit here; the payload stays 8-bit.
// 这里只有地址阶段是 16 位，载荷仍然是 8 位。
Status I2cDevice::readReg16(uint16_t reg, uint8_t *val) const
{
    if(fd_ < 0 || !val)return Status::ParamError;

    const uint8_t addr[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF)};
    if(::write(fd_, addr, 2) != 2)return Status::HwError;
    if(::read(fd_, val, 1) != 1)return Status::HwError;
    
    return Status::Ok;
}

// Only the address phase is 16-bit here; the payload stays 8-bit.
// 这里只有地址阶段是 16 位，载荷仍然是 8 位。
Status I2cDevice::writeReg16(uint16_t reg, uint8_t val) const
{
    if(fd_ < 0)return Status::ParamError;

    const uint8_t buf[3] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
        val};
    
    if(::write(fd_, buf, 3) != 3)return Status::HwError;
    
    return Status::Ok;
}

// 16-bit address phase followed by len bytes in one data phase; used by the EEPROM
// models, whose register space is addressed with 16 bits.
// 16 位地址阶段之后在一个数据阶段读取 len 个字节；供 EEPROM 型号使用，
// 这些型号的寄存器空间用 16 位地址寻址。
Status I2cDevice::readBurst16(uint16_t reg, uint8_t *buf, uint32_t len) const
{
    if(fd_ < 0 || !buf || !len)return Status::ParamError;

    const uint8_t addr[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF)};

    if(::write(fd_, addr, 2) != 2)return Status::HwError;
    if(::read(fd_, buf, len) != static_cast<ssize_t>(len))
        return Status::HwError;
    
    return Status::Ok;
}

} // namespace base
} // namespace dsc