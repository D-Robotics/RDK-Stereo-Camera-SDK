/**
 * @file i2c.hpp
 * @brief RAII wrapper for Linux i2c-dev; **NOT** thread-safe.
 * @brief Linux i2c-dev 的 RAII 封装；**非**线程安全。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSC_BASE_I2C_HPP
#define DSC_BASE_I2C_HPP

#include <cstdint>

#include "types.hpp"

namespace dsc {
namespace base {

/**
 * Owns one open /dev/i2c-<bus> file descriptor bound to a single slave address.
 * 持有一个已打开的 /dev/i2c-<bus> 文件描述符，并绑定到单个从机地址。
 *
 * Thread safety: none.  One I2cDevice must not be used from two threads at once,
 * and transactions are not atomic -- interleaved access to the same bus from another
 * object or process can corrupt the device state.
 * 线程安全：无。同一个 I2cDevice 不得被两个线程同时使用，
 * 且事务不是原子的——其他对象或进程对同一总线的交错访问可能破坏设备状态。
 */
class I2cDevice {
public:
    /**
     * Open /dev/i2c-<bus> and select the slave address.
     * 打开 /dev/i2c-<bus> 并选中从机地址。
     * @param bus  I2C bus number, i.e. the '<bus>' of /dev/i2c-<bus>.
     * @param bus  I2C 总线编号，即 /dev/i2c-<bus> 中的 '<bus>'。
     * @param addr 7-bit slave address; not validated, it is handed to the driver
     *             as-is (addresses above 0x7F are meaningless on a 7-bit bus).
     * @param addr 7 位从机地址；不做校验，原样交给驱动
     *             （在 7 位总线上大于 0x7F 的地址没有意义）。
     */
    I2cDevice(uint8_t bus, uint8_t addr);
    ~I2cDevice();

    // copying disabled
    // 拷贝被禁用。
    I2cDevice(const I2cDevice &)            = delete;
    I2cDevice &operator=(const I2cDevice &) = delete;

    // moving allowed
    // 允许移动。
    I2cDevice(I2cDevice &&other) noexcept;
    I2cDevice &operator=(I2cDevice &&other) noexcept;

    // boolean conversion semantics: true = success; false = failure
    // 布尔转换语义：true = 成功；false = 失败。
    /** Same condition as is_open(); provided so that a constructed device can be tested directly. */
    /** 与 is_open() 的条件相同；提供它以便直接检测已构造的设备。 */
    explicit operator bool() const { return fd_ >= 0; }

    /** @return true while a slave address is selected on an open bus. */
    /** @return 当已打开的总线上选中了从机地址时为 true。 */
    bool is_open() const { return fd_ >= 0; }
    /** Close the bus if open; idempotent.  Every later transaction fails with Status::ParamError. */
    /** 若总线已打开则关闭；幂等。之后每次事务都会以 Status::ParamError 失败。 */
    void close();

    /**
     * Read one 8-bit register.
     * 读取一个 8 位寄存器。
     * @param[in]  reg Register address (8-bit address phase).
     * @param[in]  reg 寄存器地址（8 位地址阶段）。
     * @param[out] val Receives the register value.
     * @param[out] val 接收寄存器值。
     * @return Status::Ok, Status::ParamError when closed or @p val is null,
     *         Status::HwError when the address phase or the read fails.
     * @return Status::Ok，对象已关闭或 @p val 为空时返回 Status::ParamError，
     *         Status::HwError 表示地址阶段或读取失败。
     */
    Status read(uint8_t reg, uint8_t *val) const;

    /**
     * Read a 16-bit register holding a 16-bit value, big-endian on the bus (MSB first).
     * 读取一个保存 16 位值的 16 位寄存器，总线上为大端（MSB 在前）。
     * @param[in]  reg Register address, transmitted MSB first.
     * @param[in]  reg 寄存器地址，MSB 在前发送。
     * @param[out] val Receives the value reassembled in host byte order.
     * @param[out] val 接收按主机字节序重新组装后的值。
     * @return Status::Ok, Status::ParamError when closed or @p val is null,
     *         Status::HwError when the address phase or the read fails.
     * @return Status::Ok，对象已关闭或 @p val 为空时返回 Status::ParamError，
     *         Status::HwError 表示地址阶段或读取失败。
     */
    Status read16(uint16_t reg, uint16_t *val) const;

    /**
     * Write one 8-bit register.
     * 写入一个 8 位寄存器。
     * @param[in] reg Register address.
     * @param[in] reg 寄存器地址。
     * @param[in] val Value to store.
     * @param[in] val 要存储的值。
     * @return Status::Ok, Status::ParamError when closed, Status::HwError on failure.
     * @return Status::Ok，对象已关闭时返回 Status::ParamError，失败时返回 Status::HwError。
     */
    Status write(uint8_t reg, uint8_t val) const;

    /**
     * Read-modify-write one register: the bits of @p val selected by @p mask replace
     * the corresponding bits of the current value, every other bit is preserved.
     * The write is skipped when the result equals the value just read.
     * 对一个寄存器做读-改-写：@p val 中被 @p mask 选中的位替换当前值的对应位，
     * 其余所有位保持不变。当结果与刚读到的值相等时跳过写入。
     * @param[in] reg  Register address.
     * @param[in] reg  寄存器地址。
     * @param[in] mask Bit mask (1 = take the bit from @p val), already in bit position.
     * @param[in] mask 位掩码（1 = 取 @p val 中的该位），已处在对应位位置。
     * @param[in] val  New bit values, already shifted into position.
     * @param[in] val  新的位值，已移位到对应位置。
     * @return Status::Ok, or the failure of the underlying read/write.
     * @return Status::Ok，或底层读/写操作的失败状态。
     */
    Status update(uint8_t reg, uint8_t mask, uint8_t val) const;

    /**
     * Write a 16-bit register and a 16-bit value, both big-endian (MSB first).
     * 写入一个 16 位寄存器和一个 16 位值，两者均为大端（MSB 在前）。
     * @param[in] reg Register address, transmitted MSB first.
     * @param[in] reg 寄存器地址，MSB 在前发送。
     * @param[in] val Value to store, transmitted MSB first.
     * @param[in] val 要存储的值，MSB 在前发送。
     * @return Status::Ok, Status::ParamError when closed, Status::HwError on failure.
     * @return Status::Ok，对象已关闭时返回 Status::ParamError，失败时返回 Status::HwError。
     */
    Status write16(uint16_t reg, uint16_t val) const;

    /**
     * Read @p len consecutive bytes starting at an 8-bit register address.
     * 从 8 位寄存器地址开始读取 @p len 个连续字节。
     * @param[in]  reg Start register address.
     * @param[in]  reg 起始寄存器地址。
     * @param[out] buf Destination, at least @p len bytes.
     * @param[out] buf 目标缓冲区，至少 @p len 个字节。
     * @param[in]  len Byte count, must not be 0.
     * @param[in]  len 字节数，不得为 0。
     * @return Status::Ok, Status::ParamError when closed or @p buf is null or
     *         @p len is 0, Status::HwError when a short transfer completes.
     * @return Status::Ok，对象已关闭、@p buf 为空或 @p len 为 0 时返回 Status::ParamError，
     *         Status::HwError 表示传输出现短读。
     */
    Status readBurst(uint8_t reg, uint8_t *buf, uint32_t len) const;

    /**
     * Read one byte from a 16-bit register address (only the address phase is 16-bit).
     * 从 16 位寄存器地址读取一个字节（只有地址阶段是 16 位）。
     * @param[in]  reg Register address, transmitted MSB first.
     * @param[in]  reg 寄存器地址，MSB 在前发送。
     * @param[out] val Receives the byte.
     * @param[out] val 接收该字节。
     * @return Status::Ok, Status::ParamError when closed or @p val is null,
     *         Status::HwError when the address phase or the read fails.
     * @return Status::Ok，对象已关闭或 @p val 为空时返回 Status::ParamError，
     *         Status::HwError 表示地址阶段或读取失败。
     */
    Status readReg16(uint16_t reg, uint8_t *val) const;

    /**
     * Write one byte to a 16-bit register address (only the address phase is 16-bit).
     * 向 16 位寄存器地址写入一个字节（只有地址阶段是 16 位）。
     * @param[in] reg Register address, transmitted MSB first.
     * @param[in] reg 寄存器地址，MSB 在前发送。
     * @param[in] val Value to store.
     * @param[in] val 要存储的值。
     * @return Status::Ok, Status::ParamError when closed, Status::HwError on failure.
     * @return Status::Ok，对象已关闭时返回 Status::ParamError，失败时返回 Status::HwError。
     */
    Status writeReg16(uint16_t reg, uint8_t val) const;

    /**
     * Read @p len consecutive bytes starting at a 16-bit register address.
     * 从 16 位寄存器地址开始读取 @p len 个连续字节。
     * @param[in]  reg Start register address, transmitted MSB first.
     * @param[in]  reg 起始寄存器地址，MSB 在前发送。
     * @param[out] buf Destination, at least @p len bytes.
     * @param[out] buf 目标缓冲区，至少 @p len 个字节。
     * @param[in]  len Byte count, must not be 0.
     * @param[in]  len 字节数，不得为 0。
     * @return Status::Ok, Status::ParamError when closed or @p buf is null or
     *         @p len is 0, Status::HwError when a short transfer completes.
     * @return Status::Ok，对象已关闭、@p buf 为空或 @p len 为 0 时返回 Status::ParamError，
     *         Status::HwError 表示传输出现短读。
     */
    Status readBurst16(uint16_t reg, uint8_t *buf, uint32_t len) const;

private:
    int fd_ = -1;
};

} // namespace base
} // namespace dsc

#endif // DSC_BASE_I2C_HPP