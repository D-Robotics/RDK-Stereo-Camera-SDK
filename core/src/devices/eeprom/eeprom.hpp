/**
 * @file eeprom.hpp
 * @brief EEPROM reader for binocular (IMU) distortion and stereo calibration parameters.
 * @brief 用于读取双目（含 IMU）畸变参数与双目标定参数的 EEPROM 读取器。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSC_DEVICES_EEPROM_EEPROM_HPP
#define DSC_DEVICES_EEPROM_EEPROM_HPP

#include <cstdint>

#include "base/i2c/i2c.hpp"
#include "types.hpp"

namespace dsc {
namespace eeprom {

/**
 * One EEPROM layout (vendor + model + version), used as the vtable of a detected device.
 * 一种 EEPROM 布局（厂商 + 型号 + 版本），用作被检测设备的 vtable。
 */
struct ModelDesc{
    /**
     * Statically allocated model name exposed through the public API.
     * EEPROM 型号名称。
     */
    const char *name;
    /**
     * Multi-line description handed to the user, one "- field: value" item per line.
     * EEPROM 描述信息。
     */
    const char *info;

    /**
     * Probe hook: read the identification area and return true when the device matches
     * this layout.  Must not modify the bus state beyond reading, and must be safe to call
     * on a mismatching device (it is called for every registered model in turn).
     * 探测函数，用于探测 EEPROM，由各驱动实现
     */
    bool   (*probe)(base::I2cDevice &bus);

    /**
     * Read hook: fill @p out completely, see Eeprom::read() for the contract.
     * 读取函数：完整填充 @p out，具体契约见 Eeprom::read()。
     */
    Status (*read) (base::I2cDevice &bus, StereoImuModel *out);
};

/**
 * Calibration EEPROM attached to an I2C bus.
 * EEPROM 类
 */
class Eeprom {
public:
    /**
     * @param bus  I2C bus number the EEPROM is wired to.
     * @param bus  EEPROM 所接的 I2C 总线编号。
     * @param addr 7-bit I2C address of the EEPROM (typically 0x50).
     * @param addr EEPROM 的 7 位 I2C 地址（通常为 0x50）。
     */
    Eeprom(uint8_t bus, uint8_t addr);
    ~Eeprom() = default;

    // non-copyable
    // 不可拷贝
    Eeprom(const Eeprom &)            = delete;
    Eeprom &operator=(const Eeprom &) = delete;

    // bool conversion: true = a model is bound; false = probe failed
    // bool 转换：true = 已绑定型号；false = 探测失败
    /**
     * @return true when a registered model matched the device.
     * @return 某个已注册型号与设备匹配时返回 true。
     */
    explicit operator bool() const { return desc_ != nullptr; }

    /**
     * Read the calibration from the detected device.
     * 从检测到的设备读取标定数据。
     * @param[out] out Receives the parsed calibration; every field is written, so the
     *             caller's previous contents are replaced.
     * @param[out] out 接收解析后的标定数据；每个字段都会被写入，因此调用方原有的内容会被替换。
     * @return Status::Ok, Status::ParamError when @p out is null, or
     *         Status::HwError when the EEPROM transfer fails.
     * @return 成功返回 Status::Ok；@p out 为空时返回 Status::ParamError；
     *         EEPROM 传输失败时返回 Status::HwError。
     */
    Status read(StereoImuModel *out);

    /**
     * @return Model string of the bound model; never null for a valid object.
     * @return 已绑定型号的型号字符串；对有效对象永不为 null。
     */
    const char *name() const;

    /**
     * @return Description string of the bound model; never null for a valid object.
     * @return 已绑定型号的描述字符串；对有效对象永不为 null。
     */
    const char *info() const;

private:
    base::I2cDevice  bus_;
    const ModelDesc *desc_ = nullptr;
};

/**
 * Register a model layout in the process-wide registry.
 * 在注册表中登记一种型号布局。
 *
 * @param[in] desc Layout to append; must not be null and must stay valid forever.
 * @param[in] desc 要追加的布局；不得为空，且必须永远保持有效。
 * @return Always true, so that the macro can initialise a bool at file scope.
 * @return 始终返回 true，以便该宏能在文件作用域初始化一个 bool。
 */
bool register_model(const ModelDesc *desc);

} // namespace eeprom
} // namespace dsc

// Self-registration macro: call at file scope in a model .cpp, after the ModelDesc definition.
// Expands to a file-local bool, so the descriptor must be a namespace-scope object with
// static lifetime; the macro is a no-op at runtime and its result may be ignored.
// 自注册宏：在型号 .cpp 的文件作用域中、ModelDesc 定义之后调用。它展开为一个文件内 bool，
// 因此描述符必须是具有静态生命周期的命名空间作用域对象；该宏在运行时无副作用，结果可忽略。
#define DSC_EEPROM_REGISTER_MODEL(DESC) \
    namespace { [[maybe_unused]] const bool dsc_reg_##DESC = ::dsc::eeprom::register_model(&DESC); }

#endif // DSC_DEVICES_EEPROM_EEPROM_HPP
