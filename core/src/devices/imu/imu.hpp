/**
 * @file imu.hpp
 * @brief Model-independent IMU discovery and FIFO interface.
 * @brief 与型号无关的 IMU 探测与 FIFO 接口。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSC_DEVICES_IMU_IMU_HPP
#define DSC_DEVICES_IMU_IMU_HPP

#include <cstddef>
#include <cstdint>

#include "base/i2c/i2c.hpp"
#include "types.hpp"

namespace dsc {
namespace imu {

/**
 * Operations and identification data supplied by one IMU model driver.
 * 由某个 IMU 型号驱动提供的操作与识别数据。
 */
struct ModelDesc {
    uint8_t who_am_i_addr; /**< Register used to identify the model. 用于识别型号的寄存器。 */
    uint8_t who_am_i_val; /**< Expected value of the identification register. 识别寄存器的期望值。 */
    const char  *name; /**< Statically allocated model name. 静态分配的型号名称。 */
    const char  *info; /**< Statically allocated, human-readable capabilities. 静态分配的、便于人阅读的能力说明。 */

    Status (*init)  (base::I2cDevice &bus, const ImuConfig &cfg); /**< Configure the sensor without starting its FIFO. 配置传感器，但不启动其 FIFO。 */
    Status (*start) (base::I2cDevice &bus); /**< Begin FIFO streaming. 开始 FIFO 数据流。 */
    void   (*stop)  (base::I2cDevice &bus); /**< Stop FIFO streaming. 停止 FIFO 数据流。 */
    Status (*read)  (base::I2cDevice &bus, ImuHwFifo16Packet *out,
                     std::size_t cap, std::size_t *n_out); /**< Read at most cap packets. 最多读取 cap 个数据包。 */
    bool   (*full)(base::I2cDevice &bus); /**< Report and acknowledge FIFO overflow as required by the model. 按型号要求上报并确认 FIFO 溢出。 */
    void   (*deinit)(base::I2cDevice &bus); /**< Restore the sensor to its idle state. 把传感器恢复到空闲状态。 */
    /**
     * Read the model's common group delay from its filter registers and convert it
     * into nanoseconds. A null pointer means the model has no documented delay, which
     * the wrapper reports as Status::Unsupported. The call reads registers, so it must
     * not run while the model is streaming.
     * 从该型号的滤波寄存器读取公共群延迟并换算成纳秒。指针为空表示该型号没有文档化的
     * 延迟，此时封装层报告 Status::Unsupported。本调用会读取寄存器，因此不得在该型号
     * 出流期间执行。
     */
    Status (*group_delay_ns)(base::I2cDevice &bus, uint64_t *delay_ns) = nullptr;
};

/**
 * Model-independent IMU wrapper around one I2C device.
 * 围绕一个 I2C 设备的与型号无关的 IMU 封装。
 *
 * Construction opens the bus and probes registered models in registration order.
 * The object owns the I2C descriptor. It is not thread-safe; callers must serialize
 * configuration, streaming, and FIFO access.
 * 构造时打开总线，并按注册顺序探测已注册的型号。对象拥有该 I2C 描述符。它不是线程
 * 安全的；调用方必须串行化配置、出流与 FIFO 访问。
 */
class Imu {
public:
    /**
     * Open bus/addr and bind the first registered model whose WHO_AM_I value matches.
     * 打开 bus/addr，并绑定第一个 WHO_AM_I 值匹配的已注册型号。
     */
    Imu(uint8_t bus, uint8_t addr);
    /**
     * Run the bound model's deinitialization callback, then close the I2C device.
     * 运行已绑定型号的去初始化回调，然后关闭 I2C 设备。
     */
    ~Imu();

    // Non-copyable: the object exclusively owns its I2C descriptor.
    // 不可拷贝：该对象独占拥有它的 I2C 描述符。
    Imu(const Imu &)            = delete;
    Imu &operator=(const Imu &) = delete;

    /**
     * @return true when a supported model was identified.
     * @return 识别到受支持的型号时返回 true。
     */
    explicit operator bool() const { return desc_ != nullptr; }

    /**
     * Configure the bound model; does not start FIFO streaming.
     * 配置已绑定的型号；不启动 FIFO 数据流。
     */
    Status init(const ImuConfig &cfg);
    /**
     * Start FIFO streaming on the bound model.
     * 在已绑定的型号上启动 FIFO 数据流。
     */
    Status start();
    /**
     * Stop FIFO streaming; model callbacks intentionally report no status.
     * 停止 FIFO 数据流；按设计，各型号回调不返回状态。
     */
    void   stop();

    /**
     * Read up to cap hardware FIFO packets into caller-owned storage.
     * 将最多 cap 个硬件 FIFO 数据包读入调用方自有的存储。
     * @param[out] out Packet array with room for cap elements.
     * @param[out] out 可容纳 cap 个元素的数据包数组。
     * @param[in] cap Maximum number of packets to read; must be non-zero.
     * @param[in] cap 最多读取的数据包个数；不得为零。
     * @param[out] n_out Actual number of packets written; set to zero when none are available.
     * @param[out] n_out 实际写入的数据包个数；没有可用数据包时置零。
     */
    Status read(ImuHwFifo16Packet *out, std::size_t cap, std::size_t *n_out);

    /**
     * @return true when the hardware FIFO reports overflow/full; false also covers read failure.
     * @return 当硬件 FIFO 上报溢出/已满时返回 true；读取失败也会返回 false。
     */
    bool full();

    /**
     * Read the sample-path group delay the bound model is currently configured for.
     * 读取已绑定型号当前配置下的采样通路群延迟。
     *
     * The delay is read back from the registers, so this must be called between init()
     * and start(): the I2C bus is not shared with the streaming reads.
     * 该延迟是从寄存器读回的，因此必须在 init() 与 start() 之间调用：I2C 总线不与出流
     * 期间的读取共用。
     *
     * @param[out] delay_ns Common group delay in nanoseconds, written only on success.
     * @param[out] delay_ns 公共群延迟，单位纳秒，仅在成功时写入。
     * @retval Status::Ok Both paths have the same documented delay.
     * @retval Status::Ok 两路具有相同的文档化延迟。
     * @retval Status::ParamError A null output pointer or no bound model.
     * @retval Status::ParamError 输出指针为空，或没有已绑定的型号。
     * @retval Status::Unsupported The paths differ or no common delay is documented.
     * @retval Status::Unsupported 两路延迟不同，或没有文档化的公共延迟。
     */
    Status group_delay_ns(uint64_t *delay_ns);

    /**
     * @return Statically allocated name of the bound model.
     * @return 已绑定型号的静态分配名称。
     */
    const char *name() const;
    /**
     * @return Statically allocated capabilities of the bound model.
     * @return 已绑定型号的静态分配能力说明。
     */
    const char *info() const;

private:
    base::I2cDevice  bus_;
    const ModelDesc *desc_ = nullptr;
};

/**
 * Append a statically allocated model descriptor to the process-wide registry.
 * Model translation units call this during static initialization; descriptors must
 * remain valid for the lifetime of the process.
 * 把静态分配的型号描述符追加到进程级注册表。各型号翻译单元在静态初始化阶段调用它；
 * 描述符必须在进程整个生命周期内保持有效。
 */
bool register_model(const ModelDesc *desc);

} // namespace imu
} // namespace dsc

// Register a namespace-scope ModelDesc from its model .cpp during static initialization.
// 在静态初始化阶段，从型号 .cpp 中注册一个命名空间作用域的 ModelDesc。
#define DSC_IMU_REGISTER_MODEL(DESC) \
    namespace { [[maybe_unused]] const bool dsc_reg_##DESC = ::dsc::imu::register_model(&DESC); }

#endif // DSC_DEVICES_IMU_IMU_HPP
