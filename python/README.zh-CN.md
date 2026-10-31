# RDK 双目摄像头模组 SDK —— Python 接口

**简体中文** | [English](README.md)

> 基于 `ctypes` 的绑定，把 RDK 双目摄像头、IMU 与标定数据以普通 numpy 数组的形式暴露出来。

## 概述

`dstereo_cam` 是 `libdstereocam` 的 Python 接口。它用 `ctypes` 镜像 C ABI，直接返回 numpy 数组，因此采集循环只需要几行普通 Python。绑定本身不做图像处理、立体匹配或滤波，也不打包原生库。

| 项目 | 说明 |
| --- | --- |
| Python | 3.10 及以上 |
| 原生库 | `libdstereocam.so`（`dstereo-cam` 包） |
| 运行时依赖 | `numpy` |
| 类型提示 | 自带 `py.typed` 与 `__init__.pyi` |
| 平台 | RDK X5、RDK S100、RDK S600 |

## 依赖

- Python 3.10 及以上，以及 `numpy`（>= 1.20）。
- 与本绑定版本匹配的 `libdstereocam`。wheel **不包含**原生库；由于绑定直接镜像 C 结构体，两者版本必须兼容。绑定按以下顺序查找原生库：

  1. 环境变量 `DSC_LIB`
  2. `ldconfig`（通过 `ctypes.util.find_library`）
  3. 默认搜索路径上的 `libdstereocam.so`

  首次加载库时会比较包内记录的版本与库自报的版本：库更旧则拒绝加载，更新则只告警。`library_version()` 与 `library_platform()` 返回实际加载到的版本与平台。

## 安装

**1. 先安装原生库**

```bash
# 检查是否已安装
dpkg -s dstereo-cam

# 未安装时，安装 core/ 构建出的 deb（在 SDK 仓库里执行 make deb）
sudo dpkg -i dstereo-cam_<版本>+<平台>_<架构>.deb
```

**2. 构建并安装绑定**

```bash
# 在板端构建（需要已安装 setuptools 与 wheel）
cd python && ./build-wheel.sh

# 安装刚构建出的 wheel
python3 -m pip install dist/dstereo_cam-*.whl
```

脚本使用 `pip wheel --no-deps --no-build-isolation` 构建，只使用板端已有的 `setuptools` 与 `wheel`，不联网。版本取自仓库根目录的 `VERSION`，因此需要在仓库检出里构建。`./build-wheel.sh clean` 会删除 `build/`、`dist/` 与 `*.egg-info`。

## 快速开始

```python
import dstereo_cam

config = dstereo_cam.preset(
    "GS130WI", dstereo_cam.CameraMode.RECT, 544, 448, 30, 200
)

with dstereo_cam.Device(config) as device:
    device.start()

    while device.available_camera() == 0:
        pass
    images = device.read_image()
    print(images["left"].shape, images["left"].timestamp_ns)

    packet = device.read_imu()
    if packet is not None:
        print(packet.accel, packet.gyro, packet.temp)
```

`preset()` 会为已知硬件填好所有字段；如果想自己填，`config()` 返回同样结构但字段全空的字典。两种情况下它都是普通嵌套字典，可以在传给 `Device` 之前修改：

```python
config["camera_config"]["stereo_layout"] = dstereo_cam.StereoLayout.TOP_BOTTOM
# 之后 read_image() 返回 {"stitched": Image}，而不是 {"left": ..., "right": ...}
```

读取标定并用 OpenCV 转换帧：

```python
calibration = device.calibration()
print(calibration.camera_left.K, calibration.install_angle)

import cv2
bgr = cv2.cvtColor(images["left"], cv2.COLOR_YUV2BGR_NV12)
```

型号名不是平台参数：`preset()` 从未加载的原生库里读取平台，和 C 的 `DSC_CONFIG` 宏行为一致。没有对应预设的硬件会抛 `ValueError`，而宏的行为是打印到 stderr 并退出。

## API

只有"当前没有数据"时 `read_image()` 与 `read_imu()` 返回 `None`，因此采集循环只需判断这一点。其余失败都会抛 `DSCError`。

### 模块级

| 名称 | 说明 |
| --- | --- |
| `preset(device, mode, width, height, fps, odr)` | 为已知硬件返回填好的配置 |
| `config()` | 返回空配置，所有字段未设置 |
| `library_version()` / `library_platform()` | 已加载的 `libdstereocam` 的版本与平台 |
| `package_version()` / `__version__` | 构建 wheel 时记录的版本，未安装时为 `None` |

### Device

| 成员 | 说明 |
| --- | --- |
| `Device(config)` | 校验配置、初始化硬件、探测 IMU |
| `start()` / `stop()` | 开始与结束采集；`start()` 返回 `self` |
| `close()` | 停止采集并释放全部资源，可安全重复调用 |
| `closed`、`stitched` | 设备是否已释放、输出是否为拼接帧 |
| `imu_name`、`imu_info`、`eeprom_name`、`eeprom_info` | 标识字符串，不存在时为 `None` |
| `available_camera()`、`available_imu()` | 队列深度；未开流时为 `0` |
| `read_image()` | 返回 `{"stitched": Image}` 或 `{"left": Image, "right": Image}`，无数据时为 `None` |
| `read_imu()` | 返回一个 `ImuPacket`，无数据时为 `None` |
| `calibration()` | 从 EEPROM 读出的完整双目与 IMU 标定 |
| `camera_intrinsics(camera)`、`imu_intrinsics()` | 单目内参，或 IMU 内参 |
| `relative_R(from, to)`、`relative_T(from, to)` | 两个参考坐标系之间的旋转 `(3, 3)` 与平移 `(3,)` |
| `convert_calibration(ref_frame, R, T)` | 切换参考坐标系，保持各设备位姿不变 |

`relative_R`/`relative_T`/`convert_calibration` 接受 `ReferenceFrame`；`R` 必须 9 个值、`T` 必须 3 个值，长度不对会在调用 C 之前抛 `ValueError`。

### 数据类型

| 类型 | 说明 |
| --- | --- |
| `Image` | `np.ndarray` 子类，形状 `(height * 3 // 2, width)`，`uint8`；携带 `timestamp_ns`、`width`、`height`，并提供零拷贝视图 `y_plane()` / `uv_plane()` |
| `ImuPacket` | `accel` `(3,)` m/s²、`gyro` `(3,)` rad/s、`temp` ℃、`is_fsync`、`timestamp_ns` |
| `CameraIntrinsics` | `fx`、`fy`、`cx`、`cy`、`K` `(3, 3)`、`dist_coeffs` `(8,)`、`dist_model` |
| `ImuIntrinsics` | 加速度计与陀螺仪的轴间耦合、刻度、零偏、噪声与随机游走 |
| `Calibration` | 两目内参、`R` / `T` 位姿，以及 `install_angle` |
| `DSCError` | 调用失败：`.code`、`.reason`、`.func`；`RuntimeError` 的子类 |
| 枚举 | `ErrorCode`、`CameraMode`、`CameraIndex`、`StereoLayout`、`FifoMode`、`DistModel`、`ReferenceFrame` |

## 配置

配置是一个普通嵌套字典，分五节，与 `dsc_config_t` 逐字段对应。`preset()` 会为已知硬件填好全部字段，只有特殊场景才需要自己构造。

| 节 | 字段 |
| --- | --- |
| `camera_config` | `bus`、`left_addr`、`right_addr`、`sensor_width`、`sensor_height`、`fps`、`line_length`、`frame_length`、`tuning_file`、`output_width`、`output_height`、`mode`、`stereo_layout`、`bus_mipi_rx`、`bus_reset_gpio`、`fsync_camera` |
| `imu_config` | `bus`、`addr`、`odr_hz`、`accel_fsr_g`、`gyro_fsr_dps`、`accel_bw_sel`、`gyro_bw_sel` |
| `eeprom_config` | `bus`、`addr` |
| `camera_fifo` | `depth`、`mode` |
| `imu_fifo` | `depth`、`mode` |

- `bus_mipi_rx` 与 `bus_reset_gpio` 是 `{总线: 槽位}` 形式的字典，因为背后的 C 数组是稀疏查找表。`bus_num` 不是字段，等于 `len(bus)`。
- `tuning_file` 是路径字符串或 `None`；`None` 表示不加载调优文件，这对 SDK 是有意义的取值，而不是"未设置"。
- 其余字段取 `None` 都会在构造 `Device` 时被拒绝，避免悄悄停留在零值。

## 特性

- **numpy 原生帧**：一帧就是形状 `(height * 3 // 2, width)` 的 `np.ndarray`，`y_plane()` / `uv_plane()` 是零拷贝视图，数据出 SDK 时不复制。
- **明确的缓冲区归属**：SDK 用 `malloc` 分配帧并转移所有权；`Image` 在最后一个引用消失时释放，因此只要还有视图或切片存活，帧就有效。
- **时间戳不随切片丢失**：`timestamp_ns` 由 `Image` 及其视图携带。返回普通 `numpy.ndarray` 的操作（例如 `cv2.cvtColor`）会丢掉它，所以要先读时间戳。
- **便于写循环的读取**：没有数据时 `read_image()` 与 `read_imu()` 返回 `None` 而不抛异常；真正的失败由其它调用抛 `DSCError`，携带 SDK 给出的错误码、原因与函数名。
- **枚举值来自已加载的库**：所有枚举值在导入时从 `libdstereocam` 读取，Python 里的名字不会与 `dstereo_cam.h` 脱节。
- **安全的生命周期**：`Device` 是上下文管理器，未关闭时会发 `ResourceWarning`；关闭过程中的失败不会掩盖正在传播的异常。
- **带类型提示**：包内提供 `py.typed` 与 `__init__.pyi`，编辑器与类型检查器能看到完整 API。

不提供：图像处理、立体匹配、IMU 滤波、录制，以及超出 `libdstereocam` 自身报告范围的设备发现。

## 硬件测试

`test/test_dstereo_cam.py` 是硬件测试而不是单元测试：它会走一遍公开 API、测量队列深度，并把采集到的帧写成 PNG。

```bash
cd python
python3 -m pip install '.[test]'                     # 测试需要 OpenCV
python3 test/test_dstereo_cam.py GS130WI rect 544 448 30 200
```

位置参数依次是 `device`、`mode`（`raw`、`resize` 或 `rect`）、`width`、`height`、`fps`、`odr`。它可以直接从仓库检出运行，无需先安装；对于本机没有的硬件，会跳过相应环节而不是判定失败。帧默认写到 `dsc_images/`，可用 `--output-dir` 指定其它目录，用 `--overwrite` 写入非空目录。
