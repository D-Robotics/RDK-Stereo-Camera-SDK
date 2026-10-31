# RDK 双目摄像头模组 SDK 核心接口说明

**简体中文** | [English](README.md)

## 1 概述

### 1.1 简介

`core` 是 RDK 双目摄像头模组 SDK 的核心部分，使用 C/C++ 开发，对外提供 C 接口，SDK 的功能都在这部分实现；Python 包和 ROS 2 封装都依赖它。

### 1.2 支持的型号与平台

| 型号 | 说明 | RDK X5 | RDK S100 | RDK S600 |
| --- | --- | --- | --- | --- |
| GS130W | 双 SC132GS，无 IMU，横屏 1280×1088，80 mm 基线 | 支持 | 支持 | 支持 |
| GS130WI | 双 SC132GS，搭载 IMU 型号为 ICM42688-P，竖屏 1088×1280，70 mm 基线 | 支持 | 支持 | 支持 |

## 2 构建与安装

### 2.1 构建

构建仅支持在 [1.2 支持的型号与平台](#12-支持的型号与平台) 中声明的平台硬件上进行，需要以下依赖：

- Horizon 多媒体库（RDK OS 自带）：`libvpf`、`libhbmem`、`libcam`（位于 `/usr/hobot/lib`）
- OpenCV 4（RDK OS 自带）
- TBB

在 `core` 目录下执行 `make <平台名>`：

```bash
make RDKX5
```

编译前会显示本次构建的平台和版本，确认后开始：

```console
  ──────────────────────────────────────────────────────────────
    RDK Stereo Camera SDK Core
      platform   RDKX5
      version    1.0.0
  ──────────────────────────────────────────────────────────────

    Check that the build platform matches the platform you will run on.

    Build? [Y/n]
```

支持的平台名：`RDKX5`、`RDKS100`、`RDKS600`。不写平台名时默认 `RDKX5`。

编译输出：

```text
build/
└── RDKX5/                                  # 目标文件与依赖文件
out/
├── RDKX5/                                  # 库、示例程序、诊断工具
└── dstereo-cam_<版本>+<平台>_<架构>.deb    # Debian 包
```

### 2.2 安装

```bash
sudo dpkg -i out/dstereo-cam_<版本>+<平台>_<架构>.deb
```

安装内容：

- `/usr/include`：`dstereo_cam.h`、`dstereo_cam_define.h`
- `/usr/lib/<multiarch>`：`libdstereocam.a`、`libdstereocam.so*`
- `/usr/bin`：`dsc` 及其子工具

确认安装：

```bash
dpkg -s dstereo-cam
```

Python 包和 ROS 2 封装依赖这里安装的库与头文件，需要先装 core。

## 3 工具说明

### 3.1 命令用法

`dsc` 是 SDK 的命令行入口，用法为 `dsc <命令> [参数]`。

| 命令 | 作用 |
| --- | --- |
| `dsc help` | 显示帮助 |
| `dsc version` | 打印 SDK 版本与编译平台 |
| `dsc detect <目标> -b <总线...> [-a <地址...>]` | 扫描 I2C 总线与地址 |
| `dsc shell` | 进入交互式调试模式，见 3.2 |

**dsc version**

```bash
dsc version
```

打印 SDK 版本和编译平台。

**dsc detect**

```bash
dsc detect <imu|eeprom|camera> -b <总线...> [-a <地址...>]
```

- `<目标>`：`imu`、`eeprom`、`camera` 三者之一
- `-b`：要扫描的 I2C 总线，必填，例如 `-b 4 6`
- `-a`：要探测的地址，默认按目标取 imu `0x68 0x69`、eeprom `0x50`、camera `0x30 0x32`

示例：

```bash
dsc detect camera -b 4 6
dsc detect eeprom -b 4 6
```

每个命令都支持 `--help`，例如 `dsc detect --help`。

使用 `dsc shell` 命令可以固定一套设备配置并连续调试，用法见 3.2。

### 3.2 交互式调试（dsc shell）

`dsc shell` 先锁定一套设备配置，再进入交互式子命令行，之后输入的命令都复用这套配置：

```bash
dsc shell -d GS130WI [-m rect] [-w 1088] [-h 1280] [-f 30] [-o 200]
```

- `-d <型号>`：设备型号，必填，可选 `GS130W` 或 `GS130WI`
- `-m <出图模式>`：出图类型，默认 `rect`
  - `raw`：原始画面，不做畸变矫正
  - `resize`：居中裁剪并缩放后的画面，保持宽高比
  - `rect`：立体校正后的画面，无畸变、左右目极线对齐
- `-w <宽度>`：输出宽度，单位像素，默认 `1088`
- `-h <高度>`：输出高度，单位像素，默认 `1280`
- `-f <帧率>`：相机帧率，默认 `30`
- `-o <采样率>`：IMU 采样率，单位 Hz，默认 `200`，可选 `200`、`500`

除 `-d` 外都可以省略。

输出宽高受出图模式约束：

- `raw` 出图模式：宽高必须等于传感器尺寸 `1088×1280`，否则初始化返回 `DSC_PARAM_ERROR`
- `resize` 与 `rect` 出图模式：宽高比需要能整除，常用 `1088×1280`、`544×640`、`272×320`、`136×160`、`68×80`

缩放能力随平台不同：RDK X5 使用 VSE，RDK S100 / S600 使用 PYM 金字塔，最大缩小 1/16。

进入后的效果：

```console
$ dsc shell -d GS130WI
dsc-shell>> help
commands:
  help                    show this help
  imu-info                print the detected IMU model and its details
  eeprom-info             print the EEPROM calibration model and its details
  calib-export <dir>      export the calibration as Kalibr YAML into <dir>
  run                     stream both channels, printing the newest data at 50 Hz
  rec -c a:b -i c:d -o d  record index ranges until Ctrl-C, then write them out
  exit | quit             leave the shell
dsc-shell>> exit
```

shell 下每个命令都有对应的源码放在 `samples/` 下，这些程序既作为子程序供 `dsc shell` 调用，也可以单独运行，作为 C 示例程序。

**imu-info**

打印探测到的 IMU 型号及其参数。

子命令 imu-info 的实现由文件 `samples/dsc-imu-info.c` 实现，详细可查看源码。

**eeprom-info**

打印 EEPROM 中的标定型号及其内容。

子命令 eeprom-info 的实现由文件 `samples/dsc-eeprom-info.c` 实现，详细可查看源码。

**calib-export &lt;dir&gt;**

把标定导出为 Kalibr 格式的 `camchain.yaml` 和 `imu.yaml` 到指定目录。

子命令 calib-export 的实现由文件 `samples/dsc-calib-export.c` 实现，详细可查看源码。

**run**

持续采集，以 50 Hz 打印最新的相机帧和 IMU 包，按 Ctrl-C 结束。

子命令 run 的实现由文件 `samples/dsc-run.c` 实现，详细可查看源码。

**rec**

按索引区间录制数据，Ctrl-C 结束后写盘。

```console
rec [-c <from:to>] [-i <from:to>] -o <dir> [--stitch]
```

- `-c`：录制索引区间 `[from, to)` 内的相机帧
- `-i`：录制索引区间 `[from, to)` 内的 IMU 包
- `-o`：输出目录
- `--stitch`：每帧输出一张左右拼接图，而不是左右目各一张

子命令 rec 的实现由文件 `samples/dsc-rec.c` 实现，详细可查看源码。

**help / exit**

显示命令列表、退出 shell。

子命令 help 与 exit 的实现由文件 `samples/dsc` 实现，详细可查看源码。

## 4 API 说明

### 4.1 头文件与链接

`dstereo_cam.h` 定义了 SDK 的全部 API，使用前必须包含。`dstereo_cam_define.h` 提供设备初始化用的便捷预定义宏（型号预设），需要时包含：

```c
#include "dstereo_cam.h"
#include "dstereo_cam_define.h"
```

链接 `libdstereocam`：

```bash
gcc main.c -ldstereocam
```

### 4.2 创建与初始化

**dsc_create**

```c
dsc_device_t *dsc_create();
```

创建一个设备句柄。此时还没有占用任何设备资源，句柄最后用 `dsc_destroy()` 释放。

**dsc_init**

```c
dsc_err_t dsc_init(dsc_device_t *dev, const dsc_config_t *cfg);
```

初始化设备：按候选总线依次探测 EEPROM 与 IMU、读取标定、配置相机与 IMU、建立采集流水线。初始化完成后仍未开流。

- `dev`：`dsc_create()` 返回的句柄
- `cfg`：完整填写的配置，字段说明见 4.3；也可以直接用 `DSC_CONFIG` 宏从型号预设生成

返回值：

- `DSC_OK`：成功
- `DSC_PARAM_ERROR`：参数、配置字段或调用顺序不合法
- `DSC_UNSUPPORTED`：该硬件配置不支持
- `DSC_NOT_FOUND`：未探测到双目相机
- `DSC_HW_ERROR`：设备通信或流水线建立失败

**dsc_deinit / dsc_destroy**

`dsc_deinit()` 释放设备资源，若仍在采集会先停流；调用后句柄仍然有效，可以再次初始化。`dsc_destroy()` 释放句柄本身，调用前必须先 `dsc_deinit()` 或至少 `dsc_stop()`。

**示例**

```c
dsc_config_t cfg = DSC_CONFIG("GS130WI", DSC_CAMERA_MODE_RECT, 544, 640, 30, 200);

dsc_device_t *dev = dsc_create();
if (dsc_init(dev, &cfg) != DSC_OK) {
    dsc_destroy(dev);
    return -1;
}

/* ... 采集 ... */

dsc_deinit(dev);
dsc_destroy(dev);
```

`DSC_CONFIG(型号, 出图模式, 宽, 高, 帧率, IMU 采样率)` 按当前编译平台和给出的型号取预设配置，省去手填字段。

### 4.3 配置说明

`dsc_config_t` 是传给 `dsc_init()` 的完整配置，由五部分组成：

| 字段 | 作用 |
| --- | --- |
| `camera_config` | 相机与平台流水线参数 |
| `imu_config` | IMU 探测与采样参数 |
| `eeprom_config` | 标定 EEPROM 探测参数 |
| `camera_fifo` | 相机帧 FIFO |
| `imu_fifo` | IMU 包 FIFO |

结构体不会自动补默认值，未赋值的字段保持零值。一般从 `DSC_CONFIG` 预设开始，只改需要变的字段。

**预配置宏**

配置项较多，逐项填写既繁琐又容易出错。为了免去逐项查阅配置含义，`dstereo_cam_define.h` 按"平台 × 型号"预定义了一套配置，一行就能拿到一份可用的配置：

- `DSC_CONFIG(型号, 出图模式, 宽, 高, 帧率, IMU 采样率)`：按当前编译平台自动选择对应预设

需要定制配置时，建议先用预配置宏生成一份可用的初始值，再按字段修改：

```c
dsc_config_t cfg = DSC_CONFIG("GS130WI", DSC_CAMERA_MODE_RECT, 544, 640, 30, 200);

cfg.camera_config.fps = 25;   /* 相机帧率 */
cfg.camera_fifo.depth = 8;    /* 相机帧 FIFO 深度 */
```

预配置宏的具体取值（各平台、各型号的总线、地址、时序等）见 `dstereo_cam_define.h`。

**camera_config**

- `bus` / `bus_num`：候选 I2C 总线，按数组顺序逐个探测
- `left_addr` / `right_addr`：左右目的 I2C 地址
- `sensor_width` / `sensor_height`：传感器原始分辨率
- `fps`：相机帧率
- `line_length` / `frame_length`：传感器时序参数
- `tuning_file`：ISP 调优文件路径，`NULL` 表示不加载
- `output_width` / `output_height`：输出分辨率，`raw` 出图模式必须等于传感器原始分辨率
- `mode`：出图模式，见 3.2 的 `-m`
- `stereo_layout`：左右目输出方式，`DSC_STEREO_LAYOUT_NONE` 为分别输出，其余取值输出一张拼接帧
- `bus_mipi_rx` / `bus_reset_gpio`：I2C 总线到 MIPI RX 与复位 GPIO 的映射
- `fsync_camera`：IMU FSYNC 接在哪一目

**imu_config**

- `bus` / `bus_num`：候选总线；`bus_num` 为 0 表示不探测 IMU
- `addr`：IMU 的 I2C 地址
- `odr_hz`：采样率
- `accel_fsr_g` / `gyro_fsr_dps`：加速度计与陀螺仪量程
- `accel_bw_sel` / `gyro_bw_sel`：滤波器档位，取值见 `dsc_get_imu_info()`

**eeprom_config**

- `bus` / `bus_num`：候选总线；`bus_num` 为 0 表示不探测 EEPROM
- `addr`：EEPROM 的 I2C 地址

**camera_fifo / imu_fifo**

- `depth`：FIFO 深度，至少为 2
- `mode`：FIFO 满时的策略，`DSC_FIFO_DROP_NEW` 丢新数据，`DSC_FIFO_DROP_OLD` 丢最旧数据

### 4.4 开流与关流

**dsc_start**

```c
dsc_err_t dsc_start(dsc_device_t *dev);
```

开始采集。探测到 IMU 时，相机会等 IMU 的 FSYNC 握手完成后才开始出图；没有 IMU 时直接开始出图。重复调用返回 `DSC_PARAM_ERROR`。

- `dev`：已初始化的句柄

返回值：

- `DSC_OK`：采集线程已启动
- `DSC_PARAM_ERROR`：句柄为空、未初始化，或已经在运行
- `DSC_HW_ERROR`：IMU 或相机流水线启动失败

**dsc_stop**

```c
void dsc_stop(dsc_device_t *dev);
```

停止采集并等待后台线程退出。未开流时调用没有副作用，停止后可以再次 `dsc_start()`。

### 4.5 获取图像数据

**dsc_available_camera**

```c
size_t dsc_available_camera(dsc_device_t *dev);
```

查询当前可获取的图像对数量。

**dsc_get_nv12_frame**

```c
dsc_err_t dsc_get_nv12_frame(dsc_device_t *dev,
                             dsc_image_nv12_t *image_left,
                             dsc_image_nv12_t *image_right);
```

取一对左右目图像，非阻塞。返回的 `data` 归调用方所有，用完必须 `free()`。

`dsc_image_nv12_t` 的字段：

- `data`：NV12 数据，Y 平面在前，UV 交织在后
- `width` / `height`：图像宽高，单位像素
- `timestamp_ns`：采集时间戳，单位纳秒，位于相机时钟

返回值：

- `DSC_OK`：取到一对已同步的图像
- `DSC_PARAM_ERROR`：参数为空，或相机 FIFO 不可用
- `DSC_UNSUPPORTED`：配置了拼接输出（`stereo_layout` 不是 `DSC_STEREO_LAYOUT_NONE`）
- `DSC_TIMEOUT`：FIFO 当前为空
- `DSC_THREAD_CLOSED`：未开流
- `DSC_HW_ERROR`：采集线程因硬件故障退出

**dsc_get_stereo_nv12_frame**

```c
dsc_err_t dsc_get_stereo_nv12_frame(dsc_device_t *dev,
                                    dsc_image_nv12_t *image);
```

取一张左右拼接好的图像，拼接方式由 `dsc_init()` 时的 `stereo_layout` 决定：水平拼接宽度翻倍，垂直拼接高度翻倍。同样非阻塞，`data` 用完必须 `free()`。

返回值与 `dsc_get_nv12_frame` 相同，区别是布局为 `DSC_STEREO_LAYOUT_NONE` 时返回 `DSC_UNSUPPORTED`。

两个函数怎么选，取决于初始化时的 `stereo_layout`：需要左右目分别处理时用 `dsc_get_nv12_frame`；配置成拼接输出、希望一次取到整张图时用 `dsc_get_stereo_nv12_frame`。

**典型用法**

```c
for (;;) {
    if (dsc_available_camera(dev) > 0) {
        dsc_image_nv12_t left, right;
        if (dsc_get_nv12_frame(dev, &left, &right) == DSC_OK) {
            /* 处理 left.data / right.data，NV12 格式 */
            free(left.data);
            free(right.data);
        }
    }
}
```

### 4.6 获取 IMU 数据

**dsc_available_imu**

```c
size_t dsc_available_imu(dsc_device_t *dev);
```

查询当前可获取的 IMU 数据包数量。

**dsc_get_imu_packet**

```c
dsc_err_t dsc_get_imu_packet(dsc_device_t *dev, dsc_imu_packet_t *out);
```

取一个 IMU 数据包，非阻塞。

`dsc_imu_packet_t` 的字段：

- `accel[3]`：加速度，单位 m/s²
- `gyro[3]`：角速度，单位 rad/s
- `temp`：温度，单位摄氏度
- `is_fsync`：该包是否为携带 FSYNC 锚点的包
- `timestamp_ns`：时间戳，单位纳秒，位于相机时钟

返回值：

- `DSC_OK`：取到一个数据包
- `DSC_PARAM_ERROR`：参数为空，或该设备没有 IMU
- `DSC_TIMEOUT`：FIFO 当前为空
- `DSC_THREAD_CLOSED`：未开流
- `DSC_HW_ERROR`：IMU 线程因硬件故障退出

没有 IMU 的型号（如 `GS130W`）不会返回数据包，一直得到 `DSC_TIMEOUT`。

**dsc_get_imu_name / dsc_get_imu_info**

```c
const char *dsc_get_imu_name(dsc_device_t *dev);
const char *dsc_get_imu_info(dsc_device_t *dev);
```

取探测到的 IMU 型号名和详细信息（量程、滤波档位等）。返回值是静态字符串，不要修改或释放；未探测到 IMU 时返回 `NULL`。

**典型用法**

```c
for (;;) {
    if (dsc_available_imu(dev) > 0) {
        dsc_imu_packet_t packet;
        if (dsc_get_imu_packet(dev, &packet) == DSC_OK) {
            /* 使用 packet.accel / packet.gyro / packet.timestamp_ns */
        }
    }
}
```

### 4.7 获取标定参数

标定数据在 `dsc_init()` 时从 EEPROM 读出，之后可以随时查询，采集与否都可以。没有探测到 EEPROM 时返回 `DSC_NOT_FOUND`。

**相机内参**

```c
dsc_err_t dsc_get_camera_intrinsics(dsc_device_t *dev,
                                    dsc_camera_index_t cam_idx,
                                    dsc_camera_intrinsics_t *intrinsics);
```

- `cam_idx`：`DSC_CAMERA_RIGHT_IDX` 或 `DSC_CAMERA_LEFT_IDX`
- `fx`、`fy`、`cx`、`cy`：焦距与主点，单位像素
- `K[9]`：行优先的 3×3 内参矩阵
- `dist_coeffs[8]`：畸变系数
- `dist_model`：畸变模型，`DSC_DIST_PINHOLE` 为 OpenCV radtan 顺序，`DSC_DIST_FISHEYE` 为等距模型

返回的内参随出图模式不同：

- `raw`：传感器原始内参
- `resize`：按裁剪与缩放换算到输出分辨率的内参，畸变系数不变（图像仍有畸变）
- `rect`：校正后的虚拟内参，理想针孔，`fx = fy`、`cx = w/2`、`cy = h/2`，畸变系数为 0

**IMU 内参**

```c
dsc_err_t dsc_get_imu_intrinsics(dsc_device_t *dev,
                                 dsc_imu_intrinsics_t *intrinsics);
```

包含轴间耦合矩阵、各轴刻度、零偏、噪声密度与随机游走。

**相对位姿**

```c
dsc_err_t dsc_get_relative_R(dsc_device_t *dev,
                             dsc_reference_frame_t from_frame,
                             dsc_reference_frame_t to_frame,
                             double *R);
dsc_err_t dsc_get_relative_T(dsc_device_t *dev,
                             dsc_reference_frame_t from_frame,
                             dsc_reference_frame_t to_frame,
                             double *T);
```

两个函数配合使用，满足 `p_to = R * p_from + T`。`from_frame` 与 `to_frame` 取 `DSC_REF_CAMERA_LEFT`、`DSC_REF_CAMERA_RIGHT` 或 `DSC_REF_IMU`。

**完整标定**

```c
dsc_err_t dsc_get_calibration(dsc_device_t *dev, dsc_calibration_t *calibration);
```

一次取回全部标定：IMU 内参、左右目内参、左右目与 IMU 的位姿，以及安装角。

**转换参考坐标系**

```c
dsc_err_t dsc_convert_calibration(dsc_device_t *dev,
                                  dsc_reference_frame_t ref_frame,
                                  const double ref_R[9],
                                  const double ref_T[3]);
```

把 `ref_frame` 指定设备的位姿设为 `(ref_R, ref_T)`，其余设备的位姿做同样的刚体变换，相对位姿保持不变。

只改内存中的标定数据，不重算立体校正；重新 `dsc_init()` 会从 EEPROM 重新加载，覆盖这里的修改。

**EEPROM 信息**

```c
const char *dsc_get_eeprom_name(dsc_device_t *dev);
const char *dsc_get_eeprom_info(dsc_device_t *dev);
```

取标定 EEPROM 的型号名与详细信息（标定厂商、格式版本、设备类型、畸变模型等）。返回值是静态字符串，不要修改或释放；未探测到 EEPROM 时返回 `NULL`。

### 4.8 获取版本与平台信息

**dsc_version**

```c
const char *dsc_version(void);
```

返回 SDK 版本号，形如 `major.minor.patch`。

**dsc_platform**

```c
const char *dsc_platform(void);
```

返回编译时选定的平台名，例如 `RDKX5`，即平台目录名。

两个函数都返回静态字符串，不要修改或释放。

```c
printf("dsc %s on %s\n", dsc_version(), dsc_platform());
```