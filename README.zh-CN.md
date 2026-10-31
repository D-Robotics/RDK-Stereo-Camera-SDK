# RDK 双目摄像头模组 SDK

**简体中文** | [English](README.md)

RDK 双目摄像头模组 SDK 用于在 RDK 开发板上使用双目摄像头模组及其板载 IMU，由原生 C 库和建立在其上的 Python、ROS 2 封装组成。

> **开发者预览版（Alpha）。** 本版本用于评估，接口、配置预设与行为可能在不另行通知的情况下变更。欢迎通过 [D-Robotics 开发者社区](https://forum.d-robotics.cc/) 反馈问题或参与贡献。

## 概述

硬件流水线只在原生库中实现一次：传感器采集与 ISP 处理、GDC 硬件矫正、从板载 EEPROM 读取标定，以及通过 FSYNC 与相机时基对齐的 IMU 采样。

| 层 | 目录 | 内容 |
| --- | --- | --- |
| core | `core/` | `libdstereocam`、公共 C 头文件、`dsc` 工具与 Debian 打包 |
| Python 封装 | `python/` | `dstereo_cam` 包，通过 `ctypes` 绑定 `libdstereocam` |
| ROS 2 封装 | `ros2/` | `dstereo_cam` 包，发布相机、IMU 与 TF 数据 |

使用封装前必须先安装 core。

## 依赖

- **硬件**：一块 RDK 开发板和一个双目摄像头模组，部分型号带 IMU。
- **core**：Horizon 多媒体库、OpenCV 4、TBB（均由 RDK OS 自带）。
- **Python 封装**：Python 3.10 及以上与 `numpy`；构建 wheel 还需要 `setuptools` 与 `wheel`。
- **ROS 2 封装**：ROS 2 Humble 或 Jazzy。网页预览与双目深度的 launch 文件还需要 TROS 的 `hobot_codec`、`websocket` 与 `hobot_stereonet`。

当前版本实现 RDK X5、RDK S100、RDK S600 三个后端。

## 仓库结构

```text
core/
  include/dstereo_cam.h        公共 C API
  include/dstereo_cam_define.h 平台配置预设
  src/                         库实现（base、devices、tools）
  samples/                     dsc 前端与示例程序
python/
  dstereo_cam/                 Python 封装，含类型声明
  test/test_dstereo_cam.py     硬件测试
ros2/src/dstereo_cam/          ROS 2 包：节点、launch 文件、文档
VERSION                        core 与 Python 封装共用的版本号
LICENSE
```

## 构建与安装

### 1. core

库、工具与 Debian 包由 `core/` 下的 GNU Makefile 构建。

```bash
cd core
make RDKX5
```

产物在 `build/<平台>/` 与 `out/<平台>/`，Debian 包在 `out/`。

```bash
# 检查是否已安装
dpkg -s dstereo-cam

# 未安装时，安装 core/ 构建出的 deb
sudo dpkg -i out/dstereo-cam_<版本>+<平台>_<架构>.deb
```

详见 [core README](core/README.zh-CN.md)。

### 2. Python 封装

wheel 在板端用已有工具链构建。

```bash
cd python && ./build-wheel.sh
python3 -m pip install dist/dstereo_cam-*.whl
```

wheel 不打包原生库，运行时查找已安装的 `libdstereocam`。

详见 [Python README](python/README.zh-CN.md)。

### 3. ROS 2 封装

```bash
cd ros2
source /opt/ros/humble/setup.bash     # 或：source /opt/ros/jazzy/setup.bash
colcon build --packages-select dstereo_cam
source install/setup.bash
```

详见 [ROS 2 README](ros2/src/dstereo_cam/README.zh-CN.md)。

## 快速开始

**命令行工具**

`dsc` 工具随 core 一起安装，源码在 [core/samples/](core/samples)，既可作为诊断工具，也可作为参考实现。

```bash
dsc help                                     # 列出可用命令
dsc version                                  # SDK 版本与编译平台
dsc detect eeprom -b 4 6                     # 扫描 I2C 总线与地址
dsc shell -d GS130WI                         # 进入交互式调试
```

**C**

```c
#include "dstereo_cam.h"
#include "dstereo_cam_define.h"

dsc_config_t config = DSC_CONFIG(
    "GS130WI", DSC_CAMERA_MODE_RECT, 1088, 1280, 30, 200);

dsc_device_t *device = dsc_create();
if (dsc_init(device, &config) != DSC_OK || dsc_start(device) != DSC_OK) {
    dsc_stop(device);
    dsc_deinit(device);
    dsc_destroy(device);
    return 1;
}
```

完整程序见 [core/samples/](core/samples)。

**Python**

```python
import dstereo_cam

config = dstereo_cam.preset(
    "GS130WI", dstereo_cam.CameraMode.RECT, 1088, 1280, 30, 200
)
with dstereo_cam.Device(config) as device:
    device.start()
    while device.available_camera() == 0:
        pass
    left = device.read_image()["left"]
```

**ROS 2**

ROS 2 包把相机、标定数据与 IMU 以标准消息发布。

```bash
ros2 launch dstereo_cam dstereo_cam.launch.py             # 图像、标定、IMU 与 TF
ros2 launch dstereo_cam dstereo_cam_websocket.launch.py   # 同上，可在浏览器预览
ros2 launch dstereo_cam dstereo_cam_stereonet.launch.py   # 在同一页面看双目深度
```

话题与参数说明见 [ROS 2 README](ros2/src/dstereo_cam/README.zh-CN.md)。

## 特性

- **硬件矫正**：`rect` 模式下的去畸变与立体矫正在 GDC 硬件上完成，无需主机侧图像处理。
- **可选双目布局**：左右目可以分别输出，也可以合成为一张图像。
- **标定来自 EEPROM**：内参、畸变系数、外参与 IMU 参数均从板载 EEPROM 读取。
- **FSYNC 对齐的 IMU**：相机作为 IMU 的时基，两路数据位于同一时钟。
- **标准 ROS 2 接口**：图像、相机信息、惯性数据与静态变换均以标准消息发布，不引入自定义消息类型。
- **版本一致**：原生库与 Python 封装共用 `VERSION` 中声明的版本号。
