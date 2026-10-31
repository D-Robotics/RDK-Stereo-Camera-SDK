# RDK 双目摄像头模组 SDK —— ROS 2 接口

**简体中文** | [English](README.md)

> 轻量的 ROS 2 驱动，直接调用 SDK 的 C 接口，把 RDK 双目摄像头及其板载 IMU 送到 ROS 2 话题上。

## 概述

`dstereo_cam` 把 RDK 双目摄像头和 IMU 以**标准 ROS 2 消息**发布。节点直接调用 SDK 的 C 接口（链接 `libdstereocam.so`），自身不做图像处理、立体匹配或滤波；`rect` 模式下的去畸变与立体矫正由 SDK 在 GDC 硬件上完成。

| 项目 | 说明 |
| --- | --- |
| 语言 | C++17（节点）+ C99（SDK 预设桥接） |
| 构建 | `ament_cmake` / `colcon` |
| 平台 | RDK X5、RDK S100、RDK S600 |
| 依赖 | ROS 2 Humble 或 Jazzy、`libdstereocam.so` |

## 依赖

- **硬件**：一块 RDK 系列开发板与一个双目摄像头模组（部分型号带 IMU）。当前支持 **RDK X5**、**RDK S100**、**RDK S600**。
- **ROS 2**：Humble 或 Jazzy。先 source 板上安装的发行版，TROS 系统上再 source TROS 的 overlay：

  ```bash
  source /opt/ros/humble/setup.bash     # 或：source /opt/ros/jazzy/setup.bash
  source /opt/tros/humble/setup.bash    # 仅 TROS，发行版与上面保持一致
  ```

- **SDK**：`dstereo-cam` 包，提供 `libdstereocam.so`、`libdstereocam.a`、`dstereo_cam.h` 与 `dstereo_cam_define.h`，需要单独安装。
- **可选**：TROS 的 `hobot_codec` 与 `websocket`（网页预览），以及 `hobot_stereonet`（双目深度）。

## 构建

**1. 先安装 SDK**

```bash
# 检查是否已安装
dpkg -s dstereo-cam

# 未安装时，安装 core/ 构建出的 deb（在 SDK 仓库里执行 make deb）
sudo dpkg -i dstereo-cam_<版本>+<平台>_<架构>.deb
```

**2. 构建 ROS 2 包**

```bash
cd ros2
source /opt/ros/humble/setup.bash     # 或：source /opt/ros/jazzy/setup.bash
colcon build --packages-select dstereo_cam
source install/setup.bash
```

在仓库检出旁构建（而不是已安装的 SDK）时，CMake 会从 `core/include` 取头文件、按名字查找 `libdstereocam`。目录结构不同时可显式指定两个路径：

```bash
colcon build --packages-select dstereo_cam --cmake-args \
  -DDSC_LIBRARY=/path/to/libdstereocam.so \
  -DDSC_INCLUDE_DIR=/path/to/include
```

## 运行节点

```bash
# 直接运行节点
ros2 run dstereo_cam dstereo_cam_node --ros-args -p camera_mode:=rect -p stitch:=top_bottom

# 或使用基础 launch 文件（推荐）
ros2 launch dstereo_cam dstereo_cam.launch.py
```

可执行文件是 `dstereo_cam_node`，节点名是 `dsc`。参数非法或 SDK 初始化失败时，节点会打印错误并退出。

## 话题

`stitch` 不为 `none` 时，左右目合成一帧：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `image_combine` | `sensor_msgs/Image` | 合成帧，`nv12` |
| `image_combine/left/camera_info` | `sensor_msgs/CameraInfo` | 左目内参，按单目尺寸 |
| `image_combine/right/camera_info` | `sensor_msgs/CameraInfo` | 右目内参，`P[3]` 携带基线 |
| `image_combine/gray` | `sensor_msgs/Image` | 可选，`mono8` |

`stitch: none` 时，左右目各自成帧：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `image_left` / `image_right` | `sensor_msgs/Image` | 左 / 右目，`nv12` |
| `image_left/camera_info` / `image_right/camera_info` | `sensor_msgs/CameraInfo` | 左 / 右目内参 |
| `image_left/gray` / `image_right/gray` | `sensor_msgs/Image` | 可选，`mono8` |

上面的图像话题名都是参数；`camera_info` 的话题名由它描述的图像话题派生，不是参数。

两种情况下节点还会发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/imu_data` | `sensor_msgs/Imu` | 角速度与线加速度；无 IMU 的设备不发布 |
| `/tf_static` | `tf2_msgs/TFMessage` | `camera_link` → `camera_right_link`，有 IMU 时再加 `imu_link` |

`/tf_static` 是 ROS 保留话题，需要移动时请重映射。

## 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `device` | `GS130WI` | 相机型号：`GS130WI` 或 `GS130W` |
| `camera_mode` | `rect` | `raw` / `resize` / `rect` |
| `stitch` | `none` | `none` / `left_right` / `right_left` / `top_bottom` / `bottom_top` |
| `output_width` | `544` | 单目输出宽度，单位像素 |
| `output_height` | `448` | 单目输出高度，单位像素 |
| `fps` | `30` | 相机帧率 |
| `odr` | `200` | IMU 采样率，单位 Hz |
| `publish_gray` | `false` | 额外发布每帧的 `mono8` 图像 |
| `image_topic` | `image_combine` | 合成帧话题（`stitch` 不为 `none` 时使用） |
| `left_image_topic` | `image_left` | 左目话题（`stitch` 为 `none` 时使用） |
| `right_image_topic` | `image_right` | 右目话题（`stitch` 为 `none` 时使用） |
| `imu_topic` | `/imu_data` | IMU 话题 |
| `timer_period_ms` | `1` | 发布定时器周期，单位毫秒 |
| `frame_id` | `camera_link` | 左目与合成帧的坐标系 |
| `right_frame_id` | `camera_right_link` | 右目的坐标系 |
| `imu_frame_id` | `imu_link` | IMU 采样的坐标系 |
| `tuning_file` | 空 | ISP 效果库；**空表示沿用平台预设**（S100 上为 `lib_sc132gs_linear.so`），`disable` 表示**不加载**调优，其它值按名字在 `/usr/hobot/lib/sensor/` 下加载，无需完整路径 |

`output_width`、`output_height`、`fps`、`odr` 必须是正整数，`timer_period_ms` 必须为正。`camera_mode` 与 `stitch` 必须是上面列出的取值之一，否则节点启动即失败。

## 特性

- **硬件矫正**：`rect` 模式下由 SDK 在 GDC 硬件上完成去畸变与立体矫正，节点自身不做图像处理。
- **可选双目布局**：左右目分开输出，另有 4 种拼接布局，共 5 种。
- **输出标定**：存在 EEPROM 时，节点发布两目的 `CameraInfo`（右目的 `P[3]` 携带基线）与静态变换。RAW 模式仍能出图但没有标定，因此不发布 `CameraInfo` 与 TF；RESIZE 与 RECT 需要标定。
- **可选 IMU**：设备带 IMU 时发布角速度与线加速度；按 `sensor_msgs/Imu` 惯例把 `orientation` 标为不可用（协方差首元素为 `-1`）。
- **有序发布**：单个定时器每个周期只发一条消息，按 SDK 时间戳交错发布图像与 IMU 采样，既保证采样顺序，也避免浅队列丢掉尾部。
- **时间戳**：消息时间戳取自 SDK 的设备时钟。第一条消息把该时钟锚定到 ROS 时间，之后沿用同一偏移，因此相机与 IMU 的时间戳保持相对时序。
- **QoS**：图像队列深度 5、IMU 队列深度 10，与 `hobot_mipi_cam` 的惯例一致。

不提供：零拷贝（共享内存上的 `hbm_img_msgs`）、自定义消息、IMU 滤波、动态参数与多设备同步。

## launch 文件

### 1. `dstereo_cam.launch.py` —— 基础启动与参数映射

```bash
ros2 launch dstereo_cam dstereo_cam.launch.py
ros2 launch dstereo_cam dstereo_cam.launch.py stitch:=top_bottom publish_gray:=true
ros2 launch dstereo_cam dstereo_cam.launch.py --show-args      # 列出全部参数与默认值
```

映射规则：**每个 launch 参数与同名节点参数一一对应**。launch 文件还声明了每个参数的目标类型（整数或布尔），命令行给的值会先转换成节点期望的类型再传下去；默认值与上表一致。

### 2. `dstereo_cam_websocket.launch.py` —— 网页预览

```bash
ros2 launch dstereo_cam dstereo_cam_websocket.launch.py
```

该文件包含基础 launch，并额外启动 `hobot_codec`（`nv12` → `jpeg`）与 `websocket` 推流节点（同时拉起 nginx）。在浏览器打开：

```text
http://<板卡 IP>:8000
```

- `stitch` 不为 `none`：合成帧走通道 0。
- `stitch: none`：左目走通道 0，右目走通道 1。

影响预览的参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `stitch` | `none` | `none` 预览两目各自的通道；其它值预览合成帧 |
| `image_topic` | `image_combine` | `stitch` 不为 `none` 时预览的话题 |
| `left_image_topic` | `image_left` | `stitch` 为 `none` 时预览的左目话题 |
| `right_image_topic` | `image_right` | `stitch` 为 `none` 时预览的右目话题 |
| `fps` | `30` | 交给编码器的帧率 |

它不声明自己的参数：上面列出的相机参数都接受，并原样传给节点。

### 3. `dstereo_cam_stereonet.launch.py` —— 双目深度

```bash
ros2 launch dstereo_cam dstereo_cam_stereonet.launch.py
```

该文件包含基础 launch（把 `stitch` 固定为 `top_bottom`，因为 `hobot_stereonet` 要求左目在上半部分），并启动 `hobot_stereonet` 深度模型与可视化节点。彩色深度图同样在浏览器里查看：

```text
http://<板卡 IP>:8000
```

需要安装 `hobot_stereonet`（TROS）。它自己的参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `stereonet_model` | `DStereoV2.4_int8_544_448.bin` | `hobot_stereonet/config` 下的模型文件，按单目尺寸 |
| `render_type` | `indoor` | 深度着色：`indoor`、`outdoor`、`indoor-reverse`、`outdoor-reverse`、`distance`、`distance-reverse` |

模型输入是**单目** `544×448`，与默认的 `output_width` / `output_height` 对应，因此换模型时两个尺寸要一起改。相机参数同样被接受；`image_topic`、`frame_id`、`right_frame_id` 会转发给 `hobot_stereonet`。
