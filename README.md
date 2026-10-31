# RDK Stereo Camera Module SDK

**English** | [简体中文](README.zh-CN.md)

The RDK Stereo Camera Module SDK supports the RDK stereo camera and its onboard IMU on RDK boards. It consists of a native C library plus Python and ROS 2 wrappers built on it.

> **Developer preview (Alpha).** This release is intended for evaluation. Interfaces, configuration presets and behaviour may change without notice. Feedback and contributions are welcome through the [D-Robotics developer community](https://forum.d-robotics.cc/).

## Overview

The hardware pipeline is implemented once, in the native library: sensor capture and ISP processing, rectification on the GDC hardware, calibration read from the onboard EEPROM, and IMU sampling aligned to the camera time base through FSYNC.

| Layer | Directory | Contents |
| --- | --- | --- |
| Core | `core/` | `libdstereocam`, the public C headers, the `dsc` tools, and Debian packaging |
| Python wrapper | `python/` | The `dstereo_cam` package, which binds `libdstereocam` through `ctypes` |
| ROS 2 wrapper | `ros2/` | The `dstereo_cam` package, which publishes camera, IMU and TF data |

Core must be installed before either wrapper can be used.

## Requirements

- **Hardware**: an RDK development board and a stereo camera module. Some models include an IMU.
- **Core**: the Horizon multimedia libraries, OpenCV 4 and TBB (all shipped with RDK OS).
- **Python wrapper**: Python 3.10 or later and `numpy`. Building the wheel additionally requires `setuptools` and `wheel`.
- **ROS 2 wrapper**: ROS 2 Humble or Jazzy. The web preview and stereo depth launch files also require the TROS packages `hobot_codec`, `websocket` and `hobot_stereonet`.

The current release implements the RDK X5, RDK S100 and RDK S600 backends.

## Repository layout

```text
core/
  include/dstereo_cam.h        Public C API
  include/dstereo_cam_define.h Platform configuration presets
  src/                         Library implementation (base, devices, tools)
  samples/                     dsc front end and sample programs
python/
  dstereo_cam/                 Python wrapper, with type stubs
  test/test_dstereo_cam.py     Hardware test
ros2/src/dstereo_cam/          ROS 2 package: node, launch files, documentation
VERSION                        Version shared by Core and the Python wrapper
LICENSE
```

## Build and installation

### 1. Core

The library, the tools and the Debian package are built with the GNU Makefile in `core/`.

```bash
cd core
make RDKX5
```

Build output is written to `build/<platform>/` and `out/<platform>/`, and the Debian package to `out/`.

```bash
# Check whether the SDK is already installed
dpkg -s dstereo-cam

# If not, install the Debian package built from core/
sudo dpkg -i out/dstereo-cam_<version>+<platform>_<arch>.deb
```

See the [Core README](core/README.md) for details.

### 2. Python wrapper

The wheel is built on the board with the toolchain already present.

```bash
cd python && ./build-wheel.sh
python3 -m pip install dist/dstereo_cam-*.whl
```

The wheel does not bundle the native library; an installed `libdstereocam` is located at run time.

See the [Python README](python/README.md) for details.

### 3. ROS 2 wrapper

```bash
cd ros2
source /opt/ros/humble/setup.bash     # or: source /opt/ros/jazzy/setup.bash
colcon build --packages-select dstereo_cam
source install/setup.bash
```

See the [ROS 2 README](ros2/src/dstereo_cam/README.md) for details.

## Getting started

**Command-line tools**

The `dsc` tools are installed together with Core, and the sample sources are kept in [core/samples/](core/samples). They serve both as diagnostic utilities and as reference implementations.

```bash
dsc help                                     # list the available commands
dsc version                                  # SDK version and build platform
dsc detect eeprom -b 4 6                     # scan I2C buses and addresses
dsc shell -d GS130WI                         # interactive debugging
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

Complete programs are provided in [core/samples/](core/samples).

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

The ROS 2 package publishes the camera, the calibration data and the IMU as standard messages.

```bash
ros2 launch dstereo_cam dstereo_cam.launch.py             # images, calibration, IMU and TF
ros2 launch dstereo_cam dstereo_cam_websocket.launch.py   # the same, previewed in a browser
ros2 launch dstereo_cam dstereo_cam_stereonet.launch.py   # stereo depth on the same page
```

See the [ROS 2 README](ros2/src/dstereo_cam/README.md) for the topic and parameter reference.

## Features

- **Hardware rectification.** In `rect` mode, undistortion and rectification are performed on the GDC hardware, so no host-side image processing is required.
- **Selectable stereo layouts.** The two eyes may be delivered separately or combined into a single frame.
- **Calibration from the EEPROM.** Intrinsics, distortion coefficients, extrinsics and IMU parameters are read from the onboard EEPROM.
- **FSYNC-aligned IMU.** The camera drives the IMU time base, so both streams are expressed on a single clock.
- **Standard ROS 2 interfaces.** Images, camera information, inertial data and static transforms are published as standard messages; no custom message types are introduced.
- **Consistent versioning.** The native library and the Python wrapper share the version declared in `VERSION`.
