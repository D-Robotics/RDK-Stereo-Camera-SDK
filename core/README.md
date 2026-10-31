# RDK Stereo Camera Module SDK Core Interface Reference

**English** | [简体中文](README.zh-CN.md)

## 1 Overview

### 1.1 Introduction

`core` is the core of the RDK Stereo Camera Module SDK. It is written in C/C++, exposes the C API, and implements all of the SDK's functionality. Both the Python package and the ROS 2 wrapper depend on it.

### 1.2 Supported Models and Platforms

| Model | Description | RDK X5 | RDK S100 | RDK S600 |
| --- | --- | --- | --- | --- |
| GS130W | Dual SC132GS, no IMU, landscape 1280×1088, 80 mm baseline | Supported | Supported | Supported |
| GS130WI | Dual SC132GS, IMU model ICM42688-P, portrait 1088×1280, 70 mm baseline | Supported | Supported | Supported |

## 2 Build and Installation

### 2.1 Build

Building is only supported on the platform hardware declared in [1.2 Supported Models and Platforms](#12-supported-models-and-platforms). The following dependencies are required:

- Horizon multimedia libraries (shipped with RDK OS): `libvpf`, `libhbmem`, `libcam` (under `/usr/hobot/lib`)
- OpenCV 4 (shipped with RDK OS)
- TBB

Run `make <platform>` in the `core` directory:

```bash
make RDKX5
```

Before compiling, the platform and version of this build are printed for confirmation:

```console
  ──────────────────────────────────────────────────────────────
    RDK Stereo Camera SDK Core
      platform   RDKX5
      version    1.0.0
  ──────────────────────────────────────────────────────────────

    Check that the build platform matches the platform you will run on.

    Build? [Y/n]
```

Supported platform names: `RDKX5`, `RDKS100`, `RDKS600`. `RDKX5` is used when no platform name is given.

Build output:

```text
build/
└── RDKX5/                                  # object files and dependency files
out/
├── RDKX5/                                  # library, sample programs, diagnostic tools
└── dstereo-cam_<version>+<platform>_<arch>.deb    # Debian package
```

### 2.2 Installation

```bash
sudo dpkg -i out/dstereo-cam_<version>+<platform>_<arch>.deb
```

Installed contents:

- `/usr/include`: `dstereo_cam.h`, `dstereo_cam_define.h`
- `/usr/lib/<multiarch>`: `libdstereocam.a`, `libdstereocam.so*`
- `/usr/bin`: `dsc` and its helper tools

Verify the installation:

```bash
dpkg -s dstereo-cam
```

The Python package and the ROS 2 wrapper depend on the library and headers installed here, so install core first.

## 3 Tools

### 3.1 Command Usage

`dsc` is the SDK's command-line entry point. The usage is `dsc <command> [args]`.

| Command | Description |
| --- | --- |
| `dsc help` | Show help |
| `dsc version` | Print the SDK version and build platform |
| `dsc detect <target> -b <bus...> [-a <addr...>]` | Scan I2C buses and addresses |
| `dsc shell` | Enter interactive debugging mode, see 3.2 |

**dsc version**

```bash
dsc version
```

Prints the SDK version and the build platform.

**dsc detect**

```bash
dsc detect <imu|eeprom|camera> -b <bus...> [-a <addr...>]
```

- `<target>`: one of `imu`, `eeprom`, `camera`
- `-b`: I2C buses to scan, required, for example `-b 4 6`
- `-a`: addresses to probe; defaults are imu `0x68 0x69`, eeprom `0x50`, camera `0x30 0x32`

Examples:

```bash
dsc detect camera -b 4 6
dsc detect eeprom -b 4 6
```

Every command supports `--help`, for example `dsc detect --help`.

The `dsc shell` command locks a device configuration for continuous debugging; see 3.2.

### 3.2 Interactive Debugging (dsc shell)

`dsc shell` first locks a device configuration, then enters an interactive sub-shell where every command reuses that configuration:

```bash
dsc shell -d GS130WI [-m rect] [-w 1088] [-h 1280] [-f 30] [-o 200]
```

- `-d <model>`: device model, required, `GS130W` or `GS130WI`
- `-m <output mode>`: output mode, default `rect`
  - `raw`: original image without distortion correction
  - `resize`: center-cropped and scaled image, aspect ratio preserved
  - `rect`: stereo-rectified image, undistorted and with epipolar-aligned left and right eyes
- `-w <width>`: output width in pixels, default `1088`
- `-h <height>`: output height in pixels, default `1280`
- `-f <fps>`: camera frame rate, default `30`
- `-o <rate>`: IMU sample rate in Hz, default `200`, `200` or `500`

Everything except `-d` may be omitted.

The output size is constrained by the output mode:

- `raw` output mode: width and height must equal the sensor size `1088×1280`, otherwise initialization returns `DSC_PARAM_ERROR`
- `resize` and `rect` output modes: the aspect ratio must divide exactly; common sizes are `1088×1280`, `544×640`, `272×320`, `136×160`, `68×80`

Scaling capability differs per platform: RDK X5 uses VSE, RDK S100 / S600 use the PYM pyramid, down to 1/16 at most.

What it looks like once entered:

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

Each command under the shell has its source in `samples/`. These programs are both called by `dsc shell` and can be run standalone as C sample programs.

**imu-info**

Prints the detected IMU model and its parameters.

The sub-command imu-info is implemented by `samples/dsc-imu-info.c`; see the source for details.

**eeprom-info**

Prints the calibration model in the EEPROM and its contents.

The sub-command eeprom-info is implemented by `samples/dsc-eeprom-info.c`; see the source for details.

**calib-export &lt;dir&gt;**

Exports the calibration as Kalibr `camchain.yaml` and `imu.yaml` into the given directory.

The sub-command calib-export is implemented by `samples/dsc-calib-export.c`; see the source for details.

**run**

Streams continuously and prints the newest camera frame and IMU packet at 50 Hz until Ctrl-C.

The sub-command run is implemented by `samples/dsc-run.c`; see the source for details.

**rec**

Records data by index range and writes it out after Ctrl-C.

```console
rec [-c <from:to>] [-i <from:to>] -o <dir> [--stitch]
```

- `-c`: record the camera frames whose index is in `[from, to)`
- `-i`: record the IMU packets whose index is in `[from, to)`
- `-o`: output directory
- `--stitch`: write one left-right stitched image per frame instead of one image per eye

The sub-command rec is implemented by `samples/dsc-rec.c`; see the source for details.

**help / exit**

Show the command list and leave the shell.

The sub-commands help and exit are implemented by `samples/dsc`; see the source for details.

## 4 API Reference

### 4.1 Header Files and Linking

`dstereo_cam.h` defines the entire SDK API and must be included before use. `dstereo_cam_define.h` provides convenience predefined macros for device initialization (model presets) and is included when needed:

```c
#include "dstereo_cam.h"
#include "dstereo_cam_define.h"
```

Link against `libdstereocam`:

```bash
gcc main.c -ldstereocam
```

### 4.2 Creation and Initialization

**dsc_create**

```c
dsc_device_t *dsc_create();
```

Creates a device handle. No device resource is occupied yet; release the handle with `dsc_destroy()`.

**dsc_init**

```c
dsc_err_t dsc_init(dsc_device_t *dev, const dsc_config_t *cfg);
```

Initializes the device: probes the EEPROM and IMU along the candidate buses in order, reads the calibration, configures the camera and IMU, and sets up the capture pipeline. Streaming stays off after initialization.

- `dev`: handle returned by `dsc_create()`
- `cfg`: fully initialized configuration; field descriptions are in 4.3, or generate it directly from a model preset with the `DSC_CONFIG` macro

Return values:

- `DSC_OK`: success
- `DSC_PARAM_ERROR`: an argument, a configuration field, or the call order is invalid
- `DSC_UNSUPPORTED`: this hardware configuration is not supported
- `DSC_NOT_FOUND`: the stereo camera was not detected
- `DSC_HW_ERROR`: device communication or pipeline setup failed

**dsc_deinit / dsc_destroy**

`dsc_deinit()` releases the device resources, stopping the stream first if it is still running. The handle stays valid afterwards and can be initialized again. `dsc_destroy()` releases the handle itself; call `dsc_deinit()` or at least `dsc_stop()` first.

**Example**

```c
dsc_config_t cfg = DSC_CONFIG("GS130WI", DSC_CAMERA_MODE_RECT, 544, 448, 30, 200);

dsc_device_t *dev = dsc_create();
if (dsc_init(dev, &cfg) != DSC_OK) {
    dsc_destroy(dev);
    return -1;
}

/* ... capture ... */

dsc_deinit(dev);
dsc_destroy(dev);
```

`DSC_CONFIG(model, output mode, width, height, fps, IMU sample rate)` takes the preset for the given model and the current build platform, so the fields need not be filled in by hand.

### 4.3 Configuration

`dsc_config_t` is the complete configuration passed to `dsc_init()` and consists of five parts:

| Field | Description |
| --- | --- |
| `camera_config` | Camera and platform pipeline parameters |
| `imu_config` | IMU discovery and sampling parameters |
| `eeprom_config` | Calibration EEPROM discovery parameters |
| `camera_fifo` | Camera frame FIFO |
| `imu_fifo` | IMU packet FIFO |

The structure is not filled with defaults automatically; unassigned fields keep their zero values. Normally start from a `DSC_CONFIG` preset and change only the fields that need to differ.

**Preset macros**

There are many configuration items, and filling them in one by one is tedious and error-prone. To avoid looking up every field, `dstereo_cam_define.h` predefines one configuration per platform × model, so a single line yields a usable configuration:

- `DSC_CONFIG(model, output mode, width, height, fps, IMU sample rate)`: selects the preset for the current build platform

To customize, start from a preset macro to get a working baseline and then change individual fields:

```c
dsc_config_t cfg = DSC_CONFIG("GS130WI", DSC_CAMERA_MODE_RECT, 544, 448, 30, 200);

cfg.camera_config.fps = 25;   /* camera frame rate */
cfg.camera_fifo.depth = 8;    /* camera frame FIFO depth */
```

The preset values for each platform and model (buses, addresses, timing, and so on) are in `dstereo_cam_define.h`.

**camera_config**

- `bus` / `bus_num`: candidate I2C buses, probed one by one in array order
- `left_addr` / `right_addr`: I2C addresses of the left and right eyes
- `sensor_width` / `sensor_height`: native sensor resolution
- `fps`: camera frame rate
- `line_length` / `frame_length`: sensor timing parameters
- `tuning_file`: path of the ISP tuning file; `NULL` disables loading
- `output_width` / `output_height`: output resolution; in `raw` output mode it must equal the native sensor resolution
- `mode`: output mode, see `-m` in 3.2
- `stereo_layout`: how the two eyes are output; `DSC_STEREO_LAYOUT_NONE` outputs them separately, any other value outputs one stitched frame
- `bus_mipi_rx` / `bus_reset_gpio`: map from I2C bus to MIPI RX and reset GPIO
- `fsync_camera`: which eye the IMU FSYNC is wired to

**imu_config**

- `bus` / `bus_num`: candidate buses; `bus_num` of 0 disables IMU probing
- `addr`: IMU I2C address
- `odr_hz`: sample rate
- `accel_fsr_g` / `gyro_fsr_dps`: accelerometer and gyroscope full-scale ranges
- `accel_bw_sel` / `gyro_bw_sel`: filter settings; values are documented by `dsc_get_imu_info()`

**eeprom_config**

- `bus` / `bus_num`: candidate buses; `bus_num` of 0 disables EEPROM probing
- `addr`: EEPROM I2C address

**camera_fifo / imu_fifo**

- `depth`: FIFO depth, at least 2
- `mode`: policy when the FIFO is full; `DSC_FIFO_DROP_NEW` drops the new item, `DSC_FIFO_DROP_OLD` drops the oldest item

### 4.4 Starting and Stopping the Stream

**dsc_start**

```c
dsc_err_t dsc_start(dsc_device_t *dev);
```

Starts capture. When an IMU is detected, the camera waits for the IMU FSYNC handshake before producing images; without an IMU it starts immediately. A repeated call returns `DSC_PARAM_ERROR`.

- `dev`: initialized handle

Return values:

- `DSC_OK`: the capture threads were started
- `DSC_PARAM_ERROR`: the handle is NULL, not initialized, or already running
- `DSC_HW_ERROR`: the IMU or camera pipeline failed to start

**dsc_stop**

```c
void dsc_stop(dsc_device_t *dev);
```

Stops capture and waits for the background threads to exit. Calling it while not streaming has no effect; `dsc_start()` may be called again afterwards.

### 4.5 Getting Image Data

**dsc_available_camera**

```c
size_t dsc_available_camera(dsc_device_t *dev);
```

Queries the number of image pairs currently available.

**dsc_get_nv12_frame**

```c
dsc_err_t dsc_get_nv12_frame(dsc_device_t *dev,
                             dsc_image_nv12_t *image_left,
                             dsc_image_nv12_t *image_right);
```

Fetches one pair of left and right images, non-blocking. The returned `data` is owned by the caller and must be released with `free()`.

Fields of `dsc_image_nv12_t`:

- `data`: NV12 data, the Y plane followed by interleaved UV
- `width` / `height`: image dimensions in pixels
- `timestamp_ns`: capture timestamp in nanoseconds on the camera clock

Return values:

- `DSC_OK`: a synchronized image pair was returned
- `DSC_PARAM_ERROR`: an argument is NULL, or the camera FIFO is unavailable
- `DSC_UNSUPPORTED`: a stitched output is configured (`stereo_layout` is not `DSC_STEREO_LAYOUT_NONE`)
- `DSC_TIMEOUT`: the FIFO is currently empty
- `DSC_THREAD_CLOSED`: capture is not running
- `DSC_HW_ERROR`: a capture thread exited after a hardware failure

**dsc_get_stereo_nv12_frame**

```c
dsc_err_t dsc_get_stereo_nv12_frame(dsc_device_t *dev,
                                    dsc_image_nv12_t *image);
```

Fetches one stitched left-right image; the layout is fixed by `stereo_layout` at `dsc_init()` time: horizontal layouts double the width, vertical layouts double the height. Also non-blocking, and `data` must be released with `free()`.

The return values are the same as `dsc_get_nv12_frame`, except that it returns `DSC_UNSUPPORTED` when the layout is `DSC_STEREO_LAYOUT_NONE`.

Which of the two to use depends on `stereo_layout` at initialization: use `dsc_get_nv12_frame` to process the two eyes separately, or `dsc_get_stereo_nv12_frame` when a stitched output is configured and one whole image is wanted per call.

**Typical usage**

```c
for (;;) {
    if (dsc_available_camera(dev) > 0) {
        dsc_image_nv12_t left, right;
        if (dsc_get_nv12_frame(dev, &left, &right) == DSC_OK) {
            /* process left.data / right.data, NV12 format */
            free(left.data);
            free(right.data);
        }
    }
}
```

### 4.6 Getting IMU Data

**dsc_available_imu**

```c
size_t dsc_available_imu(dsc_device_t *dev);
```

Queries the number of IMU packets currently available.

**dsc_get_imu_packet**

```c
dsc_err_t dsc_get_imu_packet(dsc_device_t *dev, dsc_imu_packet_t *out);
```

Fetches one IMU packet, non-blocking.

Fields of `dsc_imu_packet_t`:

- `accel[3]`: acceleration in m/s²
- `gyro[3]`: angular velocity in rad/s
- `temp`: temperature in degrees Celsius
- `is_fsync`: whether this packet carries an FSYNC anchor
- `timestamp_ns`: timestamp in nanoseconds on the camera clock

Return values:

- `DSC_OK`: one packet was returned
- `DSC_PARAM_ERROR`: an argument is NULL, or the device has no IMU
- `DSC_TIMEOUT`: the FIFO is currently empty
- `DSC_THREAD_CLOSED`: capture is not running
- `DSC_HW_ERROR`: the IMU thread exited after a hardware failure

Models without an IMU (such as `GS130W`) never return packets and always yield `DSC_TIMEOUT`.

**dsc_get_imu_name / dsc_get_imu_info**

```c
const char *dsc_get_imu_name(dsc_device_t *dev);
const char *dsc_get_imu_info(dsc_device_t *dev);
```

Return the detected IMU model name and detailed information (full-scale ranges, filter settings, and so on). The returned strings are static: do not modify or free them; `NULL` is returned when no IMU was detected.

**Typical usage**

```c
for (;;) {
    if (dsc_available_imu(dev) > 0) {
        dsc_imu_packet_t packet;
        if (dsc_get_imu_packet(dev, &packet) == DSC_OK) {
            /* use packet.accel / packet.gyro / packet.timestamp_ns */
        }
    }
}
```

### 4.7 Getting Calibration Data

The calibration is read from the EEPROM at `dsc_init()` and can be queried at any time afterwards, whether capture is running or not. `DSC_NOT_FOUND` is returned when no EEPROM was detected.

**Camera intrinsics**

```c
dsc_err_t dsc_get_camera_intrinsics(dsc_device_t *dev,
                                    dsc_camera_index_t cam_idx,
                                    dsc_camera_intrinsics_t *intrinsics);
```

- `cam_idx`: `DSC_CAMERA_RIGHT_IDX` or `DSC_CAMERA_LEFT_IDX`
- `fx`, `fy`, `cx`, `cy`: focal lengths and principal point in pixels
- `K[9]`: row-major 3×3 intrinsic matrix
- `dist_coeffs[8]`: distortion coefficients
- `dist_model`: distortion model; `DSC_DIST_PINHOLE` is the OpenCV radtan order, `DSC_DIST_FISHEYE` is the equidistant model

The returned intrinsics depend on the output mode:

- `raw`: native sensor intrinsics
- `resize`: intrinsics rescaled to the output resolution after cropping and scaling; the distortion coefficients are unchanged (the image is still distorted)
- `rect`: virtual intrinsics after rectification, an ideal pinhole with `fx = fy`, `cx = w/2`, `cy = h/2`, and zero distortion

**IMU intrinsics**

```c
dsc_err_t dsc_get_imu_intrinsics(dsc_device_t *dev,
                                 dsc_imu_intrinsics_t *intrinsics);
```

Contains the cross-axis coupling matrices, per-axis scale factors, biases, noise densities, and random walks.

**Relative poses**

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

The two functions are used together and satisfy `p_to = R * p_from + T`. `from_frame` and `to_frame` take `DSC_REF_CAMERA_LEFT`, `DSC_REF_CAMERA_RIGHT`, or `DSC_REF_IMU`.

**Complete calibration**

```c
dsc_err_t dsc_get_calibration(dsc_device_t *dev, dsc_calibration_t *calibration);
```

Returns the whole calibration at once: IMU intrinsics, left and right camera intrinsics, the poses of the left and right cameras and of the IMU, and the installation angle.

**Converting the reference frame**

```c
dsc_err_t dsc_convert_calibration(dsc_device_t *dev,
                                  dsc_reference_frame_t ref_frame,
                                  const double ref_R[9],
                                  const double ref_T[3]);
```

Sets the pose of the device named by `ref_frame` to `(ref_R, ref_T)` and applies the same rigid transform to every other device pose, so all relative poses stay unchanged.

Only the in-memory calibration is changed; the stereo rectification is not recomputed. A later `dsc_init()` reloads the calibration from the EEPROM and overwrites this change.

**EEPROM information**

```c
const char *dsc_get_eeprom_name(dsc_device_t *dev);
const char *dsc_get_eeprom_info(dsc_device_t *dev);
```

Return the model name and detailed information of the calibration EEPROM (calibration vendor, format version, device type, distortion model, and so on). The returned strings are static: do not modify or free them; `NULL` is returned when no EEPROM was detected.

### 4.8 Getting Version and Platform Information

**dsc_version**

```c
const char *dsc_version(void);
```

Returns the SDK version, in `major.minor.patch` form.

**dsc_platform**

```c
const char *dsc_platform(void);
```

Returns the platform name selected at build time, for example `RDKX5`, which is the platform directory name.

Both functions return static strings; do not modify or free them.

```c
printf("dsc %s on %s\n", dsc_version(), dsc_platform());
```
