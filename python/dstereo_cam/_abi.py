"""ctypes ABI mirror of ``core/include/dstereo_cam.h``.

Every declaration in this module corresponds to one in the C header and must
match it byte for byte.  Enum names and values are exposed as ``IntEnum``, but
every ctypes position uses ``C.c_int``: ctypes has no enum type, and a
``c_int`` subclass reads back as a non-comparable object.

This module also loads ``libdstereocam`` and applies the declarations to it.  It
contains no other policy; ownership and error handling belong to ``device``.

The SDK allocates ``dsc_image_nv12_t.data`` with ``malloc`` and transfers
ownership to the caller, which must release every non-null buffer with the
same C runtime (``ctypes.CDLL(None).free``).
"""

from __future__ import annotations

import ctypes as C
import ctypes.util
import os
import warnings
from enum import IntEnum


_DEFAULT = "libdstereocam.so"
_library = None
_package_version = None


# ---------------------------------------------------------------------------
# Enums.  Values are copied from dstereo_cam.h and are ABI facts.
# ---------------------------------------------------------------------------


class dsc_err_t(IntEnum):
    DSC_OK = 0
    DSC_PARAM_ERROR = 1
    DSC_UNSUPPORTED = 2
    DSC_NOT_FOUND = 3
    DSC_HW_ERROR = 4
    DSC_TIMEOUT = 5
    DSC_THREAD_CLOSED = 6


class dsc_fifo_mode_t(IntEnum):
    DSC_FIFO_DROP_NEW = 0
    DSC_FIFO_DROP_OLD = 1


class dsc_camera_mode_t(IntEnum):
    DSC_CAMERA_MODE_RAW = 0
    DSC_CAMERA_MODE_RESIZE = 1
    DSC_CAMERA_MODE_RECT = 2


class dsc_camera_index_t(IntEnum):
    DSC_CAMERA_RIGHT_IDX = 0
    DSC_CAMERA_LEFT_IDX = 1


class dsc_stereo_layout_t(IntEnum):
    DSC_STEREO_LAYOUT_NONE = 0
    DSC_STEREO_LAYOUT_LEFT_RIGHT = 1
    DSC_STEREO_LAYOUT_RIGHT_LEFT = 2
    DSC_STEREO_LAYOUT_TOP_BOTTOM = 3
    DSC_STEREO_LAYOUT_BOTTOM_TOP = 4


class dsc_dist_model_t(IntEnum):
    DSC_DIST_PINHOLE = 0
    DSC_DIST_FISHEYE = 1


class dsc_reference_frame_t(IntEnum):
    DSC_REF_CAMERA_RIGHT = 0
    DSC_REF_CAMERA_LEFT = 1
    DSC_REF_IMU = 2


# ---------------------------------------------------------------------------
# Configuration types
# ---------------------------------------------------------------------------


class dsc_fifo_config_t(C.Structure):
    _fields_ = [
        ("depth", C.c_size_t),  # queue depth, >= 2
        # dsc_fifo_mode_t
        ("mode", C.c_int),
    ]


class dsc_camera_config_t(C.Structure):
    _fields_ = [
        ("bus", C.c_uint8 * 32),  # candidate I2C buses, probed in order
        ("bus_num", C.c_size_t),
        ("left_addr", C.c_uint8),
        ("right_addr", C.c_uint8),
        ("sensor_width", C.c_uint32),
        ("sensor_height", C.c_uint32),
        ("fps", C.c_uint32),
        ("line_length", C.c_uint32),
        ("frame_length", C.c_uint32),
        ("tuning_file", C.c_char_p),  # NULL = do not load
        ("output_width", C.c_uint32),
        ("output_height", C.c_uint32),
        # dsc_camera_mode_t
        ("mode", C.c_int),
        # dsc_stereo_layout_t
        ("stereo_layout", C.c_int),
        ("bus_mipi_rx", C.c_uint8 * 32),  # I2C bus -> MIPI RX, 0xFF = unset
        ("bus_reset_gpio", C.c_int * 32),  # I2C bus -> reset GPIO, -1 = unset
        # dsc_camera_index_t
        ("fsync_camera", C.c_int),
    ]


class dsc_imu_config_t(C.Structure):
    _fields_ = [
        ("bus", C.c_uint8 * 32),
        ("bus_num", C.c_size_t),
        ("addr", C.c_uint8),
        ("odr_hz", C.c_uint32),
        ("accel_fsr_g", C.c_uint16),
        ("gyro_fsr_dps", C.c_uint16),
        ("accel_bw_sel", C.c_uint8),  # UI filter level 0..7
        ("gyro_bw_sel", C.c_uint8),  # 0xFF = not set
    ]


class dsc_eeprom_config_t(C.Structure):
    _fields_ = [
        ("bus", C.c_uint8 * 32),
        ("bus_num", C.c_size_t),
        ("addr", C.c_uint8),
    ]


class dsc_config_t(C.Structure):
    _fields_ = [
        ("camera_config", dsc_camera_config_t),
        ("imu_config", dsc_imu_config_t),
        ("eeprom_config", dsc_eeprom_config_t),
        ("camera_fifo", dsc_fifo_config_t),
        ("imu_fifo", dsc_fifo_config_t),
    ]


# ---------------------------------------------------------------------------
# Opaque device handle
# ---------------------------------------------------------------------------


class dsc_device_t(C.Structure):
    pass


dsc_device_p = C.POINTER(dsc_device_t)


# ---------------------------------------------------------------------------
# Camera and IMU data
# ---------------------------------------------------------------------------


class dsc_image_nv12_t(C.Structure):
    _fields_ = [
        ("data", C.POINTER(C.c_uint8)),  # tightly packed NV12, malloc'd
        ("width", C.c_uint32),
        ("height", C.c_uint32),
        ("timestamp_ns", C.c_uint64),
    ]


class dsc_imu_packet_t(C.Structure):
    _fields_ = [
        ("accel", C.c_float * 3),  # m/s^2
        ("gyro", C.c_float * 3),  # rad/s
        ("temp", C.c_float),  # degC
        ("is_fsync", C.c_bool),
        ("timestamp_ns", C.c_uint64),
    ]


# ---------------------------------------------------------------------------
# Calibration types
# ---------------------------------------------------------------------------


class dsc_camera_intrinsics_t(C.Structure):
    _fields_ = [
        ("fx", C.c_double),
        ("fy", C.c_double),
        ("cx", C.c_double),
        ("cy", C.c_double),
        ("K", C.c_double * 9),
        ("dist_coeffs", C.c_double * 8),
        # dsc_dist_model_t
        ("dist_model", C.c_int),
    ]


class dsc_imu_intrinsics_t(C.Structure):
    _fields_ = [
        ("accel_misalign", C.c_double * 9),
        ("accel_scale", C.c_double * 3),
        ("accel_bias", C.c_double * 3),
        ("accel_noise", C.c_double),
        ("accel_random_walk", C.c_double),
        ("gyro_misalign", C.c_double * 9),
        ("gyro_scale", C.c_double * 3),
        ("gyro_bias", C.c_double * 3),
        ("gyro_noise", C.c_double),
        ("gyro_random_walk", C.c_double),
    ]


class dsc_calibration_t(C.Structure):
    _fields_ = [
        ("imu", dsc_imu_intrinsics_t),
        ("camera_right", dsc_camera_intrinsics_t),
        ("camera_left", dsc_camera_intrinsics_t),
        ("camera_right_R", C.c_double * 9),
        ("camera_right_T", C.c_double * 3),
        ("camera_left_R", C.c_double * 9),
        ("camera_left_T", C.c_double * 3),
        ("imu_R", C.c_double * 9),
        ("imu_T", C.c_double * 3),
        ("camera_install_angle", C.c_int),
    ]


# ---------------------------------------------------------------------------
# Loading and binding
# ---------------------------------------------------------------------------


def bind(lib):
    """Attach the declarations from ``dstereo_cam.h`` to a loaded library."""

    lib.dsc_version.argtypes = []
    lib.dsc_version.restype = C.c_char_p

    lib.dsc_platform.argtypes = []
    lib.dsc_platform.restype = C.c_char_p

    lib.dsc_create.argtypes = []
    lib.dsc_create.restype = dsc_device_p

    lib.dsc_destroy.argtypes = [dsc_device_p]
    lib.dsc_destroy.restype = None

    lib.dsc_init.argtypes = [dsc_device_p, C.POINTER(dsc_config_t)]
    lib.dsc_init.restype = C.c_int

    lib.dsc_deinit.argtypes = [dsc_device_p]
    lib.dsc_deinit.restype = C.c_int

    lib.dsc_start.argtypes = [dsc_device_p]
    lib.dsc_start.restype = C.c_int

    lib.dsc_stop.argtypes = [dsc_device_p]
    lib.dsc_stop.restype = None

    lib.dsc_available_camera.argtypes = [dsc_device_p]
    lib.dsc_available_camera.restype = C.c_size_t

    lib.dsc_get_nv12_frame.argtypes = [
        dsc_device_p,
        C.POINTER(dsc_image_nv12_t),
        C.POINTER(dsc_image_nv12_t),
    ]
    lib.dsc_get_nv12_frame.restype = C.c_int

    lib.dsc_get_stereo_nv12_frame.argtypes = [
        dsc_device_p,
        C.POINTER(dsc_image_nv12_t),
    ]
    lib.dsc_get_stereo_nv12_frame.restype = C.c_int

    lib.dsc_available_imu.argtypes = [dsc_device_p]
    lib.dsc_available_imu.restype = C.c_size_t

    lib.dsc_get_imu_packet.argtypes = [
        dsc_device_p,
        C.POINTER(dsc_imu_packet_t),
    ]
    lib.dsc_get_imu_packet.restype = C.c_int

    lib.dsc_get_imu_name.argtypes = [dsc_device_p]
    lib.dsc_get_imu_name.restype = C.c_char_p

    lib.dsc_get_imu_info.argtypes = [dsc_device_p]
    lib.dsc_get_imu_info.restype = C.c_char_p

    lib.dsc_get_camera_intrinsics.argtypes = [
        dsc_device_p,
        C.c_int,  # dsc_camera_index_t
        C.POINTER(dsc_camera_intrinsics_t),
    ]
    lib.dsc_get_camera_intrinsics.restype = C.c_int

    lib.dsc_get_imu_intrinsics.argtypes = [
        dsc_device_p,
        C.POINTER(dsc_imu_intrinsics_t),
    ]
    lib.dsc_get_imu_intrinsics.restype = C.c_int

    lib.dsc_get_relative_R.argtypes = [
        dsc_device_p,
        C.c_int,  # dsc_reference_frame_t
        C.c_int,  # dsc_reference_frame_t
        C.POINTER(C.c_double),
    ]
    lib.dsc_get_relative_R.restype = C.c_int

    lib.dsc_get_relative_T.argtypes = [
        dsc_device_p,
        C.c_int,
        C.c_int,
        C.POINTER(C.c_double),
    ]
    lib.dsc_get_relative_T.restype = C.c_int

    lib.dsc_get_calibration.argtypes = [
        dsc_device_p,
        C.POINTER(dsc_calibration_t),
    ]
    lib.dsc_get_calibration.restype = C.c_int

    lib.dsc_convert_calibration.argtypes = [
        dsc_device_p,
        C.c_int,  # dsc_reference_frame_t
        C.POINTER(C.c_double),
        C.POINTER(C.c_double),
    ]
    lib.dsc_convert_calibration.restype = C.c_int

    lib.dsc_get_eeprom_name.argtypes = [dsc_device_p]
    lib.dsc_get_eeprom_name.restype = C.c_char_p

    lib.dsc_get_eeprom_info.argtypes = [dsc_device_p]
    lib.dsc_get_eeprom_info.restype = C.c_char_p

    return lib


def _resolve():
    """Return the library path: $DSC_LIB, then ldconfig, then the default."""
    return os.environ.get("DSC_LIB") or ctypes.util.find_library("dstereocam") or _DEFAULT


def _open(path):
    # libdstereocam declares its Horizon dependencies (libvpf, libhbmem, libcam) as
    # DT_NEEDED with RUNPATH=/usr/hobot/lib, so the dynamic loader resolves
    # them; no manual preload is needed.
    library = C.CDLL(path)
    try:
        bind(library)
    except AttributeError as exc:
        raise OSError(
            "%s does not provide the dsc_* symbols (%s); the loaded library is "
            "not libdstereocam, set DSC_LIB to the right one" % (path, exc)
        ) from None
    return library


def _version_tuple(text):
    numbers = []
    for part in text.split("+", 1)[0].split("-", 1)[0].split("."):
        if not part.isdigit():
            break
        numbers.append(int(part))
    return tuple(numbers)


def _check_version(package, library_version):
    """Compare the package with the library that was just loaded.

    A newer library may have moved a field that this module mirrors, which
    would corrupt memory instead of failing, so it is refused.  An older one is
    only worth a warning: its missing symbols are caught when they are bound.

    ``package`` is ``None`` when dsc is not installed, which skips the check.
    """
    if package is None or package == library_version:
        return
    if _version_tuple(package) > _version_tuple(library_version):
        warnings.warn(
            "dsc package %s is newer than libdstereocam %s"
            % (package, library_version),
            RuntimeWarning,
            stacklevel=3,
        )
    else:
        raise RuntimeError(
            "dsc package %s is older than libdstereocam %s; the package mirrors "
            "the C structures and would misread them"
            % (package, library_version)
        )


def set_package_version(version):
    """Record the package version checked the first time libdstereocam is loaded."""
    global _package_version
    _package_version = version


def load(version=None):
    """Return the process-wide libdstereocam handle with all declarations applied.

    Importing :mod:`dsc` only records its package version.  The native library
    is opened on this first real use, so enums, configuration helpers and type
    information remain usable on a development machine without the hardware
    runtime.  The compatibility check still runs exactly once when the library
    is opened.
    """
    global _library
    if _library is None:
        library = _open(_resolve())
        package = _package_version if version is None else version
        if package is not None:
            _check_version(package, library.dsc_version().decode())
        _library = library
    return _library


def library_version():
    """Return the loaded libdstereocam version string, or ``None``."""
    value = load().dsc_version()
    return value.decode() if value else None


def library_platform():
    """Return the platform libdstereocam was built for, such as ``"RDKX5"``."""
    value = load().dsc_platform()
    return value.decode() if value else None
