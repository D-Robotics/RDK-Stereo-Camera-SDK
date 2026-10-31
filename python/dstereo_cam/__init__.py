"""Python interface for the RDK Stereo Camera and IMU.

Typical use::

    import dstereo_cam

    config = dstereo_cam.preset(
        "GS130WI", dstereo_cam.CameraMode.RECT, 544, 448, 30, 200
    )
    with dstereo_cam.Device(config) as dev:
        dev.start()
        frame = dev.read_image()

``read_image`` and ``read_imu`` return ``None`` when there is nothing to hand
back, so a capture loop only has to test for that.  Everything else raises
:class:`DSCError`, carrying the code the SDK reported.
"""

import importlib.metadata

from . import _abi
from ._config import config
from ._device import Device, DSCError
from ._preset import preset
from ._types import (
    Calibration,
    CameraIndex,
    CameraIntrinsics,
    CameraMode,
    DistModel,
    ErrorCode,
    FifoMode,
    Image,
    ImuIntrinsics,
    ImuPacket,
    ReferenceFrame,
    StereoLayout,
)


def package_version():
    """Return the installed dstereo-cam version, or ``None`` when it is not installed.

    The version is fixed when the wheel is built, so it comes from the package
    metadata rather than from any file in the source tree.
    """
    try:
        return importlib.metadata.version("dstereo-cam")
    except importlib.metadata.PackageNotFoundError:
        return None


__version__ = package_version()

# The native library is opened on first use, not on import; the recorded package
# version is still checked at that point.  See _abi.load.
_abi.set_package_version(__version__)

library_version = _abi.library_version
library_platform = _abi.library_platform

__all__ = [
    "Device",
    "DSCError",
    "Image",
    "ImuPacket",
    "CameraIntrinsics",
    "ImuIntrinsics",
    "Calibration",
    "config",
    "preset",
    "library_version",
    "library_platform",
    "package_version",
    "__version__",
    "ErrorCode",
    "CameraMode",
    "CameraIndex",
    "StereoLayout",
    "FifoMode",
    "DistModel",
    "ReferenceFrame",
]
