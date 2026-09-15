"""Loading a ROS camera calibration YAML into a sensor_msgs/CameraInfo.

ROS 2 Humble ships ``camera_info_manager`` only as a C++ library, so a Python driver
that wants to serve calibration data has to parse the file itself. The format is the
long-standing one written by ``camera_calibration``'s ``cameracalibrator.py``:

.. code-block:: yaml

    image_width: 1920
    image_height: 1080
    camera_name: bluerov2_lowlight
    camera_matrix: {rows: 3, cols: 3, data: [...]}
    distortion_model: plumb_bob
    distortion_coefficients: {rows: 1, cols: 5, data: [...]}
    rectification_matrix: {rows: 3, cols: 3, data: [...]}
    projection_matrix: {rows: 3, cols: 4, data: [...]}

If the file is missing or unreadable the caller gets ``None`` and should fall back to
an uncalibrated CameraInfo, which is still useful: consumers can at least learn the
image size.
"""

from __future__ import annotations

from typing import Optional

import yaml
from sensor_msgs.msg import CameraInfo


def uncalibrated_camera_info(width: int, height: int, frame_id: str) -> CameraInfo:
    """A CameraInfo that carries the image size and nothing else.

    All-zero intrinsics are the documented way of saying "this camera is not
    calibrated" - consumers must check for it rather than dividing by fx.
    """
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = int(width)
    info.height = int(height)
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [0.0] * 9
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [0.0] * 12
    return info


def load_camera_info(path: str, frame_id: str) -> Optional[CameraInfo]:
    """Parse a ROS camera calibration YAML file, or return None if it cannot be used."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError):
        return None

    if not isinstance(data, dict):
        return None

    def matrix(key: str, expected: int):
        entry = data.get(key)
        if not isinstance(entry, dict):
            return None
        values = entry.get("data")
        if not isinstance(values, list) or len(values) != expected:
            return None
        return [float(v) for v in values]

    width = data.get("image_width")
    height = data.get("image_height")
    if not isinstance(width, int) or not isinstance(height, int):
        return None

    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = width
    info.height = height
    info.distortion_model = str(data.get("distortion_model", "plumb_bob"))

    k = matrix("camera_matrix", 9)
    r = matrix("rectification_matrix", 9)
    p = matrix("projection_matrix", 12)
    if k is None or p is None:
        return None
    info.k = k
    info.r = r if r is not None else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = p

    distortion = data.get("distortion_coefficients")
    if isinstance(distortion, dict) and isinstance(distortion.get("data"), list):
        info.d = [float(v) for v in distortion["data"]]
    else:
        info.d = [0.0] * 5

    return info
