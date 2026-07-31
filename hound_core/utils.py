"""ROS helpers shared by hound_core nodes (perception math lives in perception_models)."""

from __future__ import annotations

import numpy as np


def to_image(bridge, array: np.ndarray, encoding: str, header):
    """cv2_to_imgmsg + stamp with the given header, in one call."""
    msg = bridge.cv2_to_imgmsg(array, encoding=encoding)
    msg.header = header
    return msg


def stamp_key(sec: int, nanosec: int) -> int:
    """Pack a ROS time into a single int key (for matching frames by stamp)."""
    return int(sec) * 1_000_000_000 + int(nanosec)
