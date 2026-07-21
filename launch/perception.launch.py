"""Deprecated: use sensors.launch.py + hound_core.launch.py instead.

  ros2 launch hound_core sensors.launch.py   # always-on ZED + HAL
  ros2 launch hound_core hound_core.launch.py  # cuVSLAM (NITROS) + EKF + mission stack
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("hound_core")
    return LaunchDescription([
        LogInfo(msg=(
            "[hound_core/perception] DEPRECATED — use sensors.launch.py + "
            "hound_core.launch.py"
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "sensors.launch.py")
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "hound_core.launch.py")
            )
        ),
    ])
