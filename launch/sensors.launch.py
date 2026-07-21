"""Always-on sensor tier: ZED driver (NITROS) + HAL monitor.

Start this at boot and leave it running so IMU, barometer, mag, and stereo
frames stabilize before mission software starts.

Mission / algorithms (cuVSLAM, EKF, MAVROS, segmentation) are launched via
hound_core.launch.py, which loads cuVSLAM into the same NITROS container
when perception.use_nitros is true.

Usage:
  ros2 launch hound_core sensors.launch.py
"""

import os
import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import (  # noqa: E402
    SENSORS_CONTAINER_NAME,
    build_hal_monitor_node,
    build_zed_sensors_container,
    build_zed_state_publisher,
    find_ssot,
    zed_share_paths,
)


def generate_launch_description():
    ssot_file = LaunchConfiguration("ssot_file")

    def _setup(context):
        path = ssot_file.perform(context)
        if not path:
            path = find_ssot()
        print(f"[hound_core/sensors] Using SSoT: {path}")
        with open(path, "r", encoding="utf-8") as handle:
            ssot = yaml.safe_load(handle)

        cam = ssot.get("camera", {})
        zed = ssot.get("zed", {})
        hal = ssot.get("hal_monitor", {})
        vesc = ssot.get("vesc", {})
        sensors_cfg = ssot.get("sensors", {})

        camera_enabled = bool(cam.get("enabled", True))
        backend = str(cam.get("backend", "zed")).lower()
        hal_enabled = bool(hal.get("enabled", True))
        vesc_enabled = bool(vesc.get("enabled", False))
        container_name = str(sensors_cfg.get("container_name", SENSORS_CONTAINER_NAME))

        actions = [
            LogInfo(msg=(
                "[hound_core/sensors] Always-on sensor tier — "
                "expect ZED log: Transport summary: IPC=disabled, NITROS=enabled"
            )),
        ]

        if not camera_enabled:
            print("[hound_core/sensors] camera DISABLED — nothing to launch")
            return actions

        if backend != "zed":
            raise RuntimeError(
                f"sensors.launch.py currently supports camera.backend: zed (got {backend!r})"
            )

        paths = zed_share_paths(str(zed.get("camera_model", "zed2i")))
        actions.append(build_zed_state_publisher(zed, paths))
        actions.append(build_zed_sensors_container(zed, container_name=container_name))

        if hal_enabled:
            actions.append(build_hal_monitor_node(hal, cam, zed))
        else:
            print("[hound_core/sensors] hal_monitor DISABLED")

        if vesc_enabled:
            port = str(vesc.get("port", "/dev/ttyACM0"))
            respawn = bool(vesc.get("respawn", True))
            start_delay_s = float(vesc.get("start_delay_s", 0.0))
            print(f"[hound_core/sensors] vesc ENABLED: port={port}")
            vesc_node = Node(
                package="vesc_driver",
                executable="vesc_driver_node",
                name="vesc_driver",
                output="screen",
                parameters=[{"port": port}],
                remappings=[("sensors/core", "/sensors/core")],
                respawn=respawn,
            )
            if start_delay_s > 0.0:
                actions.append(TimerAction(period=start_delay_s, actions=[vesc_node]))
            else:
                actions.append(vesc_node)
        else:
            print("[hound_core/sensors] vesc DISABLED")

        return actions

    return LaunchDescription([
        DeclareLaunchArgument(
            "ssot_file",
            default_value="",
            description="Override path to SSoT.yaml (default: auto-detect)",
        ),
        OpaqueFunction(function=_setup),
    ])
