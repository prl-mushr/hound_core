#!/usr/bin/env python3
"""Isolated latency experiment: stock RealSense (+ optional MAVROS) + probe.

Diagnostic only — mission bring-up uses composite_sensing stereo_composite.
Does NOT start stereo_composite / FCU. Camera identity/rates come from SSoT
``stereo_composite``.

D455 note: IR and depth share the depth module, so both run at infra_fps (60).

Usage (mushr_jazzy, camera free — stop hound_core / stereo_composite first):

  ros2 launch hound_core hw_component_latency.launch.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import find_ssot, realsense_camera_params  # noqa: E402


def _load_ssot() -> dict:
    path = find_ssot()
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _launch_setup(context, *args, **kwargs):
    ssot = _load_ssot()
    sc = dict(ssot.get("stereo_composite") or {})

    # Experiment rates: IR@60, color@30; depth shares IR module → 60.
    infra_fps = int(
        context.launch_configurations.get("infra_fps")
        or sc.get("infra_fps", 60)
    )
    color_fps = int(
        context.launch_configurations.get("color_fps")
        or sc.get("color_fps", 30)
    )

    cam = {
        **sc,
        "enabled": True,
        "camera_name": sc.get("camera_name", "camera"),
        "fps": infra_fps,
        "infra_width": int(sc.get("infra_width", 640)),
        "infra_height": int(sc.get("infra_height", 360)),
        "depth_fps": infra_fps,
        "depth_width": int(sc.get("infra_width", 640)),
        "depth_height": int(sc.get("infra_height", 360)),
        "color_width": int(sc.get("color_width", 640)),
        "color_height": int(sc.get("color_height", 360)),
        "color_fps": color_fps,
        "enable_color": True,
        "enable_depth": True,
        "enable_sync": False,
        "align_depth": False,
        "emitter_enabled": int(sc.get("emitter_enabled", 0)),
        "enable_imu": False,
        "serial_number": sc.get("serial_number", ""),
    }

    streams = {
        "enable_infra1": True,
        "enable_infra2": True,
        "enable_depth": True,
        "enable_color": True,
    }
    params = realsense_camera_params(cam, streams=streams)

    enable_camera = LaunchConfiguration("enable_camera").perform(context)
    enable_mavros_probe = LaunchConfiguration("enable_mavros_probe").perform(context)

    actions = [
        LogInfo(
            msg=(
                f"[hw_component_latency] stock realsense2_camera: "
                f"IR+depth {cam['infra_width']}x{cam['infra_height']}@{infra_fps} "
                f"color {cam.get('color_width', 640)}x{cam.get('color_height', 360)}"
                f"@{color_fps} sync=false "
                f"(depth module FPS must match IR on D455)"
            )
        ),
    ]

    if enable_camera.lower() in ("1", "true", "yes"):
        actions.append(
            Node(
                name=cam["camera_name"],
                namespace="",
                package="realsense2_camera",
                executable="realsense2_camera_node",
                output="screen",
                parameters=[params],
            )
        )

    actions.append(
        Node(
            package="hound_core",
            executable="hw_latency_probe",
            name="hw_latency_probe",
            output="screen",
            parameters=[
                {
                    "report_period_s": float(
                        LaunchConfiguration("report_period_s").perform(context)
                    ),
                    "camera_name": cam["camera_name"],
                    "enable_realsense": True,
                    "enable_mavros": enable_mavros_probe.lower()
                    in ("1", "true", "yes"),
                }
            ],
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="Start stock realsense2_camera_node",
            ),
            DeclareLaunchArgument(
                "enable_mavros_probe",
                default_value="true",
                description=(
                    "Subscribe to MAVROS imu/mag/baro/gps (idle until FCU up)"
                ),
            ),
            DeclareLaunchArgument(
                "infra_fps",
                default_value="",
                description="Override IR/depth module FPS (default SSoT or 60)",
            ),
            DeclareLaunchArgument(
                "color_fps",
                default_value="",
                description="Override color FPS (default SSoT or 30)",
            ),
            DeclareLaunchArgument(
                "report_period_s",
                default_value="2.0",
                description="Probe summary print period",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
