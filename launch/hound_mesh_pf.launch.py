#!/usr/bin/env python3
"""mesh_pf only (SSoT ``mesh_pf:`` + lidar extrinsics).

Does NOT start lidar, cameras, EKF, or mapping. Expect ``cloud_topic``
from a live stack or bag. Ignores ``mesh_pf.enabled``.

Default ``prefix:=debug`` puts the pose under ``/debug/localization/mesh_pose``
and the node at ``/debug/mesh_pf``. Cloud stays on the SSoT name.

Usage (mushr_jazzy)::

  ros2 launch hound_core hound_mesh_pf.launch.py
  ros2 bag play /path/to/bag --clock

  ros2 launch hound_core hound_mesh_pf.launch.py prefix:=debug bag:=/path/to/bag

  # live names (no /debug), wall clock:
  ros2 launch hound_core hound_mesh_pf.launch.py prefix:= use_sim_time:=false
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import build_mesh_pf_node, find_ssot  # noqa: E402


def _truthy(s: str) -> bool:
    return str(s).strip().lower() in ("1", "true", "yes", "on")


def _setup(context, *args, **kwargs):
    ssot_file = find_ssot()
    with open(ssot_file, "r", encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle) or {}

    mesh_pf = dict(ssot.get("mesh_pf") or {})
    lidar = dict(ssot.get("lidar") or {})
    if not mesh_pf.get("map_file"):
        raise RuntimeError(f"SSoT mesh_pf.map_file empty ({ssot_file})")

    prefix = LaunchConfiguration("prefix").perform(context).strip().strip("/")
    bag = LaunchConfiguration("bag").perform(context).strip()
    use_sim = _truthy(LaunchConfiguration("use_sim_time").perform(context))
    if prefix:
        use_sim = True

    pose = str(mesh_pf.get("pose_topic", "/localization/mesh_pose"))
    if not pose.startswith("/"):
        pose = "/" + pose
    pose_out = f"/{prefix}{pose}" if prefix else pose
    cloud = str(
        mesh_pf.get("cloud_topic")
        or lidar.get("cloud_topic")
        or "/livox/cloud"
    )
    node_ns = f"/{prefix}/mesh_pf" if prefix else "/mesh_pf"

    acts = []
    if use_sim:
        acts.append(SetParameter(name="use_sim_time", value=True))
    acts.extend(
        [
            LogInfo(
                msg=(
                    f"[hound_mesh_pf] mesh_pf only (SSoT {ssot_file}); "
                    f"ignores enabled; prefix={prefix or '(none)'} "
                    f"use_sim_time={use_sim}"
                )
            ),
            LogInfo(
                msg=(
                    f"[hound_mesh_pf] cloud={cloud} (unchanged) "
                    f"pose → {pose_out}  node={node_ns} "
                    f"map={mesh_pf.get('map_file')}"
                )
            ),
            build_mesh_pf_node(mesh_pf, lidar, prefix=prefix),
        ]
    )
    if bag:
        acts.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag, "--clock"],
                output="screen",
            )
        )
        acts.append(LogInfo(msg=f"[hound_mesh_pf] playing {bag} --clock"))
    elif use_sim:
        acts.append(
            LogInfo(
                msg=(
                    "[hound_mesh_pf] play the bag in the same ROS env: "
                    "ros2 bag play <bag> --clock"
                )
            )
        )
    return acts


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "prefix",
                default_value="debug",
                description=(
                    "Output namespace (/{prefix}/localization/mesh_pose, "
                    "/{prefix}/mesh_pf). Empty = SSoT names."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Forced true when prefix is set (bag --clock).",
            ),
            DeclareLaunchArgument(
                "bag",
                default_value="",
                description="Optional rosbag path; played with --clock",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
