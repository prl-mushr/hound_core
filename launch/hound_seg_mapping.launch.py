#!/usr/bin/env python3
"""Seg + hound_mapping only (SSoT ``segmentation:`` + ``nvblox:``).

Does NOT start cameras, lidar, EKF, nav, or LL. Expect those from
``hound_core.launch.py`` or a bag (RGB, lidar, TF / odom).

Ignores ``segmentation.enabled`` and ``nvblox.enabled`` so this graph can
run while those flags are off in SSoT.

Optional ``prefix`` (default ``debug``) namespaces *outputs* so they do not
collide with topics already in the bag:

  /{prefix}/camera_*/segmentation/...
  /{prefix}/segmentation/...
  /{prefix}/hound_mapping/local_map

Inputs (cameras, /livox/cloud, TF) stay on SSoT names.

Usage (mushr_jazzy)::

  ros2 launch hound_core hound_seg_mapping.launch.py
  ros2 bag play /path/to/bag --clock

  ros2 launch hound_core hound_seg_mapping.launch.py prefix:=debug bag:=/path/to/bag

  # live names (no /debug), wall clock:
  ros2 launch hound_core hound_seg_mapping.launch.py prefix:= use_sim_time:=false

  # log LocalMap + control_state at 1 Hz; Ctrl+C writes start/goal pairs (t+5s):
  ros2 launch hound_core hound_seg_mapping.launch.py log_problems:=true
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
from launch_ros.actions import Node, SetParameter

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import (  # noqa: E402
    apply_output_topic_prefix,
    build_hound_mapping_node,
    build_segmentation_dora_actions,
    find_ssot,
)


def _truthy(s: str) -> bool:
    return str(s).strip().lower() in ("1", "true", "yes", "on")


def _setup(context, *args, **kwargs):
    ssot_file = find_ssot()
    with open(ssot_file, "r", encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle) or {}

    sc = dict(ssot.get("stereo_composite") or {})
    nvblox = dict(ssot.get("nvblox") or {})
    seg = dict(ssot.get("segmentation") or {})
    lidar = dict(ssot.get("lidar") or {})

    prefix = LaunchConfiguration("prefix").perform(context).strip().strip("/")
    bag = LaunchConfiguration("bag").perform(context).strip()
    use_sim = _truthy(LaunchConfiguration("use_sim_time").perform(context))
    if prefix:
        use_sim = True
        nvblox["bag_replay_mode"] = True
        nvblox["bag_replay_topic_prefix"] = prefix
        apply_output_topic_prefix(seg, nvblox, prefix)

    if bool(nvblox.get("use_people_mask", False)) and not bool(
        sc.get("align_depth", False)
    ):
        sc = dict(sc)
        sc["align_depth"] = True

    map_ns = f"/{prefix}/hound_mapping" if prefix else "/hound_mapping"
    acts = []
    if use_sim:
        acts.append(SetParameter(name="use_sim_time", value=True))
    acts.extend(
        [
            LogInfo(
                msg=(
                    f"[hound_seg_mapping] seg + mapping (SSoT {ssot_file}); "
                    f"ignores enabled flags; prefix={prefix or '(none)'} "
                    f"use_sim_time={use_sim}"
                )
            ),
            LogInfo(
                msg=(
                    f"[hound_seg_mapping] mapping pubs → {map_ns}/* ; "
                    "cameras/lidar/TF unchanged"
                    + (
                        f"; seg pubs under /{prefix}/..."
                        if prefix
                        else ""
                    )
                )
            ),
            *build_segmentation_dora_actions(seg),
            build_hound_mapping_node(nvblox, sc, seg, lidar),
        ]
    )
    log_problems = _truthy(LaunchConfiguration("log_problems").perform(context))
    problems_dir = LaunchConfiguration("problems_dir").perform(context).strip()
    if log_problems:
        nav = dict(ssot.get("nav") or {})
        map_topic = f"{map_ns}/local_map"
        state_topic = str(nav.get("state_topic", "/hound_fcu_control/control_state"))
        dims = int(nav.get("control_state_dims", 17))
        params = {
            "use_sim_time": use_sim,
            "local_map_topic": map_topic,
            "state_topic": state_topic,
            "control_state_dims": dims,
            "log_hz": 1.0,
            "goal_horizon_s": 5.0,
        }
        if problems_dir:
            params["problems_dir"] = problems_dir
        acts.append(
            Node(
                package="hound_nav",
                executable="log_planning_problems",
                name="hound_planning_problem_logger",
                output="screen",
                parameters=[params],
            )
        )
        acts.append(
            LogInfo(
                msg=(
                    f"[hound_seg_mapping] problem logger map={map_topic} "
                    f"state={state_topic} dir={problems_dir or '(default timestamped)'}"
                )
            )
        )

    if bag:
        acts.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag, "--clock"],
                output="screen",
            )
        )
        acts.append(LogInfo(msg=f"[hound_seg_mapping] playing {bag} --clock"))
    elif use_sim:
        acts.append(
            LogInfo(
                msg=(
                    "[hound_seg_mapping] play the bag in the same ROS env: "
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
                    "Output namespace (/{prefix}/hound_mapping, "
                    "/{prefix}/.../segmentation). Empty = SSoT names."
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
            DeclareLaunchArgument(
                "log_problems",
                default_value="false",
                description="Log LocalMap+state at 1 Hz; Ctrl+C writes t+5s start/goal pairs",
            ),
            DeclareLaunchArgument(
                "problems_dir",
                default_value="",
                description="Output dir for snapshots/problems (empty = timestamped default)",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
