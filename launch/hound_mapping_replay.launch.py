#!/usr/bin/env python3
"""Mapping-only bring-up for rosbag replay.

Starts ``hound_mapping`` with SSoT ``nvblox:`` params. Sensor subscriptions
stay on the recorded topics (lidar / cameras / TF). Mapper pubs go under
``/debug/hound_mapping/*`` so they do not collide with topics in the bag.

Does not start cameras, lidar drivers, EKF, nav, or viz.

Usage (mushr_jazzy)::

  ros2 launch hound_core hound_mapping_replay.launch.py
  ros2 bag play /path/to/bag --clock

  ros2 launch hound_core hound_mapping_replay.launch.py \\
      bag:=/path/to/bag prefix:=debug
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
from hound_launch_common import (  # noqa: E402
    build_hound_mapping_node,
    find_ssot,
)


def _setup(context, *args, **kwargs):
    ssot_file = find_ssot()
    with open(ssot_file, "r", encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle) or {}

    sc = dict(ssot.get("stereo_composite") or {})
    nvblox = dict(ssot.get("nvblox") or {})
    seg = dict(ssot.get("segmentation") or {})
    lidar = dict(ssot.get("lidar") or {})

    prefix = (
        LaunchConfiguration("prefix").perform(context).strip().strip("/") or "debug"
    )
    bag = LaunchConfiguration("bag").perform(context).strip()

    nvblox["bag_replay_mode"] = True
    nvblox["bag_replay_topic_prefix"] = prefix

    if bool(nvblox.get("use_people_mask", False)) and not bool(
        sc.get("align_depth", False)
    ):
        sc = dict(sc)
        sc["align_depth"] = True

    out = f"/{prefix}/hound_mapping"
    acts = [
        SetParameter(name="use_sim_time", value=True),
        LogInfo(
            msg=(
                f"[hound_mapping_replay] mapping only, use_sim_time, "
                f"outputs → {out}/*  (SSoT {ssot_file})"
            )
        ),
        LogInfo(
            msg=(
                f"[hound_mapping_replay] advertised: {out}/local_map "
                f"{out}/tsdf_voxels {out}/tsdf_color_mesh "
                f"{out}/extract_timing_ms  "
                "(no data until bag play --clock; play from THIS same ROS env)"
            )
        ),
        build_hound_mapping_node(nvblox, sc, seg, lidar),
    ]
    if bag:
        acts.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag, "--clock"],
                output="screen",
            )
        )
        acts.append(LogInfo(msg=f"[hound_mapping_replay] playing {bag} --clock"))
    else:
        acts.append(
            LogInfo(
                msg=(
                    "[hound_mapping_replay] in another shell with the same "
                    "ROS env:  ros2 bag play <bag> --clock"
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
                description="Output topic prefix (/{prefix}/hound_mapping/...)",
            ),
            DeclareLaunchArgument(
                "bag",
                default_value="",
                description="Optional rosbag path; played with --clock",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
