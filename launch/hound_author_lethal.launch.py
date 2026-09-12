#!/usr/bin/env python3
"""RViz no-go authoring: map-frame mesh + /initialpose polygons.

Starts a dedicated RViz config (not display_config). Everything is in map;
no bag / TF required.

  ros2 launch hound_core hound_author_lethal.launch.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import find_ssot  # noqa: E402


def _setup(context, *args, **kwargs):
    ssot_file = find_ssot()
    with open(ssot_file, encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle) or {}
    nv = dict(ssot.get("nvblox") or {})
    mesh_pf = dict(ssot.get("mesh_pf") or {})
    cake = str(nv.get("layer_cake_path") or "")
    mesh = str(mesh_pf.get("map_file") or "")
    out = str(nv.get("lethal_map_path") or "").strip()
    if not out and cake:
        out = str(Path(cake).with_suffix(".lethal.yaml"))
    rviz_cfg = Path(__file__).resolve().parent.parent / "config" / "author_lethal.rviz"
    acts = [
        LogInfo(
            msg=(
                f"[hound_author_lethal] mesh={mesh} out={out} rviz={rviz_cfg} "
                "Fixed Frame=map, 2D Pose Estimate"
            )
        ),
        Node(
            package="hound_mapping",
            executable="author_lethal_rviz.py",
            name="lethal_author",
            output="screen",
            parameters=[
                {
                    "mesh_file": mesh,
                    "layercake": cake,
                    "out": out,
                    "frame_id": "map",
                    "grid_frame_id": "map",
                    "close_radius_m": 1.0,
                    "click_topic": "/initialpose",
                    "use_sim_time": False,
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz_lethal_author",
            output="screen",
            arguments=["-d", str(rviz_cfg)],
            parameters=[{"use_sim_time": False}],
        ),
    ]
    return acts


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
