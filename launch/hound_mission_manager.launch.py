#!/usr/bin/env python3
"""Mission manager only (SSoT ``nav.mission_manager``).

Does NOT start Dora nav, cameras, mapping, or FCU.

GPS mode expects FCU (or a bag):

  /hound_fcu_control/gps/fix
  /hound_fcu_control/gps/fix_type
  /hound_fcu_control/imu
  /hound_fcu_control/mission/gps
  /hound_fcu_control/control_state   # optional, odom-aligns the goal

RViz mode: play a bag with map + TF. RViz 2D Goal Pose → map-frame YAML
(``mission_file``). Path latched on path_viz_topic. ``publish_goals``
republishes the current waypoint on goal_topic when not recording.

Outputs:

  /goal_pose                         # PoseStamped (latched; GPS or publish_goals)
  /hound_nav/mission/waypoints       # Path (odom for GPS, map for RViz)

Ignores ``nav.enabled`` and ``mission_manager.enabled``. Do not run this
together with ``hound_core.launch.py`` / ``hound_nav.launch.py`` if those
already start mission_manager.

Usage (mushr_jazzy)::

  ros2 launch hound_core hound_mission_manager.launch.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import LogInfo

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import build_mission_manager_node, find_ssot  # noqa: E402


def generate_launch_description():
    ssot_file = find_ssot()
    with open(ssot_file, "r", encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle) or {}

    nav = dict(ssot.get("nav") or {})
    mm = dict(nav.get("mission_manager") or {})
    if not mm:
        raise RuntimeError(f"SSoT has no nav.mission_manager: ({ssot_file})")

    node = build_mission_manager_node(nav, mm)
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    f"[hound_mission_manager] standalone (SSoT {ssot_file}); "
                    f"mode={mm.get('mode', 'gps')} "
                    f"wps={mm.get('gps_waypoints_topic', '/hound_fcu_control/mission/gps')} "
                    f"→ {mm.get('goal_topic', '/goal_pose')}"
                )
            ),
            node,
        ]
    )
