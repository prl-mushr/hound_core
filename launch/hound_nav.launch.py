#!/usr/bin/env python3
"""Nav-only bring-up: Dora manager + planner + controller.

Does NOT start cameras, lidar, EKF, mapping, or LL. Expect those from
``hound_core.launch.py`` (or a bag) already publishing:

  /hound_mapping/local_map
  /hound_fcu_control/control_state

Set ``nav.enabled: false`` in SSoT when the core stack is also running, or
this graph will start twice.

Usage (mushr_jazzy)::

  ros2 launch hound_core hound_nav.launch.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import LogInfo

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import build_nav_dora_actions, find_ssot  # noqa: E402


def generate_launch_description():
    ssot_file = find_ssot()
    print(f"[hound_nav] Using SSoT file: {ssot_file}")
    with open(ssot_file, "r", encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle) or {}

    nav = dict(ssot.get("nav") or {})
    if not nav:
        raise RuntimeError(f"SSoT has no nav: block ({ssot_file})")

    map_topic = str(nav.get("local_map_topic", "/hound_mapping/local_map"))
    state_topic = str(nav.get("state_topic", "/hound_fcu_control/control_state"))
    goal_topic = str(nav.get("goal_topic", "/goal_pose"))
    cmd_topic = str(nav.get("cmd_topic", "/hound_nav/cmd_ackermann"))

    if nav.get("enabled", False):
        print(
            "[hound_nav] WARN: nav.enabled is true in SSoT — do not also run "
            "hound_core.launch.py with nav on, or you get two Dora graphs"
        )

    acts = build_nav_dora_actions(nav)
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    f"[hound_nav] standalone (ignores nav.enabled): "
                    f"map={map_topic} state={state_topic} "
                    f"goal={goal_topic} cmd={cmd_topic}"
                )
            ),
            *acts,
        ]
    )
