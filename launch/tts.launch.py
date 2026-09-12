"""Standalone TTS (no cameras / FCU / nav). Uses SSoT ``tts:`` only.

  ros2 launch hound_core tts.launch.py
"""

import sys
from pathlib import Path

import yaml
from launch import LaunchDescription

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import build_tts_node, find_ssot  # noqa: E402


def generate_launch_description():
    ssot_file = find_ssot()
    with open(ssot_file, "r", encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle) or {}
    tts = dict(ssot.get("tts") or {})
    tts["enabled"] = True
    print(f"[hound_core] tts standalone from {ssot_file}")
    return LaunchDescription([build_tts_node(tts)])
