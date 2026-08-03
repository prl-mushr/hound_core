#!/usr/bin/env bash
# Smoke: stereo_composite + Dora segmentation (+ optional nvblox). No FCU/VESC.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash
set -u

SMOKE_ROOT=/root/colcon_ws/src/hound_core/config/smoke
mkdir -p "${SMOKE_ROOT}"
python3 - <<'PY'
import yaml
from pathlib import Path
src = Path("/root/colcon_ws/src/hound_core/config/SSoT.yaml")
ssot = yaml.safe_load(src.read_text())
for k in ("fcu_control", "vesc", "hal_monitor", "mavros", "ekf", "lidar", "yolo_world"):
    if k in ssot and isinstance(ssot[k], dict):
        ssot[k]["enabled"] = False
ssot["stereo_composite"]["enabled"] = True
ssot["segmentation"]["enabled"] = True
ssot["nvblox"]["enabled"] = True
ssot["nvblox"]["use_lidar"] = False
ssot["nvblox"]["use_depth"] = True
ssot["nvblox"]["use_color"] = True
ssot["launch"]["stage_delay_s"] = 2.0
# find_ssot uses ROS_WORKSPACE/src/hound_core/config/SSoT.yaml
out = Path("/tmp/hound_smoke_ws/src/hound_core/config")
out.mkdir(parents=True, exist_ok=True)
(out / "SSoT.yaml").write_text(yaml.dump(ssot))
print("smoke SSoT ->", out / "SSoT.yaml")
PY

export ROS_WORKSPACE=/tmp/hound_smoke_ws
LOG=/root/colcon_ws/src/hound_core/config/smoke/launch.log
pkill -9 -f "ros2 launch hound_core" 2>/dev/null || true
pkill -9 -f stereo_composite 2>/dev/null || true
pkill -9 -f "dora run" 2>/dev/null || true
sleep 1

ros2 launch hound_core hound_core.launch.py >"${LOG}" 2>&1 &
LPID=$!
echo "launch pid=${LPID} log=${LOG}"
sleep 40

echo "===== launch excerpt ====="
grep -E "stage |stereo_composite ENABLED|segmentation ENABLED|nvblox ENABLED|Error|error|fatal|Traceback|not found|FAILED" "${LOG}" | head -60 || true

echo "===== topics ====="
ros2 topic list | grep -E "color/image|visual_slam|segmentation|nvblox" | head -40 || true

echo "===== hz ====="
timeout 5 ros2 topic hz /camera/color/image_raw 2>&1 | tail -5 || true
timeout 5 ros2 topic hz /visual_slam/tracking/odometry 2>&1 | tail -5 || true
timeout 5 ros2 topic hz /segmentation/refined_people_mask 2>&1 | tail -5 || true

kill "${LPID}" 2>/dev/null || true
pkill -9 -f stereo_composite 2>/dev/null || true
pkill -9 -f "dora run" 2>/dev/null || true
echo "smoke done"
