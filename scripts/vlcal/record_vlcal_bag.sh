#!/usr/bin/env bash
# Record a short stationary bag for vlcal.
#
# Usage:
#   record_vlcal_bag.sh           # ALL topics → calib/vlcal/all/bags/, 15s
#   record_vlcal_bag.sh 5         # ALL topics, 5s
#   record_vlcal_bag.sh camera_front 15   # only livox + that cam
#
# Prefer -a when only lidar + cameras are running. One bag can feed all three
# cameras offline (preprocess uses explicit --image_topic).

set -euo pipefail

CALIB_ROOT="${CALIB_ROOT:-/home/hound/colcon_ws/calib/vlcal}"
POINTS_TOPIC="${POINTS_TOPIC:-/livox/cloud}"

CAM=""
DURATION="15"
if [[ "${1:-}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  DURATION="$1"
elif [[ -n "${1:-}" ]]; then
  CAM="$1"
  DURATION="${2:-15}"
fi

if [[ -n "${CAM}" ]]; then
  case "${CAM}" in
    camera_front|camera_left|camera_right) ;;
    *)
      echo "Unknown camera '${CAM}' (or pass a duration for -a mode)" >&2
      exit 1
      ;;
  esac
  OUT_DIR="${CALIB_ROOT}/${CAM}/bags"
  USE_ALL=0
else
  CAM="all"
  OUT_DIR="${CALIB_ROOT}/all/bags"
  USE_ALL=1
fi

mkdir -p "${OUT_DIR}"
STAMP="$(date +%Y_%m_%d-%H_%M_%S)"
BAG_PATH="${OUT_DIR}/rosbag2_${STAMP}"

echo "[vlcal] mode=${CAM} duration=${DURATION}s"
echo "[vlcal] output: ${BAG_PATH}"
echo "[vlcal] Hold STILL… will force-stop after ${DURATION}s"

set +e
if [[ "${USE_ALL}" -eq 1 ]]; then
  ros2 bag record -o "${BAG_PATH}" -a &
else
  ros2 bag record -o "${BAG_PATH}" --topics \
    "${POINTS_TOPIC}" \
    "/${CAM}/color/image_raw" \
    "/${CAM}/color/camera_info" &
fi
REC_PID=$!
sleep "${DURATION}"
kill -INT "${REC_PID}" 2>/dev/null
for _ in $(seq 1 20); do
  kill -0 "${REC_PID}" 2>/dev/null || break
  sleep 0.5
done
if kill -0 "${REC_PID}" 2>/dev/null; then
  echo "[vlcal] flush stalled — kill -9"
  kill -9 "${REC_PID}" 2>/dev/null || true
  pkill -9 -P "${REC_PID}" 2>/dev/null || true
fi
wait "${REC_PID}" 2>/dev/null
set -e

echo "[vlcal] done: ${BAG_PATH}"
if [[ "${USE_ALL}" -eq 1 ]]; then
  echo "[vlcal] Link into each camera for offline preprocess:"
  echo "  for c in camera_front camera_left camera_right; do"
  echo "    mkdir -p ${CALIB_ROOT}/\$c/bags"
  echo "    ln -sfn ${BAG_PATH} ${CALIB_ROOT}/\$c/bags/\$(basename ${BAG_PATH})"
  echo "  done"
fi
