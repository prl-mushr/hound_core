#!/usr/bin/env bash
# Run koide3/direct_visual_lidar_calibration (Jazzy image) against HOUND calib data.
#
# Usage:
#   run_vlcal_docker.sh preprocess <camera>
#   run_vlcal_docker.sh initial_guess_manual <camera>
#   run_vlcal_docker.sh calibrate <camera>
#   run_vlcal_docker.sh viewer <camera>
#
# Data layout (host):
#   $CALIB_ROOT/<camera>/bags/
#   $CALIB_ROOT/<camera>/preprocessed/

set -euo pipefail

CMD="${1:-}"
CAM="${2:-}"

# Prefer host workspace path; inside mushr_jazzy the same disk is often /root/colcon_ws.
if [[ -z "${CALIB_ROOT:-}" ]]; then
  if [[ -d /home/hound/colcon_ws/calib/vlcal ]]; then
    CALIB_ROOT=/home/hound/colcon_ws/calib/vlcal
  elif [[ -d /root/colcon_ws/calib/vlcal ]]; then
    CALIB_ROOT=/root/colcon_ws/calib/vlcal
  else
    CALIB_ROOT=/home/hound/colcon_ws/calib/vlcal
  fi
fi
IMAGE="${VLCAL_IMAGE:-koide3/direct_visual_lidar_calibration:jazzy}"
POINTS_TOPIC="${POINTS_TOPIC:-/livox/cloud}"

usage() {
  cat >&2 <<EOF
Usage: $0 <preprocess|initial_guess_manual|calibrate|viewer> <camera_*>

Run this on the HOST (hound), not inside mushr_jazzy — Docker-in-Docker
has no docker.sock.

Env:
  CALIB_ROOT   (default ${CALIB_ROOT})
  VLCAL_IMAGE  (default ${IMAGE})
  POINTS_TOPIC (default ${POINTS_TOPIC})
EOF
  exit 1
}

[[ -n "${CMD}" && -n "${CAM}" ]] || usage

if [[ ! -S /var/run/docker.sock ]]; then
  cat >&2 <<EOF
[vlcal] No Docker socket at /var/run/docker.sock.

You are probably inside mushr_jazzy. Exit to the host and run:

  cd /home/hound/colcon_ws/src/hound_core/scripts/vlcal
  ./run_vlcal_docker.sh ${CMD} ${CAM}

(Record bags inside mushr_jazzy is fine; preprocess/guess/calibrate must be on the host.)
EOF
  exit 1
fi

case "${CAM}" in
  camera_front|camera_left|camera_right) ;;
  *)
    echo "Unknown camera '${CAM}'" >&2
    exit 1
    ;;
esac

BAGS_HOST="${CALIB_ROOT}/${CAM}/bags"
PRE_HOST="${CALIB_ROOT}/${CAM}/preprocessed"
IMAGE_TOPIC="/${CAM}/color/image_raw"
INFO_TOPIC="/${CAM}/color/camera_info"

docker_gui_flags=(
  --rm
  --net host
  -e DISPLAY="${DISPLAY:-}"
  -e QT_X11_NO_MITSHM=1
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
)

# GPU optional — ignore failure if unavailable.
if docker info 2>/dev/null | grep -qi nvidia; then
  docker_gui_flags+=(--gpus all)
fi

if [[ -f "${HOME}/.Xauthority" ]]; then
  docker_gui_flags+=(-e XAUTHORITY=/root/.Xauthority -v "${HOME}/.Xauthority:/root/.Xauthority:ro")
fi

run_vlcal() {
  local inner_cmd=("$@")
  docker run \
    "${docker_gui_flags[@]}" \
    -v "${CALIB_ROOT}:/data" \
    "${IMAGE}" \
    "${inner_cmd[@]}"
}

case "${CMD}" in
  preprocess)
    # Resolve at least one bag directory with metadata (follow symlinks).
    shopt -s nullglob
    bag_ok=0
    for b in "${BAGS_HOST}"/rosbag2_*; do
      if [[ -f "${b}/metadata.yaml" ]] || [[ -f "${b}/metadata.yaml" ]]; then
        bag_ok=1
        break
      fi
      # broken symlink?
      if [[ -L "${b}" ]] && [[ ! -e "${b}" ]]; then
        echo "[vlcal] broken bag symlink: ${b} -> $(readlink "${b}")" >&2
      fi
    done
    if [[ ! -d "${BAGS_HOST}" ]] || [[ "${bag_ok}" -eq 0 ]]; then
      echo "No usable bags in ${BAGS_HOST}" >&2
      echo "Expected e.g. ${BAGS_HOST}/rosbag2_*/metadata.yaml" >&2
      exit 1
    fi
    mkdir -p "${PRE_HOST}"
    echo "[vlcal] preprocess ${CAM}"
    echo "         bags → ${BAGS_HOST}"
    echo "         out  → ${PRE_HOST}"
    run_vlcal ros2 run direct_visual_lidar_calibration preprocess \
      "/data/${CAM}/bags" \
      "/data/${CAM}/preprocessed" \
      --image_topic "${IMAGE_TOPIC}" \
      --camera_info_topic "${INFO_TOPIC}" \
      --points_topic "${POINTS_TOPIC}" \
      -i intensity \
      -v
    ;;
  initial_guess_manual)
    if [[ ! -d "${PRE_HOST}" ]]; then
      echo "Missing ${PRE_HOST} — run preprocess first" >&2
      exit 1
    fi
    echo "[vlcal] initial_guess_manual ${CAM}"
    echo "         Pick ≥3 cloud↔image correspondences, Estimate, Save."
    run_vlcal ros2 run direct_visual_lidar_calibration initial_guess_manual \
      "/data/${CAM}/preprocessed"
    ;;
  calibrate)
    if [[ ! -d "${PRE_HOST}" ]]; then
      echo "Missing ${PRE_HOST} — run preprocess first" >&2
      exit 1
    fi
    echo "[vlcal] calibrate ${CAM}"
    run_vlcal ros2 run direct_visual_lidar_calibration calibrate \
      "/data/${CAM}/preprocessed"
    echo "[vlcal] result: ${PRE_HOST}/calib.json  (results.T_lidar_camera)"
    ;;
  viewer)
    if [[ ! -d "${PRE_HOST}" ]]; then
      echo "Missing ${PRE_HOST} — run preprocess first" >&2
      exit 1
    fi
    echo "[vlcal] viewer ${CAM}"
    run_vlcal ros2 run direct_visual_lidar_calibration viewer \
      "/data/${CAM}/preprocessed"
    ;;
  *)
    usage
    ;;
esac
