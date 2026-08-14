#!/usr/bin/env bash
# Run vlcal natively inside mushr_jazzy (aarch64). Official Hub image is amd64-only.
#
# Usage (inside mushr_jazzy, after colcon build):
#   source /opt/ros/jazzy/setup.bash
#   source /root/colcon_ws/install/setup.bash
#   ./run_vlcal.sh use_bag rosbag2_2026_08_11-01_48_52   # link into all cams
#   ./run_vlcal.sh preprocess camera_front [bag]
#   ./run_vlcal.sh initial_guess_ssot camera_front
#   ./run_vlcal.sh calibrate camera_front

set -euo pipefail

# GTSAM / Iridescence install to /usr/local/lib (not always on the linker path).
export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

CMD="${1:-}"
ARG2="${2:-}"
ARG3="${3:-}"

if [[ -z "${CALIB_ROOT:-}" ]]; then
  if [[ -d /root/colcon_ws/calib/vlcal ]]; then
    CALIB_ROOT=/root/colcon_ws/calib/vlcal
  else
    CALIB_ROOT=/home/hound/colcon_ws/calib/vlcal
  fi
fi
POINTS_TOPIC="${POINTS_TOPIC:-/livox/cloud}"

usage() {
  cat >&2 <<EOF
Usage:
  $0 use_bag <bag>
  $0 apply_ssot [--write]
  $0 <preprocess|initial_guess_manual|initial_guess_ssot|calibrate|viewer> <camera_*> [bag]

<bag> may be:
  rosbag2_YYYY_MM_DD-HH_MM_SS          (under ${CALIB_ROOT}/)
  /absolute/or/relative/path/to/bag    (directory with metadata.yaml)

Env:
  CALIB_ROOT   (default ${CALIB_ROOT})
  POINTS_TOPIC (default ${POINTS_TOPIC})
  BAG          (optional default bag for preprocess if [bag] omitted)
  SSOT         (path to SSoT.yaml for apply_ssot / initial_guess_ssot)
EOF
  exit 1
}

resolve_bag() {
  local raw="${1:-}"
  local cand=""
  if [[ -z "${raw}" ]]; then
    echo ""
    return 0
  fi
  if [[ -d "${raw}" && -f "${raw}/metadata.yaml" ]]; then
    cand="$(cd "${raw}" && pwd)"
  elif [[ -d "${CALIB_ROOT}/${raw}" && -f "${CALIB_ROOT}/${raw}/metadata.yaml" ]]; then
    cand="$(cd "${CALIB_ROOT}/${raw}" && pwd)"
  elif [[ -d "${CALIB_ROOT}/all/bags/${raw}" && -f "${CALIB_ROOT}/all/bags/${raw}/metadata.yaml" ]]; then
    cand="$(cd "${CALIB_ROOT}/all/bags/${raw}" && pwd)"
  else
    echo "Bag not found or missing metadata.yaml: ${raw}" >&2
    echo "  looked under: ${raw}" >&2
    echo "               ${CALIB_ROOT}/${raw}" >&2
    exit 1
  fi
  echo "${cand}"
}

# Replace camera bags/ contents with a single relative symlink to the bag.
# Keeps preprocess from mixing old + new bags.
link_bag_to_camera() {
  local cam="$1"
  local bag_abs="$2"
  local bags_dir="${CALIB_ROOT}/${cam}/bags"
  local bag_name
  bag_name="$(basename "${bag_abs}")"
  mkdir -p "${bags_dir}"
  # Drop previous bag links/dirs so only the requested bag is used.
  shopt -s nullglob
  for old in "${bags_dir}"/rosbag2_*; do
    rm -rf "${old}"
  done
  shopt -u nullglob
  # Prefer relative symlink when bag lives under CALIB_ROOT.
  local link_target="${bag_abs}"
  case "${bag_abs}" in
    "${CALIB_ROOT}"/*)
      link_target="$(realpath --relative-to="${bags_dir}" "${bag_abs}")"
      ;;
  esac
  ln -sfn "${link_target}" "${bags_dir}/${bag_name}"
  echo "[vlcal] ${cam}/bags → ${bag_name} (${link_target})"
}

[[ -n "${CMD}" ]] || usage

case "${CMD}" in
  use_bag)
    BAG_RAW="${ARG2:-${BAG:-}}"
    [[ -n "${BAG_RAW}" ]] || usage
    BAG_ABS="$(resolve_bag "${BAG_RAW}")"
    echo "[vlcal] use_bag ${BAG_ABS}"
    for c in camera_front camera_left camera_right; do
      link_bag_to_camera "${c}" "${BAG_ABS}"
    done
    exit 0
    ;;
  apply_ssot)
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    SSOT="${SSOT:-/root/colcon_ws/src/hound_core/config/SSoT.yaml}"
    [[ -f "${SSOT}" ]] || SSOT="/home/hound/colcon_ws/src/hound_core/config/SSoT.yaml"
    WRITE_FLAG=()
    if [[ "${ARG2:-}" == "--write" ]]; then
      WRITE_FLAG=(--write)
    fi
    echo "[vlcal] apply_ssot → ${SSOT} ${WRITE_FLAG[*]:-(dry-run)}"
    python3 "${SCRIPT_DIR}/apply_vlcal_to_ssot.py" \
      --calib-root "${CALIB_ROOT}" \
      --ssot "${SSOT}" \
      --no-tf \
      "${WRITE_FLAG[@]}"
    exit 0
    ;;
esac

CAM="${ARG2}"
BAG_RAW="${ARG3:-${BAG:-}}"

[[ -n "${CAM}" ]] || usage
case "${CAM}" in
  camera_front|camera_left|camera_right) ;;
  *) echo "Unknown camera '${CAM}'" >&2; exit 1 ;;
esac

if ! command -v ros2 >/dev/null; then
  echo "ros2 not on PATH — source ROS + workspace first" >&2
  exit 1
fi
if ! ros2 pkg prefix direct_visual_lidar_calibration >/dev/null 2>&1; then
  echo "direct_visual_lidar_calibration not found. Build it:" >&2
  echo "  bash ${CALIB_ROOT%/calib/vlcal}/src/hound_core/scripts/vlcal/install_vlcal_deps.sh" >&2
  echo "  # or: colcon build --packages-select direct_visual_lidar_calibration" >&2
  exit 1
fi

BAGS="${CALIB_ROOT}/${CAM}/bags"
PRE="${CALIB_ROOT}/${CAM}/preprocessed"
IMAGE_TOPIC="/${CAM}/color/image_raw"
INFO_TOPIC="/${CAM}/color/camera_info"

case "${CMD}" in
  preprocess)
    if [[ -n "${BAG_RAW}" ]]; then
      BAG_ABS="$(resolve_bag "${BAG_RAW}")"
      link_bag_to_camera "${CAM}" "${BAG_ABS}"
    fi
    shopt -s nullglob
    bag_ok=0
    for b in "${BAGS}"/rosbag2_*; do
      if [[ -f "${b}/metadata.yaml" ]]; then
        bag_ok=1
        break
      fi
    done
    shopt -u nullglob
    if [[ "${bag_ok}" -eq 0 ]]; then
      echo "No usable bags in ${BAGS}" >&2
      echo "  ./run_vlcal.sh use_bag <bagname>" >&2
      echo "  ./run_vlcal.sh preprocess ${CAM} <bagname>" >&2
      exit 1
    fi
    mkdir -p "${PRE}"
    echo "[vlcal] preprocess ${CAM}"
    echo "         bags → ${BAGS}"
    echo "         out  → ${PRE}"
    ros2 run direct_visual_lidar_calibration preprocess \
      "${BAGS}" "${PRE}" \
      --image_topic "${IMAGE_TOPIC}" \
      --camera_info_topic "${INFO_TOPIC}" \
      --points_topic "${POINTS_TOPIC}" \
      -i intensity \
      -v \
      --min_distance "${MIN_DISTANCE:-0.2}"
    # RealSense camera_info often says inverse_plumb_bob; vlcal wants plumb_bob.
    python3 - <<PY
import json
from pathlib import Path
p = Path("${PRE}") / "calib.json"
cfg = json.loads(p.read_text())
m = cfg["camera"]["camera_model"]
if m in ("inverse_plumb_bob", "inverse_brown_conrady"):
    cfg["camera"]["camera_model"] = "plumb_bob"
    p.write_text(json.dumps(cfg, indent=2) + "\n")
    print(f"[vlcal] remapped camera_model {m} → plumb_bob in {p}")
PY
    ;;
  initial_guess_manual)
    [[ -d "${PRE}" ]] || { echo "Missing ${PRE} — preprocess first" >&2; exit 1; }
    echo "[vlcal] initial_guess_manual ${CAM}"
    ros2 run direct_visual_lidar_calibration initial_guess_manual "${PRE}"
    ;;
  initial_guess_ssot)
    [[ -d "${PRE}" ]] || { echo "Missing ${PRE} — preprocess first" >&2; exit 1; }
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    SSOT="${SSOT:-/root/colcon_ws/src/hound_core/config/SSoT.yaml}"
    [[ -f "${SSOT}" ]] || SSOT="/home/hound/colcon_ws/src/hound_core/config/SSoT.yaml"
    echo "[vlcal] initial_guess_ssot ${CAM} (from ${SSOT})"
    python3 "${SCRIPT_DIR}/seed_vlcal_from_ssot.py" \
      --calib-root "${CALIB_ROOT}" \
      --ssot "${SSOT}" \
      "${CAM}"
    ;;
  calibrate)
    [[ -d "${PRE}" ]] || { echo "Missing ${PRE} — preprocess first" >&2; exit 1; }
    echo "[vlcal] calibrate ${CAM}"
    ros2 run direct_visual_lidar_calibration calibrate "${PRE}"
    echo "[vlcal] result: ${PRE}/calib.json"
    ;;
  viewer)
    [[ -d "${PRE}" ]] || { echo "Missing ${PRE} — preprocess first" >&2; exit 1; }
    echo "[vlcal] viewer ${CAM}"
    ros2 run direct_visual_lidar_calibration viewer "${PRE}"
    ;;
  *)
    usage
    ;;
esac
