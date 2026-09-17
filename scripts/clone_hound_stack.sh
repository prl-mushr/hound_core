#!/usr/bin/env bash
# Clone / update HOUND colcon src repos for the SSoT stack and check out the
# branches used on the reference Jetson.
#
# Usage (from anywhere):
#   bash /path/to/hound_core/scripts/clone_hound_stack.sh
#   bash .../clone_hound_stack.sh /home/hound/colcon_ws/src
#   SRC_DIR=~/colcon_ws/src bash .../clone_hound_stack.sh --pull
#
# Env:
#   SRC_DIR   Destination for clones (default: <hound_core>/../ i.e. colcon_ws/src)
#   PULL=1    Also fast-forward pull after checkout (or pass --pull)
#
# Does not build, install Docker, or download TensorRT engines.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_SRC="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SRC_DIR="${SRC_DIR:-${DEFAULT_SRC}}"
PULL="${PULL:-0}"

for arg in "$@"; do
  case "$arg" in
    --pull|-p) PULL=1 ;;
    --help|-h)
      sed -n '2,20p' "$0"
      exit 0
      ;;
    -*)
      echo "Unknown flag: $arg" >&2
      exit 2
      ;;
    *)
      SRC_DIR="$arg"
      ;;
  esac
done

mkdir -p "${SRC_DIR}"
SRC_DIR="$(cd "${SRC_DIR}" && pwd)"
cd "${SRC_DIR}"

# git may refuse "dubious ownership" on some mounts; scope to this process only.
export GIT_CONFIG_COUNT=1
export GIT_CONFIG_KEY_0=safe.directory
export GIT_CONFIG_VALUE_0='*'

echo "[hound] SRC_DIR=${SRC_DIR}  pull=${PULL}"

# name|url|branch[|recursive]
# Covers SSoT blocks (enabled + disabled) and build deps (vesc, Livox, …).
# Pinned to remotes/branches on the reference Jetson (/home/hound/colcon_ws/src).
REPOS=(
  # Core / launch / SSoT
  "hound_core|https://github.com/prl-mushr/hound_core.git|feature/multicam-batch"
  "mushr|https://github.com/prl-mushr/mushr.git|jazzy"

  # Sensing (stereo_composite, lidar, mesh_pf)
  "composite_sensing|https://github.com/sidtalia/composite_sensing.git|feature/multicam-batch"
  "Livox-SDK2|https://github.com/sidtalia/Livox-SDK2.git|master"
  "unilidar_sdk|https://github.com/unitreerobotics/unilidar_sdk.git|main"
  "pointcloud_deskewer|https://github.com/sidtalia/pointcloud_deskewer.git|main"

  # Mapping (SSoT key: nvblox) + RViz LocalMap display
  "hound_mapping|https://github.com/sidtalia/hound_mapping.git|feature/multicam-batch-authoring"
  "hound_rviz_plugins|https://github.com/sidtalia/hound_rviz_plugins.git|main"

  # Perception (segmentation, yolo_world) + ArUco detector (pip, not ament)
  "perception_models|https://github.com/sidtalia/perception_models.git|feature/multicam-batch"
  "nanosam-src|https://github.com/NVIDIA-AI-IOT/nanosam.git|main"
  "dense_aruco|https://github.com/sidtalia/dense_aruco.git|main"
  "aruco_registration|https://github.com/sidtalia/aruco_registration.git|main"

  # Viz
  "hound_viz|https://github.com/sidtalia/hound_viz.git|main"

  # FCU / EKF / VESC (fcu_control); serial_driver lives in transport_drivers
  "inertial_nav_ros2|https://github.com/sidtalia/inertial_nav_ros2.git|main"
  "Inertial_nav_shim|https://github.com/sidtalia/Inertial_nav_shim.git|main"
  "vesc_ros2|https://github.com/sidtalia/vesc_ros2.git|ros2_implementation"
  "mavros|https://github.com/prl-mushr/mavros.git|ros2"
  "transport_drivers|https://github.com/ros-drivers/transport_drivers.git|main"

  # Nav (Dora manager / planner / controller)
  "hound_nav|https://github.com/sidtalia/hound_nav.git|main"
  "mppi|https://github.com/sidtalia/MPPI.git|main"
  "IGHAStar|https://github.com/personalrobotics/IGHAStar.git|ESDF_support"
  "robin-hood-hashing|https://github.com/martinus/robin-hood-hashing.git|master"

  # Camera–lidar calib (optional; native arm64 via scripts/vlcal/install_vlcal_deps.sh)
  "direct_visual_lidar_calibration|https://github.com/koide3/direct_visual_lidar_calibration.git|main|recursive"
)

# Packages that must not be built by colcon as ROS pkgs.
COLCON_IGNORE_PKGS=(
  Livox-SDK2
  nanosam-src
  IGHAStar
  Inertial_nav_shim
  unilidar_sdk
  dense_aruco
  robin-hood-hashing
)

ensure_repo() {
  local name="$1" url="$2" branch="$3" extra="${4:-}"
  local path="${SRC_DIR}/${name}"
  local clone_args=(--branch "${branch}" --single-branch)
  [[ "${extra}" == *recursive* ]] && clone_args+=(--recurse-submodules)

  if [[ ! -d "${path}/.git" ]]; then
    echo "[hound] clone ${name} (${branch})"
    git clone "${clone_args[@]}" "${url}" "${path}"
  else
    echo "[hound] update ${name} → ${branch}"
    # Prefer the expected origin URL (e.g. forks).
    local cur
    cur="$(git -C "${path}" remote get-url origin 2>/dev/null || true)"
    if [[ -n "${cur}" && "${cur}" != "${url}" && "${cur}" != "${url%.git}" && "${cur%.git}" != "${url}" ]]; then
      echo "[hound]   origin was ${cur}; setting to ${url}"
      git -C "${path}" remote set-url origin "${url}"
    fi
    git -C "${path}" fetch origin --prune || {
      echo "[hound]   WARN: fetch failed for ${name}; using existing remotes" >&2
    }
    # Detach-safe checkout of the tracking branch.
    if git -C "${path}" show-ref --verify --quiet "refs/remotes/origin/${branch}"; then
      git -C "${path}" checkout -B "${branch}" "origin/${branch}"
    elif git -C "${path}" show-ref --verify --quiet "refs/heads/${branch}"; then
      git -C "${path}" checkout "${branch}"
    else
      echo "[hound]   ERROR: branch ${branch} missing for ${name}" >&2
      return 1
    fi
    if [[ "${PULL}" == "1" ]]; then
      git -C "${path}" pull --ff-only origin "${branch}" || {
        echo "[hound]   WARN: ff-only pull failed for ${name} (local commits?)" >&2
      }
    fi
    if [[ "${extra}" == *recursive* ]]; then
      git -C "${path}" submodule update --init --recursive || {
        echo "[hound]   WARN: submodule update failed for ${name}" >&2
      }
    fi
  fi

  local head
  head="$(git -C "${path}" rev-parse --short HEAD)"
  echo "[hound]   ${name} @ ${branch} (${head})"
}

failed=0
for entry in "${REPOS[@]}"; do
  IFS='|' read -r name url branch extra <<<"${entry}"
  if ! ensure_repo "${name}" "${url}" "${branch}" "${extra}"; then
    failed=1
  fi
done

for name in "${COLCON_IGNORE_PKGS[@]}"; do
  path="${SRC_DIR}/${name}"
  if [[ -d "${path}" ]]; then
    touch "${path}/COLCON_IGNORE" 2>/dev/null || \
      echo "[hound] WARN: could not write COLCON_IGNORE in ${name}" >&2
    echo "[hound] COLCON_IGNORE → ${name}"
  fi
done

echo
echo "[hound] done. Next (inside mushr_jazzy / ROS env):"
echo "  1) Build Livox SDK:  cd ${SRC_DIR}/Livox-SDK2 && mkdir -p build && cd build && cmake .. && make -j\$(nproc)"
echo "  2) Dep scripts:      composite_sensing/scripts/install_deps.sh"
echo "                      perception_models/scripts/install_deps.sh"
echo "                      hound_viz/scripts/install_deps.sh"
echo "     (composite_sensing install_deps.sh installs glslang-tools — required for mesh_pf Vulkan RT)"
echo "  3) pip install -e ${SRC_DIR}/dense_aruco   # ArUco detector (COLCON_IGNORE)"
echo "  4) bash ${SRC_DIR}/hound_core/scripts/setup_tts.sh   # Piper voices"
echo "  5) colcon build --symlink-install"
echo "  6) Rebuild TRT engines (perception_models/scripts/build_all_engines.sh)"
echo "  7) Edit hound_core/config/SSoT.yaml for this robot (serials, IPs, ttys)"
echo "  optional vlcal: bash ${SRC_DIR}/hound_core/scripts/vlcal/install_vlcal_deps.sh"

if [[ "${failed}" -ne 0 ]]; then
  echo "[hound] one or more repos failed" >&2
  exit 1
fi
