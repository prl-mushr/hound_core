#!/usr/bin/env bash
# Install / rebuild vlcal deps inside mushr_jazzy (aarch64).
#
# Ephemeral (gone if you exit Docker without commit):
#   /usr/local  → GTSAM, Iridescence
#   apt packages unless the image already has them
#
# Persists on the host mount (/root/colcon_ws == /home/hound/colcon_ws):
#   src/direct_visual_lidar_calibration (+ thirdparty)
#   src/hound_core/scripts/vlcal/
#   calib/vlcal/ bags + preprocessed results
#   colcon build/install of the package (binaries still need /usr/local libs)
#
# Usage (inside mushr_jazzy as root):
#   bash /root/colcon_ws/src/hound_core/scripts/vlcal/install_vlcal_deps.sh
#   # then:
#   source /opt/ros/jazzy/setup.bash && source /root/colcon_ws/install/setup.bash
#   cd /root/colcon_ws/src/hound_core/scripts/vlcal
#   ./run_vlcal.sh preprocess camera_front
#
set -euo pipefail

NPROC="${NPROC:-$(nproc)}"
DEPS_DIR="${VLCAL_DEPS_DIR:-/tmp/vlcal_deps}"
WS="${COLCON_WS:-/root/colcon_ws}"
if [[ ! -d "${WS}/src" && -d /home/hound/colcon_ws/src ]]; then
  WS=/home/hound/colcon_ws
fi
PKG="${WS}/src/direct_visual_lidar_calibration"

export DEBIAN_FRONTEND=noninteractive
export CMAKE_PREFIX_PATH="/usr/local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"

echo "[vlcal-deps] workspace=${WS}"
echo "[vlcal-deps] deps dir=${DEPS_DIR}"

if [[ ! -d "${PKG}" ]]; then
  echo "[vlcal-deps] missing ${PKG}" >&2
  echo "  clone: git clone --recursive https://github.com/koide3/direct_visual_lidar_calibration.git ${PKG}" >&2
  exit 1
fi

# --- apt (ROS + build deps) -------------------------------------------------
echo "[vlcal-deps] apt packages"
apt-get update -qq
apt-get install -y --no-install-recommends \
  build-essential cmake git \
  libeigen3-dev libboost-all-dev libomp-dev \
  libpcl-dev libopencv-dev libfmt-dev \
  libceres-dev \
  libgl1-mesa-dev libglfw3-dev libpng-dev libjpeg-dev \
  ros-${ROS_DISTRO:-jazzy}-cv-bridge \
  ros-${ROS_DISTRO:-jazzy}-pcl-conversions \
  >/dev/null

# --- package thirdparty (persists on mount) ---------------------------------
echo "[vlcal-deps] git submodules (json, nanoflann, Sophus)"
cd "${PKG}"
# Host-mounted tree is often owned by uid != container root → dubious ownership.
git config --global --add safe.directory "${PKG}" 2>/dev/null || true
git submodule update --init --recursive thirdparty/json thirdparty/nanoflann thirdparty/Sophus \
  || git submodule update --init --recursive

if [[ ! -f "${PKG}/thirdparty/Bonxai/include/bonxai/bonxai.hpp" ]]; then
  echo "[vlcal-deps] Bonxai (header-only)"
  if [[ ! -d "${PKG}/thirdparty/Bonxai/.git" ]]; then
    rm -rf "${PKG}/thirdparty/Bonxai"
    git clone --depth 1 https://github.com/facontidavide/Bonxai.git \
      "${PKG}/thirdparty/Bonxai"
  fi
  # vlcal expects thirdparty/Bonxai/include → bonxai_core/include
  ln -sfn bonxai_core/include "${PKG}/thirdparty/Bonxai/include"
fi
test -f "${PKG}/thirdparty/Bonxai/include/bonxai/bonxai.hpp"

# Do not skip the package
rm -f "${PKG}/COLCON_IGNORE"

# FindGTSAM.cmake in-tree already treats TBB as optional (keep that patch).

mkdir -p "${DEPS_DIR}"
cd "${DEPS_DIR}"

# --- GTSAM → /usr/local -----------------------------------------------------
if [[ ! -f /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake ]]; then
  echo "[vlcal-deps] building GTSAM 4.2a9 → /usr/local (slow on Jetson)"
  if [[ ! -d gtsam ]]; then
    git clone --depth 1 --branch 4.2a9 https://github.com/borglab/gtsam.git
  fi
  cmake -S gtsam -B gtsam/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_PYTHON=OFF
  cmake --build gtsam/build -j"${NPROC}"
  cmake --install gtsam/build
else
  echo "[vlcal-deps] GTSAM already in /usr/local"
fi

# --- Iridescence → /usr/local -----------------------------------------------
# Install may register as IridescenceConfig.cmake or iridescence-config.cmake.
if [[ ! -f /usr/local/lib/cmake/Iridescence/IridescenceConfig.cmake ]] && \
   [[ ! -f /usr/local/lib/cmake/iridescence/iridescence-config.cmake ]] && \
   [[ ! -f /usr/local/lib/cmake/iridescence/IridescenceConfig.cmake ]]; then
  echo "[vlcal-deps] building Iridescence → /usr/local"
  if [[ ! -d iridescence ]]; then
    git clone --recursive --depth 1 https://github.com/koide3/iridescence.git
  fi
  cmake -S iridescence -B iridescence/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  cmake --build iridescence/build -j"${NPROC}"
  cmake --install iridescence/build
else
  echo "[vlcal-deps] Iridescence already in /usr/local"
fi

# Optional: make linker find /usr/local/lib without LD_LIBRARY_PATH
if [[ -w /etc/ld.so.conf.d ]]; then
  echo /usr/local/lib > /etc/ld.so.conf.d/usr-local.conf
  ldconfig || true
fi

# --- colcon package ---------------------------------------------------------
echo "[vlcal-deps] colcon build direct_visual_lidar_calibration"
# ROS setup.bash references optional vars; keep set -u for the rest of the script.
# shellcheck disable=SC1091
set +u
source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash
set -u
cd "${WS}"
colcon build --packages-select direct_visual_lidar_calibration \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local

echo
echo "[vlcal-deps] done."
echo "  source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
echo "  source ${WS}/install/setup.bash"
echo "  cd ${WS}/src/hound_core/scripts/vlcal"
echo "  ./run_vlcal.sh preprocess camera_front"
echo
echo "Note: /usr/local installs vanish if you exit Docker without committing the image."
echo "      Re-run this script after a fresh mushr_jazzy start."
