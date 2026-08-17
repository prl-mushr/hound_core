# HOUND stack — clone onto a new Jetson

Self-contained bring-up notes for reproducing the SSoT stack (enabled **and** disabled blocks) plus the `mushr_jazzy` Docker image. Written for a second Jetson; keep the same JetPack / L4T as the reference robot when possible.

**Reference machine (this doc’s baseline)**

| Item | Value |
|------|--------|
| Host layout | `/home/hound/colcon_ws` → in container `/root/colcon_ws` |
| OS / L4T | Ubuntu 24.04 / L4T R39.2 (JetPack matching Isaac ROS image) |
| Docker wrapper | `mushr_jazzy` → image `dbzfan2012/mushr-jazzy:aarch64` |
| Compose | `mushr/mushr_utils/install/docker-compose-build-robot.yml` |
| SSoT | `hound_core/config/SSoT.yaml` |
| Launch | `ros2 launch hound_core hound_core.launch.py` |

---

## 0. What “done” looks like

Inside `mushr_jazzy`:

1. All SSoT-related repos present under `/root/colcon_ws/src` on the correct branches.
2. Livox-SDK2 built once (static/shared lib under `Livox-SDK2/build`).
3. `colcon build --symlink-install` succeeds for the HOUND packages.
4. TensorRT `.engine` files rebuilt on **this** Jetson (do not copy engines from another Orin/JetPack).
5. `SSoT.yaml` edited for **this** robot (camera serials, IPs, ttys, extrinsics).
6. Launch brings up stereo + mapping + seg + viz (or whatever you enable).

---

## 1. Host prerequisites

1. Flash **matching JetPack / L4T** to the reference Jetson (mismatch breaks the Isaac base image and TRT engines).
2. Install Docker + NVIDIA Container Toolkit (`nvidia-ctk runtime configure --runtime=docker`).
3. Create workspace parent, e.g. `mkdir -p ~/colcon_ws/src` (or keep `/home/hound` for identical paths).
4. GitHub access for the remotes below (HTTPS token or SSH).

---

## 2. Docker image (`mushr_jazzy`)

**Preferred:** load/pull the already-built image.

```bash
# On reference Jetson — save (large)
docker save dbzfan2012/mushr-jazzy:aarch64 | gzip > mushr-jazzy-aarch64.tar.gz

# On new Jetson — load
gunzip -c mushr-jazzy-aarch64.tar.gz | docker load
# or, if Hub is reachable:
docker pull dbzfan2012/mushr-jazzy:aarch64
```

**Install wrapper + udev (VESC, etc.):**

```bash
git clone -b jazzy https://github.com/prl-mushr/mushr.git ~/colcon_ws/src/mushr
# Follow mushr_utils/install/README.md — robot=y
# Or run:  ~/colcon_ws/src/mushr/mushr_utils/install/mushr_install.bash
```

That installs `/usr/local/bin/mushr_jazzy`. Env of note:

- `MUSHR_WS_PATH` = parent of `colcon_ws` (e.g. `/home/hound`)
- Mount: `${MUSHR_WS_PATH}/colcon_ws` → `/root/colcon_ws`
- `network_mode: host`, `privileged`, `/dev` passthrough, NVIDIA runtime

Enter the container:

```bash
mushr_jazzy
# subsequent shells:
mushr_jazzy   # reuses existing container if still present
```

CUDA / `nvidia-l4t-*` stay on the **host**; the image does not replace JetPack.

---

## 3. Clone all SSoT repos (script)

Bootstrap `hound_core` first (contains the clone script), then pull everything else.

```bash
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
git clone -b feature/multicam-batch https://github.com/prl-mushr/hound_core.git

bash hound_core/scripts/clone_hound_stack.sh ~/colcon_ws/src --pull
```

The script clones/updates these remotes/branches:

| Directory | Remote | Branch |
|-----------|--------|--------|
| `hound_core` | `prl-mushr/hound_core` | `feature/multicam-batch` |
| `mushr` | `prl-mushr/mushr` | `jazzy` |
| `composite_sensing` | `sidtalia/composite_sensing` | `feature/multicam-batch` |
| `Livox-SDK2` | `sidtalia/Livox-SDK2` (fork) | `master` |
| `unilidar_sdk` | `unitreerobotics/unilidar_sdk` | `main` |
| `pointcloud_deskewer` | `sidtalia/pointcloud_deskewer` | `main` |
| `hound_mapping` | `sidtalia/hound_mapping` | `feature/multicam-batch` |
| `perception_models` | `sidtalia/perception_models` | `feature/multicam-batch` |
| `nanosam-src` | `NVIDIA-AI-IOT/nanosam` | `main` |
| `hound_viz` | `sidtalia/hound_viz` | `main` |
| `inertial_nav_ros2` | `sidtalia/inertial_nav_ros2` | `main` |
| `Inertial_nav_shim` | `sidtalia/Inertial_nav_shim` | `main` |
| `vesc_ros2` | `sidtalia/vesc_ros2` (fork) | `ros2_implementation` |
| `mavros` | `prl-mushr/mavros` | `ros2` |
| `hound_nav` | `sidtalia/hound_nav` | `main` |
| `mppi` | `sidtalia/MPPI` | `main` |
| `IGHAStar` | `personalrobotics/IGHAStar` | `ESDF_support` |

It also touches `COLCON_IGNORE` on non-ament trees (`Livox-SDK2`, `nanosam-src`, `IGHAStar`, `Inertial_nav_shim`, `unilidar_sdk`).

**Livox fork notes:** stock upstream is fine for basic clouds. The [sidtalia/Livox-SDK2](https://github.com/sidtalia/Livox-SDK2) fork keeps Mid-360S `GetInternalInfo` key trim + `DisableLivoxSdkConsoleLogger` spdlog fix. Sample JSON host IPs are **not** patched (composite generates its own config).

**vesc:** [sidtalia/vesc_ros2](https://github.com/sidtalia/vesc_ros2) is required to **build** `hound_core` (FCU embeds `vesc_driver` sources + `vesc_msgs`), even if `fcu_control.enabled: false`.

---

## 4. Build inside the container

```bash
mushr_jazzy
source /opt/ros/jazzy/setup.bash

# 4a) Livox SDK (needed when lidar.backend=livox; safe to build always)
cd /root/colcon_ws/src/Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc)

# 4b) Package deps
bash /root/colcon_ws/src/composite_sensing/scripts/install_deps.sh
bash /root/colcon_ws/src/perception_models/scripts/install_deps.sh
bash /root/colcon_ws/src/hound_viz/install_deps.sh

cd /root/colcon_ws
rosdep install --from-paths src --ignore-src -y -r   # as needed
colcon build --symlink-install
source /root/colcon_ws/install/setup.bash
```

Minimum packages for the **currently enabled** SSoT set (stereo + mapping + seg + viz + bag):  
`hound_core`, `composite_sensing`, `hound_mapping`, `perception_models`, `hound_viz` (+ `vesc_msgs` / `vesc_ros2` for linking FCU node).

---

## 5. TensorRT engines (do not copy across Jetsons)

ONNX / `.pt` can be copied; **`.engine` must be rebuilt** on the new GPU/JetPack.

```bash
# Default: FP16; uses DLA when TensorRT exposes cores, else GPU
bash /root/colcon_ws/src/perception_models/scripts/build_all_engines.sh

# Force GPU-only:
USE_DLA=0 bash /root/colcon_ws/src/perception_models/scripts/build_all_engines.sh
```

Point `SSoT.yaml` `segmentation.*_engine` / SAM paths at the engines that script writes (filenames may differ from `b3` / `_dyn` names on the reference robot).

**JetPack 7.2 / L4T R39.2:** TensorRT reports `num_DLA_cores=0` even on Orin NX
(HW + `/dev/nvhost-ctrl-nvdla*` present; cuDLA can see 2 devices). NVIDIA: DLA for
TRT comes in a later release — scripts fall back to GPU FP16 automatically.
When DLA works: vision/encoder → core 0; FiLM/decoder → core 1.
Typical paths under `perception_models/data/`:

- CLIPSeg vision + FiLM engines  
- NanoSAM image encoder + batched mask decoder  

---

## 6. Reconfigure SSoT for this robot

Edit `hound_core/config/SSoT.yaml` (same file on host and in container):

| Field | Action |
|-------|--------|
| `stereo_composite.cameras.*.serial_number` | `rs-enumerate-devices` |
| Camera / lidar `xyz` / `rpy` | Re-measure or re-run vlcal (`hound_core/scripts/vlcal/`) |
| `lidar.lidar_ip` / `local_ip` | Match NIC subnet |
| `fcu_control.fcu_url`, `vesc.port`, `gcs_url` | When enabling FCU |
| Engine paths | After TRT rebuild |
| Feature flags (`*.enabled`) | Enable only hardware that exists |

**Depth vs people mask:** if `nvblox.use_people_mask: true`, launch forces `align_depth` and depth publishes on `…/aligned_depth_to_color/…`. With `use_people_mask: false` and `align_depth: false`, depth stays on `…/depth/image_rect_raw`. Mapping depth templates must match (already handled in launch).

**Optional terrain tweak:** `nvblox.depth_ignore_bottom_fraction: 0.5` zeros the bottom half of depth before TSDF integrate (lidar untouched).

---

## 7. Launch and smoke checks

```bash
source /root/colcon_ws/install/setup.bash
ros2 launch hound_core hound_core.launch.py
```

| Check | Expect |
|-------|--------|
| `rs-enumerate-devices` | Serials match SSoT |
| `ros2 topic hz /visual_slam/tracking/odometry` | Tracking when front VSLAM is up |
| `/{cam}/color/image_raw` | ~`color_publish_fps` |
| Depth topic (native or aligned) | Matches `align_depth` / people-mask setting |
| `/hound_mapping/local_map` | When `nvblox.enabled` and depth/lidar integrating (`integrate` not stuck at `0.0`) |
| Browser | `http://<jetson-ip>:8080/` (viz) |
| Lidar (if on) | `/livox/cloud` after IP/subnet correct |

Offline (no HW): `bash /root/colcon_ws/src/hound_core/scripts/smoke_dataset_pipeline.sh` if dataset assets are present.

---

## 8. SSoT block → package map

| SSoT key | Packages / notes |
|----------|------------------|
| `stereo_composite` | `composite_sensing` (cuVSLAM + RealSense) |
| `lidar` | `composite_sensing` + built `Livox-SDK2` (or Unitree path) |
| `mesh_pf` | `composite_sensing` + Embree |
| `fcu_control` | `hound_core` FCU node; embeds VESC via `vesc_ros2`; EKF via `inertial_nav_*` |
| `nvblox` | `hound_mapping` (`mapping_node`) — SSoT key name is historical |
| `segmentation` / `yolo_world` | `perception_models` (Dora) + `nanosam-src` |
| `viz` | `hound_viz` |
| `nav` | `hound_nav` + `mppi` + `IGHAStar` |
| `hal_monitor` / `bag_recorder` | `hound_core` |
| `launch.stage_delay_s` | Stagger between enabled stages |

---

## 9. Portable vs not

**Copy / clone**

- Git remotes on the branches above  
- ONNX / `.pt` weights  
- Optional maps under `colcon_ws/maps/`  
- Docker image tarball  

**Do not rely on copying**

- `build/` / `install/`  
- `*.engine`  
- Camera serials, extrinsics, tty nodes, Livox IPs  

---

## 10. Cursor profile / chat (optional)

- **Settings/profile:** Cursor `File → Preferences → Profiles` → export / import.  
- **Agent chat history:** not officially synced. Per-chat Export, or community tools / rsync of `~/.config/Cursor/User/workspaceStorage` and `~/.cursor/projects` (quit Cursor first; workspace **path** must match for chats to reattach).

---

## Quick checklist

- [ ] Matching JetPack + Docker + NVIDIA runtime  
- [ ] Load/pull `dbzfan2012/mushr-jazzy:aarch64` + `mushr_jazzy` wrapper  
- [ ] `clone_hound_stack.sh`  
- [ ] Build Livox-SDK2  
- [ ] `install_deps.sh` ×3 + `colcon build`  
- [ ] Rebuild TRT engines; fix SSoT engine paths  
- [ ] SSoT serials / extrinsics / IPs  
- [ ] `ros2 launch hound_core hound_core.launch.py` + smoke topics  

---

*Generated for HOUND SSoT bring-up. Update remotes/branches in `hound_core/scripts/clone_hound_stack.sh` if the reference stack moves.*
