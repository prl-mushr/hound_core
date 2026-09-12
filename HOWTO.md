# HOUND how-to

Commands you actually run. Everything below is **inside `mushr_jazzy`** unless noted.
Config SSoT: `hound_core/config/SSoT.yaml`.

## Shell

```bash
# host
mushr_jazzy

# already sourced by bashrc_common
#   ROS_WORKSPACE=/root/colcon_ws
#   HOUND_SSOT=.../SSoT.yaml
#   RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

| Alias | What |
|-------|------|
| `cs` | Release `colcon build` of the workspace, then `source install/setup.bash` |
| `rviz` | `rviz2 -d .../hound_core/config/display_config.rviz` |
| `rviz_sim` | Same, with `use_sim_time:=true` (bag `/clock`) |
| `clear_map` | Drop live TSDF (`/hound_mapping/map_clear`) |
| `clear_state` | EKF reset (`/hound_fcu_control/ekf_reset`) |
| `trigger_aruco` | One-shot ArUco (`/aruco_registration/trigger`) |
| `reset_mesh_pf` | Pause mesh_pf until the next ArUco hunt (`/mesh_pf/reset`) |
| `vlcal` | Write vlcal extrinsics into SSoT (`apply_ssot --write`) |
| `show_mesh` | Marker `/ply_mesh` for the low-poly map OBJ |

Do **not** use the RViz **Camera** display on this box (second GL view aborts). Use **Image**. Plain `rviz2` (no `-d`) loads `/root/.rviz2/default.rviz` (old Unitree + nvblox plugin) and can abort too.

```bash
# one package
cd /root/colcon_ws
colcon build --packages-select composite_sensing --symlink-install
source /root/colcon_ws/install/setup.bash
```

## Live robot

```bash
ros2 launch hound_core hound_core.launch.py
```

Enabled stages come from SSoT (`*.enabled`). Staggered bring-up.

```bash
rviz
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic hz /livox/cloud
ros2 topic hz /hound_mapping/local_map
ros2 topic echo /hound_mapping/extract_timing_ms   # [integrate, bev, …, hz]
```

```bash
clear_map
clear_state
trigger_aruco
experiment_start   # clear_state, clear_map, reset mesh_pf, then ArUco hunt
```

`experiment_start` pauses mesh_pf, runs the ArUco hunt (`trigger_timeout_s`),
then mesh_pf seeds from **map←odom** and starts tracking. ArUco then
releases that TF so mesh_pf owns `map→odom`. Enable `mesh_pf.enabled`
and `wait_for_aruco_hunt: true`. Re-source `bashrc_common` for the alias.

## Audio / TTS

USB speaker (SSoT `tts.device`, default `plughw:CARD=Device,DEV=0`). Speaks
bring-up, experiment steps (clear state / map, aligning TF, TF aligned),
`/hal/recording`, plus free-form text.

Piper neural voice, offline after one download. `tts.voice`: `jarvis`
(British male) or `friday` (British/Scottish female). Falls back to
`espeak-ng` if the model is missing.

```bash
# one-time inside mushr_jazzy (alsa + Piper binary + Alan/Alba voices)
apt-get update && apt-get install -y espeak-ng alsa-utils
bash /root/colcon_ws/src/hound_core/scripts/setup_tts.sh
aplay -l   # confirm USB card "Device"

# speaker only (no ROS) — Jarvis
echo 'Hound audio ready' | \
  /root/colcon_ws/src/hound_core/share/piper/bin/piper \
  --model /root/colcon_ws/src/hound_core/share/piper/voices/en_GB-alan-medium.onnx \
  --output_file /tmp/tts.wav
aplay -D plughw:CARD=Device,DEV=0 /tmp/tts.wav

# TTS node only (SSoT tts: block; no robot stack)
ros2 launch hound_core tts.launch.py

# other shell — should hear startup, then each line
# Bool events ignore the first sample (latched idle). Prime them once:
ros2 topic pub --once /aruco_registration/hunting std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /aruco_registration/tf_aligned std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /hal/recording std_msgs/msg/Bool "{data: false}"

ros2 service call /hound/tts/test std_srvs/srv/Trigger
ros2 topic pub --once /hound/speak std_msgs/msg/String "{data: 'hello'}"
ros2 topic pub --once /hound_fcu_control/ekf_reset std_msgs/msg/Empty "{}"
ros2 topic pub --once /hound_mapping/map_clear std_msgs/msg/Empty "{}"
ros2 topic pub --once /aruco_registration/hunting std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /aruco_registration/tf_aligned std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /hal/recording std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /hal/recording std_msgs/msg/Bool "{data: false}"

speak Recording started          # alias → /hound/speak
```

Add phrases under `tts.events` in SSoT (Bool / Empty / String topics).

## Bag: seg + mapping

Does **not** start cameras / lidar / EKF / nav. Reads bag RGB, lidar, TF/odom.
Ignores `segmentation.enabled` and `nvblox.enabled`.

Default `prefix` is empty → SSoT names (`/hound_mapping/local_map`).
`prefix:=debug` puts outputs under `/debug/...` so they do not overwrite
bag topics. Inputs stay on SSoT names.

```bash
ros2 launch hound_core hound_seg_mapping.launch.py bag:=/path/to/bag
# or split:
ros2 launch hound_core hound_seg_mapping.launch.py
ros2 bag play /path/to/bag --clock
```

Keep bag topics, write under `/debug`:

```bash
ros2 launch hound_core hound_seg_mapping.launch.py prefix:=debug bag:=/path/to/bag
```

Wall clock (no `/clock`):

```bash
ros2 launch hound_core hound_seg_mapping.launch.py use_sim_time:=false
```

### Planner problems from a bag

Logs LocalMap + `control_state` at 1 Hz. Ctrl+C writes start/goal pairs
(default: goal = pose **5 s** later, XY clamped to that map). Default dir:
`/root/colcon_ws/planning_problems/<utc_stamp>/`. Needs
`/hound_fcu_control/control_state` in the bag.

```bash
ros2 launch hound_core hound_seg_mapping.launch.py \
  log_problems:=true \
  problems_dir:=/root/colcon_ws/planning_problems/my_run
ros2 bag play /path/to/bag --clock
# Ctrl+C when done → snapshots/ + problems/ + problems_index.json
```

Rebuild pairs without replaying the bag (first later pose **≥ 8 m** XY, not t+5 s).
Keeps the original `problems/` folder:

```bash
PYTHONPATH=/root/colcon_ws/src/hound_nav:${PYTHONPATH} python3 -m hound_nav.log_planning_problems \
  --pairs-only /root/colcon_ws/planning_problems/my_run \
  --min-dist 8 --until-min-dist --problems-subdir problems_8m
```

IGHA* vs BiIGHA* (SSoT `nav.Planner_config`). `--free-costmap` = every cell 255
(debug). Workspace `IGHAStar`, not IGHAStar_private.

```bash
PYTHONPATH=/root/colcon_ws/src/hound_nav:${PYTHONPATH} python3 -u \
  /root/colcon_ws/src/hound_nav/hound_nav/run_bag_playback.py \
  /root/colcon_ws/planning_problems/my_run \
  --free-costmap --exp 5000
# 8 m set:
PYTHONPATH=/root/colcon_ws/src/hound_nav:${PYTHONPATH} python3 -u \
  /root/colcon_ws/src/hound_nav/hound_nav/run_bag_playback.py \
  /root/colcon_ws/planning_problems/my_run \
  --free-costmap --exp 5000 --problems-subdir problems_8m
```

Plots + timing: `results_free_cost/` or `results_free_problems_8m/`
(`cost_vs_expansions.png`, `success_vs_expansions.png`, `timing_metrics.json`).
Omit `--free-costmap` to use logged cost. `--config /path.yaml` if not SSoT.
`--plot-only` replots existing pickles. `--cruise-speed 3.0` overrides goal
speed (default = logged `|v|` at the goal pose). After each search an OpenCV
window shows costmap, path, start box, and goal (does not wait). `--no-cv-viz`
to disable.

Viser one problem (host-network container → http://localhost:8081):

```bash
PYTHONPATH=/root/colcon_ws/src/hound_nav:${PYTHONPATH} python3 -u \
  /root/colcon_ws/src/hound_nav/hound_nav/run_bag_playback.py \
  /root/colcon_ws/planning_problems/my_run \
  --viser --problem 10 --free-costmap --exp 5000 --viser-port 8081
```

## Bag: mapping only

```bash
ros2 launch hound_core hound_mapping_replay.launch.py bag:=/path/to/bag prefix:=debug
# or
ros2 launch hound_core hound_mapping_replay.launch.py
ros2 bag play /path/to/bag --clock
```

## Mission manager only

Does **not** start Dora nav or FCU. Ignores `nav.enabled` /
`mission_manager.enabled`. Do not also run `hound_nav.launch.py` / core with
mission_manager on (two nodes).

```bash
ros2 launch hound_core hound_mission_manager.launch.py
# if install is stale:
ros2 launch /root/colcon_ws/src/hound_core/launch/hound_mission_manager.launch.py
```

### RViz map-frame mission (`mode: rviz`)

Play a bag that has the map (and TF so RViz Fixed Frame can be `map`).
`record: true` appends **2D Goal Pose** clicks (`/goal_pose`) and latches
`/hound_nav/mission/waypoints`. Clicks are **not** republished on
`/goal_pose` (avoids a feedback loop). Ctrl+C or `~/save` writes
`mission_file` (x/y/z/yaw, `frame_id: map`).

```bash
# SSoT: mode: rviz, frame_id: map, record: true, publish_goals: false
ros2 launch hound_core hound_mission_manager.launch.py
ros2 bag play /path/to/bag --clock
# RViz: Fixed Frame = map, Publish Point / 2D Goal Pose
ros2 topic echo /hound_nav/mission/waypoints
ros2 service call /mission_manager/save std_srvs/srv/Trigger {}
# undo / clear:
ros2 topic pub --once /mission_manager/undo std_msgs/msg/Empty {}
ros2 topic pub --once /mission_manager/clear std_msgs/msg/Empty {}
```

To **publish** a saved mission (current WP on `/goal_pose`): set
`record: false`, `load_on_start: true`, `publish_goals: true`, then
relaunch. Advance with:

```bash
ros2 topic pub --once /mission_manager/advance std_msgs/msg/Empty {}
```

### GPS mode (`mode: gps`)

Needs FCU (or a bag) already up. Inputs:

```bash
ros2 topic hz /hound_fcu_control/gps/fix
ros2 topic hz /hound_fcu_control/gps/fix_type
ros2 topic hz /hound_fcu_control/imu
ros2 topic echo /hound_fcu_control/gps/fix_type
ros2 topic echo /hound_fcu_control/mission/gps   # lat=x lon=y, frame wgs84
ros2 topic hz /hound_fcu_control/control_state   # optional (odom-align)
```

Outputs (latched). Empty until a GPS mission is accepted (fix_type ≥ 3,
h_acc < `max_h_acc_m`):

```bash
ros2 topic echo /goal_pose
ros2 topic echo /hound_nav/mission/waypoints
```

## Nav only

Expects `/hound_mapping/local_map` and `/hound_fcu_control/control_state` already up (live stack or bag). Set `nav.enabled: false` in SSoT if `hound_core.launch.py` is also running.

```bash
ros2 launch hound_core hound_nav.launch.py
```

### Nav bag → MP4 (live cv_viz)

Same costmap / path / start box / goal / diagnostics HUD as the planner
OpenCV window. Needs `/hound_mapping/local_map` in the bag; plan / state /
goal / diagnostics are optional (held when present).

```bash
PYTHONPATH=/root/colcon_ws/src/hound_nav:${PYTHONPATH} python3 \
  /root/colcon_ws/src/hound_nav/scripts/bag_nav_video.py \
  /root/colcon_ws/bags_nav/bidirectional/hound_2026_09_02-03_51_50
# writes sibling hound_2026_09_02-03_51_50_cv.mp4
# PYTHONPATH=... python3 .../bag_nav_video.py --list --bags /root/colcon_ws/bags_nav
#   -o /tmp/run.mp4 --fps 20 --map-size 480
```

### CUDA JIT cache (clean rebuild)

Planner / MPPI / tracking-cost `.so` files live on the bind mount, not
`~/.cache`:

`/root/colcon_ws/cache/torch_extensions/`
(`TORCH_EXTENSIONS_DIR` in `bashrc_common`). A new Docker image does **not**
need a rebuild if those `.so` files exist and the listed `.cpp`/`.cu` are
older. Startup should print `[cuda_cache] reuse …/ighastar.so` (same for
`mppi_analytical_bicycle`, `hound_tracking_cost`).

Rebuild when you change those sources, nvcc flags, or the `.so` is corrupt.

```bash
# wipe + compile all three (uses SSoT Planner_config)
rm -rf /root/colcon_ws/cache/torch_extensions
PYTHONPATH=/root/colcon_ws/src/hound_nav:${PYTHONPATH} \
  python3 -m hound_nav.jit_build --clean
# or after colcon install is current:
#   ros2 run hound_nav jit_build_cuda --clean
```

One-shot without wiping the folder first:

```bash
TORCH_EXTENSIONS_FORCE_REBUILD=1 \
  PYTHONPATH=/root/colcon_ws/src/hound_nav:${PYTHONPATH} \
  python3 -m hound_nav.jit_build
```

Skip pieces: `--skip-planner` / `--skip-dynamics` / `--skip-cost`.
First Orin compile of IGHA* can take several minutes. Next `hound_nav.launch.py`
should reuse the cache.

## mesh_pf only

Ignores `mesh_pf.enabled`. Default `prefix:=debug` → pose
`/debug/localization/mesh_pose`, node `/debug/mesh_pf`. Cloud stays
`/livox/cloud` (or SSoT `cloud_topic`). Live: `wait_for_aruco_hunt` holds
until `experiment_start` / ArUco hunt ends, then seeds from **map←odom**
∘ **odom←base** and tracks with **MICP-L** (`tracking_backend: micp`:
raycast correspondences + point-to-plane). Set `tracking_backend: pf` for
the particle filter instead. After handover the node latches **map←odom**
(ArUco stops publishing it). Missing seed TF after a hunt stays idle
(no bbox). Service `~/global_localization` re-scatters the PF bbox. Bag replay: `rviz_sim`.

Vulkan RT (`raycast_backend: auto`) needs `libvulkan-dev` + `glslang-tools`
**at cmake time**. Fresh box: run
`composite_sensing/scripts/install_deps.sh` before the first
`colcon build` of `composite_sensing` (one-shot: `hound_core/docs/context.md`
§4b). Startup must say `backend=vulkan`. `backend=embree` with no
`Vulkan RT unavailable` line means it was compiled Embree-only — install
those packages and `colcon build --packages-select composite_sensing --cmake-force-configure`.

```bash
ros2 launch hound_core hound_mesh_pf.launch.py prefix:=debug bag:=/path/to/bag
# or
ros2 launch hound_core hound_mesh_pf.launch.py
ros2 bag play /path/to/bag --clock
# live names:
ros2 launch hound_core hound_mesh_pf.launch.py prefix:= use_sim_time:=false
ros2 service call /debug/mesh_pf/global_localization std_srvs/srv/Empty
```

## Author lethal overlay (RViz)

Does **not** edit the LayerCake. Mapping ANDs the mask onto LocalMap cost
(`0` = lethal). Empty `nvblox.lethal_map_path` = all free.

Launch starts a **dedicated** RViz (`author_lethal.rviz`, Fixed Frame
`map`) and publishes the OBJ (`mesh_pf.map_file`) on `/ply_mesh`. No bag.
**2D Pose Estimate** (`/initialpose`) adds vertices. Click within **1 m of
the first vertex** (≥3 points) to close and save.

```bash
ros2 launch hound_core hound_author_lethal.launch.py
ros2 topic pub --once /lethal_author/undo std_msgs/msg/Empty {}
ros2 service call /lethal_author/save std_srvs/srv/Trigger {}
```

Then set SSoT and **restart** mapping (overlay loads once at startup):

```yaml
nvblox:
  lethal_map_path: "/root/colcon_ws/maps/Allen_backside_0/hound_tsdf.lethal.yaml"
```

## Save map / mesh

LayerCake is the persistent TSDF. LocalMap (elevation + cost) is a live topic;
latch it and write the last grid:

```bash
# queue LayerCake write (path from nvblox.layer_cake_path)
ros2 service call /hound_mapping/save_layer_cake std_srvs/srv/Trigger
# with debug prefix:
ros2 service call /debug/hound_mapping/save_layer_cake std_srvs/srv/Trigger
# sidecar: /root/colcon_ws/maps/hound_tsdf.map_odom.yaml
```

```bash
# write the next LocalMap and exit (picks /hound_mapping or /debug/hound_mapping)
python3 /root/colcon_ws/src/hound_mapping/scripts/save_elevation.py
# keep running and write on demand:
python3 /root/colcon_ws/src/hound_mapping/scripts/save_elevation.py --serve
ros2 service call /save_elevation/save std_srvs/srv/Trigger
```

Restart mapping to load a prior. Gap-fill uses the **LayerCake**
(`nvblox.prior_layer_cake_path`), not the elev yaml. `prior_xyz_yaw` is
T_odom_prior (xyz m + yaw rad); leave identity when live odom matches the save.

`nvblox.save_map_odom_tf: true` (default) snapshots `/tf_static` **map←odom**
next to the cake as `<stem>.map_odom.yaml` on LayerCake save.

That TF is the ArUco lock from the record session (also in the bag as
`/tf_static`). Play the bag so mapping has seen it, then save. Same sidecar
format as ArUco (`source: file` republishes it without the detector).

Bag already has map←odom, no remapping:

```bash
ros2 run aruco_registration extract_bag_tf --bag /path/to/bag \
  --out /root/colcon_ws/maps/hound_tsdf.map_odom.yaml
```

ArUco write-now (without saving the cake):

```bash
ros2 service call /map_odom_from_aruco/save_tf std_srvs/srv/Trigger
```

```bash
# oneshot mesh export. ros2 run does NOT read SSoT — pass nvblox.layer_cake_path.
ros2 run hound_mapping layercake_to_mesh_node --ros-args \
  -p layer_cake_path:=/root/colcon_ws/maps/Allen_backside_0/hound_tsdf.layercake \
  -p mesh_ply_path:=/root/colcon_ws/maps/Allen_backside_0/hound_tsdf_mesh.ply
```

```bash
# RViz: PointCloud2 /ply_cloud, RGB8. Turn off use_sim_time if a bag RViz is open.
python3 /root/colcon_ws/src/hound_mapping/scripts/show_ply.py /root/colcon_ws/maps/hound_tsdf_mesh.ply
# low-poly map-frame mesh for mesh_pf (voxel weld + map←odom)
python3 /root/colcon_ws/src/hound_mapping/scripts/decimate_mesh.py \
  /root/colcon_ws/maps/hound_tsdf_mesh.ply
# → hound_tsdf_mesh_pf.obj + .yaml (init_bb for mesh_pf)
# RViz Marker /ply_mesh (already in display_config). Fixed Frame: map.
python3 /root/colcon_ws/src/hound_mapping/scripts/show_mesh.py \
  /root/colcon_ws/maps/hound_tsdf_mesh_pf.obj
```

## Calib (vlcal)

Full notes: `scripts/vlcal/README.md`.

```bash
cd /root/colcon_ws/src/hound_core/scripts/vlcal
./run_vlcal.sh calibrate camera_front    # left / right the same
./run_vlcal.sh apply_ssot                # dry-run
./run_vlcal.sh apply_ssot --write        # or alias: vlcal
```

## Record a bag

RC HAL can toggle `/hal/record`. Manual:

```bash
ros2 bag record -a -o /root/colcon_ws/bags/run_$(date +%Y_%m_%d-%H_%M_%S)
```

vlcal bags: `scripts/vlcal/README.md` (`/root/colcon_ws/calib/vlcal/...`).
