# Handover: RealSense cuVSLAM + nvblox (HOUND)

Context for a new agent continuing perception work on this robot. Workspace: `/home/hound/colcon_ws/src` (builds inside Docker `mushr_jazzy` with ws at `/root/colcon_ws`).

## What this stack is

- **Primary camera path:** `hound_core` C++ node `realsense_cuvslam_node` — exclusive D455 owner.
  - IR stereo → cuVSLAM (`Odometry::Track` + `Slam::Track`) → `/visual_slam/tracking/odometry` + TF `odom` → `camera_link`
  - Optional color/depth published at capped rates for nvblox / viz
  - **Replaces** stock `realsense2_camera` + Isaac `isaac_ros_visual_slam` when `realsense_cuvslam.enabled: true`
- **Config SSoT:** `hound_core/config/SSoT.yaml`
- **Launch:** `hound_core/launch/hound_core.launch.py` + helpers in `hound_launch_common.py`
- **Map:** nvblox (`nvblox_hound.yaml` + SSoT overrides), often **RGB + lidar** (depth optional)

## Key files

| Area | Path |
|------|------|
| C++ node | `hound_core/src/realsense_cuvslam_node.cpp` |
| Headers | `include/hound_core/realsense_cuvslam_node.hpp`, `rs_pipeline.hpp`, `cuvslam_tracker.hpp`, `frame_conventions.hpp` |
| SSoT | `hound_core/config/SSoT.yaml` |
| nvblox defaults | `hound_core/config/nvblox_hound.yaml` |
| Launch wiring | `launch/hound_core.launch.py`, `launch/hound_launch_common.py` |

Build (in container):

```bash
source /opt/ros/jazzy/setup.bash && cd /root/colcon_ws && colcon build --packages-select hound_core
```

## Architecture (current, important)

### Decoupled RealSense sensors (do not revert to single `pipeline.wait_for_frames`)

**Problem:** One `rs2::pipeline` with IR@60 + color@30 caused `wait_for_frames` to sync to the RGB sensor → Track/odom oscillated ~30↔60 Hz. Confirmed: color off → steady ~60; nvblox/lidar did **not** affect Track time (~1.5 ms VO).

**Fix:** Open **stereo module** and **RGB sensor** separately (`rs_pipeline.hpp` → `open_rs_sensors`):

- Stereo: IR1+IR2 (± depth) → `rs2::syncer` → matched IR framesets for Track
- Color: own `frame_queue` → publish path only
- Capture loop drains color without gating Track

**Bug already hit:** `sensor.start(queue)` delivers **single frames**, not framesets. Must use `rs2::syncer` for IR1+IR2. Without syncer, `as<frameset>()` fails silently → **no odom**.

### Odom vs image publish rates

- **Odom:** event-driven on latest pose slot (`pose_seq`). No FPS throttle. No queue — intermediate poses overwritten if worker lags. Target ≈ `infra_fps` (60).
- **Color:** every-Nth sensor frame: `stride = round(color_fps / color_publish_fps)` (e.g. 30/15 → every 2nd). Not wall-clock (wall-clock drifted to ~13 Hz).
- **Depth:** still wall-clock period in `depth_worker` (not yet converted to every-Nth).

### Timestamps

- cuVSLAM Track: device frame metadata (ns)
- Published odom/TF/images: `node->now()` (avoid absurd nvblox TF delays from device timebase)

### Visual preset / clip

- Wired into C++ node from `realsense_cuvslam.*` with fallback to `camera.*`
- rs400 presets: 0 custom … **3 high_accuracy** … 6 remove_ir_pattern
- Apply preset **before** emitter/clip (preset can reset them)
- Note: old SSoT comment `visual_preset: 1 # High Accuracy` was **wrong** (1 = default)

## nvblox range / “map looks short” lessons

Integration can be fine while **viz** looks capped. Check node dump for:

| Param | Role | Typical trap |
|-------|------|----------------|
| `static_mapper.projective_integrator_max_integration_distance_m` | Depth/color TSDF rays | Was **5 m** in yaml |
| `static_mapper.lidar_projective_integrator_max_integration_distance_m` | Lidar TSDF rays | Separate key; wire both |
| `map_clearing_radius_m` | Deletes blocks beyond radius | Was **7 m** |
| `layer_visualization_exclusion_radius_m` | **RViz mesh/layer only** | Default **5 m** — looked like hard map limit |
| `max_back_projection_distance` | Debug back-proj cloud only | Default **5 m** |

SSoT now sets clearing/viz ~32 m and integrate max 15 m; launch applies dotted overrides e.g. `static_mapper.projective_integrator_max_integration_distance_m`.

**Weighting:** `projective_integrator_weighting_mode: "constant_dropoff"` (full weight on/in front of surface; linear taper behind within truncation). No plain `"linear"` — closest is `"linear_with_max"`.

D455 has **no** per-pixel depth confidence stream (that’s RS500/L515-style).

## Launch / SSoT gotchas

- When `realsense_cuvslam.enabled`, launch forces `camera.enabled=false` and `vslam.enabled=false`.
- `realsense_cuvslam.enable_depth` is authoritative — do not OR it with `nvblox_needs_depth` in launch.
- Params under `realsense_cuvslam:` must be read via `rsc.get(...)` in launch; `camera.*` alone is ignored on that path (hit this with `visual_preset`, `clip_distance`, `color_fps`).
- `decay_tsdf_rate_hz: 0` when no depth (view-based decay spams “Last view not set” and wipes map).
- Lidar for nvblox needs **organized** PointCloud2 (`width*height == points`); model FoV in SSoT must match.

## Timing debug (still in node)

Every 2 s:

```
cuVSLAM timing (...): vo mean/max=... slam ... iter ... track=Hz ir_frames=Hz skip_ts=...
```

Interpretation used in this chat:

- VO ~1.5 ms, track==ir_frames, skip_ts=0 → not cuVSLAM compute
- iter ~33 ms / track ~30 Hz → waiting on frames (was color sync)
- With/without nvblox/lidar: same Track times → GPU contention not the odom rate issue

## Approximate current SSoT intent (verify file; may drift)

```yaml
realsense_cuvslam:
  enabled: true
  infra_fps: 60
  color_fps: 30
  enable_color: true
  color_publish_fps: 15.0
  enable_depth: true          # was false during rate debugging; check live SSoT
  align_depth: false
  visual_preset: 3
  clip_distance: 6.0

lidar:
  enabled: true
  # cloud_scan_num etc. as configured

nvblox:
  # use_depth / use_lidar / use_color from SSoT
  lidar_projective_integrator_max_integration_distance_m: 15.0
  projective_integrator_max_integration_distance_m: 15.0
  map_clearing_radius_m: 32.0
  layer_visualization_exclusion_radius_m: 32.0
  decay_tsdf_rate_hz: 0.0     # if no depth view
```

## Non-goals / left alone

- No IMU/VIO in the C++ node yet
- Python `realsense_cuvslam_stereo.py` kept as smoke/reference
- Unitree lidar baud hardcoded in vendor node (not moved to SSoT)
- Depth publish still timer-based (not every-Nth like color)

## Suggested next work

1. If depth publish rate looks soft, apply same every-Nth logic as color (`infra_fps` / `depth_publish_fps`).
2. Re-verify odom ~60 Hz **with color+depth+nvblox+lidar** all on; watch timing logs.
3. If `align_depth: true` needed for people mask: depth worker uses syncer+`rs2::align` — validate under load.
4. Keep launch/SSoT param wiring consistent: any new `realsense_cuvslam:` key must be passed in `_build_realsense_cuvslam_node`.

## Quick health checks

```bash
ros2 topic hz /visual_slam/tracking/odometry          # ~60
ros2 topic hz /camera/color/image_raw                 # ~15 with stride
# Node log: "decoupled sensors", "color publish: every N sensor frame(s)"
# nvblox dump: layer_visualization_exclusion_radius_m and max_integration == expected
```
