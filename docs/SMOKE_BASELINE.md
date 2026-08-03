# Smoke-test baseline (workspace reorganization)

Success = `stereo_composite` + Dora segmentation (+ optional nvblox).
No stock `camera` / Isaac `vslam` SSoT blocks.

| Block | Key flags |
|-------|-----------|
| `stereo_composite` | `enabled: true`, `architecture: modular`, color on, depth on |
| `segmentation` | `enabled: true` |
| `yolo_world` | `enabled: false` (optional) |
| `nvblox` | enable for map smoke; `backend: hound` (default) or `ros` |
| `fcu_control` / `vesc` | optional (needs battery) |

Expected topics (unchanged contracts):
- `/camera/color/image_raw`, odom `/visual_slam/tracking/odometry`
- `/segmentation/refined_people_mask` (and trav/nontrav labels)
- with `nvblox.enabled: true` + `backend: hound`: `/hound_mapping/local_map` (`LocalMap`), `/hound_mapping/elev_color`
- with `backend: ros`: stock `nvblox_node` layer/mesh topics
- with `viz.enabled: true`: Viser at `http://<host>:8080/` (install `viser` via `hound_viz` `install_deps.sh`)
