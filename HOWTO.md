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
```

## Bag: seg + mapping

Does **not** start cameras / lidar / EKF / nav. Reads bag RGB, lidar, TF/odom.
Ignores `segmentation.enabled` and `nvblox.enabled`.

Default `prefix:=debug` puts outputs under `/debug/...` so they do not overwrite bag topics. Inputs stay on SSoT names.

```bash
ros2 launch hound_core hound_seg_mapping.launch.py prefix:=debug bag:=/path/to/bag
```

Split:

```bash
ros2 launch hound_core hound_seg_mapping.launch.py
ros2 bag play /path/to/bag --clock
```

Live names (no `/debug`), wall clock:

```bash
ros2 launch hound_core hound_seg_mapping.launch.py prefix:= use_sim_time:=false
```

RViz LocalMap topic when prefix is `debug`: `/debug/hound_mapping/local_map`.

Log LocalMap + control_state at 1 Hz (Ctrl+C writes start/goal pairs):

```bash
ros2 launch hound_core hound_seg_mapping.launch.py log_problems:=true
```

## Bag: mapping only

```bash
ros2 launch hound_core hound_mapping_replay.launch.py bag:=/path/to/bag prefix:=debug
# or
ros2 launch hound_core hound_mapping_replay.launch.py
ros2 bag play /path/to/bag --clock
```

## Nav only

Expects `/hound_mapping/local_map` and `/hound_fcu_control/control_state` already up (live stack or bag). Set `nav.enabled: false` in SSoT if `hound_core.launch.py` is also running.

```bash
ros2 launch hound_core hound_nav.launch.py
```

## mesh_pf only

Ignores `mesh_pf.enabled`. Default `prefix:=debug` → pose
`/debug/localization/mesh_pose`, node `/debug/mesh_pf`. Cloud stays
`/livox/cloud` (or SSoT `cloud_topic`). No pose seed — global init over
the map sidecar `init_bb`. Latches **map←odom** on `/tf_static` (needs
`odom←base_link`). A bag ArUco `/tf_static` map←odom will fight this —
drop that TF or set `mesh_pf.publish_tf: false`. Bag replay: `rviz_sim`.

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

User-authored no-go zones are a separate overlay (they do not edit the cake).
Empty `nvblox.lethal_map_path` means all free. When set, mapping loads the
mask onto the GPU and ANDs it onto LocalMap cost after slope/inpaint
(0 = lethal). Same XY frame as the LayerCake; `prior_xyz_yaw` is the
cake→live transform.

```bash
# top-down BEV from the cake, then draw polygons in the browser
python3 /root/colcon_ws/src/hound_mapping/scripts/author_lethal_map.py \
  --layercake /root/colcon_ws/maps/hound_tsdf.layercake
# writes /root/colcon_ws/maps/hound_tsdf.lethal.yaml (+ .lethal.f32)
# set nvblox.lethal_map_path to that yaml and restart mapping
```

If the sibling `.elev.yaml` is missing, the script runs `layercake_to_bev`
(needs a built `hound_mapping`). You can also extract first:

```bash
ros2 run hound_mapping layercake_to_bev -- \
  --layercake /root/colcon_ws/maps/hound_tsdf.layercake
```

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
# oneshot mesh export (default oneshot:=true — just run the node, do not wait on a service)
ros2 run hound_mapping layercake_to_mesh_node
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
