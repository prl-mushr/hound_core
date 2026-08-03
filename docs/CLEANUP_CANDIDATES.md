# Cleanup candidates

Smoke test after reorganization passed (2026-08-01).

## Removed (2026-08-01)

- `hound_core_old/`
- `opendubs-core/`
- `bimanual/`
- `nanosam_data_legacy/`

## Kept with `COLCON_IGNORE`

- `hand_pose/`
- `zed-ros2-wrapper/`

## Still optional (not removed)

| Path | Why |
|------|-----|
| `spencer_port/` | People-tracking demos only |
| `Fast-FoundationStereo/` | Stereo depth R&D |
| `bags/` | Local bag data |

## Keep

| Path | Why |
|------|-----|
| `mushr/` | Explicit keep |
| `hound_core/`, `composite_sensing/`, `perception_models/` | Active stack |
| `vesc_ros2/`, `mavros/`, `unilidar_sdk/`, `pointcloud_deskewer/` | Launch deps |
| `Inertial_nav_shim/` (+ `nav_filter`), `inertial_nav_ros2/` | EKF |
| `nanosam-src/` | Pip editable for NanoSAM |
