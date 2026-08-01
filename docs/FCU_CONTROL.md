# FCU control (in-process MAVLink + EKF + pluggable LL)

Replaces the classic `mavros_node` + `ekf_ins_node` + standalone LL triangle
with one process, RealSense-style threads, and ROS only at the edge.

## Enable

In `config/SSoT.yaml` (one file per robot):

```yaml
fcu_control:
  enabled: true
  ll_controller: ackermann   # or holonomic
mavros:
  enabled: false   # forced off by launch if both set
ekf:
  enabled: false
```

Launch will refuse to co-start mavros / standalone EKF when `fcu_control`
owns the FCU tty.

`ll_controller` selects a self-registered controller (`ackermann` or
`holonomic`). Tunables live under the shared `fcu_control.ll.*` namespace;
each controller declares only the fields it needs. Ship a separate SSoT per
robot — see `config/SSoT.holonomic.example.yaml` for the mecanum layout.

## Threads

| Thread | Affinity param | Job |
|--------|----------------|-----|
| ROS executor (main) | — | libmavconn callbacks → slots; GCS bridge; ~50 Hz pubs; vision subs |
| `ekf_worker` | `ekf_cpu` | IMU-paced `estimator_ekf` |
| `ll_worker` | `ll_cpu` | IMU-paced pluggable LL (self-actuating) |

Hot path uses `LatestSlot` (`include/hound_core/fcu_slots.hpp`). No 200 Hz IMU on DDS.

## MAVLink I/O

**RX (FCU):** `RAW_IMU` / `SCALED_IMU`, `ATTITUDE_QUATERNION`, `SCALED_PRESSURE`,
`GPS_RAW_INT`, `RC_CHANNELS`, `LOCAL_POSITION_NED`, mission + param acks, heartbeat.

**TX:** heartbeat, `PARAM_SET` / `SET_MESSAGE_INTERVAL` at boot, `VISION_POSITION_ESTIMATE`
(optional), `MANUAL_CONTROL` (ackermann), `RC_CHANNELS_OVERRIDE` + `SET_MODE`
(holonomic), mission download requests.

**GCS:** optional second `libmavconn` URL (`gcs_url`). Forwards FCU→GCS with optional
msgid throttle; blocks GCS `REQUEST_DATA_STREAM` when
`gcs_block_stream_requests: true` so QGC cannot stomp USB rates.

## ROS edge (~`ros_publish_hz`)

| Topic | Content |
|-------|---------|
| `~/imu` `~/mag` `~/baro` `~/gps/fix` | Downsampled sensors |
| `ekf/odometry` (param) | Onboard EKF (full rate from worker) |
| `~/ap/local_odometry` | ArduPilot local pose for compare |
| `~/mission/waypoints` | Pulled mission |
| `/low_level_diagnostics` | LL status |
| `/control_limits` | Ackermann only (from AckermannLlController) |

## Compare AP vs onboard EKF

Same vision stream can feed both (`send_vision_to_fcu: true`). Diff
`ekf/odometry` vs `hound_fcu_control/ap/local_odometry` in a common frame
(watch ENU/NED and origin).

## Files

- `src/hound_fcu_control_modular_node.cpp` — ROS wire-up + registry selection
- `src/ackermann_ll_controller.cpp` / `src/holonomic_ll_controller.cpp` — controllers
- `src/mavlink_bridge.cpp` — FCU/GCS MAVLink
- `include/hound_core/fcu_slots.hpp` — shared bus
- `include/hound_core/low_level_controller.hpp` — pluggable interface
