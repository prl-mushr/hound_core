# FCU control (in-process MAVLink + EKF + LL)

Replaces the classic `mavros_node` + `ekf_ins_node` + `hound_ll_control_node`
triangle with one process, RealSense-style threads, and ROS only at the edge.

## Enable

In `config/SSoT.yaml`:

```yaml
fcu_control:
  enabled: true
mavros:
  enabled: false   # forced off by launch if both set
ekf:
  enabled: false
low_level_control:
  enabled: false
```

Launch will refuse to co-start mavros / standalone EKF / standalone LL when
`fcu_control` owns the FCU tty.

## Threads

| Thread | Affinity param | Job |
|--------|----------------|-----|
| ROS executor (main) | — | libmavconn callbacks → slots; GCS bridge; ~50 Hz pubs; vision/VESC/auto subs |
| `ekf_worker` | `ekf_cpu` | IMU-paced `estimator_ekf` |
| `ll_worker` | `ll_cpu` | IMU-paced LL → `MANUAL_CONTROL` TX |

Hot path uses `LatestSlot` (`include/hound_core/fcu_slots.hpp`). No 200 Hz IMU on DDS.

## MAVLink I/O

**RX (FCU):** `RAW_IMU` / `SCALED_IMU`, `ATTITUDE_QUATERNION`, `SCALED_PRESSURE`,
`GPS_RAW_INT`, `RC_CHANNELS`, `LOCAL_POSITION_NED`, mission + param acks, heartbeat.

**TX:** heartbeat, `PARAM_SET` / `SET_MESSAGE_INTERVAL` at boot, `VISION_POSITION_ESTIMATE`
(optional), `MANUAL_CONTROL`, mission download requests.

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
| `/low_level_diagnostics` `/control_limits` | LL status |

## Compare AP vs onboard EKF

Same vision stream can feed both (`send_vision_to_fcu: true`). Diff
`ekf/odometry` vs `hound_fcu_control/ap/local_odometry` in a common frame
(watch ENU/NED and origin).

## Files

- `src/hound_fcu_control_node.cpp` — router + workers
- `src/ll_controller.cpp` — shared with classic `hound_ll_control_node`
- `include/hound_core/fcu_slots.hpp`
