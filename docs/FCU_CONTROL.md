# FCU control (in-process MAVLink + EKF + pluggable LL)

Replaces the classic `mavros_node` + `ekf_ins_node` + standalone LL triangle
with one process, RealSense-style threads, and ROS only at the edge.

## Enable

In `config/SSoT.yaml` (one file per robot):

```yaml
fcu_control:
  enabled: true
  ll_controller: ackermann   # or holonomic
```

Bring-up order: `stereo_composite` (VSLAM) then `fcu_control` (in-process EKF).
This launch does not start MAVROS, standalone EKF, or standalone `vesc_driver`.

`ll_controller` selects a self-registered controller (`ackermann` or
`holonomic`). Tunables live under the shared `fcu_control.ll.*` namespace;
each controller declares only the fields it needs. Ship a separate SSoT per
robot — see `config/SSoT.holonomic.example.yaml` for the mecanum layout.

## Threads

| Thread | Affinity param | Job |
|--------|----------------|-----|
| ROS executor (main) | — | libmavconn callbacks → slots; GCS bridge; high-rate edge timer; vision / play_tune subs |
| `aux` (~`aux_publish_hz`) | — | RC, armed, GPS sats/h_acc, mission `Path` (kept off the hot path) |
| `ekf_worker` | `ekf_cpu` | IMU-paced `estimator_ekf` |
| `ll_worker` | `ll_cpu` | IMU-paced pluggable LL (self-actuating) |
| `vesc` (if `fcu_control.vesc.enabled`) | — | UART `requestState` @ `vesc.telemetry_hz` → `FcuBus.vesc`; edge republishes `/sensors/core` |
| `ntrip` (if `ntrip.enabled`) | — | Caster HTTP → complete RTCM3 frames on `FcuBus.rtcm` (FIFO, not latest-wins) |

Hot path uses `LatestSlot` (`include/hound_core/fcu_slots.hpp`) with `fresh` /
`consume_fresh` for GPS/VSLAM/ICP. No 200 Hz IMU on DDS.
No `mavros_msgs` on the ROS edge — stock `std_msgs` / `nav_msgs` / `sensor_msgs` only.
When VESC is embedded, launch does not start standalone `vesc_driver` (exclusive VESC tty).

NTRIP (`fcu_control.ntrip.enabled`) is a background HTTP client. Complete RTCM3
frames are queued on `FcuBus.rtcm`; the high-rate ROS edge timer injects them as
`GPS_RTCM_DATA` **and** republishes each frame on `ntrip.rtcm_topic` (default
`~/rtcm`, `std_msgs/UInt8MultiArray`). The EKF still only sees `GPS_RAW_INT`
(better `eph` / `fix_type` once the chip locks). Needs an RTK-capable GPS on the
FCU.

ROS type choices for RTCM (we use `UInt8MultiArray` to stay off `mavros_msgs`):

| Type | Notes |
|------|--------|
| `std_msgs/UInt8MultiArray` | **Current.** Zero new deps; one complete RTCM3 frame in `.data`. |
| `rtcm_msgs/Message` | GNSS-ecosystem standard (`header` + `uint8[] message`). Used by many u-blox / NTRIP clients. Apt: `ros-$ROS_DISTRO-rtcm-msgs`. |
| `mavros_msgs/RTCM` | Same shape (`header` + `uint8[] data`). Only needed if something must feed MAVROS `gps_rtk` (`~/send_rtcm`). Avoided on this edge on purpose. |
| Custom msg | Same as the two above; only worth it if you refuse both packages. |

Do **not** publish MAVLink `GPS_RTCM_DATA` fragments on ROS — publish whole RTCM3 frames (what the NTRIP queue already holds).

## MAVLink I/O

**RX (FCU):** `RAW_IMU` / `SCALED_IMU`, `ATTITUDE_QUATERNION`, `SCALED_PRESSURE`,
`GPS_RAW_INT`, `RC_CHANNELS`, `LOCAL_POSITION_NED`, mission + param acks, heartbeat.

**TX:** heartbeat, `PARAM_SET` / `SET_MESSAGE_INTERVAL` at boot, `VISION_POSITION_ESTIMATE`
(optional), `MANUAL_CONTROL` (ackermann), `RC_CHANNELS_OVERRIDE` + `SET_MODE`
(holonomic), `PLAY_TUNE_V2` (from `~/play_tune`), `GPS_RTCM_DATA` (NTRIP, from the
ROS edge timer), mission download requests.

**GCS:** optional second `libmavconn` URL (`gcs_url`). Forwards FCU→GCS with optional
msgid throttle; blocks GCS `REQUEST_DATA_STREAM` when
`gcs_block_stream_requests: true` so QGC cannot stomp USB rates.

## ROS edge

High-rate (~`ros_publish_hz`):

| Topic | Content |
|-------|---------|
| `~/imu` `~/mag` `~/baro` `~/gps/fix` | Downsampled sensors |
| `ekf/odometry` (param) | Onboard EKF (full rate from worker); `child_frame=base_link` |
| TF `odom` → `base_link` | Same pose when `publish_ekf_tf: true` (VSLAM does not broadcast this) |
| `~/ap/local_odometry` | ArduPilot local pose for compare |
| `~/control_state` | `Float64MultiArray[17]`: pos,rpy,vel,A,G,steer,wheelspeed (BeamNG / PDef; IMU-synced in `ll_worker`) |

Aux thread (~`aux_publish_hz`):

| Topic | Type | Content |
|-------|------|---------|
| `~/state/armed` | `std_msgs/Bool` | Armed flag |
| `~/rc/in` | `std_msgs/UInt16MultiArray` | RC channels (µs) |
| `~/gps/satellites` | `std_msgs/UInt8` | Sat count |
| `~/gps/h_acc_mm` | `std_msgs/UInt32` | Horizontal accuracy (mm, from eph) |
| `~/mission/path` | `nav_msgs/Path` | Position WPs in map ENU (`ext_nav_origin`) |

Also:

| Topic | Content |
|-------|---------|
| `~/play_tune` (sub) | `std_msgs/String` tune → MAVLink `PLAY_TUNE_V2` |
| `~/ekf_reset` (sub) | `std_msgs/Empty` → hard-reset EKF (kill worker thread + relaunch fresh filter) |
| `/low_level_diagnostics` | LL status |
| `/control_limits` | Ackermann only (from AckermannLlController) |

### EKF measurement delays (`fcu_control.delays_ms`)

Per-source fusion delays (ms) for state recall at `IMUmsec - delay`. Tune independently — GPS, VSLAM, and ICP are not the same latency.

| Key | Applied to |
|-----|------------|
| `gps_pos` / `gps_vel` | GPS fusion |
| `vslam_pos` / `vslam_vel` / `vslam_yaw` | Ext-nav slot 0 (VSLAM) |
| `icp_pos` / `icp_vel` / `icp_yaw` | Ext-nav slot 1 (reserved; ICP is align-only today) |
| `baro` / `mag` | Baro height / magnetometer |

### Bus freshness (GPS / VSLAM / ICP)

Writers only `write()` when the source timestamp advances (`GPS_RAW_INT.time_usec`,
odom/ICP `header.stamp`). `GPS_RAW_INT` is requested at **200 Hz** so stamp
granularity is ≤ ~5 ms when AP updates a fix.

`LatestSlot` keeps last data for ROS/NTRIP (`copy_latest`). EKF uses
`consume_fresh()` — copies only if `fresh`, then clears `fresh` (data retained).
So GPS/VSLAM fuse only on new measurements; no fuse rate cap.

Mag/baro still use latest-wins + `mag_max_hz` / `baro_max_hz` (mag has no
independent freshness on `RAW_IMU`).

## Compare AP vs onboard EKF

Same vision stream can feed both (`send_vision_to_fcu: true`). Diff
`ekf/odometry` vs `hound_fcu_control/ap/local_odometry` in a common frame
(watch ENU/NED and origin).

## Files

- `src/hound_fcu_control_modular_node.cpp` — ROS wire-up + registry selection
- `src/ackermann_ll_controller.cpp` / `src/holonomic_ll_controller.cpp` — controllers
- `src/vesc_runner.cpp` — in-process VESC UART telemetry → `FcuBus.vesc`
- `src/ntrip_runner.cpp` — NTRIP caster client → `FcuBus.rtcm`
- `src/mavlink_bridge.cpp` — FCU/GCS MAVLink
- `include/hound_core/fcu_slots.hpp` — shared bus
- `include/hound_core/low_level_controller.hpp` — pluggable interface
