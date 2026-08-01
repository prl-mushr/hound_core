# Hardware interfaces (modular path)

## Controller selection (SSoT)

```yaml
fcu_control:
  enabled: true
  ll_controller: ackermann   # or holonomic
```

Launch always starts `hound_fcu_control_modular_node`. Controllers self-register;
adding a new robot type means adding one controller file + one CMake source line
+ setting `ll_controller` in that robot's SSoT.

`realsense_cuvslam.architecture: legacy|modular` remains an independent A/B switch.

## Modular layout

```
SensorBoard ──► FcuBus ──► EkfRunner / LlRunner
MavlinkBridge (owns MAVLink SensorBoard + FCU/GCS TX)

LowLevelController (ackermann | holonomic | …)
    └── self-actuates via MavlinkBridge inside tick_imu

CameraDevice (stereo + RGB + depth)
    │ wait_stereo
    ▼
VslamBackend (cuVSLAM) ──► odom / TF
```

| Interface | First adapter | Future |
|-----------|---------------|--------|
| `SensorBoard` | MAVLink FCU decode | ROS/sim proprioception |
| `MavlinkBridge` | libmavconn FCU+GCS | — |
| `LowLevelController` | Ackermann / Holonomic | quadruped, etc. |
| `CameraDevice` | `RealsenseCameraDevice` | Orbbec, etc. |
| `VslamBackend` | `CuvslamBackend` | other VO |

## Smoke checklist

Camera (modular):

```bash
# SSoT: realsense_cuvslam.architecture: modular
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic hz /camera/color/image_raw   # if enable_color
```

FCU (modular; needs ArduPilot connected):

```bash
# SSoT: fcu_control.enabled: true, ll_controller: ackermann|holonomic
ros2 topic hz /hound_fcu_control/ekf/odometry   # or configured ekf_odom_topic
ros2 topic echo /low_level_diagnostics
```

### Holonomic bench (before first drive)

1. Set `ll_controller: holonomic` and copy params from `SSoT.holonomic.example.yaml`.
2. Confirm ArduPilot Rover Omni `RCMAP_*` / `SERVO*_FUNCTION` match `ll.output_ch_*`.
3. With wheels off the ground, flip RC mode switch Manual → Auto and watch
   `RC_CHANNELS_OVERRIDE` PWM on channels 8/9/10 (or configured indices).
4. Stop publishing autonomy Twist / unplug RC past `ll.cmd_vel_timeout` and
   confirm HOLD mode request + neutral PWM.

Do not start mavros alongside `fcu_control` on the same tty — they fight for the link.
