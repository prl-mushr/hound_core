# Hardware interfaces (modular A/B path)

HOUND can run either the **legacy** monoliths or the **modular** extract.
Default is legacy so production stays unchanged.

## A/B switch (SSoT)

```yaml
realsense_cuvslam:
  architecture: legacy   # or modular → realsense_cuvslam_modular_node

fcu_control:
  architecture: legacy   # or modular → hound_fcu_control_modular_node
```

Launch picks the executable from `architecture`. Same params/topics either way.
Rollback: set `legacy` and relaunch (no git revert).

## Modular layout

```
SensorBoard ──► FcuBus ──► EkfRunner / LlRunner
MavlinkBridge (owns MAVLink SensorBoard + FCU/GCS TX)

CameraDevice (stereo + RGB + depth)
    │ wait_stereo
    ▼
VslamBackend (cuVSLAM) ──► odom / TF
    │ poll_color / poll_depth
    ▼
ROS image pubs (seg / nvblox)
```

| Interface | First adapter | Future |
|-----------|---------------|--------|
| `SensorBoard` | MAVLink FCU decode | ROS/sim proprioception, VESC on board |
| `MavlinkBridge` | libmavconn FCU+GCS | — |
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
# SSoT: fcu_control.enabled: true, architecture: modular
ros2 topic hz /hound_fcu_control/ekf/odometry   # or configured ekf_odom_topic
```

Do not start both legacy and modular FCU (or both camera nodes) — they fight for the same tty / USB device.
