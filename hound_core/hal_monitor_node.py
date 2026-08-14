"""Diagnostics monitor for HOUND: health checks, tunes, and control latency.

ROS 2 port of the legacy HAL_9000.py monitor node (recording and TF removed;
rosbag lives in bag_recorder_node).
Talks to fcu_control via stock ROS messages (no mavros_msgs).
"""

from __future__ import annotations

import glob
import os
import platform
import shlex
import subprocess
import time
from collections import deque
from pathlib import Path
from typing import Deque, Optional

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, Float64MultiArray, String, UInt8, UInt32
from vesc_msgs.msg import VescStateStamped

_LEGACY_GPU_FREQ_PATHS = (
    "/sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu/cur_freq",
    "/sys/devices/17000000.ga10b/devfreq/17000000.ga10b/cur_freq",
    "/sys/devices/17000000.ga10b/devfreq/17000000.ga10b/max_freq",
)
_GPU_FREQ_GLOB_PATTERNS = (
    "/sys/devices/platform/bus@0/*/devfreq/*/cur_freq",
    "/sys/devices/*/devfreq/*/cur_freq",
    "/sys/devices/*/devfreq/*/max_freq",
)


class TopicHzTracker:
    """Rolling-window topic rate estimator (replaces rospy's ROSTopicHz)."""

    def __init__(self, window_size: int = 10) -> None:
        self._times: Deque[float] = deque(maxlen=window_size)

    def tick(self) -> None:
        self._times.append(time.monotonic())

    def get_hz(self) -> float:
        if len(self._times) < 2:
            return 0.0
        dt = self._times[-1] - self._times[0]
        if dt <= 0.0:
            return 0.0
        return (len(self._times) - 1) / dt


class HalMonitorNode(Node):
    TUNES = {
        "low level ready": "MLO2L2A",
        "low battery": "MSO3L8dddP8ddd",
        "GPS good": "MSO3L8dP8d",
        "GPS bad": "MSO3L8ddP8dd",
        "camera ready": "MLO2L2C",
    }

    def __init__(self) -> None:
        super().__init__("HAL_9000")

        self._on_jetson = platform.machine() == "aarch64"
        self._fcu_hz = TopicHzTracker()
        self._camera_hz = TopicHzTracker()
        self._control_state_hz = TopicHzTracker()
        self._cmd_hz = TopicHzTracker()

        self._declare_parameters()

        self._fcu_init = False
        self._camera_init = False
        self._gps_status = False
        self._last_sats: Optional[int] = None
        self._last_h_acc_mm: Optional[int] = None
        self._last_map_clear = time.time()
        self._gpu_freq_path: Optional[str] = None

        # Receive-time latency: control_state arrival → cmd arrival (monotonic).
        self._last_control_state_mono: Optional[float] = None
        self._control_latency_ms: float = 0.0
        self._latency_ema_alpha = 0.2

        self._notification_pub = self.create_publisher(
            String, self._p("fcu.notification_topic"), 10
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._p("diagnostics_topic"), 2
        )

        self._fcu_monitor_enabled = bool(
            self.get_parameter("fcu.monitor_enabled").value
        )

        if self._fcu_monitor_enabled:
            self.create_subscription(
                Imu,
                self._p("fcu.monitor_topic"),
                lambda _msg: self._fcu_hz.tick(),
                qos_profile_sensor_data,
            )
        if self.get_parameter("camera.monitor_enabled").value:
            self.create_subscription(
                Image,
                self._p("camera.monitor_topic"),
                lambda _msg: self._camera_hz.tick(),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            VescStateStamped, self._p("vesc.topic"), self._voltage_cb, 1
        )
        self.create_subscription(
            UInt8, self._p("fcu.gps_satellites_topic"), self._gps_sats_cb, 1
        )
        self.create_subscription(
            UInt32, self._p("fcu.gps_h_acc_topic"), self._gps_h_acc_cb, 1
        )
        self.create_subscription(
            Bool, self._p("fcu.armed_topic"), self._armed_cb, 10
        )
        self.create_subscription(
            Float64MultiArray,
            self._p("fcu.control_state_topic"),
            self._control_state_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AckermannDriveStamped,
            self._p("cmd_topic"),
            self._cmd_cb,
            10,
        )

        startup_delay_s = float(self.get_parameter("startup_delay_s").value)
        if startup_delay_s > 0.0:
            self.get_logger().info(
                f"HAL monitor waiting {startup_delay_s:.1f}s before health actions"
            )
            time.sleep(startup_delay_s)

        if self.get_parameter("usb_reset_on_start").value:
            device = str(self.get_parameter("usb_reset_device").value)
            os.system(f'usbreset "{device}"')

        failure_action = str(self.get_parameter("fcu.failure_action").value).strip()
        if failure_action:
            self.get_logger().info(f"Running FCU failure action: {failure_action}")
            subprocess.run(shlex.split(failure_action), check=False)

        self.create_timer(2.0, self._main_loop_tick)
        self.get_logger().info("HAL monitor online (diagnostics-only)")

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _declare_parameters(self) -> None:
        self.declare_parameter("diagnostics_topic", "/SOC_diagnostics")
        self.declare_parameter("startup_delay_s", 15.0)
        self.declare_parameter("usb_reset_on_start", True)
        self.declare_parameter(
            "usb_reset_device", "ChibiOS/RT Virtual COM Port"
        )

        self.declare_parameter("battery_voltage_threshold", 14.0)
        self.declare_parameter("gps_min_satellites", 16)
        self.declare_parameter("gps_max_h_acc_mm", 1000)

        self.declare_parameter("fcu.monitor_enabled", True)
        self.declare_parameter("fcu.monitor_topic", "/hound_fcu_control/imu")
        self.declare_parameter("fcu.expected_fps", 50.0)
        self.declare_parameter("fcu.failure_action", "")
        self.declare_parameter("fcu.armed_topic", "/hound_fcu_control/state/armed")
        self.declare_parameter(
            "fcu.gps_satellites_topic", "/hound_fcu_control/gps/satellites"
        )
        self.declare_parameter(
            "fcu.gps_h_acc_topic", "/hound_fcu_control/gps/h_acc_mm"
        )
        self.declare_parameter(
            "fcu.notification_topic", "/hound_fcu_control/play_tune"
        )
        self.declare_parameter(
            "fcu.control_state_topic", "/hound_fcu_control/control_state"
        )

        self.declare_parameter("cmd_topic", "/hound_nav/cmd_ackermann")

        self.declare_parameter("camera.monitor_enabled", True)
        self.declare_parameter(
            "camera.monitor_topic", "/camera/infra1/image_rect_raw"
        )
        self.declare_parameter("camera.expected_fps", 60.0)

        self.declare_parameter("vesc.topic", "/sensors/core")

        self.declare_parameter("elevation_map_clear.enabled", False)
        self.declare_parameter(
            "elevation_map_clear.service", "/elevation_mapping/clear_map"
        )
        self.declare_parameter("elevation_map_clear.min_interval_s", 1.0)

        self.declare_parameter("jetson_diagnostics.enabled", True)
        self.declare_parameter(
            "jetson_diagnostics.gpu_freq_path",
            "",
        )

    def _resolve_gpu_freq_path(self) -> Optional[str]:
        configured = str(self.get_parameter("jetson_diagnostics.gpu_freq_path").value)
        if configured and Path(configured).is_file():
            return configured
        for path in _LEGACY_GPU_FREQ_PATHS:
            if Path(path).is_file():
                return path
        for pattern in _GPU_FREQ_GLOB_PATTERNS:
            matches = sorted(glob.glob(pattern))
            if matches:
                return matches[0]
        return None

    def _read_gpu_freq_ghz(self) -> float:
        if self._gpu_freq_path is None:
            self._gpu_freq_path = self._resolve_gpu_freq_path()
        if not self._gpu_freq_path:
            return 0.0
        try:
            raw = Path(self._gpu_freq_path).read_text().strip()
            return float(raw) * 1e-9
        except (OSError, ValueError):
            return 0.0

    def _publish_notification(self, message: str) -> None:
        tune = self.TUNES.get(message)
        if tune is None:
            return
        if self._notification_pub.get_subscription_count() == 0:
            return
        msg = String()
        msg.data = tune
        self._notification_pub.publish(msg)

    def _control_state_cb(self, _msg: Float64MultiArray) -> None:
        now = time.monotonic()
        self._control_state_hz.tick()
        self._last_control_state_mono = now

    def _cmd_cb(self, _msg: AckermannDriveStamped) -> None:
        now = time.monotonic()
        self._cmd_hz.tick()
        if self._last_control_state_mono is None:
            return
        latency_ms = (now - self._last_control_state_mono) * 1000.0
        if self._control_latency_ms <= 0.0:
            self._control_latency_ms = latency_ms
        else:
            a = self._latency_ema_alpha
            self._control_latency_ms = (
                a * latency_ms + (1.0 - a) * self._control_latency_ms
            )

    def _voltage_cb(self, msg: VescStateStamped) -> None:
        threshold = float(self.get_parameter("battery_voltage_threshold").value)
        if msg.state.voltage_input < threshold:
            self._publish_notification("low battery")

    def _gps_sats_cb(self, msg: UInt8) -> None:
        self._last_sats = int(msg.data)
        self._maybe_update_gps_status()

    def _gps_h_acc_cb(self, msg: UInt32) -> None:
        self._last_h_acc_mm = int(msg.data)
        self._maybe_update_gps_status()

    def _maybe_update_gps_status(self) -> None:
        if self._last_sats is None or self._last_h_acc_mm is None:
            return
        min_sats = int(self.get_parameter("gps_min_satellites").value)
        max_h_acc = int(self.get_parameter("gps_max_h_acc_mm").value)
        good = self._last_sats >= min_sats and self._last_h_acc_mm <= max_h_acc
        if good and not self._gps_status:
            self._publish_notification("GPS good")
            self._gps_status = True
        elif not good and self._gps_status:
            self._publish_notification("GPS bad")
            self._gps_status = False

    def _armed_cb(self, msg: Bool) -> None:
        if msg.data:
            return
        if not self.get_parameter("elevation_map_clear.enabled").value:
            return
        min_interval = float(
            self.get_parameter("elevation_map_clear.min_interval_s").value
        )
        if time.time() - self._last_map_clear < min_interval:
            return
        service = str(self.get_parameter("elevation_map_clear.service").value)
        self.get_logger().debug(f"Clearing elevation map via {service}")
        subprocess.run(
            ["ros2", "service", "call", service, "std_srvs/srv/Trigger", "{}"],
            check=False,
        )
        self._last_map_clear = time.time()

    def _publish_diagnostics(self) -> None:
        if not (
            self._on_jetson
            and self.get_parameter("jetson_diagnostics.enabled").value
        ):
            return

        temps = os.popen(
            "cat /sys/devices/virtual/thermal/thermal_zone*/temp"
        ).readlines()
        avg_temp = 0.0
        if temps:
            avg_temp = float(np.mean([float(t.strip()) * 1e-3 for t in temps]))

        freqs = os.popen(
            "cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq"
        ).readlines()
        avg_cpu = 0.0
        if freqs:
            avg_cpu = float(np.mean([float(t.strip()) * 1e-6 for t in freqs]))

        avg_gpu = self._read_gpu_freq_ghz()
        gpu_path = self._gpu_freq_path or ""

        status = DiagnosticStatus()
        status.name = "SOC"
        status.level = DiagnosticStatus.OK
        status.message = "jetson"
        status.values = [
            KeyValue(key="avg_temp", value=str(round(avg_temp, 2))),
            KeyValue(key="avg_cpu", value=str(round(avg_cpu, 2))),
            KeyValue(key="avg_gpu", value=str(round(avg_gpu, 2))),
            KeyValue(key="gpu_freq_path", value=gpu_path),
            KeyValue(key="fcu_init", value=str(self._fcu_init)),
            KeyValue(key="camera_init", value=str(self._camera_init)),
            KeyValue(
                key="control_state_hz",
                value=str(round(self._control_state_hz.get_hz(), 2)),
            ),
            KeyValue(key="cmd_hz", value=str(round(self._cmd_hz.get_hz(), 2))),
            KeyValue(
                key="control_latency_ms",
                value=str(round(self._control_latency_ms, 2)),
            ),
        ]

        msg = DiagnosticArray()
        msg.status.append(status)
        self._diagnostics_pub.publish(msg)

    def _main_loop_tick(self) -> None:
        if self._fcu_monitor_enabled:
            fcu_fps = float(self._p("fcu.expected_fps"))
            fcu_hz = self._fcu_hz.get_hz()
            if fcu_hz >= 0.8 * fcu_fps and not self._fcu_init:
                self._fcu_init = True
                self._publish_notification("low level ready")
            elif fcu_hz < 0.8 * fcu_fps and self._fcu_init:
                self._fcu_init = False
                failure_action = str(self._p("fcu.failure_action")).strip()
                if failure_action:
                    subprocess.run(shlex.split(failure_action), check=False)

        if self.get_parameter("camera.monitor_enabled").value:
            camera_fps = float(self._p("camera.expected_fps"))
            camera_hz = self._camera_hz.get_hz()
            if camera_hz >= 0.8 * camera_fps and not self._camera_init:
                self._camera_init = True
                self._publish_notification("camera ready")
                self.get_logger().info("camera READY")
            elif camera_hz < 0.8 * camera_fps:
                self._camera_init = False

        self._publish_diagnostics()


def main() -> None:
    rclpy.init()
    node = HalMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
