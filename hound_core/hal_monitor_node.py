"""Background monitor for HOUND: health checks, TF, tunes, and rosbag control.

ROS 2 port of the legacy HAL_9000.py monitor node.
"""

from __future__ import annotations

import glob
import os
import platform
import shlex
import signal
import subprocess
import time
from collections import deque
from pathlib import Path
from typing import Deque, List, Optional

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import GPSRAW, PlayTuneV2, RCIn, State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from tf2_ros import TransformBroadcaster
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
        "record start": "ML O3 L8 CD",
        "record stop": "ML O3 L8 DC",
        "GPS good": "MSO3L8dP8d",
        "GPS bad": "MSO3L8ddP8dd",
        "camera ready": "MLO2L2C",
    }

    def __init__(self) -> None:
        super().__init__("HAL_9000")

        self._on_jetson = platform.machine() == "aarch64"
        self._mavros_hz = TopicHzTracker()
        self._camera_hz = TopicHzTracker()
        self._tf_broadcaster = TransformBroadcaster(self)

        self._declare_parameters()
        self._load_record_topics()

        self._mavros_init = False
        self._camera_init = False
        self._recording_state = False
        self._gps_status = False
        self._last_map_clear = time.time()
        self._rosbag_proc: Optional[subprocess.Popen] = None
        self._record_cmd: List[str] = []
        self._gpu_freq_path: Optional[str] = None

        self._notification_pub = self.create_publisher(
            PlayTuneV2, self._p("mavros.notification_topic"), 10
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self._p("diagnostics_topic"), 2
        )

        self._mavros_monitor_enabled = bool(
            self.get_parameter("mavros.monitor_enabled").value
        )

        if self._mavros_monitor_enabled:
            self.create_subscription(
                Imu,
                self._p("mavros.monitor_topic"),
                lambda _msg: self._mavros_hz.tick(),
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
            RCIn, self._p("mavros.channel_topic"), self._channel_cb, 2
        )
        self.create_subscription(
            VescStateStamped, self._p("vesc.topic"), self._voltage_cb, 1
        )
        self.create_subscription(
            GPSRAW, self._p("mavros.gps_topic"), self._gps_cb, 1
        )
        self.create_subscription(
            PoseStamped, self._p("mavros.pose_topic"), self._pose_cb, 1
        )
        self.create_subscription(
            State, self._p("mavros.state_topic"), self._mavros_status_cb, 10
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

        failure_action = str(self.get_parameter("mavros.failure_action").value).strip()
        if failure_action:
            self.get_logger().info(f"Running MAVROS failure action: {failure_action}")
            subprocess.run(shlex.split(failure_action), check=False)

        self.create_timer(2.0, self._main_loop_tick)
        self.get_logger().info("HAL monitor online")

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _declare_parameters(self) -> None:
        self.declare_parameter("diagnostics_topic", "/SOC_diagnostics")
        self.declare_parameter("startup_delay_s", 15.0)
        self.declare_parameter("usb_reset_on_start", True)
        self.declare_parameter(
            "usb_reset_device", "ChibiOS/RT Virtual COM Port"
        )

        self.declare_parameter("bagdir", "/root/colcon_ws/bags/")
        self.declare_parameter(
            "record_topics_file",
            "/root/colcon_ws/src/hound_core/config/rosbag_record_topics.txt",
        )
        self.declare_parameter("record_split_duration_min", 5)
        self.declare_parameter("record_rc_channel", 3)
        self.declare_parameter("record_on_threshold", 1900)
        self.declare_parameter("record_off_threshold", 1100)

        self.declare_parameter("battery_voltage_threshold", 14.0)
        self.declare_parameter("gps_min_satellites", 16)
        self.declare_parameter("gps_max_h_acc_mm", 1000)

        self.declare_parameter("mavros.monitor_enabled", True)
        self.declare_parameter("mavros.monitor_topic", "/mavros/imu/data_raw")
        self.declare_parameter("mavros.expected_fps", 50.0)
        self.declare_parameter(
            "mavros.failure_action", "ros2 run mavros mav sys rate --all 50"
        )
        self.declare_parameter(
            "mavros.pose_topic", "/mavros/local_position/pose"
        )
        self.declare_parameter("mavros.state_topic", "/mavros/state")
        self.declare_parameter(
            "mavros.gps_topic", "/mavros/gpsstatus/gps1/raw"
        )
        self.declare_parameter(
            "mavros.notification_topic", "/mavros/play_tune"
        )
        self.declare_parameter("mavros.channel_topic", "/mavros/rc/in")
        self.declare_parameter("mavros.base_frame", "base_link")

        self.declare_parameter("camera.monitor_enabled", True)
        self.declare_parameter(
            "camera.monitor_topic", "/camera/infra1/image_rect_raw"
        )
        self.declare_parameter("camera.expected_fps", 60.0)
        self.declare_parameter("camera.pos", [0.15, 0.047, 0.04])
        self.declare_parameter("camera.rot", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("camera.depth_frame", "camera_link")
        self.declare_parameter(
            "camera.depth_optical_frame", "camera_infra1_optical_frame"
        )

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

    @staticmethod
    def _read_sysfs_freq_ghz(path: str) -> Optional[float]:
        try:
            with open(path, encoding="ascii") as f:
                return float(f.read().strip()) * 1e-9
        except OSError:
            return None

    def _gpu_freq_path_candidates(self) -> List[str]:
        override = str(self._p("jetson_diagnostics.gpu_freq_path")).strip()
        candidates: List[str] = []
        if override:
            candidates.append(override)

        candidates.extend(_LEGACY_GPU_FREQ_PATHS)

        discovered: List[str] = []
        for pattern in _GPU_FREQ_GLOB_PATTERNS:
            discovered.extend(glob.glob(pattern))

        def _rank(path: str) -> tuple:
            prefers_gpu = 0 if "gpu" in path.lower() else 1
            prefers_cur = 0 if path.endswith("/cur_freq") else 1
            return (prefers_gpu, prefers_cur, path)

        candidates.extend(sorted(set(discovered), key=_rank))

        unique: List[str] = []
        seen = set()
        for path in candidates:
            if path not in seen:
                seen.add(path)
                unique.append(path)
        return unique

    def _resolve_gpu_freq_path(self) -> Optional[str]:
        if self._gpu_freq_path and os.path.isfile(self._gpu_freq_path):
            return self._gpu_freq_path

        for path in self._gpu_freq_path_candidates():
            if os.path.isfile(path) and self._read_sysfs_freq_ghz(path) is not None:
                if path != self._gpu_freq_path:
                    self.get_logger().info(f"Using GPU freq sysfs path: {path}")
                self._gpu_freq_path = path
                return path

        if self._gpu_freq_path:
            self.get_logger().warning(
                f"GPU freq sysfs path unavailable: {self._gpu_freq_path}"
            )
        self._gpu_freq_path = None
        return None

    def _read_gpu_freq_ghz(self) -> float:
        path = self._resolve_gpu_freq_path()
        if path is None:
            return 0.0
        return self._read_sysfs_freq_ghz(path) or 0.0

    def _load_record_topics(self) -> None:
        topics_file = Path(str(self._p("record_topics_file")))
        if not topics_file.is_file():
            self.get_logger().warning(
                f"Record topics file not found: {topics_file}"
            )
            self._record_topics: List[str] = []
            return
        self._record_topics = [
            line.strip()
            for line in topics_file.read_text().splitlines()
            if line.strip() and not line.strip().startswith("#")
        ]

    def _publish_notification(self, message: str) -> None:
        tune = self.TUNES.get(message)
        if tune is None:
            return
        if self._notification_pub.get_subscription_count() == 0:
            return
        msg = PlayTuneV2()
        msg.format = 1
        msg.tune = tune
        self._notification_pub.publish(msg)

    def _start_recording(self) -> None:
        bagdir = Path(str(self._p("bagdir")))
        bagdir.mkdir(parents=True, exist_ok=True)
        split_min = int(self.get_parameter("record_split_duration_min").value)
        output = str(bagdir / "temp")
        self._record_cmd = [
            "ros2",
            "bag",
            "record",
            "-o",
            output,
            "--max-bag-duration",
            str(split_min * 60),
            *self._record_topics,
        ]
        self.get_logger().info(f"Starting bag record: {' '.join(self._record_cmd)}")
        self._rosbag_proc = subprocess.Popen(self._record_cmd)
        self._publish_notification("record start")

    def _stop_recording(self) -> None:
        if self._rosbag_proc is not None and self._rosbag_proc.poll() is None:
            self._rosbag_proc.send_signal(signal.SIGINT)
            try:
                self._rosbag_proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self._rosbag_proc.kill()
        self._rosbag_proc = None
        self._publish_notification("record stop")

        bagdir = Path(str(self._p("bagdir")))
        if not bagdir.is_dir():
            return
        candidates = sorted(
            (p for p in bagdir.iterdir() if p.name.startswith("temp")),
            key=lambda p: p.stat().st_mtime,
        )
        for i, path in enumerate(candidates):
            path.rename(bagdir / f"hound_{i}")

    def _voltage_cb(self, msg: VescStateStamped) -> None:
        threshold = float(self.get_parameter("battery_voltage_threshold").value)
        if msg.state.voltage_input < threshold:
            self._publish_notification("low battery")

    def _gps_cb(self, msg: GPSRAW) -> None:
        min_sats = int(self.get_parameter("gps_min_satellites").value)
        max_h_acc = int(self.get_parameter("gps_max_h_acc_mm").value)
        good = msg.satellites_visible >= min_sats and msg.h_acc <= max_h_acc
        if good and not self._gps_status:
            self._publish_notification("GPS good")
            self._gps_status = True
        elif not good and self._gps_status:
            self._publish_notification("GPS bad")
            self._gps_status = False

    def _channel_cb(self, rc: RCIn) -> None:
        if not rc.channels:
            return
        channel_idx = int(self.get_parameter("record_rc_channel").value)
        if channel_idx >= len(rc.channels):
            return
        stick = rc.channels[channel_idx]
        on_threshold = int(self.get_parameter("record_on_threshold").value)
        off_threshold = int(self.get_parameter("record_off_threshold").value)
        if not self._recording_state and stick > on_threshold:
            self.get_logger().info("RC request: start recording")
            self._start_recording()
            self._recording_state = True
        elif self._recording_state and stick < off_threshold:
            self.get_logger().info("RC request: stop recording")
            self._stop_recording()
            self._recording_state = False

    def _send_transform(
        self,
        stamp,
        frame_id: str,
        child_frame_id: str,
        translation,
        rotation,
    ) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = frame_id
        tf_msg.child_frame_id = child_frame_id
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = float(rotation[0])
        tf_msg.transform.rotation.y = float(rotation[1])
        tf_msg.transform.rotation.z = float(rotation[2])
        tf_msg.transform.rotation.w = float(rotation[3])
        self._tf_broadcaster.sendTransform(tf_msg)

    def _pose_cb(self, msg: PoseStamped) -> None:
        base_frame = str(self._p("mavros.base_frame"))
        pos = msg.pose.position
        rot = msg.pose.orientation
        stamp = msg.header.stamp

        self._send_transform(
            stamp,
            "map",
            base_frame,
            (pos.x, pos.y, pos.z),
            (rot.x, rot.y, rot.z, rot.w),
        )

        cam_pos = list(self._p("camera.pos"))
        cam_rot = list(self._p("camera.rot"))
        depth_frame = str(self._p("camera.depth_frame"))
        self._send_transform(
            stamp, base_frame, depth_frame, cam_pos, cam_rot
        )

        self._send_transform(
            stamp, "map", "odom", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        )

        depth_optical = str(self._p("camera.depth_optical_frame"))
        self._send_transform(
            stamp,
            depth_frame,
            depth_optical,
            (0.0, 0.0, 0.0),
            (-0.5, 0.5, -0.5, 0.5),
        )

    def _mavros_status_cb(self, msg: State) -> None:
        if msg.armed:
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
            KeyValue(key="mavros_init", value=str(self._mavros_init)),
            KeyValue(key="camera_init", value=str(self._camera_init)),
        ]

        msg = DiagnosticArray()
        msg.status.append(status)
        self._diagnostics_pub.publish(msg)

    def _main_loop_tick(self) -> None:
        if self._mavros_monitor_enabled:
            mavros_fps = float(self._p("mavros.expected_fps"))
            mavros_hz = self._mavros_hz.get_hz()
            if mavros_hz >= 0.8 * mavros_fps and not self._mavros_init:
                self._mavros_init = True
                self._publish_notification("low level ready")
            elif mavros_hz < 0.8 * mavros_fps and self._mavros_init:
                self._mavros_init = False
                failure_action = str(self._p("mavros.failure_action")).strip()
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
        if node._recording_state:
            node._stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
