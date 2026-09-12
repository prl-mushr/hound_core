"""Rosbag start/stop node for HOUND.

Subscribes to a Bool trigger (true=start, false=stop) and manages
`ros2 bag record` with sequential bag names (hound_0, hound_1, ...) in
bagdir / bagdir_nav, plus the same split behavior as the legacy HAL
monitor recording path.
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import List, Optional

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class BagRecorderNode(Node):
    TUNES = {
        "record start": "ML O3 L8 CD",
        "record stop": "ML O3 L8 DC",
    }

    def __init__(self) -> None:
        super().__init__("bag_recorder")

        self.declare_parameter("bagdir", "/root/colcon_ws/bags/")
        self.declare_parameter("bagdir_nav", "/root/colcon_ws/bags_nav/")
        self.declare_parameter("ssot_path", "")
        self.declare_parameter(
            "record_topics_file",
            "/root/colcon_ws/src/hound_core/config/rosbag_record_topics.txt",
        )
        self.declare_parameter("record_all_topics", True)
        self.declare_parameter("record_nav_only", False)
        self.declare_parameter(
            "record_nav_topics_file",
            "/root/colcon_ws/src/hound_core/config/rosbag_record_nav_topics.txt",
        )
        self.declare_parameter("record_split_duration_min", 5)
        self.declare_parameter("record_topic", "/hal/record")
        self.declare_parameter("recording_status_topic", "/hal/recording")
        self.declare_parameter(
            "notification_topic", "/hound_fcu_control/play_tune"
        )

        self._recording_state = False
        self._rosbag_proc: Optional[subprocess.Popen] = None
        self._record_cmd: List[str] = []
        self._record_nav_only = bool(self.get_parameter("record_nav_only").value)
        self._record_all = (
            bool(self.get_parameter("record_all_topics").value)
            and not self._record_nav_only
        )
        self._load_record_topics()

        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._recording_pub = self.create_publisher(
            Bool, str(self.get_parameter("recording_status_topic").value), latch_qos
        )
        self._notification_pub = self.create_publisher(
            String, str(self.get_parameter("notification_topic").value), 10
        )
        self._publish_recording_status(False)

        self.create_subscription(
            Bool,
            str(self.get_parameter("record_topic").value),
            self._record_cb,
            10,
        )
        self.create_timer(1.0, self._watch_proc)
        if self._record_all:
            mode = "ALL topics (-a)"
        elif self._record_nav_only:
            mode = f"nav I/O only ({len(self._record_topics)} topics)"
        else:
            mode = f"{len(self._record_topics)} topics from file"
        self.get_logger().info(
            f"Bag recorder online (trigger={self.get_parameter('record_topic').value}, "
            f"{mode})"
        )

    def _load_record_topics(self) -> None:
        self._record_topics: List[str] = []
        if self._record_all:
            return
        if self._record_nav_only:
            topics_file = Path(
                str(self.get_parameter("record_nav_topics_file").value)
            )
        else:
            topics_file = Path(str(self.get_parameter("record_topics_file").value))
        if not topics_file.is_file():
            self.get_logger().warning(
                f"Record topics file not found: {topics_file}"
            )
            return
        self._record_topics = [
            line.strip()
            for line in topics_file.read_text().splitlines()
            if line.strip() and not line.strip().startswith("#")
        ]

    def _resolve_ssot_path(self) -> Path:
        explicit = str(self.get_parameter("ssot_path").value).strip()
        if explicit:
            return Path(explicit)
        env = os.environ.get("HOUND_SSOT", "").strip()
        if env:
            return Path(env)
        for candidate in (
            Path("/root/colcon_ws/src/hound_core/config/SSoT.yaml"),
            Path("/home/hound/colcon_ws/src/hound_core/config/SSoT.yaml"),
        ):
            if candidate.is_file():
                return candidate
        return Path("")

    def _nav_bag_planner_subdir(self) -> str:
        """Read nav.Planner_config.experiment_info_default.bidirectional from SSoT."""
        ssot_path = self._resolve_ssot_path()
        if not ssot_path.is_file():
            self.get_logger().warning(
                f"SSoT not found ({ssot_path}); nav bags → unidirectional/"
            )
            return "unidirectional"
        try:
            ssot = yaml.safe_load(ssot_path.read_text(encoding="utf-8")) or {}
            exp = ((ssot.get("nav") or {}).get("Planner_config") or {}).get(
                "experiment_info_default"
            ) or {}
            bidirectional = bool(exp.get("bidirectional", False))
            subdir = "bidirectional" if bidirectional else "unidirectional"
            self.get_logger().info(
                f"SSoT {ssot_path}: bidirectional={bidirectional} → {subdir}/"
            )
            return subdir
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to read SSoT bidirectional ({ssot_path}): {exc}; "
                "using unidirectional/"
            )
            return "unidirectional"

    def _snapshot_ssot(self, bag_path: Path) -> None:
        src = self._resolve_ssot_path()
        if not src.is_file():
            self.get_logger().warning(
                f"SSoT snapshot skipped: not found ({src or 'empty path'})"
            )
            return
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if bag_path.is_dir():
                break
            time.sleep(0.05)
        if not bag_path.is_dir():
            self.get_logger().warning(
                f"SSoT snapshot skipped: bag dir not created ({bag_path})"
            )
            return
        dest = bag_path / "SSoT.yaml"
        shutil.copy2(src, dest)
        self.get_logger().info(f"SSoT snapshot: {src} → {dest}")

    def _publish_notification(self, message: str) -> None:
        tune = self.TUNES.get(message)
        if tune is None:
            return
        if self._notification_pub.get_subscription_count() == 0:
            return
        msg = String()
        msg.data = tune
        self._notification_pub.publish(msg)

    def _publish_recording_status(self, recording: bool) -> None:
        msg = Bool()
        msg.data = recording
        self._recording_pub.publish(msg)

    def _watch_proc(self) -> None:
        if not self._recording_state or self._rosbag_proc is None:
            return
        code = self._rosbag_proc.poll()
        if code is None:
            return
        self.get_logger().error(
            f"ros2 bag record exited early (code={code}); "
            "clearing recording state so RC can start again. "
            "If the dest dir already existed, that is why."
        )
        self._rosbag_proc = None
        self._recording_state = False
        self._publish_recording_status(False)

    @staticmethod
    def _next_bag_path(bagdir: Path) -> Path:
        """Return bagdir/hound_N using one past the highest existing hound_<int>."""
        highest = -1
        for entry in bagdir.iterdir():
            suffix = entry.name[6:] if entry.name.startswith("hound_") else ""
            if suffix.isdigit():
                highest = max(highest, int(suffix))
        n = highest + 1
        while True:
            candidate = bagdir / f"hound_{n}"
            if not candidate.exists():
                return candidate
            n += 1

    def _start_recording(self) -> bool:
        if self._record_nav_only:
            bagdir = Path(str(self.get_parameter("bagdir_nav").value))
            bagdir = bagdir / self._nav_bag_planner_subdir()
        else:
            bagdir = Path(str(self.get_parameter("bagdir").value))
        bagdir.mkdir(parents=True, exist_ok=True)
        split_min = int(self.get_parameter("record_split_duration_min").value)
        output = self._next_bag_path(bagdir)
        self._record_cmd = [
            "ros2",
            "bag",
            "record",
            "-o",
            str(output),
            "--max-bag-duration",
            str(split_min * 60),
        ]
        if self._record_all:
            self._record_cmd.append("-a")
        else:
            if not self._record_topics:
                self.get_logger().error(
                    "record_all_topics=false and topic list empty; not starting"
                )
                return False
            self._record_cmd.extend(self._record_topics)
        self.get_logger().info(f"Starting bag record: {' '.join(self._record_cmd)}")
        self._rosbag_proc = subprocess.Popen(self._record_cmd)
        self._snapshot_ssot(output)
        self._publish_notification("record start")
        return True

    def _stop_recording(self) -> None:
        if self._rosbag_proc is not None and self._rosbag_proc.poll() is None:
            self._rosbag_proc.send_signal(signal.SIGINT)
            try:
                self._rosbag_proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self._rosbag_proc.kill()
        self._rosbag_proc = None
        self._publish_notification("record stop")

    def _record_cb(self, msg: Bool) -> None:
        if msg.data:
            if self._recording_state:
                return
            self.get_logger().info("Record request: start")
            if not self._start_recording():
                return
            self._recording_state = True
            self._publish_recording_status(True)
        else:
            if not self._recording_state:
                return
            self.get_logger().info("Record request: stop")
            self._stop_recording()
            self._recording_state = False
            self._publish_recording_status(False)

    def shutdown(self) -> None:
        if self._recording_state:
            self._stop_recording()
            self._recording_state = False
            self._publish_recording_status(False)


def main() -> None:
    rclpy.init()
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
