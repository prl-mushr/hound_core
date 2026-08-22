"""Rosbag start/stop node for HOUND.

Subscribes to a Bool trigger (true=start, false=stop) and manages
`ros2 bag record` with the same bagdir / split / rename behavior as the
legacy HAL monitor recording path.
"""

from __future__ import annotations

import signal
import subprocess
from pathlib import Path
from typing import List, Optional

import rclpy
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
        self.declare_parameter(
            "record_topics_file",
            "/root/colcon_ws/src/hound_core/config/rosbag_record_topics.txt",
        )
        self.declare_parameter("record_all_topics", True)
        self.declare_parameter("record_split_duration_min", 5)
        self.declare_parameter("record_topic", "/hal/record")
        self.declare_parameter("recording_status_topic", "/hal/recording")
        self.declare_parameter(
            "notification_topic", "/hound_fcu_control/play_tune"
        )

        self._recording_state = False
        self._rosbag_proc: Optional[subprocess.Popen] = None
        self._record_cmd: List[str] = []
        self._record_all = bool(self.get_parameter("record_all_topics").value)
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
        mode = (
            "ALL topics (-a)"
            if self._record_all
            else f"{len(self._record_topics)} topics from file"
        )
        self.get_logger().info(
            f"Bag recorder online (trigger={self.get_parameter('record_topic').value}, "
            f"{mode})"
        )

    def _load_record_topics(self) -> None:
        self._record_topics: List[str] = []
        if self._record_all:
            return
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

    def _start_recording(self) -> None:
        bagdir = Path(str(self.get_parameter("bagdir").value))
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
        ]
        if self._record_all:
            self._record_cmd.append("-a")
        else:
            if not self._record_topics:
                self.get_logger().error(
                    "record_all_topics=false and topic list empty; not starting"
                )
                return
            self._record_cmd.extend(self._record_topics)
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

        bagdir = Path(str(self.get_parameter("bagdir").value))
        if not bagdir.is_dir():
            return
        candidates = sorted(
            (p for p in bagdir.iterdir() if p.name.startswith("temp")),
            key=lambda p: p.stat().st_mtime,
        )
        for i, path in enumerate(candidates):
            path.rename(bagdir / f"hound_{i}")

    def _record_cb(self, msg: Bool) -> None:
        if msg.data:
            if self._recording_state:
                return
            self.get_logger().info("Record request: start")
            self._start_recording()
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
