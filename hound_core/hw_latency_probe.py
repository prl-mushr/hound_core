#!/usr/bin/env python3
"""Measure end-to-end ROS transport latency for hardware components.

Latency per message:  now = node.get_clock().now()
                      age  = now - msg.header.stamp

Used to validate the split-component approach (stock RealSense driver +
MAVROS IMU/mag/baro/GPS) before wiring them into modular FCU / VSLAM nodes.

ArduPilot / MAVROS topics are subscribed when present; they stay quiet until
the FCU is connected.
"""

from __future__ import annotations

import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Image, Imu, MagneticField, NavSatFix


@dataclass
class TopicStats:
    name: str
    ages_s: Deque[float] = field(default_factory=lambda: deque(maxlen=2000))
    arrival_mono: Deque[float] = field(default_factory=lambda: deque(maxlen=2000))
    last_stamp_ns: Optional[int] = None
    count: int = 0
    drop_or_reorder: int = 0  # stamp went backwards or equal

    def on_msg(self, stamp_ns: int, age_s: float, mono: float) -> None:
        self.count += 1
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            self.drop_or_reorder += 1
        self.last_stamp_ns = stamp_ns
        self.ages_s.append(age_s)
        self.arrival_mono.append(mono)

    def hz(self, window_s: float = 2.0) -> float:
        if len(self.arrival_mono) < 2:
            return 0.0
        cutoff = self.arrival_mono[-1] - window_s
        times = [t for t in self.arrival_mono if t >= cutoff]
        if len(times) < 2:
            return 0.0
        return (len(times) - 1) / (times[-1] - times[0])

    def summary(self) -> str:
        if not self.ages_s:
            return f"{self.name:32s}  n=0  (no messages)"
        ages = list(self.ages_s)
        ages_ms = [a * 1000.0 for a in ages]
        p50 = statistics.median(ages_ms)
        mean = statistics.fmean(ages_ms)
        p95 = sorted(ages_ms)[max(0, int(0.95 * (len(ages_ms) - 1)))]
        mx = max(ages_ms)
        mn = min(ages_ms)
        return (
            f"{self.name:32s}  "
            f"hz={self.hz():6.1f}  "
            f"lat_ms mean={mean:6.1f} p50={p50:6.1f} p95={p95:6.1f} "
            f"min={mn:5.1f} max={mx:6.1f}  "
            f"n={self.count} reorder={self.drop_or_reorder}"
        )


class HwLatencyProbe(Node):
    def __init__(self) -> None:
        super().__init__("hw_latency_probe")

        self.declare_parameter("report_period_s", 2.0)
        self.declare_parameter("camera_name", "camera")
        self.declare_parameter("enable_realsense", True)
        self.declare_parameter("enable_mavros", True)
        # Optional overrides (empty = defaults under /camera and /mavros)
        self.declare_parameter("infra1_topic", "")
        self.declare_parameter("infra2_topic", "")
        self.declare_parameter("color_topic", "")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("imu_topic", "/mavros/imu/data")
        self.declare_parameter("imu_raw_topic", "/mavros/imu/data_raw")
        self.declare_parameter("mag_topic", "/mavros/imu/mag")
        self.declare_parameter("baro_topic", "/mavros/imu/static_pressure")
        self.declare_parameter("gps_topic", "/mavros/global_position/raw/fix")

        cam = str(self.get_parameter("camera_name").value)
        report_s = float(self.get_parameter("report_period_s").value)

        self._stats: dict[str, TopicStats] = {}

        if bool(self.get_parameter("enable_realsense").value):
            self._sub_image(
                self._topic("infra1_topic", f"/{cam}/infra1/image_rect_raw"),
                "rs/infra1",
            )
            self._sub_image(
                self._topic("infra2_topic", f"/{cam}/infra2/image_rect_raw"),
                "rs/infra2",
            )
            self._sub_image(
                self._topic("color_topic", f"/{cam}/color/image_raw"),
                "rs/color",
            )
            self._sub_image(
                self._topic("depth_topic", f"/{cam}/depth/image_rect_raw"),
                "rs/depth",
            )

        if bool(self.get_parameter("enable_mavros").value):
            self._sub_imu(self._topic("imu_topic", "/mavros/imu/data"), "mav/imu")
            self._sub_imu(
                self._topic("imu_raw_topic", "/mavros/imu/data_raw"), "mav/imu_raw"
            )
            topic = self._topic("mag_topic", "/mavros/imu/mag")
            self._stats["mav/mag"] = TopicStats("mav/mag")
            self.create_subscription(
                MagneticField, topic, lambda m: self._on_headered(m, "mav/mag"), qos_profile_sensor_data
            )
            topic = self._topic("baro_topic", "/mavros/imu/static_pressure")
            self._stats["mav/baro"] = TopicStats("mav/baro")
            self.create_subscription(
                FluidPressure, topic, lambda m: self._on_headered(m, "mav/baro"), qos_profile_sensor_data
            )
            topic = self._topic("gps_topic", "/mavros/global_position/raw/fix")
            self._stats["mav/gps"] = TopicStats("mav/gps")
            self.create_subscription(
                NavSatFix, topic, lambda m: self._on_headered(m, "mav/gps"), qos_profile_sensor_data
            )

        self.create_timer(report_s, self._report)
        self.get_logger().info(
            f"latency probe up: report every {report_s:.1f}s; "
            f"age = clock.now() - header.stamp"
        )

    def _topic(self, param: str, default: str) -> str:
        val = str(self.get_parameter(param).value).strip()
        return val if val else default

    def _sub_image(self, topic: str, key: str) -> None:
        self._stats[key] = TopicStats(key)
        self.create_subscription(
            Image, topic, lambda m, k=key: self._on_headered(m, k), qos_profile_sensor_data
        )
        self.get_logger().info(f"sub Image {topic} -> {key}")

    def _sub_imu(self, topic: str, key: str) -> None:
        self._stats[key] = TopicStats(key)
        self.create_subscription(
            Imu, topic, lambda m, k=key: self._on_headered(m, k), qos_profile_sensor_data
        )
        self.get_logger().info(f"sub Imu {topic} -> {key}")

    def _on_headered(self, msg, key: str) -> None:
        stamp = msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        now = self.get_clock().now()
        age_s = (now.nanoseconds - stamp_ns) * 1e-9
        self._stats[key].on_msg(stamp_ns, age_s, time.monotonic())

    def _report(self) -> None:
        lines = ["--- hw_latency_probe ---"]
        for key in sorted(self._stats.keys()):
            lines.append(self._stats[key].summary())
        self.get_logger().info("\n".join(lines))


def main() -> None:
    rclpy.init()
    node = HwLatencyProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
