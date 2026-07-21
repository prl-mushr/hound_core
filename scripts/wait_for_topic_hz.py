#!/usr/bin/env python3
"""Block until a ROS topic sustains a minimum rate for a stable window."""

from __future__ import annotations

import argparse
import importlib
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _import_msg_type(type_name: str):
    parts = type_name.split("/")
    if len(parts) == 3:
        pkg, _, msg = parts
    elif len(parts) == 2:
        pkg, msg = parts
    else:
        raise ValueError(
            f"invalid message type {type_name!r}; "
            "expected 'pkg/msg/Name' or 'pkg/Name'"
        )
    module = importlib.import_module(f"{pkg}.msg")
    return getattr(module, msg)


class TopicHzWaiter(Node):
    def __init__(self, topic: str, msg_type, min_hz: float, stable_s: float, timeout_s: float):
        super().__init__("topic_hz_waiter")
        self._min_hz = min_hz
        self._stable_s = stable_s
        self._deadline = time.monotonic() + timeout_s
        self._times: list[float] = []
        self._stable_since: float | None = None

        self.create_subscription(msg_type, topic, self._on_msg, qos_profile_sensor_data)
        self.get_logger().info(
            f"waiting for {topic}: >= {min_hz:.1f} Hz sustained {stable_s:.1f}s "
            f"(timeout {timeout_s:.0f}s)"
        )

    def _on_msg(self, _msg) -> None:
        now = time.monotonic()
        self._times.append(now)
        cutoff = now - max(self._stable_s, 1.0)
        self._times = [t for t in self._times if t >= cutoff]
        if len(self._times) < 2:
            self._stable_since = None
            return
        hz = (len(self._times) - 1) / (self._times[-1] - self._times[0])
        if hz >= self._min_hz:
            if self._stable_since is None:
                self._stable_since = now
        else:
            self._stable_since = None

    def ready(self) -> bool:
        if self._stable_since is None:
            return False
        return (time.monotonic() - self._stable_since) >= self._stable_s

    def timed_out(self) -> bool:
        return time.monotonic() >= self._deadline


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic")
    parser.add_argument("--msg-type", default="sensor_msgs/msg/Image")
    parser.add_argument("--min-hz", type=float, default=24.0)
    parser.add_argument("--stable-s", type=float, default=5.0)
    parser.add_argument("--timeout-s", type=float, default=120.0)
    args = parser.parse_args()

    msg_type = _import_msg_type(args.msg_type)

    rclpy.init()
    node = TopicHzWaiter(args.topic, msg_type, args.min_hz, args.stable_s, args.timeout_s)
    exit_code = 0
    try:
        while rclpy.ok() and not node.ready():
            if node.timed_out():
                node.get_logger().error("timed out waiting for stable topic rate")
                exit_code = 1
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        if exit_code == 0:
            node.get_logger().info(f"{args.topic} stable at >= {args.min_hz:.1f} Hz")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
