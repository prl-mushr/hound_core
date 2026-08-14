"""Aggregate RViz 2D Goal Pose clicks into a mission Path; save / replay.

Usage (inside mushr_jazzy)::

  # Click "2D Goal Pose" in RViz; each click appends a waypoint.
  # Ctrl+C saves to mission_file (if set) and exits.
  ros2 run hound_core mission_pose_builder --ros-args \\
    -p mission_file:=/root/colcon_ws/missions/track1.yaml

  # Replay a saved mission (latched Path for nav):
  ros2 run hound_core mission_pose_builder --ros-args \\
    -p mission_file:=/root/colcon_ws/missions/track1.yaml \\
    -p load_on_start:=true \\
    -p record:=false

Clear / undo while recording::
  ros2 topic pub --once /mission_pose_builder/clear std_msgs/msg/Empty {}
  ros2 topic pub --once /mission_pose_builder/undo std_msgs/msg/Empty {}
"""

from __future__ import annotations

import math
import signal
import sys
from pathlib import Path
from typing import List, Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


class MissionPoseBuilderNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_pose_builder")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/hound_fcu_control/mission/path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("load_on_start", False)
        self.declare_parameter("record", True)
        self.declare_parameter("min_separation_m", 0.5)
        self.declare_parameter("save_on_shutdown", True)
        self.declare_parameter("republish_period_s", 1.0)

        self._goal_topic = str(self.get_parameter("goal_topic").value)
        self._path_topic = str(self.get_parameter("path_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._mission_file = str(self.get_parameter("mission_file").value).strip()
        self._record = bool(self.get_parameter("record").value)
        self._min_sep = float(self.get_parameter("min_separation_m").value)
        self._save_on_shutdown = bool(self.get_parameter("save_on_shutdown").value)

        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._path_pub = self.create_publisher(Path, self._path_topic, latch_qos)
        self._poses: List[PoseStamped] = []
        self._dirty = False

        if bool(self.get_parameter("load_on_start").value) and self._mission_file:
            n = self._load_mission(Path(self._mission_file))
            if n > 0:
                self._publish()
                self.get_logger().info(
                    f"loaded {n} waypoints from {self._mission_file} → {self._path_topic}"
                )

        if self._record:
            self.create_subscription(
                PoseStamped, self._goal_topic, self._on_goal, 10
            )
            self.create_subscription(Empty, "~/clear", self._on_clear, 10)
            self.create_subscription(Empty, "~/undo", self._on_undo, 10)
            self.get_logger().info(
                f"recording: click RViz 2D Goal Pose on {self._goal_topic}; "
                f"Ctrl+C to save"
                + (f" → {self._mission_file}" if self._mission_file else "")
            )
        else:
            self.get_logger().info(
                f"replay-only: latched Path on {self._path_topic} ({len(self._poses)} pts)"
            )

        period = float(self.get_parameter("republish_period_s").value)
        if period > 0.0:
            self.create_timer(period, self._maybe_republish)

    def _maybe_republish(self) -> None:
        if self._poses:
            self._publish()

    def _on_goal(self, msg: PoseStamped) -> None:
        if not self._record:
            return
        if self._min_sep > 0.0 and self._poses:
            last = self._poses[-1].pose.position
            dx = msg.pose.position.x - last.x
            dy = msg.pose.position.y - last.y
            if (dx * dx + dy * dy) ** 0.5 < self._min_sep:
                self.get_logger().warn(
                    f"ignored goal ({self._min_sep:.2f} m from last waypoint)"
                )
                return

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = msg.header.frame_id or self._frame_id
        ps.pose = msg.pose
        self._poses.append(ps)
        self._dirty = True
        self._publish()
        self.get_logger().info(
            f"waypoint {len(self._poses)}: "
            f"({ps.pose.position.x:.2f}, {ps.pose.position.y:.2f}) "
            f"frame={ps.header.frame_id}"
        )

    def _on_clear(self, _msg: Empty) -> None:
        self._poses.clear()
        self._dirty = True
        self._publish()
        self.get_logger().info("mission cleared")

    def _on_undo(self, _msg: Empty) -> None:
        if not self._poses:
            return
        self._poses.pop()
        self._dirty = True
        self._publish()
        self.get_logger().info(f"undo → {len(self._poses)} waypoints")

    def _publish(self) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._frame_id
        if self._poses:
            # Prefer explicit frame from first click if set.
            path.header.frame_id = self._poses[0].header.frame_id or self._frame_id
        for i, ps in enumerate(self._poses):
            out = PoseStamped()
            out.header = path.header
            out.pose = ps.pose
            path.poses.append(out)
        self._path_pub.publish(path)

    def _load_mission(self, path: Path) -> int:
        if not path.is_file():
            self.get_logger().error(f"mission file not found: {path}")
            return 0
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        frame = str(data.get("frame_id", self._frame_id))
        self._frame_id = frame
        wps = data.get("waypoints") or []
        self._poses.clear()
        for wp in wps:
            ps = PoseStamped()
            ps.header.frame_id = frame
            if isinstance(wp, dict):
                ps.pose.position.x = float(wp.get("x", 0.0))
                ps.pose.position.y = float(wp.get("y", 0.0))
                ps.pose.position.z = float(wp.get("z", 0.0))
                yaw = float(wp.get("yaw", 0.0))
                ps.pose.orientation.z = math.sin(yaw * 0.5)
                ps.pose.orientation.w = math.cos(yaw * 0.5)
            elif isinstance(wp, (list, tuple)) and len(wp) >= 2:
                ps.pose.position.x = float(wp[0])
                ps.pose.position.y = float(wp[1])
                ps.pose.position.z = float(wp[2]) if len(wp) > 2 else 0.0
                ps.pose.orientation.w = 1.0
            else:
                continue
            self._poses.append(ps)
        self._dirty = False
        return len(self._poses)

    def save_mission(self, path: Optional[Path] = None) -> bool:
        out = path or (Path(self._mission_file) if self._mission_file else None)
        if out is None:
            self.get_logger().warn("no mission_file set — not saving")
            return False
        if not self._poses:
            self.get_logger().warn("empty mission — not saving")
            return False
        out.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "frame_id": self._frame_id,
            "waypoints": [
                {
                    "x": float(p.pose.position.x),
                    "y": float(p.pose.position.y),
                    "z": float(p.pose.position.z),
                }
                for p in self._poses
            ],
        }
        out.write_text(
            yaml.safe_dump(payload, default_flow_style=False, sort_keys=False),
            encoding="utf-8",
        )
        self._dirty = False
        self.get_logger().info(f"saved {len(self._poses)} waypoints → {out}")
        return True

    def shutdown_save(self) -> None:
        if self._save_on_shutdown and self._record and self._dirty and self._poses:
            self.save_mission()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionPoseBuilderNode()

    shutting_down = {"done": False}

    def _handle_sig(_signum, _frame) -> None:
        if shutting_down["done"]:
            return
        shutting_down["done"] = True
        node.get_logger().info("Ctrl+C — finalizing mission")
        node.shutdown_save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not shutting_down["done"]:
            shutting_down["done"] = True
            try:
                node.shutdown_save()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
