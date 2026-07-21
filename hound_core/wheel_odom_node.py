"""Publish wheel-speed odometry to MAVROS from VESC telemetry.

ROS 2 port of the old hound_core vesc_to_odom.py: feeds linear wheel speed
to mavlink for improved velocity estimation on high-traction surfaces.
"""

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped


class WheelOdomNode(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odom_node")
        self.erpm_gain = float(self.declare_parameter("erpm_gain", 3166.6).value)
        self.output_topic = str(
            self.declare_parameter("output_topic", "/mavros/vision_pose/vis_odom").value
        )
        self.min_publish_interval_s = float(
            self.declare_parameter("min_publish_interval_s", 0.1).value
        )
        self._last_publish_time = 0.0

        self._odom_pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.create_subscription(
            VescStateStamped, "/sensors/core", self._vesc_cb, 10
        )
        self.get_logger().info(
            f"wheel_odom publishing to {self.output_topic} (erpm_gain={self.erpm_gain})"
        )

    def _vesc_cb(self, msg: VescStateStamped) -> None:
        now = time.time()
        if now - self._last_publish_time < self.min_publish_interval_s:
            return

        speed_mps = msg.state.speed / self.erpm_gain
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.twist.twist.linear.x = speed_mps
        odom.twist.covariance[0] = max(0.01 * abs(speed_mps), 0.1)
        self._odom_pub.publish(odom)
        self._last_publish_time = now


def main() -> None:
    rclpy.init()
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
