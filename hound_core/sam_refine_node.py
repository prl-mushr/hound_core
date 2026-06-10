"""Standalone SAM refinement node (split pipeline).

Subscribes to the color stream and the CLIPSeg box prompts, and asynchronously
refines them into crisp object masks with EfficientViT-SAM. Runs on its own
timer at sam_rate_hz against the latest available frame, so it never blocks the
fast CLIPSeg dynamic mask. Boxes carry the color frame's stamp, so we refine the
exact frame they were computed from (matched via a small ring buffer).

Publishes:
  <traversability_topic> : rgb8, G=traversable / R=non-traversable / B=none
  <people_mask_topic>    : mono8 refined people (dynamic) mask
  <overlay_topic>        : optional rgb8 debug overlay on the color frame
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int64MultiArray
from cv_bridge import CvBridge

import torch

from hound_core.sam_refiner import SamRefiner
from hound_core.utils import blend_overlay, stamp_key, to_image


class SamRefineNode(Node):
    def __init__(self):
        super().__init__("sam_refine_node")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("boxes_topic", "/segmentation/boxes")
        self.declare_parameter("traversability_topic", "/segmentation/refined_traversability")
        self.declare_parameter("people_mask_topic", "/segmentation/refined_people_mask")
        self.declare_parameter("overlay_topic", "/segmentation/refined_overlay")
        self.declare_parameter("publish_overlay", False)   # debug only
        self.declare_parameter("overlay_alpha", 0.5)
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("buffer_size", 30)
        # Box class ids (= CLIPSeg group indices) -> outputs.
        self.declare_parameter("positive_groups", [0])    # traversable  -> green
        self.declare_parameter("negative_groups", [1])    # non-traversable -> red
        self.declare_parameter("people_groups", [2])      # -> people mask
        self.declare_parameter("sam_variant", "l0")
        self.declare_parameter("sam_ckpt", "/root/colcon_ws/efficientvit_sam_l0.pt")
        self.declare_parameter("sam_compile", True)
        self.declare_parameter("sam_compile_mode", "reduce-overhead")

        g = self.get_parameter
        color_topic = str(g("color_topic").value)
        boxes_topic = str(g("boxes_topic").value)
        traversability_topic = str(g("traversability_topic").value)
        people_mask_topic = str(g("people_mask_topic").value)
        overlay_topic = str(g("overlay_topic").value)
        self.publish_overlay = bool(g("publish_overlay").value)
        self.overlay_alpha = float(g("overlay_alpha").value)
        self.rate_hz = float(g("rate_hz").value)
        self.buffer_size = int(g("buffer_size").value)
        self.positive_groups = [int(x) for x in g("positive_groups").value]
        self.negative_groups = [int(x) for x in g("negative_groups").value]
        self.people_groups = [int(x) for x in g("people_groups").value]
        sam_variant = str(g("sam_variant").value)
        sam_ckpt = str(g("sam_ckpt").value)
        sam_compile = bool(g("sam_compile").value)
        sam_compile_mode = str(g("sam_compile_mode").value)

        if not torch.cuda.is_available():
            self.get_logger().error("CUDA not available; sam_refine_node needs a GPU.")
            raise RuntimeError("CUDA unavailable")
        self.dev = "cuda"
        torch.set_float32_matmul_precision("high")

        self.get_logger().info(f"Loading EfficientViT-SAM ({sam_variant}) ...")
        self.sam = SamRefiner(
            self.dev, variant=sam_variant, ckpt=sam_ckpt,
            compile_mode=(sam_compile_mode if sam_compile else "none"),
            logger=self.get_logger(),
        )

        self.bridge = CvBridge()
        self._frames = {}          # stamp_key -> (header, rgb_np)
        self._frame_order = []     # stamp_keys, oldest first
        self._latest_boxes = None
        self._last_boxes_key = None

        self.trav_pub = self.create_publisher(Image, traversability_topic, qos_profile_sensor_data)
        self.people_pub = self.create_publisher(Image, people_mask_topic, qos_profile_sensor_data)
        self.overlay_pub = (
            self.create_publisher(Image, overlay_topic, qos_profile_sensor_data)
            if self.publish_overlay else None
        )
        self.color_sub = self.create_subscription(
            Image, color_topic, self._on_color, qos_profile_sensor_data
        )
        self.boxes_sub = self.create_subscription(
            Int64MultiArray, boxes_topic, self._on_boxes, 10
        )
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"sam_refine_node up: color={color_topic} boxes={boxes_topic} -> "
            f"traversability={traversability_topic}, people={people_mask_topic} "
            f"@ {self.rate_hz} Hz"
        )

    def _on_color(self, msg: Image):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return
        key = stamp_key(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self._frames[key] = (msg.header, np.ascontiguousarray(rgb))
        self._frame_order.append(key)
        while len(self._frame_order) > self.buffer_size:
            old = self._frame_order.pop(0)
            self._frames.pop(old, None)

    def _on_boxes(self, msg: Int64MultiArray):
        self._latest_boxes = list(msg.data)

    def _parse_boxes(self, data):
        """data layout: [sec, nanosec, K, then K*(class, x0, y0, x1, y1)]."""
        if len(data) < 3:
            return None, [], []
        key = stamp_key(data[0], data[1])
        k = int(data[2])
        classes, boxes = [], []
        off = 3
        for j in range(k):
            base = off + j * 5
            if base + 5 > len(data):
                break
            classes.append(int(data[base]))
            boxes.append((int(data[base + 1]), int(data[base + 2]),
                          int(data[base + 3]), int(data[base + 4])))
        return key, classes, boxes

    def _on_timer(self):
        boxes_msg = self._latest_boxes
        if boxes_msg is None:
            return
        key, classes, boxes = self._parse_boxes(boxes_msg)
        if key is None or key == self._last_boxes_key:
            return  # nothing new
        frame = self._frames.get(key)
        if frame is None:
            return  # color frame for these boxes not in buffer (yet/anymore)
        self._last_boxes_key = key
        header, rgb = frame

        # Box class == CLIPSeg group index. Enough channels to index every group
        # even if a class had no boxes this frame.
        max_idx = max([*classes, *self.positive_groups, *self.negative_groups,
                       *self.people_groups], default=0)
        refined = self.sam.refine(rgb, classes, boxes, max_idx + 1)   # [n,H,W] in {0,1}

        trav = self._union(refined, self.positive_groups)            # [H,W]
        nontrav = self._union(refined, self.negative_groups)
        people = self._union(refined, self.people_groups)

        # Traversability RGB: R=non-traversable, G=traversable, B=none (neither).
        none = (1.0 - torch.maximum(trav, nontrav)).clamp_(0.0, 1.0)
        rgb_t = torch.stack([nontrav, trav, none], dim=-1)           # [H,W,3]
        rgb_np = (rgb_t * 255.0).to(torch.uint8).cpu().numpy()
        self.trav_pub.publish(to_image(self.bridge, rgb_np, "rgb8", header))

        # Separate people (dynamic) mask.
        people_np = (people.cpu().numpy() >= 0.5).astype(np.uint8) * 255
        self.people_pub.publish(to_image(self.bridge, people_np, "mono8", header))

        # Debug overlay: tint only where traversable/non-traversable fired.
        if self.overlay_pub is not None:
            conf = torch.maximum(trav, nontrav).cpu().numpy()
            overlay = blend_overlay(rgb, rgb_np, conf, self.overlay_alpha)
            self.overlay_pub.publish(to_image(self.bridge, overlay, "rgb8", header))

    def _union(self, refined: torch.Tensor, idxs) -> torch.Tensor:
        sel = [i for i in idxs if 0 <= i < refined.shape[0]]
        if not sel:
            return refined.new_zeros(refined.shape[1:])
        return refined[sel].amax(dim=0)


def main():
    rclpy.init()
    node = SamRefineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
