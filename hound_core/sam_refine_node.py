"""Standalone SAM refinement node (split pipeline).

Subscribes to the color stream and the CLIPSeg label map, extracts coarse box
prompts locally, and asynchronously refines them into crisp object masks with
NanoSAM or EfficientViT-SAM. Runs on its own timer at sam_rate_hz against the
latest available frame, so it never blocks the fast CLIPSeg dynamic mask.
Labels carry the color frame's stamp, so we refine the exact frame they were
computed from (matched via a small ring buffer).

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
from cv_bridge import CvBridge

import torch

from hound_core.utils import blend_overlay, labels_to_boxes, stamp_key, to_image


class SamRefineNode(Node):
    def __init__(self):
        super().__init__("sam_refine_node")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("labels_topic", "/segmentation/labels")
        self.declare_parameter("traversability_topic", "/segmentation/refined_traversability")
        self.declare_parameter("people_mask_topic", "/segmentation/refined_people_mask")
        self.declare_parameter("overlay_topic", "/segmentation/refined_overlay")
        self.declare_parameter("publish_overlay", False)   # debug only
        self.declare_parameter("overlay_alpha", 0.5)
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("buffer_size", 30)
        self.declare_parameter("sam_min_area", 0.002)
        self.declare_parameter("sam_max_boxes", 6)
        # Box class ids (= CLIPSeg group indices) -> outputs.
        self.declare_parameter("positive_groups", [0])    # traversable  -> green
        self.declare_parameter("negative_groups", [1])    # non-traversable -> red
        self.declare_parameter("people_groups", [2])      # -> people mask
        self.declare_parameter("sam_backend", "nanosam")  # "nanosam" | "efficientvit"
        self.declare_parameter(
            "sam_image_encoder",
            "/root/colcon_ws/src/nanosam/data/resnet18_image_encoder.engine",
        )
        self.declare_parameter(
            "sam_mask_decoder",
            "/root/colcon_ws/src/nanosam/data/mobile_sam_mask_decoder.engine",
        )
        self.declare_parameter("sam_variant", "l0")
        self.declare_parameter("sam_ckpt", "/root/colcon_ws/efficientvit_sam_l0.pt")
        self.declare_parameter("sam_compile", True)
        self.declare_parameter("sam_compile_mode", "reduce-overhead")

        g = self.get_parameter
        color_topic = str(g("color_topic").value)
        labels_topic = str(g("labels_topic").value)
        traversability_topic = str(g("traversability_topic").value)
        people_mask_topic = str(g("people_mask_topic").value)
        overlay_topic = str(g("overlay_topic").value)
        self.publish_overlay = bool(g("publish_overlay").value)
        self.overlay_alpha = float(g("overlay_alpha").value)
        self.rate_hz = float(g("rate_hz").value)
        self.buffer_size = int(g("buffer_size").value)
        self.sam_min_area = float(g("sam_min_area").value)
        self.sam_max_boxes = int(g("sam_max_boxes").value)
        self.positive_groups = [int(x) for x in g("positive_groups").value]
        self.negative_groups = [int(x) for x in g("negative_groups").value]
        self.people_groups = [int(x) for x in g("people_groups").value]
        sam_backend = str(g("sam_backend").value).lower()
        sam_image_encoder = str(g("sam_image_encoder").value)
        sam_mask_decoder = str(g("sam_mask_decoder").value)
        sam_variant = str(g("sam_variant").value)
        sam_ckpt = str(g("sam_ckpt").value)
        sam_compile = bool(g("sam_compile").value)
        sam_compile_mode = str(g("sam_compile_mode").value)
        self.n_groups = max(
            [*self.positive_groups, *self.negative_groups, *self.people_groups], default=0
        ) + 1

        if not torch.cuda.is_available():
            self.get_logger().error("CUDA not available; sam_refine_node needs a GPU.")
            raise RuntimeError("CUDA unavailable")
        self.dev = "cuda"
        torch.set_float32_matmul_precision("high")

        self.get_logger().info(f"Loading SAM backend '{sam_backend}' ...")
        self.sam = self._build_sam_refiner(
            sam_backend,
            image_encoder=sam_image_encoder,
            mask_decoder=sam_mask_decoder,
            variant=sam_variant,
            ckpt=sam_ckpt,
            compile_mode=(sam_compile_mode if sam_compile else "none"),
        )

        self.bridge = CvBridge()
        self._frames = {}          # stamp_key -> (header, rgb_np)
        self._frame_order = []     # stamp_keys, oldest first
        self._latest_labels = None
        self._last_labels_key = None

        self.trav_pub = self.create_publisher(Image, traversability_topic, qos_profile_sensor_data)
        self.people_pub = self.create_publisher(Image, people_mask_topic, qos_profile_sensor_data)
        self.overlay_pub = (
            self.create_publisher(Image, overlay_topic, qos_profile_sensor_data)
            if self.publish_overlay else None
        )
        self.color_sub = self.create_subscription(
            Image, color_topic, self._on_color, qos_profile_sensor_data
        )
        self.labels_sub = self.create_subscription(
            Image, labels_topic, self._on_labels, qos_profile_sensor_data
        )
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"sam_refine_node up: backend={sam_backend} color={color_topic} "
            f"labels={labels_topic} -> traversability={traversability_topic}, "
            f"people={people_mask_topic} @ {self.rate_hz} Hz"
        )

    def _build_sam_refiner(
        self,
        backend: str,
        *,
        image_encoder: str,
        mask_decoder: str,
        variant: str,
        ckpt: str,
        compile_mode: str,
    ):
        if backend == "nanosam":
            from hound_core.nano_sam_refiner import NanoSamRefiner

            return NanoSamRefiner(
                image_encoder=image_encoder,
                mask_decoder=mask_decoder,
                device=self.dev,
                logger=self.get_logger(),
            )
        if backend in ("efficientvit", "efficientvit-sam"):
            from hound_core.sam_refiner import SamRefiner

            return SamRefiner(
                self.dev,
                variant=variant,
                ckpt=ckpt,
                compile_mode=compile_mode,
                logger=self.get_logger(),
            )
        raise ValueError(
            f"Unknown sam_backend '{backend}' (expected 'nanosam' or 'efficientvit')"
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

    def _on_labels(self, msg: Image):
        try:
            labels = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return
        self._latest_labels = (
            stamp_key(msg.header.stamp.sec, msg.header.stamp.nanosec),
            msg.header,
            np.ascontiguousarray(labels),
        )

    def _on_timer(self):
        labels_msg = self._latest_labels
        if labels_msg is None:
            return
        key, header, labels_np = labels_msg
        if key == self._last_labels_key:
            return  # nothing new
        frame = self._frames.get(key)
        if frame is None:
            return  # color frame for these labels not in buffer (yet/anymore)
        self._last_labels_key = key
        _, rgb = frame

        cands = labels_to_boxes(
            labels_np, self.n_groups, self.sam_min_area, self.sam_max_boxes
        )
        classes = [cls for cls, _area, _box in cands]
        boxes = [box for _cls, _area, box in cands]

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
