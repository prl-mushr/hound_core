"""CLIPSeg semantic segmentation node.

Runs text-prompted CLIPSeg on a color stream, aggregates grouped prompts into
navigation categories (e.g. traversable / non_traversable / people), and
publishes a single label map (pixel = category id), box prompts for
sam_refine_node, and an optional debug overlay. All tunables are ROS params,
set from hound_core's SSoT via robot.launch.py.

Label map (mono8): 0 = none/unknown, otherwise group_index + 1 (so the SSoT
prompt-group order defines the ids, e.g. 1=traversable, 2=non_traversable,
3=people). Refinement lives in the separate sam_refine_node.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int64MultiArray
from cv_bridge import CvBridge

import torch
import torch.nn as nn
import torch.nn.functional as F
from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation

from hound_core.utils import (
    blend_overlay, extract_boxes, labels_to_color, probs_to_labels, to_image,
)

# CLIP normalization constants (CLIPSeg uses the CLIP image stats).
CLIP_MEAN = (0.48145466, 0.4578275, 0.40821073)
CLIP_STD = (0.26862954, 0.26130258, 0.27577711)


class _FastClipSeg(nn.Module):
    """Run the CLIP vision encoder once, then batch the cheap decoder across all
    prompts (numerically identical to the stock forward, ~Nx cheaper encoder)."""

    def __init__(self, model):
        super().__init__()
        self.model = model
        self.extract = list(model.extract_layers)

    def forward(self, pixel_values, cond):
        vo = self.model.clip.vision_model(
            pixel_values=pixel_values, output_hidden_states=True, return_dict=True
        )
        n = cond.shape[0]
        # +1: HF extracts hidden_states[i+1] (index 0 is the embeddings).
        hs = tuple(
            vo.hidden_states[i + 1].expand(n, -1, -1).contiguous()
            for i in self.extract
        )
        return self.model.decoder(hs, cond).logits.sigmoid()    # [n_prompts, res, res]


class ClipSegMaskNode(Node):
    def __init__(self):
        super().__init__("clipseg_mask_node")

        self.declare_parameter("model_name", "CIDAS/clipseg-rd16")
        # Grouped prompts: the launch flattens the SSoT dict into the flat prompt
        # list + group names + a group id per prompt. CLIPSeg runs every prompt,
        # then maps are aggregated within each group into one map per category.
        self.declare_parameter("prompts", ["a person"])
        self.declare_parameter("group_names", [""])
        self.declare_parameter("group_ids", [0])
        # Suppress prompts (e.g. sky): subtracted from every group so they get no label.
        self.declare_parameter("suppress_prompts", [""])
        # Aggregation within a group: "max" | "noisy_or" | "mean" | "sum".
        self.declare_parameter("aggregate", "max")
        self.declare_parameter("input_res", 224)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("threshold", 0.3)   # below this a pixel is label 0 (none)
        self.declare_parameter("compile_mode", "reduce-overhead")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("labels_topic", "/segmentation/labels")
        # Debug overlay (category colors blended on the image).
        self.declare_parameter("viz_overlay", True)
        self.declare_parameter("overlay_topic", "/segmentation/overlay")
        self.declare_parameter("overlay_alpha", 0.5)
        # Blob -> box extraction for the boxes published to sam_refine_node.
        self.declare_parameter("sam_min_area", 0.002)
        self.declare_parameter("sam_max_boxes", 6)
        self.declare_parameter("profile", False)
        self.declare_parameter("profile_every", 30)
        self.declare_parameter("publish_boxes", False)
        self.declare_parameter("boxes_topic", "/segmentation/boxes")

        g = self.get_parameter
        model_name = g("model_name").value
        self.subprompts = [str(p) for p in g("prompts").value]
        group_names = [str(x) for x in g("group_names").value if str(x) != ""]
        group_ids = [int(x) for x in g("group_ids").value]
        if not group_names or len(group_ids) != len(self.subprompts):
            group_names = list(self.subprompts)
            group_ids = list(range(len(self.subprompts)))
        self.group_names = group_names
        self.n_groups = len(group_names)
        self.group_index = [
            [i for i, gid in enumerate(group_ids) if gid == gi]
            for gi in range(self.n_groups)
        ]
        self.suppress_prompts = [str(x) for x in g("suppress_prompts").value if str(x) != ""]
        self.full_prompts = self.subprompts + self.suppress_prompts
        self.suppress_idx = list(range(len(self.subprompts), len(self.full_prompts)))
        self.aggregate = str(g("aggregate").value).lower()
        self.res = int(g("input_res").value)
        self.rate_hz = float(g("rate_hz").value)
        self.threshold = float(g("threshold").value)
        compile_mode = str(g("compile_mode").value)
        color_topic = str(g("color_topic").value)
        labels_topic = str(g("labels_topic").value)
        self.viz_overlay = bool(g("viz_overlay").value)
        overlay_topic = str(g("overlay_topic").value)
        self.overlay_alpha = float(g("overlay_alpha").value)
        self.sam_min_area = float(g("sam_min_area").value)
        self.sam_max_boxes = int(g("sam_max_boxes").value)
        self.profile = bool(g("profile").value)
        self.profile_every = int(g("profile_every").value)
        self.publish_boxes = bool(g("publish_boxes").value)
        boxes_topic = str(g("boxes_topic").value)
        self._prof = {}
        self._prof_n = 0

        if not torch.cuda.is_available():
            self.get_logger().error("CUDA not available; clipseg_mask_node needs a GPU.")
            raise RuntimeError("CUDA unavailable")
        self.dev = "cuda"
        torch.set_float32_matmul_precision("high")

        self.get_logger().info(f"Loading CLIPSeg model '{model_name}' ...")
        self.proc = CLIPSegProcessor.from_pretrained(model_name)
        model = CLIPSegForImageSegmentation.from_pretrained(model_name).to(self.dev).eval()

        # Precompute the (conditional) embeddings for every prompt ONCE.
        tok = self.proc.tokenizer(self.full_prompts, return_tensors="pt", padding=True).to(self.dev)
        with torch.inference_mode():
            self.cond = model.get_conditional_embeddings(
                batch_size=len(self.full_prompts),
                input_ids=tok.input_ids,
                attention_mask=tok.attention_mask,
            )
        labels = ", ".join(f"{i + 1}={n}" for i, n in enumerate(self.group_names))
        self.get_logger().info(
            f"groups: [{labels}] (aggregate='{self.aggregate}', "
            f"{len(self.subprompts)} sub-prompts, suppress={self.suppress_prompts})"
        )

        self.fast = _FastClipSeg(model).eval()
        self._fast_eager = self.fast
        if compile_mode and compile_mode.lower() != "none":
            self.get_logger().info(f"torch.compile(mode='{compile_mode}') ...")
            self.fast = torch.compile(self.fast, mode=compile_mode, dynamic=False)

        self.mean = torch.tensor(CLIP_MEAN, device=self.dev).view(1, 3, 1, 1)
        self.std = torch.tensor(CLIP_STD, device=self.dev).view(1, 3, 1, 1)

        self.bridge = CvBridge()
        self._latest = None
        self._warmup()

        self.labels_pub = self.create_publisher(Image, labels_topic, qos_profile_sensor_data)
        self.overlay_pub = (
            self.create_publisher(Image, overlay_topic, qos_profile_sensor_data)
            if self.viz_overlay else None
        )
        self.box_pub = (
            self.create_publisher(Int64MultiArray, boxes_topic, 10)
            if self.publish_boxes else None
        )

        self.sub = self.create_subscription(Image, color_topic, self._on_image, qos_profile_sensor_data)
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"clipseg_mask_node up: {color_topic} -> {labels_topic} @ {self.rate_hz} Hz "
            f"(res={self.res}, boxes={self.publish_boxes})"
        )

    # ------------------------------------------------------------------ inference
    def _warmup(self):
        dummy = torch.zeros(1, 3, self.res, self.res, device=self.dev)

        def run():
            with torch.inference_mode(), torch.autocast("cuda", dtype=torch.float16):
                for _ in range(3):
                    _ = self.fast(dummy, self.cond)
            torch.cuda.synchronize()

        try:
            run()
        except Exception as e:  # noqa: BLE001 -- torch.compile can fail
            if self.fast is not self._fast_eager:
                self.get_logger().warn(f"compile/warmup failed ({e}); eager fallback")
                self.fast = self._fast_eager
                run()
            else:
                raise
        self.get_logger().info("CLIPSeg warmup/compile complete.")

    @torch.inference_mode()
    def _infer(self, rgb_uint8: np.ndarray) -> torch.Tensor:
        """HxWx3 uint8 -> per-prompt prob maps [P,res,res] (float [0,1], cloned)."""
        t = torch.from_numpy(rgb_uint8).to(self.dev).permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
        t = F.interpolate(t, size=(self.res, self.res), mode="bilinear", align_corners=False)
        t = (t - self.mean) / self.std
        with torch.autocast("cuda", dtype=torch.float16):
            masks = self.fast(t, self.cond)
        return masks.float().clone()   # reduce-overhead reuses buffers -> clone

    def _aggregate(self, sub: torch.Tensor) -> torch.Tensor:
        """Per-sub-prompt maps [P,H,W] -> per-group maps [n_groups,H,W]."""
        out = sub.new_zeros((self.n_groups, sub.shape[1], sub.shape[2]))
        for gi, idx in enumerate(self.group_index):
            if not idx:
                continue
            s = sub[idx]
            if self.aggregate == "mean":
                out[gi] = s.mean(dim=0)
            elif self.aggregate == "sum":
                out[gi] = s.sum(dim=0).clamp_(0.0, 1.0)
            elif self.aggregate == "noisy_or":
                out[gi] = 1.0 - (1.0 - s).prod(dim=0)
            else:  # "max" (probabilistic OR)
                out[gi] = s.amax(dim=0)
        return out

    def _coarse_boxes(self, probs: torch.Tensor):
        n, h, w = probs.shape
        binary = (probs >= self.threshold).to(torch.uint8).cpu().numpy()
        return extract_boxes(binary, self.sam_min_area * h * w, self.sam_max_boxes)

    # ------------------------------------------------------------------- profiling
    def _tic(self, sync=True):
        if not self.profile:
            return 0.0
        if sync:
            torch.cuda.synchronize()
        return time.perf_counter()

    def _toc(self, key, t0, sync=True):
        if not self.profile:
            return
        if sync:
            torch.cuda.synchronize()
        self._prof[key] = self._prof.get(key, 0.0) + (time.perf_counter() - t0) * 1000.0

    def _prof_step(self):
        if not self.profile:
            return
        self._prof_n += 1
        if self._prof_n >= self.profile_every:
            parts = " ".join(f"{k}={v / self._prof_n:5.1f}" for k, v in sorted(self._prof.items()))
            self.get_logger().info(f"[profile ms/frame over {self._prof_n}] {parts}")
            self._prof, self._prof_n = {}, 0

    def _publish_boxes(self, cands, header):
        data = [int(header.stamp.sec), int(header.stamp.nanosec), len(cands)]
        for cls, _area, (x0, y0, x1, y1) in cands:
            data += [int(cls), int(x0), int(y0), int(x1), int(y1)]
        m = Int64MultiArray()
        m.data = data
        self.box_pub.publish(m)

    # ---------------------------------------------------------------------- loop
    def _on_image(self, msg: Image):
        self._latest = msg

    def _on_timer(self):
        msg = self._latest
        if msg is None:
            return
        self._latest = None  # process each frame at most once

        try:
            rgb = np.ascontiguousarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return
        h, w = rgb.shape[:2]
        hdr = msg.header

        tcs = self._tic()
        sub = self._infer(rgb)                                   # [P,res,res]
        groups = self._aggregate(sub)                            # [n_groups,res,res]
        if self.suppress_idx:                                    # subtract sky etc.
            sky = sub[self.suppress_idx].amax(dim=0)
            groups = (groups - sky.unsqueeze(0)).clamp_(0.0, 1.0)
        probs = F.interpolate(
            groups.unsqueeze(0), size=(h, w), mode="bilinear", align_corners=False
        )[0].clamp_(0.0, 1.0)                                    # [n_groups,h,w]
        self._toc("clipseg", tcs)

        if self.box_pub is not None:
            self._publish_boxes(self._coarse_boxes(probs), hdr)

        tpub = self._tic(sync=False)
        labels_np, conf_np = probs_to_labels(probs, self.threshold)
        self.labels_pub.publish(to_image(self.bridge, labels_np, "mono8", hdr))

        if self.overlay_pub is not None:
            color = labels_to_color(labels_np, self.n_groups)
            overlay = blend_overlay(rgb, color, conf_np, self.overlay_alpha)
            self.overlay_pub.publish(to_image(self.bridge, overlay, "rgb8", hdr))

        self._toc("publish", tpub, sync=False)
        self._prof_step()


def main():
    rclpy.init()
    node = ClipSegMaskNode()
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
