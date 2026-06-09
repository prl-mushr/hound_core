"""CLIPSeg people/dynamic mask node.

Subscribes to a color image, runs text-prompted CLIPSeg segmentation on the GPU
(encode-the-vision-tower-once, batch the decoder across prompts, fp16 autocast,
optional torch.compile reduce-overhead), and publishes a single binary mono8
mask of the union of all prompts. Designed to be throttled well below the camera
rate (people don't move fast relative to mapping), feeding e.g. nvblox so dynamic
objects aren't fused into the static map.

All tunables are ROS parameters, set from hound_core's SSoT via robot.launch.py.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import torch
import torch.nn as nn
import torch.nn.functional as F
from transformers import CLIPSegProcessor, CLIPSegForImageSegmentation

# CLIP normalization constants (CLIPSeg uses the CLIP image stats).
CLIP_MEAN = (0.48145466, 0.4578275, 0.40821073)
CLIP_STD = (0.26862954, 0.26130258, 0.27577711)


class _FastClipSeg(nn.Module):
    """Run the CLIP vision encoder ONCE for the image, then batch only the
    (cheap) decoder across all prompts. Numerically identical to the stock
    forward but ~Nx cheaper on the vision tower for N prompts."""

    def __init__(self, model):
        super().__init__()
        self.model = model
        self.extract = list(model.extract_layers)

    def forward(self, pixel_values, cond):
        vo = self.model.clip.vision_model(
            pixel_values=pixel_values, output_hidden_states=True, return_dict=True
        )
        n = cond.shape[0]
        # NOTE the +1: HF extracts hidden_states[i+1] (index 0 is the embeddings).
        hs = tuple(
            vo.hidden_states[i + 1].expand(n, -1, -1).contiguous()
            for i in self.extract
        )
        return self.model.decoder(hs, cond).logits.sigmoid()


class ClipSegMaskNode(Node):
    def __init__(self):
        super().__init__("clipseg_mask_node")

        self.declare_parameter("model_name", "CIDAS/clipseg-rd16")
        self.declare_parameter("prompts", ["a person", "people", "pedestrian"])
        self.declare_parameter("input_res", 224)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("threshold", 0.3)
        self.declare_parameter("compile_mode", "reduce-overhead")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("mask_topic", "/segmentation/people_mask")

        g = self.get_parameter
        model_name = g("model_name").value
        self.prompts = [str(p) for p in g("prompts").value]
        self.res = int(g("input_res").value)
        self.rate_hz = float(g("rate_hz").value)
        self.threshold = float(g("threshold").value)
        compile_mode = str(g("compile_mode").value)
        color_topic = str(g("color_topic").value)
        mask_topic = str(g("mask_topic").value)

        if not torch.cuda.is_available():
            self.get_logger().error("CUDA not available; clipseg_mask_node needs a GPU.")
            raise RuntimeError("CUDA unavailable")

        self.dev = "cuda"
        torch.set_float32_matmul_precision("high")

        self.get_logger().info(f"Loading CLIPSeg model '{model_name}' ...")
        self.proc = CLIPSegProcessor.from_pretrained(model_name)
        model = CLIPSegForImageSegmentation.from_pretrained(model_name).to(self.dev).eval()

        # Precompute the prompt (conditional) embeddings ONCE.
        tok = self.proc.tokenizer(self.prompts, return_tensors="pt", padding=True).to(self.dev)
        with torch.inference_mode():
            self.cond = model.get_conditional_embeddings(
                batch_size=len(self.prompts),
                input_ids=tok.input_ids,
                attention_mask=tok.attention_mask,
            )

        self.fast = _FastClipSeg(model).eval()
        if compile_mode and compile_mode.lower() != "none":
            self.get_logger().info(f"torch.compile(mode='{compile_mode}') ... (first call compiles)")
            self.fast = torch.compile(self.fast, mode=compile_mode, dynamic=False)

        self.mean = torch.tensor(CLIP_MEAN, device=self.dev).view(1, 3, 1, 1)
        self.std = torch.tensor(CLIP_STD, device=self.dev).view(1, 3, 1, 1)

        self.bridge = CvBridge()
        self._latest = None

        self._warmup()

        self.pub = self.create_publisher(Image, mask_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            Image, color_topic, self._on_image, qos_profile_sensor_data
        )
        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.1
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"clipseg_mask_node up: {color_topic} -> {mask_topic} "
            f"@ {self.rate_hz} Hz, res={self.res}, thr={self.threshold}, prompts={self.prompts}"
        )

    def _warmup(self):
        dummy = torch.zeros(1, 3, self.res, self.res, device=self.dev)
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.float16):
            for _ in range(3):
                _ = self.fast(dummy, self.cond)
        torch.cuda.synchronize()
        self.get_logger().info("CLIPSeg warmup/compile complete.")

    @torch.inference_mode()
    def _infer(self, rgb_uint8: np.ndarray) -> torch.Tensor:
        """rgb_uint8: HxWx3 uint8 -> probability map [1,1,res,res] (float, cloned)."""
        t = torch.from_numpy(rgb_uint8).to(self.dev)
        t = t.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
        t = F.interpolate(t, size=(self.res, self.res), mode="bilinear", align_corners=False)
        t = (t - self.mean) / self.std
        with torch.autocast("cuda", dtype=torch.float16):
            masks = self.fast(t, self.cond)            # [n_prompts, res, res]
        merged = masks.amax(dim=0).unsqueeze(0).unsqueeze(0)   # [1,1,res,res], union of prompts
        # reduce-overhead reuses static output buffers -> clone before returning.
        return merged.float().clone()

    def _on_image(self, msg: Image):
        self._latest = msg

    def _on_timer(self):
        msg = self._latest
        if msg is None:
            return
        self._latest = None  # process each frame at most once

        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        h, w = rgb.shape[:2]
        prob = self._infer(np.ascontiguousarray(rgb))
        prob = F.interpolate(prob, size=(h, w), mode="bilinear", align_corners=False)
        mask = (prob[0, 0] >= self.threshold).to(torch.uint8).mul_(255)
        mask_np = mask.cpu().numpy()

        out = self.bridge.cv2_to_imgmsg(mask_np, encoding="mono8")
        out.header = msg.header  # keep the color image's stamp + frame_id for alignment
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ClipSegMaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
