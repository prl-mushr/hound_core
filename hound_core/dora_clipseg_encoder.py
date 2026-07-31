#!/usr/bin/env python3
"""Dora node: CLIPSeg vision encoder only.

Subscribes to ROS color (rclpy), runs the CLIP vision tower once per frame, and
forwards stacked hidden states via CUDA IPC (fallback: Arrow float16) plus RGB
bytes over dora shared memory. No FiLM decode / labels here.
"""

from __future__ import annotations

import time

import numpy as np
import pyarrow as pa
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import torch

from dora import Node

from perception_models.clipseg.shared import (
    ClipSegParts,
    load_seg_config,
    prompt_layout_from_config,
)
from hound_core.cuda_ipc import cuda_ipc_supported, pack_arrow_f16, pack_cuda_tensor


def main():
    cfg = load_seg_config()
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable; clipseg encoder needs a GPU")
    torch.set_float32_matmul_precision("high")

    # Register with dora before heavy init so peers do not time out.
    dora = Node()

    rclpy.init()
    ros = RosNode("dora_clipseg_encoder")
    color_topic = str(cfg.get("color_topic", "/camera/color/image_raw"))
    bridge = CvBridge()
    latest = {"msg": None}

    def _on_image(msg: Image):
        latest["msg"] = msg

    ros.create_subscription(Image, color_topic, _on_image, qos_profile_sensor_data)

    def log(msg: str):
        ros.get_logger().info(msg)

    layout = prompt_layout_from_config(cfg)
    trt_engine_path = str(cfg.get("clipseg_vision_engine", "") or "").strip()
    use_trt = bool(trt_engine_path)

    parts = ClipSegParts(
        str(cfg.get("model_name", "CIDAS/clipseg-rd16")),
        layout,
        input_res=int(cfg.get("input_res", 224)),
        compile_mode=("none" if use_trt else str(cfg.get("compile_mode", "reduce-overhead"))),
        device="cuda",
        logger=log,
    )

    enc = None
    trt_enc = None
    if use_trt:
        from perception_models.clipseg.trt import load_clipseg_vision_engine, trt_encode_hs

        log(f"Loading CLIPSeg vision TensorRT engine: {trt_engine_path}")
        trt_enc = load_clipseg_vision_engine(trt_engine_path)
        dummy = torch.zeros(1, 3, parts.res, parts.res, device="cuda")
        for _ in range(3):
            _ = trt_encode_hs(trt_enc, dummy)
        torch.cuda.synchronize()
        log("CLIPSeg TRT vision warmup complete.")
    else:
        enc = parts.build_encoder()
        enc = parts.warmup_encoder(enc)

    profile = bool(cfg.get("profile", True))
    profile_every = max(1, int(cfg.get("profile_every", 30)))
    prof = {}
    prof_n = 0

    use_cuda_ipc = cuda_ipc_supported("cuda")
    feature_mode = "torch_cuda_ipc" if use_cuda_ipc else "arrow_f16"
    if use_cuda_ipc:
        log("feature handoff: PyTorch CUDA IPC (GPU->GPU)")
    else:
        log(
            "feature handoff: Arrow float16 "
            "(CUDA IPC unsupported on this GPU — normal on Jetson/Tegra)"
        )

    enc_tag = "trt" if use_trt else "torch"
    log(
        f"dora clipseg_encoder up: {color_topic} -> features+rgb "
        f"(encode={enc_tag}, event-driven)"
    )
    # Driven by dora/timer so the runtime can deliver STOP; ROS frames gate work.
    for event in dora:
        if event is None:
            break
        if event.get("type") == "STOP":
            break
        if event.get("type") != "INPUT" or event.get("id") != "tick":
            continue

        # Drain available image callbacks so we always encode the newest frame.
        for _ in range(8):
            rclpy.spin_once(ros, timeout_sec=0.0)
        msg = latest["msg"]
        if msg is None:
            continue
        latest["msg"] = None  # process each frame at most once

        try:
            rgb = np.ascontiguousarray(
                bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            )
        except Exception as e:  # noqa: BLE001
            ros.get_logger().warn(f"cv_bridge conversion failed: {e}")
            continue

        h, w = rgb.shape[:2]
        if profile:
            torch.cuda.synchronize()
            t0 = time.perf_counter()

        if use_trt:
            from perception_models.clipseg.trt import trt_encode_hs

            pix = parts.preprocess_rgb(rgb)
            hs = trt_encode_hs(trt_enc, pix)
        else:
            hs = parts.encode(rgb, enc)  # [L,S,D]

        if profile:
            torch.cuda.synchronize()
            prof["clipseg_encode"] = (
                prof.get("clipseg_encode", 0.0) + (time.perf_counter() - t0) * 1000.0
            )

        n_layers, seq_len, hidden = (
            int(hs.shape[0]),
            int(hs.shape[1]),
            int(hs.shape[2]),
        )

        meta = {
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "frame_id": str(msg.header.frame_id or ""),
            "height": int(h),
            "width": int(w),
            "n_layers": n_layers,
            "seq_len": seq_len,
            "hidden": hidden,
            "encode_backend": enc_tag,
        }

        if profile:
            t_ipc = time.perf_counter()

        # RGB over Arrow SHM (CPU). Features prefer PyTorch CUDA IPC.
        dora.send_output(
            "rgb",
            pa.array(rgb.reshape(-1), type=pa.uint8()),
            meta,
        )

        if use_cuda_ipc:
            try:
                buf, ipc_meta, mode = pack_cuda_tensor(hs)
                feature_mode = mode
                out_meta = {**meta, **ipc_meta, "feature_mode": mode}
                dora.send_output("features", buf, out_meta)
            except Exception as e:  # noqa: BLE001
                ros.get_logger().warn(
                    f"CUDA IPC failed at runtime ({e}); switching to Arrow float16"
                )
                use_cuda_ipc = False
                feature_mode = "arrow_f16"

        if not use_cuda_ipc:
            feature_mode = "arrow_f16"
            dora.send_output(
                "features",
                pack_arrow_f16(hs),
                {**meta, "feature_mode": "arrow_f16"},
            )

        if profile:
            # CUDA sync only needed for encode; IPC pack is mostly CPU + handle.
            prof["feature_xfer"] = (
                prof.get("feature_xfer", 0.0) + (time.perf_counter() - t_ipc) * 1000.0
            )
            prof_n += 1
            if prof_n >= profile_every:
                parts_s = " ".join(
                    f"{k}={v / prof_n:5.1f}" for k, v in sorted(prof.items())
                )
                log(
                    f"[timing ms/frame n={prof_n} mode={feature_mode}] {parts_s}"
                )
                prof, prof_n = {}, 0

    ros.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
