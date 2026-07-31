#!/usr/bin/env python3
"""Dora node: YOLO-World instance detector (CLIPSeg-shaped caching).

Cold path: stash class text embeddings via ``YoloWorldParts.set_prompts``.
Hot path: RGB → boxes (torch + FP16 today; TRT when ``yolo_world_engine`` set).

Publishes:
  * ``/yolo_world/detections`` — JSON ``std_msgs/String`` (boxes + persons)
  * dora output ``detections`` — same JSON bytes for downstream refine

Config: ``HOUND_YOLO_CONFIG`` JSON written by launch (see SSoT ``yolo_world:``).
"""

from __future__ import annotations

import json
import time

import numpy as np
import pyarrow as pa
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch

from dora import Node

from perception_models.yolo_world.shared import (
    YoloWorldParts,
    load_yolo_config,
    prompt_layout_from_config,
)


def main():
    cfg = load_yolo_config()
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable; yolo_world needs a GPU")

    dora = Node()

    rclpy.init()
    ros = RosNode("dora_yolo_world")
    bridge = CvBridge()
    color_topic = str(cfg.get("color_topic", "/camera/color/image_raw"))
    det_topic = str(cfg.get("detections_topic", "/yolo_world/detections"))
    overlay_topic = str(cfg.get("overlay_topic", "/yolo_world/overlay"))
    viz_overlay = bool(cfg.get("viz_overlay", False))
    latest = {"msg": None}

    def _on_image(msg: Image):
        latest["msg"] = msg

    ros.create_subscription(Image, color_topic, _on_image, qos_profile_sensor_data)
    pub_det = ros.create_publisher(String, det_topic, 10)
    pub_overlay = (
        ros.create_publisher(Image, overlay_topic, qos_profile_sensor_data)
        if viz_overlay
        else None
    )

    def log(msg: str):
        ros.get_logger().info(msg)

    layout = prompt_layout_from_config(cfg)
    engine = str(cfg.get("yolo_world_engine", "") or "").strip()
    if engine:
        try:
            from perception_models.yolo_world.trt import load_yolo_world_engine

            load_yolo_world_engine(engine)
        except Exception as exc:  # noqa: BLE001
            log(f"YOLO-World TRT unavailable ({exc}); using torch FP16")

    parts = YoloWorldParts(
        model_name=str(cfg.get("model", "yolov8s-world.pt")),
        prompts=layout,
        conf=float(cfg.get("conf", 0.25)),
        device=str(cfg.get("device", "cuda:0")),
        imgsz=int(cfg.get("imgsz", 640)),
        half=bool(cfg.get("half", True)),
        logger=log,
    )
    log("loading YOLO-World (stash class embeddings) ...")
    parts.warmup()
    log(
        f"dora_yolo_world up: {color_topic} -> {det_topic} "
        f"classes={parts.prompts.classes} imgsz={parts.imgsz} half={parts.half}"
    )

    profile = bool(cfg.get("profile", True))
    profile_every = max(1, int(cfg.get("profile_every", 30)))
    prof_ms = 0.0
    prof_n = 0

    for event in dora:
        if event is None:
            break
        if event.get("type") == "STOP":
            break
        if event.get("type") != "INPUT" or event.get("id") != "tick":
            continue

        for _ in range(8):
            rclpy.spin_once(ros, timeout_sec=0.0)
        msg = latest["msg"]
        if msg is None:
            continue
        latest["msg"] = None

        try:
            rgb = np.ascontiguousarray(
                bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            )
        except Exception as exc:  # noqa: BLE001
            ros.get_logger().warning(f"cv_bridge: {exc}")
            continue

        if profile:
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        boxes = parts.detect_boxes(rgb)
        if profile:
            torch.cuda.synchronize()
        dt = (time.perf_counter() - t0) * 1000.0
        prof_ms += dt
        prof_n += 1
        if profile and prof_n >= profile_every:
            log(f"[timing ms/frame n={prof_n}] yolo_detect={prof_ms / prof_n:6.1f}")
            prof_ms = 0.0
            prof_n = 0

        persons = [b for b in boxes if b.get("name") == "person"]
        payload = {
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "frame_id": str(msg.header.frame_id or ""),
            "height": int(rgb.shape[0]),
            "width": int(rgb.shape[1]),
            "classes": list(parts.prompts.classes),
            "detections": [
                {
                    "xyxy": list(b["xyxy"]),
                    "conf": b["conf"],
                    "cls": b["cls"],
                    "name": b["name"],
                }
                for b in boxes
            ],
            "persons": [
                {
                    "xyxy": list(b["xyxy"]),
                    "conf": b["conf"],
                    "cls": b["cls"],
                    "name": b["name"],
                }
                for b in persons
            ],
            "infer_ms": float(dt),
        }
        out = String()
        out.data = json.dumps(payload)
        pub_det.publish(out)

        meta = {
            "stamp_sec": payload["stamp_sec"],
            "stamp_nanosec": payload["stamp_nanosec"],
            "frame_id": payload["frame_id"],
            "height": payload["height"],
            "width": payload["width"],
            "n_det": len(boxes),
            "n_person": len(persons),
        }
        dora.send_output(
            "detections",
            pa.array(np.frombuffer(json.dumps(payload).encode("utf-8"), dtype=np.uint8)),
            meta,
        )

        if pub_overlay is not None and boxes:
            import cv2

            vis = rgb[:, :, ::-1].copy()
            for b in boxes:
                x0, y0, x1, y1 = b["xyxy"]
                color = (0, 255, 0) if b["name"] == "person" else (255, 160, 0)
                cv2.rectangle(vis, (x0, y0), (x1, y1), color, 2)
                cv2.putText(
                    vis,
                    f"{b['name']} {b['conf']:.2f}",
                    (x0, max(0, y0 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                    cv2.LINE_AA,
                )
            pub_overlay.publish(bridge.cv2_to_imgmsg(vis, encoding="bgr8"))

    ros.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
