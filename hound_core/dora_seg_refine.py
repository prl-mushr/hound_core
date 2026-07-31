#!/usr/bin/env python3
"""Dora node: FiLM + optional SAM refine with CUDA streams.

Consumes CLIPSeg features + RGB from clipseg_encoder. Always runs FiLM decode
and can publish coarse traversability from those probs (independent of NanoSAM).
When NanoSAM is enabled, FiLM decode and SAM image encode are enqueued on *two
CUDA streams* so they can overlap; then **two** mask decodes share that encode:

  * nav  — CLIPSeg trav/nontrav CCs → refined_traversability
  * manip — YOLO person/object boxes → refined_people_mask (when yolo_owns_people)

Dataflow: clipseg_encoder -> seg_refine (this node) -> ROS.
"""

from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
import torch

from dora import Node

from perception_models.clipseg.shared import (
    ClipSegParts,
    load_seg_config,
    prompt_layout_from_config,
)
from hound_core.cuda_ipc import unpack_arrow_f16, unpack_cuda_tensor
from hound_core.utils import to_image
from perception_models.sam_box_merge import (
    clipseg_nav_boxes,
    parse_yolo_detections_json,
    yolo_manip_boxes,
)
from perception_models.seg_ops import (
    blend_overlay,
    labels_to_boxes,
    labels_to_color,
    probs_to_labels,
)


def _feature_shape(meta: dict) -> tuple[int, ...]:
    return (
        int(meta["n_layers"]),
        int(meta["seq_len"]),
        int(meta["hidden"]),
    )


def _open_features(event, *, non_blocking: bool = False) -> torch.Tensor:
    meta = event["metadata"]
    mode = str(meta.get("feature_mode", "torch_cuda_ipc"))
    shape = _feature_shape(meta)

    if mode == "torch_cuda_ipc":
        return unpack_cuda_tensor(event["value"], meta).reshape(*shape).contiguous()
    if mode == "cuda_ipc":
        from dora.cuda import ipc_buffer_to_ipc_handle, open_ipc_handle

        handle = ipc_buffer_to_ipc_handle(event["value"], meta)
        with open_ipc_handle(handle, meta) as tensor:
            return tensor.reshape(*shape).contiguous().clone()
    return unpack_arrow_f16(event["value"], shape, non_blocking=non_blocking)


def _union(refined: torch.Tensor, idxs) -> torch.Tensor:
    sel = [i for i in idxs if 0 <= i < refined.shape[0]]
    if not sel:
        return refined.new_zeros(refined.shape[1:])
    return refined[sel].amax(dim=0)


def main():
    cfg = load_seg_config()
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable; seg_refine needs a GPU")
    torch.set_float32_matmul_precision("high")

    dora = Node()
    rclpy.init()
    ros = RosNode("dora_seg_refine")
    bridge = CvBridge()

    def log(msg: str):
        ros.get_logger().info(msg)

    layout = prompt_layout_from_config(cfg)
    parts = ClipSegParts(
        str(cfg.get("model_name", "CIDAS/clipseg-rd16")),
        layout,
        input_res=int(cfg.get("input_res", 224)),
        compile_mode=str(cfg.get("compile_mode", "reduce-overhead")),
        device="cuda",
        logger=log,
    )
    dec = parts.warmup_decoder(parts.build_decoder())

    aggregate = str(cfg.get("aggregate", "max")).lower()
    threshold = float(cfg.get("threshold", 0.3))
    viz_overlay = bool(cfg.get("viz_overlay", False))
    overlay_alpha = float(cfg.get("overlay_alpha", 0.5))
    labels_topic = str(cfg.get("labels_topic", "/segmentation/labels"))
    use_streams = bool(cfg.get("use_cuda_streams", True))

    labels_pub = ros.create_publisher(Image, labels_topic, qos_profile_sensor_data)
    overlay_pub = (
        ros.create_publisher(
            Image, str(cfg.get("overlay_topic", "/segmentation/overlay")), qos_profile_sensor_data
        )
        if viz_overlay
        else None
    )

    positive_groups = [int(x) for x in cfg.get("positive_groups", [0])]
    negative_groups = [int(x) for x in cfg.get("negative_groups", [1])]
    people_groups = [int(x) for x in cfg.get("people_groups", [2])]
    n_groups = max(
        [*positive_groups, *negative_groups, *people_groups, layout.n_groups - 1],
        default=0,
    ) + 1

    # When YOLO-World owns people, CLIPSeg skips people prompts/CCs; YOLO boxes
    # feed NanoSAM for refined_people_mask.
    yolo_owns_people = bool(cfg.get("yolo_owns_people", False))
    yolo_det_topic = str(
        cfg.get("yolo_detections_topic", "/yolo_world/detections")
    ).strip()
    yolo_object_group = cfg.get("yolo_object_group_idx", None)
    if yolo_object_group is not None:
        yolo_object_group = int(yolo_object_group)
    people_group_idx = int(people_groups[0]) if people_groups else max(0, n_groups - 1)
    latest_yolo: dict = {"dets": [], "stamp": 0.0}

    def _on_yolo_dets(msg: String):
        try:
            latest_yolo["dets"] = parse_yolo_detections_json(msg.data)
            latest_yolo["stamp"] = time.time()
        except Exception as exc:  # noqa: BLE001
            ros.get_logger().warning(f"yolo detections parse: {exc}")

    # Dual-decode NanoSAM: subscribe when YOLO owns people (launch sets this when
    # yolo_world.enabled). Topic default alone must not force a subscription.
    if yolo_owns_people:
        ros.create_subscription(
            String, yolo_det_topic, _on_yolo_dets, 10
        )
        log(
            f"NanoSAM dual-decode: YOLO dets on {yolo_det_topic} "
            f"(owns_people={yolo_owns_people}, people_idx={people_group_idx})"
        )

    # Coarse FiLM traversability — independent of NanoSAM.
    publish_coarse = bool(cfg.get("publish_coarse_traversability", True))
    coarse_trav_pub = coarse_people_pub = None
    if publish_coarse:
        coarse_trav_pub = ros.create_publisher(
            Image,
            str(
                cfg.get(
                    "coarse_traversability_topic",
                    "/segmentation/coarse_traversability",
                )
            ),
            qos_profile_sensor_data,
        )
        coarse_people_pub = ros.create_publisher(
            Image,
            str(
                cfg.get(
                    "coarse_people_mask_topic",
                    "/segmentation/coarse_people_mask",
                )
            ),
            qos_profile_sensor_data,
        )

    sam_enabled = bool(cfg.get("sam_node", False))
    sam_encoder = sam_decoder = None
    trav_pub = people_pub = sam_overlay_pub = None
    sam_min_area = float(cfg.get("sam_min_area", 0.002))
    sam_max_boxes_nav = int(cfg.get("sam_max_boxes", 6))
    sam_max_boxes_manip = int(
        cfg.get("sam_max_boxes_manip", cfg.get("sam_max_boxes", 6))
    )

    stream_film = stream_sam = None
    if sam_enabled:
        from perception_models.nanosam_trt import NanoSamDecoder, NanoSamEncoder

        log("Loading NanoSAM (TensorRT) ...")
        sam_encoder = NanoSamEncoder(
            str(cfg["sam_image_encoder"]),
            image_encoder_size=int(cfg.get("sam_image_size", 512)),
            device="cuda",
            logger=ros.get_logger(),
        )
        sam_decoder = NanoSamDecoder(
            str(cfg["sam_mask_decoder"]),
            orig_image_encoder_size=int(cfg.get("sam_image_size", 512)),
            device="cuda",
            logger=ros.get_logger(),
        )
        if use_streams:
            stream_film = torch.cuda.Stream()
            stream_sam = torch.cuda.Stream()
            log("CUDA streams: FiLM ∥ SAM-encode, then SAM-decode")

        trav_pub = ros.create_publisher(
            Image,
            str(cfg.get("sam_traversability_topic", "/segmentation/refined_traversability")),
            qos_profile_sensor_data,
        )
        people_pub = ros.create_publisher(
            Image,
            str(cfg.get("sam_people_mask_topic", "/segmentation/refined_people_mask")),
            qos_profile_sensor_data,
        )
        if bool(cfg.get("sam_overlay", False)):
            sam_overlay_pub = ros.create_publisher(
                Image,
                str(cfg.get("sam_overlay_topic", "/segmentation/refined_overlay")),
                qos_profile_sensor_data,
            )

    profile = bool(cfg.get("profile", True))
    profile_every = max(1, int(cfg.get("profile_every", 30)))
    prof: dict[str, float] = {}
    prof_n = 0
    qlen = max(2, int(cfg.get("pipeline_queue_size", 3)))
    rgb_buf: dict = {}
    feat_buf: dict = {}

    log(
        f"dora seg_refine up: features -> {labels_topic}"
        + (f" coarse_trav={publish_coarse}" if publish_coarse else "")
        + (" + NanoSAM" if sam_enabled else "")
        + (f" streams={use_streams}" if sam_encoder is not None else "")
        + (
            f" sam_boxes_nav={sam_max_boxes_nav} manip={sam_max_boxes_manip}"
            if sam_enabled
            else ""
        )
    )

    def _pack_trav_rgb(group_maps):
        trav = _union(group_maps, positive_groups)
        nontrav = _union(group_maps, negative_groups)
        none = (1.0 - torch.maximum(trav, nontrav)).clamp_(0.0, 1.0)
        rgb_t = torch.stack([nontrav, trav, none], dim=-1)
        rgb_np = (rgb_t * 255.0).to(torch.uint8).cpu().numpy()
        conf = torch.maximum(trav, nontrav)
        return rgb_np, conf

    def _publish_coarse(probs, rgb, hdr):
        if coarse_trav_pub is None:
            return
        rgb_np, _conf = _pack_trav_rgb(probs)
        coarse_trav_pub.publish(to_image(bridge, rgb_np, "rgb8", hdr))
        if not yolo_owns_people and coarse_people_pub is not None:
            people = _union(probs, people_groups)
            people_np = (people.cpu().numpy() >= 0.5).astype(np.uint8) * 255
            coarse_people_pub.publish(to_image(bridge, people_np, "mono8", hdr))

    def _publish_trav(refined_nav, rgb, hdr, *, people_mask=None):
        """Publish refined traversability (+ optional overlay). People separate."""
        rgb_np, conf = _pack_trav_rgb(refined_nav)
        if trav_pub is not None:
            trav_pub.publish(to_image(bridge, rgb_np, "rgb8", hdr))
        if sam_overlay_pub is not None:
            ov = blend_overlay(rgb, rgb_np, conf.cpu().numpy(), overlay_alpha)
            if people_mask is not None:
                pm = (people_mask.cpu().numpy() >= 0.5)
                ov = ov.copy()
                ov[pm, 2] = np.maximum(ov[pm, 2], 200)
            sam_overlay_pub.publish(to_image(bridge, ov, "rgb8", hdr))

    def _publish_people_mask(people_mask, hdr):
        if people_pub is None or people_mask is None:
            return
        people_np = (people_mask.cpu().numpy() >= 0.5).astype(np.uint8) * 255
        people_pub.publish(to_image(bridge, people_np, "mono8", hdr))

    def _sam_dual_decode(sam_feat, labels_np, h, w, *, profile_into: dict | None):
        """One encode's features → nav decode (+ optional manip decode).

        Returns (refined_nav, people_mask_hw).
        """
        cands = labels_to_boxes(
            labels_np, n_groups, sam_min_area, max(sam_max_boxes_nav * 2, sam_max_boxes_nav)
        )
        nav_classes, nav_boxes = clipseg_nav_boxes(
            cands,
            people_group_idx=people_group_idx,
            drop_people=yolo_owns_people,
            max_boxes=sam_max_boxes_nav,
        )
        n_nav = max(
            [*nav_classes, *positive_groups, *negative_groups, *people_groups],
            default=0,
        ) + 1
        refined_nav = sam_decoder.decode(
            sam_feat, nav_classes, nav_boxes, n_nav, (h, w)
        )
        if profile_into is not None:
            timing = sam_decoder.last_timing
            profile_into["sam_decoder_nav"] = profile_into.get(
                "sam_decoder_nav", 0.0
            ) + float(timing.get("decoder_ms", 0.0))
            profile_into["sam_n_boxes_nav"] = profile_into.get(
                "sam_n_boxes_nav", 0.0
            ) + float(timing.get("n_boxes", 0))
            if str(timing.get("decoder_mode", "")) == "batched":
                profile_into["sam_batched"] = profile_into.get("sam_batched", 0.0) + 1.0

        if yolo_owns_people:
            man_classes, man_boxes = yolo_manip_boxes(
                latest_yolo["dets"],
                people_group_idx=people_group_idx,
                object_group_idx=yolo_object_group,
                max_boxes=sam_max_boxes_manip,
            )
            n_man = max([*man_classes, people_group_idx], default=0) + 1
            if man_boxes:
                refined_man = sam_decoder.decode(
                    sam_feat, man_classes, man_boxes, n_man, (h, w)
                )
                if profile_into is not None:
                    timing = sam_decoder.last_timing
                    profile_into["sam_decoder_manip"] = profile_into.get(
                        "sam_decoder_manip", 0.0
                    ) + float(timing.get("decoder_ms", 0.0))
                    profile_into["sam_n_boxes_manip"] = profile_into.get(
                        "sam_n_boxes_manip", 0.0
                    ) + float(timing.get("n_boxes", 0))
            else:
                refined_man = torch.zeros(
                    (n_man, h, w), device=refined_nav.device, dtype=torch.float32
                )
                if profile_into is not None:
                    profile_into["sam_n_boxes_manip"] = profile_into.get(
                        "sam_n_boxes_manip", 0.0
                    )
            return refined_nav, _union(refined_man, people_groups)

        # Legacy: people CCs stay in nav decode when YOLO is off.
        return refined_nav, _union(refined_nav, people_groups)

    def _decode(hs, h, w, _feat_meta=None):
        return parts.decode_probs(
            hs, dec, height=h, width=w, aggregate=aggregate
        )

    def _process_frame(_stamp_key, meta_rgb, rgb, feat_event):
        nonlocal prof, prof_n
        h = int(meta_rgb["height"])
        w = int(meta_rgb["width"])
        hdr = Header()
        hdr.stamp = Time(
            sec=int(meta_rgb.get("stamp_sec", 0)),
            nanosec=int(meta_rgb.get("stamp_nanosec", 0)),
        )
        hdr.frame_id = str(meta_rgb.get("frame_id", ""))
        feat_meta = feat_event["metadata"]
        feat_mode = str(feat_meta.get("feature_mode", "?"))

        # --- NanoSAM dual-stream path: FiLM ∥ SAM-encode ---
        if sam_encoder is not None and stream_film is not None and stream_sam is not None:
            if profile:
                torch.cuda.synchronize()
                t_wall0 = time.perf_counter()
                ev_film0 = torch.cuda.Event(enable_timing=True)
                ev_film1 = torch.cuda.Event(enable_timing=True)
                ev_sam0 = torch.cuda.Event(enable_timing=True)
                ev_sam1 = torch.cuda.Event(enable_timing=True)

            with torch.cuda.stream(stream_film):
                if profile:
                    ev_film0.record(stream_film)
                hs = _open_features(feat_event, non_blocking=True)
                probs = _decode(hs, h, w, feat_meta)
                if profile:
                    ev_film1.record(stream_film)

            sam_input_cpu = sam_encoder.preprocess(rgb)

            with torch.cuda.stream(stream_sam):
                if profile:
                    ev_sam0.record(stream_sam)
                sam_feat = sam_encoder.encode_tensor(sam_input_cpu, synchronize=False)
                if profile:
                    ev_sam1.record(stream_sam)

            stream_film.synchronize()
            labels_np, conf_np = probs_to_labels(probs, threshold)
            labels_pub.publish(to_image(bridge, labels_np, "mono8", hdr))
            if overlay_pub is not None:
                color = labels_to_color(labels_np, layout.n_groups)
                overlay = blend_overlay(rgb, color, conf_np, overlay_alpha)
                overlay_pub.publish(to_image(bridge, overlay, "rgb8", hdr))
            _publish_coarse(probs, rgb, hdr)

            rclpy.spin_once(ros, timeout_sec=0.0)
            stream_sam.synchronize()

            frame_prof: dict[str, float] = {}
            refined_nav, people_mask = _sam_dual_decode(
                sam_feat, labels_np, h, w, profile_into=frame_prof if profile else None
            )
            _publish_trav(refined_nav, rgb, hdr, people_mask=people_mask)
            _publish_people_mask(people_mask, hdr)

            if profile:
                torch.cuda.synchronize()
                wall_ms = (time.perf_counter() - t_wall0) * 1000.0
                film_ms = float(ev_film0.elapsed_time(ev_film1))
                sam_enc_ms = float(ev_sam0.elapsed_time(ev_sam1))
                dec_ms = float(frame_prof.get("sam_decoder_nav", 0.0)) + float(
                    frame_prof.get("sam_decoder_manip", 0.0)
                )
                serial_fe = film_ms + sam_enc_ms
                overlap_ms = max(0.0, serial_fe - (wall_ms - dec_ms))
                prof["film_decode"] = prof.get("film_decode", 0.0) + film_ms
                prof["sam_encoder"] = prof.get("sam_encoder", 0.0) + sam_enc_ms
                prof["stream_wall"] = prof.get("stream_wall", 0.0) + wall_ms
                prof["stream_overlap"] = prof.get("stream_overlap", 0.0) + overlap_ms
                for k, v in frame_prof.items():
                    prof[k] = prof.get(k, 0.0) + float(v)

            rclpy.spin_once(ros, timeout_sec=0.0)
            if profile:
                prof_n += 1
                if prof_n >= profile_every:
                    parts_s = " ".join(
                        f"{k}={v / prof_n:5.1f}" for k, v in sorted(prof.items())
                    )
                    log(f"[timing ms/frame n={prof_n} mode={feat_mode}] {parts_s}")
                    prof, prof_n = {}, 0
            return

        # --- Serial path (SAM off, or use_cuda_streams=false) ---
        if profile:
            torch.cuda.synchronize()
            t0 = time.perf_counter()

        hs = _open_features(feat_event)
        if profile:
            torch.cuda.synchronize()
            t1 = time.perf_counter()
            prof["feature_open"] = prof.get("feature_open", 0.0) + (t1 - t0) * 1000.0

        probs = _decode(hs, h, w, feat_meta)
        labels_np, conf_np = probs_to_labels(probs, threshold)
        if profile:
            torch.cuda.synchronize()
            prof["film_decode"] = (
                prof.get("film_decode", 0.0) + (time.perf_counter() - t1) * 1000.0
            )

        labels_pub.publish(to_image(bridge, labels_np, "mono8", hdr))
        if overlay_pub is not None:
            color = labels_to_color(labels_np, layout.n_groups)
            overlay = blend_overlay(rgb, color, conf_np, overlay_alpha)
            overlay_pub.publish(to_image(bridge, overlay, "rgb8", hdr))
        _publish_coarse(probs, rgb, hdr)

        if sam_encoder is not None and sam_decoder is not None:
            if profile:
                torch.cuda.synchronize()
                t_e = time.perf_counter()
            sam_feat = sam_encoder.encode(rgb)
            if profile:
                torch.cuda.synchronize()
                prof["sam_encoder"] = (
                    prof.get("sam_encoder", 0.0) + (time.perf_counter() - t_e) * 1000.0
                )
            rclpy.spin_once(ros, timeout_sec=0.0)
            frame_prof: dict[str, float] = {}
            refined_nav, people_mask = _sam_dual_decode(
                sam_feat, labels_np, h, w, profile_into=frame_prof if profile else None
            )
            _publish_trav(refined_nav, rgb, hdr, people_mask=people_mask)
            _publish_people_mask(people_mask, hdr)
            if profile:
                for k, v in frame_prof.items():
                    prof[k] = prof.get(k, 0.0) + float(v)

        rclpy.spin_once(ros, timeout_sec=0.0)
        if profile:
            prof_n += 1
            if prof_n >= profile_every:
                parts_s = " ".join(
                    f"{k}={v / prof_n:5.1f}" for k, v in sorted(prof.items())
                )
                log(f"[timing ms/frame n={prof_n} mode={feat_mode}] {parts_s}")
                prof, prof_n = {}, 0

    for event in dora:
        if event is None or event.get("type") == "STOP":
            break
        if event.get("type") != "INPUT":
            continue

        eid = event["id"]
        meta = event["metadata"]
        stamp_key = (int(meta.get("stamp_sec", 0)), int(meta.get("stamp_nanosec", 0)))

        if eid == "rgb":
            hh = int(meta["height"])
            ww = int(meta["width"])
            flat = event["value"].to_numpy()
            rgb = np.asarray(flat, dtype=np.uint8).reshape(hh, ww, 3)
            rgb_buf[stamp_key] = (meta, np.ascontiguousarray(rgb))
            while len(rgb_buf) > qlen:
                rgb_buf.pop(next(iter(rgb_buf)))
        elif eid == "features":
            feat_buf[stamp_key] = event
            while len(feat_buf) > qlen:
                old = next(iter(feat_buf))
                try:
                    _ = _open_features(feat_buf.pop(old))
                    del _
                except Exception:  # noqa: BLE001
                    feat_buf.pop(old, None)
        else:
            continue

        # Drain YOLO detections while waiting for matched frames.
        if yolo_owns_people:
            for _ in range(4):
                rclpy.spin_once(ros, timeout_sec=0.0)

        if stamp_key in rgb_buf and stamp_key in feat_buf:
            meta_rgb, rgb = rgb_buf.pop(stamp_key)
            feat_event = feat_buf.pop(stamp_key)
            _process_frame(stamp_key, meta_rgb, rgb, feat_event)

    ros.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
