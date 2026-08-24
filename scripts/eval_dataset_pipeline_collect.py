#!/usr/bin/env python3
"""Collect cuVSLAM↔GT RMSE + LocalMap frames during dataset pipeline smoke.

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 scripts/eval_dataset_pipeline_collect.py \\
    --out /tmp/hound_dataset_eval --duration 75
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

import numpy as np


def _umeyama(A: np.ndarray, B: np.ndarray, with_scale: bool = True):
    """Sim(3)/SE(3) alignment: B ≈ s R A + t. Returns s, R, t."""
    assert A.shape == B.shape and A.shape[1] == 3
    n = A.shape[0]
    mu_A = A.mean(axis=0)
    mu_B = B.mean(axis=0)
    AA = A - mu_A
    BB = B - mu_B
    cov = (BB.T @ AA) / n
    U, S, Vt = np.linalg.svd(cov)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = U @ Vt
    var_A = (AA ** 2).sum() / n
    s = float(np.trace(np.diag(S)) / var_A) if with_scale and var_A > 1e-12 else 1.0
    t = mu_B - s * R @ mu_A
    return s, R, t


def _quat_angle_deg(q_est: np.ndarray, q_gt: np.ndarray) -> float:
    """Relative rotation angle (deg) between xyzw quaternions."""
    q_est = q_est / (np.linalg.norm(q_est) + 1e-12)
    q_gt = q_gt / (np.linalg.norm(q_gt) + 1e-12)
    d = abs(float(np.dot(q_est, q_gt)))
    d = min(1.0, d)
    return float(2.0 * np.degrees(np.arccos(d)))


def _parse_profile_log(log_path: Path) -> dict:
    """Pull mean ms timings from smoke launch log."""
    text = log_path.read_text(errors="replace") if log_path.is_file() else ""
    out: dict = {
        "stereo_cuvslam": [],
        "clipseg_encoder": [],
        "seg_refine": [],
        "mapping_extract": [],
    }

    # cuVSLAM timing (3.4s): track mean/max=10.27/10.27 ms  iter mean/max=...
    for m in re.finditer(
        r"cuVSLAM timing[^:]*:\s*track mean/max=([\d.]+)/([\d.]+) ms\s+"
        r"iter mean/max=([\d.]+)/([\d.]+) ms\s+track=([\d.]+) Hz\s+"
        r"ir_frames=([\d.]+) Hz",
        text,
    ):
        out["stereo_cuvslam"].append(
            {
                "track_mean_ms": float(m.group(1)),
                "track_max_ms": float(m.group(2)),
                "iter_mean_ms": float(m.group(3)),
                "iter_max_ms": float(m.group(4)),
                "track_hz": float(m.group(5)),
                "ir_hz": float(m.group(6)),
            }
        )

    # [timing ms/frame n=30 mode=trt cams=1] clipseg_encode=12.3 feature_xfer=1.2
    # Values may have spaces after '=' (e.g. clipseg_encode= 35.9).
    for m in re.finditer(
        r"\[timing ms/frame n=(\d+)[^\]]*\]\s*(.+)",
        text,
    ):
        n = int(m.group(1))
        parts = {}
        for km in re.finditer(
            r"([A-Za-z_][\w]*)\s*=\s*([-+]?[\d.]+)", m.group(2)
        ):
            parts[km.group(1)] = float(km.group(2))
        if "clipseg_encode" in parts:
            parts["n"] = n
            out["clipseg_encoder"].append(parts)
        elif "film_decode" in parts or "sam_encoder" in parts:
            parts["n"] = n
            out["seg_refine"].append(parts)

    # mapping_node: extract 9.3ms (snapshot 40.3 bev 4.5 inpaint 3.1 slope 1.3 ...)
    for m in re.finditer(
        r"extract\s+([\d.]+)ms\s*\(\s*snapshot\s+([\d.]+)\s+"
        r"bev\s+([\d.]+)\s+inpaint\s+([\d.]+)\s+slope\s+([\d.]+)",
        text,
    ):
        out["mapping_extract"].append(
            {
                "total_ms": float(m.group(1)),
                "snapshot_ms": float(m.group(2)),
                "bev_ms": float(m.group(3)),
                "inpaint_ms": float(m.group(4)),
                "slope_ms": float(m.group(5)),
            }
        )
    # Legacy: elev extract total=12.3ms snapshot=.. bev=.. inpaint=.. slope=..
    for m in re.finditer(
        r"total=([\d.]+)ms\s+snapshot=([\d.]+)ms\s+bev=([\d.]+)ms\s+"
        r"inpaint=([\d.]+)ms\s+slope=([\d.]+)ms",
        text,
    ):
        out["mapping_extract"].append(
            {
                "total_ms": float(m.group(1)),
                "snapshot_ms": float(m.group(2)),
                "bev_ms": float(m.group(3)),
                "inpaint_ms": float(m.group(4)),
                "slope_ms": float(m.group(5)),
            }
        )

    def _avg_last(rows: list, keys: list[str], last_n: int = 5) -> dict:
        if not rows:
            return {}
        use = rows[-last_n:]
        return {
            k: float(np.mean([r[k] for r in use if k in r]))
            for k in keys
            if any(k in r for r in use)
        }

    summary = {
        "stereo_cuvslam": _avg_last(
            out["stereo_cuvslam"],
            ["track_mean_ms", "track_max_ms", "iter_mean_ms", "track_hz", "ir_hz"],
        ),
        "clipseg_encoder": _avg_last(
            out["clipseg_encoder"],
            ["clipseg_encode", "feature_xfer", "n"],
        ),
        "seg_refine": _avg_last(
            out["seg_refine"],
            [
                "film_decode",
                "sam_encoder",
                "sam_decoder_nav",
                "stream_wall",
                "stream_overlap",
                "n",
            ],
        ),
        "mapping_extract": _avg_last(
            out["mapping_extract"],
            ["total_ms", "bev_ms", "inpaint_ms", "slope_ms", "snapshot_ms"],
        ),
        "raw_counts": {k: len(v) for k, v in out.items()},
    }
    return summary


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("/tmp/hound_dataset_eval"))
    ap.add_argument(
        "--dataset",
        type=Path,
        required=True,
        help="source rail_sim dataset (GT + depth/trav/poses)",
    )
    ap.add_argument("--duration", type=float, default=75.0)
    ap.add_argument("--odom", default="/visual_slam/tracking/odometry")
    ap.add_argument("--gt", default="/dataset/gt_odometry")
    ap.add_argument("--local-map", default="/hound_mapping/local_map")
    ap.add_argument("--log", type=Path, default=Path("/tmp/hound_dataset_pipeline.log"))
    ap.add_argument("--max-maps", type=int, default=512)
    args = ap.parse_args()

    import rclpy
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    from hound_mapping.msg import LocalMap

    # Shared helpers live next to this script.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from rail_eval_dataset import (  # noqa: E402
        materialize_elev_dumps,
        prepare_eval_dataset,
        read_camera_geom,
        validate_maps_vs_dataset,
    )

    args.out.mkdir(parents=True, exist_ok=True)
    # Defer prepare (which clears elev dumps) until after we have maps, so a
    # failed collect does not wipe a previous good replay tree.
    geom = read_camera_geom(args.dataset)
    print(
        f"[collect] expect LocalMap {geom['grid']}x{geom['grid']} @ {geom['voxel']}m "
        f"(will write elev → {args.out}/frames after maps arrive)",
        flush=True,
    )

    rclpy.init()
    node = Node("dataset_pipeline_collect")
    est_xyz: list[np.ndarray] = []
    est_q: list[np.ndarray] = []
    gt_xyz: list[np.ndarray] = []
    gt_q: list[np.ndarray] = []
    maps: list[dict] = []
    latest_pose = {"xyz": None, "q": None}
    t0 = time.time()

    def on_pair_est(est: Odometry) -> None:
        ex = np.array(
            [
                est.pose.pose.position.x,
                est.pose.pose.position.y,
                est.pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        eq = np.array(
            [
                est.pose.pose.orientation.x,
                est.pose.pose.orientation.y,
                est.pose.pose.orientation.z,
                est.pose.pose.orientation.w,
            ],
            dtype=np.float64,
        )
        latest_pose["xyz"] = ex
        latest_pose["q"] = eq

    def on_pair_gt(gt: Odometry) -> None:
        # Prefer GT pose for map association when odom_source=gt publishes both.
        latest_pose["xyz"] = np.array(
            [
                gt.pose.pose.position.x,
                gt.pose.pose.position.y,
                gt.pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        latest_pose["q"] = np.array(
            [
                gt.pose.pose.orientation.x,
                gt.pose.pose.orientation.y,
                gt.pose.pose.orientation.z,
                gt.pose.pose.orientation.w,
            ],
            dtype=np.float64,
        )

    def on_sync_pair(est: Odometry, gt: Odometry) -> None:
        est_xyz.append(
            np.array(
                [
                    est.pose.pose.position.x,
                    est.pose.pose.position.y,
                    est.pose.pose.position.z,
                ],
                dtype=np.float64,
            )
        )
        est_q.append(
            np.array(
                [
                    est.pose.pose.orientation.x,
                    est.pose.pose.orientation.y,
                    est.pose.pose.orientation.z,
                    est.pose.pose.orientation.w,
                ],
                dtype=np.float64,
            )
        )
        gt_xyz.append(
            np.array(
                [
                    gt.pose.pose.position.x,
                    gt.pose.pose.position.y,
                    gt.pose.pose.position.z,
                ],
                dtype=np.float64,
            )
        )
        gt_q.append(
            np.array(
                [
                    gt.pose.pose.orientation.x,
                    gt.pose.pose.orientation.y,
                    gt.pose.pose.orientation.z,
                    gt.pose.pose.orientation.w,
                ],
                dtype=np.float64,
            )
        )

    def _img32(msg) -> np.ndarray | None:
        if msg.height == 0 or msg.width == 0 or not msg.data:
            return None
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
        return arr.reshape(int(msg.height), int(msg.width))

    def on_map(msg: LocalMap) -> None:
        if len(maps) >= args.max_maps:
            return
        elev = _img32(msg.elevation)
        cost = _img32(msg.costmap)
        if elev is None or cost is None or elev.shape != cost.shape:
            return
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        res = float(msg.info.resolution)
        if latest_pose["xyz"] is not None:
            pose_xyz = latest_pose["xyz"].copy()
            pose_q = latest_pose["q"].copy()
        else:
            pose_xyz = np.array(
                [
                    ox + 0.5 * elev.shape[1] * res,
                    oy + 0.5 * elev.shape[0] * res,
                    0.0,
                ],
                dtype=np.float64,
            )
            pose_q = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        maps.append(
            {
                "elevation": elev.copy(),
                "cost": cost.copy(),
                "resolution": res,
                "origin_x": ox,
                "origin_y": oy,
                "stamp_sec": float(msg.header.stamp.sec)
                + 1e-9 * float(msg.header.stamp.nanosec),
                "pose_xyz": pose_xyz,
                "pose_q": pose_q,
            }
        )

    sub_est = node.create_subscription(
        Odometry, args.odom, on_pair_est, 10
    )
    sub_gt = node.create_subscription(
        Odometry, args.gt, on_pair_gt, 10
    )
    # Keep synced pairs for RMSE when both streams share stamps.
    mf_est = Subscriber(node, Odometry, args.odom)
    mf_gt = Subscriber(node, Odometry, args.gt)
    sync = ApproximateTimeSynchronizer(
        [mf_est, mf_gt], queue_size=50, slop=0.1
    )
    sync.registerCallback(on_sync_pair)
    node.create_subscription(
        LocalMap, args.local_map, on_map, qos_profile_sensor_data
    )

    print(
        f"[collect] listening {args.duration:.0f}s "
        f"(odom={args.odom}, gt={args.gt}, map={args.local_map})",
        flush=True,
    )
    while time.time() - t0 < args.duration and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    elapsed = time.time() - t0
    node.destroy_node()
    rclpy.shutdown()

    result: dict = {
        "elapsed_s": elapsed,
        "n_pose_pairs": len(est_xyz),
        "n_local_maps": len(maps),
        "expect_grid": geom["grid"],
        "expect_res": geom["voxel"],
    }

    if len(est_xyz) >= 2:
        A = np.stack(est_xyz, axis=0)
        B = np.stack(gt_xyz, axis=0)
        s, R, t = _umeyama(A, B, with_scale=True)
        A_al = (s * (R @ A.T)).T + t
        err = np.linalg.norm(A_al - B, axis=1)
        s1, R1, t1 = _umeyama(A, B, with_scale=False)
        A_se3 = (R1 @ A.T).T + t1
        err_se3 = np.linalg.norm(A_se3 - B, axis=1)
        ang = [
            _quat_angle_deg(est_q[i], gt_q[i]) for i in range(len(est_q))
        ]
        result["rmse"] = {
            "n": int(len(err)),
            "trans_rmse_m_sim3": float(np.sqrt(np.mean(err ** 2))),
            "trans_rmse_m_se3": float(np.sqrt(np.mean(err_se3 ** 2))),
            "trans_mean_m_sim3": float(np.mean(err)),
            "trans_median_m_sim3": float(np.median(err)),
            "trans_max_m_sim3": float(np.max(err)),
            "scale": float(s),
            "rot_mean_deg_raw": float(np.mean(ang)),
            "rot_median_deg_raw": float(np.median(ang)),
            "note": (
                "trans RMSE after Umeyama align of VSLAM traj → GT "
                "(Sim3 includes scale). rot_* is raw quat angle before align."
            ),
        }
        np.savez_compressed(
            args.out / "poses.npz",
            est_xyz=A,
            gt_xyz=B,
            est_xyz_aligned_sim3=A_al,
            est_q=np.stack(est_q),
            gt_q=np.stack(gt_q),
            err_sim3=err,
        )
    else:
        result["rmse"] = {
            "n": len(est_xyz),
            "error": "insufficient synchronized pose pairs (VSLAM may not be tracking)",
        }

    if maps:
        np.savez_compressed(
            args.out / "local_maps.npz",
            n=len(maps),
            resolution=np.array([m["resolution"] for m in maps]),
            origin_x=np.array([m["origin_x"] for m in maps]),
            origin_y=np.array([m["origin_y"] for m in maps]),
            stamp_sec=np.array([m["stamp_sec"] for m in maps]),
            pose_xyz=np.stack([m["pose_xyz"] for m in maps]),
            pose_q=np.stack([m["pose_q"] for m in maps]),
            elevation=np.stack([m["elevation"] for m in maps]),
            cost=np.stack([m["cost"] for m in maps]),
        )
        geom_bad = validate_maps_vs_dataset(maps, read_camera_geom(args.dataset))
        result["local_map_geom_ok"] = not geom_bad
        result["local_map_geom_errors"] = geom_bad
        if geom_bad:
            print(
                "[collect] ERROR: LocalMap does not match rail camera.txt:\n  - "
                + "\n  - ".join(geom_bad),
                flush=True,
            )
        else:
            geom = read_camera_geom(args.dataset)
            n_expect = int(geom.get("n_frames", 0))
            n_elev_now = len(list((args.out / "frames").glob("*_elev.f32")))
            # dump_elev_dir writes into EVAL_OUT/frames; prepare_eval_dataset
            # would wipe those. Prefer keeping mapper dumps when dense enough.
            if n_expect > 0 and n_elev_now >= max(10, int(0.5 * n_expect)):
                info = materialize_elev_dumps(
                    args.out, maps, src_dataset=args.dataset
                )
            else:
                prepare_eval_dataset(args.dataset, args.out)
                info = materialize_elev_dumps(
                    args.out, maps, src_dataset=args.dataset
                )
            result["rail_elev"] = info
            print(
                f"[collect] wrote {info['n_elev_written']} elev dumps → "
                f"{args.out}/frames/*_elev.f32  "
                f"({info['grid']}x{info['grid']} @ {info['resolution']}m, "
                f"origin_x span={info['origin_x_span']:.2f}m, "
                f"assoc={info.get('assoc')})",
                flush=True,
            )
            print(
                "[collect] replay with the SAME viewer as rail smoke:\n"
                f"  python3 {Path(__file__).resolve().parents[2]}/hound_mapping/scripts/"
                f"viser_rail_compare.py --dataset {args.out} --port 8081",
                flush=True,
            )
    else:
        print(
            "[collect] ERROR: n_local_maps=0 — left previous elev dumps intact "
            "(if any). Check mapping TF / extract_on_depth.",
            flush=True,
        )

    result["profile"] = _parse_profile_log(args.log)
    (args.out / "metrics.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    print(f"[collect] wrote {args.out}/metrics.json", flush=True)


if __name__ == "__main__":
    main()
