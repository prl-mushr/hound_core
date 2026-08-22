#!/usr/bin/env python3
"""Write vlcal ``init_T_lidar_camera`` from HOUND SSoT extrinsics.

Skips the manual picking GUI when mounts are already in SSoT. Computes
``p_lidar = T_lidar_camera * p_camera`` for the color optical frame, matching
``apply_vlcal_to_ssot.py`` (inverse direction).

Chain:
  base_link → lidar          (lidar.xyz/rpy)
  base_link → camera_front   (stereo_composite.cameras.camera_front xyz/rpy)
  camera_front → side cams   (stereo_composite.cameras.* xyz/rpy)

Examples:
  seed_vlcal_from_ssot.py camera_left camera_right
  seed_vlcal_from_ssot.py --calib-root /root/colcon_ws/calib/vlcal camera_left
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List

import yaml

from apply_vlcal_to_ssot import (
    CAMERAS,
    R_to_quat_xyzw,
    T_from_xyz_rpy_deg,
    T_inv,
    default_T_link_optical,
    fmt_list,
    lookup_T_link_optical,
)


def load_ssot(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def T_lidar_camera_from_ssot(ssot: dict, camera: str, *, use_tf: bool) -> "object":
    import numpy as np

    lidar = ssot.get("lidar") or {}
    T_base_lidar = T_from_xyz_rpy_deg(
        list(lidar.get("xyz") or [0.0, 0.0, 0.0]),
        list(lidar.get("rpy") or [0.0, 0.0, 0.0]),
    )

    cams = (ssot.get("stereo_composite") or {}).get("cameras") or {}
    front = cams.get("camera_front")
    if not front:
        raise ValueError("stereo_composite.cameras.camera_front missing in SSoT")
    T_base_front = T_from_xyz_rpy_deg(
        list(front.get("xyz") or [0.0, 0.0, 0.0]),
        list(front.get("rpy") or [0.0, 0.0, 0.0]),
    )

    if camera == "camera_front":
        T_base_cam = T_base_front
    else:
        side = cams.get(camera)
        if not side:
            raise ValueError(f"stereo_composite.cameras.{camera} missing in SSoT")
        T_front_side = T_from_xyz_rpy_deg(
            list(side.get("xyz") or [0.0, 0.0, 0.0]),
            list(side.get("rpy") or [0.0, 0.0, 0.0]),
        )
        T_base_cam = T_base_front @ T_front_side

    if use_tf:
        T_link_optical = lookup_T_link_optical(camera)
        if T_link_optical is None:
            T_link_optical = default_T_link_optical()
            optical_src = "fallback optical↔FLU (TF unavailable)"
        else:
            optical_src = f"TF {camera}_link→color_optical"
    else:
        T_link_optical = default_T_link_optical()
        optical_src = "fallback optical↔FLU"

    # Inverse of apply_vlcal_to_ssot: T_base_cam = T_base_lidar @ T_lidar_optical @ inv(T_link_optical)
    T_lidar_optical = T_inv(T_base_lidar) @ T_base_cam @ T_link_optical
    return T_lidar_optical, optical_src


def T_to_init_list(T) -> List[float]:
    xyz = [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]
    qx, qy, qz, qw = R_to_quat_xyzw(T[:3, :3])
    return xyz + [qx, qy, qz, qw]


def seed_calib_json(calib_json: Path, init: List[float], *, dry_run: bool) -> None:
    if not calib_json.is_file():
        raise FileNotFoundError(f"Missing {calib_json} — run preprocess first")

    cfg = json.loads(calib_json.read_text(encoding="utf-8"))
    cfg.setdefault("results", {})
    cfg["results"]["init_T_lidar_camera"] = init

    if dry_run:
        print(f"[vlcal] would write init_T_lidar_camera → {calib_json}")
        return

    calib_json.write_text(json.dumps(cfg, indent=2) + "\n", encoding="utf-8")
    print(f"[vlcal] wrote init_T_lidar_camera → {calib_json}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "cameras",
        nargs="+",
        choices=list(CAMERAS),
        help="camera_front | camera_left | camera_right",
    )
    ap.add_argument(
        "--calib-root",
        type=Path,
        default=None,
        help="default: /root/colcon_ws/calib/vlcal or /home/hound/...",
    )
    ap.add_argument(
        "--ssot",
        type=Path,
        default=Path("/home/hound/colcon_ws/src/hound_core/config/SSoT.yaml"),
    )
    ap.add_argument(
        "--tf",
        action="store_true",
        help="Use live ROS TF for link→color_optical (needs cameras up)",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="Print transforms only; do not patch calib.json",
    )
    args = ap.parse_args()

    calib_root = args.calib_root
    if calib_root is None:
        for candidate in (
            Path("/root/colcon_ws/calib/vlcal"),
            Path("/home/hound/colcon_ws/calib/vlcal"),
        ):
            try:
                if candidate.is_dir():
                    calib_root = candidate
                    break
            except OSError:
                continue
        if calib_root is None:
            calib_root = Path("/home/hound/colcon_ws/calib/vlcal")

    ssot = load_ssot(args.ssot)
    print(f"[vlcal] SSoT: {args.ssot}")
    print(f"[vlcal] calib root: {calib_root}")

    ok = 0
    for camera in args.cameras:
        calib_json = calib_root / camera / "preprocessed" / "calib.json"
        try:
            T, optical_src = T_lidar_camera_from_ssot(ssot, camera, use_tf=args.tf)
            init = T_to_init_list(T)
            print(f"\n=== {camera} ===")
            print(f"  optical: {optical_src}")
            print(f"  init_T_lidar_camera: {fmt_list(init)}")
            print(f"            xyz + quat xyzw")
            seed_calib_json(calib_json, init, dry_run=args.dry_run)
            ok += 1
        except (FileNotFoundError, ValueError) as exc:
            print(f"[vlcal] skip {camera}: {exc}", file=sys.stderr)

    if ok == 0:
        return 1
    if not args.dry_run:
        print("\n[vlcal] Next: ./run_vlcal.sh calibrate <camera>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
