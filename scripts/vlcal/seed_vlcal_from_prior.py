#!/usr/bin/env python3
"""Seed vlcal ``init_T_lidar_camera`` from a previous ``calib.json`` result.

vlcal ``preprocess`` rewrites ``preprocessed/calib.json``. Snapshot
``results.T_lidar_camera`` first, then after preprocess write it back as
``init_T_lidar_camera`` so ``calibrate`` only refines.

  seed_vlcal_from_prior.py snapshot camera_front
  seed_vlcal_from_prior.py seed camera_front
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional

CAMERAS = ("camera_front", "camera_left", "camera_right")


def default_calib_root() -> Path:
    for candidate in (
        Path("/root/colcon_ws/calib/vlcal"),
        Path("/home/hound/colcon_ws/calib/vlcal"),
    ):
        try:
            if candidate.is_dir():
                return candidate
        except OSError:
            continue
    return Path("/home/hound/colcon_ws/calib/vlcal")


def extract_T(calib_json: Path) -> List[float]:
    data = json.loads(calib_json.read_text(encoding="utf-8"))
    results = data.get("results") or {}
    t = results.get("T_lidar_camera") or results.get("init_T_lidar_camera")
    if not isinstance(t, list) or len(t) != 7:
        raise ValueError(f"No results.T_lidar_camera[7] in {calib_json}")
    return [float(x) for x in t]


def snapshot(calib_root: Path, camera: str) -> Path:
    src = calib_root / camera / "preprocessed" / "calib.json"
    dest = calib_root / camera / "prior_T_lidar_camera.json"
    t = extract_T(src)
    dest.write_text(
        json.dumps({"T_lidar_camera": t, "source": str(src)}, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"[vlcal] snapshot {camera} T_lidar_camera → {dest}")
    print(f"         {t}")
    return dest


def load_prior_T(calib_root: Path, camera: str) -> List[float]:
    prior = calib_root / camera / "prior_T_lidar_camera.json"
    if prior.is_file():
        data = json.loads(prior.read_text(encoding="utf-8"))
        t = data.get("T_lidar_camera")
        if isinstance(t, list) and len(t) == 7:
            print(f"[vlcal] using prior {prior}")
            return [float(x) for x in t]
    src = calib_root / camera / "preprocessed" / "calib.json"
    print(f"[vlcal] no prior file; reading {src}")
    return extract_T(src)


def seed(calib_root: Path, camera: str) -> None:
    dest = calib_root / camera / "preprocessed" / "calib.json"
    if not dest.is_file():
        raise FileNotFoundError(f"Missing {dest} — preprocess first")
    t = load_prior_T(calib_root, camera)
    cfg = json.loads(dest.read_text(encoding="utf-8"))
    cfg.setdefault("results", {})
    cfg["results"]["init_T_lidar_camera"] = t
    dest.write_text(json.dumps(cfg, indent=2) + "\n", encoding="utf-8")
    print(f"[vlcal] wrote init_T_lidar_camera → {dest}")
    print(f"         {t}")
    print("[vlcal] Next: ./run_vlcal.sh calibrate " + camera)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("action", choices=("snapshot", "seed"))
    ap.add_argument("camera", choices=list(CAMERAS))
    ap.add_argument("--calib-root", type=Path, default=None)
    args = ap.parse_args()
    root = args.calib_root or default_calib_root()
    try:
        if args.action == "snapshot":
            snapshot(root, args.camera)
        else:
            seed(root, args.camera)
    except (FileNotFoundError, ValueError) as exc:
        print(f"[vlcal] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
