#!/usr/bin/env python3
"""Build a smoke_rail_mapper-compatible eval dataset for viser_rail_compare.

Symlinks GT assets from the source rail_sim dataset, then writes per-frame
``*_elev.f32`` / ``*_elev.txt`` / ``*_cost.f32`` from pipeline LocalMaps —
same layout smoke_rail_mapper dumps into ``frames/``.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np


_SKIP_ELEV = (
    "_elev.f32",
    "_elev.txt",
    "_cost.f32",
    "_observed.u8",
)


def _relink(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        if dst.is_dir() and not dst.is_symlink():
            # Only remove empty / our managed dirs; prefer unlink symlink.
            raise RuntimeError(f"refusing to replace non-symlink dir: {dst}")
        dst.unlink()
    os.symlink(src.resolve(), dst)


def load_poses_T_wo(frames_dir: Path, n: int) -> np.ndarray:
    bin_path = frames_dir / "poses_T_wo.bin"
    if bin_path.exists():
        arr = np.fromfile(bin_path, dtype=np.float64)
        if arr.size >= n * 16:
            return arr[: n * 16].reshape(n, 4, 4)
    body_bin = frames_dir / "poses_body.bin"
    if body_bin.exists():
        arr = np.fromfile(body_bin, dtype=np.float64)
        if arr.size >= n * 16:
            return arr[: n * 16].reshape(n, 4, 4)
    poses = []
    for i in range(n):
        stem = f"{i:06d}"
        for cand in (
            frames_dir / f"{stem}_T_wo.npy",
            frames_dir / f"{stem}_front_T_wo.bin",
            frames_dir / f"{stem}_T_wo.bin",
        ):
            if cand.suffix == ".npy" and cand.exists():
                poses.append(np.load(cand))
                break
            if cand.suffix == ".bin" and cand.exists():
                poses.append(np.fromfile(cand, dtype=np.float64).reshape(4, 4))
                break
        else:
            break
    return np.asarray(poses, dtype=np.float64)


def read_camera_geom(dataset: Path) -> dict:
    kv: dict[str, str] = {}
    for line in (dataset / "camera.txt").read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            kv[k.strip()] = v.strip()
    voxel = float(kv.get("voxel_size", kv.get("map_res", 0.05)))
    window_r = float(kv.get("window_radius_m", 10.0))
    half = int(np.ceil(window_r / voxel))
    return {
        "voxel": voxel,
        "window_r": window_r,
        "grid": 2 * half,
        "n_frames": int(float(kv.get("n_frames", 0))),
        "max_depth_m": float(kv.get("max_depth_m", 6.0)),
    }


def prepare_eval_dataset(src: Path, out: Path) -> dict:
    """Symlink GT/rail/depth/trav/poses; leave elev dumps for materialize."""
    src = src.resolve()
    out = out.resolve()
    if out == src:
        raise SystemExit(
            f"EVAL_OUT must differ from DATASET ({src}) so pipeline elev "
            "dumps do not overwrite smoke_rail_mapper outputs"
        )
    geom = read_camera_geom(src)
    out.mkdir(parents=True, exist_ok=True)
    for name in ("camera.txt", "meta.yaml", "rail_path.npy"):
        _relink(src / name, out / name)
    _relink(src / "maps", out / "maps")

    frames_src = src / "frames"
    frames_out = out / "frames"
    frames_out.mkdir(parents=True, exist_ok=True)
    # Drop previous pipeline elev dumps in out/frames.
    for p in frames_out.glob("*"):
        if p.name.endswith(_SKIP_ELEV):
            p.unlink()
        elif p.is_symlink() or p.is_file():
            # Refresh symlinks for non-elev assets below.
            if not p.name.endswith(_SKIP_ELEV):
                p.unlink()

    n_linked = 0
    for p in sorted(frames_src.iterdir()):
        if any(p.name.endswith(s) for s in _SKIP_ELEV):
            continue
        _relink(p, frames_out / p.name)
        n_linked += 1

    geom["n_linked_frame_files"] = n_linked
    return geom


def _maps_from_npz(path: Path) -> list[dict]:
    data = np.load(path)
    n = int(data["n"]) if "n" in data.files else int(data["elevation"].shape[0])
    maps = []
    has_pose = "pose_xyz" in data.files
    for i in range(n):
        ox = float(data["origin_x"][i])
        oy = float(data["origin_y"][i])
        res = float(data["resolution"][i])
        elev = np.asarray(data["elevation"][i], dtype=np.float32)
        if has_pose:
            pose_xyz = np.asarray(data["pose_xyz"][i], dtype=np.float64)
        else:
            pose_xyz = np.array(
                [
                    ox + 0.5 * elev.shape[1] * res,
                    oy + 0.5 * elev.shape[0] * res,
                    0.0,
                ],
                dtype=np.float64,
            )
        maps.append(
            {
                "elevation": elev,
                "cost": np.asarray(data["cost"][i], dtype=np.float32),
                "resolution": res,
                "origin_x": ox,
                "origin_y": oy,
                "pose_xyz": pose_xyz,
                **(
                    {"stamp_sec": float(data["stamp_sec"][i])}
                    if "stamp_sec" in data.files
                    else {}
                ),
            }
        )
    return maps


def validate_maps_vs_dataset(maps: list[dict], geom: dict) -> list[str]:
    bad: list[str] = []
    if not maps:
        return ["no LocalMaps"]
    res0 = float(maps[0]["resolution"])
    h0, w0 = maps[0]["elevation"].shape
    ox = np.array([m["origin_x"] for m in maps], dtype=np.float64)
    if abs(res0 - geom["voxel"]) > 1e-4:
        bad.append(f"resolution {res0} ≠ camera.txt voxel {geom['voxel']}")
    if w0 != geom["grid"] or h0 != geom["grid"]:
        bad.append(
            f"grid {w0}x{h0} ≠ expected {geom['grid']}x{geom['grid']} "
            f"(window_r={geom['window_r']}m)"
        )
    if len(maps) > 1 and float(np.ptp(ox)) < 0.5:
        bad.append(
            f"origin_x span={float(np.ptp(ox)):.3f}m (map not tracking camera)"
        )
    return bad


def write_elev_txt(
    path: Path, height: int, width: int, resolution: float, ox: float, oy: float
) -> None:
    path.write_text(
        f"height={height}\n"
        f"width={width}\n"
        f"resolution={resolution}\n"
        f"origin_x={ox}\n"
        f"origin_y={oy}\n"
    )


def materialize_elev_dumps(
    eval_dataset: Path,
    maps: list[dict],
    *,
    src_dataset: Path | None = None,
    max_assoc_dist_m: float = 1.25,
    rate_hz: float | None = None,
) -> dict:
    """Write smoke_rail_mapper-style elev dumps for dataset frames.

    Prefer 1:1 stamp→frame when LocalMap stamps follow dataset time
    (``frame_idx = round(stamp_sec * rate_hz) - 1``). Fall back to
    map-center association with hysteresis for sparse / unsynced dumps.
    """
    geom = read_camera_geom(eval_dataset if src_dataset is None else src_dataset)
    frames = eval_dataset / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    n = geom["n_frames"]
    if n <= 0:
        poses = load_poses_T_wo(frames, 10_000)
        n = len(poses)
    else:
        poses = load_poses_T_wo(frames, n)
        n = min(n, len(poses))

    bad = validate_maps_vs_dataset(maps, geom)
    if bad:
        raise SystemExit(
            "[rail_eval] LocalMap geom does not match rail smoke / camera.txt:\n  - "
            + "\n  - ".join(bad)
            + "\nRe-run smoke_dataset_pipeline.sh (do not replay an old "
            "/tmp/.../local_maps.npz)."
        )

    # Preserve mapper dump_elev_dir outputs when already dense enough.
    # Optionally fill missing frame indices from LocalMaps (no wipe).
    existing = sorted(frames.glob("*_elev.f32"))
    existing_idx = {
        int(p.name.split("_")[0])
        for p in existing
        if p.name[:6].isdigit()
    }
    if n > 0 and len(existing) >= max(10, int(0.5 * n)):
        hz = rate_hz
        if hz is None:
            meta_p = (src_dataset or eval_dataset) / "meta.yaml"
            if meta_p.is_file():
                import yaml

                meta = yaml.safe_load(meta_p.read_text()) or {}
                dt = float(meta.get("dt_s", 0.0))
                if dt > 1e-9:
                    hz = 1.0 / dt
        filled = 0
        if hz:
            stamped = [m for m in maps if "stamp_sec" in m]
            by_frame: dict[int, dict] = {}
            for m in stamped:
                fi = int(round(float(m["stamp_sec"]) * float(hz))) - 1
                if fi >= n > 0:
                    fi = fi % n
                if 0 <= fi < n and fi not in existing_idx:
                    by_frame[fi] = m
            for fi, m in sorted(by_frame.items()):
                elev = np.asarray(m["elevation"], dtype=np.float32)
                cost = np.asarray(m["cost"], dtype=np.float32)
                stem = f"{fi:06d}"
                elev.tofile(frames / f"{stem}_elev.f32")
                cost.tofile(frames / f"{stem}_cost.f32")
                write_elev_txt(
                    frames / f"{stem}_elev.txt",
                    elev.shape[0],
                    elev.shape[1],
                    float(m["resolution"]),
                    float(m["origin_x"]),
                    float(m["origin_y"]),
                )
                existing_idx.add(fi)
                filled += 1
        ox = []
        for i in sorted(existing_idx):
            txt = frames / f"{i:06d}_elev.txt"
            if txt.is_file():
                for line in txt.read_text().splitlines():
                    if line.startswith("origin_x="):
                        ox.append(float(line.split("=", 1)[1]))
        n_written = len(existing_idx)
        return {
            "n_frames": n,
            "n_maps": len(maps),
            "n_elev_written": n_written,
            "n_elev_skipped": max(0, n - n_written),
            "n_elev_filled": filled,
            "assoc": "keep_dump_elev_dir",
            "grid": geom["grid"],
            "resolution": geom["voxel"],
            "origin_x_span": float(np.ptp(ox)) if ox else 0.0,
            "note": (
                "Kept mapping_node dump_elev_dir frames"
                + (f"; filled {filled} holes from LocalMaps" if filled else "")
            ),
        }

    for p in frames.glob("*_elev.f32"):
        p.unlink()
    for p in frames.glob("*_elev.txt"):
        p.unlink()
    for p in frames.glob("*_cost.f32"):
        p.unlink()

    # --- Prefer stamp→frame 1:1 (extract_on_depth path) ---
    hz = rate_hz
    if hz is None:
        meta_p = (src_dataset or eval_dataset) / "meta.yaml"
        if meta_p.is_file():
            import yaml

            meta = yaml.safe_load(meta_p.read_text()) or {}
            dt = float(meta.get("dt_s", 0.0))
            if dt > 1e-9:
                hz = 1.0 / dt
    stamped = [m for m in maps if "stamp_sec" in m]
    by_frame: dict[int, dict] = {}
    if hz and stamped:
        for m in stamped:
            # DatasetCameraDevice: timestamp_ns = stereo_frame_id * (1e9/hz),
            # stereo_frame_id starts at 1 → frame index = round(t*hz) - 1.
            fi = int(round(float(m["stamp_sec"]) * float(hz))) - 1
            if 0 <= fi < n:
                by_frame[fi] = m  # last write wins

    if len(by_frame) >= max(1, int(0.5 * n)):
        written = 0
        for fi, m in sorted(by_frame.items()):
            elev = np.asarray(m["elevation"], dtype=np.float32)
            cost = np.asarray(m["cost"], dtype=np.float32)
            stem = f"{fi:06d}"
            elev.tofile(frames / f"{stem}_elev.f32")
            cost.tofile(frames / f"{stem}_cost.f32")
            write_elev_txt(
                frames / f"{stem}_elev.txt",
                elev.shape[0],
                elev.shape[1],
                float(m["resolution"]),
                float(m["origin_x"]),
                float(m["origin_y"]),
            )
            written += 1
        return {
            "n_frames": n,
            "n_maps": len(maps),
            "n_elev_written": written,
            "n_elev_skipped": n - written,
            "assoc": "stamp_to_frame",
            "rate_hz": float(hz),
            "grid": geom["grid"],
            "resolution": geom["voxel"],
            "origin_x_span": float(np.ptp([m["origin_x"] for m in maps])),
            "note": "1:1 LocalMap stamp → dataset frame index",
        }

    # --- Fallback: map-center + hysteresis (sparse dumps) ---
    centers = []
    for m in maps:
        elev = np.asarray(m["elevation"])
        res = float(m["resolution"])
        cx = float(m["origin_x"]) + 0.5 * elev.shape[1] * res
        cy = float(m["origin_y"]) + 0.5 * elev.shape[0] * res
        centers.append([cx, cy])
    pose_xy = np.asarray(centers, dtype=np.float64)

    written = 0
    skipped = 0
    switches = 0
    prev_mi: int | None = None
    hold_dist = max(max_assoc_dist_m, 0.75)

    for i in range(n):
        xy = poses[i, :3, 3][:2]
        d2 = np.sum((pose_xy - xy.reshape(1, 2)) ** 2, axis=1)
        mi = int(np.argmin(d2))
        best = float(np.sqrt(d2[mi]))
        if prev_mi is not None:
            d_prev = float(np.sqrt(d2[prev_mi]))
            if d_prev <= hold_dist and d_prev <= best * 1.35 + 0.15:
                mi = prev_mi
                best = d_prev
        stem = f"{i:06d}"
        if best > max_assoc_dist_m:
            skipped += 1
            prev_mi = None
            continue
        if prev_mi is not None and mi != prev_mi:
            switches += 1
        m = maps[mi]
        elev = np.asarray(m["elevation"], dtype=np.float32)
        cost = np.asarray(m["cost"], dtype=np.float32)
        elev.tofile(frames / f"{stem}_elev.f32")
        cost.tofile(frames / f"{stem}_cost.f32")
        write_elev_txt(
            frames / f"{stem}_elev.txt",
            elev.shape[0],
            elev.shape[1],
            float(m["resolution"]),
            float(m["origin_x"]),
            float(m["origin_y"]),
        )
        written += 1
        prev_mi = mi

    return {
        "n_frames": n,
        "n_maps": len(maps),
        "n_elev_written": written,
        "n_elev_skipped": skipped,
        "n_map_switches": switches,
        "max_assoc_dist_m": max_assoc_dist_m,
        "assoc": "map_center_hysteresis",
        "grid": geom["grid"],
        "resolution": geom["voxel"],
        "origin_x_span": float(np.ptp([m["origin_x"] for m in maps])),
        "note": (
            "Sparse LocalMaps — elev only within max_assoc_dist_m; "
            "re-run smoke with extract_on_depth for 1:1."
        ),
    }


def materialize_from_npz(
    src_dataset: Path, eval_out: Path, maps_npz: Path
) -> dict:
    geom = prepare_eval_dataset(src_dataset, eval_out)
    maps = _maps_from_npz(maps_npz)
    info = materialize_elev_dumps(eval_out, maps, src_dataset=src_dataset)
    info["geom"] = geom
    return info
