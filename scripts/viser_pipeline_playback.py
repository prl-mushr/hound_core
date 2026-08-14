#!/usr/bin/env python3
"""Replay pipeline elev dumps with the SAME viewer as smoke_rail_mapper.

Preferred (after a successful smoke collect):
  python3 scripts/viser_pipeline_playback.py \\
    --dataset /tmp/hound_dataset_eval --port 8081

Also accepted (materialize LocalMap npz → eval tree, then same viewer):
  python3 scripts/viser_pipeline_playback.py \\
    --dataset /path/to/rail_sim/race-2 \\
    --maps /tmp/hound_dataset_eval/local_maps.npz \\
    --port 8081
"""

from __future__ import annotations

import argparse
import runpy
import sys
from pathlib import Path


def _looks_like_rail_source(p: Path) -> bool:
    return (p / "camera.txt").is_file() and (p / "maps" / "elevation.npy").is_file()


def _looks_like_eval(p: Path) -> bool:
    return (p / "frames").is_dir() and (
        (p / "local_maps.npz").is_file()
        or any((p / "frames").glob("*_elev.f32"))
    )


def _fail_bad_maps(src: Path, err: BaseException) -> None:
    print(str(err), file=sys.stderr, flush=True)
    n_src = len(list((src / "frames").glob("*_elev.f32")))
    raise SystemExit(
        "\nCannot materialize pipeline LocalMap dump for rail replay.\n"
        "Re-run:  bash .../smoke_dataset_pipeline.sh\n"
        "Then:    python3 .../viser_pipeline_playback.py "
        "--dataset /tmp/hound_dataset_eval --port 8081\n"
        + (
            f"\nOr view existing smoke_rail_mapper elev on the source "
            f"dataset ({n_src} dumps):\n"
            f"  python3 .../hound_mapping/scripts/viser_rail_compare.py "
            f"--dataset {src} --port 8081\n"
            if n_src
            else ""
        )
    ) from err


def _launch_rail_compare(dataset: Path, port: int, stride: int, play_hz: float) -> None:
    rail = (
        Path(__file__).resolve().parents[2]
        / "hound_mapping"
        / "scripts"
        / "viser_rail_compare.py"
    )
    if not rail.is_file():
        raise SystemExit(f"missing {rail}")
    sys.argv = [
        str(rail),
        "--dataset",
        str(dataset),
        "--port",
        str(port),
        "--stride",
        str(stride),
        "--play-hz",
        str(play_hz),
    ]
    print(
        f"[playback] launching viser_rail_compare on {dataset}  port={port}",
        flush=True,
    )
    runpy.run_path(str(rail), run_name="__main__")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--dataset",
        type=Path,
        required=True,
        help="eval root (/tmp/hound_dataset_eval) OR rail_sim source with --maps",
    )
    ap.add_argument(
        "--source-dataset",
        type=Path,
        default=None,
        help="rail_sim source (optional; defaults to --dataset when using --maps)",
    )
    ap.add_argument(
        "--eval-out",
        type=Path,
        default=None,
        help="where to write elev dumps when using --maps "
        "(default: directory containing --maps)",
    )
    ap.add_argument(
        "--maps",
        type=Path,
        default=None,
        help="optional local_maps.npz to materialize into eval frames/*_elev.f32",
    )
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--stride", type=int, default=2)
    ap.add_argument("--play-hz", type=float, default=10.0)
    args = ap.parse_args()

    here = Path(__file__).resolve().parent
    sys.path.insert(0, str(here))
    from rail_eval_dataset import materialize_from_npz

    dataset = args.dataset.resolve()

    if args.maps is not None:
        maps = args.maps.resolve()
        if not maps.is_file():
            raise SystemExit(
                f"missing --maps {maps}\n"
                "Re-run smoke_dataset_pipeline.sh first (it writes local_maps.npz "
                "and frames/*_elev.f32 under EVAL_OUT)."
            )
        src = (args.source_dataset or dataset).resolve()
        if not _looks_like_rail_source(src):
            raise SystemExit(
                f"--maps needs a rail_sim source (camera.txt + maps/elevation.npy); "
                f"got {src}"
            )
        eval_out = (
            args.eval_out.resolve()
            if args.eval_out is not None
            else Path("/tmp/hound_dataset_eval")
        )
        # Never write elev dumps into the canonical rail_sim tree.
        if eval_out == src:
            eval_out = Path("/tmp/hound_dataset_eval")
            print(
                f"[playback] refusing to overwrite {src}; "
                f"materializing into {eval_out}",
                flush=True,
            )
        print(
            f"[playback] materializing {maps} → {eval_out}/frames/*_elev.f32 "
            f"(source={src})",
            flush=True,
        )
        try:
            info = materialize_from_npz(src, eval_out, maps)
        except SystemExit as e:
            _fail_bad_maps(src, e)
        except Exception as e:
            _fail_bad_maps(src, e)
        print(f"[playback] {info}", flush=True)
        dataset = eval_out

    frames = dataset / "frames"
    n_elev = len(list(frames.glob("*_elev.f32"))) if frames.is_dir() else 0
    if n_elev == 0:
        # If user pointed at source rail_sim with existing rail elev dumps, use them.
        if _looks_like_rail_source(dataset):
            n_elev = len(list((dataset / "frames").glob("*_elev.f32")))
        if n_elev == 0:
            raise SystemExit(
                f"No *_elev.f32 under {frames}.\n"
                "Re-run smoke, then:\n"
                "  python3 .../viser_pipeline_playback.py "
                "--dataset /tmp/hound_dataset_eval --port 8081"
            )

    meta = frames / "000000_elev.txt"
    if meta.is_file():
        print(f"[playback] {n_elev} elev dumps; frame0:\n{meta.read_text()}", flush=True)

    _launch_rail_compare(dataset, args.port, args.stride, args.play_hz)


if __name__ == "__main__":
    main()
