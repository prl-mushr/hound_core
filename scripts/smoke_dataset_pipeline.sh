#!/usr/bin/env bash
# Dataset pipeline smoke: single-cam (DatasetCameraDevice) or 3-cam (dataset_replay).
# Ctrl+C (or exit) tears down launch + leftover nodes.
#
# One-liner (inside mushr_jazzy):
#   bash /root/colcon_ws/src/hound_core/scripts/smoke_dataset_pipeline.sh
# Multicam (default when race-2_multicam exists, or MULTICAM=1):
#   MULTICAM=1 bash .../smoke_dataset_pipeline.sh
#
# Env knobs:
#   MULTICAM=1|0|auto           3-cam dataset_replay (default auto → on if assets exist)
#   DATASET=.../rail_sim/...    rail dataset root
#   DATASET_VIEW=               view infix; empty = legacy stems (single-cam only)
#   RATE_HZ=                    default = 1/meta.dt_s (race-2 → 20 Hz)
#   LOOP=0                      preferred: one-pass elev dump after stack is ready
#                               (device still loops during bring-up so the pass
#                               is not eaten before seg/mapping load)
#   ENABLE_VSLAM=1              single-cam: run cuVSLAM Track (ignored for multicam)
#   ODOM_SOURCE=gt              vslam | gt  (gt → mapping uses dataset poses)
#   SAM_NODE=1|0                multicam: ClipSeg+NanoSAM refine (default 1); 0=FiLM only
#   BAKE_IR=1                   force IR bake even if ENABLE_VSLAM=0 (single-cam)
#   PORT=8080                   Viser port (live viz during smoke)
#   DURATION_S=                 collect window after LocalMap is alive
#                               (default ≈ n_frames/RATE_HZ + margin)
#   EVAL_OUT=/tmp/hound_dataset_eval
#
# Mapping geom is taken from DATASET/camera.txt (voxel_size, window_radius_m,
# max_depth_m) so LocalMap matches smoke_rail_mapper — not live SSoT (0.1 m / 32 m).
set -eo pipefail

ROOT="${ROS_WORKSPACE:-/root/colcon_ws}"
if [[ ! -d "${ROOT}/src/hound_core" && -d "/home/hound/colcon_ws/src/hound_core" ]]; then
  ROOT="/home/hound/colcon_ws"
fi
SRC="${ROOT}/src"
MULTICAM_DS="${SRC}/hound_mapping/data/rail_sim/race-2_multicam"
MULTICAM="${MULTICAM:-auto}"
if [[ "${MULTICAM}" == "auto" ]]; then
  if [[ -f "${MULTICAM_DS}/camera.txt" ]] || [[ "${DATASET:-}" == *"multicam"* ]]; then
    MULTICAM=1
  else
    MULTICAM=0
  fi
fi

if [[ "${MULTICAM}" == "1" ]]; then
  DATASET="${DATASET:-${MULTICAM_DS}}"
  ENABLE_VSLAM="${ENABLE_VSLAM:-0}"
  ODOM_SOURCE="${ODOM_SOURCE:-gt}"
  EVAL_OUT="${EVAL_OUT:-/tmp/hound_dataset_eval_multicam}"
  LOG="${LOG:-/tmp/hound_dataset_pipeline_multicam.log}"
  SMOKE_WS="${SMOKE_WS:-/tmp/hound_dataset_pipeline_multicam_ws}"
else
  DATASET="${DATASET:-${SRC}/hound_mapping/data/rail_sim/race-2}"
  ENABLE_VSLAM="${ENABLE_VSLAM:-1}"
  ODOM_SOURCE="${ODOM_SOURCE:-gt}"
  EVAL_OUT="${EVAL_OUT:-/tmp/hound_dataset_eval}"
  LOG="${LOG:-/tmp/hound_dataset_pipeline.log}"
  SMOKE_WS="${SMOKE_WS:-/tmp/hound_dataset_pipeline_ws}"
fi
DATASET_VIEW="${DATASET_VIEW-}"   # default empty (legacy race-2 naming)
BAKE_IR="${BAKE_IR:-0}"
PORT="${PORT:-8080}"
LOOP="${LOOP:-0}"
DURATION_S="${DURATION_S-}"   # empty → auto when collecting

# ROS setup.bash references optional unset vars (AMENT_TRACE_SETUP_FILES, …).
set +u
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
set -u

# Auto-generate multicam dataset (full frames @ 0.05) if missing.
if [[ "${MULTICAM}" == "1" && ! -f "${DATASET}/camera.txt" ]]; then
  HM="${SRC}/hound_mapping"
  ELEV="${ELEV:-${HM}/data/rail_sim/race-2_height_0p05.npy}"
  PATH_NPY="${PATH_NPY:-}"
  if [[ -z "${PATH_NPY}" ]]; then
    for c in \
      "${SRC}/IGHAStar/Content/standalone/race-2_kinodynamic_elevation_IGHAStar_path.npy" \
      "${SRC}/IGHAStar/Maps/Offroad/race-2_kinodynamic_elevation_IGHAStar_path.npy" \
      "${HM}/data/rail_sim/race-2_path.npy"
    do
      if [[ -f "${c}" ]]; then PATH_NPY="${c}"; break; fi
    done
  fi
  if [[ ! -f "${ELEV}" || -z "${PATH_NPY}" || ! -f "${PATH_NPY}" ]]; then
    echo "ERROR: multicam dataset missing and cannot generate (ELEV/PATH_NPY)" >&2
    exit 1
  fi
  echo "[smoke] generating multicam dataset → ${DATASET} (voxel=0.05, full frames)"
  python3 "${HM}/scripts/generate_rail_dataset.py" \
    --elev "${ELEV}" \
    --path "${PATH_NPY}" \
    --out "${DATASET}" \
    --multicam \
    --map-res 0.05 \
    --voxel-size 0.05 \
    --window-radius 10.0 \
    --step 0.5 \
    --max-frames "${MAX_FRAMES:-159}"
fi

if [[ ! -f "${DATASET}/camera.txt" ]]; then
  echo "ERROR: dataset missing camera.txt: ${DATASET}" >&2
  exit 1
fi
echo "[smoke] MULTICAM=${MULTICAM} DATASET=${DATASET}"

# Default playback rate from meta.yaml dt_s (authored frame period).
# Multicam extract cannot sustain 20 Hz + 3× depth/color without queue lag;
# default to 10 Hz unless the user sets RATE_HZ.
if [[ -z "${RATE_HZ:-}" ]]; then
  if [[ "${MULTICAM}" == "1" ]]; then
    RATE_HZ=10
  else
    RATE_HZ="$(DATASET="${DATASET}" python3 - <<'PY'
from pathlib import Path
import os
import yaml
meta = Path(os.environ["DATASET"]) / "meta.yaml"
dt = 0.05
if meta.is_file():
    m = yaml.safe_load(meta.read_text()) or {}
    dt = float(m.get("dt_s", dt))
print(f"{1.0 / max(dt, 1e-6):.6g}")
PY
)"
  fi
fi
echo "[smoke] RATE_HZ=${RATE_HZ} (dataset frame rate; override with RATE_HZ=...)"

# Collect mode (default): need a live stream while seg/mapping load, then record
# one dataset-length window. LOOP=0 on the device would finish in ~n/rate seconds
# before LocalMap exists (this run got n_local_maps=0).
COLLECT=1
if [[ "${LOOP}" == "1" && -z "${DURATION_S}" ]]; then
  COLLECT=0
  DURATION_S=0
fi
if [[ -z "${DURATION_S}" ]]; then
  DURATION_S="$(DATASET="${DATASET}" RATE_HZ="${RATE_HZ}" python3 - <<'PY'
from pathlib import Path
import os, yaml
ds = Path(os.environ["DATASET"])
rate = float(os.environ["RATE_HZ"])
n = 159
cam = ds / "camera.txt"
if cam.is_file():
    for line in cam.read_text().splitlines():
        if line.startswith("n_frames="):
            n = int(float(line.split("=", 1)[1]))
meta = ds / "meta.yaml"
if meta.is_file():
    m = yaml.safe_load(meta.read_text()) or {}
    n = int(m.get("n_frames", n))
# One full pass + margin after LocalMap is up. Prefer denser LocalMap coverage
# than n_frames so elev association does not hold-then-snap.
print(max(20, int(2.0 * n / max(rate, 0.1)) + 5))
PY
)"
fi
# Device must loop during bring-up + collect; LOOP=0 only means "exit after dump".
DEVICE_LOOP=1
if [[ "${COLLECT}" == "0" ]]; then
  DEVICE_LOOP=1
fi
echo "[smoke] COLLECT=${COLLECT} DEVICE_LOOP=${DEVICE_LOOP} (user LOOP=${LOOP}) DURATION_S=${DURATION_S} EVAL_OUT=${EVAL_OUT}"

# Bake synthetic stereo IR when cuVSLAM is on and frames lack IR (single-cam only).
need_ir=0
if [[ "${MULTICAM}" != "1" ]] && [[ "${ENABLE_VSLAM}" == "1" || "${BAKE_IR}" == "1" ]]; then
  if ! compgen -G "${DATASET}/frames/"*"_ir_left.png" > /dev/null; then
    need_ir=1
  fi
fi
if [[ "${need_ir}" == "1" ]]; then
  echo "[smoke] baking synthetic stereo IR into ${DATASET}/frames ..."
  python3 - <<PY
import sys
from pathlib import Path
import numpy as np
import cv2

hm = Path("${SRC}/hound_mapping")
sys.path.insert(0, str(hm))
from sim.heightfield_camera import synthetic_stereo_ir

frames = Path("${DATASET}") / "frames"
n = 0
for depth_p in sorted(frames.glob("*_depth.npy")):
    stem = depth_p.name[: -len("_depth.npy")]
    ir_l_p = frames / f"{stem}_ir_left.png"
    ir_r_p = frames / f"{stem}_ir_right.png"
    if ir_l_p.is_file() and ir_r_p.is_file():
        continue
    depth = np.load(depth_p).astype(np.float32)
    trav_bgr = cv2.imread(str(frames / f"{stem}_trav.png"), cv2.IMREAD_COLOR)
    if trav_bgr is None:
        print(f"skip {stem}: no trav.png", flush=True)
        continue
    trav = trav_bgr[:, :, ::-1]
    ir_l, ir_r = synthetic_stereo_ir(trav, depth)
    cv2.imwrite(str(ir_l_p), ir_l)
    cv2.imwrite(str(ir_r_p), ir_r)
    n += 1
print(f"[smoke] wrote IR for {n} frames", flush=True)
PY
fi

python3 - <<PY
import math
import os
import yaml
from pathlib import Path

src = Path("${SRC}/hound_core/config/SSoT.yaml")
ssot = yaml.safe_load(src.read_text())
for k in ("fcu_control", "vesc", "hal_monitor", "mavros", "ekf", "lidar", "yolo_world", "nav"):
    if k in ssot and isinstance(ssot[k], dict):
        ssot[k]["enabled"] = False

# Match smoke_rail_mapper: voxel / elev / window / depth from dataset camera.txt.
cam_kv = {}
for line in Path("${DATASET}/camera.txt").read_text().splitlines():
    if "=" in line:
        k, v = line.split("=", 1)
        cam_kv[k.strip()] = v.strip()
voxel = float(cam_kv.get("voxel_size", cam_kv.get("map_res", 0.05)))
window_r = float(cam_kv.get("window_radius_m", 10.0))
max_depth = float(cam_kv.get("max_depth_m", 6.0))
n_frames = int(float(cam_kv.get("n_frames", 0)))
views = [v.strip() for v in cam_kv.get("views", "front").split(",") if v.strip()]
multicam = bool(int("${MULTICAM}"))
if multicam and len(views) < 2:
    views = ["front", "left", "right"]
half = int(math.ceil(window_r / voxel))
grid = 2 * half
print(
    f"[smoke] mapping from camera.txt: voxel={voxel} elev_res={voxel} "
    f"window_r={window_r}m → LocalMap {grid}x{grid} "
    f"(not live SSoT 0.1m/32m) multicam={multicam} views={views}",
    flush=True,
)

sc = ssot["stereo_composite"]
sc["enabled"] = True
sc["architecture"] = "modular"
sc["dataset"] = "${DATASET}"
sc["dataset_path"] = "${DATASET}"
sc["replay_rate_hz"] = float("${RATE_HZ}")
sc["dataset_rate_hz"] = float("${RATE_HZ}")
sc["replay_loop"] = bool(int("${DEVICE_LOOP}"))
sc["dataset_loop"] = bool(int("${DEVICE_LOOP}"))
sc["enable_color"] = True
sc["enable_depth"] = True
sc["align_depth"] = False
sc["log_cuvslam_timing"] = True
sc["profile"] = True
rate = float("${RATE_HZ}")
sc["color_publish_fps"] = rate
sc["depth_publish_fps"] = rate
sc["odom_frame"] = "odom"

if multicam:
    sc["backend"] = "dataset_replay"
    sc["enable_vslam"] = False
    sc["odom_source"] = "gt"
    sc["base_frame"] = "base_link"
    sc["cameras"] = [{"name": v} for v in views]
    sc["warmup_frames"] = 0
else:
    sc["backend"] = "dataset"
    sc["dataset_view"] = """${DATASET_VIEW}"""
    sc["enable_vslam"] = bool(int("${ENABLE_VSLAM}"))
    sc["odom_source"] = str("${ODOM_SOURCE}").strip().lower() or "gt"
    sc["warmup_frames"] = 0 if sc["odom_source"] == "gt" else (0 if not int("${ENABLE_VSLAM}") else 30)
    sc["camera_name"] = "camera"
    sc["base_frame"] = "camera_link"

ssot["segmentation"]["enabled"] = True
ssot["segmentation"]["profile"] = True
ssot["segmentation"]["profile_every"] = 15
if multicam:
    # Default SAM on for full ClipSeg+NanoSAM path (batched encode/decode).
    # SAM_NODE=0 to skip refine (coarse FiLM only).
    ssot["segmentation"]["sam_node"] = str(os.environ.get("SAM_NODE", "1")).strip() not in (
        "0",
        "false",
        "False",
    )
    ssot["segmentation"]["camera_names"] = list(views)
    ssot["segmentation"]["color_topics"] = [f"/{v}/color/image_raw" for v in views]
    ssot["segmentation"]["clipseg_batch_n"] = len(views)
    ssot["segmentation"]["color_topic"] = f"/{views[0]}/color/image_raw"
    # Prefer a native batch-N vision engine when present.
    b3 = Path(
        "/root/colcon_ws/src/perception_models/data/clipseg_vision_rd16_224_b3.engine"
    )
    b3_alt = Path(
        "${SRC}/perception_models/data/clipseg_vision_rd16_224_b3.engine"
    )
    if b3.is_file():
        ssot["segmentation"]["clipseg_vision_engine"] = str(b3)
        print(f"[smoke] clipseg vision engine → {b3} (batch 3)", flush=True)
    elif b3_alt.is_file():
        ssot["segmentation"]["clipseg_vision_engine"] = str(b3_alt)
        print(f"[smoke] clipseg vision engine → {b3_alt} (batch 3)", flush=True)
    else:
        print(
            "[smoke] no b3 ClipSeg engine; using SSoT engine with per-image TRT loop",
            flush=True,
        )
    print(
        f"[smoke] sam_node={ssot['segmentation']['sam_node']} "
        f"(engine={ssot['segmentation'].get('sam_image_encoder', '')})",
        flush=True,
    )
else:
    ssot["segmentation"]["sam_node"] = True
    ssot["segmentation"]["color_topic"] = "/camera/color/image_raw"
    ssot["segmentation"].pop("camera_names", None)
    ssot["segmentation"].pop("color_topics", None)
    ssot["segmentation"]["clipseg_batch_n"] = 1

nv = ssot["nvblox"]
nv["enabled"] = True
nv["backend"] = "hound"
nv["use_lidar"] = False
nv["use_depth"] = True
nv["use_color"] = True
nv["use_people_mask"] = False
nv["voxel_size"] = voxel
nv["elevation_resolution"] = voxel
nv["map_clearing_radius_m"] = window_r
nv["projective_integrator_max_integration_distance_m"] = max_depth + 1.0
nv["lidar_projective_integrator_max_integration_distance_m"] = max_depth + 1.0
nv["layer_visualization_exclusion_radius_m"] = window_r
nv["max_back_projection_distance"] = window_r
nv["integrate_depth_rate_hz"] = rate
nv["integrate_color_rate_hz"] = rate
nv["map_clear_rate_hz"] = rate
nv["publish_map_rate_hz"] = rate
nv["mapper_rate_hz"] = rate
# Rail-smoke parity: one LocalMap extract per depth frame (not coalesced timer).
nv["extract_on_depth"] = True
# Write elev.f32 directly (bypass ROS LocalMap drops) into EVAL_OUT/frames.
nv["dump_elev_dir"] = "${EVAL_OUT}/frames"
nv["dump_elev_rate_hz"] = rate
nv["dump_elev_n_frames"] = n_frames
if multicam:
    nv["camera_names"] = list(views)
    nv["color_topic_template"] = "/segmentation/{name}/coarse_traversability"
    nv["map_clearing_frame_id"] = "base_link"
    nv.pop("color_topic", None)
    # Shared rate limiter across cams — allow full rate per view.
    nv["integrate_depth_rate_hz"] = rate * max(len(views), 1)
    nv["integrate_color_rate_hz"] = rate * max(len(views), 1)
else:
    nv["color_topic"] = "/segmentation/coarse_traversability"
    nv["map_clearing_frame_id"] = "camera_link"
    nv.pop("camera_names", None)
    nv.pop("color_topic_template", None)

viz = ssot.setdefault("viz", {})
viz["enabled"] = True
viz["host"] = "0.0.0.0"
viz["port"] = int("${PORT}")
viz["global_frame"] = "odom"
viz["use_lidar"] = False
viz["use_path"] = False
viz["use_mesh_pose"] = False
viz["use_local_map"] = True
viz["use_odom"] = True
viz["use_camera"] = True
viz["local_map_topic"] = "/hound_mapping/local_map"
viz["odom_topic"] = "/visual_slam/tracking/odometry"
if multicam:
    viz["base_frame"] = "base_link"
    viz["camera_topic"] = f"/{views[0]}/color/image_raw"
else:
    viz["base_frame"] = "camera_link"
    viz["camera_topic"] = "/camera/color/image_raw"

ssot["launch"]["stage_delay_s"] = 2.0
# Faster bring-up for timed evals / one-pass dumps
if int(float("${DURATION_S}" or "0")) > 0 and not multicam:
    sc["warmup_frames"] = min(int(sc.get("warmup_frames", 30)), 15)

out = Path("${SMOKE_WS}/src/hound_core/config")
out.mkdir(parents=True, exist_ok=True)
(out / "SSoT.yaml").write_text(yaml.dump(ssot, sort_keys=False))
# Hint collector how many LocalMaps to keep (≥ one pass).
hint = out / "smoke_dataset_hint.yaml"
hint.write_text(
    yaml.dump(
        {
            "n_frames": n_frames,
            "rate_hz": rate,
            "voxel_size": voxel,
            "window_radius_m": window_r,
            "max_maps": max(2 * max(n_frames, 1), int(rate * 40)),
            "loop": bool(int("${DEVICE_LOOP}")),
            "user_loop": bool(int("${LOOP}")),
            "extract_on_depth": True,
            "multicam": multicam,
            "views": views,
        },
        sort_keys=False,
    )
)
print(f"[smoke] SSoT -> {out / 'SSoT.yaml'}")
print(
    f"[smoke] backend={sc['backend']!r} dataset={sc['dataset']!r} "
    f"vslam={sc.get('enable_vslam')} odom_source={sc.get('odom_source')} "
    f"rate={sc['replay_rate_hz']}Hz device_loop={sc['replay_loop']} "
    f"seg_cams={ssot['segmentation'].get('camera_names') or [ssot['segmentation'].get('color_topic')]} "
    f"map_cams={nv.get('camera_names') or ['camera']}",
    flush=True,
)
print(f"[smoke] Viser http://<host>:${PORT}/  (elev map on /hound_mapping/local_map)")
PY

# Prepare eval dataset tree BEFORE launch so dump_elev_dir exists + GT assets linked.
DATASET="${DATASET}" EVAL_OUT="${EVAL_OUT}" SRC="${SRC}" python3 - <<'PY'
import os, sys
from pathlib import Path
sys.path.insert(0, str(Path(os.environ["SRC"]) / "hound_core" / "scripts"))
from rail_eval_dataset import prepare_eval_dataset
src = Path(os.environ["DATASET"])
out = Path(os.environ["EVAL_OUT"])
geom = prepare_eval_dataset(src, out)
print(
    f"[smoke] prepared {out} for elev dumps "
    f"(expect {geom['grid']}x{geom['grid']} @ {geom['voxel']}m)",
    flush=True,
)
PY

export ROS_WORKSPACE="${SMOKE_WS}"

cleanup() {
  local ec=$?
  trap - INT TERM EXIT
  echo "[smoke] shutting down..."
  if [[ -n "${LPID:-}" ]]; then
    kill "${LPID}" 2>/dev/null || true
    wait "${LPID}" 2>/dev/null || true
  fi
  pkill -9 -f "ros2 launch hound_core hound_core.launch" 2>/dev/null || true
  pkill -9 -f stereo_composite 2>/dev/null || true
  pkill -9 -f "dora run" 2>/dev/null || true
  pkill -9 -f dora_clipseg 2>/dev/null || true
  pkill -9 -f dora_seg_refine 2>/dev/null || true
  pkill -9 -f hound_viz 2>/dev/null || true
  pkill -9 -f mapping_node 2>/dev/null || true
  echo "[smoke] done (log: ${LOG})"
  exit "${ec}"
}
trap cleanup INT TERM EXIT

pkill -9 -f "ros2 launch hound_core hound_core.launch" 2>/dev/null || true
pkill -9 -f stereo_composite 2>/dev/null || true
pkill -9 -f "dora run" 2>/dev/null || true
sleep 1

echo "[smoke] launching (Ctrl+C to stop). log=${LOG}"
echo "[smoke] profile: stereo log_cuvslam_timing+profile, seg profile=true, mapping extract_timing"
set +e
ros2 launch hound_core hound_core.launch.py >"${LOG}" 2>&1 &
LPID=$!
set -e

# Tail useful lines; keep launch in background so trap still owns lifecycle.
sleep 3
if ! kill -0 "${LPID}" 2>/dev/null; then
  echo "[smoke] launch exited early — see ${LOG}" >&2
  tail -80 "${LOG}" >&2 || true
  exit 1
fi

echo "[smoke] launch pid=${LPID}"
echo "[smoke] grepping bring-up (also: tail -f ${LOG})"
grep -E "stage |stereo_composite ENABLED|DatasetCamera|dataset_replay|multicam_rail_replay|segmentation ENABLED|nvblox ENABLED|viz ENABLED|camera_names|Error|error|fatal|Traceback|FAILED" "${LOG}" | head -40 || true

if [[ "${DURATION_S}" != "0" ]]; then
  echo "[smoke] waiting for /hound_mapping/local_map (seg+mapper load), then collect ${DURATION_S}s → ${EVAL_OUT}"
  set +e
  python3 - <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from hound_mapping.msg import LocalMap

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
rclpy.init()
node = Node("smoke_wait_local_map")
got = {"ok": False}

def cb(_msg):
    got["ok"] = True

node.create_subscription(LocalMap, "/hound_mapping/local_map", cb, qos)
t0 = time.time()
timeout = 120.0
while time.time() - t0 < timeout and rclpy.ok() and not got["ok"]:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
if not got["ok"]:
    print(
        f"[smoke] ERROR: no LocalMap after {timeout:.0f}s — see log",
        file=sys.stderr,
        flush=True,
    )
    sys.exit(2)
print(f"[smoke] LocalMap alive after {time.time() - t0:.1f}s", flush=True)
PY
  wait_ec=$?
  set -e
  if [[ "${wait_ec}" != "0" ]]; then
    tail -80 "${LOG}" >&2 || true
    exit "${wait_ec}"
  fi
  # Extra headroom so dump_elev_dir can catch a full loop after models are warm.
  echo "[smoke] collecting ${DURATION_S}s (mapper also dumps elev → ${EVAL_OUT}/frames)"
  set +u
  MAX_MAPS="$(SMOKE_WS="${SMOKE_WS}" python3 - <<'PY'
from pathlib import Path
import os
import yaml
hint = Path(os.environ["SMOKE_WS"]) / "src/hound_core/config/smoke_dataset_hint.yaml"
n = 512
if hint.is_file():
    h = yaml.safe_load(hint.read_text()) or {}
    n = int(h.get("max_maps", n))
print(n)
PY
)"
  python3 "${SRC}/hound_core/scripts/eval_dataset_pipeline_collect.py" \
    --out "${EVAL_OUT}" \
    --dataset "${DATASET}" \
    --duration "${DURATION_S}" \
    --max-maps "${MAX_MAPS}" \
    --log "${LOG}" || true
  set -u
  # Prefer mapper-written elev dumps; report coverage.
  n_elev=$(ls -1 "${EVAL_OUT}/frames/"*_elev.f32 2>/dev/null | wc -l | tr -d ' ')
  echo "[smoke] elev dumps on disk: ${n_elev}"
  if [[ "${n_elev}" -lt 10 ]]; then
    echo "[smoke] ERROR: too few elev dumps (${n_elev}) — check dump_elev_dir / dataset stamps" >&2
  fi
  echo "[smoke] metrics done; stopping launch"
  kill "${LPID}" 2>/dev/null || true
  wait "${LPID}" 2>/dev/null || true
  if [[ -f "${EVAL_OUT}/metrics.json" ]]; then
    echo "[smoke] === metrics.json ==="
    cat "${EVAL_OUT}/metrics.json"
  fi
  if [[ "${n_elev}" -gt 0 ]]; then
    echo "[smoke] Viser replay (SAME as rail mapper smoke):"
    echo "  python3 ${SRC}/hound_mapping/scripts/viser_rail_compare.py \\"
    echo "    --dataset ${EVAL_OUT} --port 8081"
  else
    echo "[smoke] ERROR: no ${EVAL_OUT}/frames/*_elev.f32" >&2
  fi
  exit 0
fi

# Block until launch dies or Ctrl+C.
wait "${LPID}"
