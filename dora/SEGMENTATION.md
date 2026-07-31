# HOUND segmentation (dora + CLIPSeg + NanoSAM)

How the live pipeline on Jetson Orin NX was made to work. This is the
authoritative design note for the path that ships; it is **not** the package
top-level README.

Model **backends and engines** live in the `perception_models` package.
`hound_core` owns the dora/ROS nodes and launch config.

## What runs

```
ROS color (/camera/color/image_raw)
        │  rclpy subscribe
        ▼
┌─────────────────────┐
│ dora_clipseg_encoder│  CLIPSeg vision tower (TRT ~6 ms / torch ~29 ms)
└─────────┬───────────┘
          │ dora: features (Arrow f16 on Tegra) + rgb bytes
          │ queue_size from SSoT pipeline_queue_size
          ▼
┌─────────────────────┐
│   dora_seg_refine   │
│  FiLM decode  ∥     │  two CUDA streams (independent inputs)
│  NanoSAM encode     │  then batched NanoSAM decode → ROS
└─────────────────────┘
          │
          ├─ /segmentation/labels
          ├─ /segmentation/refined_traversability
          └─ /segmentation/refined_people_mask
```

Launched from `hound_core.launch.py` via `dora run` on
`dora/segmentation_dataflow.yml` with config JSON from SSoT
(`HOUND_SEG_CONFIG`).

**Rate control:** camera `color_publish_fps` (≈15 Hz works with nvblox). Do not
overdrive color to 30 Hz while refine is GPU-bound — the CLIPSeg process steals
SMs and refined output *drops*.

## Why this shape (and what failed)

| Approach | Result |
|----------|--------|
| Legacy ROS `clipseg_mask_node` + `sam_refine_node` | Removed. Desync / double color subscribe / serial box decode. |
| 4 dora processes (film \| sam_enc \| sam_dec) + queues | Throughput stayed ~`1/sum(GPU)` on one Orin; multi-process without MPS time-slices. |
| CUDA streams inside **one** refine process | Useful structure; FiLM∥encode often still serializes on 8 SMs (`stream_overlap≈0`). |
| Stock NanoSAM decoder (batch=1) | ~13 ms/box serial → ~78 ms for 6 boxes. |
| **Batched mask-decoder TRT engine** | One enqueue for K boxes (~20–30 ms). |
| Encoder @ **1024²** | ~10–11 ms isolated on NX. |
| Encoder @ **512²** + matched decoder | ~3 ms encode; ~15 Hz refined with nvblox. **This is the shipping default.** |
| CLIPSeg vision `torch.compile` + FP16 | ~29 ms eager / worse with cudagraph noise on Orin. |
| **CLIPSeg vision ONNX→TRT FP16** | ~6 ms encode; FiLM stays PyTorch (~10 ms). **Shipping default when engine path set.** |
| CLIPSeg `tile_grid>1` | Dropped — extra encode cost, not needed with NanoSAM refine. |

DeepStream / Triton: doable on Orin NX, but mainly help NVMM / scheduling, not
free parallel FLOPs for CLIPSeg+SAM. Not used here.

Tegra: cross-process CUDA IPC often unsupported → Arrow float16 feature handoff
(~few ms). Expected.

## Engines (CLIPSeg vision)

Default under `perception_models/data/` (fixed shape `[1,3,224,224]` → `[L,S,D]`):

- `clipseg_vision_rd16_224.onnx`
- `clipseg_vision_rd16_224.engine`  (FP16 TRT)

Build / rebuild on **this** Orin (inside the GPU container):

```bash
python3 /root/colcon_ws/src/perception_models/scripts/export_clipseg_vision_onnx.py
bash /root/colcon_ws/src/perception_models/scripts/build_clipseg_vision_engine.sh
python3 /root/colcon_ws/src/perception_models/scripts/bench_clipseg.py \
  --trt-engine /root/colcon_ws/src/perception_models/data/clipseg_vision_rd16_224.engine
```

Spike on this Orin NX: torch eager ~29 ms → TRT ~6 ms (~5×); hs cosine ~0.9998;
FiLM probs max-abs ~0.02. FiLM / prompt decode stay in PyTorch (smaller win,
same IPC contract `[L,S,D]`).

SSoT: set `clipseg_vision_engine` to the `.engine` path (empty = torch).

**Binarization for SAM boxes** is not inside FiLM — `threshold` (default 0.3–0.4)
gates argmax labels in `probs_to_labels`; `sam_min_area` / `sam_max_boxes` turn
nav blobs into NanoSAM prompts. When YOLO owns people, seg_refine runs **two**
mask decodes on one encode: nav (`sam_max_boxes`) then manip
(`sam_max_boxes_manip`) so people boxes cannot starve traversability.
Nudge `threshold` if TRT shifts soft probs slightly.

## Engines (NanoSAM)

Default (512), under `perception_models/data/`:

- `resnet18_image_encoder_512.engine`
- `mobile_sam_mask_decoder_batched_512.engine`

Build / rebuild the 512 pair on **this** Orin (inside the GPU container):

```bash
bash /root/colcon_ws/src/perception_models/scripts/build_nanosam_512_pair.sh \
  /root/colcon_ws/src/perception_models/data 16 32
```

A/B vs 1024: keep unsuffixed / `_batched` 1024 engines, then in SSoT:

```yaml
sam_image_size: 1024   # or 512
sam_image_encoder: ".../resnet18_image_encoder.engine"          # or *_512.engine
sam_mask_decoder:  ".../mobile_sam_mask_decoder_batched.engine" # or *_512.engine
```

`sam_image_size` **must** match both engines (drives preprocess, point scale,
and `upscale_mask` letterbox crop — wrong size → vertically “squeezed” masks).

Batched decoder only (1024 embed 64):

```bash
python3 .../export_nanosam_mask_decoder_batched_onnx.py --output .../....onnx
bash .../build_nanosam_mask_decoder_batched_engine.sh <onnx> <engine> <opt> <max>
```

## Code map

| Piece | Role |
|-------|------|
| `perception_models/clipseg/` | Torch + TRT CLIPSeg vision / FiLM |
| `perception_models/nanosam_trt/` | NanoSAM TRT encoder/decoder wrappers |
| `perception_models/seg_ops.py` | Labels, boxes, overlay math |
| `perception_models/scripts/` | Export / TRT build / bench |
| `perception_models/data/` | ONNX + engines |
| `hound_core/dora/segmentation_dataflow.yml` | 2-node graph + input queues |
| `hound_core/.../dora_clipseg_encoder.py` | ROS in → encode → dora out |
| `hound_core/.../dora_seg_refine.py` | FiLM + streamed NanoSAM → ROS out |
| `hound_core/.../cuda_ipc.py` | CUDA IPC probe + Arrow f16 pack/unpack |
| `hound_core/config/SSoT.yaml` → `segmentation:` | Prompts, engines, streams, topics |
| `nanosam-src/` | Upstream NanoSAM (`pip install -e`) |

## SSoT knobs that matter

- `enabled`, `sam_node`, prompts / `positive_groups` / `negative_groups`
- `clipseg_vision_engine` — TRT vision path; empty = torch
- `threshold` — FiLM→label gate (affects SAM box proposals)
- `publish_coarse_traversability` + `coarse_*_topic` — FiLM trav/people (works with `sam_node: false`)
- `sam_image_*`, `sam_image_size`, `sam_max_boxes`, `sam_max_boxes_manip`, `sam_min_area`
- `use_cuda_streams`, `pipeline_queue_size`
- `profile` / `profile_every` — stage logs; leave **false** for production FPS
- Color FPS via `realsense_cuvslam.color_publish_fps` (or camera block)

Nav-only (no NanoSAM): `sam_node: false`, subscribe
`/segmentation/coarse_traversability`.

Measure with `ros2 topic hz` on coarse or refined trav topics,
not only in-process timers (profilers add sync fences).

## Deps

See `hound_core/dora/requirements.txt` (`dora-rs-cli`, `dora-rs`, `pyarrow`).
`perception_models` needs torch / transformers / TensorRT / torch2trt in the
robot image, plus `pip install -e nanosam-src`.

## Lessons kept

1. One Orin GPU ≈ sum of heavy kernels; process splits need MPS to help much.
2. Batch the mask decoder; don’t serial-loop boxes on a batch=1 engine.
3. Prefer 512 NanoSAM on NX for rate; fix letterbox upscale size with canvas.
4. TRT the CLIPSeg vision tower; leave FiLM in torch until encode is no longer the bottleneck.
5. Cap camera rate to refine capacity when sharing the GPU with nvblox / VSLAM.
