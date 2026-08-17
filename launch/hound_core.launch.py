"""Mission / algorithm tier for HOUND (composite sensing, EKF, control, seg).

Bring-up is staggered (launch.stage_delay_s, default 5s between enabled stages):
  HAL → bag_recorder → stereo_composite (VSLAM) → fcu_control (EKF+LL) →
  lidar → segmentation → yolo_world → nvblox → …

fcu_control.enabled is a master switch (nested vesc/ntrip/ll/ekf only when on).
Camera + VSLAM come only from composite_sensing (stereo_composite).
MAVROS / standalone EKF / standalone vesc_driver are not launched here.

Usage:
  ros2 launch hound_core hound_core.launch.py
"""

import math
import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import (  # noqa: E402
    build_bag_recorder_node,
    build_hal_monitor_node,
    build_lidar_mesh_composite_node,
    build_mesh_pf_node,
    build_hound_mapping_node,
    build_viz_node,
    build_nav_dora_actions,
    build_unitree_lidar_actions,
    dump_temp_yaml,
    find_ssot,
    lidar_uses_composite,
)


def _schedule_stages(stages: list, delay_s: float) -> list:
    """Launch enabled stages in order with ``delay_s`` between starts.

    ``stages`` is ``[(name, actions_or_None), ...]``. Empty / None action lists
    are skipped and do not advance the clock. Periods are absolute from launch
    start (t=0, delay, 2*delay, …).
    """
    out = []
    t = 0.0
    for name, acts in stages:
        if not acts:
            print(f"[hound_core] stage SKIP: {name}")
            continue
        print(f"[hound_core] stage @{t:.0f}s: {name} ({len(acts)} action(s))")
        batch = [LogInfo(msg=f"[hound_core] starting stage: {name}"), *acts]
        if t <= 0.0:
            out.extend(batch)
        else:
            out.append(TimerAction(period=float(t), actions=batch))
        t += float(delay_s)
    return out


def _flatten_prompts(prompts):
    if isinstance(prompts, dict):
        names = list(prompts.keys())
        flat, gids = [], []
        for gi, name in enumerate(names):
            for s in (prompts[name] or []):
                flat.append(str(s))
                gids.append(gi)
        return flat, names, gids
    flat = [str(p) for p in (prompts or ["a person"])]
    return flat, flat, list(range(len(flat)))


def _build_segmentation_dora_actions(seg: dict) -> list:
    """Start dora segmentation: rgb_source -> clipseg_encoder -> seg_refine.

    ROS camera ingress stops at rgb_source (packed N×RGB). Encoder/refine are
    Dora-only until product topics. Inside seg_refine, FiLM decode and SAM
    encode overlap on CUDA streams; mask decode follows.
    """
    import json
    import os
    import tempfile

    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    prompts_flat, group_names, group_ids = _flatten_prompts(
        seg.get("prompts", ["a person"])
    )
    pos_names = [str(x) for x in seg.get("positive_groups", [])]
    neg_names = [str(x) for x in seg.get("negative_groups", [])]
    pos_idx = [i for i, n in enumerate(group_names) if n in pos_names]
    neg_idx = [i for i, n in enumerate(group_names) if n in neg_names]
    people_idx = [
        i for i, n in enumerate(group_names) if n not in pos_names and n not in neg_names
    ]

    queue_size = max(2, int(seg.get("pipeline_queue_size", 3)))
    cfg = {
        "model_name": str(seg.get("model", "CIDAS/clipseg-rd16")),
        "prompts": prompts_flat,
        "group_names": group_names,
        "group_ids": group_ids,
        "suppress_prompts": [str(x) for x in seg.get("suppress_prompts", []) if str(x)],
        "aggregate": str(seg.get("aggregate", "max")),
        "clipseg_vision_engine": str(seg.get("clipseg_vision_engine", "") or ""),
        "clipseg_film_engine": str(seg.get("clipseg_film_engine", "") or ""),
        "input_res": int(seg.get("input_res", 224)),
        "threshold": float(seg.get("threshold", 0.3)),
        "compile_mode": str(seg.get("compile_mode", "reduce-overhead")),
        "color_topic": str(seg.get("color_topic", "/camera/color/image_raw")),
        "color_topics": [str(t) for t in (seg.get("color_topics") or [])],
        "camera_names": [str(t) for t in (seg.get("camera_names") or [])],
        "dora_rgb_input": bool(seg.get("dora_rgb_input", True)),
        "ros_io": bool(seg.get("ros_io", True)),
        "use_ros_edge": bool(seg.get("use_ros_edge", True)),
        "fanout_identical": bool(seg.get("fanout_identical", False)),
        "input_hz": float(seg.get("input_hz", 0) or 0),
        "clipseg_batch_n": int(seg.get("clipseg_batch_n", 1)),
        "rgb_sync_slop_s": float(seg.get("rgb_sync_slop_s", 0.05) or 0.05),
        "rgb_sync_mode": str(seg.get("rgb_sync_mode", "latest") or "latest"),
        "rgb_sync_diag": bool(seg.get("rgb_sync_diag", True)),
        "rgb_sync_diag_every_s": float(seg.get("rgb_sync_diag_every_s", 5.0) or 5.0),
        "e2e_log": bool(seg.get("e2e_log", True)),
        "e2e_log_every_s": float(seg.get("e2e_log_every_s", 60.0) or 60.0),
        "labels_topic": str(seg.get("labels_topic", "/segmentation/labels")),
        "viz_overlay": bool(seg.get("viz_overlay", False)),
        "overlay_topic": str(seg.get("overlay_topic", "/segmentation/overlay")),
        "overlay_alpha": float(seg.get("overlay_alpha", 0.5)),
        "profile": bool(seg.get("profile", False)),
        "profile_every": int(seg.get("profile_every", 30)),
        "pipeline_queue_size": queue_size,
        "use_cuda_streams": bool(seg.get("use_cuda_streams", True)),
        "publish_coarse_traversability": bool(
            seg.get("publish_coarse_traversability", True)
        ),
        "coarse_traversability_topic": str(
            seg.get(
                "coarse_traversability_topic",
                "/segmentation/coarse_traversability",
            )
        ),
        "coarse_people_mask_topic": str(
            seg.get(
                "coarse_people_mask_topic",
                "/segmentation/coarse_people_mask",
            )
        ),
        "sam_node": bool(seg.get("sam_node", False)),
        "sam_min_area": float(seg.get("sam_min_area", 0.002)),
        "sam_max_boxes": int(seg.get("sam_max_boxes", 6)),
        "sam_max_boxes_manip": int(
            seg.get("sam_max_boxes_manip", seg.get("sam_max_boxes", 6))
        ),
        "positive_groups": pos_idx or [0],
        "negative_groups": neg_idx or [1],
        "people_groups": people_idx or [2],
        "traversability_colors": dict(seg.get("traversability_colors") or {}),
        "traversability_pack_threshold": float(
            seg.get("traversability_pack_threshold", seg.get("threshold", 0.3))
        ),
        "sam_image_encoder": str(
            seg.get(
                "sam_image_encoder",
                "/root/colcon_ws/src/perception_models/data/resnet18_image_encoder_512.engine",
            )
        ),
        "sam_mask_decoder": str(
            seg.get(
                "sam_mask_decoder",
                "/root/colcon_ws/src/perception_models/data/mobile_sam_mask_decoder_batched_512.engine",
            )
        ),
        "sam_image_size": int(seg.get("sam_image_size", 512)),
        "sam_traversability_topic": str(
            seg.get("sam_traversability_topic", "/segmentation/refined_traversability")
        ),
        "sam_people_mask_topic": str(
            seg.get("sam_people_mask_topic", "/segmentation/refined_people_mask")
        ),
        "sam_overlay": bool(seg.get("sam_overlay", False)),
        "sam_overlay_topic": str(
            seg.get("sam_overlay_topic", "/segmentation/refined_overlay")
        ),
        "yolo_owns_people": bool(seg.get("yolo_owns_people", False)),
        "yolo_detections_topic": str(
            seg.get("yolo_detections_topic", "/yolo_world/detections")
        ),
        "yolo_object_group_idx": seg.get("yolo_object_group_idx", None),
    }

    # YOLO owns people: do not invent a phantom CLIPSeg/SAM group index.
    # people_groups empty → pack_traversability skips people from SAM maps.
    if cfg["yolo_owns_people"] and not people_idx:
        cfg["people_groups"] = []

    cfg_tf = tempfile.NamedTemporaryFile(
        mode="w", delete=False, prefix="hound_seg_", suffix=".json"
    )
    json.dump(cfg, cfg_tf, indent=2)
    cfg_tf.close()
    cfg_path = cfg_tf.name

    share = get_package_share_directory("perception_models")
    prefix = get_package_prefix("perception_models")
    lib = os.path.join(prefix, "lib", "perception_models")
    # Prefer src tree when present (dev) so new nodes need no install sync.
    src_root = Path("/root/colcon_ws/src/perception_models")
    if not src_root.is_dir():
        src_root = Path("/home/hound/colcon_ws/src/perception_models")
    src_scripts = src_root / "scripts"
    src_df = src_root / "dora" / "segmentation_dataflow.yml"
    template = src_df if src_df.is_file() else (Path(share) / "dora" / "segmentation_dataflow.yml")
    text = template.read_text(encoding="utf-8")

    def _node_py(name: str) -> str:
        cand = src_scripts / name
        return str(cand) if cand.is_file() else os.path.join(lib, name)

    text = (
        text.replace("__SOURCE_PY__", _node_py("dora_rgb_source"))
        .replace("__ENCODER_PY__", _node_py("dora_clipseg_encoder"))
        .replace("__REFINE_PY__", _node_py("dora_seg_refine"))
        .replace("__VIZ_PY__", _node_py("dora_seg_viz"))
        .replace("__QUEUE__", str(queue_size))
    )
    df_tf = tempfile.NamedTemporaryFile(
        mode="w", delete=False, prefix="hound_seg_df_", suffix=".yml"
    )
    df_tf.write(text)
    df_tf.close()
    dataflow_path = df_tf.name

    print(
        f"[hound_core] segmentation ENABLED (dora streams): model={cfg['model_name']} "
        f"res={cfg['input_res']} "
        f"vision={'trt' if cfg.get('clipseg_vision_engine') else 'torch'} "
        f"film={'trt' if cfg.get('clipseg_film_engine') else 'torch'} "
        f"groups={group_names} "
        f"({len(prompts_flat)} sub-prompts) sam={cfg['sam_node']} "
        f"coarse_trav={cfg['publish_coarse_traversability']} "
        f"streams={cfg['use_cuda_streams']} queue={queue_size} "
        f"rgb_sync={cfg.get('rgb_sync_mode', 'latest')}/"
        f"{cfg.get('rgb_sync_slop_s', 0.05)}s "
        f"dora_rgb_input={cfg.get('dora_rgb_input', True)}"
    )
    seg_env = {"HOUND_SEG_CONFIG": cfg_path}
    if src_root.is_dir():
        # Dev: import perception_models from src (new nodes before colcon install).
        prev = os.environ.get("PYTHONPATH", "")
        seg_env["PYTHONPATH"] = (
            f"{src_root}:{prev}" if prev else str(src_root)
        )
    return [
        ExecuteProcess(
            cmd=["dora", "run", dataflow_path],
            additional_env=seg_env,
            output="screen",
            name="hound_seg_dora",
        )
    ]


def _build_yolo_world_dora_actions(yw: dict) -> list:
    """Start dora YOLO-World detector (stashed class embeddings + per-frame detect)."""
    import json
    import os
    import tempfile

    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    classes = [str(c) for c in (yw.get("classes") or ["person"])]
    if bool(yw.get("ensure_person", True)) and "person" not in classes:
        classes = ["person"] + classes

    cfg = {
        "model": str(yw.get("model", "yolov8s-world.pt")),
        "classes": classes,
        "ensure_person": bool(yw.get("ensure_person", True)),
        "conf": float(yw.get("conf", 0.25)),
        "imgsz": int(yw.get("imgsz", 640)),
        "half": bool(yw.get("half", True)),
        "device": str(yw.get("device", "cuda:0")),
        "color_topic": str(yw.get("color_topic", "/camera/color/image_raw")),
        "detections_topic": str(yw.get("detections_topic", "/yolo_world/detections")),
        "viz_overlay": bool(yw.get("viz_overlay", False)),
        "overlay_topic": str(yw.get("overlay_topic", "/yolo_world/overlay")),
        "yolo_world_engine": str(yw.get("yolo_world_engine", "") or ""),
        "profile": bool(yw.get("profile", True)),
        "profile_every": int(yw.get("profile_every", 30)),
    }

    cfg_tf = tempfile.NamedTemporaryFile(
        mode="w", delete=False, prefix="hound_yolo_", suffix=".json"
    )
    json.dump(cfg, cfg_tf, indent=2)
    cfg_tf.close()

    share = get_package_share_directory("perception_models")
    prefix = get_package_prefix("perception_models")
    lib = os.path.join(prefix, "lib", "perception_models")
    template = Path(share) / "dora" / "yolo_world_dataflow.yml"
    text = template.read_text(encoding="utf-8").replace(
        "__YOLO_PY__", os.path.join(lib, "dora_yolo_world")
    )
    df_tf = tempfile.NamedTemporaryFile(
        mode="w", delete=False, prefix="hound_yolo_df_", suffix=".yml"
    )
    df_tf.write(text)
    df_tf.close()

    print(
        f"[hound_core] yolo_world ENABLED (dora): model={cfg['model']} "
        f"classes={cfg['classes']} imgsz={cfg['imgsz']} half={cfg['half']}"
    )
    return [
        ExecuteProcess(
            cmd=["dora", "run", df_tf.name],
            additional_env={"HOUND_YOLO_CONFIG": cfg_tf.name},
            output="screen",
            name="hound_yolo_dora",
        )
    ]


def _iter_ssot_cameras(sc: dict) -> list[tuple[str, dict]]:
    """Normalize ``cameras`` to ``[(key, cam_dict), ...]`` (dict or legacy list)."""
    raw = sc.get("cameras")
    if isinstance(raw, dict):
        out = []
        for key, cam in raw.items():
            if not isinstance(cam, dict):
                continue
            out.append((str(key), cam))
        return out
    if isinstance(raw, list):
        out = []
        for cam in raw:
            if not isinstance(cam, dict):
                continue
            name = str(cam.get("camera_name") or cam.get("name") or "").strip()
            if not name:
                continue
            out.append((name, cam))
        return out
    return []


def _camera_extrinsic_tf(
    cam: dict,
    *,
    camera_name: str,
    default_parent: str,
    skip_child: str | None = None,
    body_frame: str | None = None,
) -> Node | None:
    """Static TF parent → ``{camera_name}_link`` from cam xyz (m) / rpy (deg)."""
    if cam.get("xyz") is None and cam.get("rpy") is None:
        return None
    child = str(cam.get("link_frame") or f"{camera_name}_link")
    if skip_child and child == skip_child:
        print(
            f"[hound_core] cam TF skip {child} (VSLAM publishes this odom TF)"
        )
        return None
    xyz = cam.get("xyz") or [0.0, 0.0, 0.0]
    rpy_deg = cam.get("rpy") or [0.0, 0.0, 0.0]
    rpy = [math.radians(float(v)) for v in rpy_deg]
    parent = str(cam.get("parent_frame") or body_frame or default_parent)
    safe = camera_name.replace("/", "_")
    print(
        f"[hound_core] cam TF {parent} → {child} "
        f"xyz={list(xyz)} rpy_deg={list(rpy_deg)}"
    )
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{safe}_extrinsic_tf",
        output="screen",
        arguments=[
            "--x",
            str(float(xyz[0])),
            "--y",
            str(float(xyz[1])),
            "--z",
            str(float(xyz[2])),
            "--roll",
            str(rpy[0]),
            "--pitch",
            str(rpy[1]),
            "--yaw",
            str(rpy[2]),
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
    )


def _cam_node_params(
    sc: dict,
    cam: dict,
    *,
    need_color: bool,
    need_depth: bool,
    camera_backend: str,
) -> dict:
    """Merge top-level stereo_composite defaults with one cameras.* entry."""
    serial = str(
        cam.get("serial_number") or cam.get("serial") or sc.get("serial_number") or ""
    ).strip()
    camera_name = str(
        cam.get("camera_name") or cam.get("name") or sc.get("camera_name") or "camera"
    ).strip()
    infra_fps = int(cam.get("infra_fps", sc.get("infra_fps", 60)))
    enable_infra = infra_fps > 0
    vslam_type = int(cam.get("vslam_type", 0 if not enable_infra else 2))
    enable_vslam = bool(
        cam.get(
            "vslam_enabled",
            cam.get("enable_vslam", vslam_type > 0 and camera_backend != "dataset"),
        )
    )
    if enable_vslam and not enable_infra:
        print(
            f"[hound_core] {camera_name}: vslam_enabled with infra_fps<=0 — "
            "forcing vslam off (IR required)"
        )
        enable_vslam = False

    # Color/depth on unless explicitly disabled on the cam entry.
    enable_color = bool(cam.get("enable_color", True)) or need_color
    enable_depth = bool(cam.get("enable_depth", True if not enable_vslam else sc.get("enable_depth", True)))
    if need_depth:
        enable_depth = True
    align_depth = bool(cam.get("align_depth", sc.get("align_depth", False)))
    if enable_depth and align_depth:
        enable_color = True

    infra_w = int(cam.get("infra_width", sc.get("infra_width", 640)))
    infra_h = int(cam.get("infra_height", sc.get("infra_height", 360)))
    depth_w = int(cam.get("depth_width", sc.get("depth_width", infra_w)))
    depth_h = int(cam.get("depth_height", sc.get("depth_height", infra_h)))
    depth_fps = int(cam.get("depth_fps", sc.get("depth_fps", 30)))
    # Shared depth module: when IR is open, node forces depth to match infra.
    if enable_infra and enable_depth:
        depth_w, depth_h, depth_fps = infra_w, infra_h, infra_fps

    warmup = int(
        cam.get(
            "warmup_frames",
            sc.get("warmup_frames", 0 if camera_backend == "dataset" else 60),
        )
    )
    body_frame = str(sc.get("base_frame", "base_link"))
    base_frame = str(
        cam.get("base_frame")
        or (body_frame if enable_vslam else f"{camera_name}_link")
    )
    xyz = cam.get("xyz") or [0.0, 0.0, 0.0]
    rpy_deg = cam.get("rpy") or [0.0, 0.0, 0.0]

    params = {
        "serial_number": serial,
        "camera_name": camera_name,
        "camera_backend": camera_backend,
        "dataset_path": str(sc.get("dataset", sc.get("dataset_path", ""))),
        "dataset_view": str(cam.get("dataset_view", sc.get("dataset_view", "front"))),
        "dataset_loop": bool(sc.get("replay_loop", sc.get("dataset_loop", True))),
        "dataset_rate_hz": float(
            sc.get("replay_rate_hz", sc.get("dataset_rate_hz", 30.0))
        ),
        "enable_vslam": enable_vslam,
        "odom_source": str(cam.get("odom_source", sc.get("odom_source", "vslam"))),
        "infra_width": infra_w,
        "infra_height": infra_h,
        "infra_fps": infra_fps,
        "enable_color": enable_color,
        "color_width": int(cam.get("color_width", sc.get("color_width", 640))),
        "color_height": int(cam.get("color_height", sc.get("color_height", 360))),
        "color_fps": int(cam.get("color_fps", sc.get("color_fps", 30))),
        "color_publish_fps": float(
            cam.get("color_publish_fps", sc.get("color_publish_fps", 10.0))
        ),
        "enable_depth": enable_depth,
        "align_depth": align_depth,
        "depth_width": depth_w,
        "depth_height": depth_h,
        "depth_fps": depth_fps,
        "depth_publish_fps": float(
            cam.get("depth_publish_fps", sc.get("depth_publish_fps", 10.0))
        ),
        "emitter_enabled": int(
            cam.get("emitter_enabled", sc.get("emitter_enabled", 0))
        ),
        "visual_preset": int(cam.get("visual_preset", sc.get("visual_preset", 3))),
        "clip_distance": float(cam.get("clip_distance", sc.get("clip_distance", 0.0))),
        "initial_reset": bool(
            cam.get("initial_reset", sc.get("initial_reset", False))
        ),
        "odom_topic": str(sc.get("odom_topic", "/visual_slam/tracking/odometry")),
        "odom_frame": str(sc.get("odom_frame", "odom")),
        "base_frame": base_frame,
        "publish_odom_tf": bool(sc.get("publish_odom_tf", False)),
        "async_sba": bool(sc.get("async_sba", True)),
        "slam_sync_mode": bool(sc.get("slam_sync_mode", False)),
        "warmup_frames": warmup,
        "log_cuvslam_timing": bool(sc.get("log_cuvslam_timing", False)),
        "log_sensor_rates": bool(sc.get("log_sensor_rates", True)),
        "sensor_rate_log_every_s": float(sc.get("sensor_rate_log_every_s", 5.0)),
        "profile": bool(cam.get("profile", sc.get("profile", False))),
    }
    # Front = color sync master (shm stamp); others phase-lock stride to front+period.
    master = str(sc.get("color_sync_master", "") or "").strip()
    if master and camera_name:
        # Must match composite_sensing::color_sync_shm_path()
        shm = "/hound_csync_" + "".join(
            (c if (c.isalnum() or c == "_") else "_") for c in master
        )
        if camera_name == master:
            params["color_sync_role"] = "master"
            params["color_sync_shm_name"] = shm
        else:
            params["color_sync_role"] = "follower"
            params["color_sync_shm_name"] = shm
    if enable_vslam:
        params["xyz.x"] = float(xyz[0])
        params["xyz.y"] = float(xyz[1])
        params["xyz.z"] = float(xyz[2])
        params["rpy.roll"] = float(rpy_deg[0])
        params["rpy.pitch"] = float(rpy_deg[1])
        params["rpy.yaw"] = float(rpy_deg[2])
    return params


def _build_stereo_composite_node_from_params(
    sc: dict,
    params: dict,
    *,
    node_name: str,
) -> Node:
    backend = str(params.get("camera_backend", "realsense"))
    arch = str(sc.get("architecture", "modular")).strip().lower()
    if backend == "dataset" and arch != "modular":
        print(
            "[hound_core] dataset backend requires architecture=modular; forcing modular"
        )
        arch = "modular"
    exe = (
        "stereo_composite_modular_node"
        if arch == "modular"
        else "stereo_composite_node"
    )
    ir = (
        f"{params['infra_width']}x{params['infra_height']}@{params['infra_fps']}"
        if int(params["infra_fps"]) > 0
        else "off"
    )
    print(
        f"[hound_core] stereo_composite ENABLED ({arch}): name={node_name} "
        f"cam={params['camera_name']} backend={backend} "
        f"serial={params['serial_number']} "
        f"vslam={params['enable_vslam']} "
        f"IR {ir} "
        f"color={params['enable_color']}@{params['color_fps']}Hz "
        f"pub={params['color_publish_fps']}Hz "
        f"color_sync={params.get('color_sync_role', 'standalone')} "
        f"depth={params['enable_depth']}@{params['depth_fps']}Hz "
        f"pub={params['depth_publish_fps']}Hz "
        f"align_depth={params['align_depth']} "
        f"odom={params['odom_topic']} exe={exe}"
    )
    return Node(
        package="composite_sensing",
        executable=exe,
        name=node_name,
        output="screen",
        parameters=[params],
    )


def _build_stereo_composite_node(
    sc: dict,
    *,
    need_color: bool,
    need_depth: bool,
    node_name: str = "stereo_composite_node",
) -> Node:
    """Single-node path (dataset / legacy flat SSoT without cameras map)."""
    backend = str(sc.get("backend", "realsense")).strip().lower()
    if backend == "dataset_replay":
        dataset = str(
            sc.get(
                "dataset",
                "/root/colcon_ws/src/hound_mapping/data/rail_sim/race-2_multicam",
            )
        )
        cams = _iter_ssot_cameras(sc)
        names = [
            str(c.get("camera_name") or c.get("name") or k) for k, c in cams
        ]
        print(
            f"[hound_core] stereo_composite dataset_replay: {dataset} "
            f"cameras={names or ['front', 'left', 'right']}"
        )
        return Node(
            package="composite_sensing",
            executable="multicam_rail_replay",
            name="multicam_rail_replay",
            output="screen",
            parameters=[
                {
                    "dataset": dataset,
                    "rate_hz": float(sc.get("replay_rate_hz", 10.0)),
                    "loop": bool(sc.get("replay_loop", True)),
                    "global_frame": str(sc.get("odom_frame", "odom")),
                    "base_frame": str(sc.get("base_frame", "base_link")),
                }
            ],
        )

    camera_backend = "dataset" if backend == "dataset" else "realsense"
    # Legacy flat keys → one synthetic cam entry.
    cam = {
        "camera_name": sc.get("camera_name", "camera"),
        "serial_number": sc.get("serial_number", ""),
        "vslam_enabled": sc.get("enable_vslam", camera_backend != "dataset"),
        "infra_width": sc.get("infra_width", 640),
        "infra_height": sc.get("infra_height", 360),
        "infra_fps": sc.get("infra_fps", 60),
        "enable_color": sc.get("enable_color", True),
        "enable_depth": sc.get("enable_depth", False),
        "color_width": sc.get("color_width", 640),
        "color_height": sc.get("color_height", 360),
        "color_fps": sc.get("color_fps", 30),
        "color_publish_fps": sc.get("color_publish_fps", 10.0),
        "depth_width": sc.get("depth_width", 640),
        "depth_height": sc.get("depth_height", 360),
        "depth_fps": sc.get("depth_fps", 30),
        "depth_publish_fps": sc.get("depth_publish_fps", 10.0),
        "align_depth": sc.get("align_depth", False),
        "emitter_enabled": sc.get("emitter_enabled", 0),
        "visual_preset": sc.get("visual_preset", 3),
        "clip_distance": sc.get("clip_distance", 0.0),
        "initial_reset": sc.get("initial_reset", False),
        "base_frame": sc.get("base_frame", "base_link"),
        "xyz": sc.get("xyz"),
        "rpy": sc.get("rpy"),
    }
    params = _cam_node_params(
        sc, cam, need_color=need_color, need_depth=need_depth, camera_backend=camera_backend
    )
    return _build_stereo_composite_node_from_params(sc, params, node_name=node_name)


def _build_stereo_composite_actions(
    sc: dict,
    *,
    need_color: bool,
    need_depth: bool,
) -> list:
    """One USB node (+ optional extrinsic TF) per ``cameras.*`` entry."""
    backend = str(sc.get("backend", "realsense")).strip().lower()
    if backend in ("dataset_replay", "dataset"):
        return [
            _build_stereo_composite_node(
                sc, need_color=need_color, need_depth=need_depth
            )
        ]

    cams = _iter_ssot_cameras(sc)
    if not cams:
        return [
            _build_stereo_composite_node(
                sc,
                need_color=need_color,
                need_depth=need_depth,
                node_name="stereo_composite_node",
            )
        ]

    camera_backend = "realsense"
    body_frame = str(sc.get("base_frame", "base_link"))
    publish_odom_tf = bool(sc.get("publish_odom_tf", False))
    vslam_link = None
    vslam_odom_child = None
    for _key, cam in cams:
        name = str(cam.get("camera_name") or cam.get("name") or _key).strip()
        infra_fps = int(cam.get("infra_fps", 60))
        vslam_type = int(cam.get("vslam_type", 0))
        enabled = bool(
            cam.get("vslam_enabled", cam.get("enable_vslam", vslam_type > 0))
        )
        if enabled and infra_fps > 0:
            vslam_link = f"{name}_link"
            vslam_odom_child = str(cam.get("base_frame") or body_frame)
            break

    parent = str(
        sc.get("parent_frame")
        or vslam_link
        or body_frame
    )

    actions = []
    for key, cam in cams:
        camera_name = str(cam.get("camera_name") or cam.get("name") or key).strip()
        if not str(cam.get("serial_number") or cam.get("serial") or "").strip():
            print(f"[hound_core] skip camera {key}: empty serial_number")
            continue
        params = _cam_node_params(
            sc,
            cam,
            need_color=need_color,
            need_depth=need_depth,
            camera_backend=camera_backend,
        )
        safe = camera_name.replace("/", "_")
        actions.append(
            _build_stereo_composite_node_from_params(
                sc, params, node_name=f"stereo_composite_{safe}"
            )
        )
        is_vslam_cam = vslam_link is not None and f"{camera_name}_link" == vslam_link
        tf = _camera_extrinsic_tf(
            cam,
            camera_name=camera_name,
            default_parent=parent,
            skip_child=(vslam_odom_child if publish_odom_tf else None),
            body_frame=(body_frame if is_vslam_cam else None),
        )
        if tf is not None:
            actions.append(tf)

    print(
        f"[hound_core] multicam: "
        f"{[str(c.get('camera_name') or k) for k, c in cams]} "
        f"({len(actions)} launch action(s))"
    )
    return actions


def _build_vesc_actions(vesc: dict) -> list:
    port = str(vesc.get("port", "/dev/ttyACM0"))
    respawn = bool(vesc.get("respawn", True))
    start_delay_s = float(vesc.get("start_delay_s", 0.0))
    wheel_odom = vesc.get("wheel_odom") or {}

    telemetry_hz = float(vesc.get("telemetry_hz", 50.0))
    nodes = [
        Node(
            package="vesc_driver",
            executable="vesc_driver_node",
            name="vesc_driver",
            output="screen",
            parameters=[{"port": port, "telemetry_hz": telemetry_hz}],
            remappings=[("sensors/core", "/sensors/core")],
            respawn=respawn,
        ),
    ]

    if bool(wheel_odom.get("enabled", False)):
        nodes.append(
            Node(
                package="hound_core",
                executable="wheel_odom_node",
                name="wheel_odom_node",
                output="screen",
                parameters=[{
                    "erpm_gain": float(wheel_odom.get("erpm_gain", 3166.6)),
                    "output_topic": str(
                        wheel_odom.get("output_topic", "/mavros/vision_pose/vis_odom")
                    ),
                    "min_publish_interval_s": float(
                        wheel_odom.get("min_publish_interval_s", 0.1)
                    ),
                }],
            )
        )

    if start_delay_s > 0.0:
        return [TimerAction(period=start_delay_s, actions=nodes)]
    return nodes


def _build_fcu_control_node(fc: dict, vesc: dict | None = None) -> Node:
    """In-process MAVLink router + EKF + pluggable LL (+ optional VESC / NTRIP)."""
    origin = fc.get("ext_nav_origin") or {}
    fcu_params = fc.get("fcu_params") or {}
    ll_cfg = fc.get("ll") or {}
    ll_controller = str(fc.get("ll_controller", "ackermann")).strip().lower()
    vesc = vesc or {}
    ntrip = fc.get("ntrip") or {}
    params = {
        "fcu_url": str(fc.get("fcu_url", "/dev/ttyACM1:921600")),
        "gcs_url": str(fc.get("gcs_url", "")),
        "ros_publish_hz": float(fc.get("ros_publish_hz", 50.0)),
        "gcs_block_stream_requests": bool(fc.get("gcs_block_stream_requests", True)),
        "gcs_throttle_hz": float(fc.get("gcs_throttle_hz", 10.0)),
        "gcs_throttle_msgids": list(fc.get("gcs_throttle_msgids", [27, 30, 31, 32, 33, 65])),
        "send_vision_to_fcu": bool(fc.get("send_vision_to_fcu", True)),
        "enable_ekf": bool(fc.get("enable_ekf", True)),
        "enable_ll": bool(fc.get("enable_ll", True)),
        "enable_baro": bool(fc.get("enable_baro", True)),
        "enable_mag": bool(fc.get("enable_mag", True)),
        "enable_gps": bool(fc.get("enable_gps", True)),
        "fuse_gps": bool(fc.get("fuse_gps", False)),
        "vision_odom_topic": str(
            fc.get("vision_odom_topic", "/visual_slam/tracking/odometry")
        ),
        "ekf_odom_topic": str(fc.get("ekf_odom_topic", "ekf/odometry")),
        "ekf_reset_topic": str(fc.get("ekf_reset_topic", "~/ekf_reset")),
        "publish_ekf_tf": bool(fc.get("publish_ekf_tf", True)),
        "ekf_odom_frame": str(fc.get("ekf_odom_frame", "odom")),
        "ekf_base_frame": str(fc.get("ekf_base_frame", "base_link")),
        "ekf_odom_hz": int(fc.get("ekf_odom_hz", 50)),
        "mag_max_hz": float(fc.get("mag_max_hz", 20.0)),
        "baro_max_hz": float(fc.get("baro_max_hz", 20.0)),
        "delays.gps_pos_ms": int((fc.get("delays_ms") or {}).get("gps_pos", 200)),
        "delays.gps_vel_ms": int((fc.get("delays_ms") or {}).get("gps_vel", 200)),
        "delays.vslam_pos_ms": int((fc.get("delays_ms") or {}).get("vslam_pos", 100)),
        "delays.vslam_vel_ms": int((fc.get("delays_ms") or {}).get("vslam_vel", 100)),
        "delays.vslam_yaw_ms": int((fc.get("delays_ms") or {}).get("vslam_yaw", 100)),
        "delays.icp_pos_ms": int((fc.get("delays_ms") or {}).get("icp_pos", 80)),
        "delays.icp_vel_ms": int((fc.get("delays_ms") or {}).get("icp_vel", 80)),
        "delays.icp_yaw_ms": int((fc.get("delays_ms") or {}).get("icp_yaw", 80)),
        "delays.baro_ms": int((fc.get("delays_ms") or {}).get("baro", 50)),
        "delays.mag_ms": int((fc.get("delays_ms") or {}).get("mag", 25)),
        "ext_nav_align": str(fc.get("ext_nav_align", "gps_compass")),
        "icp_origin_topic": str(
            fc.get("icp_origin_topic", "/localization/icp_origin")
        ),
        "ekf_cpu": int(fc.get("ekf_cpu", 2)),
        "ll_cpu": int(fc.get("ll_cpu", 3)),
        "system_id": int(fc.get("system_id", 255)),
        "component_id": int(fc.get("component_id", 191)),
        "target_system": int(fc.get("target_system", 1)),
        "target_component": int(fc.get("target_component", 1)),
        "ext_nav_origin.lat": float(origin.get("lat", 37.8715)),
        "ext_nav_origin.lon": float(origin.get("lon", -122.2730)),
        "ext_nav_origin.hgt": float(origin.get("hgt", 0.0)),
        "fcu_params.SR0_EXTRA1": int(fcu_params.get("SR0_EXTRA1", 200)),
        "fcu_params.SR0_RAW_SENS": int(fcu_params.get("SR0_RAW_SENS", 200)),
        "ll_controller": ll_controller,
        # Top-level SSoT vesc: block — in-process when fcu_control owns the stack.
        "vesc_enabled": bool(vesc.get("enabled", False)),
        "vesc_port": str(vesc.get("port", "/dev/ttyACM0")),
        "vesc_telemetry_hz": float(vesc.get("telemetry_hz", 200.0)),
        "ntrip_enabled": bool(ntrip.get("enabled", False)),
        "ntrip_server": str(ntrip.get("server", "")),
        "ntrip_user": str(ntrip.get("user", "")),
        "ntrip_password": str(ntrip.get("password", "")),
        "ntrip_mountpoint": str(ntrip.get("mountpoint", "")),
        "ntrip_gga": str(ntrip.get("gga", "bus")),
        "ntrip_static_gga": str(ntrip.get("static_gga", "")),
        "ntrip_gga_period_s": float(ntrip.get("gga_period_s", 10.0)),
        "ntrip_reconnect_s": float(ntrip.get("reconnect_s", 2.0)),
        "ntrip_rtcm_topic": str(ntrip.get("rtcm_topic", "~/rtcm")),
    }
    # Pass through the robot-specific ll.* block as ROS params. Schema depends
    # on ll_controller (ackermann vs holonomic); each controller declares what
    # it needs under the shared ``ll.*`` namespace.
    for key, value in ll_cfg.items():
        if isinstance(value, dict):
            for nested_key, nested_value in value.items():
                params[f"ll.{key}.{nested_key}"] = nested_value
        else:
            params[f"ll.{key}"] = value
    print(f"[hound_core] fcu_control ll_controller={ll_controller}")
    return Node(
        package="hound_core",
        executable="hound_fcu_control_modular_node",
        name="hound_fcu_control",
        output="screen",
        parameters=[params],
    )


def generate_launch_description():
    ssot_file = find_ssot()
    print(f"[hound_core] Using SSoT file: {ssot_file}")
    with open(ssot_file, "r", encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle)

    sc = dict(ssot.get("stereo_composite") or {})
    nvblox = ssot.get("nvblox", {})
    viz = ssot.get("viz", {})
    nav = ssot.get("nav", {})
    fcu = ssot.get("fcu_control", {})
    vesc = fcu.get("vesc") or {}
    hal = ssot.get("hal_monitor", {})
    bag_recorder = ssot.get("bag_recorder", {})
    mesh_pf = ssot.get("mesh_pf", {})
    seg = ssot.get("segmentation", {})
    yw = ssot.get("yolo_world", {})
    launch_cfg = ssot.get("launch", {})
    lidar = ssot.get("lidar", {})

    sc_enabled = bool(sc.get("enabled", False))
    stage_delay_s = float(launch_cfg.get("stage_delay_s", 5.0))
    nvblox_enabled = bool(nvblox.get("enabled", False))
    viz_enabled = bool(viz.get("enabled", False))
    nav_enabled = bool(nav.get("enabled", False))
    fcu_control_enabled = bool(fcu.get("enabled", False))
    # Nested under fcu_control: parent enabled is the master switch.
    vesc_enabled = fcu_control_enabled and bool(vesc.get("enabled", False))
    hal_enabled = bool(hal.get("enabled", False))
    bag_recorder_enabled = bool(bag_recorder.get("enabled", False))
    mesh_pf_enabled = bool(mesh_pf.get("enabled", False))
    seg_enabled = bool(seg.get("enabled", False))
    yolo_enabled = bool(yw.get("enabled", False))
    lidar_enabled = bool(lidar.get("enabled", False))

    # nvblox people mask needs depth aligned to color.
    if nvblox_enabled and bool(nvblox.get("use_people_mask", True)):
        if not bool(sc.get("align_depth", False)):
            print(
                "[hound_core] nvblox use_people_mask=true: forcing "
                "stereo_composite.align_depth=true (mask/depth must share color frame)"
            )
            sc = dict(sc)
            sc["align_depth"] = True
        if not seg_enabled:
            print(
                "[hound_core] nvblox use_people_mask=true but segmentation DISABLED "
                "-> people mask remaps will stall until SAM publishes"
            )

    need_color = (
        seg_enabled
        or yolo_enabled
        or (nvblox_enabled and bool(nvblox.get("use_color", True)))
    )
    need_depth = nvblox_enabled and bool(nvblox.get("use_depth", False))
    if need_depth is False and nvblox_enabled and "use_depth" not in nvblox:
        need_depth = not bool(nvblox.get("use_lidar", False))
    has_camera_source = sc_enabled

    # --- Stage builders (order matches bring-up sequence) --------------------
    # 1) HAL diagnostics + optional bag_recorder
    hal_acts = []
    if hal_enabled:
        fcu_hal = hal.get("fcu") or hal.get("mavros") or {}
        if fcu_control_enabled or not fcu_hal.get("monitor_enabled", True):
            # HAL camera defaults use camera_name / fps from stereo_composite.
            hal_cam = {
                "camera_name": sc.get("camera_name", "camera"),
                "fps": float(sc.get("color_publish_fps", sc.get("color_fps", 15.0))),
            }
            # Prefer front camera if multi-cam stereo_composite.
            cams = sc.get("cameras") or {}
            if isinstance(cams, dict) and cams:
                front = cams.get("camera_front") or next(iter(cams.values()))
                if isinstance(front, dict):
                    hal_cam["camera_name"] = str(
                        front.get("camera_name", "camera_front")
                    )
                    hal_cam["fps"] = float(
                        front.get(
                            "color_publish_fps",
                            front.get("color_fps", hal_cam["fps"]),
                        )
                    )
            # Drop legacy bag/RC/TF keys so undeclared params are not passed.
            hal_clean = {
                k: v
                for k, v in hal.items()
                if k
                not in (
                    "enabled",
                    "bagdir",
                    "record_topics_file",
                    "record_split_duration_min",
                    "record_rc_channel",
                    "record_on_threshold",
                    "record_off_threshold",
                    "record_topic",
                )
            }
            if isinstance(hal_clean.get("fcu"), dict):
                fcu_h = dict(hal_clean["fcu"])
                fcu_h.pop("channel_topic", None)
                fcu_h.pop("pose_topic", None)
                fcu_h.pop("base_frame", None)
                hal_clean["fcu"] = fcu_h
            if isinstance(hal_clean.get("camera"), dict):
                cam_h = dict(hal_clean["camera"])
                for drop in ("pos", "rot", "depth_frame", "depth_optical_frame"):
                    cam_h.pop(drop, None)
                hal_clean["camera"] = cam_h
            hal_acts = [build_hal_monitor_node(hal_clean, hal_cam)]
        else:
            print(
                "[hound_core] hal_monitor needs fcu_control or "
                "fcu.monitor_enabled=false"
            )
    else:
        print("[hound_core] hal_monitor DISABLED")

    bag_acts = []
    if bag_recorder_enabled:
        bag_acts = [build_bag_recorder_node(bag_recorder)]
    else:
        print("[hound_core] bag_recorder DISABLED")

    # stereo_composite first (VSLAM), then fcu_control (in-process EKF+LL+vesc)
    cam_acts = []
    if sc_enabled:
        cam_acts = _build_stereo_composite_actions(
            sc, need_color=need_color, need_depth=need_depth
        )
    else:
        print("[hound_core] stereo_composite DISABLED")

    fcu_acts = []
    if fcu_control_enabled:
        fcu_acts = [_build_fcu_control_node(fcu, vesc)]
        print("[hound_core] fcu_control ENABLED (mavlink+ekf+ll in-process)")
        if vesc_enabled:
            print(
                "[hound_core] fcu_control.vesc ENABLED in-process "
                "(no standalone vesc_driver stage)"
            )
    else:
        print("[hound_core] fcu_control DISABLED")
        if bool(vesc.get("enabled", False)):
            print(
                "[hound_core] fcu_control.vesc.enabled ignored "
                "(fcu_control.enabled is the master switch)"
            )

    # LiDAR — composite (SDK+deskew+TF) XOR legacy unitree_lidar_ros2
    lidar_acts = []
    if lidar_enabled:
        if lidar_uses_composite(lidar):
            if not bool((lidar.get("composite") or {}).get("enabled", False)):
                print(
                    f"[hound_core] lidar.backend={str(lidar.get('backend')).lower()!r}: "
                    "forcing lidar_mesh_composite"
                )
            lidar_acts = [build_lidar_mesh_composite_node(lidar)]
        else:
            backend = str(lidar.get("backend", "unitree")).lower()
            if backend == "unitree":
                lidar_acts = build_unitree_lidar_actions(lidar)
            else:
                print(
                    f"[hound_core] lidar.backend={backend!r} not supported — skipping"
                )
    else:
        print("[hound_core] lidar DISABLED")

    # 5b) Mesh PF (Embree) — subscribes to deskewed /livox/cloud
    mesh_pf_acts = []
    if mesh_pf_enabled:
        mesh_pf_acts = [build_mesh_pf_node(mesh_pf, lidar)]
    else:
        print("[hound_core] mesh_pf DISABLED")

    # 6) segmentation (dora: encoder -> FiLM/SAM refine)
    seg_acts = []
    if seg_enabled:
        replay_mode = bool(seg.get("replay_mode", False))
        if has_camera_source or replay_mode:
            seg_for_dora = dict(seg)
            if yolo_enabled:
                prompts = dict(seg_for_dora.get("prompts") or {})
                if isinstance(prompts, dict) and "people" in prompts:
                    prompts = {k: v for k, v in prompts.items() if k != "people"}
                    seg_for_dora["prompts"] = prompts
                seg_for_dora["yolo_owns_people"] = True
                seg_for_dora["yolo_detections_topic"] = str(
                    yw.get("detections_topic", "/yolo_world/detections")
                )
                print(
                    "[hound_core] yolo_world.on → CLIPSeg people prompts removed; "
                    "NanoSAM people from YOLO boxes"
                )
            seg_acts = _build_segmentation_dora_actions(seg_for_dora)
        else:
            print("[hound_core] segmentation skipped (no stereo_composite)")
    else:
        print("[hound_core] segmentation DISABLED")

    # 6b) YOLO-World
    yolo_acts = []
    if yolo_enabled:
        yw_replay = bool(yw.get("replay_mode", False))
        if has_camera_source or yw_replay:
            yolo_acts = _build_yolo_world_dora_actions(yw)
        else:
            print("[hound_core] yolo_world skipped (no stereo_composite)")
    else:
        print("[hound_core] yolo_world DISABLED")

    # hound_mapping (SSoT key: nvblox)
    nvblox_acts = []
    if nvblox_enabled:
        bag_replay = bool(nvblox.get("bag_replay_mode", False))
        if not has_camera_source and not bag_replay:
            print(
                "[hound_core] nvblox requested but stereo_composite DISABLED -> "
                "skipping (set nvblox.bag_replay_mode=true for bag-only)"
            )
        else:
            if bag_replay and not has_camera_source:
                print(
                    "[hound_core] nvblox bag_replay_mode: starting mapper without "
                    "stereo_composite (expect bag + --clock)"
                )
            nvblox_acts = [build_hound_mapping_node(nvblox, sc, seg, lidar)]
    else:
        print("[hound_core] nvblox DISABLED")

    # 9) nav (needs LocalMap + odom)
    nav_acts = []
    if nav_enabled:
        nav_acts = build_nav_dora_actions(nav)
    else:
        print("[hound_core] nav DISABLED")

    # 10) viz (subscribe-only; last so producers exist)
    viz_acts = []
    if viz_enabled:
        viz_acts = [build_viz_node(viz)]
    else:
        print("[hound_core] viz DISABLED")

    print(
        f"[hound_core] staggered bring-up: stage_delay_s={stage_delay_s} "
        "(disabled stages skipped, no delay consumed)"
    )
    stages = [
        ("hal", hal_acts),
        ("bag_recorder", bag_acts),
        ("stereo_composite", cam_acts),
        ("fcu_control", fcu_acts),
        ("lidar", lidar_acts),
        ("mesh_pf", mesh_pf_acts),
        ("segmentation", seg_acts),
        ("yolo_world", yolo_acts),
        ("nvblox", nvblox_acts),
        ("nav", nav_acts),
        ("viz", viz_acts),
    ]
    return LaunchDescription(_schedule_stages(stages, stage_delay_s))
