"""Mission / algorithm tier for HOUND (composite sensing, EKF, control, seg).

Bring-up is staggered (launch.stage_delay_s, default 5s between enabled stages):
  HAL → mavros → vesc → stereo_composite → lidar → segmentation →
  yolo_world → ekf → nvblox

Camera + VSLAM come only from composite_sensing (stereo_composite).

Usage:
  ros2 launch hound_core hound_core.launch.py
"""

import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import (  # noqa: E402
    build_ekf_node,
    build_hal_monitor_node,
    build_lidar_mesh_composite_node,
    build_nvblox_node,
    build_hound_mapping_node,
    build_viz_node,
    build_nav_node,
    build_unitree_lidar_actions,
    dump_temp_yaml,
    find_ssot,
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
    """Start dora segmentation: clipseg_encoder -> seg_refine (CUDA streams).

    Inside seg_refine, NanoSAM runs FiLM decode and SAM encode on two CUDA
    streams so they can overlap; mask decode follows. Queued hops from encoder.
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
        "input_res": int(seg.get("input_res", 224)),
        "threshold": float(seg.get("threshold", 0.3)),
        "compile_mode": str(seg.get("compile_mode", "reduce-overhead")),
        "color_topic": str(seg.get("color_topic", "/camera/color/image_raw")),
        "color_topics": [str(t) for t in (seg.get("color_topics") or [])],
        "camera_names": [str(t) for t in (seg.get("camera_names") or [])],
        "clipseg_batch_n": int(seg.get("clipseg_batch_n", 1)),
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

    # If YOLO owns people and CLIPSeg has no people group, still reserve idx for SAM.
    if cfg["yolo_owns_people"] and not people_idx:
        people_idx = [max(len(group_names), 2)]
        cfg["people_groups"] = people_idx

    cfg_tf = tempfile.NamedTemporaryFile(
        mode="w", delete=False, prefix="hound_seg_", suffix=".json"
    )
    json.dump(cfg, cfg_tf, indent=2)
    cfg_tf.close()
    cfg_path = cfg_tf.name

    share = get_package_share_directory("perception_models")
    prefix = get_package_prefix("perception_models")
    lib = os.path.join(prefix, "lib", "perception_models")
    template = Path(share) / "dora" / "segmentation_dataflow.yml"
    text = template.read_text(encoding="utf-8")
    text = (
        text.replace("__ENCODER_PY__", os.path.join(lib, "dora_clipseg_encoder"))
        .replace("__REFINE_PY__", os.path.join(lib, "dora_seg_refine"))
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
        f"groups={group_names} "
        f"({len(prompts_flat)} sub-prompts) sam={cfg['sam_node']} "
        f"coarse_trav={cfg['publish_coarse_traversability']} "
        f"streams={cfg['use_cuda_streams']} queue={queue_size}"
    )
    return [
        ExecuteProcess(
            cmd=["dora", "run", dataflow_path],
            additional_env={"HOUND_SEG_CONFIG": cfg_path},
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


def _build_stereo_composite_node(
    sc: dict,
    *,
    need_color: bool,
    need_depth: bool,
) -> Node:
    """In-process stereo camera + cuVSLAM (composite_sensing).

    ``backend: dataset_replay`` → multicam_rail_replay (GT poses, no RealSense).
    """
    backend = str(sc.get("backend", "realsense")).strip().lower()
    if backend == "dataset_replay":
        dataset = str(
            sc.get(
                "dataset",
                "/root/colcon_ws/src/hound_mapping/data/rail_sim/race-2_multicam",
            )
        )
        cams = sc.get("cameras") or []
        names = [str(c.get("name", "")) for c in cams if c.get("name")]
        print(
            f"[hound_core] stereo_composite dataset_replay: {dataset} "
            f"cameras={names or ['front','left','right']}"
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

    enable_color = bool(sc.get("enable_color", True)) or need_color
    enable_depth = bool(sc.get("enable_depth", False))
    align_depth = bool(sc.get("align_depth", False))
    if need_depth and not enable_depth:
        print(
            "[hound_core] nvblox/stack wants depth but "
            "stereo_composite.enable_depth=false — not publishing depth"
        )
    if enable_depth and align_depth:
        enable_color = True
    infra_w = int(sc.get("infra_width", 640))
    infra_h = int(sc.get("infra_height", 360))
    infra_fps = int(sc.get("infra_fps", 60))
    params = {
        "serial_number": str(sc.get("serial_number", "")),
        "camera_name": str(sc.get("camera_name", "camera")),
        "infra_width": infra_w,
        "infra_height": infra_h,
        "infra_fps": infra_fps,
        "enable_color": enable_color,
        "color_width": int(sc.get("color_width", 640)),
        "color_height": int(sc.get("color_height", 360)),
        "color_fps": int(sc.get("color_fps", 30)),
        "color_publish_fps": float(sc.get("color_publish_fps", 15.0)),
        "enable_depth": enable_depth,
        "align_depth": align_depth,
        "depth_width": int(sc.get("depth_width", infra_w)),
        "depth_height": int(sc.get("depth_height", infra_h)),
        "depth_fps": infra_fps if enable_depth else int(sc.get("depth_fps", 30)),
        "depth_publish_fps": float(sc.get("depth_publish_fps", 15.0)),
        "emitter_enabled": int(sc.get("emitter_enabled", 0)),
        "visual_preset": int(sc.get("visual_preset", 3)),
        "clip_distance": float(sc.get("clip_distance", 0.0)),
        "odom_topic": str(sc.get("odom_topic", "/visual_slam/tracking/odometry")),
        "odom_frame": str(sc.get("odom_frame", "odom")),
        "base_frame": str(sc.get("base_frame", "camera_link")),
        "async_sba": bool(sc.get("async_sba", True)),
        "slam_sync_mode": bool(sc.get("slam_sync_mode", False)),
        "warmup_frames": int(sc.get("warmup_frames", 60)),
        "log_cuvslam_timing": bool(sc.get("log_cuvslam_timing", False)),
        "profile": bool(sc.get("profile", False)),
    }
    arch = str(sc.get("architecture", "modular")).strip().lower()
    exe = (
        "stereo_composite_modular_node"
        if arch == "modular"
        else "stereo_composite_node"
    )
    print(
        f"[hound_core] stereo_composite ENABLED ({arch}): serial={params['serial_number']} "
        f"IR {params['infra_width']}x{params['infra_height']}@{params['infra_fps']} "
        f"color={params['enable_color']}@{params['color_fps']}Hz pub={params['color_publish_fps']}Hz "
        f"depth={params['enable_depth']}@{params['depth_publish_fps']}Hz "
        f"align_depth={params['align_depth']} "
        f"visual_preset={params['visual_preset']} "
        f"odom={params['odom_topic']} exe={exe}"
    )
    return Node(
        package="composite_sensing",
        executable=exe,
        name="stereo_composite_node",
        output="screen",
        parameters=[params],
    )


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
    """In-process MAVLink router + EKF + pluggable LL (+ optional VESC)."""
    origin = fc.get("ext_nav_origin") or {}
    fcu_params = fc.get("fcu_params") or {}
    ll_cfg = fc.get("ll") or {}
    ll_controller = str(fc.get("ll_controller", "ackermann")).strip().lower()
    vesc = vesc or {}
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
        "ekf_odom_hz": int(fc.get("ekf_odom_hz", 50)),
        "mag_max_hz": float(fc.get("mag_max_hz", 20.0)),
        "baro_max_hz": float(fc.get("baro_max_hz", 20.0)),
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


def _build_mavros_actions(mav: dict) -> list:
    fcu_url = str(mav.get("fcu_url", "/dev/ttyACM0:115200"))
    gcs_url = str(mav.get("gcs_url", ""))
    node_params = mav.get("node_params") or {}
    fcu_params = mav.get("fcu_params") or {}
    param_node = str(mav.get("param_node", "/mavros/param"))

    launch_arguments = {"fcu_url": fcu_url, "gcs_url": gcs_url}
    if node_params:
        launch_arguments["params_file"] = dump_temp_yaml(node_params, "hound_mavros_node_")

    actions = [
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare("mavros"), "launch", "apm.launch"]
                )
            ]),
            launch_arguments=launch_arguments.items(),
        )
    ]

    if fcu_params:
        fcu_file = dump_temp_yaml(
            {"/**": {"ros__parameters": fcu_params}}, "hound_mavros_fcu_"
        )
        load_cmd = (
            'node="%s"; pf="%s"; '
            'until ros2 node list 2>/dev/null | grep -Fxq "${node}"; do sleep 1; done; '
            'ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull '
            '"{force_pull: true}" >/dev/null 2>&1 || true; '
            'sleep 3; '
            'ros2 param load "${node}" "${pf}"'
        ) % (param_node, fcu_file)
        actions.append(ExecuteProcess(cmd=["bash", "-c", load_cmd], output="screen"))

    return actions


def generate_launch_description():
    ssot_file = find_ssot()
    print(f"[hound_core] Using SSoT file: {ssot_file}")
    with open(ssot_file, "r", encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle)

    sc = dict(ssot.get("stereo_composite") or {})
    ekf = ssot.get("ekf", {})
    nvblox = ssot.get("nvblox", {})
    viz = ssot.get("viz", {})
    nav = ssot.get("nav", {})
    mav = ssot.get("mavros", {})
    fcu = ssot.get("fcu_control", {})
    vesc = ssot.get("vesc", {})
    hal = ssot.get("hal_monitor", {})
    seg = ssot.get("segmentation", {})
    yw = ssot.get("yolo_world", {})
    launch_cfg = ssot.get("launch", {})
    lidar = ssot.get("lidar", {})

    sc_enabled = bool(sc.get("enabled", False))
    stage_delay_s = float(launch_cfg.get("stage_delay_s", 5.0))
    ekf_enabled = bool(ekf.get("enabled", False))
    nvblox_enabled = bool(nvblox.get("enabled", False))
    viz_enabled = bool(viz.get("enabled", False))
    nav_enabled = bool(nav.get("enabled", False))
    mavros_enabled = bool(mav.get("enabled", False))
    fcu_control_enabled = bool(fcu.get("enabled", False))
    vesc_enabled = bool(vesc.get("enabled", False))
    hal_enabled = bool(hal.get("enabled", False))
    seg_enabled = bool(seg.get("enabled", False))
    yolo_enabled = bool(yw.get("enabled", False))
    lidar_enabled = bool(lidar.get("enabled", False))

    # In-process FCU owner: exclusive tty — never also start mavros / standalone ekf.
    if fcu_control_enabled:
        if mavros_enabled:
            print(
                "[hound_core] fcu_control.enabled: forcing mavros.enabled=false "
                "(exclusive FCU ownership)"
            )
            mavros_enabled = False
        if ekf_enabled and bool(fcu.get("enable_ekf", True)):
            print(
                "[hound_core] fcu_control.enabled: forcing ekf.enabled=false "
                "(in-process EKF)"
            )
            ekf_enabled = False

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
    # 1) HAL
    hal_acts = []
    if hal_enabled:
        fcu_hal = hal.get("fcu") or hal.get("mavros") or {}
        if (
            mavros_enabled
            or fcu_control_enabled
            or not fcu_hal.get("monitor_enabled", True)
        ):
            # HAL camera defaults use camera_name / fps from stereo_composite.
            hal_cam = {
                "camera_name": sc.get("camera_name", "camera"),
                "fps": float(sc.get("color_publish_fps", sc.get("color_fps", 15.0))),
            }
            hal_acts = [build_hal_monitor_node(hal, hal_cam)]
        else:
            print(
                "[hound_core] hal_monitor needs mavros/fcu_control or "
                "fcu.monitor_enabled=false"
            )
    else:
        print("[hound_core] hal_monitor DISABLED")

    # 2) FCU path: in-process fcu_control XOR classic mavros
    fcu_acts = []
    mav_acts = []
    if fcu_control_enabled:
        fcu_acts = [_build_fcu_control_node(fcu, vesc)]
        print("[hound_core] fcu_control ENABLED (mavlink+ekf+ll in-process)")
    elif mavros_enabled:
        mav_acts = _build_mavros_actions(mav)
    else:
        print("[hound_core] mavros DISABLED (fcu_control also off)")

    # 3) vesc — embed in fcu_control when both enabled (exclusive VESC tty);
    #    otherwise standalone vesc_driver for the classic path.
    vesc_acts = []
    if vesc_enabled:
        if fcu_control_enabled:
            print(
                "[hound_core] vesc ENABLED in-process (fcu_control); "
                "skipping standalone vesc_driver"
            )
            wheel_odom = vesc.get("wheel_odom") or {}
            if bool(wheel_odom.get("enabled", False)):
                vesc_acts.append(
                    Node(
                        package="hound_core",
                        executable="wheel_odom_node",
                        name="wheel_odom_node",
                        output="screen",
                        parameters=[{
                            "erpm_gain": float(wheel_odom.get("erpm_gain", 3166.6)),
                            "output_topic": str(
                                wheel_odom.get(
                                    "output_topic", "/mavros/vision_pose/vis_odom"
                                )
                            ),
                            "min_publish_interval_s": float(
                                wheel_odom.get("min_publish_interval_s", 0.1)
                            ),
                        }],
                    )
                )
        else:
            vesc_cfg = dict(vesc)
            vesc_cfg["start_delay_s"] = 0.0  # stagger owns timing
            vesc_acts = _build_vesc_actions(vesc_cfg)

    # 4) stereo_composite (composite_sensing)
    cam_acts = []
    if sc_enabled:
        cam_acts = [
            _build_stereo_composite_node(
                sc, need_color=need_color, need_depth=need_depth
            )
        ]
    else:
        print("[hound_core] stereo_composite DISABLED")

    # 5) LiDAR — composite (SDK+mesh PF) XOR legacy unitree_lidar_ros2
    lidar_acts = []
    if lidar_enabled:
        comp = lidar.get("composite") or {}
        if bool(comp.get("enabled", False)):
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

    # 7) ekf (pose from stereo_composite odom topic)
    ekf_acts = []
    if ekf_enabled:
        if not sc_enabled:
            print(
                "[hound_core] ekf requested but stereo_composite DISABLED -> skipping"
            )
        else:
            ekf_acts.append(build_ekf_node(ekf))
    else:
        print("[hound_core] ekf DISABLED")

    # 8) nvblox / hound_mapping
    nvblox_acts = []
    if nvblox_enabled:
        if not has_camera_source:
            print("[hound_core] nvblox requested but stereo_composite DISABLED -> skipping")
        else:
            backend = str(nvblox.get("backend", "hound")).strip().lower()
            if backend == "ros":
                nvblox_acts = [build_nvblox_node(nvblox, sc, seg, lidar)]
            else:
                if backend != "hound":
                    print(
                        f"[hound_core] nvblox.backend={backend!r} unknown; "
                        "using hound_mapping"
                    )
                nvblox_acts = [build_hound_mapping_node(nvblox, sc, seg, lidar)]
    else:
        print("[hound_core] nvblox DISABLED")

    # 9) nav (needs LocalMap + odom)
    nav_acts = []
    if nav_enabled:
        nav_acts = [build_nav_node(nav)]
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
        ("fcu_control", fcu_acts),
        ("mavros", mav_acts),
        ("vesc", vesc_acts),
        ("stereo_composite", cam_acts),
        ("lidar", lidar_acts),
        ("segmentation", seg_acts),
        ("yolo_world", yolo_acts),
        ("ekf", ekf_acts),
        ("nvblox", nvblox_acts),
        ("nav", nav_acts),
        ("viz", viz_acts),
    ]
    return LaunchDescription(_schedule_stages(stages, stage_delay_s))
