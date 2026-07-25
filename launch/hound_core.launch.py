"""Mission / algorithm tier for HOUND (cuVSLAM, EKF, MAVROS, control, segmentation).

Bring-up is staggered (launch.stage_delay_s, default 5s between enabled stages):
  HAL → mavros → vesc → camera|realsense_cuvslam → lidar → segmentation →
  vslam → ekf → nvblox

When realsense_cuvslam.enabled, the C++ node owns the D455 (IR+cuVSLAM in-process;
optional color/depth). Stock realsense2_camera and isaac_ros_visual_slam stages
are skipped so only one process holds the USB camera.

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
    build_nvblox_node,
    build_unitree_lidar_actions,
    build_vslam_container,
    dump_temp_yaml,
    find_ssot,
    realsense_camera_params,
    realsense_streams_for_stack,
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


def _build_camera_node(
    cam: dict,
    *,
    force_color: bool = False,
    streams: dict | None = None,
) -> Node:
    camera_name = cam.get("camera_name", "camera")
    params = realsense_camera_params(cam, force_color=force_color, streams=streams)
    print(
        f"[hound_core] camera ENABLED: serial={params['serial_no']} "
        f"infra={'on' if params['enable_infra1'] else 'off'} "
        f"profile={params['depth_module.infra_profile']} "
        f"depth={'on' if params['enable_depth'] else 'off'} "
        f"color={'on' if params['enable_color'] else 'off'} "
        f"sync={params.get('enable_sync', False)} qos=SENSOR_DATA"
    )
    return Node(
        name=camera_name,
        namespace="",
        package="realsense2_camera",
        executable="realsense2_camera_node",
        output="screen",
        parameters=[params],
    )


def _build_realsense_cuvslam_node(
    cam: dict,
    rsc: dict,
    *,
    need_color: bool,
    need_depth: bool,
) -> Node:
    """In-process RealSense + cuVSLAM (exclusive camera owner).

    Stream enable flags come from SSoT (``realsense_cuvslam`` / ``camera``).
    Launch does not override an explicit ``enable_depth: false``.
    """
    camera_name = cam.get("camera_name", "camera")
    enable_color = bool(rsc.get("enable_color", cam.get("enable_color", True))) or need_color
    enable_depth = bool(rsc.get("enable_depth", cam.get("enable_depth", False)))
    align_depth = bool(rsc.get("align_depth", cam.get("align_depth", False)))
    if need_depth and not enable_depth:
        print(
            "[hound_core] nvblox/stack wants depth but "
            "realsense_cuvslam.enable_depth=false — not publishing depth"
        )
    if enable_depth and align_depth:
        enable_color = True
    params = {
        "serial_number": str(cam.get("serial_number", "")),
        "camera_name": str(camera_name),
        "infra_width": int(cam.get("infra_width", 640)),
        "infra_height": int(cam.get("infra_height", 360)),
        "infra_fps": int(rsc.get("infra_fps", cam.get("fps", 60))),
        "enable_color": enable_color,
        "color_width": int(cam.get("color_width", 640)),
        "color_height": int(cam.get("color_height", 360)),
        "color_fps": int(rsc.get("color_fps", cam.get("color_fps", 30))),
        "color_publish_fps": float(rsc.get("color_publish_fps", 15.0)),
        "enable_depth": enable_depth,
        "align_depth": align_depth,
        "depth_width": int(cam.get("depth_width", cam.get("infra_width", 640))),
        "depth_height": int(cam.get("depth_height", cam.get("infra_height", 360))),
        # Depth module shares FPS with infra when both are enabled.
        "depth_fps": int(
            rsc.get("infra_fps", cam.get("fps", 30))
            if enable_depth
            else cam.get("depth_fps", cam.get("fps", 30))
        ),
        "depth_publish_fps": float(rsc.get("depth_publish_fps", 15.0)),
        "emitter_enabled": int(cam.get("emitter_enabled", 0)),
        # rsc override optional; else camera.visual_preset (rs400 enum).
        "visual_preset": int(rsc.get("visual_preset", cam.get("visual_preset", 3))),
        "clip_distance": float(rsc.get("clip_distance", cam.get("clip_distance", 0.0))),
        "odom_topic": str(rsc.get("odom_topic", "/visual_slam/tracking/odometry")),
        "odom_frame": str(rsc.get("odom_frame", "odom")),
        "base_frame": str(rsc.get("base_frame", "camera_link")),
        "async_sba": bool(rsc.get("async_sba", True)),
        "slam_sync_mode": bool(rsc.get("slam_sync_mode", False)),
        "warmup_frames": int(rsc.get("warmup_frames", 60)),
    }
    # People-mask path may have set cam["align_depth"]=True; honor when depth is on.
    if enable_depth:
        params["align_depth"] = bool(cam.get("align_depth", params["align_depth"]))
    print(
        f"[hound_core] realsense_cuvslam ENABLED: serial={params['serial_number']} "
        f"IR {params['infra_width']}x{params['infra_height']}@{params['infra_fps']} "
        f"color={params['enable_color']}@{params['color_fps']}Hz pub={params['color_publish_fps']}Hz "
        f"depth={params['enable_depth']}@{params['depth_publish_fps']}Hz "
        f"align_depth={params['align_depth']} "
        f"visual_preset={params['visual_preset']} "
        f"odom={params['odom_topic']}"
    )
    return Node(
        package="hound_core",
        executable="realsense_cuvslam_node",
        name="realsense_cuvslam_node",
        output="screen",
        parameters=[params],
    )


def _build_segmentation_node(seg: dict) -> Node:
    _prompts_flat, _group_names, _group_ids = _flatten_prompts(
        seg.get("prompts", ["a person"])
    )
    print(
        f"[hound_core] segmentation ENABLED: model={seg.get('model', 'CIDAS/clipseg-rd16')} "
        f"res={seg.get('input_res', 224)} rate={seg.get('rate_hz', 5.0)}Hz "
        f"groups={_group_names} ({len(_prompts_flat)} sub-prompts)"
    )
    return Node(
        package="hound_core",
        executable="clipseg_mask_node",
        name="clipseg_mask_node",
        output="screen",
        parameters=[{
            "model_name": str(seg.get("model", "CIDAS/clipseg-rd16")),
            "prompts": _prompts_flat,
            "group_names": _group_names,
            "group_ids": _group_ids,
            "suppress_prompts": [str(x) for x in seg.get("suppress_prompts", [""])] or [""],
            "aggregate": str(seg.get("aggregate", "max")),
            "input_res": int(seg.get("input_res", 224)),
            "rate_hz": float(seg.get("rate_hz", 5.0)),
            "threshold": float(seg.get("threshold", 0.3)),
            "compile_mode": str(seg.get("compile_mode", "reduce-overhead")),
            "color_topic": str(seg.get("color_topic", "/camera/color/image_raw")),
            "labels_topic": str(seg.get("labels_topic", "/segmentation/labels")),
            "viz_overlay": bool(seg.get("viz_overlay", True)),
            "overlay_topic": str(seg.get("overlay_topic", "/segmentation/overlay")),
            "overlay_alpha": float(seg.get("overlay_alpha", 0.5)),
            "profile": bool(seg.get("profile", False)),
            "profile_every": int(seg.get("profile_every", 30)),
        }],
    )


def _build_sam_refine_node(seg: dict) -> Node:
    _, names, _ = _flatten_prompts(seg.get("prompts", ["a person"]))
    pos_names = [str(x) for x in seg.get("positive_groups", [])]
    neg_names = [str(x) for x in seg.get("negative_groups", [])]
    pos_idx = [i for i, n in enumerate(names) if n in pos_names]
    neg_idx = [i for i, n in enumerate(names) if n in neg_names]
    people_idx = [i for i, n in enumerate(names) if n not in pos_names and n not in neg_names]
    return Node(
        package="hound_core",
        executable="sam_refine_node",
        name="sam_refine_node",
        output="screen",
        parameters=[{
            "color_topic": str(seg.get("color_topic", "/camera/color/image_raw")),
            "labels_topic": str(seg.get("labels_topic", "/segmentation/labels")),
            "traversability_topic": str(
                seg.get("sam_traversability_topic", "/segmentation/refined_traversability")
            ),
            "people_mask_topic": str(
                seg.get("sam_people_mask_topic", "/segmentation/refined_people_mask")
            ),
            "overlay_topic": str(seg.get("sam_overlay_topic", "/segmentation/refined_overlay")),
            "publish_overlay": bool(seg.get("sam_overlay", False)),
            "overlay_alpha": float(seg.get("overlay_alpha", 0.5)),
            "rate_hz": float(seg.get("sam_rate_hz", 15.0)),
            "sam_min_area": float(seg.get("sam_min_area", 0.002)),
            "sam_max_boxes": int(seg.get("sam_max_boxes", 6)),
            "positive_groups": pos_idx or [0],
            "negative_groups": neg_idx or [1],
            "people_groups": people_idx or [2],
            "sam_backend": str(seg.get("sam_backend", "nanosam")),
            "sam_image_encoder": str(
                seg.get(
                    "sam_image_encoder",
                    "/root/colcon_ws/src/nanosam/data/resnet18_image_encoder.engine",
                )
            ),
            "sam_mask_decoder": str(
                seg.get(
                    "sam_mask_decoder",
                    "/root/colcon_ws/src/nanosam/data/mobile_sam_mask_decoder.engine",
                )
            ),
            "sam_variant": str(seg.get("sam_variant", "l0")),
            "sam_ckpt": str(seg.get("sam_ckpt", "/root/colcon_ws/efficientvit_sam_l0.pt")),
            "sam_compile": bool(seg.get("sam_compile", True)),
            "sam_compile_mode": str(seg.get("sam_compile_mode", "reduce-overhead")),
        }],
    )


def _build_vesc_actions(vesc: dict) -> list:
    port = str(vesc.get("port", "/dev/ttyACM0"))
    respawn = bool(vesc.get("respawn", True))
    start_delay_s = float(vesc.get("start_delay_s", 0.0))
    wheel_odom = vesc.get("wheel_odom") or {}

    nodes = [
        Node(
            package="vesc_driver",
            executable="vesc_driver_node",
            name="vesc_driver",
            output="screen",
            parameters=[{"port": port}],
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


def _build_low_level_control_node(ll: dict) -> Node:
    params = {
        "executor_threads": int(ll.get("executor_threads", 8)),
        "erpm_gain": float(ll.get("erpm_gain", 3166.6)),
        "steering_max": float(ll.get("steering_max", 0.488)),
        "wheelbase": float(ll.get("wheelbase", 0.29)),
        "cg_height": float(ll.get("cg_height", 0.125)),
        "track_width": float(ll.get("track_width", 0.25)),
        "wheelspeed_max": float(ll.get("wheelspeed_max", 17.0)),
        "nominal_voltage": float(ll.get("nominal_voltage", 14.8)),
        "motor_kv": float(ll.get("motor_kv", 3930.0)),
        "speed_control_kp": float(ll.get("speed_control_kp", 1.0)),
        "speed_control_ki": float(ll.get("speed_control_ki", 1.0)),
        "safe_mode": bool(ll.get("safe_mode", True)),
        "accel_gain": float(ll.get("accel_gain", 1.0)),
        "roll_gain": float(ll.get("roll_gain", 0.33)),
        "steer_slack": float(ll.get("steer_slack", 0.2)),
        "LPF_tau": float(ll.get("LPF_tau", 0.5)),
        "throttle_delta": float(ll.get("throttle_delta", 0.02)),
        "liftoff_oversteer": bool(ll.get("liftoff_oversteer", True)),
        "control_dt": float(ll.get("control_dt", 0.02)),
    }
    return Node(
        package="hound_core",
        executable="hound_ll_control_node",
        name="low_level_controller",
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


def _build_vslam_actions(cam: dict, vslam: dict) -> list:
    print(
        "[hound_core] RealSense cuVSLAM in visual_slam_launch_container "
        "(camera already staged separately; SENSOR_DATA QoS)"
    )
    return [build_vslam_container(cam, vslam)]


def generate_launch_description():
    ssot_file = find_ssot()
    print(f"[hound_core] Using SSoT file: {ssot_file}")
    with open(ssot_file, "r", encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle)

    cam = ssot.get("camera", {})
    rsc = ssot.get("realsense_cuvslam", {})
    vslam = ssot.get("vslam", {})
    ekf = ssot.get("ekf", {})
    nvblox = ssot.get("nvblox", {})
    mav = ssot.get("mavros", {})
    vesc = ssot.get("vesc", {})
    ll = ssot.get("low_level_control", {})
    hal = ssot.get("hal_monitor", {})
    seg = ssot.get("segmentation", {})
    launch_cfg = ssot.get("launch", {})
    lidar = ssot.get("lidar", {})

    rsc_enabled = bool(rsc.get("enabled", False))
    camera_enabled = bool(cam.get("enabled", True))
    stage_delay_s = float(launch_cfg.get("stage_delay_s", 5.0))
    vslam_enabled = bool(vslam.get("enabled", True))
    ekf_enabled = bool(ekf.get("enabled", False))
    nvblox_enabled = bool(nvblox.get("enabled", False))
    mavros_enabled = bool(mav.get("enabled", False))
    vesc_enabled = bool(vesc.get("enabled", False))
    ll_control_enabled = bool(ll.get("enabled", False))
    hal_enabled = bool(hal.get("enabled", False))
    seg_enabled = bool(seg.get("enabled", False))
    lidar_enabled = bool(lidar.get("enabled", False))

    # C++ node owns USB exclusively — never also start stock camera / Isaac VSLAM.
    if rsc_enabled:
        if camera_enabled:
            print(
                "[hound_core] realsense_cuvslam.enabled: forcing camera.enabled=false "
                "(exclusive USB ownership)"
            )
            camera_enabled = False
        if vslam_enabled:
            print(
                "[hound_core] realsense_cuvslam.enabled: forcing vslam.enabled=false "
                "(in-process Tracker)"
            )
            vslam_enabled = False

    # nvblox: enable depth on stock RealSense path only. realsense_cuvslam
    # streams are controlled exclusively by realsense_cuvslam.* in SSoT.
    if nvblox_enabled:
        cam = dict(cam)
        if not rsc_enabled:
            cam["enable_depth"] = True
        if bool(nvblox.get("use_people_mask", True)):
            # Mask is in color pixels — depth must be aligned to color.
            if not bool(cam.get("align_depth", False)) and not bool(
                rsc.get("align_depth", False)
            ):
                print(
                    "[hound_core] nvblox use_people_mask=true: forcing "
                    "align_depth=true (mask/depth must share color frame)"
                )
            cam["align_depth"] = True
            if rsc_enabled:
                rsc = dict(rsc)
                rsc["align_depth"] = True
        if bool(nvblox.get("use_people_mask", True)) and not seg_enabled:
            print(
                "[hound_core] nvblox use_people_mask=true but segmentation DISABLED "
                "-> people mask remaps will stall until SAM publishes"
            )
        if not bool(vslam.get("publish_odom_to_base_tf", False)):
            vslam = dict(vslam)
            vslam["publish_odom_to_base_tf"] = True
            print(
                "[hound_core] nvblox: forcing vslam.publish_odom_to_base_tf=true "
                "(needs odom→camera_link TF)"
            )

    need_color = seg_enabled or (nvblox_enabled and bool(nvblox.get("use_color", True)))
    need_depth = nvblox_enabled and bool(nvblox.get("use_depth", False))
    if need_depth is False and nvblox_enabled and "use_depth" not in nvblox:
        # Legacy: if use_depth omitted, only require camera depth when no lidar.
        need_depth = not bool(nvblox.get("use_lidar", False))
    has_camera_source = camera_enabled or rsc_enabled
    rs_streams = realsense_streams_for_stack(
        cam,
        vslam_enabled=vslam_enabled,
        nvblox_enabled=nvblox_enabled,
        seg_enabled=seg_enabled,
    )
    if camera_enabled:
        print(
            f"[hound_core] RealSense streams: infra={rs_streams['enable_infra1']} "
            f"depth={rs_streams['enable_depth']} color={rs_streams['enable_color']} "
            f"(from enabled stack nodes)"
        )

    # --- Stage builders (order matches bring-up sequence) --------------------
    # 1) HAL
    hal_acts = []
    if hal_enabled:
        if mavros_enabled or not (hal.get("mavros") or {}).get("monitor_enabled", True):
            hal_acts = [build_hal_monitor_node(hal, cam)]
        else:
            print(
                "[hound_core] hal_monitor needs mavros or mavros.monitor_enabled=false"
            )
    else:
        print("[hound_core] hal_monitor DISABLED")

    # 2) mavros
    mav_acts = _build_mavros_actions(mav) if mavros_enabled else []
    if not mavros_enabled:
        print("[hound_core] mavros DISABLED")

    # 3) vesc (+ low-level control once telemetry path exists)
    vesc_acts = []
    if vesc_enabled:
        vesc_cfg = dict(vesc)
        vesc_cfg["start_delay_s"] = 0.0  # stagger owns timing
        vesc_acts = _build_vesc_actions(vesc_cfg)

    if ll_control_enabled:
        if mavros_enabled and vesc_enabled:
            vesc_acts.append(_build_low_level_control_node(ll))
        else:
            missing = [n for n, on in (("mavros", mavros_enabled), ("vesc", vesc_enabled)) if not on]
            print(
                "[hound_core] low_level_control needs "
                + " and ".join(missing)
                + " -> skipping"
            )

    # 4) camera source: stock RealSense OR in-process realsense_cuvslam
    cam_acts = []
    if rsc_enabled:
        cam_acts = [
            _build_realsense_cuvslam_node(
                cam, rsc, need_color=need_color, need_depth=need_depth
            )
        ]
    elif camera_enabled:
        cam_acts = [_build_camera_node(cam, force_color=need_color, streams=rs_streams)]
    else:
        print("[hound_core] camera DISABLED (no realsense_cuvslam either)")

    # 5) LiDAR (Unitree UniLidar)
    lidar_acts = []
    if lidar_enabled:
        backend = str(lidar.get("backend", "unitree")).lower()
        if backend == "unitree":
            lidar_acts = build_unitree_lidar_actions(lidar)
        else:
            print(f"[hound_core] lidar.backend={backend!r} not supported — skipping")
    else:
        print("[hound_core] lidar DISABLED")

    # 6) segmentation
    seg_acts = []
    if seg_enabled:
        replay_mode = bool(seg.get("replay_mode", False))
        if has_camera_source or replay_mode:
            seg_acts = [_build_segmentation_node(seg)]
            if bool(seg.get("sam_node", False)):
                seg_acts.append(_build_sam_refine_node(seg))
        else:
            print("[hound_core] segmentation skipped (no color source)")
    else:
        print("[hound_core] segmentation DISABLED")

    # 7) vslam (skipped when realsense_cuvslam owns tracking)
    vslam_acts = []
    if vslam_enabled:
        if camera_enabled:
            vslam_acts = _build_vslam_actions(cam, vslam)
        else:
            print("[hound_core] vslam requested but no camera source -> skipping")
    else:
        print("[hound_core] vslam DISABLED")

    # 8) ekf (pose from /visual_slam/tracking/odometry — C++ or Isaac)
    ekf_acts = []
    if ekf_enabled:
        if not (vslam_enabled or rsc_enabled):
            print(
                "[hound_core] ekf requested but no VSLAM source "
                "(vslam/realsense_cuvslam) -> skipping"
            )
        else:
            ekf_acts.append(build_ekf_node(ekf))
    else:
        print("[hound_core] ekf DISABLED")

    # 9) nvblox
    nvblox_acts = []
    if nvblox_enabled:
        if not has_camera_source:
            print("[hound_core] nvblox requested but no camera -> skipping")
        else:
            nvblox_acts = [build_nvblox_node(nvblox, cam, seg, lidar)]
    else:
        print("[hound_core] nvblox DISABLED")

    print(
        f"[hound_core] staggered bring-up: stage_delay_s={stage_delay_s} "
        "(disabled stages skipped, no delay consumed)"
    )
    stages = [
        ("hal", hal_acts),
        ("mavros", mav_acts),
        ("vesc", vesc_acts),
        ("camera", cam_acts),
        ("lidar", lidar_acts),
        ("segmentation", seg_acts),
        ("vslam", vslam_acts),
        ("ekf", ekf_acts),
        ("nvblox", nvblox_acts),
    ]
    return LaunchDescription(_schedule_stages(stages, stage_delay_s))
