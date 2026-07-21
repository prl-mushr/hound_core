"""Mission / algorithm tier for HOUND (cuVSLAM, EKF, MAVROS, control, segmentation).

Bring-up is staggered (launch.stage_delay_s, default 5s between enabled stages):
  HAL → ManagedNitros → mavros → vesc → camera → lidar → segmentation →
  vslam → ekf → nvblox

When launch.assume_sensors_running is true, start sensors.launch.py first
(ZED NITROS container + HAL). RealSense uses assume_sensors_running: false.

ManagedNitros note: for RealSense there is no separate ManagedNitros process
(stock Intel driver is not a NITROS publisher; cuVSLAM's ManagedNitrosSubscriber
converts sensor_msgs). For ZED + perception.use_nitros, this stage starts the
sensors_container.

Usage:
  ros2 launch hound_core hound_core.launch.py
"""

import os
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hound_launch_common import (  # noqa: E402
    build_ekf_node,
    build_extnav_earth_align_node,
    build_hal_monitor_node,
    build_nvblox_node,
    build_vslam_composable,
    build_vslam_container,
    build_zed_sensors_container,
    build_zed_state_publisher,
    dump_temp_yaml,
    find_ssot,
    realsense_camera_params,
    sensors_container_target,
    zed_share_paths,
    zed_topic_prefix,
)

PACKAGE_NAME = "hound_core"


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


def _build_camera_node(cam: dict, force_color: bool = False) -> Node:
    camera_name = cam.get("camera_name", "camera")
    params = realsense_camera_params(cam, force_color=force_color)
    print(
        f"[hound_core] camera ENABLED: serial={params['serial_no']} "
        f"infra_profile={params['depth_module.infra_profile']} "
        f"imu={'on' if params['enable_gyro'] else 'off'} "
        f"color={'on' if params['enable_color'] else 'off'} qos=SENSOR_DATA"
    )
    return Node(
        name=camera_name,
        namespace="",
        package="realsense2_camera",
        executable="realsense2_camera_node",
        output="screen",
        parameters=[params],
    )


def _build_zed_launch(zed: dict) -> IncludeLaunchDescription:
    camera_model = str(zed.get("camera_model", "zed2i"))
    camera_name = str(zed.get("camera_name", "zed"))
    serial_number = str(zed.get("serial_number", "0"))
    override = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "config",
        "zed_vslam_sensor.yaml",
    )
    print(
        f"[hound_core] zed ENABLED (standalone): model={camera_model} "
        f"name={camera_name} serial={serial_number}"
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("zed_wrapper"),
                "launch",
                "zed_camera.launch.py",
            ])
        ]),
        launch_arguments={
            "camera_model": camera_model,
            "camera_name": camera_name,
            "serial_number": serial_number,
            "ros_params_override_path": override,
            "publish_tf": "false",
            "publish_map_tf": "false",
            "publish_imu_tf": "true",
        }.items(),
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


def _build_vslam_actions(
    cam: dict,
    vslam: dict,
    zed: dict,
    ssot: dict,
    *,
    nitros_container_running: bool,
    force_color: bool = False,
) -> list:
    backend = str(cam.get("backend", "realsense")).lower()
    use_nitros = str(
        ssot.get("perception", {}).get("use_nitros", True)
    ).lower() not in ("false", "0", "no")

    if backend == "realsense":
        # Camera is a prior stagger stage; VSLAM is its own process/container.
        print(
            "[hound_core] RealSense cuVSLAM in visual_slam_launch_container "
            "(camera already staged separately; SENSOR_DATA QoS)"
        )
        return [build_vslam_container(cam, vslam, zed=None)]

    if backend == "zed" and use_nitros:
        if not nitros_container_running:
            raise RuntimeError(
                "ZED + NITROS cuVSLAM requires sensors_container running. "
                "Start sensors.launch.py first, or set launch.assume_sensors_running: "
                "false to co-launch ZED + cuVSLAM in one shot."
            )
        target = sensors_container_target(ssot)
        vslam_node = build_vslam_composable(cam, vslam, zed, use_nitros=True)
        print(
            f"[hound_core] Loading cuVSLAM into {target} (NITROS zero-copy to ZED)"
        )
        load_action = LoadComposableNodes(
            target_container=target,
            composable_node_descriptions=[vslam_node],
        )
        start_delay_s = float(vslam.get("start_delay_s", 0.0))
        if start_delay_s > 0.0:
            load_action = TimerAction(period=start_delay_s, actions=[load_action])

        wait_script = os.path.join(
            get_package_share_directory(PACKAGE_NAME),
            "..",
            "..",
            "lib",
            PACKAGE_NAME,
            "wait_for_topic_hz.py",
        )
        wait_script = os.path.normpath(wait_script)
        if not os.path.isfile(wait_script):
            wait_script = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                "scripts",
                "wait_for_topic_hz.py",
            )

        if bool(vslam.get("wait_for_camera_ready", True)):
            prefix = zed_topic_prefix(zed)
            topic = f"{prefix}/left/gray/rect/image"
            min_hz = float(vslam.get("camera_min_fps", 24.0))
            stable_s = float(vslam.get("camera_stable_s", 5.0))
            timeout_s = float(vslam.get("camera_wait_timeout_s", 120.0))
            print(
                f"[hound_core] cuVSLAM gated on stable camera: {topic} "
                f">= {min_hz} Hz for {stable_s}s"
            )
            wait_process = ExecuteProcess(
                cmd=[
                    "python3",
                    wait_script,
                    topic,
                    "--min-hz",
                    str(min_hz),
                    "--stable-s",
                    str(stable_s),
                    "--timeout-s",
                    str(timeout_s),
                ],
                output="screen",
            )
            return [
                wait_process,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=wait_process,
                        on_exit=[load_action],
                    )
                ),
            ]

        return [load_action]

    return [build_vslam_container(cam, vslam, zed=zed if backend == "zed" else None)]


def generate_launch_description():
    ssot_file = find_ssot()
    print(f"[hound_core] Using SSoT file: {ssot_file}")
    with open(ssot_file, "r", encoding="utf-8") as handle:
        ssot = yaml.safe_load(handle)

    cam = ssot.get("camera", {})
    zed = ssot.get("zed", {})
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

    camera_backend = str(cam.get("backend", "realsense")).lower()
    camera_enabled = bool(cam.get("enabled", True))
    assume_sensors_running = bool(launch_cfg.get("assume_sensors_running", True))
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

    # nvblox needs color (+ depth); people mask needs SAM.
    if nvblox_enabled:
        cam = dict(cam)
        cam["enable_depth"] = True
        cam.setdefault("align_depth", True)
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

    use_nitros = str(
        ssot.get("perception", {}).get("use_nitros", True)
    ).lower() not in ("false", "0", "no")
    # True when ZED NITROS container is already up (sensors.launch) or we start it.
    nitros_container_running = assume_sensors_running
    zed_nitros_local = (
        not assume_sensors_running
        and camera_enabled
        and camera_backend == "zed"
        and use_nitros
    )
    need_color = seg_enabled or nvblox_enabled

    # --- Stage builders (order matches bring-up sequence) --------------------
    # 1) HAL
    hal_acts = []
    if assume_sensors_running:
        print("[hound_core] HAL expected from sensors.launch.py")
    elif hal_enabled:
        if mavros_enabled or not (hal.get("mavros") or {}).get("monitor_enabled", True):
            hal_acts = [
                build_hal_monitor_node(
                    hal, cam, zed if camera_backend == "zed" else None
                )
            ]
        else:
            print(
                "[hound_core] hal_monitor needs mavros or mavros.monitor_enabled=false"
            )
    else:
        print("[hound_core] hal_monitor DISABLED")

    # 2) ManagedNitros / ZED sensors container
    # RealSense: ManagedNitros lives inside cuVSLAM — no separate process.
    # ZED + use_nitros: this is the NITROS IPC container (camera driver included).
    nitros_acts = []
    if zed_nitros_local:
        paths = zed_share_paths(str(zed.get("camera_model", "zed2i")))
        nitros_acts = [
            build_zed_state_publisher(zed, paths),
            build_zed_sensors_container(zed),
        ]
        nitros_container_running = True
        print("[hound_core] ManagedNitros: launching ZED sensors_container")
    elif camera_backend == "realsense":
        print(
            "[hound_core] ManagedNitros: no separate launch for RealSense "
            "(ManagedNitrosSubscriber is inside cuVSLAM)"
        )
    elif assume_sensors_running and use_nitros and camera_backend == "zed":
        print("[hound_core] ManagedNitros: using existing sensors_container")
    else:
        print("[hound_core] ManagedNitros: skipped")

    # 3) mavros
    mav_acts = _build_mavros_actions(mav) if mavros_enabled else []
    if not mavros_enabled:
        print("[hound_core] mavros DISABLED")

    # 4) vesc (+ low-level control once telemetry path exists)
    vesc_acts = []
    if vesc_enabled and not assume_sensors_running:
        vesc_cfg = dict(vesc)
        vesc_cfg["start_delay_s"] = 0.0  # stagger owns timing
        vesc_acts = _build_vesc_actions(vesc_cfg)
    elif vesc_enabled:
        print("[hound_core] vesc already expected from sensors.launch.py")

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

    # 5) camera (RealSense standalone, or ZED if not already in NITROS stage)
    cam_acts = []
    if assume_sensors_running:
        print("[hound_core] camera expected from sensors.launch.py")
    elif not camera_enabled:
        print("[hound_core] camera DISABLED")
    elif zed_nitros_local:
        print("[hound_core] camera already in ManagedNitros/ZED container")
    elif camera_backend == "zed":
        cam_acts = [_build_zed_launch(zed)]
    elif camera_backend == "realsense":
        cam_acts = [_build_camera_node(cam, force_color=need_color)]
    else:
        print(f"[hound_core] unknown camera.backend={camera_backend!r}")

    # 6) LiDAR (placeholder)
    lidar_acts = []
    if lidar_enabled:
        print(
            "[hound_core] lidar.enabled=true but no driver wired yet — "
            "stage is a no-op placeholder"
        )
    else:
        print("[hound_core] lidar DISABLED (placeholder)")

    # 7) segmentation
    seg_acts = []
    if seg_enabled:
        replay_mode = bool(seg.get("replay_mode", False))
        if camera_enabled or replay_mode or assume_sensors_running:
            seg_acts = [_build_segmentation_node(seg)]
            if bool(seg.get("sam_node", False)):
                seg_acts.append(_build_sam_refine_node(seg))
        else:
            print("[hound_core] segmentation skipped (no color source)")
    else:
        print("[hound_core] segmentation DISABLED")

    # 8) vslam (separate from camera so stagger can settle the stream first)
    vslam_acts = []
    if vslam_enabled:
        if camera_enabled or assume_sensors_running or nitros_container_running:
            vslam_acts = _build_vslam_actions(
                cam,
                vslam,
                zed,
                ssot,
                nitros_container_running=nitros_container_running,
                force_color=need_color,
            )
        else:
            print("[hound_core] vslam requested but no camera source -> skipping")
    else:
        print("[hound_core] vslam DISABLED")

    # 9) ekf
    ekf_acts = []
    if ekf_enabled:
        if not vslam_enabled:
            print("[hound_core] ekf requested but vslam DISABLED -> skipping")
        else:
            if camera_backend == "zed":
                align_node = build_extnav_earth_align_node(ekf, zed)
                if align_node is not None:
                    ekf_acts.append(align_node)
            ekf_acts.append(build_ekf_node(ekf))
    else:
        print("[hound_core] ekf DISABLED")

    # 10) nvblox
    nvblox_acts = []
    if nvblox_enabled:
        if not camera_enabled and not assume_sensors_running:
            print("[hound_core] nvblox requested but no camera -> skipping")
        else:
            nvblox_acts = [build_nvblox_node(nvblox, cam, seg)]
    else:
        print("[hound_core] nvblox DISABLED")

    print(
        f"[hound_core] staggered bring-up: stage_delay_s={stage_delay_s} "
        "(disabled stages skipped, no delay consumed)"
    )
    stages = [
        ("hal", hal_acts),
        ("managed_nitros", nitros_acts),
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
