"""Bring up the hound_core stack from a single source-of-truth config.

Three independently toggleable subsystems, all configured in config/SSoT.yaml:
  - camera : Intel RealSense D455 (by serial, 60 fps stereo IR + optional IMU)
  - vslam  : NVIDIA Isaac ROS Visual SLAM (only runs if the camera is enabled)
  - mavros : ArduPilot bring-up (wraps mavros' apm.launch)

The SSoT is read at launch time, so editing values does NOT require a rebuild
(the launch file prefers the copy in src/ via $ROS_WORKSPACE, falling back to
the installed share copy).
"""

import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "hound_core"


def _dump_temp_yaml(data: dict, prefix: str) -> str:
    """Write a dict to a temp YAML file and return its path."""
    tf = tempfile.NamedTemporaryFile(
        mode="w", delete=False, prefix=prefix, suffix=".yaml"
    )
    yaml.safe_dump(data, tf, default_flow_style=False)
    tf.close()
    return tf.name


def _find_ssot() -> str:
    """Locate SSoT.yaml, preferring the editable copy in the source tree."""
    candidates = []
    workspace_root = os.getenv("ROS_WORKSPACE")
    if workspace_root:
        candidates.append(
            os.path.join(workspace_root, "src", PACKAGE_NAME, "config", "SSoT.yaml")
        )
    candidates.append(
        os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "SSoT.yaml")
    )

    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate

    raise RuntimeError(
        "Could not find SSoT.yaml. Looked in: " + ", ".join(candidates)
    )


def _build_camera_node(cam: dict, force_color: bool = False) -> Node:
    """RealSense D455: stereo IR (+ optional IMU/color), depth off, emitter off."""
    camera_name = cam.get("camera_name", "camera")
    serial_no = str(cam["serial_number"])
    infra_profile = f"{cam['infra_width']},{cam['infra_height']},{cam['fps']}"
    enable_imu = bool(cam.get("enable_imu", False))
    enable_color = bool(cam.get("enable_color", False)) or force_color
    color_profile = f"{cam.get('color_width', 640)},{cam.get('color_height', 480)},{cam.get('color_fps', 30)}"

    print(
        f"[hound_core] camera ENABLED: serial={serial_no} "
        f"infra_profile={infra_profile} imu={'on' if enable_imu else 'off'} "
        f"color={'on ' + color_profile if enable_color else 'off'}"
    )

    params = {
        "serial_no": serial_no,
        "enable_infra1": True,
        "enable_infra2": True,
        "enable_color": enable_color,
        "enable_depth": False,
        "depth_module.emitter_enabled": int(cam.get("emitter_enabled", 0)),
        "depth_module.infra_profile": infra_profile,
        "depth_module.profile": infra_profile,  # backwards compatibility
        "enable_gyro": enable_imu,
        "enable_accel": enable_imu,
        "gyro_fps": int(cam.get("gyro_fps", 200)),
        "accel_fps": int(cam.get("accel_fps", 200)),
        "unite_imu_method": 2,
    }
    if enable_color:
        params["rgb_camera.color_profile"] = color_profile

    return Node(
        name=camera_name,
        namespace="",
        package="realsense2_camera",
        executable="realsense2_camera_node",
        output="screen",
        parameters=[params],
    )


def _flatten_prompts(prompts):
    """Turn the SSoT prompts into 3 parallel arrays the node can take as ROS
    params: flat sub-prompts, group names, and a group id per sub-prompt.
    Accepts a grouped dict {group: [subprompts]} or a flat list (each its own group)."""
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


def _build_segmentation_node(seg: dict) -> Node:
    """CLIPSeg segmentation node: publishes a label map (+ optional overlay)."""
    _prompts_flat, _group_names, _group_ids = _flatten_prompts(seg.get("prompts", ["a person"]))
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
    """Standalone SAM refinement node: refines CLIPSeg boxes into a traversability
    RGB (G=traversable, R=non-traversable, B=none) + a separate people mask."""
    # Box class ids are CLIPSeg group indices; resolve which map to each output.
    _, names, _ = _flatten_prompts(seg.get("prompts", ["a person"]))
    pos_names = [str(x) for x in seg.get("positive_groups", [])]
    neg_names = [str(x) for x in seg.get("negative_groups", [])]
    pos_idx = [i for i, n in enumerate(names) if n in pos_names]
    neg_idx = [i for i, n in enumerate(names) if n in neg_names]
    people_idx = [i for i, n in enumerate(names) if n not in pos_names and n not in neg_names]
    print(
        f"[hound_core] sam_refine_node ENABLED: backend={seg.get('sam_backend', 'nanosam')} "
        f"rate={seg.get('sam_rate_hz', 15.0)}Hz "
        f"(traversable={pos_idx} non_traversable={neg_idx} people={people_idx})"
    )
    return Node(
        package="hound_core",
        executable="sam_refine_node",
        name="sam_refine_node",
        output="screen",
        parameters=[{
            "color_topic": str(seg.get("color_topic", "/camera/color/image_raw")),
            "labels_topic": str(seg.get("labels_topic", "/segmentation/labels")),
            "traversability_topic": str(seg.get("sam_traversability_topic", "/segmentation/refined_traversability")),
            "people_mask_topic": str(seg.get("sam_people_mask_topic", "/segmentation/refined_people_mask")),
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


def _build_vslam_container(cam: dict, vslam: dict) -> ComposableNodeContainer:
    """Isaac ROS Visual SLAM composable node, wired to the camera's IR streams."""
    camera_name = cam.get("camera_name", "camera")
    enable_imu = bool(cam.get("enable_imu", False))

    # TF frames published by the realsense node are prefixed with the camera name.
    base_frame = f"{camera_name}_link"
    imu_frame = f"{camera_name}_gyro_optical_frame"
    camera_optical_frames = [
        f"{camera_name}_infra1_optical_frame",
        f"{camera_name}_infra2_optical_frame",
    ]

    vslam_remappings = [
        ("visual_slam/image_0", f"{camera_name}/infra1/image_rect_raw"),
        ("visual_slam/camera_info_0", f"{camera_name}/infra1/camera_info"),
        ("visual_slam/image_1", f"{camera_name}/infra2/image_rect_raw"),
        ("visual_slam/camera_info_1", f"{camera_name}/infra2/camera_info"),
    ]
    if enable_imu:
        vslam_remappings.append(("visual_slam/imu", f"{camera_name}/imu"))

    print(
        f"[hound_core] vslam ENABLED: tracking_mode={vslam.get('tracking_mode', 0)} "
        f"num_cameras={len(camera_optical_frames)}"
    )

    visual_slam_node = ComposableNode(
        name="visual_slam_node",
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        parameters=[{
            "enable_image_denoising": bool(vslam.get("enable_image_denoising", False)),
            "rectified_images": bool(vslam.get("rectified_images", True)),
            "num_cameras": len(camera_optical_frames),
            "tracking_mode": int(vslam.get("tracking_mode", 0)),
            "gyro_noise_density": float(vslam.get("gyro_noise_density", 0.000244)),
            "gyro_random_walk": float(vslam.get("gyro_random_walk", 0.000019393)),
            "accel_noise_density": float(vslam.get("accel_noise_density", 0.001862)),
            "accel_random_walk": float(vslam.get("accel_random_walk", 0.003)),
            "calibration_frequency": float(vslam.get("calibration_frequency", 200.0)),
            "image_jitter_threshold_ms": float(
                vslam.get("image_jitter_threshold_ms", 22.0)
            ),
            "base_frame": base_frame,
            "imu_frame": imu_frame,
            "enable_slam_visualization": bool(
                vslam.get("enable_slam_visualization", True)
            ),
            "enable_landmarks_view": bool(vslam.get("enable_landmarks_view", True)),
            "enable_observations_view": bool(
                vslam.get("enable_observations_view", True)
            ),
            "camera_optical_frames": camera_optical_frames,
        }],
        remappings=vslam_remappings,
    )

    return ComposableNodeContainer(
        name="visual_slam_launch_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[visual_slam_node],
        output="screen",
    )


def _build_mavros_actions(mav: dict) -> list:
    """Bring up mavros' apm.launch, with all params sourced from the SSoT.

    - node_params (ROS params for the mavros nodes, incl. the router rate-control)
      are written to a temp params file and passed to apm.launch.
    - fcu_params (ArduPilot board params) are written to a temp file and loaded
      onto the FCU after MAVROS connects (force-pull then `ros2 param load`).
    """
    fcu_url = str(mav.get("fcu_url", "/dev/ttyACM0:115200"))
    gcs_url = str(mav.get("gcs_url", ""))
    node_params = mav.get("node_params") or {}
    fcu_params = mav.get("fcu_params") or {}
    param_node = str(mav.get("param_node", "/mavros/param"))

    launch_arguments = {"fcu_url": fcu_url, "gcs_url": gcs_url}
    if node_params:
        params_file = _dump_temp_yaml(node_params, "hound_mavros_node_")
        launch_arguments["params_file"] = params_file
    else:
        params_file = None

    print(
        f"[hound_core] mavros ENABLED: fcu_url={fcu_url} "
        f"gcs_url={gcs_url or '(none)'} "
        f"node_params={'yes' if node_params else 'no'} "
        f"fcu_params={len(fcu_params)}"
    )

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

    # Push ArduPilot board params: wait for the param node, force-pull the FCU
    # param set, then load our overrides. Mirrors opendubs' wait-and-load flow.
    if fcu_params:
        fcu_file = _dump_temp_yaml(
            {"/**": {"ros__parameters": fcu_params}}, "hound_mavros_fcu_"
        )
        load_cmd = (
            'node="%s"; pf="%s"; '
            'echo "[hound_core] waiting for ${node} to load FCU params..."; '
            'until ros2 node list 2>/dev/null | grep -Fxq "${node}"; do sleep 1; done; '
            'ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull '
            '"{force_pull: true}" >/dev/null 2>&1 || true; '
            'sleep 3; '
            'echo "[hound_core] loading FCU params into ${node}"; '
            'ros2 param load "${node}" "${pf}"'
        ) % (param_node, fcu_file)
        actions.append(
            ExecuteProcess(cmd=["bash", "-c", load_cmd], output="screen")
        )

    return actions


def generate_launch_description():
    ssot_file = _find_ssot()
    print(f"[hound_core] Using SSoT file: {ssot_file}")
    with open(ssot_file, "r") as f:
        ssot = yaml.safe_load(f)

    cam = ssot.get("camera", {})
    vslam = ssot.get("vslam", {})
    mav = ssot.get("mavros", {})
    seg = ssot.get("segmentation", {})

    camera_enabled = bool(cam.get("enabled", True))
    vslam_enabled = bool(vslam.get("enabled", True))
    mavros_enabled = bool(mav.get("enabled", False))
    seg_enabled = bool(seg.get("enabled", False))

    actions = []

    # ---- Camera (color auto-enabled when segmentation needs it) --------------
    if camera_enabled:
        need_color = seg_enabled  # CLIPSeg runs on the color stream
        actions.append(_build_camera_node(cam, force_color=need_color))
    else:
        print("[hound_core] camera DISABLED")

    # ---- Visual SLAM (requires the camera for image input) -------------------
    if vslam_enabled:
        if camera_enabled:
            actions.append(_build_vslam_container(cam, vslam))
        else:
            print(
                "[hound_core] vslam requested but camera is DISABLED -> "
                "skipping VSLAM (no image source)"
            )
    else:
        print("[hound_core] vslam DISABLED")

    # ---- MAVROS / ArduPilot --------------------------------------------------
    if mavros_enabled:
        actions.extend(_build_mavros_actions(mav))
    else:
        print("[hound_core] mavros DISABLED")

    # ---- Segmentation (CLIPSeg) ----------------------------------------------
    # Normally needs the camera for color input, but replay_mode lets it run
    # standalone against an externally-published color topic (e.g. bag_replay).
    if seg_enabled:
        replay_mode = bool(seg.get("replay_mode", False))
        sam_node = bool(seg.get("sam_node", False))
        if camera_enabled or replay_mode:
            if replay_mode and not camera_enabled:
                print(
                    "[hound_core] segmentation REPLAY MODE: camera off, consuming "
                    f"external color topic '{seg.get('color_topic', '/camera/color/image_raw')}'"
                )
            # In split mode sam_refine_node consumes color + labels and refines locally.
            actions.append(_build_segmentation_node(seg))
            if sam_node:
                actions.append(_build_sam_refine_node(seg))
        else:
            print(
                "[hound_core] segmentation requested but camera is DISABLED and "
                "replay_mode is false -> skipping (no color source)"
            )
    else:
        print("[hound_core] segmentation DISABLED")

    return LaunchDescription(actions)
