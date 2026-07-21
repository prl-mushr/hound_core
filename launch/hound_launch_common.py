"""Shared launch helpers for hound_core sensors + mission bring-up."""

from __future__ import annotations

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

PACKAGE_NAME = "hound_core"
SENSORS_CONTAINER_NAME = "sensors_container"


def flatten_params(data: dict, prefix: str = "") -> dict:
    flat = {}
    for key, value in data.items():
        name = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            flat.update(flatten_params(value, name))
        else:
            flat[name] = value
    return flat


def dump_temp_yaml(data: dict, prefix: str) -> str:
    tf = tempfile.NamedTemporaryFile(
        mode="w", delete=False, prefix=prefix, suffix=".yaml"
    )
    yaml.safe_dump(data, tf, default_flow_style=False)
    tf.close()
    return tf.name


def find_ssot() -> str:
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
    raise RuntimeError("Could not find SSoT.yaml. Looked in: " + ", ".join(candidates))


def zed_topic_prefix(zed: dict) -> str:
    camera_name = str(zed.get("camera_name", "zed"))
    node_name = str(zed.get("node_name", "zed_node"))
    return f"/{camera_name}/{node_name}"


def zed_urdf_xacro_path() -> str:
    rel = os.path.join("urdf", "zed_descr.urdf.xacro")
    for pkg in ("zed_description", "zed_wrapper"):
        try:
            candidate = os.path.join(get_package_share_directory(pkg), rel)
        except Exception:
            continue
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(f"Could not find {rel} in zed_description or zed_wrapper")


def zed_serial_number(zed: dict) -> int:
    raw = zed.get("serial_number", 0)
    try:
        return int(raw)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            f"zed.serial_number must be an integer (use 0 for first USB camera), got {raw!r}"
        ) from exc


def zed_share_paths(camera_model: str) -> dict:
    zed_share = get_package_share_directory("zed_wrapper")
    return {
        "common": os.path.join(zed_share, "config", "common_stereo.yaml"),
        "camera": os.path.join(zed_share, "config", f"{camera_model}.yaml"),
        "object_detection": os.path.join(zed_share, "config", "object_detection.yaml"),
        "custom_object_detection": os.path.join(
            zed_share, "config", "custom_object_detection.yaml"
        ),
        "xacro": zed_urdf_xacro_path(),
    }


def zed_sensor_override_path() -> str:
    return os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "config",
        "zed_vslam_sensor.yaml",
    )


def build_zed_state_publisher(zed: dict, paths: dict) -> Node:
    camera_name = str(zed.get("camera_name", "zed"))
    camera_model = str(zed.get("camera_model", "zed2i"))
    xacro_cmd = Command([
        "xacro ", paths["xacro"],
        " camera_name:=", camera_name,
        " camera_model:=", camera_model,
    ])
    return Node(
        package="robot_state_publisher",
        namespace=camera_name,
        executable="robot_state_publisher",
        name=f"{camera_name}_state_publisher",
        output="screen",
        parameters=[{"robot_description": xacro_cmd}],
        remappings=[("robot_description", f"{camera_name}_description")],
    )


def build_zed_composable(zed: dict, sensor_override: str) -> ComposableNode:
    camera_model = str(zed.get("camera_model", "zed2i"))
    camera_name = str(zed.get("camera_name", "zed"))
    node_name = str(zed.get("node_name", "zed_node"))
    serial_number = zed_serial_number(zed)
    paths = zed_share_paths(camera_model)

    node_parameters = [
        paths["common"],
        paths["camera"],
        paths["object_detection"],
        paths["custom_object_detection"],
        sensor_override,
        {
            "general.camera_name": camera_name,
            "general.camera_model": camera_model,
            "general.serial_number": serial_number,
            "pos_tracking.publish_tf": False,
            "pos_tracking.publish_map_tf": False,
            "sensors.publish_imu_tf": True,
            "debug.disable_nitros": False,
        },
    ]

    print(
        f"[hound_core] ZED composable: model={camera_model} ns={camera_name} "
        f"node={node_name} serial={serial_number} IPC=off NITROS=on"
    )

    return ComposableNode(
        package="zed_components",
        plugin="stereolabs::ZedCamera",
        namespace=camera_name,
        name=node_name,
        parameters=node_parameters,
        extra_arguments=[{"use_intra_process_comms": False}],
    )


def build_zed_sensors_container(
    zed: dict,
    *,
    container_name: str = SENSORS_CONTAINER_NAME,
) -> ComposableNodeContainer:
    zed_node = build_zed_composable(zed, zed_sensor_override_path())
    print(
        f"[hound_core] Starting {container_name} "
        "(component_container_mt: ZED sensor, NITROS enabled)"
    )
    return ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[zed_node],
        output="screen",
    )


def realsense_camera_params(cam: dict, *, force_color: bool = False) -> dict:
    """Parameters for realsense2_camera (node or composable)."""
    infra_profile = f"{cam['infra_width']},{cam['infra_height']},{cam['fps']}"
    enable_imu = bool(cam.get("enable_imu", False))
    enable_color = bool(cam.get("enable_color", False)) or force_color
    enable_depth = bool(cam.get("enable_depth", False))
    color_profile = (
        f"{cam.get('color_width', 640)},{cam.get('color_height', 480)},"
        f"{cam.get('color_fps', 30)}"
    )
    depth_profile = (
        f"{cam.get('depth_width', cam['infra_width'])},"
        f"{cam.get('depth_height', cam['infra_height'])},"
        f"{cam.get('depth_fps', cam['fps'])}"
    )
    # Align depth→color when both on so people mask (color frame) matches depth.
    align_depth = bool(cam.get("align_depth", enable_depth and enable_color))

    params = {
        "serial_no": str(cam["serial_number"]),
        "enable_infra1": True,
        "enable_infra2": True,
        "enable_color": enable_color,
        "enable_depth": enable_depth,
        "depth_module.emitter_enabled": int(cam.get("emitter_enabled", 0)),
        "depth_module.infra_profile": infra_profile,
        "depth_module.profile": depth_profile if enable_depth else infra_profile,
        "align_depth.enable": align_depth,
        "enable_gyro": enable_imu,
        "enable_accel": enable_imu,
        "gyro_fps": int(cam.get("gyro_fps", 200)),
        "accel_fps": int(cam.get("accel_fps", 200)),
        "unite_imu_method": 2,
        # Match cuVSLAM image_qos=SENSOR_DATA (BEST_EFFORT).
        "infra1_qos": "SENSOR_DATA",
        "infra2_qos": "SENSOR_DATA",
        "infra1_info_qos": "SYSTEM_DEFAULT",
        "infra2_info_qos": "SYSTEM_DEFAULT",
    }
    if enable_depth:
        params["depth_qos"] = "SENSOR_DATA"
        params["depth_info_qos"] = "SYSTEM_DEFAULT"
    if enable_color:
        params["rgb_camera.color_profile"] = color_profile
        params["color_qos"] = "SENSOR_DATA"
        params["color_info_qos"] = "SYSTEM_DEFAULT"
    if enable_imu:
        params["gyro_qos"] = "SENSOR_DATA"
        params["accel_qos"] = "SENSOR_DATA"
    if "enable_sync" in cam:
        params["enable_sync"] = bool(cam.get("enable_sync"))
    return params


def build_realsense_composable(cam: dict, *, force_color: bool = False) -> ComposableNode:
    camera_name = str(cam.get("camera_name", "camera"))
    params = realsense_camera_params(cam, force_color=force_color)
    print(
        f"[hound_core] RealSense composable: serial={params['serial_no']} "
        f"infra={params['depth_module.infra_profile']} "
        f"depth={'on ' + params['depth_module.profile'] if params['enable_depth'] else 'off'} "
        f"align={params.get('align_depth.enable', False)} "
        f"imu={'on' if params['enable_gyro'] else 'off'} "
        f"qos=SENSOR_DATA"
    )
    return ComposableNode(
        name=camera_name,
        namespace="",
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        parameters=[params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def build_vslam_composable(
    cam: dict, vslam: dict, zed: dict | None, *, use_nitros: bool
) -> ComposableNode:
    backend = str(cam.get("backend", "zed")).lower()

    if backend == "zed" and zed is not None:
        camera_name = str(zed.get("camera_name", "zed"))
        prefix = zed_topic_prefix(zed)
        enable_imu = bool(zed.get("enable_imu", True))
        base_frame = f"{camera_name}_camera_link"
        imu_frame = f"{camera_name}_imu_link"
        camera_optical_frames = [
            f"{camera_name}_left_camera_frame_optical",
            f"{camera_name}_right_camera_frame_optical",
        ]
        default_tracking = 1
    else:
        camera_name = str(cam.get("camera_name", "camera"))
        prefix = ""
        enable_imu = bool(cam.get("enable_imu", False))
        base_frame = f"{camera_name}_link"
        imu_frame = f"{camera_name}_gyro_optical_frame"
        # Relative remaps (match isaac_ros_visual_slam_realsense.launch.py).
        left_image = f"{camera_name}/infra1/image_rect_raw"
        right_image = f"{camera_name}/infra2/image_rect_raw"
        camera_optical_frames = [
            f"{camera_name}_infra1_optical_frame",
            f"{camera_name}_infra2_optical_frame",
        ]
        default_tracking = 1 if enable_imu else 0

    tracking_mode = int(vslam.get("tracking_mode", default_tracking))

    if backend == "zed" and zed is not None:
        if use_nitros:
            left_image = f"{prefix}/left/gray/rect/image"
            right_image = f"{prefix}/right/gray/rect/image"
            left_info = f"{prefix}/left/gray/rect/image/camera_info"
            right_info = f"{prefix}/right/gray/rect/image/camera_info"
            transport = "nitros"
        else:
            left_image = f"{prefix}/left/gray/rect/image"
            right_image = f"{prefix}/right/gray/rect/image"
            left_info = f"{left_image}/camera_info"
            right_info = f"{right_image}/camera_info"
            transport = "ros"
    else:
        # RealSense: camera_info is a sibling of the image topic.
        left_info = f"{camera_name}/infra1/camera_info"
        right_info = f"{camera_name}/infra2/camera_info"
        transport = "nitros-compat"  # ManagedNitrosSubscriber + SENSOR_DATA QoS

    vslam_remappings = [
        ("visual_slam/image_0", left_image),
        ("visual_slam/camera_info_0", left_info),
        ("visual_slam/image_1", right_image),
        ("visual_slam/camera_info_1", right_info),
    ]
    if enable_imu and tracking_mode == 1:
        if backend == "zed" and zed is not None:
            vslam_remappings.append(("visual_slam/imu", f"{prefix}/imu/data"))
        else:
            vslam_remappings.append(("visual_slam/imu", f"{camera_name}/imu"))

    print(
        f"[hound_core] cuVSLAM composable: transport={transport} "
        f"tracking_mode={tracking_mode} "
        f"imu={'on' if enable_imu and tracking_mode == 1 else 'off'} "
        f"image_qos=SENSOR_DATA"
    )

    return ComposableNode(
        name="visual_slam_node",
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        parameters=[{
            "enable_image_denoising": bool(vslam.get("enable_image_denoising", False)),
            "rectified_images": bool(vslam.get("rectified_images", True)),
            "num_cameras": len(camera_optical_frames),
            "tracking_mode": tracking_mode,
            "gyro_noise_density": float(vslam.get("gyro_noise_density", 0.000244)),
            "gyro_random_walk": float(vslam.get("gyro_random_walk", 0.000019393)),
            "accel_noise_density": float(vslam.get("accel_noise_density", 0.001862)),
            "accel_random_walk": float(vslam.get("accel_random_walk", 0.003)),
            "calibration_frequency": float(vslam.get("calibration_frequency", 200.0)),
            "image_jitter_threshold_ms": float(
                vslam.get("image_jitter_threshold_ms", 22.0)
            ),
            "publish_map_to_odom_tf": bool(vslam.get("publish_map_to_odom_tf", False)),
            "publish_odom_to_base_tf": bool(vslam.get("publish_odom_to_base_tf", False)),
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
            # Isaac default is SENSOR_DATA; set explicitly for RealSense BEST_EFFORT.
            "image_qos": str(vslam.get("image_qos", "SENSOR_DATA")),
            "imu_qos": str(vslam.get("imu_qos", "SENSOR_DATA")),
        }],
        remappings=vslam_remappings,
    )


def build_vslam_container(cam: dict, vslam: dict, zed: dict | None = None) -> ComposableNodeContainer:
    """cuVSLAM-only container (ZED over standard ROS images)."""
    visual_slam_node = build_vslam_composable(cam, vslam, zed, use_nitros=False)
    return ComposableNodeContainer(
        name="visual_slam_launch_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[visual_slam_node],
        output="screen",
    )


def build_realsense_vslam_container(
    cam: dict, vslam: dict, *, force_color: bool = False
) -> ComposableNodeContainer:
    """RealSense + cuVSLAM in one MT container (NVIDIA-style co-location).

    Stock Intel realsense2_camera is not a NITROS publisher; ManagedNitrosSubscriber
    still converts sensor_msgs/Image. Co-location + SENSOR_DATA QoS matches the
    official isaac_ros_visual_slam_realsense pattern more closely than split processes.
    """
    camera_node = build_realsense_composable(cam, force_color=force_color)
    vslam_node = build_vslam_composable(cam, vslam, None, use_nitros=False)
    print(
        "[hound_core] RealSense+cuVSLAM co-located in visual_slam_launch_container "
        "(component_container_mt, SENSOR_DATA QoS)"
    )
    return ComposableNodeContainer(
        name="visual_slam_launch_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[camera_node, vslam_node],
        output="screen",
    )


def build_nvblox_node(nvblox: dict, cam: dict, seg: dict) -> Node:
    """nvblox for navigation ESDF: RealSense depth+color + SAM people mask.

    Traversability RGB is intentionally not remapped — use it in Nav2/planner.
    """
    camera_name = str(cam.get("camera_name", "camera"))
    voxel_size = float(nvblox.get("voxel_size", 0.05))
    use_people_mask = bool(nvblox.get("use_people_mask", True))
    global_frame = str(nvblox.get("global_frame", "odom"))
    map_clearing_frame = str(nvblox.get("map_clearing_frame_id", f"{camera_name}_link"))

    align = bool(cam.get("align_depth", bool(cam.get("enable_depth", False)) and bool(
        cam.get("enable_color", False))))
    if align:
        depth_image = f"/{camera_name}/aligned_depth_to_color/image_raw"
        depth_info = f"/{camera_name}/aligned_depth_to_color/camera_info"
    else:
        depth_image = f"/{camera_name}/depth/image_rect_raw"
        depth_info = f"/{camera_name}/depth/camera_info"

    color_image = f"/{camera_name}/color/image_raw"
    color_info = f"/{camera_name}/color/camera_info"
    people_mask = str(
        seg.get("sam_people_mask_topic", nvblox.get(
            "people_mask_topic", "/segmentation/refined_people_mask"))
    )

    params_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "config",
        "nvblox_hound.yaml",
    )
    overrides = {
        "voxel_size": voxel_size,
        "global_frame": global_frame,
        "map_clearing_frame_id": map_clearing_frame,
        "esdf_slice_bounds_visualization_attachment_frame_id": map_clearing_frame,
        "use_segmentation": use_people_mask,
        "mapping_type": (
            "human_with_static_tsdf" if use_people_mask else "static_tsdf"
        ),
        "input_qos": "SENSOR_DATA",
        "static_mapper": {
            "esdf_slice_height": float(nvblox.get("esdf_slice_height", 0.0)),
            "esdf_slice_min_height": float(nvblox.get("esdf_slice_min_height", -0.1)),
            "esdf_slice_max_height": float(nvblox.get("esdf_slice_max_height", 0.3)),
        },
        "dynamic_mapper": {
            "esdf_slice_height": float(nvblox.get("esdf_slice_height", 0.0)),
            "esdf_slice_min_height": float(nvblox.get("esdf_slice_min_height", -0.1)),
            "esdf_slice_max_height": float(nvblox.get("esdf_slice_max_height", 0.3)),
        },
    }

    remappings = [
        ("camera_0/depth/image", depth_image),
        ("camera_0/depth/camera_info", depth_info),
        ("camera_0/color/image", color_image),
        ("camera_0/color/camera_info", color_info),
    ]
    if use_people_mask:
        remappings.extend([
            ("camera_0/mask/image", people_mask),
            ("camera_0/mask/camera_info", color_info),
        ])

    print(
        f"[hound_core] nvblox ENABLED: voxel_size={voxel_size} m "
        f"({int(voxel_size * 100)} cm) people_mask="
        f"{'on -> ' + people_mask if use_people_mask else 'off'} "
        f"depth={depth_image} global_frame={global_frame}"
    )
    print(
        "[hound_core] nvblox NOTE: traversability RGB is for Nav2/planner, "
        "not remapped into nvblox"
    )

    return Node(
        package="nvblox_ros",
        executable="nvblox_node",
        name="nvblox_node",
        output="screen",
        parameters=[params_file, overrides],
        remappings=remappings,
    )


def build_ekf_node(ekf: dict) -> Node:
    preset = str(ekf.get("preset", "zed_vslam"))
    params_file = os.path.join(
        get_package_share_directory("inertial_nav_ros2"),
        "config",
        f"ekf_{preset}.yaml",
    )
    if not os.path.exists(params_file):
        raise RuntimeError(f"EKF preset not found: {params_file}")
    print(f"[hound_core] ekf ENABLED: preset={preset} params={params_file}")
    return Node(
        package="inertial_nav_ros2",
        executable="ekf_ins_node",
        name="nav_filter_ekf",
        output="screen",
        parameters=[params_file],
    )


def build_extnav_earth_align_node(ekf: dict, zed: dict | None = None) -> Node:
    align_cfg = ekf.get("extnav_earth_align", {})
    if not bool(align_cfg.get("enabled", True)):
        return None
    params_file = os.path.join(
        get_package_share_directory("inertial_nav_ros2"),
        "config",
        "extnav_earth_align.yaml",
    )
    overrides = {}
    if zed is not None:
        prefix = zed_topic_prefix(zed)
        overrides = {
            "imu_topic": f"{prefix}/imu/data",
            "mag_topic": f"{prefix}/imu/mag",
        }
    print("[hound_core] extnav_earth_align ENABLED (static earth->local from first VSLAM vs AHRS)")
    return Node(
        package="inertial_nav_ros2",
        executable="extnav_earth_align_node.py",
        name="extnav_earth_align",
        output="screen",
        parameters=[params_file, overrides] if overrides else [params_file],
    )


def apply_hal_camera_defaults(
    hal: dict, cam: dict, zed: dict | None = None
) -> dict:
    """Return HAL params with camera monitor topics/frames matching the backend."""
    params = flatten_params({k: v for k, v in hal.items() if k != "enabled"})
    backend = str(cam.get("backend", "realsense")).lower()

    if backend == "zed" and zed is not None:
        camera_name = str(zed.get("camera_name", "zed"))
        prefix = zed_topic_prefix(zed)
        params.setdefault("camera.monitor_topic", f"{prefix}/left/gray/rect/image")
        params.setdefault("camera.expected_fps", float(cam.get("fps", 30.0)))
        params.setdefault("camera.depth_frame", f"{camera_name}_camera_link")
        params.setdefault(
            "camera.depth_optical_frame",
            f"{camera_name}_left_camera_frame_optical",
        )
    else:
        camera_name = str(cam.get("camera_name", "camera"))
        params.setdefault("camera.depth_frame", f"{camera_name}_link")
        params.setdefault(
            "camera.depth_optical_frame",
            f"{camera_name}_infra1_optical_frame",
        )
        params.setdefault(
            "camera.monitor_topic",
            f"/{camera_name}/infra1/image_rect_raw",
        )
        params.setdefault("camera.expected_fps", float(cam.get("fps", 60.0)))

    mavros_cfg = hal.get("mavros") or {}
    params.setdefault(
        "mavros.monitor_enabled",
        bool(mavros_cfg.get("monitor_enabled", True)),
    )
    return params


def build_hal_monitor_node(hal: dict, cam: dict, zed: dict | None = None) -> Node:
    params = apply_hal_camera_defaults(hal, cam, zed)
    print(
        f"[hound_core] hal_monitor ENABLED: bagdir={hal.get('bagdir')} "
        f"camera_topic={params.get('camera.monitor_topic')}"
    )
    return Node(
        package="hound_core",
        executable="hal_monitor_node",
        name="HAL_9000",
        output="screen",
        parameters=[params],
    )


def sensors_container_target(ssot: dict) -> str:
    name = str(
        ssot.get("sensors", {}).get(
            "container_name",
            ssot.get("perception", {}).get(
                "sensors_container_name", SENSORS_CONTAINER_NAME
            ),
        )
    )
    return f"/{name}"
