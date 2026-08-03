"""Shared launch helpers for hound_core sensors + mission bring-up."""

from __future__ import annotations

import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

PACKAGE_NAME = "hound_core"


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


def realsense_streams_for_stack(
    cam: dict,
    *,
    vslam_enabled: bool,
    nvblox_enabled: bool,
    seg_enabled: bool,
) -> dict:
    """Pick RealSense streams from enabled consumers (avoids idle IR/depth load)."""
    # Explicit SSoT flags still win when no consumer needs a stream.
    need_infra = vslam_enabled
    need_depth = nvblox_enabled or bool(cam.get("enable_depth", False))
    need_color = (
        seg_enabled
        or nvblox_enabled
        or bool(cam.get("enable_color", False))
    )
    return {
        "enable_infra1": need_infra,
        "enable_infra2": need_infra,
        "enable_depth": need_depth,
        "enable_color": need_color,
    }


def realsense_camera_params(
    cam: dict,
    *,
    force_color: bool = False,
    streams: dict | None = None,
) -> dict:
    """Parameters for realsense2_camera (node or composable).

    D455 depth + IR share one depth module — same resolution and FPS.
    Color is a separate RGB sensor and may run at a different rate (e.g. 30 Hz).
    """
    stream_sel = streams or {}
    infra_w = int(cam["infra_width"])
    infra_h = int(cam["infra_height"])
    infra_fps = int(cam["fps"])
    enable_imu = bool(cam.get("enable_imu", False))
    enable_infra1 = bool(stream_sel.get("enable_infra1", True))
    enable_infra2 = bool(stream_sel.get("enable_infra2", True))
    enable_color = bool(stream_sel.get("enable_color", cam.get("enable_color", False)))
    enable_color = enable_color or force_color
    enable_depth = bool(stream_sel.get("enable_depth", cam.get("enable_depth", False)))
    color_fps = int(cam.get("color_fps", 30))
    color_profile = (
        f"{cam.get('color_width', 640)},{cam.get('color_height', 480)},"
        f"{color_fps}"
    )
    depth_w = int(cam.get("depth_width", infra_w))
    depth_h = int(cam.get("depth_height", infra_h))
    depth_fps = int(cam.get("depth_fps", cam["fps"]))

    # One profile for the shared depth module (IR streams + depth).
    need_depth_module = enable_depth or enable_infra1 or enable_infra2
    if need_depth_module:
        module_w, module_h = infra_w, infra_h
        module_fps = infra_fps
        if enable_depth:
            module_w, module_h = depth_w, depth_h
            module_fps = depth_fps
            if (enable_infra1 or enable_infra2) and (
                (infra_w, infra_h, infra_fps) != (depth_w, depth_h, depth_fps)
            ):
                print(
                    f"[hound_core] RealSense: IR {infra_w}x{infra_h}@{infra_fps} "
                    f"!= depth {depth_w}x{depth_h}@{depth_fps}; "
                    f"forcing shared depth-module profile "
                    f"{module_w}x{module_h}@{module_fps}"
                )
        module_profile = f"{module_w},{module_h},{module_fps}"
    else:
        module_profile = f"{infra_w},{infra_h},{infra_fps}"

    align_depth = bool(cam.get("align_depth", enable_depth and enable_color))

    # Inter-stream hardware sync needs matching rates; keep off for 60 vs 30.
    if "enable_sync" in cam:
        enable_sync = bool(cam.get("enable_sync"))
    else:
        enable_sync = False
    if enable_sync and enable_color and need_depth_module:
        module_fps = int(module_profile.split(",")[2])
        if color_fps != module_fps:
            print(
                f"[hound_core] RealSense: enable_sync=true but color_fps={color_fps} "
                f"!= depth-module fps={module_fps}; forcing enable_sync=false"
            )
            enable_sync = False

    params = {
        "serial_no": str(cam["serial_number"]),
        "enable_infra1": enable_infra1,
        "enable_infra2": enable_infra2,
        "enable_color": enable_color,
        "enable_depth": enable_depth,
        "depth_module.emitter_enabled": int(cam.get("emitter_enabled", 0)),
        # realsense2_camera ≥4.55: depth_profile (not legacy "profile").
        "depth_module.infra_profile": module_profile,
        "depth_module.depth_profile": module_profile,
        "align_depth.enable": align_depth,
        "enable_sync": enable_sync,
        "enable_gyro": enable_imu,
        "enable_accel": enable_imu,
        "gyro_fps": int(cam.get("gyro_fps", 200)),
        "accel_fps": int(cam.get("accel_fps", 200)),
        "unite_imu_method": 2,
        "infra1_qos": "SENSOR_DATA",
        "infra2_qos": "SENSOR_DATA",
        "infra1_info_qos": "SYSTEM_DEFAULT",
        "infra2_info_qos": "SYSTEM_DEFAULT",
    }
    if enable_depth:
        params["depth_qos"] = "SENSOR_DATA"
        params["depth_info_qos"] = "SYSTEM_DEFAULT"
        if "visual_preset" in cam:
            params["depth_module.visual_preset"] = int(cam["visual_preset"])
        if "clip_distance" in cam:
            params["clip_distance"] = float(cam["clip_distance"])
    if enable_color:
        params["rgb_camera.color_profile"] = color_profile
        params["color_qos"] = "SENSOR_DATA"
        params["color_info_qos"] = "SYSTEM_DEFAULT"
    if enable_imu:
        params["gyro_qos"] = "SENSOR_DATA"
        params["accel_qos"] = "SENSOR_DATA"

    print(
        f"[hound_core] RealSense profiles: depth_module={module_profile} "
        f"(infra1={enable_infra1} infra2={enable_infra2} depth={enable_depth}) "
        f"color={'on ' + color_profile if enable_color else 'off'} "
        f"align={align_depth} sync={enable_sync}"
    )
    return params


def build_realsense_composable(
    cam: dict, *, force_color: bool = False, streams: dict | None = None
) -> ComposableNode:
    camera_name = str(cam.get("camera_name", "camera"))
    params = realsense_camera_params(cam, force_color=force_color, streams=streams)
    print(
        f"[hound_core] RealSense composable: serial={params['serial_no']} "
        f"infra={params['depth_module.infra_profile']} "
        f"depth={'on ' + params['depth_module.depth_profile'] if params['enable_depth'] else 'off'} "
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


def build_vslam_composable(cam: dict, vslam: dict) -> ComposableNode:
    """cuVSLAM composable for RealSense infra1/infra2 (nitros-compat transport)."""
    camera_name = str(cam.get("camera_name", "camera"))
    enable_imu = bool(cam.get("enable_imu", False))
    base_frame = f"{camera_name}_link"
    imu_frame = f"{camera_name}_gyro_optical_frame"
    # Relative remaps (match isaac_ros_visual_slam_realsense.launch.py).
    left_image = f"{camera_name}/infra1/image_rect_raw"
    right_image = f"{camera_name}/infra2/image_rect_raw"
    left_info = f"{camera_name}/infra1/camera_info"
    right_info = f"{camera_name}/infra2/camera_info"
    camera_optical_frames = [
        f"{camera_name}_infra1_optical_frame",
        f"{camera_name}_infra2_optical_frame",
    ]
    default_tracking = 1 if enable_imu else 0
    tracking_mode = int(vslam.get("tracking_mode", default_tracking))
    transport = "nitros-compat"  # ManagedNitrosSubscriber + SENSOR_DATA QoS

    vslam_remappings = [
        ("visual_slam/image_0", left_image),
        ("visual_slam/camera_info_0", left_info),
        ("visual_slam/image_1", right_image),
        ("visual_slam/camera_info_1", right_info),
    ]
    if enable_imu and tracking_mode == 1:
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


def build_vslam_container(cam: dict, vslam: dict) -> ComposableNodeContainer:
    """cuVSLAM-only container for RealSense infra streams."""
    visual_slam_node = build_vslam_composable(cam, vslam)
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
    vslam_node = build_vslam_composable(cam, vslam)
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


def build_nvblox_node(
    nvblox: dict, cam: dict, seg: dict, lidar: dict | None = None
) -> Node:
    """nvblox: optional depth + color/seg RGB (+ people mask) and optional lidar.

    Geometry can come from depth and/or lidar. Color alone does not build TSDF.
    People mask applies to the depth path only (not lidar).
    """
    camera_name = str(cam.get("camera_name", "camera"))
    lidar = lidar or {}
    voxel_size = float(nvblox.get("voxel_size", 0.05))
    use_people_mask = bool(nvblox.get("use_people_mask", True))
    use_lidar = bool(nvblox.get("use_lidar", False))
    # Depth is optional: RGB + lidar is a valid geometry path.
    if "use_depth" in nvblox:
        use_depth = bool(nvblox.get("use_depth"))
    else:
        use_depth = bool(cam.get("enable_depth", False))
    use_color = bool(nvblox.get("use_color", True))
    global_frame = str(nvblox.get("global_frame", "odom"))
    map_clearing_frame = str(nvblox.get("map_clearing_frame_id", f"{camera_name}_link"))

    if use_people_mask and not use_depth:
        print(
            "[hound_core] nvblox use_people_mask=true but use_depth=false — "
            "people mask has no depth path; disabling people mask"
        )
        use_people_mask = False
    if not use_depth and not use_lidar:
        print(
            "[hound_core] nvblox WARNING: use_depth=false and use_lidar=false — "
            "no geometry source (color alone will not build a TSDF)"
        )

    align = bool(cam.get("align_depth", False))
    if align:
        depth_image = f"/{camera_name}/aligned_depth_to_color/image_raw"
        depth_info = f"/{camera_name}/aligned_depth_to_color/camera_info"
    else:
        depth_image = f"/{camera_name}/depth/image_rect_raw"
        depth_info = f"/{camera_name}/depth/camera_info"

    default_color = f"/{camera_name}/color/image_raw"
    color_info = f"/{camera_name}/color/camera_info"
    color_image = str(nvblox.get("color_topic") or "").strip() or default_color
    if color_image and not color_image.startswith("/"):
        color_image = "/" + color_image
    # Allow pointing at segmentation.sam_traversability_topic by name alias.
    if color_image in ("sam_traversability", "traversability"):
        color_image = str(
            seg.get("sam_traversability_topic", "/segmentation/refined_traversability")
        )
        if not color_image.startswith("/"):
            color_image = "/" + color_image

    people_mask = str(
        nvblox.get("people_mask_topic")
        or seg.get("sam_people_mask_topic")
        or "/segmentation/refined_people_mask"
    )
    if people_mask and not people_mask.startswith("/"):
        people_mask = "/" + people_mask

    # Prefer deskewed cloud when deskewer is on.
    deskew = lidar.get("deskewer") or {}
    default_lidar_topic = str(lidar.get("cloud_topic", "/unilidar/cloud"))
    if not default_lidar_topic.startswith("/"):
        default_lidar_topic = "/" + default_lidar_topic
    if bool(deskew.get("enabled", False)):
        default_lidar_topic = str(
            deskew.get("output_topic", "/unilidar/cloud_deskewed")
        )
        if not default_lidar_topic.startswith("/"):
            default_lidar_topic = "/" + default_lidar_topic
    lidar_topic = str(nvblox.get("lidar_topic") or default_lidar_topic)
    if lidar_topic and not lidar_topic.startswith("/"):
        lidar_topic = "/" + lidar_topic

    params_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "config",
        "nvblox_hound.yaml",
    )
    overrides = {
        "voxel_size": voxel_size,
        "global_frame": global_frame,
        "map_clearing_frame_id": map_clearing_frame,
        "map_clearing_radius_m": float(nvblox.get("map_clearing_radius_m", 25.0)),
        "layer_visualization_exclusion_radius_m": float(
            nvblox.get("layer_visualization_exclusion_radius_m", 25.0)
        ),
        "layer_visualization_exclusion_height_m": float(
            nvblox.get("layer_visualization_exclusion_height_m", 3.0)
        ),
        "max_back_projection_distance": float(
            nvblox.get("max_back_projection_distance", 25.0)
        ),
        "esdf_slice_bounds_visualization_attachment_frame_id": map_clearing_frame,
        "use_depth": use_depth,
        "use_color": use_color,
        "use_lidar": use_lidar,
        "use_segmentation": use_people_mask,
        "mapping_type": (
            "human_with_static_tsdf" if use_people_mask else "static_tsdf"
        ),
        "input_qos": "SENSOR_DATA",
        "integrate_depth_rate_hz": float(
            nvblox.get("integrate_depth_rate_hz", 20.0)
        ),
        "integrate_color_rate_hz": float(
            nvblox.get("integrate_color_rate_hz", 5.0)
        ),
        "update_esdf_rate_hz": float(nvblox.get("update_esdf_rate_hz", 10.0)),
        "update_mesh_rate_hz": float(nvblox.get("update_mesh_rate_hz", 5.0)),
        "publish_layer_rate_hz": float(
            nvblox.get("publish_layer_rate_hz", 5.0)
        ),
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
    # Projective max: set both camera and lidar keys (flat dotted names so YAML
    # nested static_mapper is not wiped / left at the old 5 m depth default).
    integ_max = float(
        nvblox.get(
            "projective_integrator_max_integration_distance_m",
            nvblox.get("lidar_projective_integrator_max_integration_distance_m", 15.0),
        )
    )
    lidar_integ_max = float(
        nvblox.get("lidar_projective_integrator_max_integration_distance_m", integ_max)
    )
    overrides["static_mapper.projective_integrator_max_integration_distance_m"] = (
        integ_max
    )
    overrides[
        "static_mapper.lidar_projective_integrator_max_integration_distance_m"
    ] = lidar_integ_max
    overrides["dynamic_mapper.projective_integrator_max_integration_distance_m"] = (
        integ_max
    )
    overrides[
        "dynamic_mapper.lidar_projective_integrator_max_integration_distance_m"
    ] = lidar_integ_max
    # View-based TSDF decay needs a camera depth view. With lidar-only geometry
    # it spams "Last view not set… Decaying all voxels" and clears the map.
    if not use_depth:
        overrides["decay_tsdf_rate_hz"] = float(
            nvblox.get("decay_tsdf_rate_hz", 0.0)
        )
    elif "decay_tsdf_rate_hz" in nvblox:
        overrides["decay_tsdf_rate_hz"] = float(nvblox["decay_tsdf_rate_hz"])
    if use_lidar:
        # Prefer nvblox overrides; else mirror lidar organizer model.
        lidar_w = int(nvblox.get("lidar_width", lidar.get("lidar_width", 360)))
        lidar_h = int(nvblox.get("lidar_height", lidar.get("lidar_height", 90)))
        lidar_vfov = float(
            nvblox.get(
                "lidar_vertical_fov_rad",
                lidar.get("lidar_vertical_fov_rad", 2.094395),
            )
        )
        overrides.update({
            "lidar_width": lidar_w,
            "lidar_height": lidar_h,
            "lidar_vertical_fov_rad": lidar_vfov,
            "lidar_min_valid_range_m": float(
                nvblox.get("lidar_min_valid_range_m", 0.1)
            ),
            "use_non_equal_vertical_fov_lidar_params": bool(
                nvblox.get("use_non_equal_vertical_fov_lidar_params", False)
            ),
            "use_lidar_motion_compensation": bool(
                nvblox.get("use_lidar_motion_compensation", False)
            ),
            "pointcloud2_timestamps_are_relative": bool(
                nvblox.get("pointcloud2_timestamps_are_relative", False)
            ),
            "integrate_lidar_rate_hz": float(
                nvblox.get("integrate_lidar_rate_hz", 10.0)
            ),
        })

    remappings = [
        ("camera_0/color/image", color_image),
        ("camera_0/color/camera_info", color_info),
    ]
    if use_depth:
        remappings.extend([
            ("camera_0/depth/image", depth_image),
            ("camera_0/depth/camera_info", depth_info),
        ])
    if use_people_mask:
        remappings.extend([
            ("camera_0/mask/image", people_mask),
            ("camera_0/mask/camera_info", color_info),
        ])
    if use_lidar:
        remappings.append(("pointcloud", lidar_topic))

    print(
        f"[hound_core] nvblox ENABLED: voxel_size={voxel_size} m "
        f"use_depth={use_depth} use_lidar={use_lidar} use_color={use_color} "
        f"color={color_image} "
        f"people_mask={'on -> ' + people_mask if use_people_mask else 'off'} "
        f"lidar={'on -> ' + lidar_topic if use_lidar else 'off'} "
        f"depth={depth_image if use_depth else 'off'} "
        f"global_frame={global_frame} "
        f"clear_radius={overrides['map_clearing_radius_m']} m "
        f"viz_radius={overrides['layer_visualization_exclusion_radius_m']} m"
    )
    if use_lidar and use_people_mask:
        print(
            "[hound_core] nvblox NOTE: people mask applies to depth only; "
            "lidar integrates as static geometry (NVIDIA examples often avoid "
            "combining lidar + people mode — watch for map ghosts on people)"
        )
    if use_lidar:
        print(
            f"[hound_core] nvblox lidar model: "
            f"{overrides['lidar_width']}x{overrides['lidar_height']} "
            f"vfov={overrides['lidar_vertical_fov_rad']:.3f} rad "
            f"integrate_max={lidar_integ_max} m "
            f"(projective_max={integ_max} m) — "
            "cloud must be organized (width*height == point count)"
        )

    return Node(
        package="nvblox_ros",
        executable="nvblox_node",
        name="nvblox_node",
        output="screen",
        parameters=[params_file, overrides],
        remappings=remappings,
    )


def build_hound_mapping_node(
    nvblox: dict, cam: dict, seg: dict, lidar: dict | None = None
) -> Node:
    """In-house nvblox mapping: elevation/cost/esdf images + MapInfo (no GridMap)."""
    camera_name = str(cam.get("camera_name", "camera"))
    lidar = lidar or {}
    use_people_mask = bool(nvblox.get("use_people_mask", False))
    use_lidar = bool(nvblox.get("use_lidar", False))
    if "use_depth" in nvblox:
        use_depth = bool(nvblox.get("use_depth"))
    else:
        use_depth = bool(cam.get("enable_depth", False))
    use_color = bool(nvblox.get("use_color", True))
    global_frame = str(nvblox.get("global_frame", "odom"))
    map_clearing_frame = str(nvblox.get("map_clearing_frame_id", f"{camera_name}_link"))

    if use_people_mask and not use_depth:
        use_people_mask = False

    align = bool(cam.get("align_depth", False))
    if align:
        depth_image = f"/{camera_name}/aligned_depth_to_color/image_raw"
        depth_info = f"/{camera_name}/aligned_depth_to_color/camera_info"
    else:
        depth_image = f"/{camera_name}/depth/image_rect_raw"
        depth_info = f"/{camera_name}/depth/camera_info"

    default_color = f"/{camera_name}/color/image_raw"
    color_info = f"/{camera_name}/color/camera_info"
    color_image = str(nvblox.get("color_topic") or "").strip() or default_color
    if color_image and not color_image.startswith("/"):
        color_image = "/" + color_image
    if color_image in ("sam_traversability", "traversability"):
        color_image = str(
            seg.get("sam_traversability_topic", "/segmentation/refined_traversability")
        )
        if not color_image.startswith("/"):
            color_image = "/" + color_image

    people_mask = str(
        nvblox.get("people_mask_topic")
        or seg.get("sam_people_mask_topic")
        or "/segmentation/refined_people_mask"
    )
    if people_mask and not people_mask.startswith("/"):
        people_mask = "/" + people_mask

    deskew = lidar.get("deskewer") or {}
    default_lidar_topic = str(lidar.get("cloud_topic", "/unilidar/cloud"))
    if not default_lidar_topic.startswith("/"):
        default_lidar_topic = "/" + default_lidar_topic
    if bool(deskew.get("enabled", False)):
        default_lidar_topic = str(
            deskew.get("output_topic", "/unilidar/cloud_deskewed")
        )
        if not default_lidar_topic.startswith("/"):
            default_lidar_topic = "/" + default_lidar_topic
    lidar_topic = str(nvblox.get("lidar_topic") or default_lidar_topic)
    if lidar_topic and not lidar_topic.startswith("/"):
        lidar_topic = "/" + lidar_topic

    integ_max = float(
        nvblox.get(
            "projective_integrator_max_integration_distance_m",
            nvblox.get("lidar_projective_integrator_max_integration_distance_m", 15.0),
        )
    )
    lidar_integ_max = float(
        nvblox.get("lidar_projective_integrator_max_integration_distance_m", integ_max)
    )

    params = {
        "global_frame": global_frame,
        "map_clearing_frame_id": map_clearing_frame,
        "voxel_size": float(nvblox.get("voxel_size", 0.05)),
        "elevation_resolution": float(
            nvblox.get("elevation_resolution", nvblox.get("voxel_size", 0.05))
        ),
        "map_clearing_radius_m": float(nvblox.get("map_clearing_radius_m", 32.0)),
        "max_integration_distance_m": integ_max,
        "lidar_max_integration_distance_m": lidar_integ_max,
        "lethal_slope_deg": float(nvblox.get("lethal_slope_deg", 60.0)),
        "do_inpaint": bool(nvblox.get("do_inpaint", True)),
        "inpaint_resolution_m": float(nvblox.get("inpaint_resolution_m", 0.25)),
        "inpaint_radius": int(nvblox.get("inpaint_radius", 3)),
        "unobserved_luminance": float(nvblox.get("unobserved_luminance", 0.0)),
        "color_weight_min": float(nvblox.get("color_weight_min", 0.001)),
        "robot_height_m": float(nvblox.get("robot_height_m", 0.4)),
        "elev_z_min_rel": float(nvblox.get("elev_z_min_rel", -5.0)),
        "elev_z_max_rel": float(nvblox.get("elev_z_max_rel", 2.0)),
        "use_depth": use_depth,
        "use_color": use_color,
        "use_lidar": use_lidar,
        "use_people_mask": use_people_mask,
        "depth_topic": depth_image,
        "depth_info_topic": depth_info,
        "color_topic": color_image,
        "color_info_topic": color_info,
        "lidar_topic": lidar_topic,
        "people_mask_topic": people_mask,
        "lidar_width": int(nvblox.get("lidar_width", lidar.get("lidar_width", 360))),
        "lidar_height": int(nvblox.get("lidar_height", lidar.get("lidar_height", 90))),
        "lidar_vertical_fov_rad": float(
            nvblox.get(
                "lidar_vertical_fov_rad",
                lidar.get("lidar_vertical_fov_rad", 2.094395),
            )
        ),
        "lidar_min_valid_range_m": float(nvblox.get("lidar_min_valid_range_m", 0.3)),
        "use_lidar_motion_compensation": bool(
            nvblox.get("use_lidar_motion_compensation", False)
        ),
        "integrate_depth_rate_hz": float(nvblox.get("integrate_depth_rate_hz", 20.0)),
        "integrate_color_rate_hz": float(nvblox.get("integrate_color_rate_hz", 20.0)),
        "integrate_lidar_rate_hz": float(nvblox.get("integrate_lidar_rate_hz", 10.0)),
        "map_clear_rate_hz": float(nvblox.get("map_clear_rate_hz", 20.0)),
        "publish_map_rate_hz": float(nvblox.get("publish_map_rate_hz", 5.0)),
    }

    print(
        f"[hound_core] hound_mapping ENABLED: voxel={params['voxel_size']} "
        f"depth={use_depth} lidar={use_lidar} color={use_color} "
        f"~/local_map + ~/elev_color, clear_r={params['map_clearing_radius_m']}"
    )
    return Node(
        package="hound_mapping",
        executable="mapping_node",
        name="hound_mapping",
        output="screen",
        parameters=[params],
    )


def build_nav_node(nav: dict) -> Node:
    """IGHA* + UW_mppi navigation (MPPI main thread, planner process)."""
    params = {
        "config_path": str(nav.get("config_path", "") or ""),
        "local_map_topic": str(
            nav.get("local_map_topic", "/hound_mapping/local_map")
        ),
        "state_topic": str(
            nav.get("state_topic", "/hound_fcu_control/control_state")
        ),
        "path_topic": str(nav.get("path_topic", "/mission/path")),
        "inject_path_topic": str(
            nav.get("inject_path_topic", "/hound_nav/inject_path")
        ),
        "skip_planner": bool(nav.get("skip_planner", False)),
        "cmd_topic": str(nav.get("cmd_topic", "/hound_nav/cmd_ackermann")),
        "plan_topic": str(nav.get("plan_topic", "/hound_nav/local_plan")),
        "latency_topic": str(
            nav.get("latency_topic", "/hound_nav/state_to_cmd_ms")
        ),
        "event_driven": bool(nav.get("event_driven", True)),
        "async_bev": bool(nav.get("async_bev", True)),
        "bev_hz": float(nav.get("bev_hz", 20.0)),
        "control_rate_hz": float(nav.get("control_rate_hz", 20.0)),
        "max_steer_rad": float(nav.get("max_steer_rad", 0.6)),
        "max_speed_mps": float(nav.get("max_speed_mps", 20.0)),
    }
    print(
        f"[hound_core] nav ENABLED: map={params['local_map_topic']} "
        f"state={params['state_topic']} cmd={params['cmd_topic']} "
        f"skip_planner={params['skip_planner']} "
        f"event_driven={params['event_driven']} "
        f"async_bev={params['async_bev']}@{params['bev_hz']}Hz "
        f"rate={params['control_rate_hz']} Hz"
    )
    return Node(
        package="hound_nav",
        executable="nav_node",
        name="hound_nav",
        output="screen",
        parameters=[params],
    )


def build_viz_node(viz: dict) -> Node:
    """Viser whole-stack browser viz (subscribe-only)."""
    params = {
        "host": str(viz.get("host", "0.0.0.0")),
        "port": int(viz.get("port", 8080)),
        "global_frame": str(viz.get("global_frame", "odom")),
        "base_frame": str(viz.get("base_frame", "base_link")),
        "local_map_topic": str(
            viz.get("local_map_topic", "/hound_mapping/local_map")
        ),
        "odom_topic": str(
            viz.get("odom_topic", "/visual_slam/tracking/odometry")
        ),
        "lidar_topic": str(viz.get("lidar_topic", "/unilidar/cloud")),
        "camera_topic": str(viz.get("camera_topic", "/camera/color/image_raw")),
        "path_topic": str(viz.get("path_topic", "/mission/path")),
        "mesh_pose_topic": str(
            viz.get("mesh_pose_topic", "/localization/mesh_pose")
        ),
        "use_local_map": bool(viz.get("use_local_map", True)),
        "use_odom": bool(viz.get("use_odom", True)),
        "use_lidar": bool(viz.get("use_lidar", True)),
        "use_camera": bool(viz.get("use_camera", True)),
        "use_path": bool(viz.get("use_path", True)),
        "use_mesh_pose": bool(viz.get("use_mesh_pose", True)),
        "lidar_max_points": int(viz.get("lidar_max_points", 20000)),
        "lidar_min_period_s": float(viz.get("lidar_min_period_s", 0.2)),
        "camera_min_period_s": float(viz.get("camera_min_period_s", 0.2)),
        "map_mesh_stride": int(viz.get("map_mesh_stride", 2)),
        "map_z_exaggeration": float(viz.get("map_z_exaggeration", 1.0)),
    }
    print(
        f"[hound_core] viz ENABLED: Viser http://<host>:{params['port']}/ "
        f"map={params['local_map_topic']}"
    )
    return Node(
        package="hound_viz",
        executable="viz_node",
        name="hound_viz",
        output="screen",
        parameters=[params],
    )


def build_ekf_node(ekf: dict) -> Node:
    preset = str(ekf.get("preset", "realsense_vslam"))
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


def apply_hal_camera_defaults(hal: dict, cam: dict) -> dict:
    """Return HAL params with stereo_composite camera monitor topics/frames."""
    params = flatten_params({k: v for k, v in hal.items() if k != "enabled"})
    camera_name = str(cam.get("camera_name", "camera"))
    params.setdefault("camera.depth_frame", f"{camera_name}_link")
    params.setdefault(
        "camera.depth_optical_frame",
        f"{camera_name}_color_optical_frame",
    )
    params.setdefault(
        "camera.monitor_topic",
        f"/{camera_name}/color/image_raw",
    )
    params.setdefault("camera.expected_fps", float(cam.get("fps", 15.0)))

    fcu_cfg = hal.get("fcu") or hal.get("mavros") or {}
    params.setdefault(
        "fcu.monitor_enabled",
        bool(fcu_cfg.get("monitor_enabled", True)),
    )
    return params


def build_lidar_mesh_composite_node(lidar: dict) -> Node:
    """In-process lidar SDK + constant-twist mesh PF (composite_sensing)."""
    comp = dict(lidar.get("composite") or {})
    xyz = lidar.get("xyz") or [0.0, 0.0, 0.1]
    rpy = lidar.get("rpy") or [180.0, -15.0, 0.0]
    bb = dict(comp.get("init_bb") or {})
    params = {
        "lidar_backend": str(lidar.get("backend", "unitree")),
        "port": str(lidar.get("port", "/dev/ttyUSB0")),
        "lidar_frame": str(lidar.get("cloud_frame", "unilidar_lidar")),
        # Extrinsic xyz/rpy is base→lidar; static TF uses the same (parent should
        # match base_frame for the mesh PF body frame).
        "base_frame": str(comp.get("base_frame", "base_link")),
        "parent_frame": str(
            comp.get("base_frame", lidar.get("parent_frame", "base_link"))
        ),
        "xyz.x": float(xyz[0]),
        "xyz.y": float(xyz[1]),
        "xyz.z": float(xyz[2]),
        "rpy.roll": float(rpy[0]),
        "rpy.pitch": float(rpy[1]),
        "rpy.yaw": float(rpy[2]),
        "range_min": float(lidar.get("range_min", 0.4)),
        "range_max": float(lidar.get("range_max", 50.0)),
        "cloud_scan_num": int(lidar.get("cloud_scan_num", 18)),
        "publish_cloud": bool(comp.get("publish_cloud", True)),
        "cloud_topic": str(lidar.get("cloud_topic", "/unilidar/cloud")),
        "map_file": str(comp.get("map_file", "")),
        "map_frame": str(comp.get("map_frame", "map")),
        "pose_topic": str(comp.get("pose_topic", "/localization/mesh_pose")),
        "localize_hz": float(comp.get("localize_hz", 10.0)),
        "num_particles": int(comp.get("num_particles", 2000)),
        "beam_samples": int(comp.get("beam_samples", 64)),
        "global_init_on_start": bool(comp.get("global_init_on_start", True)),
        "init_bb.xmin": float(bb.get("xmin", -20.0)),
        "init_bb.ymin": float(bb.get("ymin", -20.0)),
        "init_bb.zmin": float(bb.get("zmin", 0.0)),
        "init_bb.xmax": float(bb.get("xmax", 20.0)),
        "init_bb.ymax": float(bb.get("ymax", 20.0)),
        "init_bb.zmax": float(bb.get("zmax", 2.0)),
    }
    print(
        f"[hound_core] lidar_mesh_composite ENABLED: port={params['port']} "
        f"map={params['map_file'] or '(none)'} pose={params['pose_topic']} "
        f"hz={params['localize_hz']}"
    )
    return Node(
        package="composite_sensing",
        executable="lidar_mesh_composite_node",
        name="lidar_mesh_composite",
        output="screen",
        parameters=[params],
    )


def build_unitree_lidar_actions(lidar: dict) -> list:
    """Unitree UniLidar serial driver (+ optional static TF / deskewer)."""
    port = str(lidar.get("port", "/dev/ttyUSB0"))
    cloud_frame = str(lidar.get("cloud_frame", "unilidar_lidar"))
    cloud_topic = str(lidar.get("cloud_topic", "unilidar/cloud"))
    imu_frame = str(lidar.get("imu_frame", "unilidar_imu"))
    imu_topic = str(lidar.get("imu_topic", "unilidar/imu"))

    params = {
        "port": port,
        "rotate_yaw_bias": float(lidar.get("rotate_yaw_bias", 0.0)),
        "range_scale": float(lidar.get("range_scale", 0.001)),
        "range_bias": float(lidar.get("range_bias", 0.0)),
        "range_max": float(lidar.get("range_max", 50.0)),
        "range_min": float(lidar.get("range_min", 0.0)),
        "cloud_frame": cloud_frame,
        "cloud_topic": cloud_topic,
        "cloud_scan_num": int(lidar.get("cloud_scan_num", 18)),
        "organize_cloud": bool(lidar.get("organize_cloud", True)),
        # Prefer explicit lidar_* ; fall back to legacy points_per_ring naming.
        "lidar_width": int(
            lidar.get("lidar_width", lidar.get("points_per_ring", 360))
        ),
        "lidar_height": int(lidar.get("lidar_height", 90)),
        "lidar_vertical_fov_rad": float(
            lidar.get("lidar_vertical_fov_rad", 2.094395)
        ),
        "imu_frame": imu_frame,
        "imu_topic": imu_topic,
        "enable_imu": bool(lidar.get("enable_imu", True)),
    }
    enable_imu = bool(params["enable_imu"])
    print(
        f"[hound_core] lidar ENABLED (unitree): port={port} "
        f"cloud={cloud_topic} ({cloud_frame}) "
        f"organize={params['organize_cloud']} "
        f"{params['lidar_width']}x{params['lidar_height']} "
        f"vfov={params['lidar_vertical_fov_rad']:.3f} "
        f"imu={'on -> ' + imu_topic if enable_imu else 'off'}"
    )
    actions = [
        Node(
            package="unitree_lidar_ros2",
            executable="unitree_lidar_ros2_node",
            name="unitree_lidar_ros2_node",
            output="screen",
            parameters=[params],
            respawn=bool(lidar.get("respawn", True)),
            respawn_delay=float(lidar.get("respawn_delay_s", 2.0)),
        )
    ]

    if bool(lidar.get("publish_static_tf", True)):
        parent = str(lidar.get("parent_frame", "camera_link"))
        xyz = lidar.get("xyz") or [0.0, 0.0, 0.0]
        # SSoT rpy / imu_rpy are degrees; static_transform_publisher wants radians.
        rpy_deg = lidar.get("rpy") or [0.0, 0.0, 0.0]
        rpy = [math.radians(float(v)) for v in rpy_deg]
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="unilidar_static_tf",
                output="screen",
                arguments=[
                    "--x", str(float(xyz[0])),
                    "--y", str(float(xyz[1])),
                    "--z", str(float(xyz[2])),
                    "--roll", str(rpy[0]),
                    "--pitch", str(rpy[1]),
                    "--yaw", str(rpy[2]),
                    "--frame-id", parent,
                    "--child-frame-id", cloud_frame,
                ],
            )
        )
        # IMU frame relative to lidar (only if IMU is enabled).
        imu_xyz = lidar.get("imu_xyz") or [0.0, 0.0, 0.0]
        imu_rpy_deg = lidar.get("imu_rpy") or [0.0, 0.0, 0.0]
        imu_rpy = [math.radians(float(v)) for v in imu_rpy_deg]
        if enable_imu and imu_frame != cloud_frame:
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="unilidar_imu_static_tf",
                    output="screen",
                    arguments=[
                        "--x", str(float(imu_xyz[0])),
                        "--y", str(float(imu_xyz[1])),
                        "--z", str(float(imu_xyz[2])),
                        "--roll", str(imu_rpy[0]),
                        "--pitch", str(imu_rpy[1]),
                        "--yaw", str(imu_rpy[2]),
                        "--frame-id", cloud_frame,
                        "--child-frame-id", imu_frame,
                    ],
                )
            )
        print(
            f"[hound_core] lidar static TF: {parent} → {cloud_frame} "
            f"(xyz={list(xyz)} rpy_deg={list(rpy_deg)})"
        )

    deskew = lidar.get("deskewer") or {}
    if bool(deskew.get("enabled", False)):
        deskew_params = {
            "input_topic": str(deskew.get("input_topic", f"/{cloud_topic.lstrip('/')}")),
            "output_topic": str(deskew.get("output_topic", "/unilidar/cloud_deskewed")),
            "target_frame": str(deskew.get("target_frame", cloud_frame)),
            "timestamp_field": str(deskew.get("timestamp_field", "time")),
            "timestamp_unit": str(deskew.get("timestamp_unit", "seconds")),
            "deskew_reference": str(deskew.get("deskew_reference", "end")),
            "tf_buffer_duration": float(deskew.get("tf_buffer_duration", 10.0)),
            "tf_lookup_timeout": float(deskew.get("tf_lookup_timeout", 0.0)),
        }
        # Normalize topics to absolute paths for the deskewer node.
        if not deskew_params["input_topic"].startswith("/"):
            deskew_params["input_topic"] = "/" + deskew_params["input_topic"]
        print(
            f"[hound_core] lidar deskewer: {deskew_params['input_topic']} → "
            f"{deskew_params['output_topic']}"
        )
        actions.append(
            Node(
                package="pointcloud_deskewer",
                executable="deskewer",
                name="unilidar_deskewer",
                output="screen",
                parameters=[deskew_params],
            )
        )

    return actions


def build_hal_monitor_node(hal: dict, cam: dict) -> Node:
    params = apply_hal_camera_defaults(hal, cam)
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
