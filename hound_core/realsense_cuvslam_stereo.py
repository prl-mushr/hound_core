#!/usr/bin/env python3
"""pyrealsense2 IR stereo → pycuvslam Tracker (VO + SLAM), optional ROS odom + TF.

Adapted from NVIDIA cuVSLAM examples/realsense/run_stereo.py:
  https://github.com/nvidia-isaac/cuVSLAM/blob/main/examples/realsense/run_stereo.py

Publishes nav_msgs/Odometry (RELIABLE) and TF odom → camera_link from the SLAM pose.

Example:
  ros2 run hound_core realsense_cuvslam_stereo -- --fps 60
  ros2 topic hz /visual_slam/tracking/odometry
  ros2 run tf2_ros tf2_echo odom camera_link
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import pyrealsense2 as rs

import cuvslam as vslam

from hound_core.rs_cuvslam_camera_utils import get_rs_stereo_rig

DEFAULT_SERIAL = "030422250301"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 360
DEFAULT_FPS = 30
WARMUP_FRAMES = 60
IMAGE_JITTER_THRESHOLD_NS = 35 * 1e6  # 35 ms
PRINT_EVERY_S = 1.0
DEFAULT_ODOM_TOPIC = "/visual_slam/tracking/odometry"
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_BASE_FRAME = "camera_link"


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--serial", default=DEFAULT_SERIAL, help="RealSense serial number")
    p.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    p.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    p.add_argument("--fps", type=int, default=DEFAULT_FPS, help="IR stream FPS (30 or 60)")
    p.add_argument(
        "--warmup-frames",
        type=int,
        default=WARMUP_FRAMES,
        help="Frames to skip before calling tracker.track()",
    )
    p.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after N tracked frames (0 = run until Ctrl-C)",
    )
    p.add_argument(
        "--no-ros",
        action="store_true",
        help="Disable ROS odom/TF publish (tracker-only smoke)",
    )
    p.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    p.add_argument("--odom-frame", default=DEFAULT_ODOM_FRAME)
    p.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    return p.parse_args(argv)


def disable_emitter(pipeline: rs.pipeline, config: rs.config) -> None:
    wrapper = rs.pipeline_wrapper(pipeline)
    profile = config.resolve(wrapper)
    device = profile.get_device()
    depth_sensor = device.query_sensors()[0]
    if depth_sensor.supports(rs.option.emitter_enabled):
        depth_sensor.set_option(rs.option.emitter_enabled, 0)


def grab_stereo_calibration(
    pipeline: rs.pipeline, config: rs.config
) -> dict:
    """Start briefly, read IR intrinsics/extrinsics, stop."""
    pipeline.start(config)
    frames = pipeline.wait_for_frames()
    left_profile = frames.get_infrared_frame(1).profile.as_video_stream_profile()
    right_profile = frames.get_infrared_frame(2).profile.as_video_stream_profile()
    camera_params = {
        "left": {"intrinsics": left_profile.intrinsics},
        "right": {
            "intrinsics": right_profile.intrinsics,
            "extrinsics": right_profile.get_extrinsics_to(left_profile),
        },
    }
    pipeline.stop()
    return camera_params


def _quat_xyzw(pose) -> np.ndarray:
    """cuVSLAM Pose.rotation is xyzw (matches geometry_msgs / scipy)."""
    q = np.asarray(pose.rotation, dtype=float).reshape(-1)
    if q.shape[0] != 4:
        raise ValueError(f"unexpected quaternion shape {q.shape}")
    return q


# cuVSLAM / OpenCV optical: X right, Y down, Z forward.
# ROS REP-103 FLU (camera_link): X forward, Y left, Z up.
# Maps a vector from optical coords → FLU coords (same physical axes remap).
_R_FLU_FROM_OPTICAL = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=float,
)
_R_OPTICAL_FROM_FLU = _R_FLU_FROM_OPTICAL.T


def optical_pose_to_ros_flu(pose) -> Tuple[np.ndarray, np.ndarray]:
    """Convert cuVSLAM world←optical pose to ROS odom←camera_link (FLU).

    cuVSLAM's world is initialized in the first optical frame, so we rotate
    both the world and the child into FLU. Identity stays identity; moving
    the camera along its optical axis increases +X in odom.
    """
    from scipy.spatial.transform import Rotation

    t_opt = np.asarray(pose.translation, dtype=float).reshape(3)
    R_wo = Rotation.from_quat(_quat_xyzw(pose)).as_matrix()  # world←optical

    # T_flu_world←flu_link = R_flu←opt * T_opt_world←opt * T_opt←flu
    R_wl = _R_FLU_FROM_OPTICAL @ R_wo @ _R_OPTICAL_FROM_FLU
    t_wl = _R_FLU_FROM_OPTICAL @ t_opt
    q_wl = Rotation.from_matrix(R_wl).as_quat()  # xyzw
    return t_wl, q_wl


class RosOdomPublisher:
    """Publish Odometry + TF odom→base in ROS FLU convention.

    Odometry uses RELIABLE (RViz / most nav consumers). TF uses the
    TransformBroadcaster defaults.
    """

    def __init__(self, odom_topic: str, odom_frame: str, base_frame: str):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from tf2_ros import TransformBroadcaster

        if not rclpy.ok():
            rclpy.init(args=None)

        self._Odometry = Odometry
        self._TransformStamped = TransformStamped
        self.node = Node("realsense_cuvslam_stereo")
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        # RELIABLE so RViz / Nav2 / EKF can subscribe without QoS mismatch.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_pub = self.node.create_publisher(Odometry, odom_topic, odom_qos)
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self._prev_t: Optional[np.ndarray] = None
        self._prev_stamp_ns: Optional[int] = None

        self.node.get_logger().info(
            f"publishing odom={odom_topic} TF {odom_frame}→{base_frame} "
            f"(RELIABLE, optical→FLU)"
        )

    def publish(self, stamp_ns: int, pose) -> None:
        from builtin_interfaces.msg import Time

        t, q = optical_pose_to_ros_flu(pose)

        stamp = Time()
        stamp.sec = int(stamp_ns // 1_000_000_000)
        stamp.nanosec = int(stamp_ns % 1_000_000_000)

        odom = self._Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(t[0])
        odom.pose.pose.position.y = float(t[1])
        odom.pose.pose.position.z = float(t[2])
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])

        # Finite-difference twist in odom (FLU) frame.
        if self._prev_t is not None and self._prev_stamp_ns is not None:
            dt = (stamp_ns - self._prev_stamp_ns) * 1e-9
            if dt > 1e-6:
                v = (t - self._prev_t) / dt
                odom.twist.twist.linear.x = float(v[0])
                odom.twist.twist.linear.y = float(v[1])
                odom.twist.twist.linear.z = float(v[2])
        self._prev_t = t.copy()
        self._prev_stamp_ns = stamp_ns

        self.odom_pub.publish(odom)

        tf = self._TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = float(t[0])
        tf.transform.translation.y = float(t[1])
        tf.transform.translation.z = float(t[2])
        tf.transform.rotation.x = float(q[0])
        tf.transform.rotation.y = float(q[1])
        tf.transform.rotation.z = float(q[2])
        tf.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(tf)

    def spin_once(self) -> None:
        import rclpy

        rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self) -> None:
        import rclpy

        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None) -> int:
    args = parse_args(argv)
    resolution: Tuple[int, int] = (args.width, args.height)

    config = rs.config()
    config.enable_device(str(args.serial))
    config.enable_stream(
        rs.stream.infrared, 1, resolution[0], resolution[1], rs.format.y8, args.fps
    )
    config.enable_stream(
        rs.stream.infrared, 2, resolution[0], resolution[1], rs.format.y8, args.fps
    )

    pipeline = rs.pipeline()
    print(
        f"[realsense_cuvslam_stereo] opening serial={args.serial} "
        f"{resolution[0]}x{resolution[1]}@{args.fps} IR stereo (emitter off)"
    )

    try:
        camera_params = grab_stereo_calibration(pipeline, config)
    except RuntimeError as exc:
        print(f"[realsense_cuvslam_stereo] failed to open camera: {exc}", file=sys.stderr)
        print(
            "Is another process using the RealSense (realsense2_camera / hound_core)?",
            file=sys.stderr,
        )
        return 1

    disable_emitter(pipeline, config)

    odom_cfg = vslam.Tracker.OdometryConfig(
        async_sba=True,
        enable_final_landmarks_export=False,
        enable_observations_export=False,
        rectified_stereo_camera=True,
    )
    # Enabling SlamConfig turns on mapping / loop closure (not VO-only).
    slam_cfg = vslam.Tracker.SlamConfig(sync_mode=False)
    rig = get_rs_stereo_rig(camera_params)
    tracker = vslam.Tracker(rig, odom_cfg, slam_cfg)
    print(
        "[realsense_cuvslam_stereo] Tracker ready "
        "(VO+SLAM, async_sba=True, sync_mode=False); streaming…"
    )
    ros_pub: Optional[RosOdomPublisher] = None
    if not args.no_ros:
        try:
            ros_pub = RosOdomPublisher(
                args.odom_topic, args.odom_frame, args.base_frame
            )
        except Exception as exc:
            print(f"[realsense_cuvslam_stereo] ROS init failed: {exc}", file=sys.stderr)
            return 1

    pipeline.start(config)

    frame_id = 0
    tracked = 0
    prev_timestamp: Optional[int] = None
    stamps: Deque[float] = deque(maxlen=max(args.fps * 2, 30))
    last_print = time.monotonic()
    last_t = np.zeros(3)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            left_frame = frames.get_infrared_frame(1)
            right_frame = frames.get_infrared_frame(2)
            if not left_frame or not right_frame:
                continue

            frame_id += 1
            timestamp = int(left_frame.timestamp * 1e6)  # ms → ns (NVIDIA example)

            if prev_timestamp is not None:
                gap = timestamp - prev_timestamp
                if gap > IMAGE_JITTER_THRESHOLD_NS:
                    print(
                        f"[warn] timestamp gap {gap / 1e6:.1f} ms "
                        f"(threshold {IMAGE_JITTER_THRESHOLD_NS / 1e6:.0f} ms)"
                    )
            prev_timestamp = timestamp

            if frame_id <= args.warmup_frames:
                if frame_id == args.warmup_frames:
                    print(
                        f"[realsense_cuvslam_stereo] warmup done ({args.warmup_frames} frames)"
                    )
                continue

            images = (
                np.asanyarray(left_frame.get_data()),
                np.asanyarray(right_frame.get_data()),
            )
            odom_pose_estimate, slam_pose = tracker.track(timestamp, images)
            # Prefer SLAM pose (loop-closure corrected); fall back to VO.
            if slam_pose is not None:
                pose = slam_pose
            elif (
                odom_pose_estimate is not None
                and odom_pose_estimate.world_from_rig is not None
            ):
                pose = odom_pose_estimate.world_from_rig.pose
            else:
                print("[warn] pose tracking not valid")
                continue
            last_t, _ = optical_pose_to_ros_flu(pose)
            tracked += 1
            now = time.monotonic()
            stamps.append(now)

            if ros_pub is not None:
                ros_pub.publish(timestamp, pose)
                ros_pub.spin_once()

            if now - last_print >= PRINT_EVERY_S:
                hz = 0.0
                if len(stamps) >= 2:
                    dt = stamps[-1] - stamps[0]
                    if dt > 0:
                        hz = (len(stamps) - 1) / dt
                print(
                    f"tracked={tracked} hz={hz:.1f} "
                    f"t=[{last_t[0]:+.3f}, {last_t[1]:+.3f}, {last_t[2]:+.3f}]"
                )
                last_print = now

            if args.max_frames > 0 and tracked >= args.max_frames:
                print(
                    f"[realsense_cuvslam_stereo] reached --max-frames={args.max_frames}; done"
                )
                break
    except KeyboardInterrupt:
        print("\n[realsense_cuvslam_stereo] interrupted")
    finally:
        pipeline.stop()
        if ros_pub is not None:
            ros_pub.shutdown()

    return 0 if tracked > 0 else 2


if __name__ == "__main__":
    sys.exit(main())
