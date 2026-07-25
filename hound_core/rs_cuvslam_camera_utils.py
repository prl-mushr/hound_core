#!/usr/bin/env python3
# Copyright helpers adapted from NVIDIA cuVSLAM examples/realsense/camera_utils.py
"""Minimal RealSense → cuVSLAM rig helpers (stereo IR only)."""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np
from scipy.spatial.transform import Rotation

import cuvslam as vslam


def transform_to_pose(transform_matrix=None) -> vslam.Pose:
    """Convert a RealSense extrinsics object to a vslam.Pose."""
    if transform_matrix:
        rotation_matrix = np.array(transform_matrix.rotation).reshape([3, 3])
        translation_vec = transform_matrix.translation
        rotation_quat = Rotation.from_matrix(rotation_matrix).as_quat()
        return vslam.Pose(rotation=rotation_quat, translation=translation_vec)

    rotation_quat = Rotation.from_matrix(np.eye(3)).as_quat()
    return vslam.Pose(rotation=rotation_quat, translation=[0.0, 0.0, 0.0])


def get_rs_camera(
    rs_intrinsics, transform_matrix: Optional[Any] = None
) -> vslam.Camera:
    """Create a cuVSLAM Camera from RealSense intrinsics (+ optional extrinsics)."""
    cam = vslam.Camera()
    cam.distortion = vslam.Distortion(vslam.Distortion.Model.Pinhole)
    cam.focal = rs_intrinsics.fx, rs_intrinsics.fy
    cam.principal = rs_intrinsics.ppx, rs_intrinsics.ppy
    cam.size = rs_intrinsics.width, rs_intrinsics.height
    if transform_matrix is not None:
        cam.rig_from_camera = transform_to_pose(transform_matrix)
    return cam


def get_rs_stereo_rig(camera_params: Dict[str, Dict[str, Any]]) -> vslam.Rig:
    """Build a stereo Rig from left/right RealSense intrinsics + right→left extrinsics."""
    rig = vslam.Rig()
    cameras = [get_rs_camera(camera_params["left"]["intrinsics"])]
    if "right" in camera_params:
        cameras.append(
            get_rs_camera(
                camera_params["right"]["intrinsics"],
                camera_params["right"]["extrinsics"],
            )
        )
    rig.cameras = cameras
    return rig
