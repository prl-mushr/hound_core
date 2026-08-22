#!/usr/bin/env python3
"""Convert direct_visual_lidar_calibration results into HOUND SSoT mounts.

Reads each camera's preprocessed/calib.json ``results.T_lidar_camera``
(x, y, z, qx, qy, qz, qw) where p_lidar = T_lidar_camera * p_camera and
camera = color optical frame. Emits base_link→{cam}_link as xyz (m) + rpy (deg)
into stereo_composite.cameras.* (and legacy hal_monitor.camera pos/rot if present).

Default: dry-run print. Pass --write to patch SSoT.yaml.

Examples:
  apply_vlcal_to_ssot.py
  apply_vlcal_to_ssot.py --calib-root /home/hound/colcon_ws/calib/vlcal --write
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

CAMERAS = ("camera_front", "camera_left", "camera_right")


def rpy_deg_to_R(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Intrinsic XYZ (roll-pitch-yaw) → 3x3, matching tf2 Quaternion.setRPY."""
    r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def R_to_rpy_deg(R: np.ndarray) -> Tuple[float, float, float]:
    """Inverse of rpy_deg_to_R (tf2 / Eigen ZYX intrinsic ≈ setRPY)."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-8:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))  # type: ignore[return-value]


def quat_xyzw_to_R(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qz * qw),
                2 * (qx * qz + qy * qw),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ],
        dtype=float,
    )


def R_to_quat_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    t = np.trace(R)
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    return (float(qx), float(qy), float(qz), float(qw))


def T_from_xyz_quat(xyz: List[float], q_xyzw: List[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_xyzw_to_R(*q_xyzw)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def T_from_xyz_rpy_deg(xyz: List[float], rpy_deg: List[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_deg_to_R(*rpy_deg)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def T_inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def T_to_xyz_rpy_deg(T: np.ndarray) -> Tuple[List[float], List[float]]:
    xyz = [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]
    rpy = list(R_to_rpy_deg(T[:3, :3]))
    return xyz, rpy


def fmt_list(vals: List[float], nd: int = 6) -> str:
    return "[" + ", ".join(f"{v:.{nd}f}" for v in vals) + "]"


def default_T_link_optical() -> np.ndarray:
    """ROS camera optical (zfwd/xright/ydown) at same origin as FLU link (xfwd/yleft/zup).

    Used when TF lookup is unavailable. RealSense color has a small baseline vs
    infra1; prefer live TF when cameras are running.
    """
    T = np.eye(4)
    # Columns = optical basis in link: x_opt→-Y, y_opt→-Z, z_opt→+X
    T[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=float,
    )
    return T


def lookup_T_link_optical(camera: str) -> Optional[np.ndarray]:
    """Try ROS TF: {cam}_link → {cam}_color_optical_frame."""
    try:
        import rclpy
        from rclpy.duration import Duration
        from tf2_ros import Buffer, TransformListener
    except ImportError:
        return None

    link = f"{camera}_link"
    optical = f"{camera}_color_optical_frame"
    rclpy.init(args=None)
    node = rclpy.create_node("vlcal_tf_lookup")
    buf = Buffer()
    TransformListener(buf, node)
    try:
        deadline = node.get_clock().now() + Duration(seconds=3.0)
        while rclpy.ok() and node.get_clock().now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if buf.can_transform(link, optical, rclpy.time.Time()):
                break
        if not buf.can_transform(link, optical, rclpy.time.Time()):
            return None
        tf = buf.lookup_transform(link, optical, rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        return T_from_xyz_quat(
            [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
        )
    except Exception as exc:  # noqa: BLE001
        print(f"[vlcal] TF lookup {link}→{optical} failed: {exc}", file=sys.stderr)
        return None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def load_ssot(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_T_lidar_camera(calib_json: Path) -> np.ndarray:
    data = json.loads(calib_json.read_text(encoding="utf-8"))
    arr = data.get("results", {}).get("T_lidar_camera")
    if not arr or len(arr) != 7:
        raise ValueError(f"Missing results.T_lidar_camera[7] in {calib_json}")
    x, y, z, qx, qy, qz, qw = arr
    return T_from_xyz_quat([x, y, z], [qx, qy, qz, qw])


def patch_ssot_text(
    text: str,
    *,
    front_xyz: List[float],
    front_rpy: List[float],
    front_quat: List[float],
    left_xyz: Optional[List[float]],
    left_rpy: Optional[List[float]],
    right_xyz: Optional[List[float]],
    right_rpy: Optional[List[float]],
) -> str:
    # HAL body→front: under hal_monitor.camera
    # Match "    pos:" / "    rot:" that follow camera: block — use unique-ish values
    # by replacing the known hal_monitor camera pos/rot (4-space then pos).
    # Safer: replace first occurrence of "    pos: [" after "  camera:" in hal_monitor.

    # Use markers unique to SSoT layout.
    # front HAL (pos/rot) + stereo_composite front mount (xyz/rpy, base_link)
    text = _replace_hal_camera_pose(text, front_xyz, front_quat)
    text = _replace_camera_mount(text, "camera_front", front_xyz, front_rpy)

    if left_xyz is not None and left_rpy is not None:
        text = _replace_camera_mount(text, "camera_left", left_xyz, left_rpy)
    if right_xyz is not None and right_rpy is not None:
        text = _replace_camera_mount(text, "camera_right", right_xyz, right_rpy)
    return text


def _replace_hal_camera_pose(
    text: str, xyz: List[float], quat_xyzw: List[float]
) -> str:
    """Patch hal_monitor.camera pos/rot if present; no-op when HAL only monitors FPS."""
    lines = text.splitlines(keepends=True)
    out: List[str] = []
    in_hal = False
    in_cam = False
    replaced_pos = replaced_rot = False
    for line in lines:
        if line.startswith("hal_monitor:"):
            in_hal = True
            in_cam = False
        elif in_hal and line and not line.startswith(" ") and not line.startswith("#"):
            # Top-level key after hal_monitor
            in_hal = False
            in_cam = False
        elif in_hal and line.startswith("  camera:"):
            in_cam = True
        elif in_hal and in_cam and line.startswith("  ") and not line.startswith("    ") and line.strip().endswith(":"):
            # Sibling of camera under hal_monitor (e.g. vesc:)
            in_cam = False
        elif in_hal and in_cam and not replaced_pos and line.lstrip().startswith("pos:"):
            indent = line[: len(line) - len(line.lstrip())]
            out.append(f"{indent}pos: {fmt_list(xyz)}\n")
            replaced_pos = True
            continue
        elif in_hal and in_cam and not replaced_rot and line.lstrip().startswith("rot:"):
            indent = line[: len(line) - len(line.lstrip())]
            out.append(f"{indent}rot: {fmt_list(quat_xyzw)}\n")
            replaced_rot = True
            continue
        out.append(line)
    if replaced_pos and replaced_rot:
        return "".join(out)
    if not replaced_pos and not replaced_rot:
        return text
    raise RuntimeError("Failed to patch hal_monitor.camera pos/rot (partial match)")


def _replace_camera_mount(
    text: str, camera: str, xyz: List[float], rpy: List[float]
) -> str:
    lines = text.splitlines(keepends=True)
    out: List[str] = []
    in_cam = False
    replaced_xyz = replaced_rpy = False
    header = f"    {camera}:"
    for line in lines:
        if line.startswith(header):
            in_cam = True
            replaced_xyz = replaced_rpy = False
            out.append(line)
            continue
        if in_cam and line.startswith("    camera_") and not line.startswith(header):
            in_cam = False
        if in_cam and line.startswith("lidar:"):
            in_cam = False
        if in_cam and not replaced_xyz and line.lstrip().startswith("xyz:"):
            indent = line[: len(line) - len(line.lstrip())]
            comment = ""
            if "#" in line:
                comment = "  #" + line.split("#", 1)[1].rstrip("\n")
            out.append(f"{indent}xyz: {fmt_list(xyz)}{comment}\n")
            replaced_xyz = True
            continue
        if in_cam and not replaced_rpy and line.lstrip().startswith("rpy:"):
            indent = line[: len(line) - len(line.lstrip())]
            comment = ""
            if "#" in line:
                comment = "  #" + line.split("#", 1)[1].rstrip("\n")
            out.append(f"{indent}rpy: {fmt_list(rpy, nd=3)}{comment}\n")
            replaced_rpy = True
            in_cam = False
            continue
        out.append(line)
    if not (replaced_xyz and replaced_rpy):
        raise RuntimeError(f"Failed to patch {camera} xyz/rpy")
    return "".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--calib-root",
        type=Path,
        default=Path("/home/hound/colcon_ws/calib/vlcal"),
    )
    ap.add_argument(
        "--ssot",
        type=Path,
        default=Path("/home/hound/colcon_ws/src/hound_core/config/SSoT.yaml"),
    )
    ap.add_argument(
        "--write",
        action="store_true",
        help="Patch SSoT.yaml in place (default: dry-run print only)",
    )
    ap.add_argument(
        "--no-tf",
        action="store_true",
        help="Skip ROS TF lookup; use optical↔FLU rotation fallback",
    )
    args = ap.parse_args()

    ssot = load_ssot(args.ssot)
    lidar = ssot.get("lidar") or {}
    lidar_xyz = list(lidar.get("xyz") or [0.0, 0.0, 0.0])
    lidar_rpy = list(lidar.get("rpy") or [0.0, 0.0, 0.0])
    T_base_lidar = T_from_xyz_rpy_deg(lidar_xyz, lidar_rpy)

    print(f"[vlcal] SSoT lidar xyz={fmt_list(lidar_xyz)} rpy_deg={fmt_list(lidar_rpy, 3)}")
    print(f"[vlcal] calib root: {args.calib_root}")

    T_base_link: Dict[str, np.ndarray] = {}
    sources: Dict[str, str] = {}

    for cam in CAMERAS:
        calib_json = args.calib_root / cam / "preprocessed" / "calib.json"
        if not calib_json.is_file():
            print(f"[vlcal] skip {cam}: missing {calib_json}")
            continue
        T_lidar_optical = load_T_lidar_camera(calib_json)
        if args.no_tf:
            T_link_optical = default_T_link_optical()
            src = "fallback optical↔FLU"
        else:
            T_lo = lookup_T_link_optical(cam)
            if T_lo is None:
                T_link_optical = default_T_link_optical()
                src = "fallback optical↔FLU (TF unavailable)"
            else:
                T_link_optical = T_lo
                src = "TF {cam}_link→color_optical"

        # p_lidar = T_lidar_optical * p_optical
        # p_optical = inv(T_link_optical) * p_link  → T_lidar_link = T_lidar_optical * inv(T_link_optical)
        T_lidar_link = T_lidar_optical @ T_inv(T_link_optical)
        T_base_cam = T_base_lidar @ T_lidar_link
        T_base_link[cam] = T_base_cam
        sources[cam] = src

        xyz, rpy = T_to_xyz_rpy_deg(T_base_cam)
        print(f"\n=== {cam} ===")
        print(f"  optical source: {src}")
        print(f"  base_link → {cam}_link")
        print(f"    xyz: {fmt_list(xyz)}")
        print(f"    rpy_deg: {fmt_list(rpy, 3)}")

    if "camera_front" not in T_base_link:
        print(
            "\n[vlcal] Need camera_front calib.json to write HAL / side-relative mounts.",
            file=sys.stderr,
        )
        return 1

    T_base_front = T_base_link["camera_front"]
    front_xyz, front_rpy = T_to_xyz_rpy_deg(T_base_front)
    front_quat = list(R_to_quat_xyzw(T_base_front[:3, :3]))

    print("\n=== SSoT mapping ===")
    print("stereo_composite.cameras.camera_front")
    print("  (base_link → camera_front_link; EKF owns odom→base_link TF):")
    print(f"  pos/xyz: {fmt_list(front_xyz)}")
    print(f"  rot: {fmt_list(front_quat)}  # xyzw")
    print(f"  rpy_deg: {fmt_list(front_rpy, 3)}")

    side_xyz_rpy: Dict[str, Tuple[List[float], List[float]]] = {}
    for side in ("camera_left", "camera_right"):
        if side not in T_base_link:
            continue
        T_front_side = T_inv(T_base_front) @ T_base_link[side]
        xyz, rpy = T_to_xyz_rpy_deg(T_front_side)
        side_xyz_rpy[side] = (xyz, rpy)
        print(f"stereo_composite.cameras.{side} (camera_front_link → {side}_link):")
        print(f"  xyz: {fmt_list(xyz)}")
        print(f"  rpy_deg: {fmt_list(rpy, 3)}")

    if not args.write:
        print("\n[vlcal] dry-run only. Re-run with --write to patch SSoT.yaml")
        return 0

    raw = args.ssot.read_text(encoding="utf-8")
    left = side_xyz_rpy.get("camera_left")
    right = side_xyz_rpy.get("camera_right")
    patched = patch_ssot_text(
        raw,
        front_xyz=front_xyz,
        front_rpy=front_rpy,
        front_quat=front_quat,
        left_xyz=left[0] if left else None,
        left_rpy=left[1] if left else None,
        right_xyz=right[0] if right else None,
        right_rpy=right[1] if right else None,
    )
    bak = args.ssot.with_suffix(args.ssot.suffix + ".bak_vlcal")
    bak.write_text(raw, encoding="utf-8")
    args.ssot.write_text(patched, encoding="utf-8")
    print(f"\n[vlcal] wrote {args.ssot} (backup {bak})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
