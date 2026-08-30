#!/usr/bin/env python3
"""Estimate base_link ← Livox roll/pitch from bag-averaged IMU accelerations.

Averages /livox/imu and /hound_fcu_control/imu (EKF/FCU, FLU, base_link) over a
rosbag2. Both report specific force in m/s² (Livox already scaled from g). The
mean vectors should be the same gravity in two frames, so the rotation that maps
Livox accel → EKF accel is the same transform as SSoT lidar.xyz/rpy rotation
(imu_rpy is identity). Gravity does not observe a twist about the gravity axis;
roll/pitch are reported with SSoT yaw held fixed.

Example:
  ros2 run hound_core livox_ekf_imu_rpy.py --bag /path/to/bag
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

G = 9.80665


def rpy_deg_to_R(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """tf2 Quaternion.setRPY / static_transform_publisher (Rz Ry Rx)."""
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
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-8:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def rot_from_to(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Minimum-angle R with R @ a_hat = b_hat."""
    a = a / (np.linalg.norm(a) + 1e-15)
    b = b / (np.linalg.norm(b) + 1e-15)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        if c > 0.0:
            return np.eye(3)
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        axis = axis - a * np.dot(axis, a)
        axis /= np.linalg.norm(axis)
        return rot_about(axis, math.pi)
    k = np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
    )
    return np.eye(3) + k + k @ k * ((1.0 - c) / (s * s))


def rot_about(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / (np.linalg.norm(axis) + 1e-15)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def angle_between(u: np.ndarray, v: np.ndarray) -> float:
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    c = float(np.clip(np.dot(u, v) / (nu * nv + 1e-15), -1.0, 1.0))
    return math.degrees(math.acos(c))


def gravity_rpy_deg(a: np.ndarray) -> Tuple[float, float]:
    """Roll/pitch of a FLU frame vs +Z-up, treating mean accel as +up (specific force)."""
    n = a / (np.linalg.norm(a) + 1e-15)
    roll = math.degrees(math.atan2(n[1], n[2]))
    pitch = math.degrees(math.atan2(-n[0], math.hypot(n[1], n[2])))
    return roll, pitch


def fit_rp_fixed_yaw(
    a_src: np.ndarray, a_dst: np.ndarray, yaw_deg: float, seed_rp: Sequence[float]
) -> Tuple[float, float]:
    """Gauss–Newton roll/pitch so R(r,p,yaw) @ a_src ≈ a_dst."""
    u = a_src / (np.linalg.norm(a_src) + 1e-15)
    v = a_dst / (np.linalg.norm(a_dst) + 1e-15)
    r, p = float(seed_rp[0]), float(seed_rp[1])
    eps = 1e-3
    for _ in range(20):
        pred = rpy_deg_to_R(r, p, yaw_deg) @ u
        err = pred - v
        Jr = (rpy_deg_to_R(r + eps, p, yaw_deg) @ u - pred) / eps
        Jp = (rpy_deg_to_R(r, p + eps, yaw_deg) @ u - pred) / eps
        J = np.column_stack((Jr, Jp))
        step, *_ = np.linalg.lstsq(J, -err, rcond=None)
        r += float(step[0])
        p += float(step[1])
        if float(np.linalg.norm(step)) < 1e-6:
            break
    return wrap_deg(r), wrap_deg(p)


def open_bag(bag_path: str):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    last_err: Optional[Exception] = None
    for storage_id in ("mcap", "sqlite3"):
        reader = SequentialReader()
        try:
            reader.open(
                StorageOptions(uri=bag_path, storage_id=storage_id),
                ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr",
                ),
            )
            return reader
        except Exception as exc:  # noqa: BLE001
            last_err = exc
    raise RuntimeError(f"could not open bag {bag_path}: {last_err}")


def imu_xyz(msg) -> Tuple[np.ndarray, np.ndarray]:
    a = msg.linear_acceleration
    g = msg.angular_velocity
    return (
        np.array([a.x, a.y, a.z], dtype=float),
        np.array([g.x, g.y, g.z], dtype=float),
    )


def accumulate(
    bag_path: str,
    topics: Sequence[str],
    *,
    accel_gate: float,
    gyro_gate: float,
) -> Dict[str, dict]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_bag(bag_path)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in topic_types]
    if missing:
        known = ", ".join(sorted(topic_types)) or "(none)"
        raise RuntimeError(f"bag missing {missing}; topics: {known}")

    try:
        from rosbag2_py import StorageFilter

        reader.set_filter(StorageFilter(topics=list(topics)))
    except Exception:  # noqa: BLE001
        pass

    acc: Dict[str, dict] = {
        t: {
            "sum": np.zeros(3),
            "sum_gated": np.zeros(3),
            "n": 0,
            "n_gated": 0,
            "n_skip": 0,
        }
        for t in topics
    }
    decoders = {t: get_message(topic_types[t]) for t in topics}

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic not in acc:
            continue
        msg = deserialize_message(data, decoders[topic])
        a, w = imu_xyz(msg)
        slot = acc[topic]
        slot["sum"] += a
        slot["n"] += 1
        if accel_gate > 0.0 and abs(np.linalg.norm(a) - G) > accel_gate:
            slot["n_skip"] += 1
            continue
        if gyro_gate > 0.0 and np.linalg.norm(w) > gyro_gate:
            slot["n_skip"] += 1
            continue
        slot["sum_gated"] += a
        slot["n_gated"] += 1

    use_gate = all(slot["n_gated"] > 0 for slot in acc.values())
    for slot in acc.values():
        n = slot["n_gated"] if use_gate else slot["n"]
        s = slot["sum_gated"] if use_gate else slot["sum"]
        slot["mean"] = s / max(n, 1)
        slot["used_gate"] = use_gate
    return acc


def load_ssot_lidar_rpy(path: Path) -> Tuple[List[float], List[float]]:
    with path.open(encoding="utf-8") as f:
        ssot = yaml.safe_load(f)
    lidar = ssot.get("lidar") or {}
    rpy = [float(v) for v in (lidar.get("rpy") or [0.0, 0.0, 0.0])]
    imu_rpy = [float(v) for v in (lidar.get("imu_rpy") or [0.0, 0.0, 0.0])]
    return rpy, imu_rpy


def fmt(v: Sequence[float], nd: int = 3) -> str:
    return "[" + ", ".join(f"{float(x):.{nd}f}" for x in v) + "]"


def default_ssot() -> Path:
    here = Path(__file__).resolve()
    cand = here.parents[1] / "config" / "SSoT.yaml"
    return cand if cand.is_file() else Path("SSoT.yaml")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Average Livox and EKF/FCU IMU accel over a rosbag and estimate "
            "the roll-pitch of base_link ← livox (SSoT lidar.rpy)."
        )
    )
    parser.add_argument("--bag", required=True, help="rosbag2 directory")
    parser.add_argument("--livox-topic", default="/livox/imu")
    parser.add_argument(
        "--ekf-topic",
        default="/hound_fcu_control/imu",
        help="FCU/EKF IMU in base_link (FLU)",
    )
    parser.add_argument("--ssot", type=Path, default=default_ssot())
    parser.add_argument(
        "--accel-gate",
        type=float,
        default=2.0,
        help="Keep samples with ||a|-g| below this (m/s²). 0 = no gate",
    )
    parser.add_argument(
        "--gyro-gate",
        type=float,
        default=0.5,
        help="Keep samples with ||ω|| below this (rad/s). 0 = no gate",
    )
    args = parser.parse_args(argv)

    stats = accumulate(
        args.bag,
        (args.livox_topic, args.ekf_topic),
        accel_gate=args.accel_gate,
        gyro_gate=args.gyro_gate,
    )
    livox = stats[args.livox_topic]
    ekf = stats[args.ekf_topic]
    if livox["n"] == 0 or ekf["n"] == 0:
        print("no IMU samples on one or both topics", file=sys.stderr)
        return 1

    a_l = livox["mean"]
    a_e = ekf["mean"]
    if np.linalg.norm(a_l) < 1.0 or np.linalg.norm(a_e) < 1.0:
        print("mean accel too small; bag may be empty/corrupt", file=sys.stderr)
        return 1

    R_min = rot_from_to(a_l, a_e)
    rpy_min = R_to_rpy_deg(R_min)

    ssot_rpy = [178.901, -14.278, -0.513]
    imu_rpy = [0.0, 0.0, 0.0]
    if args.ssot.is_file():
        ssot_rpy, imu_rpy = load_ssot_lidar_rpy(args.ssot)
    R_ssot = rpy_deg_to_R(*ssot_rpy) @ rpy_deg_to_R(*imu_rpy)

    seed = (ssot_rpy[0], ssot_rpy[1])
    rp_fit = fit_rp_fixed_yaw(a_l, a_e, ssot_rpy[2], seed)
    rpy_fit = [rp_fit[0], rp_fit[1], ssot_rpy[2]]
    R_fit = rpy_deg_to_R(*rpy_fit)

    used = "gated" if livox.get("used_gate") else "all samples (gate empty)"
    print(f"bag: {args.bag}")
    print(f"mean from {used}  (g={G:.5f})")
    print(
        f"  {args.livox_topic}: n={livox['n']} gated={livox['n_gated']}  "
        f"mean={fmt(a_l, 4)}  |a|={np.linalg.norm(a_l):.4f}"
    )
    print(
        f"  {args.ekf_topic}: n={ekf['n']} gated={ekf['n_gated']}  "
        f"mean={fmt(a_e, 4)}  |a|={np.linalg.norm(a_e):.4f}"
    )
    print(
        f"gravity rpy (yaw unused)  livox={fmt(gravity_rpy_deg(a_l))}  "
        f"ekf={fmt(gravity_rpy_deg(a_e))}"
    )
    print()
    print(f"min-angle R  livox→base  rpy_deg: {fmt(rpy_min)}")
    print(f"SSoT lidar.rpy:           {fmt(ssot_rpy)}")
    if any(abs(x) > 1e-9 for x in imu_rpy):
        print(f"SSoT imu_rpy (composed):  {fmt(imu_rpy)}")
    print(f"fit roll/pitch, SSoT yaw: {fmt(rpy_fit)}")
    print()
    print(
        f"gravity residual  SSoT={angle_between(R_ssot @ a_l, a_e):.3f} deg  "
        f"min-R={angle_between(R_min @ a_l, a_e):.3f} deg  "
        f"fit={angle_between(R_fit @ a_l, a_e):.3f} deg"
    )
    d = [wrap_deg(rpy_fit[i] - ssot_rpy[i]) for i in range(3)]
    print(f"fit − SSoT rpy_deg:       {fmt(d)}")
    print()
    print("# suggested SSoT lidar.rpy (xyz unchanged, yaw from SSoT):")
    print(f"  rpy: {fmt(rpy_fit)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
