"""Replay a ROS 1 (.bag) — or ROS 2 — recording onto live ROS 2 topics.

Reads the bag with the pure-python `rosbags` library (no ROS 1 install needed),
converts each message to its ROS 2 equivalent, and publishes it with the bag's
original timing so downstream nodes (e.g. clipseg_mask_node, Isaac VSLAM) see a
realistic stream for offline evaluation.

Requires (pip, install inside your container):  pip install rosbags

Examples (paths are inside the container, i.e. /root/colcon_ws/...)
------------------------------------------------------------------
# Inspect what's in the bag (topics, types, counts, duration):
ros2 run hound_core bag_replay /root/colcon_ws/bags/hound_25.bag --info

# Replay everything at real-time:
ros2 run hound_core bag_replay /root/colcon_ws/bags/hound_25.bag

# Replay only the color stream (already on the topic clipseg wants), looping:
ros2 run hound_core bag_replay /root/colcon_ws/bags/hound_25.bag \
    --topics /camera/color/image_raw --loop

# If a bag only has compressed color, decode it to a raw Image:
ros2 run hound_core bag_replay <bag> --decode-compressed
"""

import argparse
import importlib
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Field-name fallbacks: ROS 1 uses different names than ROS 2 for these.
_ALT_FIELDS = {
    "sec": ("secs",),
    "nanosec": ("nsecs",),
    "d": ("D",),
    "k": ("K",),
    "r": ("R",),
    "p": ("P",),
}

# msgtypes that behave like high-rate sensor data -> best-effort QoS by default.
_SENSOR_HINTS = ("Image", "Imu", "PointCloud", "LaserScan", "Range", "MagneticField")

_class_cache = {}


def _msg_class(msgtype: str):
    """Resolve 'pkg/msg/Type' (or 'pkg/Type') to a ROS 2 message class."""
    if msgtype in _class_cache:
        return _class_cache[msgtype]
    parts = msgtype.split("/")
    pkg, name = parts[0], parts[-1]
    mod = importlib.import_module(f"{pkg}.msg")
    cls = getattr(mod, name)
    _class_cache[msgtype] = cls
    return cls


def _getattr_alt(src, fname):
    if hasattr(src, fname):
        return getattr(src, fname)
    up = fname.upper()
    if hasattr(src, up):
        return getattr(src, up)
    for alt in _ALT_FIELDS.get(fname, ()):
        if hasattr(src, alt):
            return getattr(src, alt)
    return None


def _parse_type(ftype: str):
    """Return (base_type, is_array) from a rosidl field type string."""
    if ftype.startswith("sequence<"):
        inner = ftype[len("sequence<"):].rstrip(">")
        base = inner.split(",")[0].strip()
        return base, True
    if ftype.endswith("]"):
        return ftype[: ftype.index("[")], True
    return ftype, False


def _convert_value(val, base, is_array):
    if is_array:
        if base in ("uint8", "char", "octet") and hasattr(val, "tobytes"):
            return val.tobytes()
        if "/" in base:
            cls = _msg_class(base)
            return [_convert(v, cls) for v in val]
        if hasattr(val, "tolist"):
            return val.tolist()
        return list(val)
    if "/" in base:
        return _convert(val, _msg_class(base))
    return val


def _convert(src, dst_cls):
    """Reflectively copy a rosbags message `src` into a fresh ROS 2 `dst_cls`."""
    dst = dst_cls()
    for fname, ftype in dst.get_fields_and_field_types().items():
        sval = _getattr_alt(src, fname)
        if sval is None:
            continue
        base, is_array = _parse_type(ftype)
        try:
            setattr(dst, fname, _convert_value(sval, base, is_array))
        except Exception:  # noqa: BLE001 — keep field default on any mismatch
            pass
    return dst


def _decode_compressed_to_image(src):
    """Decode a sensor_msgs/CompressedImage into a raw bgr8 sensor_msgs/Image."""
    import numpy as np
    import cv2
    from sensor_msgs.msg import Image

    data = src.data
    buf = data.tobytes() if hasattr(data, "tobytes") else bytes(data)
    arr = np.frombuffer(buf, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return None
    out = Image()
    out.header = _convert(src.header, _msg_class("std_msgs/msg/Header"))
    out.height, out.width = int(img.shape[0]), int(img.shape[1])
    out.encoding = "bgr8"
    out.is_bigendian = 0
    out.step = out.width * 3
    out.data = img.tobytes()
    return out


def _is_sensor(msgtype: str) -> bool:
    return any(h in msgtype for h in _SENSOR_HINTS)


def cmd_info(reader):
    total = sum(c.msgcount for c in reader.connections)
    print(f"\nbag duration : {reader.duration / 1e9:.2f} s")
    print(f"start time   : {reader.start_time / 1e9:.3f}")
    print(f"end time     : {reader.end_time / 1e9:.3f}")
    print(f"message count: {total}\n")
    print(f"{'topic':<45} {'type':<38} {'count':>10}")
    print("-" * 95)
    for conn in sorted(reader.connections, key=lambda c: c.topic):
        print(f"{conn.topic:<45} {conn.msgtype:<38} {conn.msgcount:>10}")
    print()


def cmd_replay(reader, node, args):
    remap = {}
    for pair in args.remap or []:
        if ":=" not in pair:
            node.get_logger().warn(f"ignoring bad --remap '{pair}' (use old:=new)")
            continue
        src_t, dst_t = pair.split(":=", 1)
        remap[src_t] = dst_t

    wanted = set(args.topics) if args.topics else None
    conns = [c for c in reader.connections if wanted is None or c.topic in wanted]
    if not conns:
        node.get_logger().error("no matching topics in bag; use --info to list them")
        return

    pubs = {}

    def get_pub(topic, cls, msgtype):
        if topic not in pubs:
            if args.reliable:
                qos = 10
            elif args.best_effort:
                qos = qos_profile_sensor_data
            else:
                qos = qos_profile_sensor_data if _is_sensor(msgtype) else 10
            pubs[topic] = node.create_publisher(cls, topic, qos)
            node.get_logger().info(f"publishing {msgtype} on {topic}")
        return pubs[topic]

    bag_start = reader.start_time
    n_pub = 0

    def run_once():
        nonlocal n_pub
        wall0 = time.monotonic()
        for conn, timestamp, raw in reader.messages(connections=conns):
            if not rclpy.ok():
                return False
            rel = (timestamp - bag_start) / 1e9
            if args.start and rel < args.start:
                continue
            if args.duration and rel > args.start + args.duration:
                break

            # honour bag timing, scaled by --rate
            if args.rate > 0:
                target = (rel - (args.start or 0.0)) / args.rate
                ahead = target - (time.monotonic() - wall0)
                if ahead > 0:
                    time.sleep(min(ahead, 5.0))

            msg = reader.deserialize(raw, conn.msgtype)

            if args.decode_compressed and conn.msgtype.endswith("CompressedImage"):
                ros_msg = _decode_compressed_to_image(msg)
                if ros_msg is None:
                    continue
                out_topic = remap.get(conn.topic, conn.topic.replace("/compressed", ""))
                cls = _msg_class("sensor_msgs/msg/Image")
            else:
                cls = _msg_class(conn.msgtype)
                ros_msg = _convert(msg, cls)
                out_topic = remap.get(conn.topic, conn.topic)

            if args.retime and hasattr(ros_msg, "header"):
                ros_msg.header.stamp = node.get_clock().now().to_msg()

            get_pub(out_topic, cls, conn.msgtype).publish(ros_msg)
            n_pub += 1
        return True

    node.get_logger().info(
        f"replaying {len(conns)} topic(s) at {args.rate}x"
        + (" (looping)" if args.loop else "")
    )
    try:
        keep_going = run_once()
        while keep_going and args.loop and rclpy.ok():
            node.get_logger().info("loop: restarting bag")
            keep_going = run_once()
    except KeyboardInterrupt:
        pass
    node.get_logger().info(f"done — published {n_pub} messages")


def main():
    parser = argparse.ArgumentParser(description="Replay a ROS1/ROS2 bag onto ROS 2 topics.")
    parser.add_argument("bag", help="path to the .bag file (ROS 1) or bag directory (ROS 2)")
    parser.add_argument("--info", action="store_true", help="list topics/types/counts and exit")
    parser.add_argument("--topics", nargs="+", help="only replay these bag topics")
    parser.add_argument("--remap", nargs="+", help="topic remaps, e.g. /old:=/new")
    parser.add_argument("--rate", type=float, default=1.0, help="playback speed multiplier (default 1.0)")
    parser.add_argument("--loop", action="store_true", help="loop the bag forever")
    parser.add_argument("--start", type=float, default=0.0, help="start offset (s) into the bag")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds (0 = full bag)")
    parser.add_argument("--retime", action="store_true", help="stamp messages with current time instead of bag time")
    parser.add_argument("--decode-compressed", action="store_true", help="decode CompressedImage -> raw Image")
    parser.add_argument("--reliable", action="store_true", help="force RELIABLE QoS on all publishers")
    parser.add_argument("--best-effort", action="store_true", help="force BEST_EFFORT QoS on all publishers")
    args = parser.parse_args()

    try:
        from rosbags.highlevel import AnyReader
    except ImportError:
        print(
            "ERROR: the 'rosbags' package is required to read ROS 1 bags.\n"
            "Install it inside your container with:\n\n"
            "    pip install rosbags\n",
            file=sys.stderr,
        )
        sys.exit(1)

    from pathlib import Path

    bag_path = Path(args.bag)
    if not bag_path.exists():
        print(f"ERROR: bag not found: {bag_path}", file=sys.stderr)
        sys.exit(1)

    with AnyReader([bag_path]) as reader:
        if args.info:
            cmd_info(reader)
            return

        rclpy.init()
        node = rclpy.create_node("bag_replay")
        try:
            cmd_replay(reader, node, args)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
