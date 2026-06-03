#!/usr/bin/env python3
"""Print current dual-arm joint angles from /joint_states (radians and degrees)."""
from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

LEFT = [
    "Left_Shoulder_Pitch_Joint",
    "Left_Shoulder_Roll_Joint",
    "Left_Shoulder_Yaw_Joint",
    "Left_Elbow_Pitch_Joint",
    "Left_Wrist_Yaw_Joint",
    "Left_Wrist_Roll_Joint",
    "Left_Wrist_Pitch_Joint",
]
RIGHT = [
    "Right_Shoulder_Pitch_Joint",
    "Right_Shoulder_Roll_Joint",
    "Right_Shoulder_Yaw_Joint",
    "Right_Elbow_Pitch_Joint",
    "Right_Wrist_Yaw_Joint",
    "Right_Wrist_Roll_Joint",
    "Right_Wrist_Pitch_Joint",
]


class _OnceSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("read_joint_angles_once")
        self._msg: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg: JointState) -> None:
        self._msg = msg


def _lookup(msg: JointState, name: str) -> float | None:
    try:
        i = msg.name.index(name)
        return float(msg.position[i])
    except (ValueError, IndexError):
        return None


def _print_arm(title: str, names: list[str], msg: JointState) -> None:
    print(title)
    for n in names:
        rad = _lookup(msg, n)
        if rad is None:
            print(f"  {n}: (missing)")
            continue
        deg = math.degrees(rad)
        print(f"  {n}: {rad:+.4f} rad  ({deg:+.2f} deg)")


def main() -> int:
    rclpy.init()
    node = _OnceSubscriber()
    deadline = node.get_clock().now().nanoseconds + int(5e9)
    while node._msg is None and rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node._msg is None:
        print("超时: 5s 内未收到 /joint_states", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = node._msg
    print(f"stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
    _print_arm("=== 左臂 ===", LEFT, msg)
    _print_arm("=== 右臂 ===", RIGHT, msg)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
