#!/usr/bin/env python3
"""Record anchor poses to example_point.txt via joystick SAVE/DELETE.

SAVE  (LB+RB, cmd_evt=19): append current TF pose (x y yaw) — only in MANUAL+RL_MOVE.
DELETE (LT+RT, cmd_evt=20): remove last non-comment line.

Recommended recording order: A1 -> B4 -> C1 -> C4 (last 4 lines used for generation).

Usage (with task_race.launch.py or standalone):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 record_anchor_points.py
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int8MultiArray
from tf2_msgs.msg import TFMessage

EVENT_SAVE = 19
EVENT_DELETE = 20
CONTROL_MODE_MANUAL = 0
STATE_RL_MOVE = 5

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_ANCHOR_FILE = (
    WORKSPACE
    / "src/commands/lidar_nav_demo_cpp/config/trajectories/example_point.txt"
)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def read_anchor_lines(path: Path) -> list[str]:
    if not path.is_file():
        return []
    lines: list[str] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            line = raw.rstrip("\n")
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            lines.append(line)
    return lines


def write_anchor_lines(path: Path, data_lines: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# Anchor points: x y yaw (one per line)\n"
        "# 5th from last = startup pickup; last 4 = A1, B4, C1, C4\n"
    )
    body = "\n".join(data_lines)
    suffix = "\n" if data_lines else ""
    path.write_text(header + body + suffix, encoding="utf-8")


class AnchorPointRecorder(Node):
    def __init__(
        self,
        anchor_file: Path,
        target_frame: str,
        child_frame: str,
        tf_topic: str,
        cooldown_s: float,
    ):
        super().__init__("anchor_point_recorder")
        self.anchor_file = anchor_file
        self.target_frame = target_frame
        self.child_frame = child_frame
        self.cooldown_s = cooldown_s

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got_tf = False

        self.control_mode = CONTROL_MODE_MANUAL
        self.lower_state = 0

        self.last_action_time = self.get_clock().now()

        self.event_sub = self.create_subscription(
            Int8, "/quad/cmd_evt", self.event_callback, 10
        )
        self.state_sub = self.create_subscription(
            Int8MultiArray, "/quad/state_array", self.state_callback, 10
        )
        self.tf_sub = self.create_subscription(TFMessage, tf_topic, self.tf_callback, 50)

        self.get_logger().info(f"Anchor file: {anchor_file}")
        self.get_logger().info(
            f"SAVE=LB+RB DELETE=LT+RT when MANUAL+RL_MOVE; TF {target_frame}->{child_frame}"
        )

    def state_callback(self, msg: Int8MultiArray) -> None:
        if len(msg.data) >= 2:
            self.control_mode = int(msg.data[0])
            self.lower_state = int(msg.data[1])

    def tf_callback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.target_frame
                and transform.child_frame_id == self.child_frame
            ):
                self.x = transform.transform.translation.x
                self.y = transform.transform.translation.y
                q = transform.transform.rotation
                self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                self.got_tf = True
                break

    def _armed(self) -> bool:
        return (
            self.control_mode == CONTROL_MODE_MANUAL
            and self.lower_state == STATE_RL_MOVE
        )

    def _cooldown_ok(self) -> bool:
        elapsed = (self.get_clock().now() - self.last_action_time).nanoseconds * 1e-9
        return elapsed >= self.cooldown_s

    def event_callback(self, msg: Int8) -> None:
        if msg.data == EVENT_SAVE:
            self._handle_save()
        elif msg.data == EVENT_DELETE:
            self._handle_delete()

    def _handle_save(self) -> None:
        if not self._armed():
            self.get_logger().warn("SAVE ignored: need MANUAL + RL_MOVE")
            return
        if not self._cooldown_ok():
            return
        if not self.got_tf:
            self.get_logger().warn("SAVE ignored: no TF yet")
            return

        lines = read_anchor_lines(self.anchor_file)
        lines.append(f"{self.x:.4f} {self.y:.4f} {self.yaw:.4f}")
        write_anchor_lines(self.anchor_file, lines)
        self.last_action_time = self.get_clock().now()
        self.get_logger().info(
            f"Saved anchor #{len(lines)}: x={self.x:.4f} y={self.y:.4f} yaw={self.yaw:.4f}"
        )

    def _handle_delete(self) -> None:
        if not self._armed():
            self.get_logger().warn("DELETE ignored: need MANUAL + RL_MOVE")
            return
        if not self._cooldown_ok():
            return

        lines = read_anchor_lines(self.anchor_file)
        if not lines:
            self.get_logger().warn("DELETE ignored: anchor file empty")
            return
        removed = lines.pop()
        write_anchor_lines(self.anchor_file, lines)
        self.last_action_time = self.get_clock().now()
        self.get_logger().info(f"Deleted last anchor, was: {removed}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--anchor-file", type=Path, default=DEFAULT_ANCHOR_FILE)
    parser.add_argument("--target-frame", default="camera_init")
    parser.add_argument("--child-frame", default="aft_mapped")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--cooldown-s", type=float, default=0.3)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = AnchorPointRecorder(
        anchor_file=args.anchor_file,
        target_frame=args.target_frame,
        child_frame=args.child_frame,
        tf_topic=args.tf_topic,
        cooldown_s=args.cooldown_s,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
