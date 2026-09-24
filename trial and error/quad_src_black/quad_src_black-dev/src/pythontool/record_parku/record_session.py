#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from quad.msg import Joystick, StateCommand
from rclpy.node import Node
from std_msgs.msg import Int8, Int8MultiArray, String
from tf2_msgs.msg import TFMessage

from common import quat_to_yaw


class CsvStream:
    def __init__(self, path: Path, header: list[str]):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.file = path.open("w", encoding="utf-8", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(header)
        self.file.flush()

    def write(self, row: list[object]) -> None:
        self.writer.writerow(row)
        self.file.flush()

    def close(self) -> None:
        self.file.close()


class RecordParkuSession(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("record_parku_session")
        self.output_dir: Path = args.output_dir.resolve()
        self.duration_sec: float = args.duration
        self.target_frame: str = args.target_frame
        self.child_frame: str = args.child_frame
        self.start_wall = time.time()
        self.t0 = self.get_clock().now()
        self.done = False

        self.pose = CsvStream(
            self.output_dir / "raw_pose.csv",
            ["time", "x", "y", "yaw"],
        )
        self.joy = CsvStream(
            self.output_dir / "raw_joy.csv",
            ["time"] + [f"axis_{i}" for i in range(6)] + [f"button_{i}" for i in range(23)],
        )
        self.cmd_evt = CsvStream(
            self.output_dir / "raw_cmd_evt.csv",
            ["time", "event_code"],
        )
        self.state_array = CsvStream(
            self.output_dir / "raw_state_array.csv",
            ["time", "mode", "lower_state", "lower_policy"],
        )
        self.task_state_command = CsvStream(
            self.output_dir / "raw_task_state_command.csv",
            ["time", "command_id"],
        )
        self.current_state = CsvStream(
            self.output_dir / "raw_current_state.csv",
            ["time", "state"],
        )
        self.cmd_vel = CsvStream(
            self.output_dir / "raw_cmd_vel.csv",
            ["time", "linear_x", "linear_y", "angular_z"],
        )

        self.streams = [
            self.pose,
            self.joy,
            self.cmd_evt,
            self.state_array,
            self.task_state_command,
            self.current_state,
            self.cmd_vel,
        ]

        self.create_subscription(TFMessage, args.tf_topic, self.tf_callback, 100)
        self.create_subscription(Joystick, args.joy_topic, self.joy_callback, 50)
        self.create_subscription(Int8, args.cmd_evt_topic, self.cmd_evt_callback, 50)
        self.create_subscription(Int8MultiArray, args.state_array_topic, self.state_array_callback, 50)
        self.create_subscription(StateCommand, args.task_state_command_topic, self.task_state_command_callback, 50)
        self.create_subscription(String, args.current_state_topic, self.current_state_callback, 50)
        self.create_subscription(Twist, args.cmd_vel_topic, self.cmd_vel_callback, 50)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.write_metadata(args)
        self.get_logger().info(f"Recording record_parku session to {self.output_dir}")

    def now_sec(self) -> float:
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def write_metadata(self, args: argparse.Namespace) -> None:
        metadata = {
            "created_unix_time": self.start_wall,
            "target_frame": args.target_frame,
            "child_frame": args.child_frame,
            "topics": {
                "tf": args.tf_topic,
                "joy": args.joy_topic,
                "cmd_evt": args.cmd_evt_topic,
                "state_array": args.state_array_topic,
                "task_state_command": args.task_state_command_topic,
                "current_state": args.current_state_topic,
                "cmd_vel": args.cmd_vel_topic,
            },
        }
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    def tf_callback(self, msg: TFMessage) -> None:
        t = self.now_sec()
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.target_frame
                and transform.child_frame_id == self.child_frame
            ):
                tr = transform.transform.translation
                q = transform.transform.rotation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                self.pose.write([
                    f"{t:.6f}",
                    f"{tr.x:.6f}",
                    f"{tr.y:.6f}",
                    f"{yaw:.6f}",
                ])
                return

    def joy_callback(self, msg: Joystick) -> None:
        axes = list(msg.axes)
        buttons = list(msg.buttons)
        self.joy.write(
            [f"{self.now_sec():.6f}"]
            + [f"{v:.6f}" for v in axes]
            + [int(v) for v in buttons]
        )

    def cmd_evt_callback(self, msg: Int8) -> None:
        self.cmd_evt.write([f"{self.now_sec():.6f}", int(msg.data)])

    def state_array_callback(self, msg: Int8MultiArray) -> None:
        values = list(msg.data) + ["", "", ""]
        self.state_array.write([f"{self.now_sec():.6f}", values[0], values[1], values[2]])

    def task_state_command_callback(self, msg: StateCommand) -> None:
        self.task_state_command.write([f"{self.now_sec():.6f}", int(msg.command_id)])

    def current_state_callback(self, msg: String) -> None:
        self.current_state.write([f"{self.now_sec():.6f}", msg.data])

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel.write([
            f"{self.now_sec():.6f}",
            f"{msg.linear.x:.6f}",
            f"{msg.linear.y:.6f}",
            f"{msg.angular.z:.6f}",
        ])

    def timer_callback(self) -> None:
        if self.duration_sec > 0 and self.now_sec() >= self.duration_sec:
            self.done = True

    def close(self) -> None:
        for stream in self.streams:
            stream.close()


def parse_args() -> argparse.Namespace:
    default_dir = Path(__file__).resolve().parent / "sessions" / time.strftime("%Y%m%d_%H%M%S")
    parser = argparse.ArgumentParser(description="Record pose, joystick, and high-level events for record_parku")
    parser.add_argument("-o", "--output-dir", type=Path, default=default_dir)
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; <=0 records until Ctrl+C")
    parser.add_argument("--target-frame", default="camera_init")
    parser.add_argument("--child-frame", default="aft_mapped")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--joy-topic", default="/quad/js_pub")
    parser.add_argument("--cmd-evt-topic", default="/quad/cmd_evt")
    parser.add_argument("--state-array-topic", default="/quad/state_array")
    parser.add_argument("--task-state-command-topic", default="/quad/task_state_command")
    parser.add_argument("--current-state-topic", default="/quad/current_state")
    parser.add_argument("--cmd-vel-topic", default="/quad/cmd_vel")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = RecordParkuSession(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted; closing session files.")
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

