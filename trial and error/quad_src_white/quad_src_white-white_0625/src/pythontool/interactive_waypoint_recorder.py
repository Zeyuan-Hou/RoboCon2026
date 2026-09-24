#!/usr/bin/env python3
"""Interactive field waypoint recorder.

Default field mode records:
  optional startup pickup, pickup A1/A4/B4, dropoff C1/C4

It appends labeled rows to example_point.txt, then runs the existing field
waypoint generator to update manip_points_generated.yaml and waypoints_generated.yaml.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_ANCHOR_FILE = (
    WORKSPACE
    / "src/commands/lidar_nav_demo_cpp/config/trajectories/example_point.txt"
)
DEFAULT_LINE_OUTPUT = (
    WORKSPACE
    / "src/commands/lidar_nav_demo_cpp/config/trajectories/manual_line_points.txt"
)
GENERATOR = WORKSPACE / "src/pythontool/generate_field_waypoints.py"

# Field marking offsets in the robot body frame. Units: meters, meters, radians.
# forward: robot body +x, left: robot body +y.
# Saved point = measured TF point + body offset rotated by measured yaw.

# [1/6] Move robot to: startup pickup 可选
# Enter=save, p=print pose, u=undo, s=skip optional startup, q=quit
# > p
# x=0.4064 y=2.9680 yaw=3.1387 source=tf age=0.03s

# [1/6] Move robot to: startup pickup 可选
# Enter=save, p=print pose, u=undo, s=skip optional startup, q=quit
# > p
# x=0.4938 y=3.3358 yaw=-3.1385 source=tf age=0.09s todo 
PICKUP_OFFSET_FORWARD = -0.4078
PICKUP_OFFSET_LEFT = 0.0874
PICKUP_OFFSET_YAW = 0.0



# [1/6] Move robot to: startup pickup 可选
# Enter=save, p=print pose, u=undo, s=skip optional startup, q=quit
# > p
# x=-0.1411 y=5.4419 yaw=-0.0317 source=tf age=0.09s

# [1/6] Move robot to: startup pickup 可选
# Enter=save, p=print pose, u=undo, s=skip optional startup, q=quit
# > p
# x=-0.2999 y=5.0992 yaw=0.0176 source=tf age=0.06s
DROPOFF_OFFSET_FORWARD = -0.4027
DROPOFF_OFFSET_LEFT = 0.1588
DROPOFF_OFFSET_YAW = 0.0

PICKUP_LABEL_TEXT = {
    "A1": "A1 前排取货1",
    "A2": "A2 前排取货2",
    "A3": "A3 前排取货3",
    "A4": "A4 前排取货4",
    "B1": "B1 后排取货1",
    "B2": "B2 后排取货2",
    "B3": "B3 后排取货3",
    "B4": "B4 后排取货4",
}
DROPOFF_LABEL_TEXT = {
    "C1": "C1 放货1低层",
    "C2": "C2 放货2低层",
    "C3": "C3 放货3低层",
    "C4": "C4 放货4低层",
}


@dataclass
class Pose:
    x: float
    y: float
    yaw: float
    stamp_s: float
    source: str


@dataclass
class RecordStep:
    label: str
    text: str
    optional: bool = False


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_labels(raw: str, valid: dict[str, str], name: str) -> list[str]:
    labels = [part.strip().upper() for part in raw.split(",") if part.strip()]
    if not labels:
        raise ValueError(f"{name} labels cannot be empty")
    invalid = [label for label in labels if label not in valid]
    if invalid:
        raise ValueError(f"Invalid {name} labels: {', '.join(invalid)}")
    if len(set(labels)) != len(labels):
        raise ValueError(f"Duplicate {name} labels are not allowed")
    return labels


class PoseSource(Node):
    def __init__(
        self,
        source: str = "tf",
        current_pose_topic: str | None = None,
        tf_topic: str = "/tf",
        target_frame: str = "camera_init",
        child_frame: str = "aft_mapped",
    ):
        super().__init__("interactive_waypoint_recorder")
        # Keep source/current_pose_topic accepted for older callers, but this
        # recorder intentionally uses only TF in the same [x, y, yaw] format
        # published by /quad/current_pose in lidar_nav_demo_cpp.
        _ = source, current_pose_topic
        self.target_frame = target_frame
        self.child_frame = child_frame
        self.tf_pose: Pose | None = None
        self.create_subscription(TFMessage, tf_topic, self.tf_callback, 50)

    def tf_callback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.target_frame
                and transform.child_frame_id == self.child_frame
            ):
                trans = transform.transform.translation
                rot = transform.transform.rotation
                self.tf_pose = Pose(
                    float(trans.x),
                    float(trans.y),
                    quat_to_yaw(rot.x, rot.y, rot.z, rot.w),
                    time.monotonic(),
                    "tf",
                )
                break

    def best_pose(self, timeout_s: float) -> Pose | None:
        now = time.monotonic()
        pose = self.tf_pose
        return pose if pose and now - pose.stamp_s <= timeout_s else None


def spin_for_pose(node: PoseSource, timeout_s: float, source_timeout_s: float) -> Pose | None:
    deadline = time.monotonic() + timeout_s
    pose = node.best_pose(source_timeout_s)
    while rclpy.ok() and pose is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        pose = node.best_pose(source_timeout_s)
    return pose


def refresh_pose(node: PoseSource, duration_s: float, source_timeout_s: float) -> Pose | None:
    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
    return node.best_pose(source_timeout_s)


def format_pose(pose: Pose) -> str:
    age = time.monotonic() - pose.stamp_s
    return (
        f"x={pose.x:.4f} y={pose.y:.4f} yaw={pose.yaw:.4f} "
        f"source={pose.source} age={age:.2f}s"
    )


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def offset_for_step(step: RecordStep) -> tuple[float, float, float]:
    if step.label.startswith("startup:") or step.label.startswith("pickup:"):
        return PICKUP_OFFSET_FORWARD, PICKUP_OFFSET_LEFT, PICKUP_OFFSET_YAW
    if step.label.startswith("dropoff:"):
        return DROPOFF_OFFSET_FORWARD, DROPOFF_OFFSET_LEFT, DROPOFF_OFFSET_YAW
    return 0.0, 0.0, 0.0


def body_offset_to_map(forward: float, left: float, yaw: float) -> tuple[float, float]:
    # Same convention as lidar_nav_demo_cpp: body.x is forward, body.y is left.
    return (
        -math.sin(yaw) * forward - math.cos(yaw) * left,
        math.cos(yaw) * forward - math.sin(yaw) * left,
    )


def apply_field_offset(step: RecordStep, pose: Pose) -> Pose:
    forward, left, dyaw = offset_for_step(step)
    dx, dy = body_offset_to_map(forward, left, pose.yaw)
    return Pose(
        pose.x + dx,
        pose.y + dy,
        normalize_angle(pose.yaw + dyaw),
        pose.stamp_s,
        f"{pose.source}+body_offset",
    )


def make_field_steps(pickup_labels: list[str], dropoff_labels: list[str]) -> list[RecordStep]:
    steps = [RecordStep("startup:pickup", "startup pickup 可选", optional=True)]
    for label in pickup_labels:
        steps.append(RecordStep(f"pickup:{label}", PICKUP_LABEL_TEXT[label]))
    for label in dropoff_labels:
        steps.append(RecordStep(f"dropoff:{label}", DROPOFF_LABEL_TEXT[label]))
    return steps


def parse_offline_records(
    input_path: Path,
    steps: list[RecordStep],
) -> list[tuple[RecordStep, Pose]]:
    expected_by_label = {step.label: step for step in steps}
    records_by_label: dict[str, Pose] = {}

    try:
        lines = input_path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ValueError(f"Cannot read offline input {input_path}: {exc}") from exc

    for line_no, raw_line in enumerate(lines, start=1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 4:
            raise ValueError(
                f"{input_path}:{line_no}: expected 'label x y yaw', got {raw_line!r}"
            )

        label = parts[0]
        if label not in expected_by_label:
            expected = ", ".join(expected_by_label)
            raise ValueError(
                f"{input_path}:{line_no}: unexpected label {label!r}; "
                f"expected one of: {expected}"
            )
        if label in records_by_label:
            raise ValueError(f"{input_path}:{line_no}: duplicate label {label!r}")

        try:
            x = float(parts[1])
            y = float(parts[2])
            yaw = float(parts[3])
        except ValueError as exc:
            raise ValueError(
                f"{input_path}:{line_no}: x/y/yaw must be numeric"
            ) from exc

        records_by_label[label] = Pose(x, y, yaw, time.monotonic(), "offline")

    missing = [step.label for step in steps if step.label not in records_by_label]
    if missing:
        raise ValueError(
            f"{input_path}: missing required label(s): {', '.join(missing)}"
        )

    return [(step, records_by_label[step.label]) for step in steps]


def append_field_records(anchor_file: Path, records: list[tuple[RecordStep, Pose]]) -> None:
    anchor_file.parent.mkdir(parents=True, exist_ok=True)
    with anchor_file.open("a", encoding="utf-8") as handle:
        handle.write("\n# interactive_waypoint_recorder session\n")
        for step, pose in records:
            handle.write(
                f"{pose.x:.4f} {pose.y:.4f} {pose.yaw:.4f} # label={step.label}\n"
            )


def write_line_records(output: Path, poses: list[Pose]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as handle:
        handle.write("# Manual line points: index x y yaw_rad\n")
        for idx, pose in enumerate(poses):
            handle.write(f"{idx} {pose.x:.6f} {pose.y:.6f} {pose.yaw:.6f}\n")


def run_generator(anchor_file: Path, *, viz: bool) -> int:
    cmd = [
        sys.executable,
        str(GENERATOR),
        "--anchors",
        str(anchor_file),
    ]
    if viz:
        cmd.append("--viz")
    return subprocess.call(cmd, cwd=str(WORKSPACE))


def record_field(args: argparse.Namespace, node: PoseSource) -> int:
    pickup_labels = parse_labels(args.pickup_labels, PICKUP_LABEL_TEXT, "pickup")
    dropoff_labels = parse_labels(args.dropoff_labels, DROPOFF_LABEL_TEXT, "dropoff")
    if len(pickup_labels) < 3:
        raise ValueError("field mode needs at least 3 pickup labels")
    if len(dropoff_labels) < 2:
        raise ValueError("field mode needs at least 2 dropoff labels")

    steps = make_field_steps(pickup_labels, dropoff_labels)
    records: list[tuple[RecordStep, Pose]] = []

    print("Waiting for pose...")
    pose = spin_for_pose(node, args.wait_pose_timeout_s, args.pose_timeout_s)
    if pose is None:
        print("No pose received. Check /tf.", file=sys.stderr)
        return 1

    idx = 0
    while idx < len(steps):
        step = steps[idx]
        print(f"\n[{idx + 1}/{len(steps)}] Move robot to: {step.text}")
        print("Enter=save, p=print pose, u=undo, s=skip optional startup, q=quit")
        command = input("> ").strip().lower()
        if command == "q":
            print("Quit without writing.")
            return 1
        if command == "p":
            pose = refresh_pose(node, args.sample_window_s, args.pose_timeout_s)
            print(format_pose(pose) if pose else "No fresh pose")
            continue
        if command == "u":
            if records:
                removed_step, removed_pose = records.pop()
                idx = max(0, idx - 1)
                print(f"Removed {removed_step.text}: {format_pose(removed_pose)}")
            else:
                print("Nothing to undo.")
            continue
        if command == "s":
            if step.optional:
                print(f"Skipped {step.text}")
                idx += 1
            else:
                print("Only optional startup can be skipped.")
            continue
        if command:
            print("Unknown command.")
            continue

        pose = refresh_pose(node, args.sample_window_s, args.pose_timeout_s)
        if pose is None:
            print("No fresh pose; not saved.")
            continue
        saved_pose = apply_field_offset(step, pose)
        records.append((step, saved_pose))
        print(f"Measured {step.text}: {format_pose(pose)}")
        print(f"Saved {step.text}: {format_pose(saved_pose)}")
        idx += 1

    append_field_records(args.anchor_file, records)
    print(f"\nAppended {len(records)} records to {args.anchor_file}")

    if args.no_generate:
        return 0
    return run_generator(args.anchor_file, viz=not args.no_viz)


def record_offline(args: argparse.Namespace) -> int:
    pickup_labels = parse_labels(args.pickup_labels, PICKUP_LABEL_TEXT, "pickup")
    dropoff_labels = parse_labels(args.dropoff_labels, DROPOFF_LABEL_TEXT, "dropoff")
    if len(pickup_labels) < 3:
        raise ValueError("offline mode needs at least 3 pickup labels")
    if len(dropoff_labels) < 2:
        raise ValueError("offline mode needs at least 2 dropoff labels")

    steps = make_field_steps(pickup_labels, dropoff_labels)
    try:
        measured_records = parse_offline_records(args.offline_input, steps)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    saved_records: list[tuple[RecordStep, Pose]] = []
    for step, measured_pose in measured_records:
        saved_pose = apply_field_offset(step, measured_pose)
        saved_records.append((step, saved_pose))
        print(f"Measured {step.text}: {format_pose(measured_pose)}")
        print(f"Saved {step.text}: {format_pose(saved_pose)}")

    append_field_records(args.anchor_file, saved_records)
    print(f"\nAppended {len(saved_records)} records to {args.anchor_file}")

    if args.no_generate:
        return 0
    return run_generator(args.anchor_file, viz=not args.no_viz)


def record_line(args: argparse.Namespace, node: PoseSource) -> int:
    poses: list[Pose] = []
    print("Line mode: Enter=save point, p=print pose, u=undo, done=write, q=quit")
    while True:
        command = input("> ").strip().lower()
        if command == "q":
            print("Quit without writing.")
            return 1
        if command == "done":
            if len(poses) < 2:
                print("Need at least 2 points before done.")
                continue
            write_line_records(args.output, poses)
            print(f"Wrote {len(poses)} points to {args.output}")
            return 0
        if command == "p":
            pose = refresh_pose(node, args.sample_window_s, args.pose_timeout_s)
            print(format_pose(pose) if pose else "No fresh pose")
            continue
        if command == "u":
            if poses:
                pose = poses.pop()
                print(f"Removed point {len(poses)}: {format_pose(pose)}")
            else:
                print("Nothing to undo.")
            continue
        if command:
            print("Unknown command.")
            continue

        pose = refresh_pose(node, args.sample_window_s, args.pose_timeout_s)
        if pose is None:
            print("No fresh pose; not saved.")
            continue
        poses.append(pose)
        print(f"Saved point {len(poses) - 1}: {format_pose(pose)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("field", "line", "offline"), default="field")
    parser.add_argument(
        "--source",
        choices=("auto", "current_pose", "tf"),
        default="tf",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--current-pose-topic",
        default="/quad/current_pose",
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--target-frame", default="camera_init")
    parser.add_argument("--child-frame", default="aft_mapped")
    parser.add_argument("--pose-timeout-s", type=float, default=1.0)
    parser.add_argument("--wait-pose-timeout-s", type=float, default=10.0)
    parser.add_argument("--sample-window-s", type=float, default=0.25)
    parser.add_argument("--anchor-file", type=Path, default=DEFAULT_ANCHOR_FILE)
    parser.add_argument(
        "--offline-input",
        type=Path,
        default=None,
        help="offline mode input file: label x y yaw per line",
    )
    parser.add_argument("--pickup-labels", default="A1,A4,B4")
    parser.add_argument("--dropoff-labels", default="C1,C4")
    parser.add_argument("--no-generate", action="store_true")
    parser.add_argument("--no-viz", action="store_true")
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_LINE_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.anchor_file = args.anchor_file.resolve()
    if args.offline_input is not None:
        args.offline_input = args.offline_input.resolve()
    args.output = args.output.resolve()

    if args.mode == "offline":
        if args.offline_input is None:
            print("--offline-input is required when --mode offline", file=sys.stderr)
            return 1
        return record_offline(args)

    rclpy.init()
    node = PoseSource(
        tf_topic=args.tf_topic,
        target_frame=args.target_frame,
        child_frame=args.child_frame,
        source=args.source,
        current_pose_topic=args.current_pose_topic,
    )
    try:
        if args.mode == "field":
            return record_field(args, node)
        return record_line(args, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
