#!/usr/bin/env python3
"""Interactively record obstacle key points for generated trajectories.

The output is a self-contained JSON file under:
  src/commands/task_manager/traj/generated/<session>/obstacle_points.json

All yaw values are lidar/map yaw in radians. In this robot's convention the
body heading used for visualization is lidar_yaw + pi/2.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy

from interactive_waypoint_recorder import (
    Pose,
    PoseSource,
    format_pose,
    refresh_pose,
    spin_for_pose,
)


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_ROOT = WORKSPACE / "src/commands/task_manager/traj/generated"
DEFAULT_SEQUENCE = [
    "start_transit",
    "pole",
    "sand",
    "crawl",
    "slope",
    "bridge_a",
    "bridge_b",
    "stairs",
    "wall",
    "end_transit",
]
PATH_OUTPUT_NAMES = {
    "pole": "pole_generated",
    "sand": "sand_generated",
    "slope": "slope_generated",
    "bridge_a": "bridge_a_generated",
    "bridge_b": "bridge_b_generated",
    "stairs": "stairs_generated",
}


@dataclass(frozen=True)
class PointStep:
    obstacle: str
    point: str
    text: str
    template: str
    optional: bool = False


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_sequence(raw: str) -> list[str]:
    sequence = [part.strip() for part in raw.split(",") if part.strip()]
    if not sequence:
        raise ValueError("--sequence cannot be empty")
    unknown = [name for name in sequence if name not in DEFAULT_SEQUENCE]
    if unknown:
        raise ValueError(f"unknown obstacle in --sequence: {', '.join(unknown)}")
    return sequence


def point_order_for_obstacle(name: str, pole_controls: int) -> tuple[str, list[str], set[str]]:
    if name in ("start_transit", "end_transit"):
        return "transit", ["start"], {"start"}
    if name == "pole":
        controls = [f"control_{index}" for index in range(1, pole_controls + 1)]
        return "spline", ["start", *controls, "end"], set()
    if name == "sand":
        return "l_shape", ["start", "corner", "end"], set()
    if name in ("slope", "bridge_a", "bridge_b", "stairs"):
        return "line", ["start", "end"], set()
    if name in ("crawl", "wall"):
        return "start_only", ["start"], set()
    raise ValueError(f"unknown obstacle: {name}")


def point_text(obstacle: str, point: str, template: str) -> str:
    if template == "spline" and point.startswith("control_"):
        return f"{obstacle}.{point} snake control point"
    if template == "l_shape" and point == "corner":
        return f"{obstacle}.corner L-shape corner"
    if point == "start":
        return f"{obstacle}.start obstacle/nav start point"
    if point == "end":
        return f"{obstacle}.end path end point"
    return f"{obstacle}.{point}"


def make_steps(sequence: list[str], pole_controls: int) -> list[PointStep]:
    steps: list[PointStep] = []
    for obstacle in sequence:
        template, points, optional_points = point_order_for_obstacle(obstacle, pole_controls)
        for point in points:
            optional = point in optional_points
            steps.append(
                PointStep(
                    obstacle=obstacle,
                    point=point,
                    text=point_text(obstacle, point, template),
                    template=template,
                    optional=optional,
                )
            )
    return steps


def pose_to_json(pose: Pose) -> dict[str, Any]:
    return {
        "x": round(pose.x, 6),
        "y": round(pose.y, 6),
        "yaw": round(pose.yaw, 6),
        "source": pose.source,
        "recorded_monotonic_s": round(pose.stamp_s, 6),
        "recorded_wall_time": datetime.now().isoformat(timespec="seconds"),
    }


def build_output_json(
    *,
    session: str,
    session_dir: Path,
    sequence: list[str],
    steps: list[PointStep],
    records: list[tuple[PointStep, Pose]],
    args: argparse.Namespace,
) -> dict[str, Any]:
    obstacles: dict[str, dict[str, Any]] = {}
    point_orders: dict[str, list[str]] = {}
    for step in steps:
        slot = obstacles.setdefault(
            step.obstacle,
            {
                "template": step.template,
                "points": {},
            },
        )
        if step.obstacle in PATH_OUTPUT_NAMES:
            base = PATH_OUTPUT_NAMES[step.obstacle]
            slot["outputs"] = {
                "raw": f"{base}_raw.txt",
                "smooth": f"{base}_smooth.txt",
            }
        point_orders.setdefault(step.obstacle, []).append(step.point)

    for step, pose in records:
        obstacles[step.obstacle]["points"][step.point] = pose_to_json(pose)

    for obstacle, point_order in point_orders.items():
        obstacles[obstacle]["point_order"] = point_order

    return {
        "schema": "obstacle_points.v1",
        "session": session,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "workspace": str(WORKSPACE),
        "session_dir": str(session_dir),
        "yaw_convention": "lidar_yaw_rad",
        "body_heading_note": "body_heading_rad = lidar_yaw_rad + pi/2",
        "pose_source": {
            "mode": args.source,
            "current_pose_topic": args.current_pose_topic,
            "tf_topic": args.tf_topic,
            "target_frame": args.target_frame,
            "child_frame": args.child_frame,
        },
        "sequence": sequence,
        "obstacles": obstacles,
    }


def write_points_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("auto", "current_pose", "tf"), default="auto")
    parser.add_argument("--current-pose-topic", default="/quad/current_pose")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--target-frame", default="camera_init")
    parser.add_argument("--child-frame", default="aft_mapped")
    parser.add_argument("--pose-timeout-s", type=float, default=1.0)
    parser.add_argument("--wait-pose-timeout-s", type=float, default=10.0)
    parser.add_argument("--sample-window-s", type=float, default=0.25)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--session", default=None, help="session directory name, default timestamp")
    parser.add_argument(
        "--sequence",
        default=",".join(DEFAULT_SEQUENCE),
        help="comma-separated obstacle sequence to record",
    )
    parser.add_argument(
        "--pole-controls",
        type=int,
        default=6,
        help="number of pole snake control points between start and end",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.pole_controls < 0:
        print("--pole-controls must be >= 0")
        return 1
    if args.pose_timeout_s <= 0 or args.wait_pose_timeout_s <= 0 or args.sample_window_s <= 0:
        print("pose timeout and sample window values must be positive")
        return 1

    try:
        sequence = parse_sequence(args.sequence)
    except ValueError as exc:
        print(f"Error: {exc}")
        return 1

    session = args.session or timestamp()
    session_dir = args.output_root.resolve() / session
    output_path = session_dir / "obstacle_points.json"
    steps = make_steps(sequence, args.pole_controls)
    records: list[tuple[PointStep, Pose]] = []

    rclpy.init()
    node = PoseSource(
        source=args.source,
        current_pose_topic=args.current_pose_topic,
        tf_topic=args.tf_topic,
        target_frame=args.target_frame,
        child_frame=args.child_frame,
    )
    try:
        print("Waiting for pose...")
        pose = spin_for_pose(node, args.wait_pose_timeout_s, args.pose_timeout_s)
        if pose is None:
            print("No pose received. Check /quad/current_pose or /tf.")
            return 1
        print(f"Pose ready: {format_pose(pose)}")
        print(f"Output: {output_path}")
        print("All yaw values are lidar yaw in radians.")

        idx = 0
        while idx < len(steps):
            step = steps[idx]
            optional_hint = ", s=skip optional" if step.optional else ""
            print(f"\n[{idx + 1}/{len(steps)}] Move robot to: {step.text}")
            print(f"Enter=save, p=print pose, u=undo{optional_hint}, q=quit")
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
                    print("This point is required.")
                continue
            if command:
                print("Unknown command.")
                continue

            pose = refresh_pose(node, args.sample_window_s, args.pose_timeout_s)
            if pose is None:
                print("No fresh pose; not saved.")
                continue
            records.append((step, pose))
            print(f"Saved {step.text}: {format_pose(pose)}")
            idx += 1

        data = build_output_json(
            session=session,
            session_dir=session_dir,
            sequence=sequence,
            steps=steps,
            records=records,
            args=args,
        )
        write_points_json(output_path, data)
        print(f"\nWrote {len(records)} points to {output_path}")
        print("Next:")
        print(f"  python3 src/pythontool/generate_obstacle_paths.py --points {output_path} --viz")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
