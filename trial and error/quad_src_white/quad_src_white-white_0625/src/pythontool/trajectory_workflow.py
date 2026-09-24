#!/usr/bin/env python3
"""
Interactive obstacle trajectory recording workflow.

This tool keeps every recording attempt under:
  src/commands/task_manager/traj/records/<obstacle>/<timestamp>/

Each attempt writes raw, trimmed, smoothed, and record_info.txt files. Only
attempts marked successful are copied to the official trajectory file currently
referenced by obstacle_race_params.yaml.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import smooth_trajectory as smooth


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent
TRAJ_DIR = WORKSPACE_ROOT / "src/commands/task_manager/traj"
RECORDS_DIR = TRAJ_DIR / "records"


@dataclass(frozen=True)
class ObstacleSpec:
    name: str
    title: str
    official_path: Path


OBSTACLES: tuple[ObstacleSpec, ...] = (
    ObstacleSpec("start_transit", "start transit", TRAJ_DIR / "start1_smooth.txt"),
    ObstacleSpec("pole", "pole", TRAJ_DIR / "pole_final.txt"),
    ObstacleSpec("sand", "sand", TRAJ_DIR / "sand_final.txt"),
    ObstacleSpec("slope", "slope", TRAJ_DIR / "slope_final.txt"),
    ObstacleSpec("bridge_a", "bridge A", TRAJ_DIR / "bridge_a_final.txt"),
    ObstacleSpec("bridge_b", "bridge B", TRAJ_DIR / "bridge_b_final.txt"),
    ObstacleSpec("bridge_a2stairs", "bridge A to stairs", TRAJ_DIR / "bridge_a2stairs1_smooth.txt"),
    ObstacleSpec("stairs", "stairs", TRAJ_DIR / "stairs_final.txt"),
)


@dataclass(frozen=True)
class WorkflowConfig:
    duration: float
    interval: float
    target_frame: str
    child_frame: str
    tf_topic: str
    spacing: float
    window: int
    passes: int
    dt: float
    yaw_mode: str
    pos_eps: float
    step_eps: float
    min_trim_points: int


@dataclass(frozen=True)
class ProcessStats:
    raw_points: int
    trimmed_points: int
    smooth_points: int
    raw_length: float
    trimmed_length: float
    smooth_length: float
    leading_removed: int
    trailing_removed: int
    max_smooth_offset: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record, trim, smooth, and publish obstacle trajectories")
    parser.add_argument("--duration", type=float, default=30.0, help="default recording duration in seconds")
    parser.add_argument("--interval", type=float, default=0.1, help="recording sample interval in seconds")
    parser.add_argument("--target-frame", default="camera_init", help="TF parent frame")
    parser.add_argument("--child-frame", default="aft_mapped", help="TF child frame")
    parser.add_argument("--tf-topic", default="/tf", help="TF topic")
    parser.add_argument("--spacing", type=float, default=0.05, help="smooth output resampling spacing in meters")
    parser.add_argument("--window", type=int, default=5, help="odd moving average window")
    parser.add_argument("--passes", type=int, default=1, help="smoothing passes")
    parser.add_argument("--dt", type=float, default=0.1, help="output time step in seconds")
    parser.add_argument(
        "--yaw-mode",
        choices=("tangent", "smooth", "keep"),
        default="smooth",
        help="smoothed yaw mode",
    )
    parser.add_argument("--pos-eps", type=float, default=0.08, help="dwell trim anchor distance threshold")
    parser.add_argument("--step-eps", type=float, default=0.04, help="dwell trim step distance threshold")
    parser.add_argument("--min-trim-points", type=int, default=3, help="ignore smaller dwell trims")
    parser.add_argument(
        "--process-existing",
        type=Path,
        default=None,
        help="offline mode: process an existing raw txt instead of recording from TF",
    )
    parser.add_argument(
        "--obstacle",
        choices=[spec.name for spec in OBSTACLES],
        default=None,
        help="obstacle to use with --process-existing, or default selection in interactive mode",
    )
    parser.add_argument(
        "--success",
        action="store_true",
        help="with --process-existing, publish the smoothed file as a successful attempt",
    )
    return parser.parse_args()


def validate_config(config: WorkflowConfig) -> None:
    if config.duration <= 0.0:
        raise ValueError("--duration must be positive")
    if config.interval <= 0.0:
        raise ValueError("--interval must be positive")
    if config.spacing <= 0.0:
        raise ValueError("--spacing must be positive")
    if config.dt <= 0.0:
        raise ValueError("--dt must be positive")
    if config.window < 1 or config.window % 2 == 0:
        raise ValueError("--window must be a positive odd integer")
    if config.passes < 0:
        raise ValueError("--passes must be non-negative")
    if config.pos_eps <= 0.0 or config.step_eps <= 0.0:
        raise ValueError("--pos-eps and --step-eps must be positive")
    if config.min_trim_points < 0:
        raise ValueError("--min-trim-points must be non-negative")


def config_from_args(args: argparse.Namespace) -> WorkflowConfig:
    config = WorkflowConfig(
        duration=args.duration,
        interval=args.interval,
        target_frame=args.target_frame,
        child_frame=args.child_frame,
        tf_topic=args.tf_topic,
        spacing=args.spacing,
        window=args.window,
        passes=args.passes,
        dt=args.dt,
        yaw_mode=args.yaw_mode,
        pos_eps=args.pos_eps,
        step_eps=args.step_eps,
        min_trim_points=args.min_trim_points,
    )
    validate_config(config)
    return config


def obstacle_by_name(name: str) -> ObstacleSpec:
    for spec in OBSTACLES:
        if spec.name == name:
            return spec
    raise ValueError(f"unknown obstacle: {name}")


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def unique_attempt_dir(obstacle: str) -> Path:
    base = RECORDS_DIR / obstacle / timestamp()
    attempt_dir = base
    suffix = 1
    while attempt_dir.exists():
        suffix += 1
        attempt_dir = base.with_name(f"{base.name}_{suffix:02d}")
    attempt_dir.mkdir(parents=True, exist_ok=False)
    return attempt_dir


def prompt_float(label: str, default: float) -> float:
    while True:
        raw = input(f"{label} [{default:g}]: ").strip()
        if not raw:
            return default
        try:
            value = float(raw)
        except ValueError:
            print("Please enter a number.")
            continue
        if value <= 0.0:
            print("Please enter a positive value.")
            continue
        return value


def prompt_yes_no(label: str, default: bool = False) -> bool:
    default_hint = "Y/n" if default else "y/N"
    while True:
        raw = input(f"{label} [{default_hint}]: ").strip().lower()
        if not raw:
            return default
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False
        print("Please answer y or n.")


def choose_obstacle(default_name: str | None = None) -> ObstacleSpec | None:
    print("")
    print("Select obstacle:")
    for index, spec in enumerate(OBSTACLES, start=1):
        default_marker = " *" if spec.name == default_name else ""
        print(f"  {index}. {spec.name:<16} {spec.title}{default_marker}")
    print("  q. quit")

    while True:
        raw = input("Obstacle: ").strip().lower()
        if not raw and default_name:
            return obstacle_by_name(default_name)
        if raw in ("q", "quit", "exit"):
            return None
        if raw.isdigit():
            index = int(raw)
            if 1 <= index <= len(OBSTACLES):
                return OBSTACLES[index - 1]
        for spec in OBSTACLES:
            if raw == spec.name:
                return spec
        print("Please enter a listed number, obstacle name, or q.")


def write_trimmed_trajectory(
    path: Path,
    source: Path,
    header_lines: list[str],
    points: list[smooth.TrajectoryPoint],
    config: WorkflowConfig,
    leading_removed: int,
    trailing_removed: int,
) -> None:
    lines = [
        "# Trimmed trajectory: time_sec x y yaw_rad (lidar/map frame)",
        f"# Source: {source}",
        (
            "# trajectory_workflow.py "
            f"pos_eps={config.pos_eps:.3f} step_eps={config.step_eps:.3f} "
            f"min_trim_points={config.min_trim_points} dt={config.dt:.3f} "
            f"leading_removed={leading_removed} trailing_removed={trailing_removed}"
        ),
    ]
    for line in header_lines:
        if line and line not in lines:
            lines.append(line)
    for point in points:
        lines.append(f"{point.t:.3f} {point.x:.6f} {point.y:.6f} {point.yaw:.6f}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def process_trajectory(
    raw_path: Path,
    trimmed_path: Path,
    smooth_path: Path,
    config: WorkflowConfig,
) -> ProcessStats:
    header_lines, raw_points = smooth.parse_trajectory(raw_path)
    raw_length = smooth.path_length(raw_points)

    trimmed_source, leading_removed, trailing_removed = smooth.trim_dwell_segments(
        raw_points,
        config.pos_eps,
        config.step_eps,
        config.min_trim_points,
    )
    trimmed_points = smooth.retime(trimmed_source, config.dt)
    trimmed_length = smooth.path_length(trimmed_points)
    write_trimmed_trajectory(
        trimmed_path,
        raw_path,
        header_lines,
        trimmed_points,
        config,
        leading_removed,
        trailing_removed,
    )

    resampled = smooth.resample_by_spacing(
        [smooth.TrajectoryPoint(p.t, p.x, p.y, p.yaw) for p in trimmed_source],
        config.spacing,
    )
    smoothed_xy = smooth.smooth_xy(resampled, config.window, config.passes)
    with_yaw = smooth.apply_yaw_mode(
        smoothed_xy,
        resampled,
        config.yaw_mode,
        config.window,
        config.passes,
    )
    with_yaw = smooth.blend_endpoint_yaws(
        with_yaw,
        trimmed_source[0].yaw,
        trimmed_source[-1].yaw,
        smooth.ENDPOINT_YAW_BLEND_DISTANCE,
    )
    smooth_points = smooth.retime(with_yaw, config.dt)
    smooth.write_trajectory(
        smooth_path,
        header_lines,
        raw_path,
        smooth_points,
        config.spacing,
        config.window,
        config.passes,
        config.dt,
        config.yaw_mode,
        True,
    )

    return ProcessStats(
        raw_points=len(raw_points),
        trimmed_points=len(trimmed_points),
        smooth_points=len(smooth_points),
        raw_length=raw_length,
        trimmed_length=trimmed_length,
        smooth_length=smooth.path_length(smooth_points),
        leading_removed=leading_removed,
        trailing_removed=trailing_removed,
        max_smooth_offset=smooth.max_xy_offset(resampled, smoothed_xy),
    )


def publish_successful_attempt(spec: ObstacleSpec, smooth_path: Path, attempt_stamp: str) -> Path | None:
    official_path = spec.official_path
    official_path.parent.mkdir(parents=True, exist_ok=True)
    backup_path: Path | None = None
    if official_path.exists():
        backup_path = official_path.with_name(f"{official_path.stem}.bak_{attempt_stamp}{official_path.suffix}")
        shutil.copy2(official_path, backup_path)
    shutil.copy2(smooth_path, official_path)
    return backup_path


def write_record_info(
    path: Path,
    spec: ObstacleSpec,
    config: WorkflowConfig,
    success: bool,
    raw_path: Path,
    trimmed_path: Path,
    smooth_path: Path,
    stats: ProcessStats,
    backup_path: Path | None,
    published_path: Path | None,
) -> None:
    lines = [
        "trajectory_workflow record",
        f"created_at: {datetime.now().isoformat(timespec='seconds')}",
        f"obstacle: {spec.name}",
        f"success: {str(success).lower()}",
        f"raw_file: {raw_path}",
        f"trimmed_file: {trimmed_path}",
        f"smooth_file: {smooth_path}",
        f"published_file: {published_path if published_path else ''}",
        f"backup_file: {backup_path if backup_path else ''}",
        "",
        "recording:",
        f"  duration: {config.duration:.3f}",
        f"  interval: {config.interval:.3f}",
        f"  target_frame: {config.target_frame}",
        f"  child_frame: {config.child_frame}",
        f"  tf_topic: {config.tf_topic}",
        "",
        "processing:",
        "  trim_dwell: true",
        f"  pos_eps: {config.pos_eps:.3f}",
        f"  step_eps: {config.step_eps:.3f}",
        f"  min_trim_points: {config.min_trim_points}",
        f"  spacing: {config.spacing:.3f}",
        f"  window: {config.window}",
        f"  passes: {config.passes}",
        f"  dt: {config.dt:.3f}",
        f"  yaw_mode: {config.yaw_mode}",
        "",
        "stats:",
        f"  raw_points: {stats.raw_points}",
        f"  trimmed_points: {stats.trimmed_points}",
        f"  smooth_points: {stats.smooth_points}",
        f"  raw_length_m: {stats.raw_length:.3f}",
        f"  trimmed_length_m: {stats.trimmed_length:.3f}",
        f"  smooth_length_m: {stats.smooth_length:.3f}",
        f"  leading_removed: {stats.leading_removed}",
        f"  trailing_removed: {stats.trailing_removed}",
        f"  max_smooth_offset_m: {stats.max_smooth_offset:.3f}",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def record_once(raw_path: Path, config: WorkflowConfig) -> None:
    import rclpy
    from record_trajectory import TrajectoryRecorder

    node = TrajectoryRecorder(
        output_path=raw_path.resolve(),
        duration_sec=config.duration,
        interval_sec=config.interval,
        target_frame=config.target_frame,
        child_frame=config.child_frame,
        tf_topic=config.tf_topic,
    )
    try:
        while rclpy.ok() and not node.recording_done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted; finishing if samples exist...")
        if not node.recording_done and len(node.samples) >= 2:
            node._finish()
    finally:
        node.destroy_node()

    if not raw_path.is_file():
        raise RuntimeError(f"recording did not write a raw trajectory: {raw_path}")


def run_attempt(
    spec: ObstacleSpec,
    config: WorkflowConfig,
    success: bool | None = None,
    existing_raw: Path | None = None,
) -> Path:
    attempt_dir = unique_attempt_dir(spec.name)
    attempt_stamp = attempt_dir.name
    raw_path = attempt_dir / f"{spec.name}_raw.txt"
    trimmed_path = attempt_dir / f"{spec.name}_trimmed.txt"
    smooth_path = attempt_dir / f"{spec.name}_smooth.txt"
    info_path = attempt_dir / "record_info.txt"

    if existing_raw is None:
        print(f"\nRecording {spec.name} -> {raw_path}")
        record_once(raw_path, config)
    else:
        source = existing_raw.resolve()
        if not source.is_file():
            raise FileNotFoundError(f"raw input does not exist: {source}")
        shutil.copy2(source, raw_path)
        print(f"\nCopied existing raw trajectory: {source}")

    stats = process_trajectory(raw_path, trimmed_path, smooth_path, config)
    print(
        "Processed: "
        f"raw {stats.raw_points} pts/{stats.raw_length:.2f} m, "
        f"trimmed {stats.trimmed_points} pts/{stats.trimmed_length:.2f} m, "
        f"smooth {stats.smooth_points} pts/{stats.smooth_length:.2f} m"
    )
    print(
        f"Trim removed head={stats.leading_removed}, tail={stats.trailing_removed}; "
        f"max smooth offset={stats.max_smooth_offset:.3f} m"
    )

    if success is None:
        success = prompt_yes_no("Mark this attempt successful and publish it?", default=False)

    backup_path = None
    published_path = None
    if success:
        backup_path = publish_successful_attempt(spec, smooth_path, attempt_stamp)
        published_path = spec.official_path
        print(f"Published smooth trajectory: {published_path}")
        if backup_path:
            print(f"Backed up previous official file: {backup_path}")
    else:
        print("Attempt saved only; official trajectory was not changed.")

    write_record_info(
        info_path,
        spec,
        config,
        success,
        raw_path,
        trimmed_path,
        smooth_path,
        stats,
        backup_path,
        published_path,
    )
    print(f"Attempt directory: {attempt_dir}")
    return attempt_dir


def interactive_loop(config: WorkflowConfig, default_obstacle: str | None) -> int:
    import rclpy

    rclpy.init()
    try:
        while True:
            spec = choose_obstacle(default_obstacle)
            if spec is None:
                return 0

            duration = prompt_float("Duration seconds", config.duration)
            interval = prompt_float("Sample interval seconds", config.interval)
            attempt_config = WorkflowConfig(
                duration=duration,
                interval=interval,
                target_frame=config.target_frame,
                child_frame=config.child_frame,
                tf_topic=config.tf_topic,
                spacing=config.spacing,
                window=config.window,
                passes=config.passes,
                dt=config.dt,
                yaw_mode=config.yaw_mode,
                pos_eps=config.pos_eps,
                step_eps=config.step_eps,
                min_trim_points=config.min_trim_points,
            )
            validate_config(attempt_config)
            try:
                run_attempt(spec, attempt_config)
            except Exception as exc:
                print(f"Attempt failed: {exc}", file=sys.stderr)

            if not prompt_yes_no("Record another trajectory?", default=True):
                return 0
    finally:
        rclpy.shutdown()


def main() -> int:
    args = parse_args()
    try:
        config = config_from_args(args)
        if args.process_existing is not None:
            if args.obstacle is None:
                raise ValueError("--obstacle is required with --process-existing")
            run_attempt(
                obstacle_by_name(args.obstacle),
                config,
                success=args.success,
                existing_raw=args.process_existing,
            )
            return 0
        return interactive_loop(config, args.obstacle)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
