#!/usr/bin/env python3
"""Apply generated obstacle points and paths to the official obstacle config.

Default mode is a dry run. Use --apply to copy official trajectory files and
update obstacle_race_params.yaml.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE = SCRIPT_DIR.parents[1]
TRAJ_DIR = WORKSPACE / "src/commands/task_manager/traj"
DEFAULT_CONFIG = WORKSPACE / "src/commands/task_manager/config/obstacle_race_params.yaml"

PATH_OBSTACLES = ("pole", "sand", "slope", "bridge_a", "bridge_b", "stairs")
KNOWN_OBSTACLES = (
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
)
OFFICIAL_PATHS = {
    "pole": TRAJ_DIR / "pole_final.txt",
    "sand": TRAJ_DIR / "sand_final.txt",
    "slope": TRAJ_DIR / "slope_final.txt",
    "bridge_a": TRAJ_DIR / "bridge_a_final.txt",
    "bridge_b": TRAJ_DIR / "bridge_b_final.txt",
    "stairs": TRAJ_DIR / "stairs_final.txt",
}


@dataclass(frozen=True)
class KeyPoint:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TrajectoryPoint:
    t: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CopyAction:
    obstacle: str
    source: Path
    official: Path
    backup: Path | None
    points: int
    length_m: float


@dataclass(frozen=True)
class NavUpdate:
    obstacle: str
    target: KeyPoint


@dataclass(frozen=True)
class PathUpdate:
    obstacle: str
    official: Path


@dataclass(frozen=True)
class ApplyPlan:
    points_path: Path
    output_dir: Path
    config_path: Path
    timestamp: str
    nav_updates: list[NavUpdate]
    path_updates: list[PathUpdate]
    copy_actions: list[CopyAction]
    config_backup: Path
    report_path: Path
    new_config_text: str


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_distance(a: float, b: float) -> float:
    return abs(normalize_angle(a - b))


def parse_key_point(raw: Any, label: str) -> KeyPoint:
    if not isinstance(raw, dict):
        raise ValueError(f"{label} must be an object")
    try:
        return KeyPoint(float(raw["x"]), float(raw["y"]), float(raw["yaw"]))
    except KeyError as exc:
        raise ValueError(f"{label} missing field: {exc.args[0]}") from exc
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} x/y/yaw must be numeric") from exc


def load_points(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        data = json.load(handle)
    if data.get("schema") != "obstacle_points.v1":
        raise ValueError(f"{path}: unsupported schema {data.get('schema')!r}")
    if data.get("yaw_convention") != "lidar_yaw_rad":
        raise ValueError(f"{path}: yaw_convention must be lidar_yaw_rad")
    if not isinstance(data.get("obstacles"), dict):
        raise ValueError(f"{path}: missing obstacles object")
    return data


def obstacle_start(data: dict[str, Any], obstacle: str) -> KeyPoint:
    obstacles = data["obstacles"]
    if obstacle not in obstacles:
        raise ValueError(f"missing obstacle in points JSON: {obstacle}")
    points = obstacles[obstacle].get("points")
    if not isinstance(points, dict):
        raise ValueError(f"{obstacle}.points must be an object")
    if "start" not in points:
        raise ValueError(f"{obstacle}.points.start is required for nav_target")
    return parse_key_point(points["start"], f"{obstacle}.start")


def obstacle_end(data: dict[str, Any], obstacle: str) -> KeyPoint:
    points = data["obstacles"][obstacle].get("points")
    if not isinstance(points, dict) or "end" not in points:
        raise ValueError(f"{obstacle}.points.end is required for path endpoint check")
    return parse_key_point(points["end"], f"{obstacle}.end")


def parse_trajectory(path: Path) -> list[TrajectoryPoint]:
    rows: list[TrajectoryPoint] = []
    with path.open(encoding="utf-8") as handle:
        for line_no, raw in enumerate(handle, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                raise ValueError(f"{path}:{line_no}: expected four columns: time x y yaw")
            try:
                t, x, y, yaw = map(float, parts[:4])
            except ValueError as exc:
                raise ValueError(f"{path}:{line_no}: non-numeric trajectory row") from exc
            rows.append(TrajectoryPoint(t, x, y, yaw))
    if len(rows) < 2:
        raise ValueError(f"{path}: trajectory needs at least 2 valid points")
    for index in range(1, len(rows)):
        if rows[index].t <= rows[index - 1].t:
            raise ValueError(f"{path}: time must be strictly increasing at index {index}")
    if path_length(rows) <= 1e-6:
        raise ValueError(f"{path}: trajectory length is too small")
    return rows


def path_length(points: list[TrajectoryPoint]) -> float:
    return sum(
        math.hypot(points[index].x - points[index - 1].x, points[index].y - points[index - 1].y)
        for index in range(1, len(points))
    )


def check_endpoints(
    obstacle: str,
    rows: list[TrajectoryPoint],
    start: KeyPoint,
    end: KeyPoint,
    pos_tolerance: float,
    yaw_tolerance: float,
) -> None:
    first = rows[0]
    last = rows[-1]
    start_error = math.hypot(first.x - start.x, first.y - start.y)
    end_error = math.hypot(last.x - end.x, last.y - end.y)
    start_yaw_error = angle_distance(first.yaw, start.yaw)
    end_yaw_error = angle_distance(last.yaw, end.yaw)
    if start_error > pos_tolerance:
        raise ValueError(f"{obstacle}: smooth start xy differs from JSON start by {start_error:.3f} m")
    if end_error > pos_tolerance:
        raise ValueError(f"{obstacle}: smooth end xy differs from JSON end by {end_error:.3f} m")
    if start_yaw_error > yaw_tolerance:
        raise ValueError(f"{obstacle}: smooth start yaw differs from JSON start by {start_yaw_error:.3f} rad")
    if end_yaw_error > yaw_tolerance:
        raise ValueError(f"{obstacle}: smooth end yaw differs from JSON end by {end_yaw_error:.3f} rad")


def generated_smooth_path(points_path: Path, obstacle: str) -> Path:
    return points_path.parent / f"{obstacle}_generated_smooth.txt"


def parse_obstacle_list(raw: str | None, data: dict[str, Any]) -> list[str]:
    if raw is None:
        sequence = data.get("sequence")
        if isinstance(sequence, list):
            return [name for name in sequence if isinstance(name, str) and name in KNOWN_OBSTACLES]
        return list(KNOWN_OBSTACLES)
    obstacles = [part.strip() for part in raw.split(",") if part.strip()]
    if not obstacles:
        raise ValueError("--obstacles cannot be empty")
    unknown = [name for name in obstacles if name not in KNOWN_OBSTACLES]
    if unknown:
        raise ValueError(f"unknown obstacle(s): {', '.join(unknown)}")
    return obstacles


def indent_of(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def is_real_line(line: str) -> bool:
    stripped = line.strip()
    return bool(stripped) and not stripped.startswith("#")


def find_key_block(lines: list[str], key: str, start: int = 0, end: int | None = None) -> tuple[int, int, int]:
    end = len(lines) if end is None else end
    pattern = re.compile(rf"^(\s*){re.escape(key)}:\s*(?:#.*)?$")
    for index in range(start, end):
        match = pattern.match(lines[index])
        if not match:
            continue
        base_indent = len(match.group(1))
        block_end = index + 1
        while block_end < end:
            if is_real_line(lines[block_end]) and indent_of(lines[block_end]) <= base_indent:
                break
            block_end += 1
        return index, block_end, base_indent
    raise ValueError(f"cannot find YAML block: {key}")


def replace_field_in_block(
    lines: list[str],
    block_start: int,
    block_end: int,
    field: str,
    value: str,
) -> None:
    pattern = re.compile(rf"^(\s*){re.escape(field)}:\s*.*$")
    for index in range(block_start + 1, block_end):
        match = pattern.match(lines[index])
        if not match:
            continue
        lines[index] = f"{match.group(1)}{field}: {value}\n"
        return
    raise ValueError(f"cannot find field '{field}' in block starting at line {block_start + 1}")


def set_nav_target(lines: list[str], obstacle: str, target: KeyPoint) -> None:
    seq_start, seq_end, _ = find_key_block(lines, "obstacle_sequence")
    slot_start, slot_end, _ = find_key_block(lines, obstacle, seq_start + 1, seq_end)
    value = f"[{target.x:.4f}, {target.y:.4f}, {target.yaw:.4f}]"
    replace_field_in_block(lines, slot_start, slot_end, "nav_target", value)


def set_trajectory_file(lines: list[str], obstacle: str, path: Path) -> None:
    profiles_start, profiles_end, _ = find_key_block(lines, "path_profiles")
    slot_start, slot_end, _ = find_key_block(lines, obstacle, profiles_start + 1, profiles_end)
    replace_field_in_block(lines, slot_start, slot_end, "trajectory_file", f'"{path.resolve()}"')


def build_new_config_text(config_text: str, nav_updates: list[NavUpdate], path_updates: list[PathUpdate]) -> str:
    lines = config_text.splitlines(keepends=True)
    for update in nav_updates:
        set_nav_target(lines, update.obstacle, update.target)
    for update in path_updates:
        set_trajectory_file(lines, update.obstacle, update.official)
    return "".join(lines)


def make_plan(args: argparse.Namespace) -> ApplyPlan:
    points_path = args.points.resolve()
    config_path = args.config.resolve()
    if not points_path.is_file():
        raise FileNotFoundError(f"points JSON does not exist: {points_path}")
    if not config_path.is_file():
        raise FileNotFoundError(f"obstacle config does not exist: {config_path}")

    data = load_points(points_path)
    selected = parse_obstacle_list(args.obstacles, data)
    selected_paths = [name for name in selected if name in PATH_OBSTACLES]

    nav_updates = [NavUpdate(name, obstacle_start(data, name)) for name in selected]
    copy_actions: list[CopyAction] = []
    path_updates: list[PathUpdate] = []

    stamp = timestamp()
    for obstacle in selected_paths:
        source = generated_smooth_path(points_path, obstacle)
        if not source.is_file():
            raise FileNotFoundError(
                f"missing generated smooth path for {obstacle}: {source}. "
                "Run generate_obstacle_paths.py first."
            )
        rows = parse_trajectory(source)
        start = obstacle_start(data, obstacle)
        end = obstacle_end(data, obstacle)
        check_endpoints(
            obstacle,
            rows,
            start,
            end,
            args.endpoint_pos_tolerance,
            args.endpoint_yaw_tolerance,
        )
        official = OFFICIAL_PATHS[obstacle].resolve()
        backup = (
            official.with_name(f"{official.stem}.bak_{stamp}{official.suffix}")
            if official.exists()
            else None
        )
        copy_actions.append(
            CopyAction(
                obstacle=obstacle,
                source=source.resolve(),
                official=official,
                backup=backup,
                points=len(rows),
                length_m=path_length(rows),
            )
        )
        path_updates.append(PathUpdate(obstacle, official))

    config_text = config_path.read_text(encoding="utf-8")
    new_config_text = build_new_config_text(config_text, nav_updates, path_updates)
    config_backup = config_path.with_name(f"{config_path.stem}.bak_{stamp}{config_path.suffix}")
    report_path = points_path.parent / f"apply_report_{stamp}.txt"

    return ApplyPlan(
        points_path=points_path,
        output_dir=points_path.parent,
        config_path=config_path,
        timestamp=stamp,
        nav_updates=nav_updates,
        path_updates=path_updates,
        copy_actions=copy_actions,
        config_backup=config_backup,
        report_path=report_path,
        new_config_text=new_config_text,
    )


def format_report(plan: ApplyPlan, applied: bool) -> str:
    lines = [
        "obstacle generated config apply report",
        f"created_at: {datetime.now().isoformat(timespec='seconds')}",
        f"mode: {'apply' if applied else 'dry-run'}",
        f"points_file: {plan.points_path}",
        f"config_file: {plan.config_path}",
        f"config_backup: {plan.config_backup}",
        "",
        "nav_target_updates:",
    ]
    for update in plan.nav_updates:
        lines.append(
            f"  {update.obstacle}: "
            f"[{update.target.x:.4f}, {update.target.y:.4f}, {update.target.yaw:.4f}]"
        )

    lines.append("")
    lines.append("trajectory_publications:")
    if not plan.copy_actions:
        lines.append("  none")
    for action in plan.copy_actions:
        lines.extend(
            [
                f"  {action.obstacle}:",
                f"    source: {action.source}",
                f"    official: {action.official}",
                f"    backup: {action.backup if action.backup else ''}",
                f"    points: {action.points}",
                f"    length_m: {action.length_m:.3f}",
            ]
        )

    lines.append("")
    lines.append("trajectory_file_updates:")
    if not plan.path_updates:
        lines.append("  none")
    for update in plan.path_updates:
        lines.append(f"  {update.obstacle}: {update.official}")

    if not applied:
        lines.append("")
        lines.append("dry_run_note: no files were modified; rerun with --apply to write changes.")
    return "\n".join(lines) + "\n"


def print_plan(plan: ApplyPlan, applied: bool) -> None:
    print(format_report(plan, applied).rstrip())


def apply_plan(plan: ApplyPlan) -> None:
    if plan.config_path.exists():
        shutil.copy2(plan.config_path, plan.config_backup)

    for action in plan.copy_actions:
        action.official.parent.mkdir(parents=True, exist_ok=True)
        if action.backup is not None:
            shutil.copy2(action.official, action.backup)
        shutil.copy2(action.source, action.official)

    plan.config_path.write_text(plan.new_config_text, encoding="utf-8")
    plan.report_path.write_text(format_report(plan, applied=True), encoding="utf-8")


def verify_applied(plan: ApplyPlan) -> None:
    config_text = plan.config_path.read_text(encoding="utf-8")
    for action in plan.copy_actions:
        rows = parse_trajectory(action.official)
        if not rows:
            raise RuntimeError(f"published trajectory is empty: {action.official}")
    for update in plan.path_updates:
        expected = f'trajectory_file: "{update.official.resolve()}"'
        if expected not in config_text:
            raise RuntimeError(f"config did not update trajectory_file for {update.obstacle}")
    for update in plan.nav_updates:
        expected = f"nav_target: [{update.target.x:.4f}, {update.target.y:.4f}, {update.target.yaw:.4f}]"
        if expected not in config_text:
            raise RuntimeError(f"config did not update nav_target for {update.obstacle}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--points", type=Path, required=True, help="generated obstacle_points.json")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="obstacle_race_params.yaml")
    parser.add_argument(
        "--obstacles",
        default=None,
        help="comma-separated slots to apply; default uses sequence from obstacle_points.json",
    )
    parser.add_argument("--apply", action="store_true", help="write files; default is dry-run")
    parser.add_argument(
        "--endpoint-pos-tolerance",
        type=float,
        default=0.08,
        help="max allowed generated endpoint xy error in meters",
    )
    parser.add_argument(
        "--endpoint-yaw-tolerance",
        type=float,
        default=0.15,
        help="max allowed generated endpoint yaw error in radians",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.endpoint_pos_tolerance < 0:
        raise ValueError("--endpoint-pos-tolerance must be >= 0")
    if args.endpoint_yaw_tolerance < 0:
        raise ValueError("--endpoint-yaw-tolerance must be >= 0")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        plan = make_plan(args)
        if not args.apply:
            print_plan(plan, applied=False)
            return 0

        apply_plan(plan)
        verify_applied(plan)
        print_plan(plan, applied=True)
        print(f"\nWrote report: {plan.report_path}")
        return 0
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
