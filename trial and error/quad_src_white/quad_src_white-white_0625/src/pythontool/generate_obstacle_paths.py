#!/usr/bin/env python3
"""Generate smooth obstacle trajectories from recorded key-point JSON."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import smooth_trajectory as smooth


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE = SCRIPT_DIR.parents[1]
PLOT_SCRIPT = SCRIPT_DIR / "plot_trajectory.py"
PATH_OBSTACLES = ("pole", "sand", "slope", "bridge_a", "bridge_b", "stairs")
DEFAULT_OUTPUT_NAMES = {
    "pole": "pole_generated",
    "sand": "sand_generated",
    "slope": "slope_generated",
    "bridge_a": "bridge_a_generated",
    "bridge_b": "bridge_b_generated",
    "stairs": "stairs_generated",
}
REQUIRED_POINTS = {
    "spline": ("start", "end"),
    "l_shape": ("start", "corner", "end"),
    "line": ("start", "end"),
    "start_only": ("start",),
    "transit": ("start",),
}


@dataclass(frozen=True)
class KeyPoint:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PathBuildResult:
    obstacle: str
    raw_path: Path
    smooth_path: Path
    raw_points: int
    smooth_points: int
    length_m: float


def normalize_angle(angle: float) -> float:
    return smooth.normalize_angle(angle)


def lidar_yaw_from_tangent(dx: float, dy: float, fallback: float) -> float:
    if math.hypot(dx, dy) <= 1e-9:
        return fallback
    return normalize_angle(math.atan2(dy, dx) - math.pi * 0.5)


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


def obstacle_points(data: dict[str, Any], obstacle: str) -> tuple[str, dict[str, KeyPoint], list[str]]:
    obstacles = data["obstacles"]
    if obstacle not in obstacles:
        raise ValueError(f"missing obstacle: {obstacle}")
    raw_slot = obstacles[obstacle]
    if not isinstance(raw_slot, dict):
        raise ValueError(f"{obstacle} must be an object")
    template = str(raw_slot.get("template", ""))
    raw_points = raw_slot.get("points")
    if not isinstance(raw_points, dict):
        raise ValueError(f"{obstacle}.points must be an object")

    parsed = {
        name: parse_key_point(value, f"{obstacle}.{name}")
        for name, value in raw_points.items()
    }
    point_order = raw_slot.get("point_order")
    if point_order is None:
        point_order = list(parsed.keys())
    if not isinstance(point_order, list) or not all(isinstance(item, str) for item in point_order):
        raise ValueError(f"{obstacle}.point_order must be a string array")
    return template, parsed, point_order


def validate_required_points(obstacle: str, template: str, points: dict[str, KeyPoint]) -> None:
    if template not in REQUIRED_POINTS:
        raise ValueError(f"{obstacle}: unsupported template {template!r}")
    missing = [name for name in REQUIRED_POINTS[template] if name not in points]
    if missing:
        raise ValueError(f"{obstacle}: missing required point(s): {', '.join(missing)}")


def as_smooth_point(point: KeyPoint, t: float = 0.0) -> smooth.TrajectoryPoint:
    return smooth.TrajectoryPoint(t=t, x=point.x, y=point.y, yaw=point.yaw)


def assign_tangent_yaws(points: list[smooth.TrajectoryPoint], start_yaw: float, end_yaw: float) -> list[smooth.TrajectoryPoint]:
    if not points:
        return []
    assigned: list[smooth.TrajectoryPoint] = []
    last_yaw = start_yaw
    for index, point in enumerate(points):
        if index == 0:
            yaw = start_yaw
        elif index == len(points) - 1:
            yaw = end_yaw
        else:
            prev_point = points[index - 1]
            next_point = points[index + 1]
            yaw = lidar_yaw_from_tangent(next_point.x - prev_point.x, next_point.y - prev_point.y, last_yaw)
            last_yaw = yaw
        assigned.append(smooth.TrajectoryPoint(point.t, point.x, point.y, yaw, point.s))
    return assigned


def append_unique(points: list[smooth.TrajectoryPoint], point: smooth.TrajectoryPoint) -> None:
    if points and math.hypot(points[-1].x - point.x, points[-1].y - point.y) <= 1e-6:
        points[-1] = point
        return
    points.append(point)


def sample_line(a: KeyPoint, b: KeyPoint, raw_step: float) -> list[smooth.TrajectoryPoint]:
    distance = math.hypot(b.x - a.x, b.y - a.y)
    count = max(1, int(math.ceil(distance / raw_step)))
    out: list[smooth.TrajectoryPoint] = []
    for index in range(count + 1):
        alpha = index / count
        yaw = normalize_angle(a.yaw + alpha * normalize_angle(b.yaw - a.yaw))
        out.append(
            smooth.TrajectoryPoint(
                t=0.0,
                x=a.x + alpha * (b.x - a.x),
                y=a.y + alpha * (b.y - a.y),
                yaw=yaw,
            )
        )
    return out


def generate_line(points: dict[str, KeyPoint], raw_step: float) -> list[smooth.TrajectoryPoint]:
    return sample_line(points["start"], points["end"], raw_step)


def generate_l_shape(points: dict[str, KeyPoint], raw_step: float, corner_radius: float) -> list[smooth.TrajectoryPoint]:
    start = points["start"]
    corner = points["corner"]
    end = points["end"]

    v1x = start.x - corner.x
    v1y = start.y - corner.y
    v2x = end.x - corner.x
    v2y = end.y - corner.y
    len1 = math.hypot(v1x, v1y)
    len2 = math.hypot(v2x, v2y)
    if len1 <= 1e-6 or len2 <= 1e-6 or corner_radius <= 1e-6:
        polyline = [
            *sample_line(start, corner, raw_step)[:-1],
            *sample_line(corner, end, raw_step),
        ]
        return assign_tangent_yaws(polyline, start.yaw, end.yaw)

    radius = min(corner_radius, len1 * 0.45, len2 * 0.45)
    in_point = KeyPoint(corner.x + (v1x / len1) * radius, corner.y + (v1y / len1) * radius, corner.yaw)
    out_point = KeyPoint(corner.x + (v2x / len2) * radius, corner.y + (v2y / len2) * radius, corner.yaw)

    result: list[smooth.TrajectoryPoint] = []
    for point in sample_line(start, in_point, raw_step):
        append_unique(result, point)

    arc_len_est = math.hypot(out_point.x - in_point.x, out_point.y - in_point.y) + radius
    arc_count = max(4, int(math.ceil(arc_len_est / raw_step)))
    for index in range(1, arc_count):
        t = index / arc_count
        one_minus = 1.0 - t
        x = one_minus * one_minus * in_point.x + 2.0 * one_minus * t * corner.x + t * t * out_point.x
        y = one_minus * one_minus * in_point.y + 2.0 * one_minus * t * corner.y + t * t * out_point.y
        append_unique(result, smooth.TrajectoryPoint(0.0, x, y, corner.yaw))

    for point in sample_line(out_point, end, raw_step):
        append_unique(result, point)
    return assign_tangent_yaws(result, start.yaw, end.yaw)


def catmull_rom_point(
    p0: KeyPoint,
    p1: KeyPoint,
    p2: KeyPoint,
    p3: KeyPoint,
    t: float,
) -> tuple[float, float]:
    t2 = t * t
    t3 = t2 * t
    x = 0.5 * (
        2.0 * p1.x
        + (-p0.x + p2.x) * t
        + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2
        + (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3
    )
    y = 0.5 * (
        2.0 * p1.y
        + (-p0.y + p2.y) * t
        + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2
        + (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3
    )
    return x, y


def generate_spline(points: dict[str, KeyPoint], point_order: list[str], raw_step: float) -> list[smooth.TrajectoryPoint]:
    ordered_names = [name for name in point_order if name in points]
    if "start" not in ordered_names:
        ordered_names.insert(0, "start")
    if "end" not in ordered_names:
        ordered_names.append("end")
    ordered = [points[name] for name in ordered_names]
    if len(ordered) < 2:
        raise ValueError("spline needs at least start and end")

    result: list[smooth.TrajectoryPoint] = []
    for index in range(len(ordered) - 1):
        p0 = ordered[max(0, index - 1)]
        p1 = ordered[index]
        p2 = ordered[index + 1]
        p3 = ordered[min(len(ordered) - 1, index + 2)]
        segment_len = math.hypot(p2.x - p1.x, p2.y - p1.y)
        count = max(4, int(math.ceil(segment_len / raw_step)))
        for sample_index in range(count):
            t = sample_index / count
            x, y = catmull_rom_point(p0, p1, p2, p3, t)
            append_unique(result, smooth.TrajectoryPoint(0.0, x, y, p1.yaw))
    append_unique(result, as_smooth_point(ordered[-1]))
    return assign_tangent_yaws(result, ordered[0].yaw, ordered[-1].yaw)


def retime(points: list[smooth.TrajectoryPoint], dt: float) -> list[smooth.TrajectoryPoint]:
    return [smooth.TrajectoryPoint(index * dt, p.x, p.y, p.yaw, p.s) for index, p in enumerate(points)]


def generate_raw_points(
    obstacle: str,
    template: str,
    points: dict[str, KeyPoint],
    point_order: list[str],
    raw_step: float,
    corner_radius: float,
    dt: float,
) -> list[smooth.TrajectoryPoint]:
    if template == "line":
        raw = generate_line(points, raw_step)
    elif template == "l_shape":
        raw = generate_l_shape(points, raw_step, corner_radius)
    elif template == "spline":
        raw = generate_spline(points, point_order, raw_step)
    else:
        raise ValueError(f"{obstacle}: template {template!r} does not generate trajectories")
    if len(raw) < 2:
        raise ValueError(f"{obstacle}: generated fewer than 2 raw points")
    return retime(raw, dt)


def write_simple_trajectory(path: Path, points: list[smooth.TrajectoryPoint], title: str, source: Path) -> None:
    lines = [
        f"# {title}: time_sec x y yaw_rad (lidar/map frame)",
        f"# Source points: {source}",
        "# Generated by generate_obstacle_paths.py",
    ]
    for point in points:
        lines.append(f"{point.t:.3f} {point.x:.6f} {point.y:.6f} {point.yaw:.6f}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def smooth_generated_points(
    raw_points: list[smooth.TrajectoryPoint],
    spacing: float,
    window: int,
    passes: int,
    dt: float,
    yaw_mode: str,
) -> list[smooth.TrajectoryPoint]:
    start_yaw = raw_points[0].yaw
    end_yaw = raw_points[-1].yaw
    resampled = smooth.resample_by_spacing([
        smooth.TrajectoryPoint(p.t, p.x, p.y, p.yaw) for p in raw_points
    ], spacing)
    smoothed_xy = smooth.smooth_xy(resampled, window, passes)
    with_yaw = smooth.apply_yaw_mode(smoothed_xy, resampled, yaw_mode, window, passes)
    with_yaw = smooth.blend_endpoint_yaws(
        with_yaw,
        start_yaw,
        end_yaw,
        smooth.ENDPOINT_YAW_BLEND_DISTANCE,
    )
    return retime(with_yaw, dt)


def output_base_name(slot: dict[str, Any], obstacle: str) -> str:
    outputs = slot.get("outputs")
    if isinstance(outputs, dict):
        raw_name = outputs.get("smooth") or outputs.get("raw")
        if isinstance(raw_name, str) and raw_name:
            stem = Path(raw_name).stem
            if stem.endswith("_smooth"):
                return stem[: -len("_smooth")]
            if stem.endswith("_raw"):
                return stem[: -len("_raw")]
            return stem
    return DEFAULT_OUTPUT_NAMES[obstacle]


def build_obstacle_path(
    data: dict[str, Any],
    points_path: Path,
    output_dir: Path,
    obstacle: str,
    args: argparse.Namespace,
) -> PathBuildResult:
    template, points, point_order = obstacle_points(data, obstacle)
    validate_required_points(obstacle, template, points)
    if template not in ("line", "l_shape", "spline"):
        raise ValueError(f"{obstacle}: template {template!r} cannot produce a trajectory")

    raw_points = generate_raw_points(
        obstacle,
        template,
        points,
        point_order,
        args.raw_step,
        args.corner_radius,
        args.dt,
    )
    smooth_points = smooth_generated_points(
        raw_points,
        args.spacing,
        args.window,
        args.passes,
        args.dt,
        args.yaw_mode,
    )

    slot = data["obstacles"][obstacle]
    base = output_base_name(slot, obstacle)
    raw_path = output_dir / f"{base}_raw.txt"
    smooth_path = output_dir / f"{base}_smooth.txt"
    length = smooth.path_length(smooth_points)

    if not args.dry_run:
        write_simple_trajectory(raw_path, raw_points, "Raw generated trajectory", points_path)
        smooth.write_trajectory(
            smooth_path,
            [
                "# Generated smooth obstacle trajectory",
                "# body_heading_rad = lidar_yaw_rad + pi/2",
            ],
            raw_path,
            smooth_points,
            args.spacing,
            args.window,
            args.passes,
            args.dt,
            args.yaw_mode,
            False,
        )

    return PathBuildResult(
        obstacle=obstacle,
        raw_path=raw_path,
        smooth_path=smooth_path,
        raw_points=len(raw_points),
        smooth_points=len(smooth_points),
        length_m=length,
    )


def format_nav_summary(data: dict[str, Any]) -> str:
    lines = [
        "# Generated nav_target summary from obstacle_points.json",
        "# Copy values manually into obstacle_race_params.yaml after field verification.",
        "# yaw values are lidar_yaw_rad.",
        "",
        "obstacle_sequence:",
    ]
    for obstacle in data.get("sequence", []):
        try:
            _, points, _ = obstacle_points(data, obstacle)
        except ValueError:
            continue
        start = points.get("start")
        if start is None:
            continue
        lines.append(f"  {obstacle}:")
        lines.append(f"    nav_target: [{start.x:.4f}, {start.y:.4f}, {start.yaw:.4f}]")
    return "\n".join(lines) + "\n"


def write_summary_files(output_dir: Path, data: dict[str, Any], results: list[PathBuildResult]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "nav_targets_summary.yaml").write_text(format_nav_summary(data), encoding="utf-8")

    lines = [
        "# Generated obstacle trajectory summary",
        "# yaw values are lidar_yaw_rad; body_heading_rad = lidar_yaw_rad + pi/2",
        "",
    ]
    for result in results:
        lines.append(
            f"{result.obstacle}: raw={result.raw_path.name} smooth={result.smooth_path.name} "
            f"raw_points={result.raw_points} smooth_points={result.smooth_points} "
            f"length_m={result.length_m:.3f}"
        )
    (output_dir / "paths_summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_viz(results: list[PathBuildResult]) -> int:
    failures = 0
    for result in results:
        cmd = [sys.executable, str(PLOT_SCRIPT), str(result.smooth_path)]
        completed = subprocess.run(cmd, cwd=str(WORKSPACE), check=False)
        if completed.returncode != 0:
            print(f"Warning: visualization failed for {result.smooth_path}", file=sys.stderr)
            failures += 1
    return failures


def parse_obstacle_list(raw: str) -> list[str]:
    obstacles = [part.strip() for part in raw.split(",") if part.strip()]
    if not obstacles:
        raise ValueError("--obstacles cannot be empty")
    unknown = [name for name in obstacles if name not in PATH_OBSTACLES]
    if unknown:
        raise ValueError(f"unknown path obstacle(s): {', '.join(unknown)}")
    return obstacles


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--points", type=Path, required=True, help="obstacle_points.json from record_obstacle_points.py")
    parser.add_argument("--output-dir", type=Path, default=None, help="output directory, default next to --points")
    parser.add_argument("--obstacles", default=",".join(PATH_OBSTACLES), help="comma-separated path obstacles to generate")
    parser.add_argument("--spacing", type=float, default=0.05, help="smooth output spacing in meters")
    parser.add_argument("--window", type=int, default=5, help="odd moving average window")
    parser.add_argument("--passes", type=int, default=1, help="smoothing passes")
    parser.add_argument("--dt", type=float, default=0.1, help="output time step in seconds")
    parser.add_argument("--yaw-mode", choices=("tangent", "smooth", "keep"), default="tangent")
    parser.add_argument("--raw-step", type=float, default=0.05, help="raw geometry sampling step in meters")
    parser.add_argument("--corner-radius", type=float, default=0.25, help="L-shape corner rounding radius in meters")
    parser.add_argument("--viz", action="store_true", help="plot generated smooth trajectories")
    parser.add_argument("--dry-run", action="store_true", help="validate and print stats without writing files")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> list[str]:
    if args.spacing <= 0 or args.raw_step <= 0 or args.dt <= 0:
        raise ValueError("--spacing, --raw-step, and --dt must be positive")
    if args.window < 1 or args.window % 2 == 0:
        raise ValueError("--window must be a positive odd integer")
    if args.passes < 0:
        raise ValueError("--passes must be non-negative")
    if args.corner_radius < 0:
        raise ValueError("--corner-radius must be >= 0")
    return parse_obstacle_list(args.obstacles)


def main() -> int:
    args = parse_args()
    try:
        obstacles = validate_args(args)
        points_path = args.points.resolve()
        output_dir = args.output_dir.resolve() if args.output_dir else points_path.parent
        data = load_points(points_path)

        results = [
            build_obstacle_path(data, points_path, output_dir, obstacle, args)
            for obstacle in obstacles
        ]

        print(f"Input: {points_path}")
        print(f"Output dir: {output_dir}")
        for result in results:
            print(
                f"{result.obstacle}: raw_points={result.raw_points}, "
                f"smooth_points={result.smooth_points}, length={result.length_m:.3f} m"
            )
            print(f"  raw:    {result.raw_path}")
            print(f"  smooth: {result.smooth_path}")

        if args.dry_run:
            print("Dry run: no files written.")
            return 0

        write_summary_files(output_dir, data, results)
        print(f"Summary: {output_dir / 'nav_targets_summary.yaml'}")
        print(f"Summary: {output_dir / 'paths_summary.txt'}")

        if args.viz:
            failures = run_viz(results)
            if failures:
                print(f"Visualization completed with {failures} warning(s).", file=sys.stderr)
        return 0
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
