#!/usr/bin/env python3
"""Plot recorded trajectory txt files with lookahead reference samples."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass
class TrajectoryPoint:
    t: float
    x: float
    y: float
    yaw: float
    s: float = 0.0


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def parse_trajectory(path: Path) -> list[TrajectoryPoint]:
    points: list[TrajectoryPoint] = []
    with path.open(encoding="utf-8") as f:
        for line_no, raw in enumerate(f, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                raise ValueError(f"{path}:{line_no}: expected 4 columns: time x y yaw")
            t, x, y, yaw = map(float, parts[:4])
            points.append(TrajectoryPoint(t, x, y, yaw))

    if len(points) < 2:
        raise ValueError(f"{path}: need at least 2 trajectory points")

    accumulated = 0.0
    points[0].s = 0.0
    for i in range(1, len(points)):
        accumulated += math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y)
        points[i].s = accumulated
    if accumulated <= 1e-6:
        raise ValueError(f"{path}: path length is too small")
    return points


def sample_at_s(points: list[TrajectoryPoint], s: float) -> TrajectoryPoint:
    if s <= points[0].s:
        return points[0]
    if s >= points[-1].s:
        return points[-1]

    for a, b in zip(points, points[1:]):
        if a.s <= s <= b.s:
            ds = b.s - a.s
            alpha = (s - a.s) / ds if ds > 1e-9 else 0.0
            dyaw = normalize_angle(b.yaw - a.yaw)
            return TrajectoryPoint(
                t=a.t + alpha * (b.t - a.t),
                x=a.x + alpha * (b.x - a.x),
                y=a.y + alpha * (b.y - a.y),
                yaw=normalize_angle(a.yaw + alpha * dyaw),
                s=s,
            )
    return points[-1]


def body_heading_from_lidar_yaw(lidar_yaw: float, drive_mode: str) -> float:
    heading = normalize_angle(lidar_yaw + math.pi * 0.5)
    if drive_mode == "reverse":
        heading = normalize_angle(heading + math.pi)
    return heading


def output_paths(input_path: Path, output_prefix: Path | None) -> tuple[Path, Path]:
    if output_prefix is not None:
        return output_prefix.with_suffix(".png"), output_prefix.with_suffix(".svg")
    return (
        input_path.with_name(input_path.stem + "_viz.png"),
        input_path.with_name(input_path.stem + "_viz.svg"),
    )


def collect_inputs(inputs: list[Path], pattern: str, recursive: bool) -> tuple[list[tuple[Path, bool]], int]:
    candidates: list[tuple[Path, bool]] = []
    discovered_count = 0

    for input_path in inputs:
        resolved = input_path.resolve()
        if resolved.is_dir():
            iterator = resolved.rglob(pattern) if recursive else resolved.glob(pattern)
            found = sorted(path.resolve() for path in iterator if path.is_file())
            discovered_count += len(found)
            candidates.extend((path, True) for path in found)
        else:
            candidates.append((resolved, False))

    seen: set[Path] = set()
    unique: list[tuple[Path, bool]] = []
    for path, discovered in candidates:
        if path in seen:
            continue
        seen.add(path)
        unique.append((path, discovered))
    return unique, discovered_count


def plot_trajectories(
    trajectories: list[tuple[Path, list[TrajectoryPoint]]],
    png_path: Path,
    svg_path: Path,
    lookahead: float,
    drive_mode: str,
    arrow_step: float,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required: python3 -m pip install matplotlib") from exc

    fig, ax = plt.subplots(figsize=(9, 8))
    summary_lines: list[str] = []

    for idx, (path, points) in enumerate(trajectories):
        xs = [p.x for p in points]
        ys = [p.y for p in points]
        label = path.stem
        length = points[-1].s
        avg_step = length / max(len(points) - 1, 1)
        summary_lines.append(f"{label}: n={len(points)} len={length:.2f}m avg={avg_step:.2f}m")

        (line,) = ax.plot(xs, ys, linewidth=2.0, label=label)
        color = line.get_color()
        ax.scatter([points[0].x], [points[0].y], marker="o", s=55, color=color)
        ax.scatter([points[-1].x], [points[-1].y], marker="s", s=55, color=color)
        ax.text(points[0].x, points[0].y, f" {label} start", fontsize=8)
        ax.text(points[-1].x, points[-1].y, f" {label} end", fontsize=8)

        arrow_s = 0.0
        while arrow_s <= length + 1e-9:
            base = sample_at_s(points, arrow_s)
            ref = sample_at_s(points, min(length, arrow_s + lookahead))
            heading = body_heading_from_lidar_yaw(ref.yaw, drive_mode)
            ax.plot([base.x, ref.x], [base.y, ref.y], linestyle="--", linewidth=0.8, color=color, alpha=0.4)
            ax.quiver(
                ref.x,
                ref.y,
                math.cos(heading),
                math.sin(heading),
                angles="xy",
                scale_units="xy",
                scale=7.0,
                width=0.003,
                color=color,
                alpha=0.9,
            )
            arrow_s += max(arrow_step, 1e-3)

        if idx == 0:
            ax.scatter([], [], marker="o", s=55, color="black", label="start")
            ax.scatter([], [], marker="s", s=55, color="black", label="end")

    ax.set_title(
        f"Trajectory lookahead visualization ({drive_mode}, lookahead={lookahead:.2f} m, arrows=body heading)"
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, linestyle=":", linewidth=0.7)
    ax.legend(loc="best")
    ax.text(
        0.01,
        0.01,
        "\n".join(summary_lines),
        transform=ax.transAxes,
        fontsize=9,
        va="bottom",
        ha="left",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "alpha": 0.8, "edgecolor": "#cccccc"},
    )

    fig.tight_layout()
    png_path.parent.mkdir(parents=True, exist_ok=True)
    svg_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png_path, dpi=160)
    fig.savefig(svg_path)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot recorded trajectory txt files")
    parser.add_argument("trajectories", type=Path, nargs="+", help="trajectory txt file(s) or directories")
    parser.add_argument("--lookahead", type=float, default=0.35, help="lookahead distance in meters")
    parser.add_argument(
        "--drive-mode",
        choices=("forward", "reverse"),
        default="forward",
        help="body heading display mode; trajectory yaw is treated as lidar yaw",
    )
    parser.add_argument("--arrow-step", type=float, default=0.5, help="distance between displayed arrows")
    parser.add_argument("--pattern", default="*.txt", help="file pattern used when an input is a directory")
    parser.add_argument("--recursive", action="store_true", help="recursively scan directory inputs")
    parser.add_argument(
        "-o",
        "--output-prefix",
        type=Path,
        default=None,
        help="single-trajectory output prefix; writes <prefix>.png and <prefix>.svg",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.lookahead < 0:
        print("lookahead must be non-negative", file=sys.stderr)
        return 1

    has_directory_input = any(path.resolve().is_dir() for path in args.trajectories)
    candidates, discovered_count = collect_inputs(args.trajectories, args.pattern, args.recursive)
    if not candidates:
        print("no trajectory files found", file=sys.stderr)
        return 1
    if args.output_prefix is not None and (has_directory_input or len(candidates) != 1):
        print("--output-prefix can only be used with one explicit trajectory file", file=sys.stderr)
        return 1

    plotted = 0
    skipped = 0
    for path, discovered in candidates:
        try:
            points = parse_trajectory(path)
        except Exception as exc:
            if discovered:
                skipped += 1
                print(f"warning: skipping {path}: {exc}", file=sys.stderr)
                continue
            print(f"error: {exc}", file=sys.stderr)
            return 1

        png_path, svg_path = output_paths(path, args.output_prefix)
        plot_trajectories(
            [(path, points)],
            png_path.resolve(),
            svg_path.resolve(),
            args.lookahead,
            args.drive_mode,
            args.arrow_step,
        )
        plotted += 1
        print(f"wrote {png_path.resolve()}")
        print(f"wrote {svg_path.resolve()}")

    print(
        f"summary: inputs={len(args.trajectories)}, discovered={discovered_count}, "
        f"plotted={plotted}, skipped={skipped}"
    )
    if plotted == 0:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
