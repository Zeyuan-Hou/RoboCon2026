#!/usr/bin/env python3
"""Visualize generated field pickup/dropoff waypoints as SVG (no matplotlib)."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

from field_waypoint_math import load_anchors_from_file

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_GENERATED = (
    WORKSPACE / "src/commands/lidar_nav_demo_cpp/config/manip_points_generated.yaml"
)
DEFAULT_ANCHORS = (
    WORKSPACE
    / "src/commands/lidar_nav_demo_cpp/config/trajectories/example_point.txt"
)
DEFAULT_VIZ = (
    WORKSPACE / "src/commands/lidar_nav_demo_cpp/config/field_waypoints_viz.svg"
)

MARGIN_M = 0.5
LABEL_SIZE_M = 0.5
PX_PER_M = 120.0


def _parse_flat_array(text: str, key: str) -> list[float]:
    pattern = re.compile(rf"{key}:\s*\[([^\]]+)\]")
    match = pattern.search(text)
    if not match:
        raise KeyError(key)
    return [float(v.strip()) for v in match.group(1).split(",") if v.strip()]


def _parse_xy(text: str, key: str) -> list[float]:
    pattern = re.compile(rf"{key}:\s*\[([^\]]+)\]")
    match = pattern.search(text)
    if not match:
        return []
    return [float(v.strip()) for v in match.group(1).split(",")]


def _load_result_from_generated(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    pickup_flat = _parse_flat_array(text, "pickup_points")
    dropoff_flat = _parse_flat_array(text, "dropoff_points")
    pickup_points = []
    for i in range(0, len(pickup_flat), 4):
        pickup_points.append(tuple(pickup_flat[i : i + 4]))
    dropoff_points = []
    for i in range(0, len(dropoff_flat), 3):
        dropoff_points.append(tuple(dropoff_flat[i : i + 3]))

    def xy(key: str, fallback: list[float]) -> list[float]:
        vals = _parse_xy(text, key)
        return vals if len(vals) >= 2 else fallback

    return {
        "pickup_points": pickup_points,
        "dropoff_points": dropoff_points,
        "a1_map": xy("a1_map", [pickup_points[0][0], pickup_points[0][1]]),
        "b4_map": xy("b4_map", [pickup_points[7][0], pickup_points[7][1]]),
        "c1_map": xy("c1_map", [dropoff_points[0][0], dropoff_points[0][1]]),
        "c4_low_map": xy("c4_low_map", [dropoff_points[3][0], dropoff_points[3][1]]),
    }


def _world_to_svg(x: float, y: float, xmin: float, ymax: float) -> tuple[float, float]:
    sx = (x - xmin) * PX_PER_M
    sy = (ymax - y) * PX_PER_M
    return sx, sy


def _draw_star(lines: list[str], x: float, y: float, label: str, color: str,
               xmin: float, ymax: float, label_px: float) -> None:
    sx, sy = _world_to_svg(x, y, xmin, ymax)
    star_r = label_px * 0.45
    lines.append(
        f'<polygon points="{sx:.1f},{sy - star_r:.1f} {sx + star_r:.1f},{sy:.1f} '
        f'{sx:.1f},{sy + star_r:.1f} {sx - star_r:.1f},{sy:.1f}" fill="{color}"/>'
    )
    lines.append(
        f'<text x="{sx:.1f}" y="{sy - star_r - 6:.1f}" text-anchor="middle" '
        f'font-size="12" fill="{color}">{label}</text>'
    )


def _draw_row_guide(
    lines: list[str],
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    color: str,
    xmin: float,
    ymax: float,
    extend_m: float = 0.4,
) -> None:
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return
    ux, uy = dx / length, dy / length
    ex0 = x0 - ux * extend_m
    ey0 = y0 - uy * extend_m
    ex1 = x1 + ux * extend_m
    ey1 = y1 + uy * extend_m
    sx0, sy0 = _world_to_svg(ex0, ey0, xmin, ymax)
    sx1, sy1 = _world_to_svg(ex1, ey1, xmin, ymax)
    lines.append(
        f'<line x1="{sx0:.1f}" y1="{sy0:.1f}" x2="{sx1:.1f}" y2="{sy1:.1f}" '
        f'stroke="{color}" stroke-width="1.5" stroke-dasharray="6,4" opacity="0.55"/>'
    )


def render_field_waypoints(result: dict, output_path: Path) -> None:
    pickup = result["pickup_points"]
    dropoff = result["dropoff_points"]
    a1 = result["a1_map"]
    b4 = result.get("b4_map", [pickup[7][0], pickup[7][1]])
    c1 = result.get("c1_map", [dropoff[0][0], dropoff[0][1]])
    c4 = result["c4_low_map"]

    anchor_xs = [a1[0], b4[0], c1[0], c4[0]]
    anchor_ys = [a1[1], b4[1], c1[1], c4[1]]
    xs = [p[0] for p in pickup] + [d[0] for d in dropoff] + anchor_xs
    ys = [p[1] for p in pickup] + [d[1] for d in dropoff] + anchor_ys

    xmin, xmax = min(xs) - MARGIN_M, max(xs) + MARGIN_M
    ymin, ymax = min(ys) - MARGIN_M, max(ys) + MARGIN_M

    width = int(math.ceil((xmax - xmin) * PX_PER_M))
    height = int(math.ceil((ymax - ymin) * PX_PER_M))
    label_px = LABEL_SIZE_M * PX_PER_M
    font_px = max(10.0, label_px * 0.55)

    lines: list[str] = []
    lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    lines.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">'
    )
    lines.append('<rect width="100%" height="100%" fill="#f8f8f8"/>')

    grid_step = 0.5
    gx = xmin
    while gx <= xmax + 1e-6:
        sx, _ = _world_to_svg(gx, ymin, xmin, ymax)
        _, sy0 = _world_to_svg(gx, ymin, xmin, ymax)
        _, sy1 = _world_to_svg(gx, ymax, xmin, ymax)
        lines.append(
            f'<line x1="{sx:.1f}" y1="{sy0:.1f}" x2="{sx:.1f}" y2="{sy1:.1f}" '
            f'stroke="#ddd" stroke-width="1"/>'
        )
        gx += grid_step
    gy = ymin
    while gy <= ymax + 1e-6:
        sx0, sy = _world_to_svg(xmin, gy, xmin, ymax)
        sx1, _ = _world_to_svg(xmax, gy, xmin, ymax)
        lines.append(
            f'<line x1="{sx0:.1f}" y1="{sy:.1f}" x2="{sx1:.1f}" y2="{sy:.1f}" '
            f'stroke="#ddd" stroke-width="1"/>'
        )
        gy += grid_step

    lines.append(
        f'<text x="12" y="20" font-size="14" fill="#333">'
        f'Field waypoints (parallel A/B/C rows; pickup blue, dropoff red)</text>'
    )

    # Row guide lines: A (0-3), B (4-7), C low (0-3 dropoff)
    _draw_row_guide(
        lines, pickup[0][0], pickup[0][1], pickup[3][0], pickup[3][1],
        "#2ca02c", xmin, ymax,
    )
    _draw_row_guide(
        lines, pickup[4][0], pickup[4][1], pickup[7][0], pickup[7][1],
        "#9467bd", xmin, ymax,
    )
    _draw_row_guide(
        lines, dropoff[0][0], dropoff[0][1], dropoff[3][0], dropoff[3][1],
        "#ff7f0e", xmin, ymax,
    )

    def marker(x: float, y: float, idx: str, color: str, shape: str) -> None:
        sx, sy = _world_to_svg(x, y, xmin, ymax)
        if shape == "circle":
            r = label_px * 0.35
            lines.append(
                f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{r:.1f}" fill="{color}"/>'
            )
        else:
            half = label_px * 0.35
            lines.append(
                f'<rect x="{sx - half:.1f}" y="{sy - half:.1f}" '
                f'width="{2 * half:.1f}" height="{2 * half:.1f}" fill="{color}"/>'
            )
        lines.append(
            f'<text x="{sx:.1f}" y="{sy:.1f}" text-anchor="middle" '
            f'dominant-baseline="central" font-size="{font_px:.1f}" '
            f'font-weight="bold" fill="white">{idx}</text>'
        )

    for i, (x, y, _yaw, _front) in enumerate(pickup):
        marker(x, y, str(i), "#1f77b4", "circle")

    for i, (x, y, _yaw) in enumerate(dropoff):
        marker(x, y, str(i), "#d62728", "square")

    _draw_star(lines, a1[0], a1[1], "A1", "#2ca02c", xmin, ymax, label_px)
    _draw_star(lines, b4[0], b4[1], "B4", "#9467bd", xmin, ymax, label_px)
    _draw_star(lines, c1[0], c1[1], "C1", "#8c564b", xmin, ymax, label_px)
    _draw_star(lines, c4[0], c4[1], "C4", "#ff7f0e", xmin, ymax, label_px)

    lines.append("</svg>")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generated", type=Path, default=DEFAULT_GENERATED)
    parser.add_argument("--anchors", type=Path, default=DEFAULT_ANCHORS)
    parser.add_argument("--output", type=Path, default=DEFAULT_VIZ)
    parser.add_argument("--from-anchors", action="store_true")
    args = parser.parse_args()

    out = args.output
    if out.suffix.lower() == ".png":
        out = out.with_suffix(".svg")

    if args.from_anchors:
        result = load_anchors_from_file(args.anchors)
    else:
        if not args.generated.is_file():
            print(f"Missing {args.generated}; use --from-anchors or run generate first")
            return 1
        result = _load_result_from_generated(args.generated)

    render_field_waypoints(result, out)
    print(f"Wrote: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
