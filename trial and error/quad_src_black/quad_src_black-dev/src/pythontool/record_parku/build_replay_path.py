#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from common import (
    ReplayPoint,
    compute_arc_lengths,
    filter_pose_points,
    median_filter_replay_xy,
    read_pose_csv,
    remove_small_loops,
    resample_path,
    smooth_replay_xy,
)


def read_events(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def protected_s_values(events: list[dict[str, str]], action_sequences: list[dict[str, str]]) -> list[float]:
    values: list[float] = []
    for event in events:
        try:
            values.append(float(event["s"]))
        except (KeyError, ValueError):
            continue
    for sequence in action_sequences:
        try:
            values.append(float(sequence["trigger_s"]))
        except (KeyError, ValueError):
            continue
    return values


def write_replay_path(path: Path, points: list[ReplayPoint]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["s", "x", "y", "yaw"])
        for p in points:
            writer.writerow([f"{p.s:.6f}", f"{p.x:.6f}", f"{p.y:.6f}", f"{p.yaw:.6f}"])


def svg_escape(text: object) -> str:
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def sample_replay_by_s(points: list[ReplayPoint], query_s: float) -> ReplayPoint:
    if query_s <= points[0].s:
        return points[0]
    if query_s >= points[-1].s:
        return points[-1]
    lo = 0
    hi = len(points) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if points[mid].s < query_s:
            lo = mid
        else:
            hi = mid
    a = points[lo]
    b = points[hi]
    u = (query_s - a.s) / max(b.s - a.s, 1e-9)
    dyaw = b.yaw - a.yaw
    while dyaw > math.pi:
        dyaw -= 2.0 * math.pi
    while dyaw < -math.pi:
        dyaw += 2.0 * math.pi
    yaw = a.yaw + u * dyaw
    return ReplayPoint(
        query_s,
        a.x + u * (b.x - a.x),
        a.y + u * (b.y - a.y),
        math.atan2(math.sin(yaw), math.cos(yaw)),
    )


def yaw_arrow_samples(
    points: list[ReplayPoint],
    interval_s: float = 0.3,
    left_offset: float = 0.10,
    length: float = 0.18,
) -> list[tuple[float, float, float, float]]:
    if not points or interval_s <= 0.0:
        return []
    out: list[tuple[float, float, float, float]] = []
    s = points[0].s
    while s <= points[-1].s + 1e-9:
        p = sample_replay_by_s(points, s)
        hx = math.cos(p.yaw)
        hy = math.sin(p.yaw)
        lx = -hy
        ly = hx
        x0 = p.x + lx * left_offset
        y0 = p.y + ly * left_offset
        out.append((x0, y0, x0 + hx * length, y0 + hy * length))
        s += interval_s
    return out


def event_color(name: str) -> str:
    if name in {"JUMP", "STRIDE", "SMALL_JUMP"}:
        return "#d62728"
    if name.startswith("POLICY_"):
        return "#1f77b4"
    if name.startswith("ENTER_"):
        return "#2ca02c"
    return "#6f42c1"


def write_route_svg(path: Path, points: list[ReplayPoint], events: list[dict[str, str]]) -> None:
    if not points:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    for e in events:
        xs.append(float(e["x"]))
        ys.append(float(e["y"]))

    pad = 0.5
    xmin, xmax = min(xs) - pad, max(xs) + pad
    ymin, ymax = min(ys) - pad, max(ys) + pad
    width, height, margin = 1000.0, 760.0, 55.0
    span_x = max(xmax - xmin, 1e-6)
    span_y = max(ymax - ymin, 1e-6)
    grid_step = 0.2
    major_step = 1.0

    def to_svg(x: float, y: float) -> tuple[float, float]:
        sx = margin + (x - xmin) / span_x * (width - 2.0 * margin)
        sy = height - margin - (y - ymin) / span_y * (height - 2.0 * margin)
        return sx, sy

    def grid_values(vmin: float, vmax: float, step: float) -> list[float]:
        start = math.floor(vmin / step) * step
        count = int(math.ceil((vmax - start) / step)) + 1
        return [start + idx * step for idx in range(count)]

    def is_major_grid(value: float) -> bool:
        return abs(value / major_step - round(value / major_step)) < 1e-6

    lines: list[str] = []
    lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" viewBox="0 0 {width:.0f} {height:.0f}">')
    lines.append('<defs><marker id="yaw-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto" markerUnits="strokeWidth"><path d="M0,0 L8,4 L0,8 Z" fill="#17becf"/></marker></defs>')
    lines.append('<rect width="100%" height="100%" fill="#fafafa"/>')
    lines.append('<text x="18" y="28" font-size="16" fill="#222">record_parku route.svg</text>')
    lines.append(f'<text x="18" y="48" font-size="12" fill="#555">points={len(points)} length={points[-1].s:.3f}m events={len(events)} yaw arrows=0.3m</text>')

    for gx in grid_values(xmin, xmax, grid_step):
        if gx < xmin or gx > xmax:
            continue
        a = to_svg(gx, ymin)
        b = to_svg(gx, ymax)
        stroke = "#e0e0e0" if is_major_grid(gx) else "#eeeeee"
        width_px = "1.0" if is_major_grid(gx) else "0.6"
        lines.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{stroke}" stroke-width="{width_px}"/>')
    for gy in grid_values(ymin, ymax, grid_step):
        if gy < ymin or gy > ymax:
            continue
        a = to_svg(xmin, gy)
        b = to_svg(xmax, gy)
        stroke = "#e0e0e0" if is_major_grid(gy) else "#eeeeee"
        width_px = "1.0" if is_major_grid(gy) else "0.6"
        lines.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{stroke}" stroke-width="{width_px}"/>')

    if xmin <= 0.0 <= xmax:
        a = to_svg(0.0, ymin)
        b = to_svg(0.0, ymax)
        lines.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="#777" stroke-width="1.6"/>')
        lines.append(f'<text x="{b[0] + 6:.1f}" y="{b[1] + 13:.1f}" font-size="12" fill="#555">y (m)</text>')
    if ymin <= 0.0 <= ymax:
        a = to_svg(xmin, 0.0)
        b = to_svg(xmax, 0.0)
        lines.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="#777" stroke-width="1.6"/>')
        lines.append(f'<text x="{b[0] - 42:.1f}" y="{b[1] - 7:.1f}" font-size="12" fill="#555">x (m)</text>')

    tick_y = to_svg(0.0, 0.0)[1] if ymin <= 0.0 <= ymax else height - margin + 18.0
    for gx in grid_values(xmin, xmax, major_step):
        if xmin <= gx <= xmax:
            p = to_svg(gx, 0.0 if ymin <= 0.0 <= ymax else ymin)
            lines.append(f'<text x="{p[0] - 8:.1f}" y="{tick_y + 14:.1f}" font-size="10" fill="#777">{gx:.0f}</text>')
    tick_x = to_svg(0.0, 0.0)[0] if xmin <= 0.0 <= xmax else margin - 34.0
    for gy in grid_values(ymin, ymax, major_step):
        if ymin <= gy <= ymax and abs(gy) > 1e-6:
            p = to_svg(0.0 if xmin <= 0.0 <= xmax else xmin, gy)
            lines.append(f'<text x="{tick_x + 6:.1f}" y="{p[1] + 3:.1f}" font-size="10" fill="#777">{gy:.0f}</text>')

    polyline = " ".join(f"{to_svg(p.x, p.y)[0]:.1f},{to_svg(p.x, p.y)[1]:.1f}" for p in points)
    lines.append(f'<polyline points="{polyline}" fill="none" stroke="#222" stroke-width="2.5"/>')
    for x0, y0, x1, y1 in yaw_arrow_samples(points):
        a = to_svg(x0, y0)
        b = to_svg(x1, y1)
        lines.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="#17becf" stroke-width="1.3" marker-end="url(#yaw-arrow)"/>')

    start = to_svg(points[0].x, points[0].y)
    end = to_svg(points[-1].x, points[-1].y)
    lines.append(f'<circle cx="{start[0]:.1f}" cy="{start[1]:.1f}" r="8" fill="#2ca02c"/>')
    lines.append(f'<text x="{start[0] + 10:.1f}" y="{start[1] - 8:.1f}" font-size="12">START s=0</text>')
    lines.append(f'<circle cx="{end[0]:.1f}" cy="{end[1]:.1f}" r="8" fill="#d62728"/>')
    lines.append(f'<text x="{end[0] + 10:.1f}" y="{end[1] - 8:.1f}" font-size="12">END s={points[-1].s:.2f}</text>')

    for e in events:
        x = float(e["x"])
        y = float(e["y"])
        s = float(e["s"])
        name = e["event_name"]
        p = to_svg(x, y)
        color = event_color(name)
        lines.append(f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="6" fill="{color}" stroke="#111"/>')
        lines.append(f'<text x="{p[0] + 8:.1f}" y="{p[1] + 4:.1f}" font-size="11" fill="#222">{svg_escape(name)} s={s:.2f}</text>')

    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build replay_path.csv and route.svg from a record_parku session")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--sample-step", type=float, default=0.02)
    parser.add_argument("--min-step", type=float, default=0.005, help="drop raw pose points closer than this")
    parser.add_argument("--min-yaw-step", type=float, default=0.03, help="keep raw pose points whose yaw changes by at least this many radians")
    parser.add_argument("--yaw-weight", type=float, default=0.25, help="meters of progress per radian of yaw change")
    parser.add_argument("--max-jump", type=float, default=0.50, help="drop raw pose jumps larger than this")
    parser.add_argument("--xy-median-window", type=float, default=0.06, help="s-window in meters for removing isolated x/y spikes; 0 disables it")
    parser.add_argument("--xy-smooth-window", type=float, default=0.10, help="s-window in meters for light x/y smoothing; 0 disables it")
    parser.add_argument("--xy-smooth-passes", type=int, default=1, help="number of x/y smoothing passes")
    parser.add_argument("--remove-small-loops", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--loop-close-xy", type=float, default=0.08, help="x/y distance in meters for detecting loop closure")
    parser.add_argument("--loop-min-span-s", type=float, default=0.12, help="minimum s span in meters for loop removal")
    parser.add_argument("--loop-max-span-s", type=float, default=0.80, help="maximum s span in meters for loop removal")
    parser.add_argument("--loop-protect-margin-s", type=float, default=0.20, help="do not remove loops near events/action sequences")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--svg", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    session = args.session.resolve()
    output = args.output.resolve() if args.output else session / "replay_path.csv"
    svg = args.svg.resolve() if args.svg else session / "route.svg"

    raw = read_pose_csv(session / "raw_pose.csv")
    events = read_events(session / "events.csv")
    action_sequences = read_events(session / "action_sequences.csv")
    filtered = filter_pose_points(raw, args.min_step, args.max_jump, args.min_yaw_step)
    with_s = compute_arc_lengths(filtered, args.yaw_weight)
    loop_removed_points = 0
    if args.remove_small_loops:
        with_s, loop_removed_points = remove_small_loops(
            with_s,
            args.loop_close_xy,
            args.loop_min_span_s,
            args.loop_max_span_s,
            protected_s_values(events, action_sequences),
            args.loop_protect_margin_s,
        )
    replay = resample_path(with_s, args.sample_step)
    replay = median_filter_replay_xy(replay, args.xy_median_window)
    replay = smooth_replay_xy(replay, args.xy_smooth_window, args.xy_smooth_passes)

    write_replay_path(output, replay)
    write_route_svg(svg, replay, events)

    print(f"Raw poses: {len(raw)}")
    print(f"Filtered poses: {len(filtered)}")
    print(f"Loop-removed pose points: {loop_removed_points}")
    print(f"Replay points: {len(replay)}")
    print(f"Total length: {replay[-1].s:.3f} m")
    print(
        "Small loop removal: "
        f"{'on' if args.remove_small_loops else 'off'}, "
        f"close_xy={args.loop_close_xy:.3f}m, "
        f"span=[{args.loop_min_span_s:.3f},{args.loop_max_span_s:.3f}]m, "
        f"protect_margin={args.loop_protect_margin_s:.3f}m"
    )
    print(f"XY median window: {args.xy_median_window:.3f} m")
    print(f"XY smooth window: {args.xy_smooth_window:.3f} m, passes={args.xy_smooth_passes}")
    print(f"Wrote {output}")
    print(f"Wrote {svg}")


if __name__ == "__main__":
    main()
