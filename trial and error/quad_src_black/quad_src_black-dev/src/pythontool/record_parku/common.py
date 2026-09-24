#!/usr/bin/env python3
from __future__ import annotations

import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


EVENT_NAMES: dict[int, str] = {
    0: "ENABLE",
    1: "DISABLE",
    2: "STAND",
    3: "DAMPING",
    4: "ENTER_RL",
    5: "STRIDE",
    6: "SMALL_JUMP",
    7: "JUMP",
    8: "KNEEL_CRAWL",
    9: "POLICY_TROT",
    10: "POLICY_CREEP",
    11: "POLICY_UPSTAIR",
    12: "ENTER_AUTO",
    13: "ENTER_MANUAL",
    14: "QR_RECOGNITION",
    15: "PICK_UP",
    16: "PLACE_LOW",
    17: "PLACE_HIGH",
    18: "LAY_ARM",
    19: "SAVE",
    20: "DELETE",
    21: "POLICY_KNEEL_CRAWL",
}

NON_REPEAT_ON_REWIND = {
    "ENABLE",
    "DISABLE",
    "ENTER_AUTO",
    "ENTER_MANUAL",
}


@dataclass
class PoseSample:
    time: float
    x: float
    y: float
    yaw: float
    s: float = 0.0


@dataclass
class ReplayPoint:
    s: float
    x: float
    y: float
    yaw: float


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_yaws(yaws: Iterable[float]) -> list[float]:
    values = list(yaws)
    if not values:
        return []

    out = [values[0]]
    offset = 0.0
    prev_raw = values[0]
    for raw in values[1:]:
        diff = raw - prev_raw
        if diff > math.pi:
            offset -= 2.0 * math.pi
        elif diff < -math.pi:
            offset += 2.0 * math.pi
        out.append(raw + offset)
        prev_raw = raw
    return out


def dist_xy(a: PoseSample | ReplayPoint, b: PoseSample | ReplayPoint) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def compute_arc_lengths(points: list[PoseSample], yaw_weight: float = 0.0) -> list[PoseSample]:
    if not points:
        return []

    out: list[PoseSample] = []
    s = 0.0
    prev = points[0]
    out.append(PoseSample(prev.time, prev.x, prev.y, prev.yaw, s))
    for p in points[1:]:
        dxy = math.hypot(p.x - prev.x, p.y - prev.y)
        dyaw = abs(normalize_angle(p.yaw - prev.yaw))
        s += math.hypot(dxy, yaw_weight * dyaw)
        out.append(PoseSample(p.time, p.x, p.y, p.yaw, s))
        prev = p
    return out


def read_pose_csv(path: Path) -> list[PoseSample]:
    points: list[PoseSample] = []
    with path.open(encoding="utf-8", newline="") as f:
        reader = csv.DictReader(row for row in f if not row.lstrip().startswith("#"))
        for row in reader:
            points.append(PoseSample(
                time=float(row["time"]),
                x=float(row["x"]),
                y=float(row["y"]),
                yaw=float(row["yaw"]),
                s=float(row.get("s") or 0.0),
            ))
    return points


def write_pose_csv(path: Path, points: list[PoseSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "s", "x", "y", "yaw"])
        for p in points:
            writer.writerow([
                f"{p.time:.6f}",
                f"{p.s:.6f}",
                f"{p.x:.6f}",
                f"{p.y:.6f}",
                f"{p.yaw:.6f}",
            ])


def filter_pose_points(
    points: list[PoseSample],
    min_step: float,
    max_jump: float,
    min_yaw_step: float = 0.0,
) -> list[PoseSample]:
    if not points:
        return []

    out = [points[0]]
    for p in points[1:]:
        d = math.hypot(p.x - out[-1].x, p.y - out[-1].y)
        dyaw = abs(normalize_angle(p.yaw - out[-1].yaw))
        if d > max_jump:
            continue
        if d >= min_step or dyaw >= min_yaw_step:
            out.append(p)
    if len(out) == 1 and len(points) > 1:
        out.append(points[-1])
    return out


def sample_by_s(points: list[PoseSample], s_query: float) -> ReplayPoint:
    if not points:
        raise ValueError("cannot sample empty path")
    if s_query <= points[0].s:
        p = points[0]
        return ReplayPoint(p.s, p.x, p.y, p.yaw)
    if s_query >= points[-1].s:
        p = points[-1]
        return ReplayPoint(p.s, p.x, p.y, p.yaw)

    s_values = [p.s for p in points]
    hi = bisect.bisect_left(s_values, s_query)
    lo = max(0, hi - 1)
    a = points[lo]
    b = points[hi]
    span = max(b.s - a.s, 1e-9)
    u = (s_query - a.s) / span

    yaws = unwrap_yaws([a.yaw, b.yaw])
    yaw = normalize_angle(yaws[0] + u * (yaws[1] - yaws[0]))
    return ReplayPoint(
        s=s_query,
        x=a.x + u * (b.x - a.x),
        y=a.y + u * (b.y - a.y),
        yaw=yaw,
    )


def resample_path(points: list[PoseSample], step: float) -> list[ReplayPoint]:
    if len(points) < 2:
        raise ValueError("need at least two pose points to resample")
    if step <= 0.0:
        raise ValueError("sample step must be positive")

    total = points[-1].s
    out: list[ReplayPoint] = []
    s = 0.0
    while s < total:
        out.append(sample_by_s(points, s))
        s += step
    out.append(sample_by_s(points, total))
    return out


def has_protected_s(protected_s: list[float], begin_s: float, end_s: float, margin_s: float) -> bool:
    lo = min(begin_s, end_s) - max(0.0, margin_s)
    hi = max(begin_s, end_s) + max(0.0, margin_s)
    return any(lo <= s <= hi for s in protected_s)


def remove_small_loops(
    points: list[PoseSample],
    close_xy: float,
    min_span_s: float,
    max_span_s: float,
    protected_s: list[float],
    protected_margin_s: float,
) -> tuple[list[PoseSample], int]:
    """Remove local x/y loops while preserving endpoint s values for event alignment."""
    if len(points) < 3 or close_xy <= 0.0 or max_span_s <= 0.0:
        return list(points), 0

    out: list[PoseSample] = []
    removed = 0
    i = 0
    while i < len(points):
        if i >= len(points) - 2:
            out.extend(points[i:])
            break

        best_j: int | None = None
        for j in range(i + 2, len(points)):
            span_s = points[j].s - points[i].s
            if span_s > max_span_s:
                break
            if span_s < min_span_s:
                continue
            if dist_xy(points[i], points[j]) > close_xy:
                continue
            if has_protected_s(protected_s, points[i].s, points[j].s, protected_margin_s):
                continue
            best_j = j

        if best_j is None:
            out.append(points[i])
            i += 1
            continue

        out.append(points[i])
        removed += best_j - i - 1
        i = best_j

    if out and out[-1].s != points[-1].s:
        out.append(points[-1])
    return out, removed


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def _window_indices_by_s(points: list[ReplayPoint], index: int, window_m: float) -> tuple[int, int]:
    if window_m <= 0.0:
        return index, index + 1
    center_s = points[index].s
    lo = index
    while lo > 0 and center_s - points[lo - 1].s <= window_m:
        lo -= 1
    hi = index + 1
    while hi < len(points) and points[hi].s - center_s <= window_m:
        hi += 1
    return lo, hi


def median_filter_replay_xy(points: list[ReplayPoint], window_m: float) -> list[ReplayPoint]:
    """Remove isolated x/y spikes while preserving s and yaw exactly."""
    if len(points) < 3 or window_m <= 0.0:
        return list(points)

    out: list[ReplayPoint] = []
    for i, p in enumerate(points):
        if i == 0 or i == len(points) - 1:
            out.append(ReplayPoint(p.s, p.x, p.y, p.yaw))
            continue
        lo, hi = _window_indices_by_s(points, i, window_m)
        xs = [q.x for q in points[lo:hi]]
        ys = [q.y for q in points[lo:hi]]
        out.append(ReplayPoint(p.s, _median(xs), _median(ys), p.yaw))
    return out


def smooth_replay_xy(points: list[ReplayPoint], window_m: float, passes: int = 1) -> list[ReplayPoint]:
    """Lightly smooth x/y using an s-window; s and yaw are not changed."""
    if len(points) < 3 or window_m <= 0.0 or passes <= 0:
        return list(points)

    current = list(points)
    for _ in range(passes):
        out: list[ReplayPoint] = []
        for i, p in enumerate(current):
            if i == 0 or i == len(current) - 1:
                out.append(ReplayPoint(p.s, p.x, p.y, p.yaw))
                continue
            lo, hi = _window_indices_by_s(current, i, window_m)
            weighted_x = 0.0
            weighted_y = 0.0
            weight_sum = 0.0
            for q in current[lo:hi]:
                distance_s = abs(q.s - p.s)
                weight = max(0.0, 1.0 - distance_s / max(window_m, 1e-9))
                if weight <= 0.0:
                    continue
                weighted_x += q.x * weight
                weighted_y += q.y * weight
                weight_sum += weight
            if weight_sum <= 0.0:
                out.append(ReplayPoint(p.s, p.x, p.y, p.yaw))
            else:
                out.append(ReplayPoint(p.s, weighted_x / weight_sum, weighted_y / weight_sum, p.yaw))
        current = out
    return current


def nearest_pose_by_time(points: list[PoseSample], t: float) -> PoseSample:
    if not points:
        raise ValueError("cannot match event to empty pose list")
    times = [p.time for p in points]
    idx = bisect.bisect_left(times, t)
    candidates = []
    if idx < len(points):
        candidates.append(points[idx])
    if idx > 0:
        candidates.append(points[idx - 1])
    return min(candidates, key=lambda p: abs(p.time - t))


def event_name(event_code: int) -> str:
    return EVENT_NAMES.get(event_code, f"UNKNOWN_{event_code}")


def repeat_on_rewind(name: str) -> bool:
    return name not in NON_REPEAT_ON_REWIND
