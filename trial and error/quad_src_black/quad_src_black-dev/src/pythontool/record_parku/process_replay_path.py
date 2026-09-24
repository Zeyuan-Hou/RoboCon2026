#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path

from build_replay_path import read_events, svg_escape, write_replay_path, write_route_svg
from common import ReplayPoint, normalize_angle, unwrap_yaws


@dataclass
class SourcePoint:
    source_s: float
    x: float
    y: float
    yaw: float = 0.0
    s: float = 0.0


@dataclass
class CircleCandidate:
    s0: float
    s1: float
    cx: float
    cy: float
    r: float
    residual: float
    turn: float
    weight: float


@dataclass
class CircleCluster:
    cx: float
    cy: float
    r: float
    weight: float
    count: int


@dataclass
class TemplateFit:
    centers: list[tuple[float, float]]
    radius: float
    theta: float
    score: float
    clusters: list[CircleCluster]


def read_replay_path(path: Path) -> list[ReplayPoint]:
    points: list[ReplayPoint] = []
    with path.open(encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            points.append(ReplayPoint(
                s=float(row["s"]),
                x=float(row["x"]),
                y=float(row["y"]),
                yaw=float(row.get("yaw") or 0.0),
            ))
    if len(points) < 2:
        raise SystemExit(f"Need at least two replay points: {path}")
    return points


def sample_replay_by_s(points: list[ReplayPoint], query_s: float) -> ReplayPoint:
    if query_s <= points[0].s:
        return points[0]
    if query_s >= points[-1].s:
        return points[-1]
    values = [p.s for p in points]
    hi = bisect.bisect_left(values, query_s)
    lo = max(0, hi - 1)
    a = points[lo]
    b = points[hi]
    u = (query_s - a.s) / max(b.s - a.s, 1e-9)
    yaws = unwrap_yaws([a.yaw, b.yaw])
    return ReplayPoint(
        query_s,
        a.x + u * (b.x - a.x),
        a.y + u * (b.y - a.y),
        normalize_angle(yaws[0] + u * (yaws[1] - yaws[0])),
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
    end_s = points[-1].s
    while s <= end_s + 1e-9:
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


def write_rows(path: Path, rows: list[dict[str, str]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def event_fieldnames(rows: list[dict[str, str]]) -> list[str]:
    return list(rows[0].keys()) if rows else []


def first_upstairs_interval(events: list[dict[str, str]]) -> tuple[float, float]:
    starts = [
        idx for idx, row in enumerate(events)
        if row.get("event_name") == "POLICY_UPSTAIR"
    ]
    if not starts:
        raise SystemExit("No POLICY_UPSTAIR event found.")
    start_idx = starts[0]
    start_s = float(events[start_idx]["s"])
    end_s: float | None = None
    for row in events[start_idx + 1:]:
        if row.get("event_name") == "POLICY_TROT":
            end_s = float(row["s"])
            break
    if end_s is None:
        raise SystemExit("No POLICY_TROT event found after first POLICY_UPSTAIR.")
    return start_s, end_s


def upstairs_intervals(events: list[dict[str, str]]) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    for start_idx, row in enumerate(events):
        if row.get("event_name") != "POLICY_UPSTAIR":
            continue
        start_s = float(row["s"])
        for end_row in events[start_idx + 1:]:
            if end_row.get("event_name") == "POLICY_TROT":
                intervals.append((start_s, float(end_row["s"])))
                break
    return intervals


def policy_pair_intervals(events: list[dict[str, str]], names: set[str]) -> list[tuple[float, float]]:
    matched: list[tuple[float, str]] = []
    for row in events:
        name = row.get("event_name", "")
        if name in names:
            matched.append((float(row["s"]), name))

    intervals: list[tuple[float, float]] = []
    for (a_s, a_name), (b_s, b_name) in zip(matched, matched[1:]):
        if a_name != b_name:
            intervals.append((min(a_s, b_s), max(a_s, b_s)))
    return intervals


def solve3(matrix: list[list[float]], vector: list[float]) -> tuple[float, float, float] | None:
    a = [row[:] + [rhs] for row, rhs in zip(matrix, vector)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(a[row][col]))
        if abs(a[pivot][col]) < 1e-12:
            return None
        a[col], a[pivot] = a[pivot], a[col]
        denom = a[col][col]
        for item in range(col, 4):
            a[col][item] /= denom
        for row in range(3):
            if row == col:
                continue
            factor = a[row][col]
            for item in range(col, 4):
                a[row][item] -= factor * a[col][item]
    return a[0][3], a[1][3], a[2][3]


def fit_circle(points: list[ReplayPoint]) -> tuple[float, float, float, float] | None:
    if len(points) < 6:
        return None
    sums = [[0.0] * 3 for _ in range(3)]
    rhs = [0.0, 0.0, 0.0]
    for p in points:
        row = [2.0 * p.x, 2.0 * p.y, 1.0]
        value = p.x * p.x + p.y * p.y
        for i in range(3):
            rhs[i] += row[i] * value
            for j in range(3):
                sums[i][j] += row[i] * row[j]
    solved = solve3(sums, rhs)
    if solved is None:
        return None
    cx, cy, c = solved
    radius_sq = cx * cx + cy * cy + c
    if radius_sq <= 1e-9:
        return None
    r = math.sqrt(radius_sq)
    residual = math.sqrt(sum((math.hypot(p.x - cx, p.y - cy) - r) ** 2 for p in points) / len(points))
    return cx, cy, r, residual


def angle_turn(points: list[ReplayPoint], cx: float, cy: float) -> float:
    angles = [math.atan2(p.y - cy, p.x - cx) for p in points]
    unwrapped = unwrap_yaws(angles)
    return abs(unwrapped[-1] - unwrapped[0])


def circle_candidates(points: list[ReplayPoint], window_s: float, step_s: float) -> list[CircleCandidate]:
    if not points:
        return []
    out: list[CircleCandidate] = []
    begin = points[0].s
    end = points[-1].s
    s0 = begin
    while s0 + window_s <= end + 1e-9:
        s1 = s0 + window_s
        segment = [p for p in points if s0 <= p.s <= s1]
        fitted = fit_circle(segment)
        if fitted is not None:
            cx, cy, radius, residual = fitted
            turn = angle_turn(segment, cx, cy)
            if 0.25 <= radius <= 2.5 and turn >= 0.45 and residual <= 0.08:
                out.append(CircleCandidate(
                    s0=s0,
                    s1=s1,
                    cx=cx,
                    cy=cy,
                    r=radius,
                    residual=residual,
                    turn=turn,
                    weight=turn / max(residual, 0.01),
                ))
        s0 += step_s
    return out


def cluster_candidates(candidates: list[CircleCandidate], merge_dist: float) -> list[CircleCluster]:
    clusters: list[list[CircleCandidate]] = []
    for candidate in sorted(candidates, key=lambda item: item.weight, reverse=True):
        for cluster in clusters:
            cx = sum(item.cx * item.weight for item in cluster) / sum(item.weight for item in cluster)
            cy = sum(item.cy * item.weight for item in cluster) / sum(item.weight for item in cluster)
            if math.hypot(candidate.cx - cx, candidate.cy - cy) <= merge_dist:
                cluster.append(candidate)
                break
        else:
            clusters.append([candidate])

    out: list[CircleCluster] = []
    for cluster in clusters:
        weight = sum(item.weight for item in cluster)
        cx = sum(item.cx * item.weight for item in cluster) / weight
        cy = sum(item.cy * item.weight for item in cluster) / weight
        radius = sum(item.r * item.weight for item in cluster) / weight
        out.append(CircleCluster(cx, cy, radius, weight, len(cluster)))
    return sorted(out, key=lambda item: item.weight, reverse=True)


def rotate(point: tuple[float, float], theta: float) -> tuple[float, float]:
    c = math.cos(theta)
    s = math.sin(theta)
    return c * point[0] - s * point[1], s * point[0] + c * point[1]


def fit_circle_template(
    clusters: list[CircleCluster],
    spacing: float,
    down_sign: int,
    radius_override: float | None,
    fixed_theta_deg: float | None,
) -> TemplateFit:
    if not clusters:
        raise SystemExit("No circle candidates found before the first upstairs segment.")
    usable = clusters[: min(10, len(clusters))]
    local = [(0.0, 0.0), (spacing, 0.0), (2.0 * spacing, 0.0), (2.0 * spacing, down_sign * spacing)]

    if fixed_theta_deg is None:
        theta_seeds: list[float] = []
        for i, a in enumerate(usable):
            for b in usable[i + 1:]:
                dist = math.hypot(b.cx - a.cx, b.cy - a.cy)
                if 0.6 * spacing <= dist <= 2.4 * spacing:
                    theta_seeds.append(math.atan2(b.cy - a.cy, b.cx - a.cx))
                    theta_seeds.append(math.atan2(a.cy - b.cy, a.cx - b.cx))
        theta_seeds.extend(idx * math.pi / 36.0 for idx in range(-36, 37))
    else:
        theta_seeds = [math.radians(fixed_theta_deg)]

    best: tuple[float, float, float, float, list[tuple[float, float]]] | None = None
    for theta in theta_seeds:
        rotated = [rotate(item, theta) for item in local]
        for cluster in usable:
            for template_point in rotated:
                tx = cluster.cx - template_point[0]
                ty = cluster.cy - template_point[1]
                centers = [(tx + item[0], ty + item[1]) for item in rotated]
                score = 0.0
                for item in usable:
                    nearest = min(math.hypot(item.cx - cx, item.cy - cy) for cx, cy in centers)
                    score += item.weight * nearest * nearest
                if best is None or score < best[0]:
                    best = (score, theta, tx, ty, centers)
    if best is None:
        raise SystemExit("Could not fit circle template.")

    score, theta, _, _, centers = best
    if radius_override is not None:
        radius = radius_override
    else:
        weighted_r = 0.0
        weight_sum = 0.0
        for cluster in usable:
            nearest = min(math.hypot(cluster.cx - cx, cluster.cy - cy) for cx, cy in centers)
            if nearest <= 0.45:
                weighted_r += cluster.r * cluster.weight
                weight_sum += cluster.weight
        radius = weighted_r / weight_sum if weight_sum > 0.0 else usable[0].r
    return TemplateFit(centers=centers, radius=radius, theta=theta, score=score, clusters=usable)


def manual_circle_template(
    x: float,
    y: float,
    theta_deg: float,
    spacing: float,
    down_sign: int,
    radius: float,
    clusters: list[CircleCluster],
) -> TemplateFit:
    theta = math.radians(theta_deg)
    local = [(0.0, 0.0), (spacing, 0.0), (2.0 * spacing, 0.0), (2.0 * spacing, down_sign * spacing)]
    rotated = [rotate(item, theta) for item in local]
    centers = [(x + item[0], y + item[1]) for item in rotated]
    return TemplateFit(centers=centers, radius=radius, theta=theta, score=0.0, clusters=clusters[: min(10, len(clusters))])


def smooth_source_xy(points: list[SourcePoint], window: int, passes: int) -> list[SourcePoint]:
    if window <= 0 or passes <= 0 or len(points) < 3:
        return points
    current = list(points)
    for _ in range(passes):
        out: list[SourcePoint] = []
        for idx, point in enumerate(current):
            if idx == 0 or idx == len(current) - 1:
                out.append(point)
                continue
            lo = max(0, idx - window)
            hi = min(len(current), idx + window + 1)
            weight_sum = 0.0
            x_sum = 0.0
            y_sum = 0.0
            for j in range(lo, hi):
                weight = 1.0 / (1.0 + abs(j - idx))
                x_sum += current[j].x * weight
                y_sum += current[j].y * weight
                weight_sum += weight
            out.append(SourcePoint(point.source_s, x_sum / weight_sum, y_sum / weight_sum, point.yaw, point.s))
        current = out
    return current


def circularize_pre_upstairs(
    points: list[ReplayPoint],
    first_upstairs_s: float,
    fit: TemplateFit,
    strength: float,
    blend_band: float,
    endpoint_guard_s: float,
    smooth_window: int,
    smooth_passes: int,
) -> list[SourcePoint]:
    out: list[SourcePoint] = []
    for p in points:
        if p.s > first_upstairs_s:
            break
        nearest_center = min(fit.centers, key=lambda center: math.hypot(p.x - center[0], p.y - center[1]))
        dx = p.x - nearest_center[0]
        dy = p.y - nearest_center[1]
        dist = max(math.hypot(dx, dy), 1e-9)
        residual = abs(dist - fit.radius)
        guard = min(max((p.s - points[0].s) / max(endpoint_guard_s, 1e-9), 0.0), 1.0)
        guard *= min(max((first_upstairs_s - p.s) / max(endpoint_guard_s, 1e-9), 0.0), 1.0)
        blend = strength * guard * max(0.0, 1.0 - residual / max(blend_band, 1e-9))
        target_x = nearest_center[0] + dx / dist * fit.radius
        target_y = nearest_center[1] + dy / dist * fit.radius
        out.append(SourcePoint(
            p.s,
            p.x + blend * (target_x - p.x),
            p.y + blend * (target_y - p.y),
            p.yaw,
        ))
    return smooth_source_xy(out, smooth_window, smooth_passes)


def copy_pre_upstairs(
    points: list[ReplayPoint],
    first_upstairs_s: float,
    smooth_window: int,
    smooth_passes: int,
) -> list[SourcePoint]:
    out = [
        SourcePoint(p.s, p.x, p.y, p.yaw)
        for p in points
        if p.s <= first_upstairs_s
    ]
    return smooth_source_xy(out, smooth_window, smooth_passes)


def gap_smooth_pre_upstairs(
    points: list[ReplayPoint],
    first_upstairs_s: float,
    fit: TemplateFit,
    safe_radius: float,
    passes: int,
    alpha: float,
    attract: float,
    attract_window: int,
) -> list[SourcePoint]:
    out = [
        SourcePoint(p.s, p.x, p.y, p.yaw)
        for p in points
        if p.s <= first_upstairs_s
    ]
    if len(out) < 5:
        return out

    c1, c2, c3, c4 = fit.centers
    gaps = [
        ((c1[0] + c2[0]) * 0.5, (c1[1] + c2[1]) * 0.5),
        ((c2[0] + c3[0]) * 0.5, (c2[1] + c3[1]) * 0.5),
        ((c3[0] + c4[0]) * 0.5, (c3[1] + c4[1]) * 0.5),
    ]
    anchors = {0, len(out) - 1}
    gap_targets: list[tuple[int, float, float]] = []
    used: set[int] = set()
    for gx, gy in gaps:
        idx = min(
            (idx for idx in range(1, len(out) - 1) if idx not in used),
            key=lambda idx: math.hypot(out[idx].x - gx, out[idx].y - gy),
        )
        gap_targets.append((idx, gx, gy))
        used.add(idx)

    centers = fit.centers
    for _ in range(max(0, passes)):
        current = [SourcePoint(p.source_s, p.x, p.y, p.yaw, p.s) for p in out]
        for idx in range(1, len(out) - 1):
            if idx in anchors:
                continue
            target_x = 0.5 * (current[idx - 1].x + current[idx + 1].x)
            target_y = 0.5 * (current[idx - 1].y + current[idx + 1].y)
            out[idx].x = current[idx].x + alpha * (target_x - current[idx].x)
            out[idx].y = current[idx].y + alpha * (target_y - current[idx].y)
        for target_idx, gx, gy in gap_targets:
            lo = max(1, target_idx - attract_window)
            hi = min(len(out) - 1, target_idx + attract_window + 1)
            for idx in range(lo, hi):
                distance = (idx - target_idx) / max(float(attract_window), 1.0)
                weight = math.exp(-0.5 * distance * distance)
                pull = attract * weight
                out[idx].x += pull * (gx - out[idx].x)
                out[idx].y += pull * (gy - out[idx].y)
        for idx in range(1, len(out) - 1):
            for cx, cy in centers:
                dx = out[idx].x - cx
                dy = out[idx].y - cy
                dist = math.hypot(dx, dy)
                if 1e-9 < dist < safe_radius:
                    out[idx].x = cx + dx / dist * safe_radius
                    out[idx].y = cy + dy / dist * safe_radius
    return out


def repel_pre_upstairs(
    points: list[ReplayPoint],
    first_upstairs_s: float,
    fit: TemplateFit,
    safe_radius: float,
    margin: float,
    passes: int,
    alpha: float,
) -> list[SourcePoint]:
    out = [
        SourcePoint(p.s, p.x, p.y, p.yaw)
        for p in points
        if p.s <= first_upstairs_s
    ]
    if len(out) < 5:
        return out
    centers = fit.centers
    guard_radius = safe_radius + margin
    for _ in range(max(0, passes)):
        current = [SourcePoint(p.source_s, p.x, p.y, p.yaw, p.s) for p in out]
        for idx in range(1, len(out) - 1):
            target_x = 0.5 * (current[idx - 1].x + current[idx + 1].x)
            target_y = 0.5 * (current[idx - 1].y + current[idx + 1].y)
            out[idx].x = current[idx].x + alpha * (target_x - current[idx].x)
            out[idx].y = current[idx].y + alpha * (target_y - current[idx].y)
            for cx, cy in centers:
                dx = out[idx].x - cx
                dy = out[idx].y - cy
                dist = math.hypot(dx, dy)
                if 1e-9 < dist < guard_radius:
                    target_dist = safe_radius if dist < safe_radius else dist + 0.25 * (guard_radius - dist)
                    out[idx].x = cx + dx / dist * target_dist
                    out[idx].y = cy + dy / dist * target_dist
    return out


def unit(dx: float, dy: float) -> tuple[float, float]:
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return 0.0, 0.0
    return dx / length, dy / length


def append_line(out: list[SourcePoint], start: tuple[float, float], end: tuple[float, float], step: float) -> None:
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(math.ceil(length / step)))
    for idx in range(count):
        u = idx / count
        out.append(SourcePoint(0.0, start[0] + u * (end[0] - start[0]), start[1] + u * (end[1] - start[1])))


def angle_lerp_values(a0: float, a1: float, count: int) -> list[float]:
    while a1 - a0 > math.pi:
        a1 -= 2.0 * math.pi
    while a1 - a0 < -math.pi:
        a1 += 2.0 * math.pi
    return [a0 + (a1 - a0) * idx / count for idx in range(count)]


def l_shape_path(
    start: ReplayPoint,
    end: ReplayPoint,
    radius: float,
    step: float,
    corner_mode: str,
    old_segment: list[ReplayPoint],
) -> list[SourcePoint]:
    candidate_corners: list[tuple[float, float]]
    if corner_mode == "start_x_end_y":
        candidate_corners = [(start.x, end.y)]
    elif corner_mode == "end_x_start_y":
        candidate_corners = [(end.x, start.y)]
    else:
        candidate_corners = [(start.x, end.y), (end.x, start.y)]

    best: list[SourcePoint] | None = None
    best_score = float("inf")
    for corner in candidate_corners:
        d0 = unit(corner[0] - start.x, corner[1] - start.y)
        d1 = unit(end.x - corner[0], end.y - corner[1])
        len0 = math.hypot(corner[0] - start.x, corner[1] - start.y)
        len1 = math.hypot(end.x - corner[0], end.y - corner[1])
        if len0 <= 1e-6 or len1 <= 1e-6:
            continue
        r = min(radius, 0.45 * len0, 0.45 * len1)
        t1 = (corner[0] - d0[0] * r, corner[1] - d0[1] * r)
        t2 = (corner[0] + d1[0] * r, corner[1] + d1[1] * r)
        center = (corner[0] - d0[0] * r + d1[0] * r, corner[1] - d0[1] * r + d1[1] * r)
        path: list[SourcePoint] = []
        append_line(path, (start.x, start.y), t1, step)
        a0 = math.atan2(t1[1] - center[1], t1[0] - center[0])
        a1 = math.atan2(t2[1] - center[1], t2[0] - center[0])
        arc_len = abs(normalize_angle(a1 - a0)) * r
        arc_count = max(2, int(math.ceil(arc_len / step)))
        for angle in angle_lerp_values(a0, a1, arc_count):
            path.append(SourcePoint(0.0, center[0] + math.cos(angle) * r, center[1] + math.sin(angle) * r))
        append_line(path, t2, (end.x, end.y), step)
        path.append(SourcePoint(0.0, end.x, end.y))
        score = mean_distance_to_polyline(old_segment, path)
        if score < best_score:
            best = path
            best_score = score
    if best is None:
        raise SystemExit("Could not build L-shaped first upstairs segment.")
    assign_source_s_by_progress(best, start.s, end.s)
    return dedupe_source_points(best)


def assign_source_s_by_progress(points: list[SourcePoint], source_start: float, source_end: float) -> None:
    if not points:
        return
    lengths = [0.0]
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += math.hypot(b.x - a.x, b.y - a.y)
        lengths.append(total)
    for point, length in zip(points, lengths):
        u = length / max(total, 1e-9)
        point.source_s = source_start + u * (source_end - source_start)


def point_segment_distance(px: float, py: float, a: SourcePoint | ReplayPoint, b: SourcePoint | ReplayPoint) -> float:
    vx = b.x - a.x
    vy = b.y - a.y
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return math.hypot(px - a.x, py - a.y)
    u = max(0.0, min(1.0, ((px - a.x) * vx + (py - a.y) * vy) / denom))
    qx = a.x + u * vx
    qy = a.y + u * vy
    return math.hypot(px - qx, py - qy)


def mean_distance_to_polyline(points: list[ReplayPoint], line: list[SourcePoint]) -> float:
    if len(line) < 2 or not points:
        return float("inf")
    total = 0.0
    for point in points[:: max(1, len(points) // 80)]:
        total += min(point_segment_distance(point.x, point.y, a, b) for a, b in zip(line, line[1:]))
    return total / max(1, len(points[:: max(1, len(points) // 80)]))


def dedupe_source_points(points: list[SourcePoint]) -> list[SourcePoint]:
    out: list[SourcePoint] = []
    for point in points:
        if out and math.hypot(point.x - out[-1].x, point.y - out[-1].y) <= 1e-7:
            out[-1] = point
        else:
            out.append(point)
    return out


def compute_tangent_yaws(points: list[SourcePoint]) -> None:
    if len(points) < 2:
        return
    for idx, point in enumerate(points):
        if idx == 0:
            a, b = points[0], points[1]
        elif idx == len(points) - 1:
            a, b = points[-2], points[-1]
        else:
            a, b = points[idx - 1], points[idx + 1]
        point.yaw = math.atan2(b.y - a.y, b.x - a.x)


def compute_global_s(points: list[SourcePoint], yaw_weight: float) -> None:
    if not points:
        return
    points[0].s = 0.0
    total = 0.0
    prev = points[0]
    for point in points[1:]:
        dxy = math.hypot(point.x - prev.x, point.y - prev.y)
        dyaw = abs(normalize_angle(point.yaw - prev.yaw))
        total += math.hypot(dxy, yaw_weight * dyaw)
        point.s = total
        prev = point


def sample_source_by_new_s(points: list[SourcePoint], query_s: float) -> SourcePoint:
    if query_s <= points[0].s:
        return SourcePoint(points[0].source_s, points[0].x, points[0].y, points[0].yaw, query_s)
    if query_s >= points[-1].s:
        return SourcePoint(points[-1].source_s, points[-1].x, points[-1].y, points[-1].yaw, query_s)
    values = [p.s for p in points]
    hi = bisect.bisect_left(values, query_s)
    lo = max(0, hi - 1)
    a = points[lo]
    b = points[hi]
    u = (query_s - a.s) / max(b.s - a.s, 1e-9)
    yaws = unwrap_yaws([a.yaw, b.yaw])
    return SourcePoint(
        a.source_s + u * (b.source_s - a.source_s),
        a.x + u * (b.x - a.x),
        a.y + u * (b.y - a.y),
        normalize_angle(yaws[0] + u * (yaws[1] - yaws[0])),
        query_s,
    )


def resample_processed(points: list[SourcePoint], step: float) -> list[SourcePoint]:
    compute_tangent_yaws(points)
    compute_global_s(points, yaw_weight=0.0)
    total = points[-1].s
    out: list[SourcePoint] = []
    s = 0.0
    while s < total:
        out.append(sample_source_by_new_s(points, s))
        s += step
    out.append(sample_source_by_new_s(points, total))
    compute_tangent_yaws(out)
    return out


def apply_recorded_yaws(points: list[SourcePoint], reference: list[ReplayPoint]) -> None:
    for point in points:
        recorded = sample_replay_by_s(reference, point.source_s)
        point.yaw = recorded.yaw


def post_smooth_source_points(
    points: list[SourcePoint],
    window_s: float,
    passes: int,
    protected_ranges: list[tuple[float, float]],
    protect_margin_s: float,
) -> list[SourcePoint]:
    if len(points) < 3 or window_s <= 0.0 or passes <= 0:
        return points

    def is_protected(source_s: float) -> bool:
        for begin_s, end_s in protected_ranges:
            if begin_s - protect_margin_s <= source_s <= end_s + protect_margin_s:
                return True
        return False

    current = [SourcePoint(p.source_s, p.x, p.y, p.yaw, p.s) for p in points]
    sigma = max(window_s * 0.5, 1e-9)
    for _ in range(passes):
        out: list[SourcePoint] = []
        for idx, point in enumerate(current):
            if idx == 0 or idx == len(current) - 1 or is_protected(point.source_s):
                out.append(SourcePoint(point.source_s, point.x, point.y, point.yaw, point.s))
                continue
            x_sum = 0.0
            y_sum = 0.0
            weight_sum = 0.0
            lo = idx
            while lo > 0 and point.source_s - current[lo - 1].source_s <= window_s:
                lo -= 1
            hi = idx + 1
            while hi < len(current) and current[hi].source_s - point.source_s <= window_s:
                hi += 1
            for neighbor in current[lo:hi]:
                if is_protected(neighbor.source_s):
                    continue
                ds = neighbor.source_s - point.source_s
                weight = math.exp(-0.5 * (ds / sigma) ** 2)
                x_sum += neighbor.x * weight
                y_sum += neighbor.y * weight
                weight_sum += weight
            if weight_sum <= 0.0:
                out.append(SourcePoint(point.source_s, point.x, point.y, point.yaw, point.s))
            else:
                out.append(SourcePoint(point.source_s, x_sum / weight_sum, y_sum / weight_sum, point.yaw, point.s))
        current = out
    compute_tangent_yaws(current)
    return current


def containing_interval(source_s: float, intervals: list[tuple[float, float]]) -> tuple[float, float] | None:
    for begin_s, end_s in intervals:
        if begin_s <= source_s <= end_s:
            return begin_s, end_s
    return None


def smooth_source_intervals(
    points: list[SourcePoint],
    intervals: list[tuple[float, float]],
    window_s: float,
    passes: int,
    endpoint_guard_s: float,
    blend: float,
) -> list[SourcePoint]:
    if len(points) < 3 or not intervals or window_s <= 0.0 or passes <= 0 or blend <= 0.0:
        return points

    current = [SourcePoint(p.source_s, p.x, p.y, p.yaw, p.s) for p in points]
    sigma = max(window_s * 0.5, 1e-9)
    blend = min(max(blend, 0.0), 1.0)
    for _ in range(passes):
        out: list[SourcePoint] = []
        for idx, point in enumerate(current):
            interval = containing_interval(point.source_s, intervals)
            if idx == 0 or idx == len(current) - 1 or interval is None:
                out.append(SourcePoint(point.source_s, point.x, point.y, point.yaw, point.s))
                continue

            begin_s, end_s = interval
            guard = 1.0
            if endpoint_guard_s > 0.0:
                edge_s = min(point.source_s - begin_s, end_s - point.source_s)
                t = min(max(edge_s / endpoint_guard_s, 0.0), 1.0)
                guard = t * t * (3.0 - 2.0 * t)
            if guard <= 0.0:
                out.append(SourcePoint(point.source_s, point.x, point.y, point.yaw, point.s))
                continue

            x_sum = 0.0
            y_sum = 0.0
            weight_sum = 0.0
            lo = idx
            while lo > 0 and point.source_s - current[lo - 1].source_s <= window_s:
                lo -= 1
            hi = idx + 1
            while hi < len(current) and current[hi].source_s - point.source_s <= window_s:
                hi += 1
            for neighbor in current[lo:hi]:
                if not (begin_s <= neighbor.source_s <= end_s):
                    continue
                ds = neighbor.source_s - point.source_s
                weight = math.exp(-0.5 * (ds / sigma) ** 2)
                x_sum += neighbor.x * weight
                y_sum += neighbor.y * weight
                weight_sum += weight
            if weight_sum <= 0.0:
                out.append(SourcePoint(point.source_s, point.x, point.y, point.yaw, point.s))
                continue

            target_x = x_sum / weight_sum
            target_y = y_sum / weight_sum
            weight = blend * guard
            out.append(SourcePoint(
                point.source_s,
                point.x + weight * (target_x - point.x),
                point.y + weight * (target_y - point.y),
                point.yaw,
                point.s,
            ))
        current = out
    compute_tangent_yaws(current)
    return current


def point_is_in_ranges(source_s: float, ranges: list[tuple[float, float]], margin_s: float) -> bool:
    for begin_s, end_s in ranges:
        if begin_s - margin_s <= source_s <= end_s + margin_s:
            return True
    return False


def source_path_length(points: list[SourcePoint], begin_idx: int, end_idx: int) -> float:
    total = 0.0
    for a, b in zip(points[begin_idx:end_idx], points[begin_idx + 1:end_idx + 1]):
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total


def line_segment_stats(points: list[SourcePoint], begin_idx: int, end_idx: int) -> tuple[float, float, float, float]:
    a = points[begin_idx]
    b = points[end_idx]
    vx = b.x - a.x
    vy = b.y - a.y
    chord = math.hypot(vx, vy)
    if chord <= 1e-9:
        return 0.0, float("inf"), float("inf"), float("inf")

    max_dev = 0.0
    dev_sum = 0.0
    count = 0
    for point in points[begin_idx + 1:end_idx]:
        dev = abs((point.x - a.x) * vy - (point.y - a.y) * vx) / chord
        max_dev = max(max_dev, dev)
        dev_sum += dev
        count += 1
    mean_dev = dev_sum / max(count, 1)
    path_len = source_path_length(points, begin_idx, end_idx)
    return chord, max_dev, mean_dev, path_len


def max_heading_deviation_from_line(points: list[SourcePoint], begin_idx: int, end_idx: int) -> float:
    a = points[begin_idx]
    b = points[end_idx]
    line_heading = math.atan2(b.y - a.y, b.x - a.x)
    max_dev = 0.0
    for p0, p1 in zip(points[begin_idx:end_idx], points[begin_idx + 1:end_idx + 1]):
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        if math.hypot(dx, dy) <= 1e-6:
            continue
        max_dev = max(max_dev, abs(normalize_angle(math.atan2(dy, dx) - line_heading)))
    return max_dev


def rdp_simplify_indices(points: list[SourcePoint], begin_idx: int, end_idx: int, epsilon: float) -> list[int]:
    if end_idx <= begin_idx + 1:
        return [begin_idx, end_idx]
    keep = {begin_idx, end_idx}
    stack = [(begin_idx, end_idx)]
    while stack:
        start_idx, stop_idx = stack.pop()
        a = points[start_idx]
        b = points[stop_idx]
        vx = b.x - a.x
        vy = b.y - a.y
        chord = math.hypot(vx, vy)
        if chord <= 1e-9:
            continue
        max_dev = -1.0
        split_idx: int | None = None
        for idx in range(start_idx + 1, stop_idx):
            p = points[idx]
            dev = abs((p.x - a.x) * vy - (p.y - a.y) * vx) / chord
            if dev > max_dev:
                max_dev = dev
                split_idx = idx
        if split_idx is not None and max_dev > epsilon:
            keep.add(split_idx)
            stack.append((start_idx, split_idx))
            stack.append((split_idx, stop_idx))
    return sorted(keep)


def straighten_long_near_lines(
    points: list[SourcePoint],
    min_length: float,
    max_deviation: float,
    max_mean_deviation: float,
    max_heading_deg: float,
    max_path_ratio: float,
    blend: float,
    taper_s: float,
    protected_ranges: list[tuple[float, float]],
    protect_margin_s: float,
) -> tuple[list[SourcePoint], int]:
    if (
        len(points) < 4
        or min_length <= 0.0
        or max_deviation <= 0.0
        or blend <= 0.0
    ):
        return points, 0

    current = [SourcePoint(p.source_s, p.x, p.y, p.yaw, p.s) for p in points]
    protected = [
        point_is_in_ranges(point.source_s, protected_ranges, protect_margin_s)
        for point in current
    ]
    max_heading = math.radians(max_heading_deg)
    applied: list[tuple[int, int]] = []
    min_points = 8

    def candidate_ok(begin_idx: int, end_idx: int) -> bool:
        if end_idx - begin_idx + 1 < min_points:
            return False
        if any(protected[begin_idx:end_idx + 1]):
            return False
        chord, max_dev, mean_dev, path_len = line_segment_stats(current, begin_idx, end_idx)
        if chord < min_length:
            return False
        if max_dev > max_deviation or mean_dev > max_mean_deviation:
            return False
        if path_len / max(chord, 1e-9) > max_path_ratio:
            return False
        return max_heading_deviation_from_line(current, begin_idx, end_idx) <= max_heading

    chunks: list[tuple[int, int]] = []
    idx = 0
    while idx < len(current):
        while idx < len(current) and protected[idx]:
            idx += 1
        begin_idx = idx
        while idx < len(current) and not protected[idx]:
            idx += 1
        end_idx = idx - 1
        if end_idx - begin_idx + 1 >= min_points:
            chunks.append((begin_idx, end_idx))

    for chunk_begin, chunk_end in chunks:
        simplified = rdp_simplify_indices(current, chunk_begin, chunk_end, max_deviation)
        for idx, best_end in zip(simplified, simplified[1:]):
            if not candidate_ok(idx, best_end):
                continue

            applied.append((idx, best_end))
            a = current[idx]
            b = current[best_end]
            span_s = max(b.source_s - a.source_s, 1e-9)
            taper = min(max(taper_s, 0.0), 0.5 * span_s)
            for point_idx in range(idx + 1, best_end):
                point = current[point_idx]
                u = (point.source_s - a.source_s) / span_s
                line_x = a.x + u * (b.x - a.x)
                line_y = a.y + u * (b.y - a.y)
                if taper > 0.0:
                    edge_s = min(point.source_s - a.source_s, b.source_s - point.source_s)
                    t = min(max(edge_s / taper, 0.0), 1.0)
                    edge_weight = t * t * (3.0 - 2.0 * t)
                else:
                    edge_weight = 1.0
                weight = min(max(blend, 0.0), 1.0) * edge_weight
                point.x += weight * (line_x - point.x)
                point.y += weight * (line_y - point.y)

    compute_tangent_yaws(current)
    return current, len(applied)


def sample_by_source_s(points: list[SourcePoint], query_source_s: float) -> SourcePoint:
    if query_source_s <= points[0].source_s:
        return points[0]
    if query_source_s >= points[-1].source_s:
        return points[-1]
    values = [p.source_s for p in points]
    hi = bisect.bisect_left(values, query_source_s)
    lo = max(0, hi - 1)
    a = points[lo]
    b = points[hi]
    u = (query_source_s - a.source_s) / max(b.source_s - a.source_s, 1e-9)
    yaws = unwrap_yaws([a.yaw, b.yaw])
    return SourcePoint(
        query_source_s,
        a.x + u * (b.x - a.x),
        a.y + u * (b.y - a.y),
        normalize_angle(yaws[0] + u * (yaws[1] - yaws[0])),
        a.s + u * (b.s - a.s),
    )


def replay_from_source(points: list[SourcePoint], yaw_weight: float) -> list[ReplayPoint]:
    copied = [SourcePoint(p.source_s, p.x, p.y, p.yaw, p.s) for p in points]
    compute_global_s(copied, yaw_weight)
    return [ReplayPoint(p.s, p.x, p.y, normalize_angle(p.yaw)) for p in copied]


def update_events(events: list[dict[str, str]], processed: list[SourcePoint]) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    for row in events:
        updated = dict(row)
        p = sample_by_source_s(processed, float(row["s"]))
        updated["s"] = f"{p.s:.6f}"
        updated["x"] = f"{p.x:.6f}"
        updated["y"] = f"{p.y:.6f}"
        updated["yaw"] = f"{normalize_angle(p.yaw):.6f}"
        out.append(updated)
    return out


def update_action_sequences(rows: list[dict[str, str]], processed: list[SourcePoint]) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    for row in rows:
        updated = dict(row)
        p = sample_by_source_s(processed, float(row["trigger_s"]))
        updated["trigger_s"] = f"{p.s:.6f}"
        updated["trigger_x"] = f"{p.x:.6f}"
        updated["trigger_y"] = f"{p.y:.6f}"
        updated["trigger_yaw"] = f"{normalize_angle(p.yaw):.6f}"
        out.append(updated)
    return out


def write_debug_svg(
    path: Path,
    original: list[ReplayPoint],
    processed: list[ReplayPoint],
    fit: TemplateFit,
    events: list[dict[str, str]],
) -> None:
    xs = [p.x for p in original] + [p.x for p in processed] + [c[0] for c in fit.centers]
    ys = [p.y for p in original] + [p.y for p in processed] + [c[1] for c in fit.centers]
    pad = max(0.6, fit.radius + 0.25)
    xmin, xmax = min(xs) - pad, max(xs) + pad
    ymin, ymax = min(ys) - pad, max(ys) + pad
    width, height, margin = 1100.0, 820.0, 60.0
    sx = (width - 2 * margin) / max(xmax - xmin, 1e-9)
    sy = (height - 2 * margin) / max(ymax - ymin, 1e-9)

    def to_svg(x: float, y: float) -> tuple[float, float]:
        return margin + (x - xmin) * sx, height - margin - (y - ymin) * sy

    def polyline(points: list[ReplayPoint], color: str, width_px: float, opacity: float) -> str:
        values = " ".join(f"{to_svg(p.x, p.y)[0]:.1f},{to_svg(p.x, p.y)[1]:.1f}" for p in points)
        return f'<polyline points="{values}" fill="none" stroke="{color}" stroke-width="{width_px}" opacity="{opacity}"/>'

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" viewBox="0 0 {width:.0f} {height:.0f}">',
        '<rect width="100%" height="100%" fill="#fbfbfb"/>',
        '<text x="18" y="28" font-size="16" fill="#222">processed replay path debug</text>',
        '<text x="18" y="48" font-size="12" fill="#666">gray=original black=processed blue=template centers/circles</text>',
        polyline(original, "#999", 2.0, 0.55),
    ]
    for idx, (cx, cy) in enumerate(fit.centers, start=1):
        px, py = to_svg(cx, cy)
        rx = fit.radius * sx
        ry = fit.radius * sy
        lines.append(f'<ellipse cx="{px:.1f}" cy="{py:.1f}" rx="{rx:.1f}" ry="{ry:.1f}" fill="none" stroke="#1f77b4" stroke-width="1.5" stroke-dasharray="6 4"/>')
        lines.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="5" fill="#1f77b4"/>')
        lines.append(f'<text x="{px + 7:.1f}" y="{py - 7:.1f}" font-size="12" fill="#1f77b4">C{idx}</text>')
    lines.append(polyline(processed, "#111", 2.4, 1.0))
    for row in events:
        px, py = to_svg(float(row["x"]), float(row["y"]))
        lines.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="4" fill="#d62728"/>')
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_routenew_svg(
    path: Path,
    original: list[ReplayPoint],
    processed: list[ReplayPoint],
    fit: TemplateFit,
    events: list[dict[str, str]],
) -> None:
    xs = [p.x for p in original] + [p.x for p in processed] + [c[0] for c in fit.centers]
    ys = [p.y for p in original] + [p.y for p in processed] + [c[1] for c in fit.centers]
    pad = max(0.7, fit.radius + 0.35)
    xmin, xmax = min(xs) - pad, max(xs) + pad
    ymin, ymax = min(ys) - pad, max(ys) + pad
    width, height, margin = 1200.0, 860.0, 70.0
    span_x = max(xmax - xmin, 1e-9)
    span_y = max(ymax - ymin, 1e-9)

    def to_svg(x: float, y: float) -> tuple[float, float]:
        sx = margin + (x - xmin) / span_x * (width - 2.0 * margin)
        sy = height - margin - (y - ymin) / span_y * (height - 2.0 * margin)
        return sx, sy

    def grid_values(vmin: float, vmax: float, step: float) -> list[float]:
        start = math.floor(vmin / step) * step
        count = int(math.ceil((vmax - start) / step)) + 1
        return [start + idx * step for idx in range(count)]

    def polyline(points: list[ReplayPoint], color: str, width_px: float, opacity: float, dash: str = "") -> str:
        values = " ".join(f"{to_svg(p.x, p.y)[0]:.1f},{to_svg(p.x, p.y)[1]:.1f}" for p in points)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        return f'<polyline points="{values}" fill="none" stroke="{color}" stroke-width="{width_px}" opacity="{opacity}"{dash_attr}/>'

    lines: list[str] = []
    lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" viewBox="0 0 {width:.0f} {height:.0f}">')
    lines.append('<defs><marker id="yaw-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto" markerUnits="strokeWidth"><path d="M0,0 L8,4 L0,8 Z" fill="#17becf"/></marker></defs>')
    lines.append('<rect width="100%" height="100%" fill="#fafafa"/>')
    lines.append('<text x="18" y="28" font-size="17" fill="#222">routenew.svg - processed replay path</text>')
    lines.append(f'<text x="18" y="50" font-size="12" fill="#555">new length={processed[-1].s:.3f}m  original length={original[-1].s:.3f}m  grid=0.2m  yaw arrows=0.3m</text>')

    for gx in grid_values(xmin, xmax, 0.2):
        if gx < xmin or gx > xmax:
            continue
        a = to_svg(gx, ymin)
        b = to_svg(gx, ymax)
        major = abs(gx - round(gx)) < 1e-6
        lines.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{"#dddddd" if major else "#eeeeee"}" stroke-width="{"1.0" if major else "0.6"}"/>')
    for gy in grid_values(ymin, ymax, 0.2):
        if gy < ymin or gy > ymax:
            continue
        a = to_svg(xmin, gy)
        b = to_svg(xmax, gy)
        major = abs(gy - round(gy)) < 1e-6
        lines.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{"#dddddd" if major else "#eeeeee"}" stroke-width="{"1.0" if major else "0.6"}"/>')

    if xmin <= 0.0 <= xmax:
        a = to_svg(0.0, ymin)
        b = to_svg(0.0, ymax)
        lines.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="#666" stroke-width="1.8"/>')
        lines.append(f'<text x="{b[0] + 8:.1f}" y="{b[1] + 15:.1f}" font-size="13" fill="#444">y (m)</text>')
    if ymin <= 0.0 <= ymax:
        a = to_svg(xmin, 0.0)
        b = to_svg(xmax, 0.0)
        lines.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="#666" stroke-width="1.8"/>')
        lines.append(f'<text x="{b[0] - 44:.1f}" y="{b[1] - 8:.1f}" font-size="13" fill="#444">x (m)</text>')

    tick_y = to_svg(0.0, 0.0)[1] if ymin <= 0.0 <= ymax else height - margin + 16.0
    for gx in grid_values(xmin, xmax, 1.0):
        if xmin <= gx <= xmax:
            p = to_svg(gx, 0.0 if ymin <= 0.0 <= ymax else ymin)
            lines.append(f'<text x="{p[0] - 8:.1f}" y="{tick_y + 15:.1f}" font-size="10" fill="#777">{gx:.0f}</text>')
    tick_x = to_svg(0.0, 0.0)[0] if xmin <= 0.0 <= xmax else margin - 34.0
    for gy in grid_values(ymin, ymax, 1.0):
        if ymin <= gy <= ymax and abs(gy) > 1e-6:
            p = to_svg(0.0 if xmin <= 0.0 <= xmax else xmin, gy)
            lines.append(f'<text x="{tick_x + 7:.1f}" y="{p[1] + 3:.1f}" font-size="10" fill="#777">{gy:.0f}</text>')

    lines.append(polyline(original, "#9a9a9a", 2.0, 0.45, "7 5"))
    for idx, (cx, cy) in enumerate(fit.centers, start=1):
        px, py = to_svg(cx, cy)
        rx = fit.radius / span_x * (width - 2.0 * margin)
        ry = fit.radius / span_y * (height - 2.0 * margin)
        lines.append(f'<ellipse cx="{px:.1f}" cy="{py:.1f}" rx="{rx:.1f}" ry="{ry:.1f}" fill="#1f77b4" fill-opacity="0.06" stroke="#1f77b4" stroke-width="1.6"/>')
        lines.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="5" fill="#1f77b4"/>')
        lines.append(f'<text x="{px + 8:.1f}" y="{py - 8:.1f}" font-size="12" fill="#1f77b4">C{idx}</text>')
    c1, c2, c3, c4 = fit.centers
    for idx, (gx, gy) in enumerate([
        ((c1[0] + c2[0]) * 0.5, (c1[1] + c2[1]) * 0.5),
        ((c2[0] + c3[0]) * 0.5, (c2[1] + c3[1]) * 0.5),
        ((c3[0] + c4[0]) * 0.5, (c3[1] + c4[1]) * 0.5),
    ], start=1):
        px, py = to_svg(gx, gy)
        lines.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="5" fill="#9467bd" stroke="#111" stroke-width="0.8"/>')
        lines.append(f'<text x="{px + 7:.1f}" y="{py - 7:.1f}" font-size="11" fill="#5b3f8c">G{idx}</text>')
    lines.append(polyline(processed, "#111", 3.0, 1.0))
    for x0, y0, x1, y1 in yaw_arrow_samples(processed):
        a = to_svg(x0, y0)
        b = to_svg(x1, y1)
        lines.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="#17becf" stroke-width="1.4" marker-end="url(#yaw-arrow)"/>')

    start = to_svg(processed[0].x, processed[0].y)
    end = to_svg(processed[-1].x, processed[-1].y)
    lines.append(f'<circle cx="{start[0]:.1f}" cy="{start[1]:.1f}" r="8" fill="#2ca02c"/>')
    lines.append(f'<text x="{start[0] + 10:.1f}" y="{start[1] - 8:.1f}" font-size="12" fill="#222">START</text>')
    lines.append(f'<circle cx="{end[0]:.1f}" cy="{end[1]:.1f}" r="8" fill="#d62728"/>')
    lines.append(f'<text x="{end[0] + 10:.1f}" y="{end[1] - 8:.1f}" font-size="12" fill="#222">END</text>')

    for row in events:
        p = to_svg(float(row["x"]), float(row["y"]))
        name = row.get("event_name", "")
        s = float(row["s"])
        lines.append(f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="5" fill="#ff7f0e" stroke="#111" stroke-width="0.8"/>')
        lines.append(f'<text x="{p[0] + 7:.1f}" y="{p[1] + 4:.1f}" font-size="10" fill="#222">{svg_escape(name)} s={s:.2f}</text>')

    legend_x, legend_y = width - 240.0, 24.0
    lines.append(f'<rect x="{legend_x:.1f}" y="{legend_y:.1f}" width="214" height="104" fill="#fafafa" stroke="#ddd"/>')
    lines.append(f'<line x1="{legend_x + 14:.1f}" y1="{legend_y + 22:.1f}" x2="{legend_x + 58:.1f}" y2="{legend_y + 22:.1f}" stroke="#111" stroke-width="3"/>')
    lines.append(f'<text x="{legend_x + 68:.1f}" y="{legend_y + 26:.1f}" font-size="12" fill="#333">processed path</text>')
    lines.append(f'<line x1="{legend_x + 14:.1f}" y1="{legend_y + 44:.1f}" x2="{legend_x + 58:.1f}" y2="{legend_y + 44:.1f}" stroke="#9a9a9a" stroke-width="2" opacity="0.55" stroke-dasharray="7 5"/>')
    lines.append(f'<text x="{legend_x + 68:.1f}" y="{legend_y + 48:.1f}" font-size="12" fill="#333">original path</text>')
    lines.append(f'<circle cx="{legend_x + 36:.1f}" cy="{legend_y + 66:.1f}" r="6" fill="#1f77b4" fill-opacity="0.25" stroke="#1f77b4"/>')
    lines.append(f'<text x="{legend_x + 68:.1f}" y="{legend_y + 70:.1f}" font-size="12" fill="#333">fitted circles</text>')
    lines.append(f'<line x1="{legend_x + 22:.1f}" y1="{legend_y + 88:.1f}" x2="{legend_x + 52:.1f}" y2="{legend_y + 88:.1f}" stroke="#17becf" stroke-width="1.6" marker-end="url(#yaw-arrow)"/>')
    lines.append(f'<text x="{legend_x + 68:.1f}" y="{legend_y + 92:.1f}" font-size="12" fill="#333">yaw every 0.3m</text>')

    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_routenew_png(
    path: Path,
    original: list[ReplayPoint],
    processed: list[ReplayPoint],
    fit: TemplateFit,
    events: list[dict[str, str]],
) -> None:
    from PIL import Image, ImageDraw

    xs = [p.x for p in original] + [p.x for p in processed] + [c[0] for c in fit.centers]
    ys = [p.y for p in original] + [p.y for p in processed] + [c[1] for c in fit.centers]
    pad = max(0.7, fit.radius + 0.35)
    xmin, xmax = min(xs) - pad, max(xs) + pad
    ymin, ymax = min(ys) - pad, max(ys) + pad
    width, height, margin = 1400, 1000, 90
    span_x = max(xmax - xmin, 1e-9)
    span_y = max(ymax - ymin, 1e-9)

    def to_px(x: float, y: float) -> tuple[int, int]:
        px = margin + (x - xmin) / span_x * (width - 2 * margin)
        py = height - margin - (y - ymin) / span_y * (height - 2 * margin)
        return round(px), round(py)

    def grid_values(vmin: float, vmax: float, step: float) -> list[float]:
        start = math.floor(vmin / step) * step
        count = int(math.ceil((vmax - start) / step)) + 1
        return [start + idx * step for idx in range(count)]

    def draw_polyline(draw: ImageDraw.ImageDraw, points: list[ReplayPoint], color: tuple[int, int, int], width_px: int) -> None:
        if len(points) < 2:
            return
        draw.line([to_px(p.x, p.y) for p in points], fill=color, width=width_px, joint="curve")

    def draw_yaw_arrow(draw: ImageDraw.ImageDraw, x0: float, y0: float, x1: float, y1: float) -> None:
        sx, sy = to_px(x0, y0)
        ex, ey = to_px(x1, y1)
        draw_px_arrow(draw, sx, sy, ex, ey)

    def draw_px_arrow(draw: ImageDraw.ImageDraw, sx: int, sy: int, ex: int, ey: int) -> None:
        color = (23, 190, 207)
        draw.line([(sx, sy), (ex, ey)], fill=color, width=2)
        angle = math.atan2(ey - sy, ex - sx)
        head_len = 8.0
        head_angle = 0.55
        for sign in (-1.0, 1.0):
            hx = ex - head_len * math.cos(angle + sign * head_angle)
            hy = ey - head_len * math.sin(angle + sign * head_angle)
            draw.line([(ex, ey), (round(hx), round(hy))], fill=color, width=2)

    img = Image.new("RGB", (width, height), (250, 250, 250))
    draw = ImageDraw.Draw(img)

    for gx in grid_values(xmin, xmax, 0.2):
        if gx < xmin or gx > xmax:
            continue
        color = (218, 218, 218) if abs(gx - round(gx)) < 1e-6 else (236, 236, 236)
        x0, y0 = to_px(gx, ymin)
        x1, y1 = to_px(gx, ymax)
        draw.line([(x0, y0), (x1, y1)], fill=color, width=1)
    for gy in grid_values(ymin, ymax, 0.2):
        if gy < ymin or gy > ymax:
            continue
        color = (218, 218, 218) if abs(gy - round(gy)) < 1e-6 else (236, 236, 236)
        x0, y0 = to_px(xmin, gy)
        x1, y1 = to_px(xmax, gy)
        draw.line([(x0, y0), (x1, y1)], fill=color, width=1)

    if xmin <= 0.0 <= xmax:
        draw.line([to_px(0.0, ymin), to_px(0.0, ymax)], fill=(80, 80, 80), width=3)
    if ymin <= 0.0 <= ymax:
        draw.line([to_px(xmin, 0.0), to_px(xmax, 0.0)], fill=(80, 80, 80), width=3)

    draw_polyline(draw, original, (160, 160, 160), 2)
    for cx, cy in fit.centers:
        px, py = to_px(cx, cy)
        rx = fit.radius / span_x * (width - 2 * margin)
        ry = fit.radius / span_y * (height - 2 * margin)
        box = [round(px - rx), round(py - ry), round(px + rx), round(py + ry)]
        draw.ellipse(box, outline=(31, 119, 180), width=3)
        draw.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(31, 119, 180))
    c1, c2, c3, c4 = fit.centers
    for gx, gy in [
        ((c1[0] + c2[0]) * 0.5, (c1[1] + c2[1]) * 0.5),
        ((c2[0] + c3[0]) * 0.5, (c2[1] + c3[1]) * 0.5),
        ((c3[0] + c4[0]) * 0.5, (c3[1] + c4[1]) * 0.5),
    ]:
        px, py = to_px(gx, gy)
        draw.ellipse([px - 7, py - 7, px + 7, py + 7], fill=(148, 103, 189), outline=(20, 20, 20), width=1)
    draw_polyline(draw, processed, (0, 0, 0), 5)
    for arrow in yaw_arrow_samples(processed):
        draw_yaw_arrow(draw, *arrow)

    sx, sy = to_px(processed[0].x, processed[0].y)
    ex, ey = to_px(processed[-1].x, processed[-1].y)
    draw.ellipse([sx - 10, sy - 10, sx + 10, sy + 10], fill=(44, 160, 44))
    draw.ellipse([ex - 10, ey - 10, ex + 10, ey + 10], fill=(214, 39, 40))
    for row in events:
        px, py = to_px(float(row["x"]), float(row["y"]))
        draw.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(255, 127, 14), outline=(20, 20, 20), width=1)

    draw.rectangle([18, 16, 470, 76], fill=(250, 250, 250), outline=(210, 210, 210))
    draw.text((28, 24), "routenew.png - processed replay path", fill=(20, 20, 20))
    draw.text((28, 46), f"black=new  gray=original  cyan=yaw every 0.3m  grid=0.2m  length={processed[-1].s:.3f}m", fill=(70, 70, 70))
    draw.rectangle([width - 260, 18, width - 20, 124], fill=(250, 250, 250), outline=(210, 210, 210))
    draw.line([(width - 238, 40), (width - 190, 40)], fill=(0, 0, 0), width=5)
    draw.text((width - 178, 32), "processed path", fill=(30, 30, 30))
    draw.line([(width - 238, 62), (width - 190, 62)], fill=(160, 160, 160), width=2)
    draw.text((width - 178, 54), "original path", fill=(30, 30, 30))
    draw.ellipse([width - 220, 78, width - 208, 90], outline=(31, 119, 180), width=2)
    draw.text((width - 178, 76), "fitted circles", fill=(30, 30, 30))
    draw.ellipse([width - 224, 96, width - 210, 110], fill=(148, 103, 189), outline=(20, 20, 20))
    draw.text((width - 178, 96), "gap midpoints", fill=(30, 30, 30))
    draw_px_arrow(draw, width - 238, 118, width - 194, 118)
    draw.text((width - 178, 112), "yaw arrows", fill=(30, 30, 30))

    img.save(path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline process record_parku replay path geometry.")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--sample-step", type=float, default=0.02)
    parser.add_argument("--yaw-weight", type=float, default=0.0, help="Yaw contribution when recomputing s; default 0 uses physical x/y path length.")
    parser.add_argument("--yaw-mode", choices=("recorded", "tangent"), default="recorded", help="Final yaw source. recorded keeps recorded pose yaw by source_s; tangent keeps the old path-tangent yaw.")
    parser.add_argument("--template-spacing", type=float, default=1.0)
    parser.add_argument("--template-down-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--template-theta-deg", type=float, default=0.0, help="Auto-fit four-circle row angle. Default 0 means C2=C1+(1,0), C3=C1+(2,0), C4=C3+(0,-1).")
    parser.add_argument("--template-free-rotation", action="store_true", help="Allow the four-circle template to rotate during auto fitting.")
    parser.add_argument("--circle-mode", choices=("off", "smooth", "gap", "repel", "project"), default="smooth", help="How to process the pre-upstairs circle section. smooth keeps the original shape; repel only pushes away from circle centers; gap pulls toward circle-pair midpoints; project is the old aggressive projection.")
    parser.add_argument("--circle-radius", type=float, default=None, help="Override fitted pass radius around obstacle centers.")
    parser.add_argument("--circle-template-x", type=float, default=None, help="Manual C1 x for the four-circle template visualization.")
    parser.add_argument("--circle-template-y", type=float, default=None, help="Manual C1 y for the four-circle template visualization.")
    parser.add_argument("--circle-template-theta-deg", type=float, default=0.0, help="Manual four-circle template row angle in degrees.")
    parser.add_argument("--circle-template-radius", type=float, default=0.55, help="Manual four-circle visualization/pass radius.")
    parser.add_argument("--circle-strength", type=float, default=0.9)
    parser.add_argument("--circle-blend-band", type=float, default=0.28)
    parser.add_argument("--circle-endpoint-guard-s", type=float, default=0.18)
    parser.add_argument("--circle-smooth-window", type=int, default=3)
    parser.add_argument("--circle-smooth-passes", type=int, default=1)
    parser.add_argument("--gap-safe-radius", type=float, default=0.42, help="Minimum distance from fitted circle centers in gap mode.")
    parser.add_argument("--gap-smooth-passes", type=int, default=100)
    parser.add_argument("--gap-smooth-alpha", type=float, default=0.16)
    parser.add_argument("--gap-attract", type=float, default=0.012, help="Soft attraction toward circle-pair midpoints in gap mode.")
    parser.add_argument("--gap-attract-window", type=int, default=70, help="Number of neighboring samples pulled toward each gap midpoint.")
    parser.add_argument("--repel-safe-radius", type=float, default=0.52, help="Minimum distance from fitted circle centers in repel mode.")
    parser.add_argument("--repel-margin", type=float, default=0.12)
    parser.add_argument("--repel-smooth-passes", type=int, default=30)
    parser.add_argument("--repel-smooth-alpha", type=float, default=0.12)
    parser.add_argument("--circle-fit-window-s", type=float, default=1.2)
    parser.add_argument("--circle-fit-step-s", type=float, default=0.10)
    parser.add_argument("--post-smooth-window-s", type=float, default=0.10, help="Final Gaussian smoothing window in source-s meters; 0 disables it.")
    parser.add_argument("--post-smooth-passes", type=int, default=2)
    parser.add_argument("--post-smooth-protect-margin-s", type=float, default=0.05)
    parser.add_argument("--smooth-second-upstairs", action=argparse.BooleanOptionalAction, default=True, help="Apply local smoothing to the second POLICY_UPSTAIR -> POLICY_TROT interval.")
    parser.add_argument("--second-upstairs-smooth-window-s", type=float, default=0.18)
    parser.add_argument("--second-upstairs-smooth-passes", type=int, default=2)
    parser.add_argument("--second-upstairs-smooth-guard-s", type=float, default=0.20)
    parser.add_argument("--second-upstairs-smooth-blend", type=float, default=0.75)
    parser.add_argument("--straighten-long-lines", action=argparse.BooleanOptionalAction, default=True, help="Straighten long near-linear segments after smoothing.")
    parser.add_argument("--straighten-min-length", type=float, default=2.0, help="Minimum chord length in meters for long-line straightening.")
    parser.add_argument("--straighten-max-deviation", type=float, default=0.14, help="Maximum lateral deviation from the segment line in meters.")
    parser.add_argument("--straighten-max-mean-deviation", type=float, default=0.06, help="Maximum mean lateral deviation from the segment line in meters.")
    parser.add_argument("--straighten-max-heading-deg", type=float, default=35.0, help="Maximum local heading deviation from the segment heading.")
    parser.add_argument("--straighten-max-path-ratio", type=float, default=1.08, help="Maximum path/chord length ratio for a near-line segment.")
    parser.add_argument("--straighten-blend", type=float, default=1.0, help="How strongly detected near-lines are pulled to the fitted line.")
    parser.add_argument("--straighten-taper-s", type=float, default=0.30, help="Endpoint taper distance for straightened segments.")
    parser.add_argument("--straighten-protect-margin-s", type=float, default=0.10, help="Source-s margin around upstairs policy segments not straightened.")
    parser.add_argument("--straighten-protect-kneel-creep", action=argparse.BooleanOptionalAction, default=True, help="Do not straighten between POLICY_CREEP and POLICY_KNEEL_CRAWL.")
    parser.add_argument("--l-radius", type=float, default=0.45)
    parser.add_argument("--l-corner", choices=("auto", "start_x_end_y", "end_x_start_y"), default="auto")
    parser.add_argument("--output-prefix", default="processed")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    session = args.session.resolve()
    replay_path = session / "replay_path.csv"
    events_path = session / "events.csv"
    actions_path = session / "action_sequences.csv"

    original = read_replay_path(replay_path)
    events = read_events(events_path)
    actions = read_events(actions_path)
    start_s, end_s = first_upstairs_interval(events)

    pre = [p for p in original if p.s <= start_s]
    candidates = circle_candidates(pre, args.circle_fit_window_s, args.circle_fit_step_s)
    clusters = cluster_candidates(candidates, merge_dist=0.45)
    if args.circle_template_x is not None or args.circle_template_y is not None:
        if args.circle_template_x is None or args.circle_template_y is None:
            raise SystemExit("--circle-template-x and --circle-template-y must be provided together.")
        fit = manual_circle_template(
            args.circle_template_x,
            args.circle_template_y,
            args.circle_template_theta_deg,
            args.template_spacing,
            args.template_down_sign,
            args.circle_radius if args.circle_radius is not None else args.circle_template_radius,
            clusters,
        )
    else:
        fit = fit_circle_template(
            clusters,
            args.template_spacing,
            args.template_down_sign,
            args.circle_radius,
            None if args.template_free_rotation else args.template_theta_deg,
        )

    processed: list[SourcePoint] = []
    if args.circle_mode == "project":
        pre_processed = circularize_pre_upstairs(
            original,
            start_s,
            fit,
            args.circle_strength,
            args.circle_blend_band,
            args.circle_endpoint_guard_s,
            args.circle_smooth_window,
            args.circle_smooth_passes,
        )
    elif args.circle_mode == "smooth":
        pre_processed = copy_pre_upstairs(
            original,
            start_s,
            args.circle_smooth_window,
            args.circle_smooth_passes,
        )
    elif args.circle_mode == "gap":
        pre_processed = gap_smooth_pre_upstairs(
            original,
            start_s,
            fit,
            args.gap_safe_radius,
            args.gap_smooth_passes,
            args.gap_smooth_alpha,
            args.gap_attract,
            args.gap_attract_window,
        )
    elif args.circle_mode == "repel":
        pre_processed = repel_pre_upstairs(
            original,
            start_s,
            fit,
            args.repel_safe_radius,
            args.repel_margin,
            args.repel_smooth_passes,
            args.repel_smooth_alpha,
        )
    else:
        pre_processed = copy_pre_upstairs(original, start_s, 0, 0)
    processed.extend(pre_processed[:-1])

    start = sample_replay_by_s(original, start_s)
    end = sample_replay_by_s(original, end_s)
    old_upstairs = [p for p in original if start_s <= p.s <= end_s]
    processed.extend(l_shape_path(start, end, args.l_radius, args.sample_step, args.l_corner, old_upstairs))

    for p in original:
        if p.s > end_s:
            processed.append(SourcePoint(p.s, p.x, p.y, p.yaw))
    processed = dedupe_source_points(processed)
    processed.sort(key=lambda point: point.source_s)
    protected_ranges = upstairs_intervals(events)
    second_upstairs_ranges = protected_ranges[1:2] if args.smooth_second_upstairs else []
    straighten_protected_ranges = list(protected_ranges)
    kneel_creep_ranges: list[tuple[float, float]] = []
    if args.straighten_protect_kneel_creep:
        kneel_creep_ranges = policy_pair_intervals(events, {"POLICY_CREEP", "POLICY_KNEEL_CRAWL"})
        straighten_protected_ranges.extend(kneel_creep_ranges)
    processed = post_smooth_source_points(
        processed,
        args.post_smooth_window_s,
        args.post_smooth_passes,
        protected_ranges,
        args.post_smooth_protect_margin_s,
    )
    processed = smooth_source_intervals(
        processed,
        second_upstairs_ranges,
        args.second_upstairs_smooth_window_s,
        args.second_upstairs_smooth_passes,
        args.second_upstairs_smooth_guard_s,
        args.second_upstairs_smooth_blend,
    )
    straightened_segments = 0
    if args.straighten_long_lines:
        processed, straightened_segments = straighten_long_near_lines(
            processed,
            args.straighten_min_length,
            args.straighten_max_deviation,
            args.straighten_max_mean_deviation,
            args.straighten_max_heading_deg,
            args.straighten_max_path_ratio,
            args.straighten_blend,
            args.straighten_taper_s,
            straighten_protected_ranges,
            args.straighten_protect_margin_s,
        )

    resampled = resample_processed(processed, args.sample_step)
    if args.yaw_mode == "recorded":
        apply_recorded_yaws(resampled, original)
    final_replay = replay_from_source(resampled, args.yaw_weight)
    final_source = [
        SourcePoint(src.source_s, replay.x, replay.y, replay.yaw, replay.s)
        for src, replay in zip(resampled, final_replay)
    ]

    updated_events = update_events(events, final_source)
    updated_actions = update_action_sequences(actions, final_source)

    replay_out = session / f"replay_path_{args.output_prefix}.csv"
    events_out = session / f"events_{args.output_prefix}.csv"
    actions_out = session / f"action_sequences_{args.output_prefix}.csv"
    route_out = session / f"route_{args.output_prefix}.svg"
    debug_out = session / f"route_{args.output_prefix}_debug.svg"
    routenew_out = session / "routenew.svg"
    routenew_png_out = session / "routenew.png"

    write_replay_path(replay_out, final_replay)
    if updated_events:
        write_rows(events_out, updated_events, event_fieldnames(events))
    if updated_actions:
        write_rows(actions_out, updated_actions, event_fieldnames(actions))
    write_route_svg(route_out, final_replay, updated_events)
    write_routenew_svg(routenew_out, original, final_replay, fit, updated_events)
    write_routenew_png(routenew_png_out, original, final_replay, fit, updated_events)
    write_debug_svg(debug_out, original, final_replay, fit, updated_events)

    print(f"First upstairs interval: s=[{start_s:.3f}, {end_s:.3f}]")
    print(f"Circle mode: {args.circle_mode}")
    print(f"Circle candidates: {len(candidates)}, clusters: {len(clusters)}")
    print(f"Template pass radius: {fit.radius:.3f} m")
    for idx, (cx, cy) in enumerate(fit.centers, start=1):
        print(f"  C{idx}: x={cx:.3f}, y={cy:.3f}")
    print(f"Original length: {original[-1].s:.3f} m")
    print(f"Processed length: {final_replay[-1].s:.3f} m")
    print(f"Yaw mode: {args.yaw_mode}")
    print(f"Smoothed second upstairs intervals: {len(second_upstairs_ranges)}")
    print(f"Straightened long near-lines: {straightened_segments}")
    print(f"Straighten-protected kneel/creep intervals: {len(kneel_creep_ranges)}")
    print(f"Wrote {replay_out}")
    print(f"Wrote {events_out}")
    print(f"Wrote {actions_out}")
    print(f"Wrote {route_out}")
    print(f"Wrote {routenew_out}")
    print(f"Wrote {routenew_png_out}")
    print(f"Wrote {debug_out}")


if __name__ == "__main__":
    main()
