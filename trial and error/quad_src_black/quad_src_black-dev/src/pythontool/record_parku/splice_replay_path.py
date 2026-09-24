#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

from build_replay_path import read_events, write_replay_path, write_route_svg
from common import ReplayPoint, normalize_angle, unwrap_yaws


@dataclass
class SourcePoint:
    source_s: float
    x: float
    y: float
    yaw: float = 0.0
    s: float = 0.0


@dataclass
class MatchResult:
    start_idx: int
    end_idx: int
    start_dist: float
    end_dist: float
    reversed_b: bool
    wraps_boundary: bool = False
    score: float = 0.0


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


def write_rows(path: Path, rows: list[dict[str, str]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def row_fieldnames(rows: list[dict[str, str]]) -> list[str]:
    return list(rows[0].keys()) if rows else []


def nearest_indices(
    points: list[ReplayPoint],
    target: ReplayPoint,
    count: int,
    min_s: float | None,
    max_s: float | None,
) -> list[tuple[int, float]]:
    candidates: list[tuple[int, float]] = []
    for idx, point in enumerate(points):
        if min_s is not None and point.s < min_s:
            continue
        if max_s is not None and point.s > max_s:
            continue
        dist = math.hypot(point.x - target.x, point.y - target.y)
        candidates.append((idx, dist))
    return sorted(candidates, key=lambda item: item[1])[:max(1, count)]


def match_replacement_segment(
    path_a: list[ReplayPoint],
    path_b: list[ReplayPoint],
    allow_reverse_b: bool,
    allow_wrap: bool,
    candidate_count: int,
    max_replacement_span_ratio: float,
    max_replacement_extra_s: float,
    endpoint_weight: float,
    span_weight: float,
    a_search_start_s: float | None,
    a_search_end_s: float | None,
) -> MatchResult:
    candidates: list[MatchResult] = []
    a_total = max(path_a[-1].s - path_a[0].s, path_a[-1].s, 1e-9)
    b_length = max(path_b[-1].s - path_b[0].s, local_lengths(path_b)[-1], 1e-9)
    max_span = max(
        b_length * max_replacement_span_ratio,
        b_length + max_replacement_extra_s,
    )

    def add_candidates(b_start: ReplayPoint, b_end: ReplayPoint, reversed_b: bool) -> None:
        starts = nearest_indices(path_a, b_start, candidate_count, a_search_start_s, a_search_end_s)
        ends = nearest_indices(path_a, b_end, candidate_count, a_search_start_s, a_search_end_s)
        for start_idx, start_dist in starts:
            for end_idx, end_dist in ends:
                wraps_boundary = False
                if start_idx < end_idx:
                    span = path_a[end_idx].s - path_a[start_idx].s
                elif allow_wrap and start_idx > end_idx:
                    span = (a_total - path_a[start_idx].s) + path_a[end_idx].s
                    wraps_boundary = True
                else:
                    continue
                if span <= 0.0 or span > max_span:
                    continue
                span_error = abs(span - b_length)
                score = endpoint_weight * (start_dist + end_dist) + span_weight * span_error
                candidates.append(MatchResult(
                    start_idx,
                    end_idx,
                    start_dist,
                    end_dist,
                    reversed_b,
                    wraps_boundary,
                    score,
                ))

    add_candidates(path_b[0], path_b[-1], False)
    if allow_reverse_b:
        add_candidates(path_b[-1], path_b[0], True)

    if not candidates:
        raise SystemExit(
            "Could not find a local ordered replacement interval in path A. "
            "Try increasing --max-replacement-span-ratio / --max-replacement-extra-s, "
            "or provide --a-search-start-s and --a-search-end-s around the intended patch area."
        )

    return min(candidates, key=lambda item: item.score)


def local_lengths(points: list[ReplayPoint]) -> list[float]:
    lengths = [0.0]
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += math.hypot(b.x - a.x, b.y - a.y)
        lengths.append(total)
    return lengths


def old_s_to_match_source_s(old_s: float, path_a: list[ReplayPoint], match: MatchResult) -> float:
    if not match.wraps_boundary:
        return old_s

    start_s = path_a[match.start_idx].s
    end_s = path_a[match.end_idx].s
    total_s = path_a[-1].s
    kept_span = start_s - end_s
    if end_s <= old_s <= start_s:
        return old_s - end_s
    if old_s > start_s:
        return kept_span + (old_s - start_s)
    return kept_span + (total_s - start_s) + old_s


def build_spliced_source_path(
    path_a: list[ReplayPoint],
    path_b: list[ReplayPoint],
    match: MatchResult,
) -> list[SourcePoint]:
    b_points = list(reversed(path_b)) if match.reversed_b else list(path_b)
    b_lengths = local_lengths(b_points)
    b_total = max(b_lengths[-1], 1e-9)

    out: list[SourcePoint] = []
    if match.wraps_boundary:
        source_start_s = old_s_to_match_source_s(path_a[match.start_idx].s, path_a, match)
        source_end_s = source_start_s + (path_a[-1].s - path_a[match.start_idx].s) + path_a[match.end_idx].s

        for point in path_a[match.end_idx:match.start_idx]:
            out.append(SourcePoint(old_s_to_match_source_s(point.s, path_a, match), point.x, point.y, point.yaw))
    else:
        source_start_s = path_a[match.start_idx].s
        source_end_s = path_a[match.end_idx].s

        for point in path_a[:match.start_idx]:
            out.append(SourcePoint(point.s, point.x, point.y, point.yaw))

    for point, length in zip(b_points, b_lengths):
        u = length / b_total
        source_s = source_start_s + u * (source_end_s - source_start_s)
        out.append(SourcePoint(source_s, point.x, point.y, point.yaw))

    if not match.wraps_boundary:
        for point in path_a[match.end_idx + 1:]:
            out.append(SourcePoint(point.s, point.x, point.y, point.yaw))

    out.sort(key=lambda point: point.source_s)
    return dedupe_source_points(out)


def replacement_source_bounds(path_a: list[ReplayPoint], match: MatchResult) -> tuple[float, float]:
    if match.wraps_boundary:
        source_start_s = old_s_to_match_source_s(path_a[match.start_idx].s, path_a, match)
        source_end_s = source_start_s + (path_a[-1].s - path_a[match.start_idx].s) + path_a[match.end_idx].s
        return source_start_s, source_end_s
    return path_a[match.start_idx].s, path_a[match.end_idx].s


def old_s_is_replaced(old_s: float, path_a: list[ReplayPoint], match: MatchResult) -> bool:
    start_s = path_a[match.start_idx].s
    end_s = path_a[match.end_idx].s
    if match.wraps_boundary:
        return old_s >= start_s or old_s <= end_s
    return start_s <= old_s <= end_s


def b_s_to_replacement_source_s(
    b_s: float,
    path_a: list[ReplayPoint],
    path_b: list[ReplayPoint],
    match: MatchResult,
) -> float:
    source_start_s, source_end_s = replacement_source_bounds(path_a, match)
    b_total = max(path_b[-1].s - path_b[0].s, local_lengths(path_b)[-1], 1e-9)
    if match.reversed_b:
        progress = (path_b[-1].s - b_s) / b_total
    else:
        progress = (b_s - path_b[0].s) / b_total
    progress = min(max(progress, 0.0), 1.0)
    return source_start_s + progress * (source_end_s - source_start_s)


def dedupe_source_points(points: list[SourcePoint]) -> list[SourcePoint]:
    out: list[SourcePoint] = []
    for point in points:
        if out and abs(point.source_s - out[-1].source_s) <= 1e-9:
            out[-1] = point
        elif out and math.hypot(point.x - out[-1].x, point.y - out[-1].y) <= 1e-8:
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


def smoothstep(value: float) -> float:
    value = min(max(value, 0.0), 1.0)
    return value * value * (3.0 - 2.0 * value)


def smooth_seams(
    points: list[SourcePoint],
    seam_source_s: list[float],
    window_s: float,
    passes: int,
    blend: float,
    yaw_mode: str,
) -> list[SourcePoint]:
    if len(points) < 3 or not seam_source_s or window_s <= 0.0 or passes <= 0 or blend <= 0.0:
        return points

    current = [SourcePoint(p.source_s, p.x, p.y, p.yaw, p.s) for p in points]
    sigma = max(window_s * 0.45, 1e-9)
    blend = min(max(blend, 0.0), 1.0)
    for _ in range(passes):
        out: list[SourcePoint] = []
        for idx, point in enumerate(current):
            if idx == 0 or idx == len(current) - 1:
                out.append(point)
                continue

            seam_dist = min(abs(point.source_s - seam_s) for seam_s in seam_source_s)
            if seam_dist > window_s:
                out.append(point)
                continue

            lo = idx
            while lo > 0 and point.source_s - current[lo - 1].source_s <= window_s:
                lo -= 1
            hi = idx + 1
            while hi < len(current) and current[hi].source_s - point.source_s <= window_s:
                hi += 1

            x_sum = 0.0
            y_sum = 0.0
            weight_sum = 0.0
            for neighbor in current[lo:hi]:
                ds = neighbor.source_s - point.source_s
                weight = math.exp(-0.5 * (ds / sigma) ** 2)
                x_sum += neighbor.x * weight
                y_sum += neighbor.y * weight
                weight_sum += weight
            if weight_sum <= 0.0:
                out.append(point)
                continue

            influence = blend * smoothstep(1.0 - seam_dist / window_s)
            target_x = x_sum / weight_sum
            target_y = y_sum / weight_sum
            out.append(SourcePoint(
                point.source_s,
                point.x + influence * (target_x - point.x),
                point.y + influence * (target_y - point.y),
                point.yaw,
                point.s,
            ))
        current = out
    if yaw_mode == "tangent":
        compute_tangent_yaws(current)
    return current


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


def resample_processed(points: list[SourcePoint], step: float, yaw_weight: float, yaw_mode: str) -> list[SourcePoint]:
    if yaw_mode == "tangent":
        compute_tangent_yaws(points)
    compute_global_s(points, yaw_weight)
    total = points[-1].s
    out: list[SourcePoint] = []
    s = 0.0
    while s < total:
        out.append(sample_source_by_new_s(points, s))
        s += step
    out.append(sample_source_by_new_s(points, total))
    if yaw_mode == "tangent":
        compute_tangent_yaws(out)
    compute_global_s(out, yaw_weight)
    return out


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


def is_state_switch_name(name: str) -> bool:
    return name.startswith("POLICY_") or name in {"ENTER_RL", "KNEEL_CRAWL"}


def action_sequence_has_state_switch(row: dict[str, str]) -> bool:
    try:
        steps = json.loads(row.get("steps_json", "[]"))
    except json.JSONDecodeError:
        return False
    for step in steps:
        if is_state_switch_name(str(step.get("event_name", ""))):
            return True
    return False


def row_xy_distance_to_endpoints(row: dict[str, str], endpoints: list[ReplayPoint], x_key: str, y_key: str) -> float:
    x = float(row[x_key])
    y = float(row[y_key])
    return min(math.hypot(x - endpoint.x, y - endpoint.y) for endpoint in endpoints)


def effective_b_endpoints(path_b: list[ReplayPoint], match: MatchResult) -> list[ReplayPoint]:
    return [path_b[-1], path_b[0]] if match.reversed_b else [path_b[0], path_b[-1]]


def map_a_events(
    events: list[dict[str, str]],
    processed: list[SourcePoint],
    path_a: list[ReplayPoint],
    match: MatchResult,
    seam_state_radius: float,
    b_endpoints: list[ReplayPoint],
) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    for row in events:
        old_s = float(row["s"])
        if old_s_is_replaced(old_s, path_a, match):
            continue
        if (
            seam_state_radius > 0.0
            and is_state_switch_name(row.get("event_name", ""))
            and row_xy_distance_to_endpoints(row, b_endpoints, "x", "y") <= seam_state_radius
        ):
            continue
        updated = dict(row)
        source_s = old_s_to_match_source_s(old_s, path_a, match)
        point = sample_by_source_s(processed, source_s)
        updated["s"] = f"{point.s:.6f}"
        updated["x"] = f"{point.x:.6f}"
        updated["y"] = f"{point.y:.6f}"
        updated["yaw"] = f"{normalize_angle(point.yaw):.6f}"
        out.append(updated)
    return out


def map_b_events(
    events: list[dict[str, str]],
    processed: list[SourcePoint],
    path_a: list[ReplayPoint],
    path_b: list[ReplayPoint],
    match: MatchResult,
) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    for row in events:
        updated = dict(row)
        source_s = b_s_to_replacement_source_s(float(row["s"]), path_a, path_b, match)
        point = sample_by_source_s(processed, source_s)
        updated["s"] = f"{point.s:.6f}"
        updated["x"] = f"{point.x:.6f}"
        updated["y"] = f"{point.y:.6f}"
        updated["yaw"] = f"{normalize_angle(point.yaw):.6f}"
        out.append(updated)
    return out


def map_a_action_sequences(
    rows: list[dict[str, str]],
    processed: list[SourcePoint],
    path_a: list[ReplayPoint],
    match: MatchResult,
    seam_state_radius: float,
    b_endpoints: list[ReplayPoint],
) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    for row in rows:
        old_s = float(row["trigger_s"])
        if old_s_is_replaced(old_s, path_a, match):
            continue
        if (
            seam_state_radius > 0.0
            and action_sequence_has_state_switch(row)
            and row_xy_distance_to_endpoints(row, b_endpoints, "trigger_x", "trigger_y") <= seam_state_radius
        ):
            continue
        updated = dict(row)
        source_s = old_s_to_match_source_s(old_s, path_a, match)
        point = sample_by_source_s(processed, source_s)
        updated["trigger_s"] = f"{point.s:.6f}"
        updated["trigger_x"] = f"{point.x:.6f}"
        updated["trigger_y"] = f"{point.y:.6f}"
        updated["trigger_yaw"] = f"{normalize_angle(point.yaw):.6f}"
        out.append(updated)
    return out


def map_b_action_sequences(
    rows: list[dict[str, str]],
    processed: list[SourcePoint],
    path_a: list[ReplayPoint],
    path_b: list[ReplayPoint],
    match: MatchResult,
) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    for row in rows:
        updated = dict(row)
        source_s = b_s_to_replacement_source_s(float(row["trigger_s"]), path_a, path_b, match)
        point = sample_by_source_s(processed, source_s)
        updated["trigger_s"] = f"{point.s:.6f}"
        updated["trigger_x"] = f"{point.x:.6f}"
        updated["trigger_y"] = f"{point.y:.6f}"
        updated["trigger_yaw"] = f"{normalize_angle(point.yaw):.6f}"
        out.append(updated)
    return out


def merge_sort_events(a_rows: list[dict[str, str]], b_rows: list[dict[str, str]]) -> list[dict[str, str]]:
    rows = sorted(a_rows + b_rows, key=lambda row: (float(row["s"]), float(row.get("time") or 0.0)))
    for idx, row in enumerate(rows):
        if "event_id" in row:
            row["event_id"] = str(idx)
    return rows


def merge_sort_action_sequences(a_rows: list[dict[str, str]], b_rows: list[dict[str, str]]) -> list[dict[str, str]]:
    rows = sorted(a_rows + b_rows, key=lambda row: (float(row["trigger_s"]), float(row.get("start_time") or 0.0)))
    for idx, row in enumerate(rows):
        if "sequence_id" in row:
            row["sequence_id"] = str(idx)
    return rows


def to_replay(points: list[SourcePoint]) -> list[ReplayPoint]:
    return [ReplayPoint(p.s, p.x, p.y, normalize_angle(p.yaw)) for p in points]


def resolve_inputs(args: argparse.Namespace) -> tuple[Path, Path, Path | None, Path | None, Path | None, Path | None, Path]:
    session = args.session.resolve() if args.session else None
    path_a = args.path_a.resolve() if args.path_a else None
    if path_a is None:
        if session is None:
            raise SystemExit("Provide either --session or --path-a.")
        path_a = session / "replay_path.csv"

    path_b = args.path_b.resolve()
    events = args.events.resolve() if args.events else (session / "events.csv" if session else None)
    actions = args.actions.resolve() if args.actions else (session / "action_sequences.csv" if session else None)
    events_b = args.events_b.resolve() if args.events_b else path_b.parent / "events.csv"
    actions_b = args.actions_b.resolve() if args.actions_b else path_b.parent / "action_sequences.csv"
    output_dir = args.output_dir.resolve() if args.output_dir else path_a.parent
    return path_a, path_b, events, actions, events_b, actions_b, output_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replace one segment of replay path A with replay path B.")
    parser.add_argument("--session", type=Path, default=None, help="Session directory; defaults A/events/actions/output paths.")
    parser.add_argument("--path-a", type=Path, default=None, help="Full replay path A. Defaults to SESSION/replay_path.csv.")
    parser.add_argument("--path-b", type=Path, required=True, help="Replacement replay path B.")
    parser.add_argument("--events", type=Path, default=None, help="Events CSV to update. Defaults to SESSION/events.csv.")
    parser.add_argument("--actions", type=Path, default=None, help="Action sequences CSV to update. Defaults to SESSION/action_sequences.csv.")
    parser.add_argument("--events-b", type=Path, default=None, help="Events CSV from replacement path B. Defaults to PATH_B directory/events.csv.")
    parser.add_argument("--actions-b", type=Path, default=None, help="Action sequences CSV from replacement path B. Defaults to PATH_B directory/action_sequences.csv.")
    parser.add_argument("--output-dir", type=Path, default=None, help="Output directory. Defaults to path A directory.")
    parser.add_argument("--output-prefix", default="spliced")
    parser.add_argument("--sample-step", type=float, default=0.02)
    parser.add_argument("--yaw-weight", type=float, default=0.0)
    parser.add_argument("--yaw-mode", choices=("recorded", "tangent"), default="recorded", help="Final yaw source. recorded preserves A/B recorded yaw; tangent uses path-tangent yaw.")
    parser.add_argument("--allow-reverse-b", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--allow-wrap", action=argparse.BooleanOptionalAction, default=True, help="Allow replacing the wrap-around segment across path A end/start.")
    parser.add_argument("--max-endpoint-distance", type=float, default=0.80, help="Warn if a matched endpoint is farther than this in meters.")
    parser.add_argument("--candidate-count", type=int, default=80, help="Number of nearest endpoint candidates to consider on path A.")
    parser.add_argument("--max-replacement-span-ratio", type=float, default=2.0, help="Reject A replacement spans longer than this times path B length.")
    parser.add_argument("--max-replacement-extra-s", type=float, default=2.0, help="Also allow this many extra meters over path B length.")
    parser.add_argument("--endpoint-weight", type=float, default=1.0)
    parser.add_argument("--span-weight", type=float, default=0.35)
    parser.add_argument("--a-search-start-s", type=float, default=None, help="Only search path A from this source s.")
    parser.add_argument("--a-search-end-s", type=float, default=None, help="Only search path A up to this source s.")
    parser.add_argument("--seam-window-s", type=float, default=0.35, help="Source-s smoothing window around each splice seam.")
    parser.add_argument("--seam-smooth-passes", type=int, default=3)
    parser.add_argument("--seam-blend", type=float, default=0.85)
    parser.add_argument("--drop-a-seam-state-radius", type=float, default=0.30, help="Drop A state/policy switches within this XY radius of B endpoints.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    path_a_file, path_b_file, events_file, actions_file, events_b_file, actions_b_file, output_dir = resolve_inputs(args)

    path_a = read_replay_path(path_a_file)
    path_b = read_replay_path(path_b_file)
    match = match_replacement_segment(
        path_a,
        path_b,
        args.allow_reverse_b,
        args.allow_wrap,
        args.candidate_count,
        args.max_replacement_span_ratio,
        args.max_replacement_extra_s,
        args.endpoint_weight,
        args.span_weight,
        args.a_search_start_s,
        args.a_search_end_s,
    )

    if match.wraps_boundary:
        replacement_span_s = (path_a[-1].s - path_a[match.start_idx].s) + path_a[match.end_idx].s
    else:
        replacement_span_s = path_a[match.end_idx].s - path_a[match.start_idx].s
    if replacement_span_s <= 0.0:
        raise SystemExit("Matched replacement span is empty.")

    spliced = build_spliced_source_path(path_a, path_b, match)
    seam_source_s = [
        old_s_to_match_source_s(path_a[match.start_idx].s, path_a, match),
        old_s_to_match_source_s(path_a[match.end_idx].s, path_a, match),
    ]
    spliced = smooth_seams(
        spliced,
        seam_source_s,
        args.seam_window_s,
        args.seam_smooth_passes,
        args.seam_blend,
        args.yaw_mode,
    )
    resampled = resample_processed(spliced, args.sample_step, args.yaw_weight, args.yaw_mode)
    replay_out_points = to_replay(resampled)

    events = read_events(events_file) if events_file and events_file.exists() else []
    actions = read_events(actions_file) if actions_file and actions_file.exists() else []
    events_b = read_events(events_b_file) if events_b_file and events_b_file.exists() else []
    actions_b = read_events(actions_b_file) if actions_b_file and actions_b_file.exists() else []
    b_endpoints = effective_b_endpoints(path_b, match)
    mapped_a_events = map_a_events(events, resampled, path_a, match, args.drop_a_seam_state_radius, b_endpoints) if events else []
    mapped_b_events = map_b_events(events_b, resampled, path_a, path_b, match) if events_b else []
    updated_events = merge_sort_events(mapped_a_events, mapped_b_events)
    mapped_a_actions = map_a_action_sequences(actions, resampled, path_a, match, args.drop_a_seam_state_radius, b_endpoints) if actions else []
    mapped_b_actions = map_b_action_sequences(actions_b, resampled, path_a, path_b, match) if actions_b else []
    updated_actions = merge_sort_action_sequences(mapped_a_actions, mapped_b_actions)

    replay_out = output_dir / f"replay_path_{args.output_prefix}.csv"
    events_out = output_dir / f"events_{args.output_prefix}.csv"
    actions_out = output_dir / f"action_sequences_{args.output_prefix}.csv"
    route_out = output_dir / f"route_{args.output_prefix}.svg"

    write_replay_path(replay_out, replay_out_points)
    if updated_events:
        write_rows(events_out, updated_events, row_fieldnames(events))
    if updated_actions:
        write_rows(actions_out, updated_actions, row_fieldnames(actions))
    write_route_svg(route_out, replay_out_points, updated_events)

    print(f"Path A: {path_a_file}")
    print(f"Path B: {path_b_file}")
    print(f"Matched A indices: start={match.start_idx}, end={match.end_idx}")
    print(f"Matched A source_s: start={path_a[match.start_idx].s:.3f}, end={path_a[match.end_idx].s:.3f}")
    print(f"Endpoint distances: start={match.start_dist:.3f}m, end={match.end_dist:.3f}m")
    if match.start_dist > args.max_endpoint_distance or match.end_dist > args.max_endpoint_distance:
        print(f"WARNING: an endpoint match exceeds {args.max_endpoint_distance:.3f}m.")
    print(f"Path B reversed: {'yes' if match.reversed_b else 'no'}")
    print(f"Wrap-around replacement: {'yes' if match.wraps_boundary else 'no'}")
    print(f"Match score: {match.score:.3f}")
    print(f"Original A length: {path_a[-1].s:.3f}m")
    print(f"Replacement source span: {replacement_span_s:.3f}m")
    print(f"Path B length: {max(path_b[-1].s - path_b[0].s, local_lengths(path_b)[-1]):.3f}m")
    print(f"Spliced length: {replay_out_points[-1].s:.3f}m")
    print(f"Yaw mode: {args.yaw_mode}")
    print(f"Wrote {replay_out}")
    if updated_events:
        print(f"Wrote {events_out}")
    if updated_actions:
        print(f"Wrote {actions_out}")
    print(f"Wrote {route_out}")
    print(f"A events kept: {len(mapped_a_events)}/{len(events)}")
    print(f"B events inserted: {len(mapped_b_events)}")
    print(f"A action sequences kept: {len(mapped_a_actions)}/{len(actions)}")
    print(f"B action sequences inserted: {len(mapped_b_actions)}")


if __name__ == "__main__":
    main()
