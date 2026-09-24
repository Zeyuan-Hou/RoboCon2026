#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from common import (
    compute_arc_lengths,
    event_name,
    nearest_pose_by_time,
    read_pose_csv,
    repeat_on_rewind,
    write_pose_csv,
)


DEFAULT_ACTION_EVENT_CODES = [2, 4, 5, 6, 7, 8, 21]
DEFAULT_EXCLUDE_EVENT_CODES = [0, 1, 3, 12, 13, 14, 15, 16, 17, 18, 19, 20]

STATE_RL_MOVE = 5
STATE_FIXED_DOWN = 2

STATE_TRANSITIONS = {
    1: {2: 2},
    2: {2: 3, 5: 7, 6: 8, 7: 6, 8: 9},
    3: {2: 2, 4: 5},
    5: {2: 3, 7: 6},
    6: {2: 2},
    7: {2: 2},
    8: {2: 2},
    9: {2: 2},
}

REQUIRED_PRE_STATE = {
    5: STATE_FIXED_DOWN,
    6: STATE_FIXED_DOWN,
    7: STATE_FIXED_DOWN,
    8: STATE_FIXED_DOWN,
}
ALWAYS_KEEP_EVENTS = {21}

DEFAULT_EVENT_DURATIONS = {
    2: 1.0,
    4: 0.3,
    5: 1.0,
    6: 3.0,
    7: 5.5,
    8: 1.0,
    21: 0.3,
}


def read_cmd_events(path: Path) -> list[tuple[float, int]]:
    if not path.exists():
        return []
    events: list[tuple[float, int]] = []
    with path.open(encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            events.append((float(row["time"]), int(row["event_code"])))
    return events


def write_events_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "event_id",
        "time",
        "s",
        "x",
        "y",
        "yaw",
        "event_code",
        "event_name",
        "source",
        "repeat_on_rewind",
    ]
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_action_sequences_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "sequence_id",
        "trigger_s",
        "trigger_x",
        "trigger_y",
        "trigger_yaw",
        "start_time",
        "end_time",
        "duration_s",
        "steps_json",
        "repeat_on_rewind",
    ]
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def parse_event_codes(value: str) -> set[int]:
    return {int(item.strip()) for item in value.split(",") if item.strip()}


def parse_event_durations(value: str) -> dict[int, float]:
    durations = dict(DEFAULT_EVENT_DURATIONS)
    if not value:
        return durations
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        code_text, duration_text = item.split(":", 1)
        durations[int(code_text.strip())] = float(duration_text.strip())
    return durations


def apply_state_event(state: int, event_code: int) -> int:
    return STATE_TRANSITIONS.get(state, {}).get(event_code, state)


def shortest_event_path(start_state: int, target_state: int, allowed_events: set[int]) -> list[int] | None:
    if start_state == target_state:
        return []
    queue: list[tuple[int, list[int]]] = [(start_state, [])]
    visited = {start_state}
    while queue:
        state, path = queue.pop(0)
        for event_code in sorted(allowed_events):
            next_state = apply_state_event(state, event_code)
            if next_state == state:
                continue
            next_path = path + [event_code]
            if next_state == target_state:
                return next_path
            if next_state not in visited:
                visited.add(next_state)
                queue.append((next_state, next_path))
    return None


def simulate_final_state(event_codes: list[int], start_state: int) -> int:
    state = start_state
    for code in event_codes:
        state = apply_state_event(state, code)
    return state


def compress_action_codes(event_codes: list[int], start_state: int) -> list[int]:
    final_state = simulate_final_state(event_codes, start_state)
    keep_indices = [
        i for i, code in enumerate(event_codes)
        if code in REQUIRED_PRE_STATE or code in ALWAYS_KEEP_EVENTS
    ]
    compressed: list[int] = []
    state = start_state
    cursor = 0

    for index in keep_indices:
        code = event_codes[index]
        if code in REQUIRED_PRE_STATE:
            target_state = REQUIRED_PRE_STATE[code]
            allowed = set(event_codes[cursor:index])
            path = shortest_event_path(state, target_state, allowed)
            if path is None:
                path = event_codes[cursor:index]
            compressed.extend(path)
            for path_code in path:
                state = apply_state_event(state, path_code)

        compressed.append(code)
        state = apply_state_event(state, code)
        cursor = index + 1

    allowed = set(event_codes[cursor:])
    path = shortest_event_path(state, final_state, allowed)
    if path is None:
        path = event_codes[cursor:]
    compressed.extend(path)
    return compressed


def build_steps(event_codes: list[int], event_durations: dict[int, float]) -> tuple[list[dict[str, object]], float]:
    steps: list[dict[str, object]] = []
    elapsed = 0.0
    for code in event_codes:
        steps.append({
            "delay_s": round(elapsed, 6),
            "event_code": code,
            "event_name": event_name(code),
        })
        elapsed += max(0.0, event_durations.get(code, 0.3))
    return steps, elapsed


def build_action_sequences(
    event_rows: list[dict[str, object]],
    action_event_codes: set[int],
    max_dt: float,
    max_xy: float,
    max_ds: float,
    start_state: int,
    event_durations: dict[int, float],
) -> list[dict[str, object]]:
    action_rows = [
        row for row in event_rows
        if int(row["event_code"]) in action_event_codes
    ]
    if not action_rows:
        return []

    groups: list[list[dict[str, object]]] = []
    current: list[dict[str, object]] = [action_rows[0]]
    for row in action_rows[1:]:
        anchor = current[0]
        xy = math.hypot(float(row["x"]) - float(anchor["x"]), float(row["y"]) - float(anchor["y"]))
        if xy <= max_xy:
            current.append(row)
        else:
            groups.append(current)
            current = [row]
    groups.append(current)

    rows: list[dict[str, object]] = []
    for sequence_id, group in enumerate(groups):
        first = group[0]
        last = group[-1]
        start_time = float(first["time"])
        original_codes = [int(row["event_code"]) for row in group]
        compressed_codes = compress_action_codes(original_codes, start_state)
        steps, duration_s = build_steps(compressed_codes, event_durations)
        if not steps:
            continue
        repeat = any(str(row["repeat_on_rewind"]).lower() == "true" for row in group)
        rows.append({
            "sequence_id": len(rows),
            "trigger_s": first["s"],
            "trigger_x": first["x"],
            "trigger_y": first["y"],
            "trigger_yaw": first["yaw"],
            "start_time": first["time"],
            "end_time": last["time"],
            "duration_s": f"{duration_s:.6f}",
            "steps_json": json.dumps(steps, ensure_ascii=False, separators=(",", ":")),
            "repeat_on_rewind": "true" if repeat else "false",
        })
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract clean events and bind them to trajectory arc length")
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--action-output", type=Path, default=None)
    parser.add_argument("--yaw-weight", type=float, default=0.25, help="meters of progress per radian of yaw change; use same value as build_replay_path.py")
    parser.add_argument("--action-event-codes", default=",".join(str(code) for code in DEFAULT_ACTION_EVENT_CODES))
    parser.add_argument(
        "--exclude-event-codes",
        default=",".join(str(code) for code in DEFAULT_EXCLUDE_EVENT_CODES),
        help="comma-separated cmd_evt codes that should not be replayed; default filters global setup/stop/task events",
    )
    parser.add_argument("--action-group-max-dt", type=float, default=2.0)
    parser.add_argument("--action-group-max-xy", type=float, default=0.20)
    parser.add_argument("--action-group-max-ds", type=float, default=0.30)
    parser.add_argument("--action-start-state", type=int, default=STATE_RL_MOVE, help="lower FSM state before each replay action sequence; default RL_MOVE=5")
    parser.add_argument(
        "--action-event-durations",
        default=",".join(f"{code}:{duration}" for code, duration in DEFAULT_EVENT_DURATIONS.items()),
        help="comma-separated event_code:seconds used to rebuild sequence delays after FSM simplification",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    session = args.session.resolve()
    output = args.output.resolve() if args.output else session / "events.csv"
    action_output = args.action_output.resolve() if args.action_output else session / "action_sequences.csv"

    poses = compute_arc_lengths(read_pose_csv(session / "raw_pose.csv"), args.yaw_weight)
    if len(poses) < 2:
        raise SystemExit(f"{session / 'raw_pose.csv'} needs at least two pose samples")

    write_pose_csv(session / "raw_pose_with_s.csv", poses)

    rows: list[dict[str, object]] = []
    excluded_codes = parse_event_codes(args.exclude_event_codes)
    skipped_events = 0
    for t, code in read_cmd_events(session / "raw_cmd_evt.csv"):
        if code in excluded_codes:
            skipped_events += 1
            continue
        pose = nearest_pose_by_time(poses, t)
        name = event_name(code)
        rows.append({
            "event_id": len(rows),
            "time": f"{t:.6f}",
            "s": f"{pose.s:.6f}",
            "x": f"{pose.x:.6f}",
            "y": f"{pose.y:.6f}",
            "yaw": f"{pose.yaw:.6f}",
            "event_code": code,
            "event_name": name,
            "source": "cmd_evt",
            "repeat_on_rewind": "true" if repeat_on_rewind(name) else "false",
        })

    write_events_csv(output, rows)
    action_rows = build_action_sequences(
        rows,
        parse_event_codes(args.action_event_codes),
        args.action_group_max_dt,
        args.action_group_max_xy,
        args.action_group_max_ds,
        args.action_start_state,
        parse_event_durations(args.action_event_durations),
    )
    write_action_sequences_csv(action_output, action_rows)
    print(f"Wrote {len(rows)} events -> {output}")
    if skipped_events:
        print(f"Skipped {skipped_events} excluded raw events: {sorted(excluded_codes)}")
    print(f"Wrote {len(action_rows)} action sequences -> {action_output}")
    print(f"Wrote pose arc lengths -> {session / 'raw_pose_with_s.csv'}")


if __name__ == "__main__":
    main()
