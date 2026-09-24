#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
from datetime import datetime
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SESSIONS_ROOT = SCRIPT_DIR / "sessions"


def resolve_session(value: str | Path, sessions_root: Path) -> Path:
    path = Path(value)
    if path.is_absolute() or len(path.parts) > 1:
        return path.resolve()
    return (sessions_root / path).resolve()


def run_cmd(cmd: list[str]) -> None:
    print("+ " + " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def backup_standard_outputs(session: Path, label: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = session / f"backup_before_{label}_{stamp}"
    backup_dir.mkdir(parents=True, exist_ok=True)
    for name in ("replay_path.csv", "events.csv", "action_sequences.csv", "route.svg"):
        src = session / name
        if src.exists():
            shutil.copy2(src, backup_dir / name)
    return backup_dir


def promote_outputs(session: Path, prefix: str, route_name: str | None = None) -> None:
    mapping = [
        (session / f"replay_path_{prefix}.csv", session / "replay_path.csv"),
        (session / f"events_{prefix}.csv", session / "events.csv"),
        (session / f"action_sequences_{prefix}.csv", session / "action_sequences.csv"),
    ]
    for src, dst in mapping:
        if not src.exists():
            raise SystemExit(f"Expected output missing: {src}")
        shutil.copy2(src, dst)

    route_src = session / (route_name or f"route_{prefix}.svg")
    if route_src.exists():
        shutil.copy2(route_src, session / "route.svg")


def extract_events(session: Path, args: argparse.Namespace) -> None:
    run_cmd([
        "python3",
        str(SCRIPT_DIR / "extract_events.py"),
        "--session",
        str(session),
        "--yaw-weight",
        f"{args.extract_yaw_weight}",
    ])


def build_replay_path(session: Path, args: argparse.Namespace) -> None:
    run_cmd([
        "python3",
        str(SCRIPT_DIR / "build_replay_path.py"),
        "--session",
        str(session),
        "--sample-step",
        f"{args.sample_step}",
        "--yaw-weight",
        f"{args.build_yaw_weight}",
        "--min-yaw-step",
        f"{args.min_yaw_step}",
        "--remove-small-loops",
        "--loop-close-xy",
        f"{args.loop_close_xy}",
        "--loop-max-span-s",
        f"{args.loop_max_span_s}",
        "--xy-median-window",
        f"{args.xy_median_window}",
        "--xy-smooth-window",
        f"{args.xy_smooth_window}",
        "--xy-smooth-passes",
        f"{args.xy_smooth_passes}",
    ])


def splice_patch_session(target_session: Path, patch_session: Path, args: argparse.Namespace) -> None:
    cmd = [
        "python3",
        str(SCRIPT_DIR / "splice_replay_path.py"),
        "--path-a",
        str(target_session / "replay_path.csv"),
        "--events",
        str(target_session / "events.csv"),
        "--actions",
        str(target_session / "action_sequences.csv"),
        "--path-b",
        str(patch_session / "replay_path.csv"),
        "--events-b",
        str(patch_session / "events.csv"),
        "--actions-b",
        str(patch_session / "action_sequences.csv"),
        "--output-dir",
        str(target_session),
        "--output-prefix",
        "spliced",
        "--seam-window-s",
        f"{args.seam_window_s}",
        "--seam-smooth-passes",
        f"{args.seam_smooth_passes}",
        "--seam-blend",
        f"{args.seam_blend}",
        "--drop-a-seam-state-radius",
        f"{args.drop_a_seam_state_radius}",
        "--yaw-mode",
        args.yaw_mode,
    ]
    if not args.allow_wrap:
        cmd.append("--no-allow-wrap")
    if not args.allow_reverse_b:
        cmd.append("--no-allow-reverse-b")
    if args.a_search_start_s is not None:
        cmd.extend(["--a-search-start-s", f"{args.a_search_start_s}"])
    if args.a_search_end_s is not None:
        cmd.extend(["--a-search-end-s", f"{args.a_search_end_s}"])
    run_cmd(cmd)
    promote_outputs(target_session, "spliced")


def process_replay_path(session: Path, args: argparse.Namespace) -> None:
    run_cmd([
        "python3",
        str(SCRIPT_DIR / "process_replay_path.py"),
        "--session",
        str(session),
        "--circle-mode",
        args.circle_mode,
        "--repel-safe-radius",
        f"{args.repel_safe_radius}",
        "--repel-margin",
        f"{args.repel_margin}",
        "--repel-smooth-passes",
        f"{args.repel_smooth_passes}",
        "--repel-smooth-alpha",
        f"{args.repel_smooth_alpha}",
        "--post-smooth-window-s",
        f"{args.post_smooth_window_s}",
        "--post-smooth-passes",
        f"{args.post_smooth_passes}",
        "--yaw-mode",
        args.yaw_mode,
        "--output-prefix",
        "processed",
    ])
    promote_outputs(session, "processed", route_name="route_processed.svg")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="One-command record_parku replay preparation with stable output filenames."
    )
    parser.add_argument("session", help="Target session name such as test08, or an absolute session path.")
    parser.add_argument("--sessions-root", type=Path, default=DEFAULT_SESSIONS_ROOT)
    parser.add_argument("--patch-session", default=None, help="Optional session B to splice into target session A.")
    parser.add_argument("--skip-extract", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--process-final", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--yaw-mode", choices=("recorded", "tangent"), default="recorded", help="Final replay yaw source for process/splice stages.")

    parser.add_argument("--extract-yaw-weight", type=float, default=0.25)
    parser.add_argument("--sample-step", type=float, default=0.02)
    parser.add_argument("--build-yaw-weight", type=float, default=0.25)
    parser.add_argument("--min-yaw-step", type=float, default=0.03)
    parser.add_argument("--loop-close-xy", type=float, default=0.08)
    parser.add_argument("--loop-max-span-s", type=float, default=0.35)
    parser.add_argument("--xy-median-window", type=float, default=0.06)
    parser.add_argument("--xy-smooth-window", type=float, default=0.10)
    parser.add_argument("--xy-smooth-passes", type=int, default=1)

    parser.add_argument("--allow-wrap", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--allow-reverse-b", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--a-search-start-s", type=float, default=None)
    parser.add_argument("--a-search-end-s", type=float, default=None)
    parser.add_argument("--seam-window-s", type=float, default=0.35)
    parser.add_argument("--seam-smooth-passes", type=int, default=3)
    parser.add_argument("--seam-blend", type=float, default=0.85)
    parser.add_argument("--drop-a-seam-state-radius", type=float, default=0.30)

    parser.add_argument("--circle-mode", choices=("off", "smooth", "gap", "repel", "project"), default="repel")
    parser.add_argument("--repel-safe-radius", type=float, default=0.55)
    parser.add_argument("--repel-margin", type=float, default=0.10)
    parser.add_argument("--repel-smooth-passes", type=int, default=25)
    parser.add_argument("--repel-smooth-alpha", type=float, default=0.10)
    parser.add_argument("--post-smooth-window-s", type=float, default=0.16)
    parser.add_argument("--post-smooth-passes", type=int, default=3)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sessions_root = args.sessions_root.resolve()
    target_session = resolve_session(args.session, sessions_root)
    patch_session = resolve_session(args.patch_session, sessions_root) if args.patch_session else None

    if not target_session.exists():
        raise SystemExit(f"Target session does not exist: {target_session}")
    if patch_session is not None and not patch_session.exists():
        raise SystemExit(f"Patch session does not exist: {patch_session}")

    backup_dir = backup_standard_outputs(target_session, "prepare")
    print(f"Backed up existing target outputs to {backup_dir}", flush=True)

    if not args.skip_extract:
        extract_events(target_session, args)
        if patch_session is not None:
            extract_events(patch_session, args)
    if not args.skip_build:
        build_replay_path(target_session, args)
        if patch_session is not None:
            build_replay_path(patch_session, args)

    if patch_session is not None:
        splice_patch_session(target_session, patch_session, args)

    if args.process_final:
        process_replay_path(target_session, args)

    print("Final canonical outputs:", flush=True)
    print(f"  {target_session / 'replay_path.csv'}", flush=True)
    print(f"  {target_session / 'events.csv'}", flush=True)
    print(f"  {target_session / 'action_sequences.csv'}", flush=True)
    print(f"  {target_session / 'route.svg'}", flush=True)


if __name__ == "__main__":
    main()
