#!/usr/bin/env python3
"""Generate all manip waypoints from four anchors in example_point.txt.

Usage:
  python3 generate_field_waypoints.py --viz

Anchors (example_point.txt):
  - 5th from last = startup first-pickup pose (overrides pickup[startup_pickup_index])
  - 4th from last = A1, 3rd = B4, 2nd = C1, last = C4

Output (overwritten):
  src/commands/lidar_nav_demo_cpp/config/manip_points_generated.yaml
  src/commands/lidar_nav_demo_cpp/config/waypoints_generated.yaml (from waypoints.yaml template)
  src/commands/lidar_nav_demo_cpp/config/trajectories/record_pass.txt
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

from field_waypoint_math import (
    DROPOFF_HIGH_LABELS,
    DROPOFF_LOW_LABELS,
    ENTRY_TRAJECTORY_END_LOCAL,
    ENTRY_TRAJECTORY_START_LOCAL,
    PICKUP_LABELS,
    TrajectoryRecord,
    apply_startup_pickup_override,
    build_entry_trajectory,
    flatten_dropoff,
    flatten_pickup,
    load_anchors_from_file,
    parse_startup_pickup_anchor_from_file,
)

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_ANCHOR_FILE = (
    WORKSPACE
    / "src/commands/lidar_nav_demo_cpp/config/trajectories/example_point.txt"
)
DEFAULT_WAYPOINTS_TEMPLATE = (
    WORKSPACE / "src/commands/lidar_nav_demo_cpp/config/waypoints.yaml"
)
DEFAULT_OUTPUT = (
    WORKSPACE / "src/commands/lidar_nav_demo_cpp/config/manip_points_generated.yaml"
)
DEFAULT_WAYPOINTS_GENERATED = (
    WORKSPACE / "src/commands/lidar_nav_demo_cpp/config/waypoints_generated.yaml"
)
DEFAULT_VIZ = (
    WORKSPACE / "src/commands/lidar_nav_demo_cpp/config/field_waypoints_viz.svg"
)
DEFAULT_ENTRY_TRAJECTORY_OUTPUT = (
    WORKSPACE
    / "src/commands/lidar_nav_demo_cpp/config/trajectories/record_pass.txt"
)

GENERATED_HEADER = (
    "# AUTO-GENERATED from waypoints.yaml + anchor geometry.\n"
    "# Regenerate: python3 src/pythontool/generate_field_waypoints.py --viz\n"
    "# Non-pickup/dropoff params copied from waypoints.yaml; edit that file.\n"
)

PICKUP_SECTION_RE = re.compile(
    r"    # =+ 取货点 pickup_points =+.*?"
    r"(?=    # =+ 放货点 dropoff_points =+)",
    re.DOTALL,
)
DROPOFF_SECTION_RE = re.compile(
    r"    # =+ 放货点 dropoff_points =+.*?"
    r"(?=    # =+ 取货顺序与放货映射 =+)",
    re.DOTALL,
)
STARTUP_PICKUP_ENABLE_RE = re.compile(
    r"^\s*startup_pickup_enable:\s*(true|false)\s*",
    re.MULTILINE | re.IGNORECASE,
)
STARTUP_PICKUP_INDEX_RE = re.compile(
    r"^\s*startup_pickup_index:\s*(-?\d+)\s*",
    re.MULTILINE,
)
HIGH_LAYER_Y_OFFSET_RE = re.compile(
    r"^\s*dropoff_high_layer_y_offset:\s*"
    r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*",
    re.MULTILINE,
)
DEFAULT_HIGH_LAYER_Y_OFFSET = 0.025


def _read_startup_pickup_config(template_path: Path) -> tuple[bool, int]:
    content = template_path.read_text(encoding="utf-8")
    enable_match = STARTUP_PICKUP_ENABLE_RE.search(content)
    index_match = STARTUP_PICKUP_INDEX_RE.search(content)
    enabled = (
        enable_match.group(1).lower() == "true" if enable_match else False
    )
    index = int(index_match.group(1)) if index_match else 1
    return enabled, index


def _read_high_layer_y_offset(template_path: Path) -> float:
    content = template_path.read_text(encoding="utf-8")
    match = HIGH_LAYER_Y_OFFSET_RE.search(content)
    return float(match.group(1)) if match else DEFAULT_HIGH_LAYER_Y_OFFSET


def _apply_startup_pickup_from_anchors(
    result: dict,
    anchor_path: Path,
    template_path: Path,
) -> dict:
    enabled, pickup_index = _read_startup_pickup_config(template_path)
    if not enabled:
        return result

    startup_row = parse_startup_pickup_anchor_from_file(anchor_path)
    if startup_row is None:
        print(
            f"WARNING: startup_pickup_enable=true but {anchor_path.name} "
            f"has no startup pickup anchor; skip startup pickup override"
        )
        return result

    return apply_startup_pickup_override(result, startup_row, pickup_index)


def _fmt_yaml_list(values: list[float]) -> str:
    inner = ", ".join(_fmt_num(v) for v in values)
    return f"[{inner}]"


def _fmt_num(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text else "0"


def _pickup_section(result: dict) -> str:
    pickup = _fmt_yaml_list(flatten_pickup(result["pickup_points"]))
    lines = [
        "    # ================= 取货点 pickup_points =================",
        "    # 格式: [x, y, yaw, is_front, ...]；is_front: 1.0=前排, 0.0=后排",
        "    # AUTO-GENERATED pickup_points — edit example_point.txt anchors",
        "    # 雷达系 x=右、y=从启动区面向赛场；index 0~3=A1~A4, 4~7=B1~B4",
        f"    pickup_points: {pickup}",
    ]
    override_idx = result.get("startup_pickup_override", {}).get("pickup_index")
    for i, (x, y, yaw, is_front) in enumerate(result["pickup_points"]):
        tag = PICKUP_LABELS[i]
        if override_idx is not None and i == override_idx:
            tag += " (startup anchor, 倒数第5行)"
        lines.append(
            f"    #   [{i}] {_fmt_num(x)}, {_fmt_num(y)}, {_fmt_num(yaw)}, "
            f"{_fmt_num(is_front)}  {tag}"
        )
    return "\n".join(lines)


def _dropoff_section(result: dict) -> str:
    dropoff = _fmt_yaml_list(flatten_dropoff(result["dropoff_points"]))
    lines = [
        "    # ================= 放货点 dropoff_points =================",
        "    # 格式: [x, y, yaw]；共 8 点 = 4 坑位 × 2 层",
        "    # AUTO-GENERATED dropoff_points — edit example_point.txt anchors",
        "    # index 0~3: 低层；4~7: 高层",
        "    # 高层相对低层沿地图 +y 方向前进偏移，单位 m",
        f"    dropoff_high_layer_y_offset: {_fmt_num(result['high_layer_y_offset'])}",
        f"    dropoff_points: {dropoff}",
    ]
    for i, (x, y, yaw) in enumerate(result["dropoff_points"]):
        label = DROPOFF_LOW_LABELS[i] if i < 4 else DROPOFF_HIGH_LABELS[i - 4]
        lines.append(
            f"    #   [{i}] {_fmt_num(x)}, {_fmt_num(y)}, {_fmt_num(yaw)}  {label}"
        )
    return "\n".join(lines)


def write_waypoints_generated_yaml(
    result: dict,
    output_path: Path,
    template_path: Path,
) -> None:
    """Copy waypoints.yaml, replace pickup/dropoff with single-line generated arrays."""
    if not template_path.is_file():
        raise FileNotFoundError(f"Template missing: {template_path}")

    content = template_path.read_text(encoding="utf-8")
    content, n_pickup = PICKUP_SECTION_RE.subn(_pickup_section(result) + "\n\n", content, count=1)
    if n_pickup != 1:
        raise RuntimeError("Failed to patch pickup_points in waypoints.yaml template")

    content, n_dropoff = DROPOFF_SECTION_RE.subn(
        _dropoff_section(result) + "\n\n", content, count=1
    )
    if n_dropoff != 1:
        raise RuntimeError("Failed to patch dropoff_points in waypoints.yaml template")

    output_path.write_text(GENERATED_HEADER + content, encoding="utf-8")


def write_generated_yaml(result: dict, output_path: Path) -> None:
    pickup = _fmt_yaml_list(flatten_pickup(result["pickup_points"]))
    dropoff = _fmt_yaml_list(flatten_dropoff(result["dropoff_points"]))
    header = (
        "# AUTO-GENERATED by generate_field_waypoints.py — do not hand-edit.\n"
        "# Regenerate: python3 src/pythontool/generate_field_waypoints.py --viz\n"
        "# Anchors: example_point.txt (labeled multi-anchor preferred; legacy last rows supported)\n"
    )
    body = (
        f"pickup_points: {pickup}\n"
        f"dropoff_points: {dropoff}\n"
        "generated_from:\n"
        f"  model: {result.get('model', 'unknown')}\n"
        f"  a1_map: [{_fmt_num(result['a1_map'][0])}, {_fmt_num(result['a1_map'][1])}]\n"
        f"  b4_map: [{_fmt_num(result['b4_map'][0])}, {_fmt_num(result['b4_map'][1])}]\n"
        f"  c1_map: [{_fmt_num(result['c1_map'][0])}, {_fmt_num(result['c1_map'][1])}]\n"
        f"  c4_low_map: [{_fmt_num(result['c4_low_map'][0])}, {_fmt_num(result['c4_low_map'][1])}]\n"
        f"  theta_ab_rad: {_fmt_num(result['theta_ab_rad'])}\n"
        f"  theta_c_rad: {_fmt_num(result['theta_c_rad'])}\n"
        f"  theta_row_rad: {_fmt_num(result['theta_row_rad'])}\n"
        f"  scale_ab: {_fmt_num(result['scale_ab'])}\n"
        f"  d_c_row: {_fmt_num(result['d_c_row'])}\n"
        f"  row_parallel: true\n"
        f"  forward_yaw: {_fmt_num(result['forward_yaw'])}\n"
        f"  reverse_yaw: {_fmt_num(result['reverse_yaw'])}\n"
        f"  high_layer_y_offset: {_fmt_num(result['high_layer_y_offset'])}\n"
    )
    output_path.write_text(header + body, encoding="utf-8")


def write_entry_trajectory(result: dict, output_path: Path) -> list[TrajectoryRecord]:
    trajectory = build_entry_trajectory(result)
    lines = [
        "# AUTO-GENERATED by generate_field_waypoints.py from navigation-semantic anchors",
        "# Local frame: A1=(0,0), +x along A1->A4, +y along pickup rows",
        (
            "# local_start "
            f"{ENTRY_TRAJECTORY_START_LOCAL[0]:.6f} "
            f"{ENTRY_TRAJECTORY_START_LOCAL[1]:.6f} "
            "local_end "
            f"{ENTRY_TRAJECTORY_END_LOCAL[0]:.6f} "
            f"{ENTRY_TRAJECTORY_END_LOCAL[1]:.6f}"
        ),
        "# AUTO_PARKU trajectory: time_sec x y yaw_rad (lidar/map frame)",
    ]
    for point in trajectory:
        lines.append(f"{point.t:.3f} {point.x:.6f} {point.y:.6f} {point.yaw:.6f}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return trajectory


def print_summary(result: dict) -> None:
    print("=== Field waypoint generation ===")
    print(f"model           : {result.get('model', 'unknown')}")
    print(
        f"theta_ab        : {result['theta_ab_rad']:.4f} rad ({result['theta_ab_deg']:.2f} deg)"
    )
    print(
        f"theta_c         : {result['theta_c_rad']:.4f} rad ({result['theta_c_deg']:.2f} deg)"
    )
    print(
        f"theta_row       : {result['theta_row_rad']:.4f} rad ({result['theta_row_deg']:.2f} deg)"
    )
    print(
        f"theta_ab-c delta: {result['theta_row_delta_rad']:.4f} rad "
        f"({result['theta_row_delta_deg']:.2f} deg) [diagnostic]"
    )
    print(f"scale_ab        : {result['scale_ab']:.4f}")
    print(f"d_c_row         : {result['d_c_row']:.4f} m (C row local dy)")
    print(f"row_parallel    : {result['row_parallel']}")
    print(f"forward_yaw     : {result['forward_yaw']:.4f} rad (dropoff)")
    print(f"reverse_yaw     : {result['reverse_yaw']:.4f} rad (pickup)")

    override = result.get("startup_pickup_override")
    if override is not None:
        idx = override["pickup_index"]
        bx, by = override["before_map"]
        ax, ay = override["anchor_map"]
        print(
            f"startup override: pickup[{idx}] "
            f"({bx:.4f},{by:.4f}) -> anchor ({ax:.4f},{ay:.4f}) "
            f"yaw={override['yaw']:.4f}"
        )

    checks = result["checks"]
    print(f"self-check A1   : err={checks['a1_match_m']:.6f} m")
    print(f"self-check B4   : err={checks['b4_match_m']:.6f} m")
    print(f"self-check C1   : err={checks['c1_match_m']:.6f} m")
    print(f"self-check C4   : err={checks['c4_match_m']:.6f} m")

    labeled_residuals = result.get("labeled_residuals", {})
    if labeled_residuals:
        print("\nLabeled anchor residuals:")
        for label in sorted(labeled_residuals):
            err = labeled_residuals[label]
            warn = "  WARNING" if err > 0.03 else ""
            print(f"  {label:<12} err={err:.4f} m{warn}")

    warn_threshold = 0.03 if labeled_residuals else 0.01
    for name, err in (
        ("B4", checks["b4_match_m"]),
        ("C1", checks["c1_match_m"]),
        ("C4", checks["c4_match_m"]),
    ):
        if err > warn_threshold:
            print(
                f"  WARNING: {name} residual {err:.4f} m "
                f"(parallel-row fit vs anchor)"
            )

    print("\nPickup (index 0..7):")
    for i, (x, y, yaw, is_front) in enumerate(result["pickup_points"]):
        row = "front" if is_front > 0.5 else "rear"
        print(f"  [{i}] ({x:.4f}, {y:.4f}, yaw={yaw:.4f}) {row}")

    print("\nDropoff (index 0..7):")
    for i, (x, y, yaw) in enumerate(result["dropoff_points"]):
        layer = "low" if i < 4 else "high"
        print(f"  [{i}] ({x:.4f}, {y:.4f}, yaw={yaw:.4f}) {layer}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--anchors",
        type=Path,
        default=DEFAULT_ANCHOR_FILE,
        help="Anchor point file (x y yaw per line)",
    )
    parser.add_argument(
        "--template",
        type=Path,
        default=DEFAULT_WAYPOINTS_TEMPLATE,
        help="waypoints.yaml template for waypoints_generated.yaml",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Generated manip points yaml (manip_points_generated.yaml)",
    )
    parser.add_argument(
        "--waypoints-generated",
        type=Path,
        default=DEFAULT_WAYPOINTS_GENERATED,
        help="Full node config yaml (waypoints_generated.yaml)",
    )
    parser.add_argument(
        "--skip-waypoints-generated",
        action="store_true",
        help="Do not update waypoints_generated.yaml",
    )
    parser.add_argument(
        "--high-layer-y-offset",
        type=float,
        default=None,
        help=(
            "High-layer dropoff y offset over low layer; "
            "default reads dropoff_high_layer_y_offset from --template "
            f"or falls back to {DEFAULT_HIGH_LAYER_Y_OFFSET}"
        ),
    )
    parser.add_argument(
        "--forward-yaw",
        type=float,
        default=None,
        help="Override forward yaw (rad); default average of C1/C4",
    )
    parser.add_argument(
        "--reverse-yaw",
        type=float,
        default=None,
        help="Override reverse yaw (rad); default average of A1/B4",
    )
    parser.add_argument(
        "--viz",
        action="store_true",
        help="Also write field_waypoints_viz.svg",
    )
    parser.add_argument(
        "--viz-output",
        type=Path,
        default=DEFAULT_VIZ,
        help="Visualization SVG path",
    )
    parser.add_argument(
        "--entry-trajectory-output",
        type=Path,
        default=DEFAULT_ENTRY_TRAJECTORY_OUTPUT,
        help="Generated logistics entry trajectory path (record_pass.txt)",
    )
    parser.add_argument(
        "--skip-entry-trajectory",
        action="store_true",
        help="Do not update record_pass.txt",
    )
    args = parser.parse_args()

    high_layer_y_offset = (
        args.high_layer_y_offset
        if args.high_layer_y_offset is not None
        else _read_high_layer_y_offset(args.template)
    )

    result = load_anchors_from_file(
        args.anchors,
        high_layer_y_offset=high_layer_y_offset,
        forward_yaw_override=args.forward_yaw,
        reverse_yaw_override=args.reverse_yaw,
    )
    result = _apply_startup_pickup_from_anchors(
        result, args.anchors, args.template
    )
    write_generated_yaml(result, args.output)
    print_summary(result)
    print(f"\nWrote: {args.output}")

    if not args.skip_waypoints_generated:
        write_waypoints_generated_yaml(
            result, args.waypoints_generated, args.template
        )
        print(f"Wrote: {args.waypoints_generated}")

    if not args.skip_entry_trajectory:
        entry_traj = write_entry_trajectory(result, args.entry_trajectory_output)
        first = entry_traj[0]
        last = entry_traj[-1]
        print(
            "entry trajectory local: "
            f"start={ENTRY_TRAJECTORY_START_LOCAL} end={ENTRY_TRAJECTORY_END_LOCAL}"
        )
        print(
            "entry trajectory map  : "
            f"start=({first.x:.4f}, {first.y:.4f}, {first.yaw:.4f}) "
            f"end=({last.x:.4f}, {last.y:.4f}, {last.yaw:.4f})"
        )
        print(f"Wrote: {args.entry_trajectory_output}")

    if args.viz:
        from visualize_field_waypoints import render_field_waypoints

        render_field_waypoints(result, args.viz_output)
        print(f"Wrote: {args.viz_output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
