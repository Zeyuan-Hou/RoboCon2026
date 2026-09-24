#!/usr/bin/env python3
"""Inject pickup/dropoff slot overlays into existing segment_*.svg files."""

from __future__ import annotations

import re
import sys
from pathlib import Path

# Match waypoints.yaml defaults (update if config changes)
PICKUP_POINTS = [
    (1.45, 2.40, 3.08, 1.0),
    (0.59, 2.44, 3.09, 1.0),
    (-0.32, 2.51, 3.08, 1.0),
    (-1.17, 2.57, 3.06, 1.0),
    (-1.11, 3.42, 3.09, 0.0),
    (-0.25, 3.375, 3.06, 0.0),
    (0.685, 3.326, 3.07, 0.0),
    (1.515, 3.255, 3.05, 0.0),
]
DROPOFF_POINTS = [
    (-0.85, 4.74, -0.05),
    (-0.08, 4.68, -0.072),
    (0.73, 4.60, -0.05),
    (1.54, 4.54, -0.05),
    (-0.85, 4.76, -0.07),
    (-0.085, 4.70, -0.062),
    (0.72, 4.64, -0.065),
    (1.54, 4.61, -0.05),
]
EIGHTBOXES_MAP = [4, 5, 6, 7, 3, 2, 1, 0]
DROPOFF_COLOR_ORDER = [0, 1, 2, 3]
DROPOFF_LAYER_STRIDE = 4
COLORS = {0: "#2ca02c", 1: "#888888", 2: "#1f77b4", 3: "#d62728"}
COLOR_LABEL = {0: "G", 1: "S", 2: "B", 3: "R"}
COLOR_NAME = {0: "green", 1: "gray", 2: "blue", 3: "red"}

X_MIN, X_MAX, Y_MIN, Y_MAX = -5.0, 5.0, 0.0, 9.0
WIDTH, HEIGHT, MARGIN = 800, 720, 40


def to_svg(x: float, y: float) -> tuple[float, float]:
    x_span = max(X_MAX - X_MIN, 1e-6)
    y_span = max(Y_MAX - Y_MIN, 1e-6)
    sx = MARGIN + (x - X_MIN) / x_span * (WIDTH - 2 * MARGIN)
    sy = HEIGHT - MARGIN - (y - Y_MIN) / y_span * (HEIGHT - 2 * MARGIN)
    return sx, sy


def map_square(cx: float, cy: float, side: float, fill: str, stroke: str, sw: float) -> str:
    half = side * 0.5
    nw = to_svg(cx - half, cy + half)
    se = to_svg(cx + half, cy - half)
    x = min(nw[0], se[0])
    y = min(nw[1], se[1])
    w = abs(se[0] - nw[0])
    h = abs(se[1] - nw[1])
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>\n'
    )


def build_tasks(classes: list[int], ocr_class: int | None) -> list[int]:
    order = list(range(8))
    if ocr_class is not None:
        order = [i for i in order if classes[i] == ocr_class]
        order += [i for i in range(8) if classes[i] != ocr_class]

    def is_front(slot: int) -> bool:
        phys = EIGHTBOXES_MAP[slot]
        return PICKUP_POINTS[phys][3] >= 0.5

    order.sort(key=is_front)
    return [s for s in order if 0 <= classes[s] <= 3]


def overlay_svg(classes: list[int], ocr_class: int | None) -> str:
    tasks = build_tasks(classes, ocr_class)
    priority = tasks[0] if tasks else None
    out: list[str] = []

    for drop_idx, (x, y, _yaw) in enumerate(DROPOFF_POINTS):
        layer = 1 if drop_idx >= DROPOFF_LAYER_STRIDE else 0
        slot = drop_idx % DROPOFF_LAYER_STRIDE
        cargo = DROPOFF_COLOR_ORDER[slot]
        out.append(map_square(x, y, 0.22, COLORS[cargo], "#444444", 1.0))
        sx, sy = to_svg(x, y)
        l2 = "L2" if layer else ""
        out.append(
            f'<text x="{sx:.1f}" y="{sy + 4:.1f}" font-size="11" fill="#111" '
            f'text-anchor="middle" font-weight="bold">D{drop_idx}{COLOR_LABEL[cargo]}{l2}</text>\n'
        )

    for string_idx in range(8):
        cargo = classes[string_idx]
        if cargo < 0 or cargo > 3:
            continue
        phys = EIGHTBOXES_MAP[string_idx]
        x, y = PICKUP_POINTS[phys][0], PICKUP_POINTS[phys][1]
        if string_idx == priority:
            out.append(map_square(x, y, 0.32, "none", "#ffd700", 3.5))
        out.append(map_square(x, y, 0.25, COLORS[cargo], "#222222", 1.2))
        sx, sy = to_svg(x, y)
        out.append(
            f'<text x="{sx:.1f}" y="{sy + 4:.1f}" font-size="12" fill="#111" '
            f'text-anchor="middle" font-weight="bold">{string_idx}</text>\n'
        )

    task_summary = ", ".join(
        f"T{i}:{s}({COLOR_NAME[classes[s]]})" for i, s in enumerate(tasks[:8])
    )
    out.append(
        '<rect x="40" y="30" width="460" height="36" fill="#ffffffcc" stroke="#999" rx="4"/>\n'
    )
    out.append(
        f'<text x="48" y="52" font-size="11" fill="#333">'
        f'eightboxes={"".join(map(str, classes))}  visit: {task_summary}</text>\n'
    )
    out.append(
        '<rect x="520" y="8" width="270" height="92" fill="#ffffffcc" stroke="#999" rx="4"/>\n'
        '<text x="528" y="24" font-size="11" fill="#333">Pickup: eightboxes slot / color</text>\n'
        '<text x="528" y="40" font-size="11" fill="#333">Dropoff: Dindex + G/S/B/R (+L2)</text>\n'
        '<rect x="528" y="68" width="14" height="14" fill="none" stroke="#ffd700" stroke-width="3"/>\n'
        '<text x="548" y="80" font-size="10" fill="#333">priority 1st pickup (yellow)</text>\n'
    )
    return "".join(out)


def patch_file(path: Path, overlay: str) -> None:
    text = path.read_text(encoding="utf-8")
    text = re.sub(r"<!-- SLOT_OVERLAY_START -->.*?<!-- SLOT_OVERLAY_END -->\n?", "", text, flags=re.S)
    text = text.replace("</svg>", f"<!-- SLOT_OVERLAY_START -->\n{overlay}<!-- SLOT_OVERLAY_END -->\n</svg>")
    path.write_text(text, encoding="utf-8")


def main() -> int:
    classes = [0, 0, 1, 1, 0, 0, 0, 0]
    ocr_class = 0
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if len(arg) == 8 and arg.isdigit():
            classes = [int(c) for c in arg]
        else:
            classes = [int(c) for c in sys.argv[1:9]]
    if len(sys.argv) > 2 and len(sys.argv[1]) != 8:
        ocr_class = int(sys.argv[-1])

    viz_dir = Path("/home/cat/hitcrt_quad2026_ws/maps/lidar_nav_path_viz")
    overlay = overlay_svg(classes, ocr_class)
    for svg in sorted(viz_dir.glob("segment_*.svg")):
        patch_file(svg, overlay)
        print(f"patched {svg.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
