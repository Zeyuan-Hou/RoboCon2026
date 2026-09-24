#!/usr/bin/env python3
"""2D obstacle-race path visualizer.

Shows expected obstacle navigation path and actual travelled xy path in one
tkinter window. It is intentionally standalone: no ROS package installation or
launch file is required.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
import time
from pathlib import Path
from typing import Callable, Iterable

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


WORKSPACE = Path("/home/cat/hitcrt_quad_ws")
DEFAULT_CONFIG = WORKSPACE / "src/config/quad_parku.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot expected obstacle path and actual robot xy path."
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="Path to quad_parku.yaml or another obstacle_nav config.",
    )
    parser.add_argument(
        "--replay-path",
        default="",
        help="Override record_parku_replay_path from config.",
    )
    parser.add_argument(
        "--pose-topic",
        default="/quad/lidar_pose_xyyaw",
        help="Live pose topic. Default expects geometry_msgs/Point x/y/z=yaw.",
    )
    parser.add_argument(
        "--pose-type",
        choices=("float_array", "point"),
        default="point",
        help="Pose message type: float_array for [x,y,yaw], point for geometry_msgs/Point.",
    )
    parser.add_argument(
        "--max-actual-points",
        type=int,
        default=20000,
        help="Maximum live path points kept in memory.",
    )
    parser.add_argument(
        "--refresh-sec",
        type=float,
        default=0.1,
        help="Plot refresh interval in seconds.",
    )
    parser.add_argument(
        "--no-equal",
        action="store_true",
        help="Do not force equal x/y aspect ratio.",
    )
    return parser.parse_args()


def load_yaml_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise SystemExit(f"Config file not found: {path}") from exc


def strip_yaml_comments(text: str) -> str:
    lines: list[str] = []
    for line in text.splitlines():
        in_single = False
        in_double = False
        out = []
        for char in line:
            if char == "'" and not in_double:
                in_single = not in_single
            elif char == '"' and not in_single:
                in_double = not in_double
            elif char == "#" and not in_single and not in_double:
                break
            out.append(char)
        lines.append("".join(out))
    return "\n".join(lines)


def parse_scalar(text: str, key: str) -> str | None:
    pattern = re.compile(rf"^\s*{re.escape(key)}\s*:\s*(.+?)\s*$", re.MULTILINE)
    match = pattern.search(text)
    if not match:
        return None
    value = match.group(1).strip()
    if value.startswith(("'", '"')) and value.endswith(("'", '"')):
        value = value[1:-1]
    return value


def parse_array(text: str, key: str) -> list[float]:
    start = text.find(f"{key}:")
    if start < 0:
        return []
    open_idx = text.find("[", start)
    if open_idx < 0:
        return []

    depth = 0
    close_idx = -1
    for idx in range(open_idx, len(text)):
        if text[idx] == "[":
            depth += 1
        elif text[idx] == "]":
            depth -= 1
            if depth == 0:
                close_idx = idx
                break
    if close_idx < 0:
        return []

    body = strip_yaml_comments(text[open_idx + 1 : close_idx])
    values: list[float] = []
    for token in re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", body):
        values.append(float(token))
    return values


def parse_string_list(text: str, key: str) -> list[str]:
    start = text.find(f"{key}:")
    if start < 0:
        return []
    open_idx = text.find("[", start)
    if open_idx < 0:
        return []
    close_idx = text.find("]", open_idx)
    if close_idx < 0:
        return []

    body = strip_yaml_comments(text[open_idx + 1 : close_idx])
    return [
        item.strip().strip("'\"")
        for item in body.split(",")
        if item.strip()
    ]


def load_config(config_path: Path) -> dict[str, object]:
    text = load_yaml_text(config_path)
    mode = parse_scalar(text, "navigation_mode") or "RECORD_PARKU_REPLAY"
    replay_path = parse_scalar(text, "record_parku_replay_path") or ""
    target_points = parse_array(text, "target_points")
    trajectory_files = parse_string_list(text, "trajectory_files")
    return {
        "navigation_mode": mode,
        "record_parku_replay_path": replay_path,
        "target_points": target_points,
        "trajectory_files": trajectory_files,
    }


def resolve_path(path_text: str, base_dir: Path) -> Path:
    path = Path(path_text).expanduser()
    if path.is_absolute():
        return path
    return (base_dir / path).resolve()


def read_csv_path(path: Path) -> tuple[list[float], list[float], list[float]]:
    if not path.exists():
        raise SystemExit(f"Expected path file not found: {path}")

    xs: list[float] = []
    ys: list[float] = []
    yaws: list[float] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        sample = file.read(2048)
        file.seek(0)
        has_header = csv.Sniffer().has_header(sample)
        if has_header:
            reader = csv.DictReader(file)
            for row in reader:
                try:
                    xs.append(float(row["x"]))
                    ys.append(float(row["y"]))
                    yaws.append(float(row.get("yaw", 0.0)))
                except (KeyError, TypeError, ValueError):
                    continue
        else:
            reader = csv.reader(file)
            for row in reader:
                values = [float(item) for item in row if item.strip()]
                if len(values) >= 4:
                    _, x, y, yaw = values[:4]
                elif len(values) >= 3:
                    x, y, yaw = values[:3]
                else:
                    continue
                xs.append(x)
                ys.append(y)
                yaws.append(yaw)
    if not xs:
        raise SystemExit(f"No x/y points loaded from: {path}")
    return xs, ys, yaws


def read_txt_path(path: Path) -> tuple[list[float], list[float], list[float]]:
    if not path.exists():
        raise SystemExit(f"Trajectory file not found: {path}")

    xs: list[float] = []
    ys: list[float] = []
    yaws: list[float] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        values = [float(item) for item in re.split(r"[\s,]+", line) if item]
        if len(values) >= 4:
            _, x, y, yaw = values[:4]
        elif len(values) >= 3:
            x, y, yaw = values[:3]
        else:
            continue
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
    if not xs:
        raise SystemExit(f"No x/y points loaded from: {path}")
    return xs, ys, yaws


def path_from_flat_points(values: Iterable[float]) -> tuple[list[float], list[float], list[float]]:
    vals = list(values)
    xs: list[float] = []
    ys: list[float] = []
    yaws: list[float] = []
    for idx in range(0, len(vals) - 2, 3):
        xs.append(vals[idx])
        ys.append(vals[idx + 1])
        yaws.append(vals[idx + 2])
    if not xs:
        raise SystemExit("navigation_mode is POINT_SEQUENCE but target_points is empty.")
    return xs, ys, yaws


def load_expected_path(
    config_path: Path,
    replay_override: str,
) -> tuple[list[float], list[float], list[float], str]:
    config = load_config(config_path)
    mode = str(config["navigation_mode"])
    base_dir = config_path.parent

    if replay_override:
        replay_path = resolve_path(replay_override, base_dir)
        xs, ys, yaws = read_csv_path(replay_path)
        return xs, ys, yaws, f"override replay: {replay_path}"

    if mode == "RECORD_PARKU_REPLAY":
        replay_value = str(config["record_parku_replay_path"])
        if not replay_value:
            raise SystemExit("record_parku_replay_path is empty in config.")
        replay_path = resolve_path(replay_value, base_dir)
        xs, ys, yaws = read_csv_path(replay_path)
        return xs, ys, yaws, f"RECORD_PARKU_REPLAY: {replay_path}"

    if mode == "POINT_SEQUENCE":
        xs, ys, yaws = path_from_flat_points(config["target_points"])
        return xs, ys, yaws, "POINT_SEQUENCE: target_points"

    if mode == "TRAJECTORY_SEQUENCE":
        files = [resolve_path(item, base_dir) for item in config["trajectory_files"]]
        if not files:
            raise SystemExit("TRAJECTORY_SEQUENCE selected but trajectory_files is empty.")
        all_x: list[float] = []
        all_y: list[float] = []
        all_yaw: list[float] = []
        for file_path in files:
            xs, ys, yaws = read_txt_path(file_path)
            all_x.extend(xs)
            all_y.extend(ys)
            all_yaw.extend(yaws)
        return all_x, all_y, all_yaw, "TRAJECTORY_SEQUENCE: " + ", ".join(map(str, files))

    raise SystemExit(f"Unsupported navigation_mode: {mode}")


class LivePoseNode(Node):
    def __init__(self, topic: str, pose_type: str, max_points: int) -> None:
        super().__init__("obstacle_nav_xy_plot")
        self.actual_x: list[float] = []
        self.actual_y: list[float] = []
        self.current_yaw = 0.0
        self.max_points = max(10, max_points)
        self.received = False

        if pose_type == "point":
            self.subscription = self.create_subscription(Point, topic, self._point_callback, 10)
        else:
            self.subscription = self.create_subscription(
                Float32MultiArray, topic, self._array_callback, 10
            )

    def _append_pose(self, x: float, y: float, yaw: float) -> None:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            return
        self.actual_x.append(x)
        self.actual_y.append(y)
        if len(self.actual_x) > self.max_points:
            overflow = len(self.actual_x) - self.max_points
            del self.actual_x[:overflow]
            del self.actual_y[:overflow]
        self.current_yaw = yaw
        self.received = True

    def _array_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        yaw = float(msg.data[2]) if len(msg.data) >= 3 else self.current_yaw
        self._append_pose(float(msg.data[0]), float(msg.data[1]), yaw)

    def _point_callback(self, msg: Point) -> None:
        self._append_pose(float(msg.x), float(msg.y), float(msg.z))


def compute_bounds(
    expected_x: list[float],
    expected_y: list[float],
    actual_x: list[float],
    actual_y: list[float],
) -> tuple[float, float, float, float]:
    xs = expected_x + actual_x
    ys = expected_y + actual_y
    if not xs or not ys:
        return -1.0, 1.0, -1.0, 1.0
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    pad = max(0.5, 0.08 * max(xmax - xmin, ymax - ymin, 1.0))
    return xmin - pad, xmax + pad, ymin - pad, ymax + pad


def downsample_points(
    xs: list[float],
    ys: list[float],
    max_points: int = 5000,
) -> tuple[list[float], list[float]]:
    if len(xs) <= max_points:
        return xs, ys
    step = max(1, math.ceil(len(xs) / max_points))
    return xs[::step], ys[::step]


class TkPathPlotter:
    def __init__(
        self,
        tk,
        expected_x: list[float],
        expected_y: list[float],
        source: str,
        pose_topic: str,
        equal_aspect: bool,
    ) -> None:
        self.tk = tk
        self.expected_x = expected_x
        self.expected_y = expected_y
        self.source = source
        self.pose_topic = pose_topic
        self.equal_aspect = equal_aspect
        self.closed = False

        self.root = tk.Tk()
        self.root.title("Obstacle Nav XY Path")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.canvas = tk.Canvas(self.root, width=1000, height=760, bg="white")
        self.canvas.pack(fill="both", expand=True)

    def close(self) -> None:
        self.closed = True
        try:
            self.root.destroy()
        except self.tk.TclError:
            pass

    def _transform(
        self,
        xmin: float,
        xmax: float,
        ymin: float,
        ymax: float,
        width: int,
        height: int,
    ) -> Callable[[float, float], tuple[float, float]]:
        margin = 55.0
        plot_w = max(1.0, width - 2.0 * margin)
        plot_h = max(1.0, height - 2.0 * margin)
        span_x = max(1e-6, xmax - xmin)
        span_y = max(1e-6, ymax - ymin)

        if self.equal_aspect:
            scale = min(plot_w / span_x, plot_h / span_y)
            used_w = span_x * scale
            used_h = span_y * scale
            offset_x = margin + (plot_w - used_w) / 2.0
            offset_y = margin + (plot_h - used_h) / 2.0

            def to_px(x: float, y: float) -> tuple[float, float]:
                px = offset_x + (x - xmin) * scale
                py = height - offset_y - (y - ymin) * scale
                return px, py

            return to_px

        scale_x = plot_w / span_x
        scale_y = plot_h / span_y

        def to_px(x: float, y: float) -> tuple[float, float]:
            px = margin + (x - xmin) * scale_x
            py = height - margin - (y - ymin) * scale_y
            return px, py

        return to_px

    def _draw_polyline(
        self,
        xs: list[float],
        ys: list[float],
        to_px: Callable[[float, float], tuple[float, float]],
        color: str,
        width: int,
    ) -> None:
        if len(xs) < 2:
            return
        draw_x, draw_y = downsample_points(xs, ys)
        coords: list[float] = []
        for x, y in zip(draw_x, draw_y):
            px, py = to_px(x, y)
            coords.extend([px, py])
        self.canvas.create_line(*coords, fill=color, width=width, smooth=False)

    def update(
        self,
        actual_x: list[float],
        actual_y: list[float],
        yaw: float,
        received: bool,
    ) -> None:
        width = max(300, self.canvas.winfo_width())
        height = max(260, self.canvas.winfo_height())
        xmin, xmax, ymin, ymax = compute_bounds(
            self.expected_x, self.expected_y, actual_x, actual_y
        )
        to_px = self._transform(xmin, xmax, ymin, ymax, width, height)

        self.canvas.delete("all")
        self.canvas.create_text(
            16,
            12,
            anchor="nw",
            text=(
                f"{self.source}\n"
                f"pose: {self.pose_topic}\n"
                f"x range: [{xmin:.2f}, {xmax:.2f}]  y range: [{ymin:.2f}, {ymax:.2f}]"
            ),
            fill="#111827",
            font=("TkDefaultFont", 10),
        )
        self.canvas.create_text(18, height - 38, anchor="sw", text="blue: expected", fill="#2563eb")
        self.canvas.create_text(18, height - 22, anchor="sw", text="green: actual", fill="#16a34a")
        self.canvas.create_text(18, height - 6, anchor="sw", text="red: current pose", fill="#dc2626")

        self._draw_polyline(self.expected_x, self.expected_y, to_px, "#2563eb", 2)
        self._draw_polyline(actual_x, actual_y, to_px, "#16a34a", 2)

        if actual_x and actual_y:
            cx = actual_x[-1]
            cy = actual_y[-1]
            px, py = to_px(cx, cy)
            hx, hy = to_px(cx + math.cos(yaw) * 0.35, cy + math.sin(yaw) * 0.35)
            self.canvas.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#dc2626", outline="")
            self.canvas.create_line(px, py, hx, hy, fill="#dc2626", width=3, arrow="last")
            self.canvas.create_text(
                width - 18,
                16,
                anchor="ne",
                text=f"x={cx:.3f}\ny={cy:.3f}\nyaw={yaw:.3f}\nactual points={len(actual_x)}",
                fill="#111827",
                font=("TkDefaultFont", 10),
            )
        elif not received:
            self.canvas.create_text(
                width / 2,
                height / 2,
                text="waiting for live pose...",
                fill="#6b7280",
                font=("TkDefaultFont", 18),
            )

        self.root.update_idletasks()
        self.root.update()


def main() -> int:
    args = parse_args()
    config_path = Path(args.config).expanduser()
    expected_x, expected_y, expected_yaw, source = load_expected_path(
        config_path, args.replay_path
    )

    try:
        import tkinter as tk
    except ImportError as exc:
        raise SystemExit(
            "tkinter is not available. Install python3-tk or run on a desktop environment."
        ) from exc

    rclpy.init()
    node = LivePoseNode(args.pose_topic, args.pose_type, args.max_actual_points)
    plotter = TkPathPlotter(
        tk,
        expected_x,
        expected_y,
        source,
        args.pose_topic,
        equal_aspect=not args.no_equal,
    )

    print(f"[info] loaded expected path: {source}")
    print(f"[info] expected points: {len(expected_x)}")
    print(f"[info] listening pose topic: {args.pose_topic} ({args.pose_type})")
    print("[info] close the plot window or press Ctrl+C to exit")

    try:
        while rclpy.ok() and not plotter.closed:
            rclpy.spin_once(node, timeout_sec=0.01)
            plotter.update(node.actual_x, node.actual_y, node.current_yaw, node.received)
            time.sleep(max(0.02, args.refresh_sec))
    except KeyboardInterrupt:
        pass
    except tk.TclError:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
