#!/usr/bin/env python3
"""
修剪 AUTO_PARKU / 绕杆轨迹 txt：去掉首尾「停留」段，并重排时间为 0, 0.1, 0.2, ...

输入格式（与 record_trajectory.py 一致）:
  # 注释行
  time_sec x y yaw_rad

用法:
  python3 trim_trajectory.py -i ../commands/task_manager/traj/pole2.txt
  python3 trim_trajectory.py -i pole2.txt -o pole2_trimmed.txt
  python3 trim_trajectory.py -i pole2.txt --in-place
  python3 trim_trajectory.py -i pole2.txt --pos-eps 0.08 --step-eps 0.03
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass
class TrajectoryPoint:
    t: float
    x: float
    y: float
    yaw: float


def parse_trajectory(path: Path) -> tuple[list[str], list[TrajectoryPoint]]:
    """返回 (文件头注释行, 数据点)。"""
    header_lines: list[str] = []
    points: list[TrajectoryPoint] = []

    with path.open(encoding="utf-8") as f:
        for line_no, raw in enumerate(f, start=1):
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                if not points:
                    header_lines.append(raw.rstrip("\n"))
                continue

            parts = line.split()
            if len(parts) < 4:
                raise ValueError(f"{path}:{line_no}: 需要 4 列 time x y yaw，当前: {line!r}")

            t, x, y, yaw = map(float, parts[:4])
            points.append(TrajectoryPoint(t, x, y, yaw))

    if len(points) < 2:
        raise ValueError(f"{path}: 至少需要 2 个有效轨迹点，当前 {len(points)} 个")

    return header_lines, points


def dist_xy(a: TrajectoryPoint, b: TrajectoryPoint) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def find_leading_dwell_end(points: list[TrajectoryPoint], pos_eps: float, step_eps: float) -> int:
    """
    返回首部应保留的第一个下标（此前为停留段，删除）。

    判定：相对起点位移 < pos_eps，且相对上一采样位移 < step_eps，视为仍在停留。
    一旦出现相对起点位移 >= pos_eps，或相对上一点步长 >= step_eps，则从该点起保留。
    """
    n = len(points)
    if n <= 1:
        return 0

    anchor = points[0]
    for i in range(1, n):
        from_anchor = dist_xy(points[i], anchor)
        from_prev = dist_xy(points[i], points[i - 1])
        if from_anchor >= pos_eps or from_prev >= step_eps:
            return i

    return n - 1


def find_trailing_dwell_start(points: list[TrajectoryPoint], pos_eps: float, step_eps: float) -> int:
    """
    返回尾部应保留的最后一个下标（此后为停留段，删除）。

    从末尾锚点向前扫描，逻辑与首部对称。
    """
    n = len(points)
    if n <= 1:
        return 0

    anchor = points[-1]
    for i in range(n - 2, -1, -1):
        from_anchor = dist_xy(points[i], anchor)
        from_next = dist_xy(points[i], points[i + 1])
        if from_anchor >= pos_eps or from_next >= step_eps:
            return i

    return 0


def trim_dwell_segments(
    points: list[TrajectoryPoint],
    pos_eps: float,
    step_eps: float,
    min_trim_points: int,
) -> tuple[int, int]:
    """返回 (start_idx, end_idx) 闭区间，均为保留范围。"""
    start = find_leading_dwell_end(points, pos_eps, step_eps)
    end = find_trailing_dwell_start(points, pos_eps, step_eps)

    if start > end:
        raise ValueError(
            f"修剪后无有效点: start={start}, end={end}（请放宽 --pos-eps / --step-eps）"
        )

    leading_removed = start
    trailing_removed = len(points) - 1 - end

    if leading_removed > 0 and leading_removed < min_trim_points:
        start = 0
        leading_removed = 0
    if trailing_removed > 0 and trailing_removed < min_trim_points:
        end = len(points) - 1
        trailing_removed = 0

    if start > end:
        raise ValueError("修剪后无有效点（min_trim_points 抑制后区间为空）")

    return start, end


def retime_uniform(points: list[TrajectoryPoint], dt: float) -> list[TrajectoryPoint]:
    """时间从 0 开始，步长 dt。"""
    return [
        TrajectoryPoint(i * dt, p.x, p.y, p.yaw)
        for i, p in enumerate(points)
    ]


def write_trajectory(
    path: Path,
    header_lines: list[str],
    points: list[TrajectoryPoint],
    dt: float,
) -> None:
    lines: list[str] = []
    if header_lines:
        lines.extend(header_lines)
    else:
        lines.append("# AUTO_PARKU trajectory: time_sec x y yaw_rad (lidar/map frame)")
        lines.append(f"# Retimed: t=0, dt={dt:.3f}s")

    for p in points:
        lines.append(f"{p.t:.3f} {p.x:.6f} {p.y:.6f} {p.yaw:.6f}")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    default_input = (
        Path(__file__).resolve().parent.parent
        / "commands/task_manager/traj/pole2.txt"
    )

    parser = argparse.ArgumentParser(description="修剪轨迹 txt：去首尾停留并重排时间")
    parser.add_argument(
        "-i",
        "--input",
        type=Path,
        default=default_input,
        help="输入轨迹文件",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="输出文件（默认: <input>_trimmed.txt）",
    )
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="直接覆盖输入文件",
    )
    parser.add_argument(
        "--pos-eps",
        type=float,
        default=0.08,
        help="相对首尾锚点位移小于该值(m)视为停留，默认 0.08",
    )
    parser.add_argument(
        "--step-eps",
        type=float,
        default=0.04,
        help="相邻采样位移小于该值(m)视为未移动，默认 0.04",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.1,
        help="重排后时间步长(s)，默认 0.1",
    )
    parser.add_argument(
        "--min-trim-points",
        type=int,
        default=3,
        help="首尾删除点数少于此值时不删（避免误剪），默认 3",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印统计，不写文件",
    )
    args = parser.parse_args()

    input_path = args.input.resolve()
    if not input_path.is_file():
        print(f"错误: 找不到输入文件 {input_path}", file=sys.stderr)
        return 1

    if args.in_place and args.output is not None:
        print("错误: 不能同时指定 --output 与 --in-place", file=sys.stderr)
        return 1

    output_path = input_path if args.in_place else (
        args.output.resolve()
        if args.output is not None
        else input_path.with_name(input_path.stem + "" + input_path.suffix)
    )

    header_lines, points = parse_trajectory(input_path)
    n_orig = len(points)
    t_orig = points[-1].t - points[0].t

    start, end = trim_dwell_segments(
        points, args.pos_eps, args.step_eps, args.min_trim_points
    )
    trimmed = points[start : end + 1]
    retimed = retime_uniform(trimmed, args.dt)

    print(f"输入: {input_path}")
    print(f"原始点数: {n_orig}, 时间跨度: {t_orig:.3f} s")
    print(f"删除首部: {start} 点, 删除尾部: {n_orig - 1 - end} 点")
    print(f"保留点数: {len(retimed)}, 新时间跨度: {(len(retimed) - 1) * args.dt:.3f} s")
    if retimed:
        print(
            f"保留区间位置: "
            f"({retimed[0].x:.3f}, {retimed[0].y:.3f}) -> ({retimed[-1].x:.3f}, {retimed[-1].y:.3f})"
        )

    if args.dry_run:
        return 0

    write_trajectory(output_path, header_lines, retimed, args.dt)
    print(f"已写入: {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
