#!/usr/bin/env python3
"""
平滑轨迹 txt：裁剪停留段、按弧长等距重采样、局部平滑 x/y，并重新生成 yaw。

输入/输出格式与 record_trajectory.py 一致：
  time_sec x y yaw_rad

默认不依赖 numpy/scipy，适合现场机器直接使用。

处理流程：
  1. 读取 time x y yaw 轨迹文件。
  2. 可选 --trim-dwell 裁剪首尾停留段。
  3. 按路径弧长用 --spacing 等距重采样。
  4. 用 --window / --passes 对 x/y 做局部滑动平均。
  5. 按 --yaw-mode 生成中间段 yaw。
  6. 起点/终点 yaw 精确保留有效输入轨迹原值，首尾约 0.3m 内平滑过渡。
  7. 按 --dt 重排输出时间。

最简单启动：
python3 src/pythontool/smooth_trajectory.py \
    -i src/commands/task_manager/traj/slope1.txt \
    --yaw-mode smooth

指定输出和常用参数：
  python3 src/pythontool/smooth_trajectory.py \
    -i src/commands/task_manager/traj/pole2.txt \
    -o src/commands/task_manager/traj/pole2_smooth.txt \
    --spacing 0.05 --window 5 --dt 0.1 --yaw-mode tangent

带首尾停留段裁剪：
  python3 src/pythontool/smooth_trajectory.py \
    -i src/commands/task_manager/traj/start1.txt \
    -o src/commands/task_manager/traj/start1_smooth.txt \
    --trim-dwell

yaw 模式说明：
  --yaw-mode tangent
    根据平滑后的 x/y 路径切线重新计算雷达 yaw。
    优点是几何上干净，适合录制 yaw 抖动明显的轨迹。
    缺点是路径局部形状变化大时，yaw 也可能跟着变化。

  --yaw-mode smooth
    对原始 yaw 先 unwrap，再做滑动平均，最后 wrap 回 [-pi, pi]。
    优点是更尊重录制时的机体朝向，通常首尾和倒车轨迹更柔顺。
    如果 tangent 模式 yaw 波动较大，优先试这个模式。

  --yaw-mode keep
    保留重采样插值得到的 yaw，不额外平滑中间段。
    适合原始 yaw 已经很稳定，只想优化 x/y 点距和平滑度的情况。

常用调参：
  --spacing 0.05
    输出点间距，越小点越密；一般 0.05m 足够。

  --window 5
    x/y 平滑窗口，必须是奇数。轨迹锯齿大可试 7 或 9，
    但窗口太大会让弯道切角。

  --passes 1
    平滑次数。一般保持 1；需要更平滑可试 2。

  --dry-run
    只打印统计，不写文件。正式覆盖或生成新文件前建议先跑。
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
    s: float = 0.0


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def parse_trajectory(path: Path) -> tuple[list[str], list[TrajectoryPoint]]:
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

    for i in range(1, len(points)):
        if points[i].t <= points[i - 1].t:
            raise ValueError(f"{path}: time 必须严格递增，index={i}")

    return header_lines, points


def compute_arc_lengths(points: list[TrajectoryPoint]) -> float:
    total = 0.0
    points[0].s = 0.0
    for i in range(1, len(points)):
        total += math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y)
        points[i].s = total
    if total <= 1e-6:
        raise ValueError("轨迹长度过小，无法重采样")
    return total


def dist_xy(a: TrajectoryPoint, b: TrajectoryPoint) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def find_leading_dwell_end(points: list[TrajectoryPoint], pos_eps: float, step_eps: float) -> int:
    anchor = points[0]
    for i in range(1, len(points)):
        from_anchor = dist_xy(points[i], anchor)
        from_prev = dist_xy(points[i], points[i - 1])
        if from_anchor >= pos_eps or from_prev >= step_eps:
            return i
    return len(points) - 1


def find_trailing_dwell_start(points: list[TrajectoryPoint], pos_eps: float, step_eps: float) -> int:
    anchor = points[-1]
    for i in range(len(points) - 2, -1, -1):
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
) -> tuple[list[TrajectoryPoint], int, int]:
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

    return points[start : end + 1], leading_removed, trailing_removed


def sample_at_s(points: list[TrajectoryPoint], s: float) -> TrajectoryPoint:
    if s <= points[0].s:
        p = points[0]
        return TrajectoryPoint(0.0, p.x, p.y, p.yaw, s)
    if s >= points[-1].s:
        p = points[-1]
        return TrajectoryPoint(0.0, p.x, p.y, p.yaw, s)

    for a, b in zip(points, points[1:]):
        if a.s <= s <= b.s:
            ds = b.s - a.s
            alpha = (s - a.s) / ds if ds > 1e-9 else 0.0
            dyaw = normalize_angle(b.yaw - a.yaw)
            return TrajectoryPoint(
                t=0.0,
                x=a.x + alpha * (b.x - a.x),
                y=a.y + alpha * (b.y - a.y),
                yaw=normalize_angle(a.yaw + alpha * dyaw),
                s=s,
            )

    p = points[-1]
    return TrajectoryPoint(0.0, p.x, p.y, p.yaw, s)


def resample_by_spacing(points: list[TrajectoryPoint], spacing: float) -> list[TrajectoryPoint]:
    total_s = compute_arc_lengths(points)
    samples: list[TrajectoryPoint] = []
    s = 0.0
    while s < total_s:
        samples.append(sample_at_s(points, s))
        s += spacing
    if not samples or abs(samples[-1].s - total_s) > 1e-6:
        samples.append(sample_at_s(points, total_s))
    return samples


def moving_average(values: list[float], window: int) -> list[float]:
    if window <= 1 or len(values) <= 2:
        return values[:]
    half = window // 2
    out = values[:]
    for i in range(1, len(values) - 1):
        lo = max(0, i - half)
        hi = min(len(values), i + half + 1)
        out[i] = sum(values[lo:hi]) / (hi - lo)
    out[0] = values[0]
    out[-1] = values[-1]
    return out


def smooth_xy(points: list[TrajectoryPoint], window: int, passes: int) -> list[TrajectoryPoint]:
    if window <= 1 or passes <= 0 or len(points) <= 2:
        return [TrajectoryPoint(p.t, p.x, p.y, p.yaw, p.s) for p in points]

    xs = [p.x for p in points]
    ys = [p.y for p in points]
    for _ in range(passes):
        xs = moving_average(xs, window)
        ys = moving_average(ys, window)

    return [
        TrajectoryPoint(p.t, xs[i], ys[i], p.yaw, p.s)
        for i, p in enumerate(points)
    ]


def unwrap_angles(angles: list[float]) -> list[float]:
    if not angles:
        return []
    out = [angles[0]]
    for angle in angles[1:]:
        prev = out[-1]
        delta = normalize_angle(angle - prev)
        out.append(prev + delta)
    return out


def apply_yaw_mode(
    smoothed: list[TrajectoryPoint],
    resampled: list[TrajectoryPoint],
    yaw_mode: str,
    window: int,
    passes: int,
) -> list[TrajectoryPoint]:
    if yaw_mode == "keep":
        yaws = [p.yaw for p in resampled]
    elif yaw_mode == "smooth":
        yaws = unwrap_angles([p.yaw for p in resampled])
        for _ in range(max(0, passes)):
            yaws = moving_average(yaws, window)
        yaws = [normalize_angle(yaw) for yaw in yaws]
    elif yaw_mode == "tangent":
        yaws = tangent_lidar_yaws(smoothed, [p.yaw for p in resampled])
    else:
        raise ValueError(f"未知 yaw_mode: {yaw_mode}")

    return [
        TrajectoryPoint(p.t, p.x, p.y, normalize_angle(yaws[i]), p.s)
        for i, p in enumerate(smoothed)
    ]


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def blend_angle(from_yaw: float, to_yaw: float, weight: float) -> float:
    return normalize_angle(from_yaw + smoothstep(weight) * normalize_angle(to_yaw - from_yaw))


def blend_endpoint_yaws(
    points: list[TrajectoryPoint],
    start_yaw: float,
    end_yaw: float,
    blend_distance: float,
) -> list[TrajectoryPoint]:
    """首尾保留原始 yaw，并在 blend_distance 内平滑过渡到生成 yaw。"""
    if not points:
        return []
    if len(points) == 1 or blend_distance <= 0.0:
        preserved = [TrajectoryPoint(p.t, p.x, p.y, p.yaw, p.s) for p in points]
        preserved[0].yaw = normalize_angle(start_yaw)
        preserved[-1].yaw = normalize_angle(end_yaw)
        return preserved

    total_s = points[-1].s
    blended: list[TrajectoryPoint] = []
    for p in points:
        yaw = p.yaw
        if p.s <= blend_distance:
            yaw = blend_angle(start_yaw, yaw, p.s / blend_distance)
        dist_to_end = max(0.0, total_s - p.s)
        if dist_to_end <= blend_distance:
            yaw = blend_angle(end_yaw, yaw, dist_to_end / blend_distance)
        blended.append(TrajectoryPoint(p.t, p.x, p.y, normalize_angle(yaw), p.s))

    blended[0].yaw = normalize_angle(start_yaw)
    blended[-1].yaw = normalize_angle(end_yaw)
    return blended


ENDPOINT_YAW_BLEND_DISTANCE = 0.30


def tangent_lidar_yaws(points: list[TrajectoryPoint], fallback_yaws: list[float]) -> list[float]:
    yaws: list[float] = []
    last_valid = fallback_yaws[0] if fallback_yaws else 0.0
    for i, point in enumerate(points):
        if len(points) == 1:
            dx = 0.0
            dy = 0.0
        elif i == 0:
            dx = points[1].x - point.x
            dy = points[1].y - point.y
        elif i == len(points) - 1:
            dx = point.x - points[i - 1].x
            dy = point.y - points[i - 1].y
        else:
            dx = points[i + 1].x - points[i - 1].x
            dy = points[i + 1].y - points[i - 1].y

        if math.hypot(dx, dy) <= 1e-9:
            yaw = last_valid
        else:
            # 轨迹切线是车体前进方向；当前控制代码的轨迹 yaw 是雷达 yaw。
            yaw = normalize_angle(math.atan2(dy, dx) - math.pi * 0.5)
            last_valid = yaw
        yaws.append(yaw)
    return yaws


def retime(points: list[TrajectoryPoint], dt: float) -> list[TrajectoryPoint]:
    return [
        TrajectoryPoint(i * dt, p.x, p.y, p.yaw, p.s)
        for i, p in enumerate(points)
    ]


def path_length(points: list[TrajectoryPoint]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(
        math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y)
        for i in range(1, len(points))
    )


def max_xy_offset(a: list[TrajectoryPoint], b: list[TrajectoryPoint]) -> float:
    n = min(len(a), len(b))
    if n == 0:
        return 0.0
    return max(math.hypot(a[i].x - b[i].x, a[i].y - b[i].y) for i in range(n))


def write_trajectory(
    path: Path,
    header_lines: list[str],
    source: Path,
    points: list[TrajectoryPoint],
    spacing: float,
    window: int,
    passes: int,
    dt: float,
    yaw_mode: str,
    trim_dwell: bool,
) -> None:
    lines: list[str] = []
    lines.append("# Smoothed trajectory: time_sec x y yaw_rad (lidar/map frame)")
    lines.append(f"# Source: {source}")
    lines.append(
        f"# smooth_trajectory.py spacing={spacing:.3f} window={window} "
        f"passes={passes} dt={dt:.3f} yaw_mode={yaw_mode} trim_dwell={trim_dwell}"
    )
    for line in header_lines:
        if line and line not in lines:
            lines.append(line)
    for p in points:
        lines.append(f"{p.t:.3f} {p.x:.6f} {p.y:.6f} {p.yaw:.6f}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    default_input = (
        Path(__file__).resolve().parent.parent
        / "commands/task_manager/traj/pole2.txt"
    )
    parser = argparse.ArgumentParser(description="平滑轨迹 txt：等距重采样 + 局部平滑")
    parser.add_argument("-i", "--input", type=Path, default=default_input, help="输入轨迹文件")
    parser.add_argument("-o", "--output", type=Path, default=None, help="输出文件，默认 <input>_smooth.txt")
    parser.add_argument("--spacing", type=float, default=0.05, help="等距重采样间隔(m)，默认 0.05")
    parser.add_argument("--window", type=int, default=5, help="x/y 平滑窗口，必须为奇数，默认 5")
    parser.add_argument("--passes", type=int, default=1, help="平滑次数，默认 1")
    parser.add_argument("--dt", type=float, default=0.1, help="输出时间步长(s)，默认 0.1")
    parser.add_argument(
        "--yaw-mode",
        choices=("tangent", "smooth", "keep"),
        default="tangent",
        help=(
            "yaw 生成方式：tangent=按路径切线重算雷达yaw，smooth=平滑原yaw，"
            "keep=保留插值yaw；中间段按该模式生成，起点/终点yaw精确保留，"
            "首尾约0.3m平滑过渡"
        ),
    )
    parser.add_argument("--trim-dwell", action="store_true", help="启用首尾停留段裁剪")
    parser.add_argument("--pos-eps", type=float, default=0.08, help="停留段相对锚点位移阈值(m)，默认 0.08")
    parser.add_argument("--step-eps", type=float, default=0.04, help="停留段相邻点步长阈值(m)，默认 0.04")
    parser.add_argument("--min-trim-points", type=int, default=3, help="删除点数少于该值时不删，默认 3")
    parser.add_argument("--dry-run", action="store_true", help="只打印统计，不写文件")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.spacing <= 0:
        raise ValueError("--spacing 必须大于 0")
    if args.dt <= 0:
        raise ValueError("--dt 必须大于 0")
    if args.window < 1:
        raise ValueError("--window 必须大于等于 1")
    if args.window % 2 == 0:
        raise ValueError("--window 必须是奇数")
    if args.passes < 0:
        raise ValueError("--passes 必须大于等于 0")
    if args.pos_eps <= 0 or args.step_eps <= 0:
        raise ValueError("--pos-eps 和 --step-eps 必须大于 0")
    if args.min_trim_points < 0:
        raise ValueError("--min-trim-points 必须大于等于 0")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)

        input_path = args.input.resolve()
        if not input_path.is_file():
            raise FileNotFoundError(f"找不到输入文件: {input_path}")
        output_path = (
            args.output.resolve()
            if args.output is not None
            else input_path.with_name(input_path.stem + "_smooth" + input_path.suffix)
        )

        header_lines, original = parse_trajectory(input_path)
        raw_length = path_length(original)

        source_points = original
        leading_removed = 0
        trailing_removed = 0
        if args.trim_dwell:
            source_points, leading_removed, trailing_removed = trim_dwell_segments(
                original, args.pos_eps, args.step_eps, args.min_trim_points
            )
        start_yaw = source_points[0].yaw
        end_yaw = source_points[-1].yaw

        resampled = resample_by_spacing([
            TrajectoryPoint(p.t, p.x, p.y, p.yaw) for p in source_points
        ], args.spacing)
        smoothed_xy = smooth_xy(resampled, args.window, args.passes)
        with_yaw = apply_yaw_mode(smoothed_xy, resampled, args.yaw_mode, args.window, args.passes)
        with_yaw = blend_endpoint_yaws(
            with_yaw, start_yaw, end_yaw, ENDPOINT_YAW_BLEND_DISTANCE
        )
        output_points = retime(with_yaw, args.dt)

        output_length = path_length(output_points)
        avg_step = output_length / max(len(output_points) - 1, 1)
        max_offset = max_xy_offset(resampled, smoothed_xy)

        print(f"输入: {input_path}")
        print(f"输出: {output_path}")
        print(f"原始点数: {len(original)}, 原始长度: {raw_length:.3f} m")
        if args.trim_dwell:
            print(f"裁剪停留段: 删除首部 {leading_removed} 点, 删除尾部 {trailing_removed} 点")
        else:
            print("裁剪停留段: 未启用")
        print(f"重采样间隔: {args.spacing:.3f} m, 平滑窗口: {args.window}, 平滑次数: {args.passes}")
        print(f"yaw 模式: {args.yaw_mode}, 输出 dt: {args.dt:.3f} s")
        print(f"输出点数: {len(output_points)}, 输出长度: {output_length:.3f} m, 平均点距: {avg_step:.3f} m")
        print(f"最大平滑偏移: {max_offset:.3f} m")
        print(
            "起终点: "
            f"({output_points[0].x:.3f}, {output_points[0].y:.3f}) -> "
            f"({output_points[-1].x:.3f}, {output_points[-1].y:.3f})"
        )

        if args.dry_run:
            return 0

        write_trajectory(
            output_path,
            header_lines,
            input_path,
            output_points,
            args.spacing,
            args.window,
            args.passes,
            args.dt,
            args.yaw_mode,
            args.trim_dwell,
        )
        print(f"已写入: {output_path}")
        return 0
    except Exception as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
