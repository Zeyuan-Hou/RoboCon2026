#!/usr/bin/env python3
"""
订阅 /tf，按固定间隔记录 camera_init -> aft_mapped 位姿，生成 AUTO_PARKU 轨迹 txt。

格式（与 example_path.txt 一致）:
  time_sec x y yaw_rad

用法:
  source /opt/ros/humble/setup.bash   # 或你的 ROS2 环境
  source install/setup.bash
  python3 record_trajectory.py
  python3 record_trajectory.py -o /path/to/my_path.txt --duration 20 --interval 0.1
"""

import argparse
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """从四元数提取 yaw（与 lidar_nav_demo_node.py / tf2 getRPY 的 yaw 一致）。"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class TrajectoryRecorder(Node):
    def __init__(
        self,
        output_path: Path,
        duration_sec: float,
        interval_sec: float,
        target_frame: str,
        child_frame: str,
        tf_topic: str,
    ):
        super().__init__('trajectory_recorder')

        self.output_path = output_path
        self.duration_sec = duration_sec
        self.interval_sec = interval_sec
        self.target_frame = target_frame
        self.child_frame = child_frame

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got_tf = False

        self.samples: list[tuple[float, float, float, float]] = []
        self.recording_done = False
        self._record_started = False
        self._t0 = None

        self.tf_sub = self.create_subscription(
            TFMessage, tf_topic, self.tf_callback, 100
        )
        self.sample_timer = self.create_timer(interval_sec, self.sample_callback)

        self.get_logger().info(
            f'Recording TF {target_frame} -> {child_frame} from {tf_topic}, '
            f'interval={interval_sec}s, duration={duration_sec}s'
        )
        self.get_logger().info(f'Output: {output_path}')

    def tf_callback(self, msg: TFMessage) -> None:
        """与 lidar_nav_callbacks.cpp tf_callback 相同：取指定 parent/child 的平移与 yaw。"""
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.target_frame
                and transform.child_frame_id == self.child_frame
            ):
                self.x = transform.transform.translation.x
                self.y = transform.transform.translation.y
                q = transform.transform.rotation
                self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                self.got_tf = True
                break

    def sample_callback(self) -> None:
        if self.recording_done:
            return

        now = self.get_clock().now()
        if not self._record_started:
            if not self.got_tf:
                self.get_logger().warn('Waiting for TF before recording...')
                return
            self._t0 = now
            self._record_started = True
            self._append_sample(0.0)
            return

        elapsed = (now - self._t0).nanoseconds * 1e-9
        if elapsed >= self.duration_sec:
            self._finish()
            return

        self._append_sample(elapsed)

    def _append_sample(self, t: float) -> None:
        if not self.got_tf:
            return
        # 避免同一时刻重复写入（定时器与启动瞬间）
        if self.samples and abs(self.samples[-1][0] - t) < 1e-6:
            return
        self.samples.append((t, self.x, self.y, self.yaw))

    def _finish(self) -> None:
        if self.recording_done:
            return
        self.recording_done = True

        if len(self.samples) < 2:
            self.get_logger().error(
                f'Only {len(self.samples)} samples; need at least 2. File not written.'
            )
            return

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open('w', encoding='utf-8') as f:
            f.write('# AUTO_PARKU trajectory: time_sec x y yaw_rad (lidar/map frame)\n')
            for t, x, y, yaw in self.samples:
                f.write(f'{t:.3f} {x:.6f} {y:.6f} {yaw:.6f}\n')

        self.get_logger().info(
            f'Saved {len(self.samples)} points, '
            f't=[0, {self.samples[-1][0]:.3f}] s -> {self.output_path}'
        )


def parse_args() -> argparse.Namespace:
    default_out = (
        Path(__file__).resolve().parent.parent
        / 'commands/task_manager/traj/stairs1.txt'
        # slope
        # bridge_a
        # bridge_a2stairs
        # stairs
    )
    parser = argparse.ArgumentParser(description='Record lidar TF trajectory to txt')
    parser.add_argument('-o', '--output', type=Path, default=default_out, help='输出 txt 路径')
    parser.add_argument('--duration', type=float, default=30.0, help='录制时长 (秒)')
    parser.add_argument('--interval', type=float, default=0.1, help='采样间隔 (秒)')
    parser.add_argument('--target-frame', default='camera_init', help='TF parent frame')
    parser.add_argument('--child-frame', default='aft_mapped', help='TF child frame')
    parser.add_argument('--tf-topic', default='/tf', help='TF 话题')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.duration <= 0 or args.interval <= 0:
        print('duration and interval must be positive', file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = TrajectoryRecorder(
        output_path=args.output.resolve(),
        duration_sec=args.duration,
        interval_sec=args.interval,
        target_frame=args.target_frame,
        child_frame=args.child_frame,
        tf_topic=args.tf_topic,
    )

    try:
        while rclpy.ok() and not node.recording_done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted; finishing if samples exist...')
        if not node.recording_done and len(node.samples) >= 2:
            node._finish()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
