#!/usr/bin/env python3
"""
轨迹记录脚本
订阅 /quad/manipulator/motor_state 话题，记录电机位置轨迹到CSV文件

使用方法:
1. 手动控制机械臂到起始位置
2. 运行此脚本开始记录: python3 record_trajectory.py
3. 手动控制机械臂完成抓取动作
4. 按 Ctrl+C 停止记录
5. 轨迹将保存到 grab_trajectory.csv
"""

import rclpy
from rclpy.node import Node
from quad.msg import MotorState
import csv
import signal
import sys


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # 创建订阅器
        self.subscription = self.create_subscription(
            MotorState,
            '/quad/manipulator/motor_state',
            self.motor_state_callback,
            10
        )

        # 轨迹数据
        self.trajectory = []
        self.start_time = None
        self.recording = False

        self.get_logger().info('轨迹记录器已启动')
        self.get_logger().info('等待电机状态消息...')
        self.get_logger().info('按 Ctrl+C 停止记录并保存')

    def motor_state_callback(self, msg):
        if len(msg.pos) < 2:
            return

        # 获取当前时间
        current_time = self.get_clock().now()

        # 第一次收到消息时记录开始时间
        if self.start_time is None:
            self.start_time = current_time
            self.recording = True
            self.get_logger().info('开始记录轨迹...')

        # 计算相对时间（秒）
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        # 记录数据点
        self.trajectory.append({
            'timestamp': elapsed,
            'pos0': msg.pos[0],
            'pos1': msg.pos[1]
        })

    def save_trajectory(self, filename='grab_trajectory.csv'):
        if not self.trajectory:
            self.get_logger().warn('没有记录到轨迹数据')
            return

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'pos0', 'pos1'])
            for point in self.trajectory:
                writer.writerow([point['timestamp'], point['pos0'], point['pos1']])

        self.get_logger().info(f'轨迹已保存到 {filename}')
        self.get_logger().info(f'共记录 {len(self.trajectory)} 个点')


def main(args=None):
    rclpy.init(args=args)
    recorder = TrajectoryRecorder()

    # 设置信号处理
    def signal_handler(sig, frame):
        recorder.get_logger().info('停止记录...')
        recorder.save_trajectory()
        recorder.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.save_trajectory()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
