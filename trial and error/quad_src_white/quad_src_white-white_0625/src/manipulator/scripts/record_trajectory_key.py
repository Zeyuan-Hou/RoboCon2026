#!/usr/bin/env python3
"""
按键式轨迹记录脚本
订阅 /quad/manipulator/motor_state 话题，按用户按键记录电机位置关键点

使用方法:
1. 运行此脚本: python3 record_trajectory_key.py
2. 手动控制机械臂到目标位置
3. 按 Enter 键记录当前位置为一个轨迹点
4. 重复步骤 2-3 记录多个关键点
5. 按 Ctrl+C 停止记录并保存
6. 轨迹将保存到 grab_trajectory_key.csv

特点:
- 只记录用户指定的关键点，而非连续轨迹
- 适合记录稀疏的轨迹关键点，后续可通过插值生成完整轨迹
- 可配合 fit_pos_relationship.py 进行轨迹拟合
"""

import rclpy
from rclpy.node import Node
from quad.msg import MotorState
import csv
import signal
import sys
import threading
import time


class KeyTrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('key_trajectory_recorder')

        # 创建订阅器
        self.subscription = self.create_subscription(
            MotorState,
            '/quad/manipulator/motor_state',
            self.motor_state_callback,
            10
        )

        # 当前电机状态
        self.current_pos0 = None
        self.current_pos1 = None
        self.state_received = False

        # 轨迹数据
        self.trajectory = []
        self.point_index = 0

        # 线程锁
        self.lock = threading.Lock()

        self.get_logger().info('========================================')
        self.get_logger().info('按键式轨迹记录器已启动')
        self.get_logger().info('========================================')
        self.get_logger().info('等待电机状态消息...')
        self.get_logger().info('')
        self.get_logger().info('操作说明:')
        self.get_logger().info('  - 按 Enter 键: 记录当前电机位置为一个轨迹点')
        self.get_logger().info('  - 按 Ctrl+C  : 停止记录并保存到CSV文件')
        self.get_logger().info('')
        self.get_logger().info('提示: 建议记录 5-20 个关键点，覆盖整个运动过程')

    def motor_state_callback(self, msg):
        if len(msg.pos) < 2:
            return

        with self.lock:
            self.current_pos0 = msg.pos[0]
            self.current_pos1 = msg.pos[1]
            if not self.state_received:
                self.state_received = True
                self.get_logger().info('已连接到电机状态话题，可以开始记录')
                self.get_logger().info('请按 Enter 键记录第一个轨迹点...')

    def record_point(self):
        """记录当前位置为一个轨迹点"""
        with self.lock:
            if not self.state_received:
                self.get_logger().warn('尚未收到电机状态，无法记录')
                return False

            self.point_index += 1
            self.trajectory.append({
                'index': self.point_index,
                'pos0': self.current_pos0,
                'pos1': self.current_pos1
            })

            self.get_logger().info(
                f'已记录第 {self.point_index} 个点: '
                f'pos0={self.current_pos0:.4f}, pos1={self.current_pos1:.4f}'
            )
            return True

    def save_trajectory(self, filename='grab_trajectory_key.csv'):
        """保存轨迹到CSV文件"""
        if not self.trajectory:
            self.get_logger().warn('没有记录到轨迹数据')
            return

        # 生成时间戳（等间隔）
        duration = 3.0  # 假设总时长 3 秒
        dt = duration / len(self.trajectory) if len(self.trajectory) > 1 else 0

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'pos0', 'pos1'])
            for i, point in enumerate(self.trajectory):
                timestamp = i * dt
                writer.writerow([timestamp, point['pos0'], point['pos1']])

        self.get_logger().info('========================================')
        self.get_logger().info(f'轨迹已保存到 {filename}')
        self.get_logger().info(f'共记录 {len(self.trajectory)} 个关键点')
        self.get_logger().info(f'建议总时长: {duration:.1f}s')
        self.get_logger().info('========================================')
        self.get_logger().info('')
        self.get_logger().info('后续步骤:')
        self.get_logger().info('1. 使用 fit_pos_relationship.py 进行轨迹拟合')
        self.get_logger().info('2. 生成平滑的完整轨迹供 arm_node_td 使用')

    def print_summary(self):
        """打印记录的轨迹摘要"""
        if not self.trajectory:
            return

        self.get_logger().info('')
        self.get_logger().info('当前记录的轨迹点:')
        self.get_logger().info('  序号  |  pos0      |  pos1')
        self.get_logger().info('  ------|-----------|-----------')
        for p in self.trajectory:
            self.get_logger().info(
                f"  {p['index']:4d}  |  {p['pos0']:9.4f}  |  {p['pos1']:9.4f}"
            )


def input_thread_func(recorder):
    """按键监听线程"""
    while rclpy.ok():
        try:
            # 等待用户输入
            user_input = input()

            # 空输入（直接按Enter）表示记录一个点
            if user_input.strip() == '':
                recorder.record_point()
            # 输入 's' 或 'S' 显示摘要
            elif user_input.strip().lower() == 's':
                recorder.print_summary()
            # 输入 'q' 或 'Q' 退出
            elif user_input.strip().lower() == 'q':
                recorder.get_logger().info('用户请求退出...')
                recorder.save_trajectory()
                rclpy.shutdown()
                break
            else:
                recorder.get_logger().info(
                    f"未知命令: '{user_input}'，按 Enter 记录点，按 's' 显示摘要，按 'q' 退出"
                )

        except EOFError:
            break
        except Exception as e:
            recorder.get_logger().error(f'输入线程错误: {e}')


def main(args=None):
    rclpy.init(args=args)
    recorder = KeyTrajectoryRecorder()

    # 启动按键监听线程
    input_thread = threading.Thread(target=input_thread_func, args=(recorder,))
    input_thread.daemon = True
    input_thread.start()

    # 设置信号处理
    def signal_handler(sig, frame):
        recorder.get_logger().info('')
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
