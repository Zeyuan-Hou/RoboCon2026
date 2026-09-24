#!/usr/bin/env python3
"""
使用 ROS2 原生接口将 rosbag2 转换为 CSV 轨迹文件
不需要安装 rosbags 库

使用方法:
ros2 run arm_node rosbag_to_csv_ros2 <bag_directory> [output_csv]

示例:
ros2 run arm_node rosbag_to_csv_ros2 ~/grab_trajectory grab_trajectory.csv
"""

import sys
import csv
import os
import sqlite3


def read_rosbag2_sqlite(bag_dir, topic_name='/quad/manipulator/motor_state'):
    """
    从 rosbag2 SQLite 数据库读取消息
    rosbag2 存储格式:
    - metadata.yaml: 元数据
    - <bag_name>_0.db3: SQLite 数据库
    """
    # 查找 db3 文件
    db_files = [f for f in os.listdir(bag_dir) if f.endswith('.db3')]
    if not db_files:
        print(f"错误: 在 {bag_dir} 中未找到 .db3 文件")
        return None

    db_path = os.path.join(bag_dir, db_files[0])
    print(f"读取数据库: {db_path}")

    # 连接数据库
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # 获取话题 ID
    cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
    result = cursor.fetchone()
    if not result:
        print(f"错误: 未找到话题 {topic_name}")
        # 列出可用话题
        cursor.execute("SELECT name FROM topics")
        topics = cursor.fetchall()
        print("可用话题:")
        for t in topics:
            print(f"  - {t[0]}")
        return None

    topic_id = result[0]
    print(f"话题 ID: {topic_id}")

    # 读取消息
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,)
    )

    messages = cursor.fetchall()
    conn.close()

    print(f"读取到 {len(messages)} 条消息")
    return messages


def parse_motor_state_data(data_bytes):
    """
    解析 MotorState 消息的二进制数据
    这是简化版，假设消息格式为:
    - float64[] pos
    - float64[] vel
    - float64[] cur
    """
    import struct

    # ROS2 消息序列化格式:
    # 4字节: 数组长度 (小端)
    # 然后是数组数据

    offset = 0

    # 解析 pos 数组
    pos_len = struct.unpack('<I', data_bytes[offset:offset+4])[0]
    offset += 4
    pos = []
    for i in range(pos_len):
        pos.append(struct.unpack('<d', data_bytes[offset:offset+8])[0])
        offset += 8

    # 解析 vel 数组 (跳过)
    vel_len = struct.unpack('<I', data_bytes[offset:offset+4])[0]
    offset += 4 + vel_len * 8

    # 解析 cur 数组 (跳过)
    cur_len = struct.unpack('<I', data_bytes[offset:offset+4])[0]
    offset += 4 + cur_len * 8

    return pos


def convert_bag_to_csv(bag_dir, output_csv):
    """转换 bag 到 CSV"""
    messages = read_rosbag2_sqlite(bag_dir)
    if not messages:
        return False

    trajectory = []
    start_time = None

    for timestamp, data in messages:
        # 转换时间戳为秒
        t_sec = timestamp / 1e9

        if start_time is None:
            start_time = t_sec

        elapsed = t_sec - start_time

        # 解析消息数据
        try:
            pos = parse_motor_state_data(data)
            if len(pos) >= 2:
                trajectory.append({
                    'timestamp': elapsed,
                    'pos0': pos[0],
                    'pos1': pos[1]
                })
        except Exception as e:
            print(f"解析消息失败: {e}")
            continue

    if not trajectory:
        print("错误: 没有提取到轨迹数据")
        return False

    # 保存为 CSV
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'pos0', 'pos1'])
        for point in trajectory:
            writer.writerow([point['timestamp'], point['pos0'], point['pos1']])

    print(f"成功转换: {len(trajectory)} 个点")
    print(f"输出文件: {output_csv}")
    print(f"轨迹时长: {trajectory[-1]['timestamp']:.3f} 秒")
    return True


def main():
    if len(sys.argv) < 2:
        print("用法: python3 rosbag_to_csv_ros2.py <bag_directory> [output_csv]")
        print("示例: python3 rosbag_to_csv_ros2.py ~/grab_trajectory grab_trajectory.csv")
        sys.exit(1)

    bag_dir = sys.argv[1]
    output_csv = sys.argv[2] if len(sys.argv) > 2 else 'grab_trajectory.csv'

    if not os.path.isdir(bag_dir):
        print(f"错误: {bag_dir} 不是有效的目录")
        sys.exit(1)

    convert_bag_to_csv(bag_dir, output_csv)


if __name__ == '__main__':
    main()
