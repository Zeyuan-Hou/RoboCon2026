#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
线性关系轨迹生成脚本

根据 pos1 = k * pos0 + b 的线性关系生成平滑轨迹

功能：
1. 在指定的 pos0 范围内生成等间隔轨迹点
2. 根据线性方程计算对应的 pos1
3. 对 pos0 进行平滑处理（可选）
4. 生成等时间间隔的 CSV 轨迹文件

使用方法：
    cd /home/guo/HITCRT_QUAD/hitcrt_quad_ws/src/manipulator/scripts
    python3 generate_linear_trajectory.py

输出：
    linear_trajectory.csv - 生成的轨迹文件
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# ==================== 配置参数 ====================
OUTPUT_DIR = '/home/guo/HITCRT_QUAD/hitcrt_quad_ws/src/manipulator/scripts'

# pos0 范围
POS0_START = -1.0   # pos0 起始位置
POS0_END = -5.0     # pos0 结束位置

# 线性关系参数: pos1 = k * pos0 + b
K = -0.425820       # 斜率
B = 0.085607        # 截距

# 轨迹参数
NUM_POINTS = 500    # 轨迹点数
TRAJECTORY_DURATION = 3.0  # 轨迹总时长（秒）

# 平滑参数
USE_SMOOTHING = False  # 是否对 pos0 进行平滑（使用正弦加速度曲线）


def generate_trajectory(pos0_start, pos0_end, k, b, num_points, duration, use_smoothing=True):
    """
    生成轨迹
    
    参数:
        pos0_start: pos0 起始位置
        pos0_end: pos0 结束位置
        k: 线性方程斜率
        b: 线性方程截距
        num_points: 轨迹点数
        duration: 轨迹总时长
        use_smoothing: 是否使用 S 曲线平滑
    
    返回:
        t, pos0, pos1
    """
    # 生成时间戳
    t = np.linspace(0, duration, num_points)
    
    if use_smoothing:
        # 使用 S 曲线（正弦加速度）进行平滑
        # 归一化时间 [0, 1]
        tau = t / duration
        # S 曲线: 从 0 平滑过渡到 1
        # 使用 0.5 * (1 - cos(pi * tau)) 保证起点和终点速度为 0
        s = 0.5 * (1 - np.cos(np.pi * tau))
        # 映射到 pos0 范围
        pos0 = pos0_start + s * (pos0_end - pos0_start)
    else:
        # 线性插值
        pos0 = np.linspace(pos0_start, pos0_end, num_points)
    
    # 根据线性关系计算 pos1
    pos1 = k * pos0 + b
    
    return t, pos0, pos1


def plot_trajectory(t, pos0, pos1, k, b, output_dir):
    """绘制轨迹图"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # pos0 时间曲线
    ax = axes[0, 0]
    ax.plot(t, pos0, 'b-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Motor0 Position (rad)')
    ax.set_title('Motor0 Position vs Time')
    ax.grid(True, alpha=0.3)
    
    # pos1 时间曲线
    ax = axes[0, 1]
    ax.plot(t, pos1, 'r-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Motor1 Position (rad)')
    ax.set_title('Motor1 Position vs Time')
    ax.grid(True, alpha=0.3)
    
    # pos1 vs pos0 关系
    ax = axes[1, 0]
    ax.plot(pos0, pos1, 'g-', linewidth=2, label='Trajectory')
    # 绘制理论直线
    pos0_theory = np.linspace(pos0.min(), pos0.max(), 100)
    pos1_theory = k * pos0_theory + b
    ax.plot(pos0_theory, pos1_theory, 'r--', linewidth=1, alpha=0.7, label='Theory')
    ax.set_xlabel('Motor0 Position (rad)')
    ax.set_ylabel('Motor1 Position (rad)')
    ax.set_title(f'Motor1 vs Motor0 (pos1={k:.6f}*pos0+{b:.6f})')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 速度曲线
    ax = axes[1, 1]
    vel0 = np.diff(pos0) / np.diff(t)
    vel1 = np.diff(pos1) / np.diff(t)
    ax.plot(t[:-1], vel0, 'b-', linewidth=1.5, label='Motor0 Velocity')
    ax.plot(t[:-1], vel1, 'r-', linewidth=1.5, label='Motor1 Velocity')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (rad/s)')
    ax.set_title('Velocity Profile')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plot_path = os.path.join(output_dir, 'linear_trajectory_plot.png')
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    print(f"轨迹图已保存: {plot_path}")
    plt.close()


def save_trajectory(t, pos0, pos1, filepath):
    """保存轨迹到 CSV 文件"""
    df = pd.DataFrame({
        'timestamp': t,
        'pos0': pos0,
        'pos1': pos1
    })
    df.to_csv(filepath, index=False)
    print(f"\n轨迹已保存: {filepath}")
    print(f"总点数: {len(t)}")
    print(f"时间范围: [{t[0]:.4f}, {t[-1]:.4f}] s")
    print(f"motor0 范围: [{pos0[0]:.4f}, {pos0[-1]:.4f}] rad")
    print(f"motor1 范围: [{pos1[0]:.4f}, {pos1[-1]:.4f}] rad")


def main():
    print("=" * 60)
    print("线性关系轨迹生成脚本")
    print("=" * 60)
    
    print(f"\n参数配置:")
    print(f"  pos0 范围: [{POS0_START}, {POS0_END}] rad")
    print(f"  线性方程: pos1 = {K:.6f} * pos0 + {B:.6f}")
    print(f"  轨迹点数: {NUM_POINTS}")
    print(f"  轨迹时长: {TRAJECTORY_DURATION} s")
    print(f"  平滑模式: {'S曲线' if USE_SMOOTHING else '线性'}")
    
    # 生成轨迹
    print("\n[1/3] 生成轨迹...")
    t, pos0, pos1 = generate_trajectory(
        POS0_START, POS0_END, K, B,
        NUM_POINTS, TRAJECTORY_DURATION, USE_SMOOTHING
    )
    
    # 绘制轨迹
    print("[2/3] 绘制轨迹图...")
    plot_trajectory(t, pos0, pos1, K, B, OUTPUT_DIR)
    
    # 保存轨迹
    print("[3/3] 保存轨迹...")
    output_file = os.path.join(OUTPUT_DIR, 'linear_trajectory.csv')
    save_trajectory(t, pos0, pos1, output_file)
    
    # 打印使用建议
    print(f"\n{'='*60}")
    print("使用建议")
    print(f"{'='*60}")
    print(f"1. 生成的轨迹文件: {output_file}")
    print(f"2. 在 arm_node_td 参数中设置:")
    print(f'   trajectory_file: "{output_file}"')
    print(f"   grab_motor0_pos: {pos0[0]:.4f}")
    print(f"   grab_motor1_pos: {pos1[0]:.4f}")
    print(f"3. 轨迹时长: {t[-1]:.4f}s")
    print(f"4. 轨迹图: {os.path.join(OUTPUT_DIR, 'linear_trajectory_plot.png')}")
    print(f"\n注意: 此轨迹基于线性关系生成，确保腕部角度关系一致")
    print("=" * 60)


if __name__ == '__main__':
    main()
