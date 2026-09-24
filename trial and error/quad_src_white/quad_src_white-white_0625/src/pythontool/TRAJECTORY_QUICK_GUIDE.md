# 障碍路径录制与绘制快速指南

这份指南只讲现场最常用操作：录制、标记成功、保存路径、批量画图。

## 1. 准备

先启动机器人定位，确认 `/tf` 中有：

```text
camera_init -> aft_mapped
```

然后在工作区根目录运行命令：

```bash
cd /home/cat/hitcrt_quad2026_ws
```

## 2. 一键录制路径

推荐使用一体化录制工具。它会自动完成：

- 选择障碍
- 录制原始路径
- 剪切首尾停留段
- 平滑优化路径
- 保存 raw / trimmed / smooth 三份 txt
- 询问本次是否成功
- 成功时自动备份旧正式路径，并发布新 smooth 路径

一键命令：

```bash
bash -lc 'cd /home/cat/hitcrt_quad2026_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && 


python3 src/pythontool/trajectory_workflow.py'
```

进入菜单后：

1. 输入障碍编号或名称，例如 `pole`、`sand`、`bridge_a`。
2. 输入录制时长，直接回车默认 `30` 秒。
3. 输入采样间隔，直接回车默认 `0.1` 秒。
4. 机器人走完路径后，程序会自动生成优化文件。
5. 如果这次跑得好，回答 `y` 发布为正式路径；如果失败，回答 `n`，只保存记录不覆盖正式路径。

## 3. 录制文件保存位置

每次录制都会保存到：

```text
src/commands/task_manager/traj/records/<障碍名>/<时间戳>/
```

例如：

```text
src/commands/task_manager/traj/records/pole/20260629_145618/
```

里面有：

```text
pole_raw.txt       原始录制路径
pole_trimmed.txt   剪切后的路径
pole_smooth.txt    优化后的路径
record_info.txt    本次录制参数、是否成功、点数、长度等信息
```

正式比赛路径仍在：

```text
src/commands/task_manager/traj/
```

只有录制结束后标记成功，工具才会更新正式路径。旧正式路径会自动备份为：

```text
xxx_smooth.bak_<时间戳>.txt
```

## 4. 一键绘制某个文件夹里的所有路径

绘制脚本支持传入文件夹。默认会扫描该文件夹第一层所有 `*.txt`，并为每条路径单独生成一张图。

一键绘制正式路径目录：

```bash
bash -lc 'cd /home/cat/hitcrt_quad2026_ws && python3 src/pythontool/plot_trajectory.py src/commands/task_manager/traj'
```

只绘制优化后的路径：

```bash
bash -lc 'cd /home/cat/hitcrt_quad2026_ws && python3 src/pythontool/plot_trajectory.py src/commands/task_manager/traj --pattern "*_smooth.txt"'
```

递归绘制历史录制记录：

```bash
bash -lc 'cd /home/cat/hitcrt_quad2026_ws && python3 src/pythontool/plot_trajectory.py src/commands/task_manager/traj/records --recursive'
```

输出文件在每条轨迹 txt 的同目录：

```text
<轨迹名>_viz.png
<轨迹名>_viz.svg
```

## 5. 绘制单条路径

```bash
bash -lc 'cd /home/cat/hitcrt_quad2026_ws && python3 src/pythontool/plot_trajectory.py src/commands/task_manager/traj/pole_final.txt'
```

倒车路径想看倒车车头方向时，加：

```bash
--drive-mode reverse
```

完整例子：

```bash
bash -lc 'cd /home/cat/hitcrt_quad2026_ws && python3 src/pythontool/plot_trajectory.py src/commands/task_manager/traj/pole_final.txt --drive-mode reverse'
```

## 6. 图怎么看

- 实线：路径位置。
- 圆点：起点。
- 方块：终点。
- 箭头：机器人车头朝向。
- 左下角文字：点数、路径长度、平均点距。

注意：图里的箭头是车头朝向，不是实际速度方向。实际前进或倒车要看运行时的 `cmd_vel.linear.x`。

## 7. 常用障碍名

```text
start_transit
pole
sand
slope
bridge_a
bridge_b
bridge_a2stairs
stairs
```

录制时输入这些名字即可直接选择对应障碍。

## 8. 推荐现场流程

```text
1. 启动定位和机器人基础程序。
2. 运行一键录制命令。
3. 选择障碍并录制。
4. 跑得好就标记成功，跑歪就标记失败。
5. 运行绘图命令检查 smooth 路径。
6. 启动障碍赛流程测试新路径。
```
