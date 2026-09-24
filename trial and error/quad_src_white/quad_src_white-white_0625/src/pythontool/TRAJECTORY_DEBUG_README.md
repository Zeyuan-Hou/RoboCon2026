# 轨迹录制、调试与前瞻跟踪说明

本文用于调试障碍赛中的 `action: "path"` 轨迹跟踪，包括录制轨迹、裁剪轨迹、可视化轨迹，以及在 `obstacle_race_params.yaml` 中配置前进/倒车跟踪。

## 1. 轨迹文件格式

轨迹文件是 txt，每个有效数据行 4 列：

```text
time_sec x y yaw
```

示例：

```text
# AUTO_PARKU trajectory: time_sec x y yaw_rad (lidar/map frame)
0.000 -0.219701 1.353889 0.295256
0.100 -0.244876 1.409973 0.305815
```

字段含义：

- `time_sec`：录制时刻。当前障碍赛前瞻跟踪不再严格按时间回放，但仍要求时间单调递增。
- `x/y`：雷达定位坐标系下的位置。
- `yaw`：雷达/TF yaw，不是车头 yaw。

车头朝向和雷达 yaw 的关系：

```text
forward 车头朝向 = lidar_yaw + pi/2
reverse 车头朝向 = lidar_yaw + pi/2 + pi
```

控制器内部仍然跟踪雷达 yaw，因为当前 TF 位姿里的 `yaw_` 也是雷达 yaw。可视化脚本为了方便人看，会把箭头转换成车头朝向。

## 2. 录制轨迹

录制脚本：

```text
src/pythontool/record_trajectory.py
```

运行前先启动机器人定位，并确保 `/tf` 里有：

```text
camera_init -> aft_mapped
```

常用命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/pythontool/record_trajectory.py \
  -o src/commands/task_manager/traj/pole2.txt \
  --duration 20 \
  --interval 0.1
```

参数：

- `-o / --output`：输出轨迹文件。
- `--duration`：录制时长，单位秒。
- `--interval`：采样间隔，单位秒。
- `--target-frame`：默认 `camera_init`。
- `--child-frame`：默认 `aft_mapped`。
- `--tf-topic`：默认 `/tf`。

如果不指定 `-o`，默认写到：

```text
src/commands/task_manager/traj/pole2.txt
```

## 3. 裁剪轨迹

裁剪脚本：

```text
src/pythontool/trim_trajectory.py
```

作用：

- 删除开头原地停留段。
- 删除结尾原地停留段。
- 把时间重排为 `0.0, 0.1, 0.2, ...`。

先 dry-run 看会删多少点：

```bash
python3 src/pythontool/trim_trajectory.py \
  -i src/commands/task_manager/traj/pole2.txt \
  --dry-run
```

输出到新文件：

```bash
python3 src/pythontool/trim_trajectory.py \
  -i src/commands/task_manager/traj/pole2.txt \
  -o src/commands/task_manager/traj/pole2_trimmed.txt
```

直接覆盖原文件：

```bash
python3 src/pythontool/trim_trajectory.py \
  -i src/commands/task_manager/traj/pole2.txt \
  --in-place
```

常调参数：

```text
--pos-eps   相对首/尾锚点位移小于该值时认为还在停留，默认 0.08 m
--step-eps  相邻采样点位移小于该值时认为没动，默认 0.04 m
--dt        重排后的时间间隔，默认 0.1 s
```

## 4. 可视化轨迹

可视化脚本：

```text
src/pythontool/plot_trajectory.py
```

画单条轨迹：

```bash
python3 src/pythontool/plot_trajectory.py \
  src/commands/task_manager/traj/pole2.txt
```

画倒车车头朝向：

```bash
python3 src/pythontool/plot_trajectory.py \
  src/commands/task_manager/traj/pole2.txt \
  --drive-mode reverse
```

画多条轨迹：

```bash
python3 src/pythontool/plot_trajectory.py \
  src/commands/task_manager/traj/pole2.txt \
  src/commands/task_manager/traj/sand1.txt
```

输出：

- 单文件默认输出 `<轨迹名>_viz.png` 和 `<轨迹名>_viz.svg`。
- 多文件默认输出 `trajectory_viz.png` 和 `trajectory_viz.svg`。

图中含义：

- 实线：轨迹点连线。
- 圆点：起点。
- 方块：终点。
- 虚线小段：示例点到前瞻参考点。
- 箭头：车头朝向，不是速度方向。

脚本不读取实车速度。实际速度方向要看运行时 `cmd_vel.linear.x`：

```text
linear.x > 0  前进
linear.x < 0  倒车
```

## 5. 前瞻跟踪算法

当前障碍赛 `path` 跟踪逻辑在：

```text
src/commands/task_manager/src/task_executor_node.cpp
```

核心流程：

1. 读取轨迹文件，解析 `time x y yaw`。
2. 预计算每个点的累计路径长度 `s`。
3. 每个控制周期用当前雷达位姿 `(x_, y_)` 找轨迹线段上的最近投影点。
4. 参考点取：

```text
target_s = closest_s + lookahead_distance
```

5. `target_s` 超过终点时夹到终点。
6. `x/y/yaw` 按路径弧长插值。
7. 用 PD 根据当前位姿和参考点误差输出 `cmd_vel`。

路径进度 `closest_s` 会单调不回退，避免弯道或定位噪声导致参考点跳回旧段。

结束条件：

```text
进度到达末段，并且当前位置到轨迹终点距离 < finish_tolerance
```

不再按录制时间强制结束。

## 6. YAML 配置

配置文件：

```text
src/commands/task_manager/config/obstacle_race_params.yaml
```

每个 `path_profiles.<slot>` 可以单独配置轨迹和跟踪参数：

```yaml
path_profiles:
  pole:
    trajectory_file: "/home/cat/hitcrt_quad2026_ws/src/commands/task_manager/traj/pole2.txt"
    drive_mode: "forward"
    kp_x: 0.8
    kp_y: 0.75
    kp_yaw: 0.8
    kd_x: 0.0
    kd_y: 0.0
    kd_yaw: 0.0
    lin_vel_creep_min: 0.0
    lidar_offset_x: 0.0
    lidar_offset_y: 0.0
    max_vx: 0.75
    max_vy: 0.5
    max_dyaw: 0.8
    lookahead_distance: 0.35
    finish_tolerance: 0.25
```

关键参数：

- `trajectory_file`：轨迹 txt 文件路径。
- `drive_mode: "forward"`：沿路径前进，车头按 `lidar_yaw + pi/2`。
- `drive_mode: "reverse"`：沿路径倒车，车头按 `lidar_yaw + pi/2 + pi`。
- `lookahead_distance`：前瞻距离。小一点贴线，太小可能抖；大一点更平滑，太大可能切弯。
- `finish_tolerance`：终点距离容差。
- `kp_x/kp_y/kp_yaw`：位置和 yaw 比例增益。
- `kd_x/kd_y/kd_yaw`：误差微分增益。
- `max_vx/max_vy/max_dyaw`：速度限幅。

推荐初值：

```yaml
lookahead_distance: 0.35
finish_tolerance: 0.25
```

如果绕杆切角明显，可以先把：

```yaml
lookahead_distance: 0.25
```

如果轨迹抖动或定位噪声明显，可以尝试：

```yaml
lookahead_distance: 0.45
```

## 7. 倒车轨迹调试

倒车不是把轨迹文件倒过来走。当前定义是：

```text
轨迹顺序仍然从起点到终点
车头朝向反过来
控制器自然输出负 linear.x
```

配置：

```yaml
drive_mode: "reverse"
```

可视化检查：

```bash
python3 src/pythontool/plot_trajectory.py \
  src/commands/task_manager/traj/pole2.txt \
  --drive-mode reverse
```

实车检查：

```bash
ros2 topic echo /quad/cmd_vel
```

重点看：

```text
linear.x 是否主要为负
angular.z 是否没有长期打满
```

## 8. 现场调试顺序

推荐流程：

1. 录制轨迹。
2. 用 `plot_trajectory.py` 看轨迹形状、起终点、车头朝向。
3. 用 `trim_trajectory.py --dry-run` 检查首尾停留段。
4. 必要时裁剪轨迹。
5. 在 `obstacle_race_params.yaml` 中指向新轨迹。
6. 设置 `drive_mode`、`lookahead_distance`、`finish_tolerance`。
7. 启动障碍赛或单项调试。
8. 观察日志里的 `POLE_LOOKAHEAD`。

日志示例字段：

```text
s=当前路径进度
target=前瞻点路径进度
mode=forward/reverse
pos=当前雷达位姿
ref=参考点位姿
err_body=机体系误差
cmd=输出速度
```

## 9. 常见问题

### 轨迹跟得太靠近、抖动

优先增大：

```yaml
lookahead_distance
```

例如从 `0.35` 调到 `0.45`。

### 弯道切角太明显

优先减小：

```yaml
lookahead_distance
```

例如从 `0.35` 调到 `0.25`。

### 到终点不结束

检查：

- 当前位置是否真的接近轨迹最后一个点。
- `finish_tolerance` 是否太小。
- 轨迹终点是否录在了机器人实际能到的位置。

可以把：

```yaml
finish_tolerance: 0.35
```

临时放宽验证。

### 倒车时方向不对

检查：

- `drive_mode` 是否为 `"reverse"`。
- 可视化图里的箭头是否是期望车头方向。
- `cmd_vel.linear.x` 是否主要为负。

如果 `linear.x` 长期为正，通常说明参考点仍在当前机体前方，或轨迹 yaw/坐标系关系需要重新确认。

### yaw 看起来差 90 度

这是正常现象。轨迹文件里的 yaw 是雷达 yaw，车头朝向是它的法向：

```text
body_heading = lidar_yaw + pi/2
```

不要直接把轨迹文件 yaw 当车头 yaw 看。

## 10. 快速命令集合

录制：

```bash
python3 src/pythontool/record_trajectory.py \
  -o src/commands/task_manager/traj/pole2.txt \
  --duration 20 \
  --interval 0.1
```

裁剪 dry-run：

```bash
python3 src/pythontool/trim_trajectory.py \
  -i src/commands/task_manager/traj/pole2.txt \
  --dry-run
```

可视化 forward：

```bash
python3 src/pythontool/plot_trajectory.py \
  src/commands/task_manager/traj/pole2.txt
```

可视化 reverse：

```bash
python3 src/pythontool/plot_trajectory.py \
  src/commands/task_manager/traj/pole2.txt \
  --drive-mode reverse
```

编译任务管理包：

```bash
colcon build --packages-select task_manager
```
