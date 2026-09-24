# 障碍赛代码导读与修改说明

本文档聚焦当前仓库里的障碍赛链路：`task_manager` 是主入口，`lidar_nav_demo_cpp` 中只有可复用的录制轨迹回放思路，不是当前障碍赛的主状态机。

## 1. 先看结论

障碍赛主流程由三个节点组成：

| 节点 | 代码 | 作用 |
| --- | --- | --- |
| `task_manager_node` | `src/commands/task_manager/src/task_state_machine.cpp` | 上层状态机：切 `AUTO_NAV`、`QR_RECOGNITION`、各障碍任务，并给底层发状态事件。 |
| `obstacle_nav_node` | `src/commands/task_manager/src/obstacle_nav_node.cpp` | 障碍点自动导航：只在 `AUTO_NAV` 且 `nav_enable=true` 时发布 `cmd_vel`。 |
| `task_executor_node` | `src/commands/task_manager/src/task_executor_node.cpp` | 执行器：二维码视觉伺服、木桥直线、上楼梯、矮杆匍匐、绕杆轨迹回放。 |

整机启动文件：

```bash
ros2 launch /home/cat/hitcrt_quad_ws/src/launch/run.launch.py
```

整机参数入口：

```text
/home/cat/hitcrt_quad_ws/src/config/quad_run_cfg.yaml
```

注意：当前 `quad_run_cfg.yaml` 里 `task_manager_node.mission_mode` 是 `logistics`。跑障碍赛时应改为：

```yaml
/quad/task_manager_node:
  ros__parameters:
    mission_mode: "obstacle"
```

否则 `task_manager_node` 会走任务赛取放物块流程，`obstacle_nav_node` 虽然被 `run.launch.py` 启动，也不会按障碍赛链路工作。

## 2. 一次完整障碍赛怎么流转

核心握手如下：

```text
手柄/外部命令让 task_manager_node 进入 AUTO_NAV
  -> task_manager_node 发布 /quad/nav_enable=true
  -> obstacle_nav_node 跟踪当前障碍点，发布 /quad/cmd_vel
  -> 到达障碍点停车，发布 /quad/nav_status=true
  -> task_manager_node 切到 QR_RECOGNITION，并发布 /quad/nav_enable=false
  -> task_executor_node 做二维码视觉伺服
  -> 对准后发布 /quad/task_done(task_id)
  -> task_manager_node 根据 task_id 切到对应障碍任务
  -> 执行器或状态机完成障碍后发布/触发 task_done(-1)
  -> task_manager_node 回到 AUTO_NAV，进入下一个障碍点
```
y
最后一个障碍点导航完后：

```text
obstacle_nav_node 发布 /quad/nav_finished=true
task_manager_node 在 AUTO_NAV 收到 nav_finished
task_manager_node 切回 IDLE
```

## 3. 代码导读

### 3.1 状态机：`task_state_machine.cpp`

重点看这些函数：

| 函数 | 作用 |
| --- | --- |
| `publish_state_and_nav_enable()` | 每 20 ms 发布当前状态；只有 `current_state_ == AUTO_NAV` 时 `/quad/nav_enable=true`。 |
| `state_cmd_callback()` | 接收外部 `task_state_command`，`CMD_AUTO_NAV` 在障碍赛模式下进入 `AUTO_NAV`。 |
| `nav_status_callback()` | `AUTO_NAV` 中收到 `nav_status=true` 后切 `QR_RECOGNITION`。 |
| `task_done_callback()` | `QR_RECOGNITION` 中收到正数 `task_id` 后分派障碍任务；收到 `-1` 表示某段任务完成。 |
| `switch_state_from_task_id()` | 二维码任务 ID 到具体障碍状态的映射。 |
| `dispatch_current_state()` | 按当前状态调用各 `handle_*`。 |

当前代码里的任务 ID 映射以 `switch_state_from_task_id()` 为准：

| Task ID | 状态 | 执行方式 |
| --- | --- | --- |
| `1` | `TASK_STAIR_UP` / `TASK_STAIR_MOVING` | 切上楼梯策略，然后执行器雷达直线闭环。 |
| `2` | `TASK_BRIDGE_CROSS` | 执行器雷达直线过木桥。 |
| `3` | `TASK_WALL_CROSS` | 状态机发 `JUMP` 动作序列。 |
| `4` | `TASK_SAND_INOUT` | 状态机发 `STRIDE` 动作序列。 |
| `5` | `TASK_CRAWL_CROSS` / `TASK_CRAWL_MOVING` | 切匍匐策略，然后执行器低姿态直线前进。 |
| `6` | `TASK_POLE_AROUND` | 执行器读取录制轨迹并按时间轴绕杆。 |

如果要改任务 ID，对应改 `switch_state_from_task_id()`；如果视觉节点的二维码 ID 到 `task_type` 的换算也变了，还要同步检查发布 `qr_detection_result` 的节点。

### 3.2 障碍点导航：`obstacle_nav_node.cpp`

这个节点只负责“从一个障碍前停车点走到下一个障碍前停车点”，不负责识别二维码和具体过障碍。

重点看这些函数：

| 函数 | 作用 |
| --- | --- |
| `load_parameters()` | 读取导航模式、目标点、PID、限速、容差。 |
| `load_point_sequence()` | 把 `target_points` 三元组解析成目标点列表。 |
| `load_trajectory_file()` | 读取轨迹文件，格式是 `x y yaw`，没有时间列。 |
| `nav_enable_callback()` | 处理 `nav_enable` 上升沿/下降沿：上升沿加载当前目标，下降沿停车等待。 |
| `control_loop()` | 导航主循环：等 TF、检查越界、算误差、到点或发速度。 |
| `compute_waypoint_errors()` | 将雷达世界系误差转换到机体系误差。 |
| `compute_pd_velocity_cmd()` | PD 控制输出 `cmd_vel`；yaw 误差大时先只转向。 |
| `publish_nav_status_once()` | 当前障碍目标完成后只发一次 `nav_status=true`。 |
| `publish_nav_finished_once()` | 全部障碍目标完成后只发一次 `nav_finished=true`。 |

支持两种导航模式：

```yaml
navigation_mode: "POINT_SEQUENCE"
target_points: [
  -0.17, 1.34, -0.006,
  1.17, 2.54, 3.06
]
```

`POINT_SEQUENCE` 适合每个障碍前只有一个停车点。

```yaml
navigation_mode: "TRAJECTORY_SEQUENCE"
trajectory_files:
  - "/absolute/path/to/obstacle_1_nav.txt"
  - "/absolute/path/to/obstacle_2_nav.txt"
```

`TRAJECTORY_SEQUENCE` 适合一个障碍前要沿多个点进入。文件格式：

```text
# x y yaw
0.0 0.5 0.0
0.0 1.0 0.0
0.1 1.5 0.1
```

这里的轨迹是“逐点导航”，不是按时间回放；每个文件走到最后一个点才发布 `nav_status=true`。

### 3.3 任务执行器：`task_executor_node.cpp`

执行器监听 `/quad/current_state`，根据状态进入不同 `ExecutorMode`。

重点看：

| 函数 | 作用 |
| --- | --- |
| `apply_executor_mode_from_state_string()` | 状态字符串到执行器模式的映射。 |
| `control_loop()` | 分发到视觉、匍匐、木桥、楼梯、绕杆。 |
| `execute_visual_servoing()` | 二维码视觉伺服，对准后发 `task_done(task_id)`。 |
| `execute_bridge_cross()` | 木桥雷达直线闭环，完成后发 `task_done(-1)`。 |
| `execute_lidar_crawl()` | 矮杆匍匐直线，完成后发 `task_done(-1)`。 |
| `execute_stair_up()` | 上楼梯直线闭环，完成后发 `task_done(-1)`。 |
| `execute_pole_around()` | 绕杆按 `time x y yaw` 录制轨迹回放，完成后发 `task_done(-1)`。 |

视觉伺服使用 `quad/msg/QrResult`：

```text
qr.x, qr.y, qr.yaw, qr.task_type
```

代码把视觉误差转到机体系：

```cpp
err_x_m = err_qr_y_mm / 1000.0;
err_y_m = -err_qr_x_mm / 1000.0;
err_yaw_rad = visual_yaw_error_sign * err_qr_yaw_deg * pi / 180.0;
```

如果视觉左右、前后或旋转方向反了，优先改 `quad_run_cfg.yaml` 里的 `visual_servoing.yaw_error_sign` 和各 `task_N_offset`；如果相机坐标定义根本不同，再改 `compute_visual_body_errors()`。

## 4. 调参入口

整机比赛优先改：

```text
/home/cat/hitcrt_quad_ws/src/config/quad_run_cfg.yaml
```

包内示例参数：

```text
src/commands/task_manager/config/obstacle_nav_params.yaml
src/commands/task_manager/config/task_executor_params.yaml
```

这些更适合单独 `ros2 run` 某个节点时使用。`run.launch.py` 不读取它们，而是读取 `src/config/quad_run_cfg.yaml`。

### 4.1 障碍点导航参数

位置：

```yaml
/quad/obstacle_nav_node:
  ros__parameters:
```

常改参数：

| 参数 | 什么时候改 |
| --- | --- |
| `navigation_mode` | 单停车点用 `POINT_SEQUENCE`；多点进入用 `TRAJECTORY_SEQUENCE`。 |
| `target_points` | 改每个障碍前停车点坐标。 |
| `trajectory_files` | 改每个障碍前多点路径文件。 |
| `position_tolerance` / `yaw_tolerance` | 到点太难就放宽；停车太粗就收紧。 |
| `kp_x` / `kp_y` / `kp_yaw` | 误差收敛慢就加；震荡就减。 |
| `max_vx` / `max_vy` / `max_dyaw` | 初调先小，稳定后再放开。 |
| `yaw_first_threshold` | 朝向偏差超过该值时先原地转。 |

### 4.2 视觉伺服参数

位置：

```yaml
/quad/task_executor_node:
  ros__parameters:
    visual_servoing:
```

每个任务可以单独配置期望二维码偏置：

```yaml
task_2_offset: [416.29, 1032.14, 10.0]
```

格式是：

```text
[x_mm, y_mm, yaw_deg]
```

含义是“对准完成时，二维码应该出现在相机坐标里的目标位置”。现场调试时看：

```bash
ros2 topic echo /quad/qr_detection_result
ros2 topic echo /quad/visual_servoing/debug_errors
ros2 topic echo /quad/cmd_vel
```

### 4.3 各障碍任务参数

位置：

```yaml
/quad/task_executor_node:
  ros__parameters:
    tasks:
```

常见修改：

| 任务 | 参数 |
| --- | --- |
| 上楼梯 | `stair_forward_speed`、`stair_kp_y`、`stair_kp_yaw`、`stair_max_distance` |
| 矮杆 | `crawl_forward_speed`、`crawl_kp_y`、`crawl_kp_yaw`、`crawl_target_dist` |
| 绕杆 | `pole_trajectory_file`、`pole_kp_*`、`pole_max_*`、`pole_lidar_offset_*` |
| 木桥 | `straight_target_dist`、`straight_max_speed`、`straight_kp_y`、`kp_xy`、`kp_yaw` |
| 高墙/沙坑 | `/quad/task_manager_node.task_timers` 中的 `prep/action/buffer/recover` |

## 5. 使用步骤

1. 确认 `quad_run_cfg.yaml`：

```yaml
/quad/task_manager_node:
  ros__parameters:
    mission_mode: "obstacle"
```

2. 配置 `/quad/obstacle_nav_node.target_points`，先只放 1 个近距离点做验证。

3. 编译：

```bash
colcon build --packages-select task_manager
source install/setup.bash
```

4. 启动整机：

```bash
ros2 launch /home/cat/hitcrt_quad_ws/src/launch/run.launch.py
```

5. 进入自动模式后观察：

```bash
ros2 topic echo /quad/current_state
ros2 topic echo /quad/nav_enable
ros2 topic echo /quad/obstacle_nav/current_pose
ros2 topic echo /quad/obstacle_nav/current_target
ros2 topic echo /quad/obstacle_nav/debug_error
ros2 topic echo /quad/nav_status
```

6. 到点后调视觉伺服：

```bash
ros2 topic echo /quad/qr_detection_result
ros2 topic echo /quad/visual_servoing/debug_errors
ros2 topic echo /quad/task_done
```

7. 确认 `task_done(task_id)` 后进入对应障碍任务，任务完成后应回到 `AUTO_NAV`。

## 6. 单节点调试

只调障碍点导航：

```bash
ros2 run task_manager obstacle_nav_node \
  --ros-args --remap __ns:=/quad \
  --params-file /home/cat/hitcrt_quad_ws/src/commands/task_manager/config/obstacle_nav_params.yaml
```

手动给导航使能：

```bash
ros2 topic pub /quad/nav_enable std_msgs/msg/Bool "{data: true}" --once
```

只调执行器时，需要有 `/quad/current_state` 和对应输入。例如测试木桥：

```bash
ros2 topic pub /quad/current_state std_msgs/msg/String "{data: TASK_BRIDGE_CROSS}" --once
```

测试二维码入口：

```bash
ros2 topic pub /quad/current_state std_msgs/msg/String "{data: QR_RECOGNITION}" --once
```

## 7. `lidar_nav_demo_cpp` 和障碍赛的关系

`lidar_nav_demo_cpp` 当前主要服务任务赛取放物块。它的 `lidar_nav_demo_node` 有三种模式：

| 模式 | 参数 | 是否障碍赛主链路 |
| --- | --- | --- |
| 任务赛物流导航 | `auto_parku_enable=false` 且 `straight_line_enable=false` | 否，任务赛用。 |
| 直线模式 | `straight_line_enable=true` | 可用于底盘/雷达直线调试，不是障碍赛主入口。 |
| `AUTO_PARKU` 轨迹回放 | `auto_parku_enable=true` | 可参考轨迹回放实现，不是当前障碍赛状态机。 |

可参考代码：

| 文件 | 可参考点 |
| --- | --- |
| `src/commands/lidar_nav_demo_cpp/src/lidar_nav_auto_parku.cpp` | `load_recorded_trajectory()`、`sample_recorded_trajectory()`、`control_recorded_trajectory_replay()`。 |
| `src/commands/lidar_nav_demo_cpp/config/trajectories/example_path.txt` | `time_sec x y yaw` 轨迹格式示例。 |
| `src/commands/lidar_nav_demo_cpp/config/trajectories/recorded_path.txt` | 当前 `quad_run_cfg.yaml` 中绕杆 `pole_trajectory_file` 指向的轨迹文件。 |

关键区别：

| 用途 | 文件格式 | 执行方式 |
| --- | --- | --- |
| `obstacle_nav_node` 的 `TRAJECTORY_SEQUENCE` | `x y yaw` | 逐点到达，每个点都做位置/yaw 到点判断。 |
| `task_executor_node` 绕杆 | `time_sec x y yaw` | 按时间轴插值回放，PD 纠偏。 |
| `lidar_nav_demo_cpp AUTO_PARKU` | `time_sec x y yaw` | 按时间轴插值回放，结束后发 `nav_status=true`。 |

所以，如果要改“障碍赛绕杆”，优先改 `task_executor_node` 的 `execute_pole_around()` 和 `quad_run_cfg.yaml` 的 `tasks.pole_*`；如果只是想借鉴更完整的录轨迹回放逻辑，可以参考 `lidar_nav_auto_parku.cpp`。

## 8. 修改指南

### 8.1 新增一个障碍任务

1. 在 `task_state_machine.hpp` 的 `State` 里新增状态。
2. 在 `task_state_machine.cpp` 的 `state_to_string()`、`dispatch_current_state()` 里补分支。
3. 在 `switch_state_from_task_id()` 里把新的 `task_id` 映射到新状态。
4. 如果任务需要连续速度控制，在 `task_executor_node.hpp` 的 `ExecutorMode` 新增模式。
5. 在 `apply_executor_mode_from_state_string()` 里把状态字符串映射到执行器模式。
6. 在 `control_loop()` 里调用新的 `execute_*()`。
7. 任务结束时发布 `finish_task(-1)`，状态机就会回到 `nav_state`。
8. 在 `quad_run_cfg.yaml` 增加对应参数。

### 8.2 改障碍点顺序

只改 `/quad/obstacle_nav_node.target_points` 的顺序即可。每三个数是一组：

```text
x, y, yaw
```

`obstacle_nav_node` 内部用 `current_target_index_` 顺序递增；每到一个障碍点，就发 `nav_status=true`，等待状态机执行二维码和障碍任务。

### 8.3 把障碍点从单点改成多点路径

改为：

```yaml
navigation_mode: "TRAJECTORY_SEQUENCE"
trajectory_files:
  - "/home/cat/hitcrt_quad_ws/path/to/obstacle_1.txt"
```

轨迹文件每行：

```text
x y yaw
```

如果你手上的文件是 `time_sec x y yaw`，不能直接给 `obstacle_nav_node` 用；要么去掉时间列，要么改 `load_trajectory_file()` 支持四列并忽略第一列。

### 8.4 改绕杆轨迹

修改：

```yaml
/quad/task_executor_node:
  ros__parameters:
    tasks:
      pole_trajectory_file: "/absolute/path/to/pole_path.txt"
```

文件格式：

```text
time_sec x y yaw
```

要求：

- 至少两行有效点。
- 时间严格递增。
- 坐标是雷达世界系 `camera_init` 下的绝对位姿。

### 8.5 改导航控制方向

如果自动导航前后/左右方向反，先看 `obstacle_nav_node.cpp` 的 `compute_waypoint_errors()`：

```cpp
err_x_body = cos(yaw) * dy_world - sin(yaw) * dx_world;
err_y_body = -cos(yaw) * dx_world - sin(yaw) * dy_world;
```

如果只是速度太猛，不改代码，先调 `max_vx/max_vy/max_dyaw` 和 `kp_*`。如果坐标系定义变了，再改这里和 `task_executor_node` 中相同用途的误差变换。

## 9. 常见坑

| 现象 | 优先检查 |
| --- | --- |
| 进入自动后车不动 | `mission_mode` 是否是 `obstacle`；`/quad/nav_enable` 是否为 true；`/tf` 是否有 `camera_init -> aft_mapped`。 |
| 到点后不切二维码 | `/quad/nav_status` 是否发布；`task_manager_node` 是否处在 `AUTO_NAV`。 |
| 二维码对准后不进任务 | `/quad/task_done` 是否发正数 task_id；`switch_state_from_task_id()` 是否有该 ID。 |
| 任务完成后不回导航 | 执行器是否发布 `task_done(-1)`；状态机当前是否在对应任务状态。 |
| 最后不结束 | `/quad/nav_finished` 是否发布；状态机是否还在 `AUTO_NAV`。 |
| 绕杆不动 | `tasks.pole_trajectory_file` 是否为空、文件是否存在、时间列是否递增。 |
| 雷达突然停车并报“雷达飘了” | `lidar_pose_guard` 检测到 x/y 越界，先查定位是否漂移或坐标原点是否错。 |

