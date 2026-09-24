# Launch 启动指南

本文说明实机调试时三个主要 launch 的用途、启动命令和适用场景。

## 启动前准备

每次新开终端先进入工作区并加载环境：

```bash
cd ~/hitcrt_quad2026_ws
source install/setup.bash
```

如果改过 launch、C++ 节点或消息接口，先重新编译：

```bash
colcon build --symlink-install
source install/setup.bash
```

基础实机参数主要在 `src/config/quad_run_cfg.yaml` 中配置。障碍赛专用参数在 `src/commands/task_manager/config/obstacle_race_params.yaml` 中配置。

## 1. 基础实机栈：run.launch.py

启动命令：

```bash
ros2 launch quad run.launch.py
```

适用情况：

- 测试四足基础控制链路。
- 测试手柄、状态机、RL、RKNN、Unitree 电机、IMU、ArUco。
- 不需要障碍赛自动导航，也不需要任务赛取放物块流程。

启动节点：

- `aruco_processor_node`
- `task_manager_node`
- `joystick_input_node`
- `joystick_handler_node`
- `state_machine`
- `pos_control_node`
- `rl_control_node`
- `rknn_quad_node`
- `unitree_motor_node`
- `hipnuc_imu/talker`

不包含：

- `obstacle_nav_node`
- `lidar_nav_demo_node`
- 机械臂相关节点

## 2. 障碍赛：obstacle_race.launch.py

启动命令：

```bash
ros2 launch quad obstacle_race.launch.py
```

适用情况：

- 测试障碍赛完整流程。
- 需要 `AUTO_NAV` 阶段由雷达位姿自动导航到障碍点。
- 不需要任务赛取放物块和机械臂。

关键行为：

- 启动 `obstacle_nav_node`。
- `task_manager_node` 会覆盖为 `mission_mode: obstacle`。
- `obstacle_nav_node` 订阅 `/quad/nav_enable`，在 `AUTO_NAV` 时发布 `/quad/cmd_vel`。
- 到达障碍点后发布 `/quad/nav_status`，全部点完成后发布 `/quad/nav_finished`。

自动导航开关：

```bash
ros2 launch quad obstacle_race.launch.py obstacle_nav_enable:=false
```

设置为 `false` 时不启动 `obstacle_nav_node`，可用于只测试障碍赛基础栈。

主要配置：

- `src/commands/task_manager/config/obstacle_race_params.yaml` 中的 `/quad/obstacle_nav_node`
- `src/commands/task_manager/config/obstacle_race_params.yaml` 中的 `/quad/task_executor_node.visual_servoing`
- 障碍点：`target_points`
- 导航模式：`navigation_mode`
- 到点阈值：`position_tolerance`、`yaw_tolerance`
- 控制参数：`kp_x`、`kp_y`、`kp_yaw`、`max_vx`、`max_vy`、`max_dyaw`

## 3. 任务赛：task_race.launch.py

启动命令：

```bash
ros2 launch quad task_race.launch.py
```

适用情况：

- 测试任务赛取放物块流程。
- 需要物流导航、取货点/放货点、机械臂、气泵联动。
- 不应同时启动 `obstacle_nav_node`，避免多个节点抢 `/quad/cmd_vel`。

关键行为：

- 启动 `lidar_nav_demo_node`。
- `task_manager_node` 会覆盖为 `mission_mode: logistics`。
- `task_manager_node` 通过 `/quad/logistics_nav_enable` 控制任务赛导航。
- `lidar_nav_demo_node` 到达取放点后发布 `/quad/logistics_nav_event`。
- 机械臂流程由 `manipulator_manager_node`、`arm_node_td`、`pump_node`、`3508_motor_node` 配合完成。

任务赛额外节点：

- `lidar_nav_demo_node`
- `3508_motor_node`
- `arm_node_td`
- `pump_node`
- `manipulator_manager_node`

主要配置：

- `src/commands/lidar_nav_demo_cpp/config/waypoints.yaml`
- `src/commands/lidar_nav_demo_cpp/config/prepath.yaml`
- `src/config/quad_run_cfg.yaml` 中的 `/quad/task_manager_node`
- `src/config/quad_run_cfg.yaml` 中的 `/quad/rknn_quad_node`

常用参数：

```bash
ros2 launch quad task_race.launch.py waypoints_file:=/absolute/path/to/waypoints.yaml
ros2 launch quad task_race.launch.py anchor_recorder_enable:=true
```

注意：`anchor_recorder_enable` 默认是 `false`。当前工作区没有 `src/pythontool/record_anchor_points.py` 时，不要打开该参数。

## 快速选择

| 目标 | 使用 launch |
| --- | --- |
| 基础实机控制、手柄、RL、RKNN、IMU | `run.launch.py` |
| 障碍赛自动导航 | `obstacle_race.launch.py` |
| 任务赛取放物块、机械臂联动 | `task_race.launch.py` |

## 常见检查命令

查看当前任务状态：

```bash
ros2 topic echo /quad/current_state
```

障碍赛导航使能：

```bash
ros2 topic echo /quad/nav_enable
```

任务赛导航使能：

```bash
ros2 topic echo /quad/logistics_nav_enable
```

查看速度输出：

```bash
ros2 topic echo /quad/cmd_vel
```

查看 RKNN 模型路径：

```bash
ros2 param get /quad/rknn_quad_node trot_model_path
```
