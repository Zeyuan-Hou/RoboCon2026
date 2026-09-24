# 二维码视觉伺服调试指南

本文说明如何使用 `qr_servo_debug.launch.py` 单独调试二维码视觉伺服参数。该调试入口尽量复用障碍赛全流程的节点和参数表，只是不启动雷达自动导航节点，避免自动导航抢占 `/quad/cmd_vel`。

## 1. 适用场景

使用本 launch 时，适合调试：

- `visual_servoing.kp_x / kp_y / kp_yaw`
- `visual_servoing.max_vx / max_vy / max_wz`
- `visual_servoing.tolerance_x_m / tolerance_y_m / tolerance_yaw_rad`
- `visual_servoing.default_offset`
- `visual_servoing.lateral_cmd_sign`
- `visual_servoing.yaw_error_sign`
- `obstacle_sequence.<slot>.visual_offset`

调好的参数仍写在障碍赛专用配置文件：

```text
src/commands/task_manager/config/obstacle_race_params.yaml
```

之后启动完整障碍赛：

```bash
ros2 launch quad obstacle_race.launch.py
```

这些视觉伺服参数会直接生效，不需要迁移。

## 2. 调试 launch 的特点

启动文件：

```text
src/launch/qr_servo_debug.launch.py
```

它会启动与障碍赛一致的核心节点：

- `aruco_processor_node`
- `task_manager_node`
- `task_executor_node`
- `joystick_input_node`
- `joystick_handler_node`
- `state_machine`
- `pos_control_node`
- `rl_control_node`
- `rknn_quad_node`
- `unitree_motor_node`
- `hipnuc_imu`

它不会启动：

```text
obstacle_nav_node
```

因此不会有雷达自动导航节点发布 `/quad/cmd_vel`。

为了避免视觉伺服完成后进入木桥、高墙、绕杆等后续障碍任务，本 launch 只对 `task_manager_node` 做了局部 remap：

```text
task_done -> task_done_debug_ignored
```

也就是说：

- `task_executor_node` 仍正常发布 `/quad/task_done`
- 可以正常观察视觉伺服完成信号
- `task_manager_node` 不会消费这个完成信号
- 状态机不会继续进入后续障碍动作

完整障碍赛 `obstacle_race.launch.py` 没有这个 remap，因此完整流程不受影响。

## 3. 启动方式

如果改过 `task_executor_node.cpp/.hpp`，先编译再启动：

```bash
cd /home/cat/hitcrt_quad2026_ws
colcon build --packages-select task_manager
source install/setup.bash
ros2 launch quad qr_servo_debug.launch.py
```

如果只是改了 `src/commands/task_manager/config/obstacle_race_params.yaml` 里的参数，通常不需要重新编译，只需要重新启动 launch：

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch quad qr_servo_debug.launch.py
```

单独测试某个二维码时，要让 `obstacle_sequence.order` 第一个槽位就是当前二维码对应的槽位。例如测试木桥 A 到点伺服：

```yaml
obstacle_sequence:
  order: [bridge_a]
```

如果当前槽位仍是 `slope`，但手里拿的是 `bridge_a` 的二维码，`task_executor_node` 会因为 task_id 不匹配而停车。

## 4. 手柄流程

推荐按键顺序：

```text
LT + START  使能底层
LB + A      站立/起身
LB + X      进入 RL_MOVE
LT + A      进入 AUTO
LT + Y      进入 QR_RECOGNITION，启动二维码伺服
RT + A      回到手动，准备下一轮
```

说明：

- `LT + A` 让系统进入自动模式。
- `LT + Y` 在自动模式下触发二维码识别状态。
- 进入 `QR_RECOGNITION` 后，`task_executor_node` 会根据 `/quad/qr_detection_result` 计算误差并发布 `/quad/cmd_vel`。
- 对准完成后，`task_executor_node` 会停车并发布 `/quad/task_done`。
- 本调试 launch 中，`task_manager_node` 不会响应该 `/quad/task_done`，所以不会继续进入障碍任务。

如果底层已经安全使能，也可以用话题切到自动和二维码状态：

```bash
ros2 topic pub --once /quad/task_state_command quad/msg/StateCommand "{command_id: 1}"
ros2 topic pub --once /quad/task_state_command quad/msg/StateCommand "{command_id: 3}"
```

## 5. 推荐观察话题

另开终端，加载环境：

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
```

查看当前槽位：

```bash
ros2 topic echo /quad/obstacle_sequence/current_slot
```

查看当前任务状态：

```bash
ros2 topic echo /quad/current_state
```

期望进入伺服时看到：

```text
QR_RECOGNITION
```

查看二维码识别结果：

```bash
ros2 topic echo /quad/qr_detection_result
```

查看视觉伺服误差：

```bash
ros2 topic echo /quad/visual_servoing/debug_errors
```

其中：

```text
x: 前后误差，单位 m
y: 左右误差，单位 m
z: yaw 误差，单位 rad
```

查看速度输出：

```bash
ros2 topic echo /quad/cmd_vel
```

查看视觉伺服完成信号：

```bash
ros2 topic echo /quad/task_done
```

确认没有启动雷达自动导航节点：

```bash
ros2 node list | grep obstacle_nav
```

正常情况下不应看到 `obstacle_nav_node`。

## 6. 参数位置

二维码视觉伺服参数位于：

```text
src/commands/task_manager/config/obstacle_race_params.yaml
```

节点段：

```yaml
/quad/task_manager_node:
  ros__parameters:
    obstacle_sequence:
      slope:
        expected_task_id: 4
        visual_offset: [0.0, 1200.0, 0.0]

/quad/task_executor_node:
  ros__parameters:
    visual_servoing:
      default_offset: [0.0, 700.0, 0.0]
      tolerance_x_m: 0.1
      tolerance_y_m: 0.1
      tolerance_yaw_rad: 0.08
      kp_x: 0.7
      kp_y: 1.0
      kp_yaw: 0.45
      max_vx: 0.5
      max_vy: 0.5
      max_wz: 0.25
      lateral_cmd_sign: -1.0
      yaw_error_sign: 1.0
```

偏置格式：

```text
[x_mm, y_mm, yaw_deg]
```

含义：

- `x_mm`：二维码视觉系左右偏置，右为正
- `y_mm`：二维码视觉系前后距离，前为正
- `yaw_deg`：二维码视觉系 yaw，顺时针为正

## 7. 调参建议

先低速调试，建议从速度限幅开始保守：

```yaml
max_vx: 0.15
max_vy: 0.15
max_wz: 0.15
```

确认方向正确后，再逐步增大限幅和增益。

如果前后方向反了，重点检查：

```text
/quad/visual_servoing/debug_errors.x
/quad/cmd_vel.linear.x
```

如果左右方向反了，重点检查：

```text
/quad/visual_servoing/debug_errors.y
/quad/cmd_vel.linear.y
```

方向确认反了时，优先在下面两个值之间切换：

```yaml
lateral_cmd_sign: -1.0
```

或：

```yaml
lateral_cmd_sign: 1.0
```

如果 yaw 方向反了，优先调整：

```yaml
yaw_error_sign: -1.0
```

或：

```yaml
yaw_error_sign: 1.0
```

如果速度来回震荡：

- 降低对应方向的 `kp_*`
- 降低对应方向的 `max_*`
- 适当放宽对应方向的 `tolerance_*`

如果对准太慢：

- 先适当增大 `max_*`
- 再小幅增大 `kp_*`
- 不要一次同时大幅修改多个参数

## 8. 一轮调试流程

推荐每轮这样做：

1. 修改 `src/commands/task_manager/config/obstacle_race_params.yaml` 中的 `visual_servoing` 参数。
2. 重启 `qr_servo_debug.launch.py`。
3. 按手柄进入 `RL_MOVE`。
4. 按 `LT + A` 进入自动模式。
5. 按 `LT + Y` 进入 `QR_RECOGNITION`。
6. 观察 `/quad/visual_servoing/debug_errors` 是否收敛。
7. 观察 `/quad/cmd_vel` 是否平稳。
8. 对准完成后观察 `/quad/task_done`。
9. 按 `RT + A` 回到手动，准备下一轮。

## 9. 切回完整障碍赛

调参完成后，直接启动完整障碍赛：

```bash
cd ~/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch quad obstacle_race.launch.py
```

完整障碍赛会使用同一份：

```text
src/commands/task_manager/config/obstacle_race_params.yaml
```

此时 `task_manager_node` 会正常消费 `/quad/task_done`，视觉伺服完成后会进入对应障碍任务。
