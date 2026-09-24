# Task Manager 全流程测试指南

本文档用于真机或半实物联调 `task_manager`：包含启动方式、手柄按键、状态机变化、二维码触发和各障碍任务测试方法。

## 1. 启动

在工作区根目录：

```bash
cd ~/hitcrt_quad_ws
colcon build --packages-select task_manager aruco joystick_input state_machine
source install/setup.bash
ros2 launch quad run.launch.py
```

当前主配置文件：

```text
src/config/quad_run_cfg.yaml
```

当前绕杆轨迹配置项：

```yaml
/quad/task_executor_node:
  ros__parameters:
    tasks:
      pole_trajectory_file: "/home/cat/hitcrt_quad_ws/src/commands/task_manager/config/recorded_path.txt"
```

绕杆轨迹格式必须是：

```txt
time_sec x y yaw
```

## 2. 推荐监控话题

另开终端：

```bash
source install/setup.bash
ros2 topic echo /quad/state_array
ros2 topic echo /quad/current_state
ros2 topic echo /quad/task_state_command
ros2 topic echo /quad/task_done
ros2 topic echo /quad/nav_enable
ros2 topic echo /quad/nav_status
ros2 topic echo /quad/cmd_vel
ros2 topic echo /quad/qr_detection_result
```

## 3. 手柄按键

| 按键 | 作用 |
| --- | --- |
| `LT + START` | ENABLE，`STOP -> PASSIVE` |
| `RT + START` | DISABLE |
| `LB + A` | 站立/趴下切换，`PASSIVE -> FIXED_DOWN -> FIXED_STAND` |
| `LB + B` | DAMPING/急停 |
| `LB + X` | 进入 RL，`FIXED_STAND -> RL_MOVE` |
| `LT + A` | 进入 AUTO，`MANUAL -> AUTO` |
| `RT + A` | 回到 MANUAL，`AUTO -> MANUAL` |
| `LT + Y` | AUTO 模式下进入 QR |
| `RB + A` | STRIDE，沙坑动作 |
| `RB + Y` | JUMP，高墙动作 |
| `RB + HAT_UP` | 切 TROT 策略 |
| `RB + HAT_DOWN` | 切 CREEP 策略 |
| `RB + HAT_LEFT` | 切 UPSTAIR 策略 |

注意：`LT + Y` 只有在 `AUTO` 模式下才会让状态机发布 `CMD_QR_RECOGNITION`。

## 4. 手柄进入全流程

推荐顺序：

```text
LT + START
  -> state_array: PASSIVE

LB + A
  -> FIXED_DOWN

LB + A
  -> FIXED_STAND

LB + X
  -> RL_MOVE

LT + A
  -> MANUAL -> AUTO
  -> task_manager: AUTO_NAV

等待 obstacle_nav_node 到点
  -> nav_status=true
  -> task_manager: QR_RECOGNITION

二维码识别并对准
  -> task_done(task_id)
  -> task_manager: TASK_*

障碍完成
  -> task_done(-1)
  -> task_manager: AUTO_NAV
```

如果不想等自动导航到点，也可以在 AUTO 下按：

```text
LT + Y
```

## 5. 状态含义

`/quad/state_array`：

```text
data[0]: 控制模式
  0 = MANUAL
  1 = AUTO

data[1]: 底层状态
  0 = STOP
  1 = PASSIVE
  2 = FIXED_DOWN
  3 = FIXED_STAND
  5 = RL_MOVE
  6 = JUMP
  7 = STRIDE
  9 = KNEEL_CRAWL

data[2]: 步态策略
  0 = TROT
  1 = CREEP
  2 = UPSTAIR
```

`/quad/current_state`：

```text
IDLE
AUTO_NAV
QR_RECOGNITION
TASK_CRAWL_CROSS
TASK_CRAWL_MOVING
TASK_BRIDGE_CROSS
TASK_WALL_CROSS
TASK_SAND_INOUT
TASK_STAIR_UP
TASK_STAIR_MOVING
TASK_POLE_AROUND
```

## 6. 二维码任务 ID

`aruco_processor_node` 中：

```cpp
task_type = marker_id - 18;
```

| marker_id | task_id | 任务 |
| --- | --- | --- |
| 19 | 1 | 矮杆 |
| 20 | 2 | 木桥 |
| 21 | 3 | 高墙 |
| 22 | 4 | 沙坑 |
| 23 | 5 | 矮杆 |
| 24 | 6 | 绕杆 |

视觉伺服对准后，`task_executor_node` 发布：

```text
/quad/task_done: task_id
```

## 7. 各障碍流程

### 7.1 矮杆 task_id=1

```text
QR_RECOGNITION
 -> TASK_CRAWL_CROSS
 -> 切 POLICY_CREEP
 -> TASK_CRAWL_MOVING
 -> 定速匍匐
 -> task_done(-1)
 -> 切 POLICY_TROT
 -> AUTO_NAV
```

### 7.2 木桥 task_id=2

```text
QR_RECOGNITION
 -> TASK_BRIDGE_CROSS
 -> ENTER_RL
 -> /tf 直线闭环
 -> task_done(-1)
 -> AUTO_NAV
```

### 7.3 高墙 task_id=3

```text
QR_RECOGNITION
 -> TASK_WALL_CROSS
 -> JUMP 动作序列
 -> AUTO_NAV
```

### 7.4 沙坑 task_id=4

```text
QR_RECOGNITION
 -> TASK_SAND_INOUT
 -> STRIDE 动作序列
 -> AUTO_NAV
```

### 7.5 矮杆 task_id=5

```text
QR_RECOGNITION
 -> TASK_CRAWL_CROSS
 -> 切 POLICY_CREEP
 -> TASK_CRAWL_MOVING
 -> 定速匍匐
 -> task_done(-1)
 -> 切 POLICY_TROT
 -> AUTO_NAV
```

### 7.6 绕杆 task_id=6

```text
QR_RECOGNITION
 -> TASK_POLE_AROUND
 -> ENTER_RL
 -> 读取 recorded_path.txt
 -> 按 time_sec x y yaw 时间轴回放
 -> task_done(-1)
 -> AUTO_NAV
```

## 8. 不跑自动导航时单测障碍

先直接进入 QR：

```bash
ros2 topic pub --once /quad/task_state_command quad/msg/StateCommand "{command_id: 3}"
```

再模拟二维码识别结果：

```bash
ros2 topic pub --once /quad/task_done std_msgs/msg/Int32 "{data: 1}"  # 矮杆
ros2 topic pub --once /quad/task_done std_msgs/msg/Int32 "{data: 2}"  # 木桥
ros2 topic pub --once /quad/task_done std_msgs/msg/Int32 "{data: 3}"  # 高墙
ros2 topic pub --once /quad/task_done std_msgs/msg/Int32 "{data: 4}"  # 沙坑
ros2 topic pub --once /quad/task_done std_msgs/msg/Int32 "{data: 5}"  # 矮杆
ros2 topic pub --once /quad/task_done std_msgs/msg/Int32 "{data: 6}"  # 绕杆
```

观察：

```bash
ros2 topic echo /quad/current_state
ros2 topic echo /quad/cmd_evt
ros2 topic echo /quad/cmd_vel
ros2 topic echo /quad/task_done
```

## 9. 推荐联调顺序

```text
1. 测手柄和底层状态
2. 测 AUTO/QR 入口
3. 测 aruco 二维码识别
4. 测 QR 视觉伺服
5. 手动发布 task_done 单测各障碍
6. 最后跑 AUTO_NAV -> QR -> TASK_* 全流程
```

## 10. 常见问题

`LT + Y` 没进入 QR：

```text
确认已经 LT + A 进入 AUTO。
/quad/state_array 的 data[0] 应该是 1。
```

二维码识别有结果但不发布 `task_done`：

```text
说明还未对准到阈值。
检查 /quad/cmd_vel 是否在视觉伺服。
检查 visual_servoing.default_offset 和 task_*_offset。
```

绕杆不动：

```text
检查 /quad/current_state 是否为 TASK_POLE_AROUND。
检查 /tf 是否有 camera_init -> aft_mapped。
检查 pole_trajectory_file 是否存在，且文件为四列 time_sec x y yaw。
```

绕杆跟不上：

```text
降低录制轨迹速度。
增大 pole_max_vx / pole_max_vy / pole_max_dyaw。
微调 pole_kp_x / pole_kp_y / pole_kp_yaw。
必要时加入 pole_kd_*。
```
