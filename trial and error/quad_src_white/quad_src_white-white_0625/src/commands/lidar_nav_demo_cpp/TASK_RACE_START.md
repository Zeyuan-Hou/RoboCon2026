# 任务赛开始操作指南

本文档说明如何启动任务赛流程，以及开始任务赛所需的手柄操作。

## 1. 启动前配置检查

### 1.1 `task_manager` 模式

任务赛需要 `task_manager_node` 进入 `logistics` 模式。

如果使用任务赛专用 launch：

```bash
ros2 launch /home/cat/hitcrt_quad2026_ws/src/launch/task_race.launch.py
```

launch 会自动覆盖：

```yaml
mission_mode: "logistics"
```

如果手动分节点启动，需要在 `src/config/quad_run_cfg.yaml` 中确认：

```yaml
/quad/task_manager_node:
  ros__parameters:
    mission_mode: "logistics"
```

### 1.2 任务赛导航模式

确认 `src/commands/lidar_nav_demo_cpp/config/waypoints.yaml`：

```yaml
auto_parku_enable: false
straight_line_enable: false
external_manipulation_enable: true
enable_control: false
enable_topic: "/quad/logistics_nav_enable"
```

说明：

- `auto_parku_enable: false`：不走录制轨迹回放。
- `straight_line_enable: false`：不走直线调试模式。
- `external_manipulation_enable: true`：到取/放点后交给 `task_manager` 触发机械臂。
- `enable_control: false`：导航由 `task_manager` 使能。
- `enable_topic: "/quad/logistics_nav_enable"`：任务赛专用导航使能话题。

## 2. 启动任务赛系统

在工作区根目录执行：

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch /home/cat/hitcrt_quad2026_ws/src/launch/task_race.launch.py
```

该 launch 会启动：

- `task_manager_node`
- `lidar_nav_demo_node`
- 底层 `state_machine`
- 手柄输入节点
- 四足 RL / 位控 / 电机 / IMU 相关节点
- 机械臂 `arm_node` / `pump_node` / `manipulator_manager_node`

该 launch 不会启动障碍赛的 `obstacle_nav_node`，避免和任务赛导航同时发布 `/quad/cmd_vel`。

## 3. 手柄操作顺序

按键映射来自 `joystick_handler.py`。

### 3.1 使能底层

```text
LT + START
```

对应事件：

```text
ENABLE
```

### 3.2 站立

```text
LB + A
```

对应事件：

```text
UP_DOWN
```

用于让机器人从阻尼/趴下状态切到站立相关状态。

### 3.3 进入 RL 行走态

```text
LB + X
```

对应事件：

```text
ENTER_RL
```

进入自动任务赛前，底层需要处于：

```text
RL_MOVE = 5
```

否则 `ENTER_AUTO` 会被底层状态机拒绝。

### 3.4 进入任务赛自动模式

```text
LT + A
```

对应事件：

```text
ENTER_AUTO
```

如果 `mission_mode == logistics`，`task_manager` 会进入：

```text
LOGISTICS_NAV
```

并发布：

```text
/quad/logistics_nav_enable = true
```

随后 `lidar_nav_demo_node` 开始任务赛导航。

## 4. 任务赛运行过程

### 4.1 导航阶段

```text
LOGISTICS_NAV
```

此时：

- `task_manager` 发布 `/quad/logistics_nav_enable=true`
- `lidar_nav_demo_node` 根据生成的航点序列发布 `/quad/cmd_vel`

### 4.2 到达取货点

到达 `PICKUP` 点后：

```text
lidar_nav_demo_node 发布 PICKUP_REACHED
task_manager 切到 TASK_PICKUP_BLOCK
```

随后：

```text
RL_MOVE -> FIXED_STAND
发布 /manipulator/cmd = 1
等待 pickup_wait_s
FIXED_STAND -> RL_MOVE
回到 LOGISTICS_NAV
```

`/manipulator/cmd = 1` 表示：

```text
GRAB / PICKUP
```

### 4.3 到达放货点

到达 `DROPOFF` 点后：

```text
lidar_nav_demo_node 发布 DROPOFF_REACHED
task_manager 切到 TASK_DROPOFF_BLOCK
```

随后：

```text
RL_MOVE -> FIXED_STAND
发布 /manipulator/cmd = 2 或 3
等待 dropoff_wait_s
FIXED_STAND -> RL_MOVE
回到 LOGISTICS_NAV
```

放置命令：

```text
2 = PLACE_LOW
3 = PLACE_HIGH
```

同一个放置点的逻辑：

```text
第一次到达同一个放置点 -> PLACE_LOW
第二次及以后到达同一个放置点 -> PLACE_HIGH
```

## 5. 运行中建议观察的话题

### 5.1 上层任务状态

```bash
ros2 topic echo /quad/current_state
```

期望状态包括：

```text
LOGISTICS_NAV
TASK_PICKUP_BLOCK
TASK_DROPOFF_BLOCK
LOGISTICS_DONE
```

### 5.2 任务赛导航使能

```bash
ros2 topic echo /quad/logistics_nav_enable
```

含义：

```text
true  -> 允许 lidar_nav_demo_node 导航
false -> 正在取/放物块或任务暂停
```

### 5.3 到点事件

```bash
ros2 topic echo /quad/logistics_nav_event
```

消息格式：

```text
[event_type, waypoint_index, seq, action_id]
```

事件码：

```text
1 = PICKUP_REACHED
2 = DROPOFF_REACHED
3 = LOGISTICS_FINISHED
```

### 5.4 机械臂命令

```bash
ros2 topic echo /manipulator/cmd
```

命令值：

```text
1 = GRAB / PICKUP
2 = PLACE_LOW
3 = PLACE_HIGH
```

### 5.5 底层状态

```bash
ros2 topic echo /quad/state_array
```

格式：

```text
[control_mode, lower_state, policy]
```

常用值：

```text
control_mode: 0=MANUAL, 1=AUTO
lower_state: 3=FIXED_STAND, 5=RL_MOVE
policy: 0=TROT, 1=CREEP, 2=UPSTAIR
```

## 6. 退出自动 / 急停

### 6.1 切回手动

```text
RT + A
```

对应事件：

```text
ENTER_MANUAL
```

### 6.2 急停 / 阻尼

```text
LB + B
```

对应事件：

```text
DAMPING
```

## 7. 最小操作清单

```text
1. 启动 task_race.launch.py
2. 等待底层、雷达、机械臂节点正常
3. LT + START 使能底层
4. LB + A 站立
5. LB + X 进入 RL_MOVE
6. 等 OCR/eightboxes 或 fallback 路径生成
7. LT + A 进入任务赛自动模式
8. 观察 /quad/current_state 和 /manipulator/cmd
```
