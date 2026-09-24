# 任务赛取放物块流程改动总结

本文档总结本次为任务赛取放物块流程做的代码与配置改动。

## 目标

原先 `lidar_nav_demo_cpp` 到达取货点或放货点后，只会停车并等待 `wait_time`，没有真正触发机械臂，也没有让四足机器人从 RL 行走模式切到站立模式。

本次改动后，任务赛流程变为：

```text
雷达导航到取/放点
  ↓
通知 task_manager
  ↓
暂停任务赛导航
  ↓
底层从 RL_MOVE 切到 FIXED_STAND
  ↓
发布一次机械臂动作命令
  ↓
等待取/放动作完成
  ↓
底层切回 RL_MOVE
  ↓
恢复任务赛导航
```

## 主要改动

### 1. `lidar_nav_demo_cpp` 增加任务赛到点事件

新增任务赛事件话题：

```text
/quad/logistics_nav_event
std_msgs/Int32MultiArray
```

消息格式：

```text
[event_type, waypoint_index, seq, action_id]
```

`action_id` 在放置事件中表示物理放置点 ID。`dropoff_points` 中坐标和 yaw 相同的点会使用同一个 ID，用于判断是否需要高层叠放。

事件码：

```text
1 = PICKUP_REACHED
2 = DROPOFF_REACHED
3 = LOGISTICS_FINISHED
```

相关文件：

- `include/lidar_nav_demo_cpp/lidar_nav_demo_node.hpp`
- `src/lidar_nav_demo_node.cpp`
- `src/lidar_nav_path.cpp`
- `src/lidar_nav_control.cpp`

### 2. `lidar_nav_demo_cpp` 区分本地等待和外部机械臂接管

新增参数：

```yaml
external_manipulation_enable: true
```

当为 `true` 时：

- 到达 `PICKUP` / `DROPOFF` 点后发布 `/quad/logistics_nav_event`；
- 本节点停止继续导航；
- 等待 `task_manager` 重新发布导航使能后继续下一个航点。

当为 `false` 时：

- 保留旧逻辑，到点后本节点内部等待 `wait_time` 再继续。

### 3. 拆分任务赛导航使能话题

新增任务赛专用导航使能：

```text
/quad/logistics_nav_enable
std_msgs/Bool
```

原因：

- 障碍赛的 `obstacle_nav_node` 使用 `/quad/nav_enable`；
- 任务赛的 `lidar_nav_demo_node` 使用 `/quad/logistics_nav_enable`；
- 两套导航节点不会因为同一个使能话题同时向 `/quad/cmd_vel` 发速度。

### 4. `task_manager` 增加任务赛状态机

新增上层状态：

```text
LOGISTICS_NAV
TASK_PICKUP_BLOCK
TASK_DROPOFF_BLOCK
LOGISTICS_DONE
```

任务赛模式由参数控制：

```yaml
mission_mode: "logistics"
```

当 `mission_mode == "logistics"` 时，收到 `CMD_AUTO_NAV` 会进入 `LOGISTICS_NAV`，而不是障碍赛的 `AUTO_NAV`。

### 5. `task_manager` 接管取/放物块动作序列

新增取/放物块内部阶段：

```text
STOP_NAV
TO_STAND
STAND_SETTLE
TRIGGER_ARM
WAIT_ARM
BACK_TO_RL
RL_SETTLE
DONE
```

具体流程：

```text
TASK_PICKUP_BLOCK / TASK_DROPOFF_BLOCK
  ↓
发布 /quad/logistics_nav_enable=false
  ↓
发送 UP_DOWN，等待底层进入 FIXED_STAND
  ↓
发布一次 /manipulator/cmd
  ↓
等待 pickup_wait_s / dropoff_wait_s
  ↓
发送 ENTER_RL，等待底层进入 RL_MOVE
  ↓
回到 LOGISTICS_NAV
```

### 6. 机械臂接口

机械臂接口使用一个话题：

```text
/manipulator/cmd
std_msgs/UInt8
```

默认命令值：

```text
1 = GRAB / PICKUP
2 = PLACE_LOW
3 = PLACE_HIGH
```

`task_manager` 只发布一次命令。机械臂节点在自己的循环中读取该命令并执行。

放置命令选择逻辑：

```text
第一次到达某个物理放置点 -> 2 = PLACE_LOW
第二次及以后到达同一个放置点 -> 3 = PLACE_HIGH
```

### 7. 配置更新

`src/config/quad_run_cfg.yaml` 增加：

```yaml
/quad/task_manager_node:
  ros__parameters:
    mission_mode: "obstacle"

    manipulation:
      command_topic: "/manipulator/cmd"
      pickup_cmd: 1
      place_low_cmd: 2
      place_high_cmd: 3
      pickup_wait_s: 4.0
      dropoff_wait_s: 4.0
      stand_settle_s: 0.5
      rl_settle_s: 0.3
      lower_state_timeout_s: 3.0
```

默认仍保留 `mission_mode: "obstacle"`，避免影响障碍赛。任务赛调试时需要改成：

```yaml
mission_mode: "logistics"
```

`src/commands/lidar_nav_demo_cpp/config/waypoints.yaml` 更新：

```yaml
auto_parku_enable: false
straight_line_enable: false
external_manipulation_enable: true
enable_topic: "/quad/logistics_nav_enable"
```

### 8. 文档更新

新增任务赛调试指南：

```text
src/commands/lidar_nav_demo_cpp/TASK_RACE_DEBUG_GUIDE.md
```

更新任务赛 README：

```text
src/commands/lidar_nav_demo_cpp/README.md
```

## 调试时重点观察的话题

```bash
ros2 topic echo /quad/current_state
ros2 topic echo /quad/logistics_nav_enable
ros2 topic echo /quad/logistics_nav_event
ros2 topic echo /manipulator/cmd
ros2 topic echo /quad/state_array
```

期望现象：

```text
LOGISTICS_NAV
  ↓ 到取货点
TASK_PICKUP_BLOCK
  ↓ 发布 /manipulator/cmd = 1
LOGISTICS_NAV
  ↓ 到放货点
TASK_DROPOFF_BLOCK
  ↓ 发布 /manipulator/cmd = 2 或 3
LOGISTICS_NAV
```

## 验证结果

已通过编译：

```bash
colcon build --packages-select task_manager lidar_nav_demo_cpp
```

编译结果：

```text
2 packages finished
```
