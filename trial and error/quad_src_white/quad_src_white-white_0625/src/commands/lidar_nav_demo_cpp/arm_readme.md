# Manipulator Manager Node

机械臂管理节点，基于时间的分段控制，协调 arm_node 和 pump_node 完成抓取/放置任务。

## 功能说明

- 实现三种任务流程：
  - **GRAB**: 抓取物块（移动到GRAB位置 → 气泵吸合 → 抬升到LIFT位置）
  - **PLACE_LOW**: 低层放置（移动到PLACE_LOW位置 → 气泵释放 → 抬升到LIFT位置）
  - **PLACE_HIGH**: 高层叠放（移动到PLACE_HIGH位置 → 气泵释放 → 抬升到LIFT位置）
- 状态推进基于时间（分段函数），不依赖设备反馈
- 任务完成后保持最后状态

## 使用流程

### 1. 启动节点

```bash
# 使用 launch 文件启动所有相关节点
ros2 launch manipulator_manager_node manipulator.launch.py

# 或单独启动
ros2 run manipulator_manager_node manipulator_manager_node
```

### 2. 前置条件（重要）

**在执行操纵任务前，必须先通过状态机切换到 FIXED_STAND 状态：**

```bash
# 发布 cmd_evt 命令，切换到 FIXED_STAND 状态
ros2 topic pub /cmd_evt std_msgs/msg/Int8 "data: 2" --once
```

cmd_evt 命令值：

- `FIXED_STAND - 固定站立（执行操纵任务前必须处于此状态）
- 现在已经改为有RL_MOVE直接

### 3. 执行操纵任务

```bash
# GRAB - 抓取物块
ros2 topic pub /manipulator/cmd std_msgs/msg/UInt8 "data: 1" --once

# PLACE_LOW - 低层放置
ros2 topic pub /manipulator/cmd std_msgs/msg/UInt8 "data: 2" --once

# PLACE_HIGH - 高层叠放
ros2 topic pub /manipulator/cmd std_msgs/msg/UInt8 "data: 3" --once
```

## 话题接口

### 订阅话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/manipulator/cmd` | `std_msgs/UInt8` | 顶层控制命令<br>1=GRAB, 2=PLACE_LOW, 3=PLACE_HIGH |

### 发布话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/quad/arm/state_cmd` | `std_msgs/UInt8` | 机械臂目标状态命令<br>0=NONE, 1=GRAB, 2=LIFT, 3=PLACE_LOW, 4=PLACE_HIGH |
| `/quad/pump_cmd` | `std_msgs/Int8` | 气泵控制命令<br>0=STOP, 1=GRAB, 2=RELEASE |
| `/manipulator/state` | `std_msgs/String` | 当前任务状态<br>IDLE/RUNNING/DONE/ERROR |
| `/manipulator/result` | `std_msgs/Bool` | 任务结果反馈<br>true=成功, false=失败 |

## 参数配置

### GRAB 任务时间参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `grab_arm_time_ms` | 1500 | 移动到GRAB位置时间(ms) |
| `grab_pump_time_ms` | 500 | 气泵吸合时间(ms) |
| `grab_lift_time_ms` | 1500 | 抬升到LIFT位置时间(ms) |

**GRAB 任务时序：**

```
0ms          1500ms       2000ms       3500ms
|-------------|------------|------------|
   移动GRAB     气泵吸合      移动LIFT
```

### PLACE_LOW 任务时间参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `place_low_arm_time_ms` | 2000 | 移动到PLACE_LOW位置时间(ms) |
| `place_low_release_time_ms` | 500 | 气泵释放时间(ms) |
| `place_low_lift_time_ms` | 2000 | 抬升到LIFT位置时间(ms) |

### PLACE_HIGH 任务时间参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `place_high_arm_time_ms` | 1500 | 移动到PLACE_HIGH位置时间(ms) |
| `place_high_release_time_ms` | 1000 | 气泵释放时间(ms) |
| `place_high_lift_time_ms` | 1500 | 抬升到LIFT位置时间(ms) |

### 控制参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `control_freq_hz` | 100.0 | 控制循环频率(Hz) |

## 完整使用示例

```bash
# 1. 启动所有节点
ros2 launch manipulator_manager_node manipulator.launch.py

# 2. 等待状态机就绪，切换到 FIXED_STAND
ros2 topic pub /cmd_evt std_msgs/msg/Int8 "data: 1" --once

# 3. 执行抓取任务
ros2 topic pub /manipulator/cmd std_msgs/msg/UInt8 "data: 1" --once

# 4. 等待任务完成（约3.5秒）

# 5. 移动到放置位置，执行低层放置
ros2 topic pub /manipulator/cmd std_msgs/msg/UInt8 "data: 2" --once

# 6. 或执行高层叠放
# ros2 topic pub /manipulator/cmd std_msgs/msg/UInt8 "data: 3" --once
```

## 注意事项

1. **必须先切换到 FIXED_STAND 状态**，否则机械臂运动可能与机身运动冲突
2. 任务执行期间（RUNNING状态）会忽略新的命令
3. 任务完成后保持最后状态，不会自动复位
4. 可通过 `/manipulator/state` 话题监控任务状态
