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
| `/quad/arm/motor_state` | `std_msgs/Float64MultiArray` | 机械臂电机状态反馈<br>data[0]=电机0电流, data[1]=电机1电流<br>data[2]=电机0速度, data[3]=电机1速度<br>data[4]=电机0位置(相对零位), data[5]=电机1位置(相对零位) |

### 发布话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/quad/arm/state_cmd` | `std_msgs/UInt8` | 机械臂目标状态命令<br>0=NONE, 1=GRAB, 2=LIFT, 3=PLACE_LOW, 4=PLACE_HIGH |
| `/quad/pump_cmd` | `std_msgs/Int8` | 气泵控制命令<br>0=STOP, 1=GRAB, 2=RELEASE |
| `/manipulator/state` | `std_msgs/String` | 当前任务状态<br>IDLE/RUNNING/DONE/ERROR |
| `/manipulator/result` | `std_msgs/Bool` | 任务结果反馈<br>true=成功, false=失败 |
| `/manipulator/suction_detect` | `std_msgs/Bool` | 吸取检测结果（GRAB任务）<br>true=吸取成功, false=吸取失败 |

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

### 吸取检测参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `suction_detect_enable` | true | 是否启用吸取检测 |
| `suction_work_threshold` | 150000.0 | 吸取做功阈值（需根据实际测试调整） |
| `suction_pos_threshold` | 0.0 | 大臂位置阈值（低于此值认为无物块） |
| `suction_pos_debounce_ms` | 50.0 | 位置消抖窗口(ms) |

**吸取检测原理：**
在GRAB任务的LIFT阶段，采用双重判定：先检查大臂位置判断下方是否有物块，再用做功阈值确认吸取状态。

- **位置门槛**：大臂（motor0）位置低于 `suction_pos_threshold` 时认为无物块，直接判失败。消抖窗口 `suction_pos_debounce_ms` 内连续低于阈值才确认，避免噪声误判。
- **做功检测**：位置高于阈值后，累计电机做功（功率对时间积分），与 `suction_work_threshold` 比较。
- 最终判定需同时满足：位置高于阈值（有物块）**且** 做功大于阈值（吸取成功）。

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
5. 吸取检测仅在GRAB任务的LIFT阶段进行，检测结果通过 `/manipulator/suction_detect` 发布
6. 做功阈值需要根据实际测试调整，建议先观察日志中的work值再设定合适阈值
