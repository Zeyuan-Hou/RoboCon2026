# Arm Node

二自由度机械臂控制节点，控制2个电机。

## 功能特性

- **机械臂控制**：2个电机位置闭环，支持3状态切换（GRAB/LIFT/STACK）
- **平滑过渡**：状态切换时自动进行余弦插值
- **重力补偿**：电机0（大臂）带有基于关节角度的重力补偿
- **动态调参**：支持运行时通过ROS2参数动态调整PID参数

## 话题接口

### 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/quad/manipulator/motor_state` | `quad/msg/MotorState` | 电机状态反馈 [pos0, pos1], [vel0, vel1], [cur0, cur1] |
| `/quad/manipulator/state_cmd` | `std_msgs/UInt8` | 机械臂状态命令 (0=GRAB, 1=LIFT, 2=STACK) |

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/quad/manipulator/motor_cmd` | `std_msgs/Float64MultiArray` | 电机电流指令 [arm0_cur, arm1_cur] |

## 状态定义

| 状态值 | 状态名 | 说明 |
|--------|--------|------|
| 0 | GRAB | 抓取位姿 |
| 1 | LIFT | 搬运位姿 |
| 2 | STACK | 叠放位姿 |

## 参数配置

### 控制参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `control_freq_hz` | double | 400.0 | 控制频率(Hz) |
| `output_max` | double | 8000.0 | PID输出上限 |
| `integral_max` | double | 8000.0 | 积分限幅 |
| `deadband` | double | 0.0 | 死区 |
| `arm_interpolate_time` | double | 1.0 | 机械臂插值过渡时间(s) |

### 机械臂电机0 PID参数 (大臂，带重力补偿)

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `motor0_Kp` | double | 1800.0 | 比例系数 |
| `motor0_Ki` | double | 0.1 | 积分系数 |
| `motor0_Kd` | double | 20.0 | 微分系数 |
| `grav_comp_cur` | double | 700.0 | 重力补偿电流系数 |

### 机械臂电机1 PID参数 (小臂)

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `motor1_Kp` | double | 0.0 | 比例系数 |
| `motor1_Ki` | double | 0.0 | 积分系数 |
| `motor1_Kd` | double | 0.0 | 微分系数 |

### 状态目标位置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `grab_motor0_pos` | double | 0.0 | GRAB状态大臂位置 |
| `grab_motor1_pos` | double | 0.0 | GRAB状态小臂位置 |
| `lift_motor0_pos` | double | 1.0 | LIFT状态大臂位置 |
| `lift_motor1_pos` | double | 0.0 | LIFT状态小臂位置 |
| `stack_motor0_pos` | double | 4.0 | STACK状态大臂位置 |
| `stack_motor1_pos` | double | 0.0 | STACK状态小臂位置 |

## 使用示例

```bash
# 启动节点
ros2 run arm_node arm_node

# 切换到LIFT状态
ros2 topic pub /quad/manipulator/state_cmd std_msgs/msg/UInt8 "data: 1"

# 切换到STACK状态
ros2 topic pub /quad/manipulator/state_cmd std_msgs/msg/UInt8 "data: 2"

# 动态调整PID参数
ros2 param set /arm_node motor0_Kp 2000.0
```

## 节点关系

```
manipulator_manager_node
         │
         ▼
/quad/manipulator/state_cmd
         │
         ▼
    arm_node
         │
         ▼
/quad/manipulator/motor_cmd
```
