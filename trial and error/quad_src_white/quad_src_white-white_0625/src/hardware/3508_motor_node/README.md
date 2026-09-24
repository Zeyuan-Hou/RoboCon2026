# M3508 电机驱动节点

鲁班猫通过 CAN 总线控制 RM3508 电机。

## CAN 配置

### 临时配置

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000  # 1Mbps
sudo ip link set can0 up
```

### 开机自启

1. 创建脚本 `/usr/local/bin/setup_can.sh`：

```bash
#!/bin/bash
set -e

# 等待 can0 设备出现
while ! ip link show can0 >/dev/null 2>&1
do
    sleep 0.5
done

# 重新配置
ip link set can0 down || true
ip link set can0 type can bitrate 1000000
ip link set can0 txqueuelen 100
ip link set can0 up
```

1. 赋权：`sudo chmod +x /usr/local/bin/setup_can.sh`

2. 创建 systemd 服务 `/etc/systemd/system/can-auto.service`：

```ini
[Unit]
Description=Configure CAN0
After=systemd-modules-load.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup_can.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

3. 启用服务：`sudo systemctl enable can-auto.service`

## ROS2 接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `/quad/manipulator/motor_cmd` | `std_msgs/Float64MultiArray` | 电流指令 `[i1, i2]` |
| 发布 | `/quad/manipulator/motor_state` | `quad_msg/MotorState` | 位置/速度/电流反馈 |

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `interface` | string | `can0` | CAN 接口名称 |
| `current_limit` | int | `10000` | 电流限幅 (`±current_limit`) |

## 测试

### 发送控制指令

```bash
# 单次发送
ros2 topic pub --once /quad/manipulator/motor_cmd std_msgs/msg/Float64MultiArray "{data: [1000.0, -1000.0]}"

# 循环发送 (10 Hz)
ros2 topic pub /quad/manipulator/motor_cmd std_msgs/msg/Float64MultiArray "{data: [1000.0, -1000.0]}" -r 10
```

### 查看状态

```bash
ros2 topic echo /quad/manipulator/motor_state
```
