# Pump Node

气泵和电磁阀控制节点，用于机械臂吸盘抓取和释放物体。

## 依赖安装

```bash
sudo apt install libgpiod-dev gpiod
```

## 配置GPIO权限

创建udev规则文件：

```bash
sudo gedit /etc/udev/rules.d/99-gpio.rules
```

写入内容：

```txt
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0666"
```

重新加载规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 编译

```bash
colcon build --packages-select pump_node
```

## 运行

```bash
ros2 run pump_node pump_node
```

## 参数配置

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `pump_chip` | `gpiochip1` | 气泵GPIO芯片名 |
| `pump_line` | `30` | 气泵GPIO线号 |
| `valve_chip` | `gpiochip1` | 电磁阀GPIO芯片名 |
| `valve_line` | `31` | 电磁阀GPIO线号 |

### 自定义参数运行

```bash
ros2 run pump_node pump_node --ros-args \
  -p pump_chip:=gpiochip1 \
  -p pump_line:=30 \
  -p valve_chip:=gpiochip1 \
  -p valve_line:=31
```

## ROS2接口

### 订阅话题

**`/pump_cmd`** (`std_msgs/msg/Int8`)

控制命令：

| 命令值 | 命令 | 动作说明 |
|--------|------|----------|
| `0` | STOP | 气泵关，电磁阀关 |
| `1` | GRAB | 电磁阀关，气泵开（持续吸附） |
| `2` | RELEASE | 释放物体：气泵关 → 延时50ms → 电磁阀开200ms → 电磁阀关 |

### 发布话题

**`/pump_state`** (`std_msgs/msg/String`)

当前状态：

| 状态值 | 说明 |
|--------|------|
| `IDLE` | 空闲状态 |
| `HOLDING` | 吸附中（气泵开，电磁阀关） |
| `RELEASING` | 释放中 |
| `STOPPED` | 已停止 |

## 使用示例

### 抓取物体

```bash
ros2 topic pub /pump_cmd std_msgs/msg/Int8 "{data: 1}"
```

### 释放物体

```bash
ros2 topic pub /pump_cmd std_msgs/msg/Int8 "{data: 2}"
```

### 停止

```bash
ros2 topic pub /pump_cmd std_msgs/msg/Int8 "{data: 0}"
```

### 查看状态

```bash
ros2 topic echo /pump_state
```

## 硬件接线

- 气泵：通过继电器/电子开关连接到指定GPIO
- 电磁阀：通过继电器/电子开关连接到指定GPIO

## 注意事项

1. 确保GPIO编号正确，可通过 `gpioinfo` 查看可用GPIO
2. 运行前确保有GPIO操作权限
3. RELEASE命令执行期间（约250ms），新的命令会中断当前操作
4. 节点退出时会自动关闭气泵和电磁阀

## 调试

查看GPIO状态：

```bash
gpioinfo | grep -E "(pump|valve)"
```

测试GPIO输出：

```bash
gpioset --mode=wait gpiochip1  14=1  # 打开气泵
gpioset --mode=wait gpiochip1  14=0  # 关闭气泵
gpioset --mode=wait gpiochip1  15=1  # 打开电磁阀
gpioset --mode=wait gpiochip1  15=0  # 关闭电磁阀
```
