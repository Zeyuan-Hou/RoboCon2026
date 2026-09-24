# 0531 Morning 修改总结

本文记录 5 月 31 日上午围绕任务赛流程的关键修改，主要包含机械臂避挡和导航到点消抖。

## 1. 机械臂避挡左右看

### 问题

任务赛启动后，`lidar_nav_demo_node` 会先进行左右看/预扫描，用于识别物块或等待路径生成。

原先使用 `place_low_cmd=2` 试图让机械臂下到低层位置，但 `2` 在 `manipulator_manager_node` 中是完整低层放置流程：

```text
ARM_PLACE_LOW
-> PUMP_RELEASE
-> ARM_LIFT
```

因此最终机械臂会回到抬起位置，仍可能遮挡左右看视野。

### 修改

在 `manipulator_manager_node.cpp` 中新增命令：

```text
0 = INIT_STANDBY
```

收到 `/manipulator/cmd = 0` 后执行：

```text
ARM_PLACE_LOW
PUMP_STOP
```

该命令只让机械臂进入低层待机姿态，不释放、不抬起。

保留原有命令：

```text
1 = GRAB
2 = PLACE_LOW 完整低层放置流程
3 = PLACE_HIGH 完整高层放置流程
```

### 当前任务赛启动流程

```text
按下任务赛启动
-> task_manager 发布 /manipulator/cmd = 0
-> 机械臂进入 PLACE_LOW 下方待机
-> 等待 init_to_place_low_wait_s
-> 进入 LOGISTICS_NAV
-> lidar_nav_demo_node 开始左右看/预扫描
```

### 相关参数

位置：`src/config/quad_run_cfg.yaml`

```yaml
init_cmd: 0
place_low_after_init: false
init_to_place_low_wait_s: 8.0
after_place_low_wait_s: 2.0
```

参数说明：

```text
init_cmd
  任务赛开始时发送的机械臂初始化/低层待机命令。

place_low_after_init
  当前为 false，表示启动阶段不再额外发送 place_low_cmd=2，避免触发完整放置流程并回到 LIFT。

init_to_place_low_wait_s
  发送 init_cmd 后等待多久再进入 LOGISTICS_NAV。
  当前主要用于保证机械臂已经下到待机姿态，再开始左右看。

after_place_low_wait_s
  当前不生效。
  只有 place_low_after_init=true 时，才表示额外发送 place_low_cmd 后再等待多久进入 LOGISTICS_NAV。
```

## 2. 导航到点消抖

### 问题

原先任务赛导航到点判定是单帧判断：

```text
dist < 0.1
abs(err_yaw) < 0.1
```

只要某一帧 TF/雷达数据进入阈值，就会立即判定到点，容易因为定位抖动产生不稳定触发。

### 修改

在 `lidar_nav_demo_cpp` 中加入到点消抖。

现在到点需要满足：

```text
dist < arrival_position_tolerance
abs(err_yaw) < arrival_yaw_tolerance
并且连续保持 arrival_stable_time_s 秒
```

确认到点后才会触发：

```text
Reached waypoint
PICKUP_REACHED / DROPOFF_REACHED
```

### 相关参数

位置：`src/commands/lidar_nav_demo_cpp/config/waypoints.yaml`

```yaml
arrival_position_tolerance: 0.12
arrival_yaw_tolerance: 0.10
arrival_stable_time_s: 0.30
```

参数说明：

```text
arrival_position_tolerance
  到点距离阈值，单位 m。
  当前 0.12 表示距离小于 12 cm 才进入到点候选。

arrival_yaw_tolerance
  到点航向阈值，单位 rad。
  当前 0.10 rad 约为 5.7 度。

arrival_stable_time_s
  到点消抖时间，单位 s。
  当前 0.30 表示连续 0.3 秒都满足距离和航向阈值，才确认到点。
```

### 调试日志

进入到点候选时会看到类似日志：

```text
Arrival candidate WP[3]: dist=0.095/0.120 yaw=0.040/0.100 stable=0.18/0.30s
```

含义：

```text
dist=0.095/0.120
  当前距离 9.5 cm，小于 12 cm 阈值。

yaw=0.040/0.100
  当前航向误差 0.04 rad，小于 0.10 rad 阈值。

stable=0.18/0.30s
  已连续稳定 0.18 秒，还需要达到 0.30 秒才确认到点。
```

### 消抖重置条件

以下情况会重置到点候选状态：

```text
路径重新生成
外部控制目标更新
导航失能
切换到下一个 waypoint
误差离开阈值
```

