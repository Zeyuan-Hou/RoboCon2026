# 障碍赛 Nav2 最小接入流程

本文档给出从编译、Nav2 基础接口、坐标系转换，到接入障碍赛 `AUTO_NAV` 流程的最小实现路径。

当前代码中的职责划分：

```text
task_manager_node
  -> 发布 nav_enable
  -> 发布 obstacle_sequence/nav_target [index, x, y, yaw]

obstacle_nav_node
  -> 障碍赛导航上位节点
  -> navigation_backend=LOCAL_PD 时走旧版 PD
  -> navigation_backend=NAV2 时调用 /navigate_to_pose
  -> Nav2 成功后发布 nav_status=true

Nav2
  -> 执行 NavigateToPose
  -> 第一版直接输出 /quad/cmd_vel
```

## 0. 当前进度记录

已完成：

- `lidar_pose_bridge_node` 已从旧雷达定位 `camera_init -> aft_mapped` 桥接出 Nav2 所需链路：

```text
map -> odom -> base_link
```

- `lidar_pose_bridge_node` 已发布 `/quad/odom`：
  - `header.frame_id = odom`
  - `child_frame_id = base_link`
  - `pose` 来自雷达定位位姿
  - `twist` 由雷达位姿窗口差分 + 低通滤波得到
- `ros2 run tf2_ros tf2_echo map base_link` 已验证可连续输出。
- `/quad/odom` 已验证 pose 连续更新，twist 经滤波后锯齿明显降低。
- `colcon build --packages-select task_manager` 已通过。
- `colcon build --packages-select quad` 已通过，用于安装更新后的 `src/launch/lidar_pose_bridge.launch.py`。

当前仍待完成：

- 启动 Nav2 bringup，确认 `/navigate_to_pose` action 存在。
- 准备或接入 `map.yaml` / `map.pgm`。
- 用 RViz 或命令行单独发送 `NavigateToPose` 目标。
- 再接入障碍赛 `AUTO_NAV` 流程。

## 1. 编译环境

### 操作

安装 Nav2 和消息包：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

重新编译 `task_manager`：

```bash
cd /home/cat/hitcrt_quad2026_ws
colcon build --packages-select task_manager
colcon build --packages-select quad
source install/setup.bash
```

### 期望结果

- `colcon build --packages-select task_manager` 成功。
- `colcon build --packages-select quad` 成功，确保 `ros2 launch quad lidar_pose_bridge.launch.py` 使用最新 launch 文件。
- 如果没有安装 `nav2_msgs`，代码仍能编译，但 `navigation_backend=NAV2` 运行时会提示未编译 Nav2 支持。
- 安装 Nav2 后重新编译，`obstacle_nav_node` 会启用真正的 `NavigateToPose` action client。

## 1.1 三阶段调试路线

现场建议按下面三阶段推进，不要一开始就把 Nav2、障碍赛状态机、障碍动作全部混在一起调。

```text
阶段 A：单独调试 Nav2
  目标：验证 Nav2 本体、map/odom/base_link、/quad/odom、/quad/cmd_vel。
  不启动：task_manager_node、task_executor_node、obstacle_nav_node。

阶段 B：调试不带 Nav2 的障碍赛
  目标：验证障碍顺序、AUTO_NAV 触发、旧 LOCAL_PD 到点、障碍动作。
  不启动：Nav2 bringup。

阶段 C：调试带 Nav2 的障碍赛
  目标：把 AUTO_NAV 导航后端从 LOCAL_PD 换成 Nav2。
  启动：Nav2 bringup + obstacle_race.launch.py。
```

### 阶段 A：单独调试 Nav2

适用场景：

- 只验证 Nav2 能否接收 `/navigate_to_pose` goal。
- 只检查 Nav2 是否能基于 `/quad/odom` 和 `map -> odom -> base_link` 输出 `/quad/cmd_vel`。
- 不希望 `task_manager_node` 或 `obstacle_nav_node` 干扰 `/quad/cmd_vel`。

需要配置：

- `src/config/quad_run_cfg.yaml`
  - 确认 `/quad/lidar_pose_bridge_node.publish_nav2_tf: true`
  - 确认 `/quad/lidar_pose_bridge_node.publish_odom: true`
- `nav2_params.yaml`
  - `odom_topic: /quad/odom`
  - `cmd_vel_topic: /quad/cmd_vel`
  - `global_frame: map`
  - `robot_base_frame: base_link`
- 准备 `map.yaml` / `map.pgm`。
- `src/commands/task_manager/config/obstacle_race_params.yaml` 此阶段不生效，因为不启动障碍赛节点。

启动顺序：

终端 1，基础运动栈。只看 Nav2 是否发速度时可先不启动；需要实机运动时再启动。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch quad run.launch.py
```

终端 2，雷达定位。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch point_lio mapping_mid360.launch.py
```

终端 3，TF 和 odom 桥接。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch quad lidar_pose_bridge.launch.py
```

终端 4，Nav2 bringup。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=false \
  autostart:=true \
  map:=/absolute/path/to/map.yaml \
  params_file:=/absolute/path/to/nav2_params.yaml
```

终端 5，发一个近距离目标。

```bash
source /home/cat/hitcrt_quad2026_ws/install/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.0, y: 0.0, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    }
  }
}"
```

检查命令：

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /map --once
ros2 topic echo /quad/odom
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /quad/cmd_vel
ros2 lifecycle nodes
```

通过标准：

- `/navigate_to_pose` 存在。
- `/map` 能发布。
- `/quad/odom` 连续更新。
- `map -> base_link` 可查询。
- 发送 Nav2 goal 后 `/quad/cmd_vel` 有合理速度输出。

### 阶段 B：调试不带 Nav2 的障碍赛

适用场景：

- 先验证障碍赛状态机、`obstacle_sequence.order`、障碍动作是否正常。
- 使用旧版 `LOCAL_PD` 导航，不依赖 Nav2、`map.yaml`、`nav2_params.yaml`。
- 适合现场快速回退和排查“到底是 Nav2 问题还是障碍赛流程问题”。

需要配置 `src/commands/task_manager/config/obstacle_race_params.yaml`：

```yaml
/**:
  ros__parameters:
    obstacle_sequence:
      enabled: true
      order: [pole, sand, crawl]

/quad/obstacle_nav_node:
  ros__parameters:
    navigation_backend: "LOCAL_PD"
    navigation_mode: "EXTERNAL_TARGET"
    target_frame: "camera_init"
    child_frame: "aft_mapped"
```

说明：

- `navigation_backend: "LOCAL_PD"` 表示 `obstacle_nav_node` 自己根据 TF 闭环，直接发布 `/quad/cmd_vel`。
- `navigation_mode: "EXTERNAL_TARGET"` 表示目标点来自 `task_manager_node` 发布的 `/quad/obstacle_sequence/nav_target`。
- 若临时不用表驱动障碍顺序，才切回 `obstacle_sequence.enabled: false` + `navigation_mode: "POINT_SEQUENCE"`。

启动顺序：

终端 1，雷达定位。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch point_lio mapping_mid360.launch.py
```

终端 2，完整障碍赛，但导航后端为 `LOCAL_PD`。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch quad obstacle_race.launch.py
```

可选：如果只是测试状态机和障碍动作，不想启动导航节点抢 `/quad/cmd_vel`：

```bash
ros2 launch quad obstacle_race.launch.py obstacle_nav_enable:=false
```

检查命令：

```bash
ros2 param get /quad/obstacle_nav_node navigation_backend
ros2 param get /quad/obstacle_nav_node navigation_mode
ros2 topic echo /quad/current_state
ros2 topic echo /quad/nav_enable
ros2 topic echo /quad/obstacle_sequence/nav_target
ros2 topic echo /quad/nav_status
ros2 topic echo /quad/cmd_vel
ros2 run tf2_ros tf2_echo camera_init aft_mapped
```

通过标准：

- `navigation_backend` 为 `LOCAL_PD`。
- `AUTO_NAV` 时 `/quad/nav_enable=true`。
- `/quad/obstacle_sequence/nav_target` 发布 `[index, x, y, yaw]`。
- `obstacle_nav_node` 直接输出 `/quad/cmd_vel`。
- 到点后 `/quad/nav_status=true`，状态机进入 QR、视觉伺服或具体障碍动作。

### 阶段 C：调试带 Nav2 的障碍赛

适用场景：

- 阶段 A 已经确认 Nav2 能单独导航。
- 阶段 B 已经确认障碍赛状态机和动作链路正常。
- 现在只替换 `AUTO_NAV` 的导航后端。

需要配置 `src/commands/task_manager/config/obstacle_race_params.yaml`：

```yaml
/**:
  ros__parameters:
    obstacle_sequence:
      enabled: true
      order: [pole, sand, crawl]

/quad/obstacle_nav_node:
  ros__parameters:
    navigation_backend: "NAV2"
    navigation_mode: "EXTERNAL_TARGET"
    nav2_action_name: "/navigate_to_pose"
    goal_frame_id: "map"
    nav2_action_server_wait_s: 0.1
    nav2_retry_period_s: 0.5
```

需要配置 `nav2_params.yaml`：

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /quad/odom

controller_server:
  ros__parameters:
    cmd_vel_topic: /quad/cmd_vel
```

启动顺序：

终端 1，雷达定位。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch point_lio mapping_mid360.launch.py
```

终端 2，TF 和 odom 桥接。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch quad lidar_pose_bridge.launch.py
```

终端 3，Nav2 bringup。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=false \
  autostart:=true \
  map:=/absolute/path/to/map.yaml \
  params_file:=/absolute/path/to/nav2_params.yaml
```

终端 4，先确认 Nav2 服务端在线。

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /quad/odom --once
ros2 run tf2_ros tf2_echo map base_link
```

终端 5，启动障碍赛。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch quad obstacle_race.launch.py
```

检查命令：

```bash
ros2 param get /quad/obstacle_nav_node navigation_backend
ros2 param get /quad/obstacle_nav_node navigation_mode
ros2 param get /quad/obstacle_nav_node goal_frame_id
ros2 topic echo /quad/nav_enable
ros2 topic echo /quad/obstacle_sequence/nav_target
ros2 action info /navigate_to_pose
ros2 topic echo /quad/nav_status
ros2 topic echo /quad/cmd_vel
```

通过标准：

- `navigation_backend` 为 `NAV2`。
- `AUTO_NAV` 时 `/quad/nav_enable=true`。
- `/quad/obstacle_sequence/nav_target` 发布 `[index, x, y, yaw]`。
- `obstacle_nav_node` 日志显示发送 Nav2 goal。
- Nav2 action `SUCCEEDED` 后，`obstacle_nav_node` 发布 `/quad/nav_status=true`。
- `task_manager_node` 收到后离开 `AUTO_NAV`，进入 QR、视觉伺服或具体障碍动作。

## 2. 最小坐标系

Nav2 推荐 TF 链：

```text
map -> odom -> base_link -> lidar
```

当前旧障碍赛导航通常使用：

```text
camera_init -> aft_mapped
```

最小接入时建议先做一个临时 `localization_bridge`，把旧雷达定位桥成 Nav2 标准 TF。

### 最小桥接策略

第一版先不做复杂融合：

```text
camera_init 视为 map
aft_mapped 视为雷达/机体定位结果
map -> odom 发布 identity
odom -> base_link 使用 camera_init -> aft_mapped 的位姿
base_link -> lidar 使用静态外参
```

如果雷达坐标点实际代表雷达中心而不是机体中心，需要用已有外参修正：

```text
base_link_pose = lidar_pose * inverse(base_link_to_lidar)
```

如果暂时没有精确外参，可以先把 `base_link -> lidar` 设为近似：

```bash
ros2 run tf2_ros static_transform_publisher \
  0.05 0.0 0.0 0.0 0.0 0.0 base_link lidar
```

### `localization_bridge` 最小输入输出

订阅：

```text
/tf
  读取 camera_init -> aft_mapped
```

发布：

```text
/quad/odom
  nav_msgs/msg/Odometry
  header.frame_id = "odom"
  child_frame_id = "base_link"

/tf
  map -> odom
  odom -> base_link
```

### 期望结果

运行：

```bash
ros2 launch quad lidar_pose_bridge.launch.py
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /quad/odom --field pose.pose.position
ros2 topic echo /quad/odom --field twist.twist.linear
ros2 topic echo /quad/odom --field twist.twist.angular
```

应看到：

- `map -> base_link` 连续更新。
- `/quad/odom` 位置、yaw 与旧雷达定位基本一致；机器人移动/转动时 twist 应出现非零速度。
- 机器人原地转动时，`base_link` 不绕错误中心大幅画圈。

`tf2_echo` 刚启动时偶尔先打印：

```text
Invalid frame ID "map" ... frame does not exist
```

只要后面能连续打印 `Translation` / `Rotation`，就表示 TF 已经可查询。这通常只是 `tf2_echo` 的本地 buffer 刚启动时还没收到 TF，不是故障。

如果持续出现下面日志，且一直没有 `Translation` / `Rotation` 输出：

```text
Invalid frame ID "map" ... frame does not exist
```

说明当前还没有节点发布 `map` 相关 TF。先确认：

```bash
ros2 launch point_lio mapping_mid360.launch.py
ros2 launch quad lidar_pose_bridge.launch.py
ros2 run tf2_ros tf2_echo camera_init aft_mapped
ros2 run tf2_ros tf2_echo map base_link
```

`camera_init -> aft_mapped` 有输出后，`lidar_pose_bridge_node` 才能桥接出 `map -> odom -> base_link`。

如果 `lidar_pose_bridge_node` 日志为：

```text
Waiting for TF camera_init -> aft_mapped: "camera_init" passed to lookupTransform argument target_frame does not exist.
```

这不是桥接节点自身故障，而是上游定位还没有发布 `camera_init -> aft_mapped`。先检查 point_lio 是否已经启动、雷达/IMU 是否有数据、以及 `/tf` 中实际 frame 名是否仍叫 `camera_init`、`aft_mapped`。

### `/quad/odom` 速度滤波

`/quad/odom.twist` 由雷达位姿窗口差分得到。早期相邻帧差分会把雷达定位的小抖动放大成速度尖峰，当前实现已经改为：

```text
最近 velocity_window_s 时间窗口的位姿差分
  -> 速度限幅
  -> 指数低通
  -> 死区归零
```

当前现场参数在 `src/config/quad_run_cfg.yaml`：

```yaml
velocity_filter_alpha: 0.12
velocity_window_s: 0.30
linear_velocity_deadband: 0.03
angular_velocity_deadband: 0.02
max_linear_velocity: 1.00
max_angular_velocity: 1.50
```

- `velocity_window_s` 越大越平滑，但速度响应越慢；现场抖动大可先升到 `0.40`。
- `velocity_filter_alpha` 越小越平滑，但速度响应越慢；现场抖动大可先降到 `0.08`。
- `linear_velocity_deadband` 用于把静止时的小速度抖动压成 0。
- `max_linear_velocity` 用于挡掉不可能的差分尖峰。

现场判断建议：

- 只前进/后退时，`twist.twist.linear.x` 应明显变化，`linear.y` 应接近 0。
- 只横移时，`linear.y` 应明显变化。
- 原地转向时，`twist.twist.angular.z` 应明显变化，`linear.x/y` 应接近 0。
- 如果机器人没动但 `linear.x/y` 长时间非零，优先增大 `linear_velocity_deadband` 到 `0.04`。
- 如果速度仍有细锯齿，优先增大 `velocity_window_s` 到 `0.40` 或降低 `velocity_filter_alpha` 到 `0.08`。
- 如果 Nav2 控制响应明显变慢，再把 `velocity_window_s` 降到 `0.20`，或把 `velocity_filter_alpha` 升到 `0.15`。

### `/quad/odom` 字段速查

`/quad/odom` 类型为 `nav_msgs/msg/Odometry`：

```text
header.frame_id: odom
child_frame_id: base_link
pose.pose: base_link 在 odom 中的位置和姿态
twist.twist: base_link 坐标系下的线速度和角速度
covariance: 6x6 置信度矩阵按行展开为 36 个数字
```

常用观察命令：

```bash
ros2 topic echo /quad/odom --field pose.pose.position
ros2 topic echo /quad/odom --field pose.pose.orientation
ros2 topic echo /quad/odom --field twist.twist.linear
ros2 topic echo /quad/odom --field twist.twist.angular
```

## 3. Nav2 最小参数

第一阶段先验证 Nav2 能单独跑通，低矮障碍、keepout、点云层可以后加。

### 最小 `nav2_params.yaml` 要点

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /quad/odom

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    cmd_vel_topic: /quad/cmd_vel
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: -0.10
      max_vel_x: 0.35
      min_vel_y: -0.20
      max_vel_y: 0.20
      max_vel_theta: 0.70
      acc_lim_x: 0.5
      acc_lim_y: 0.4
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      decel_lim_y: -0.4
      decel_lim_theta: -1.0
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.15
      sim_time: 1.2
      stateful: true

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 5.0
      height: 5.0
      resolution: 0.05

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
```

机器人 footprint 第一版建议：

```yaml
footprint: "[[0.35, 0.25], [0.35, -0.25], [-0.35, -0.25], [-0.35, 0.25]]"
```

### 地图要求

需要一份先验地图：

```text
map.yaml
map.pgm
```

低矮障碍和固定禁行区第一版建议直接画进地图，标为 occupied cell。

### 期望结果

启动 Nav2 后检查：

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /map --once
ros2 topic echo /quad/odom
ros2 run tf2_ros tf2_echo map base_link
```

应看到：

- `/navigate_to_pose` action 存在。
- `/map` 能发布。
- `/quad/odom` 连续更新。
- `map -> base_link` TF 可查询。

## 4. 单独验证 Nav2

在接入障碍赛前，先用 RViz 或命令行发一个近距离目标。

### 操作

用 RViz 的 `2D Nav Goal` 发送目标，或用 action 命令：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.0, y: 0.0, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    }
  }
}"
```

观察速度：

```bash
ros2 topic echo /quad/cmd_vel
```

### 期望结果

- Nav2 接收 goal。
- `/quad/cmd_vel` 有合理速度输出。
- 机器人能朝目标移动并到点。
- 到点误差第一版达到 10-20cm 即可。

如果 Nav2 不动，优先检查：

```bash
ros2 lifecycle nodes
ros2 topic echo /quad/odom
ros2 run tf2_ros tf2_echo map base_link
```

## 5. 配置 `obstacle_nav_node`

当前障碍赛配置在：

```text
src/commands/task_manager/config/obstacle_race_params.yaml
```

Nav2 后端参数：

```yaml
/quad/obstacle_nav_node:
  ros__parameters:
    navigation_backend: "NAV2"
    navigation_mode: "EXTERNAL_TARGET"
    nav2_action_name: "/navigate_to_pose"
    goal_frame_id: "map"
    nav2_action_server_wait_s: 0.1
    nav2_retry_period_s: 0.5
```

如果现场目标点仍是 `camera_init` 坐标，而 Nav2 也临时使用 `camera_init` 作为全局 frame，可以改成：

```yaml
goal_frame_id: "camera_init"
```

但推荐最终统一到：

```text
goal_frame_id: "map"
```

### 期望结果

启动 `obstacle_nav_node` 后：

```bash
ros2 param get /quad/obstacle_nav_node navigation_backend
ros2 param get /quad/obstacle_nav_node navigation_mode
ros2 param get /quad/obstacle_nav_node goal_frame_id
```

应得到：

```text
NAV2
EXTERNAL_TARGET
map
```

## 6. 接入障碍赛流程

### 启动顺序

建议现场按这个顺序：

1. 启动雷达定位。
2. 启动 `localization_bridge` 和静态 TF。
3. 启动 Nav2。
4. 用 RViz 单独验证 `/navigate_to_pose`。
5. 启动障碍赛：

```bash
ros2 launch quad obstacle_race.launch.py
```

### AUTO_NAV 数据流

```text
task_manager_node
  发布 /quad/nav_enable=true
  发布 /quad/obstacle_sequence/nav_target [index, x, y, yaw]

obstacle_nav_node
  收到 nav_enable 和 nav_target
  调用 /navigate_to_pose

Nav2
  发布 /quad/cmd_vel
  到点后 action SUCCEEDED

obstacle_nav_node
  发布 /quad/nav_status=true

task_manager_node
  进入 QR_RECOGNITION 或具体障碍任务
```

### 观察命令

```bash
ros2 topic echo /quad/nav_enable
ros2 topic echo /quad/obstacle_sequence/nav_target
ros2 topic echo /quad/nav_status
ros2 action list | grep navigate_to_pose
ros2 action info /navigate_to_pose
ros2 topic echo /quad/cmd_vel
```

### 期望结果

- `AUTO_NAV` 时 `/quad/nav_enable=true`。
- `/quad/obstacle_sequence/nav_target` 发布 `[index, x, y, yaw]`。
- `obstacle_nav_node` 日志显示发送 Nav2 goal。
- Nav2 成功后 `/quad/nav_status=true`。
- `task_manager_node` 收到后离开 `AUTO_NAV`，进入 QR 或障碍任务。

## 7. 回退旧导航

如果 Nav2 当天不稳定，可以回退旧 PD：

```yaml
/quad/obstacle_nav_node:
  ros__parameters:
    navigation_backend: "LOCAL_PD"
```

### 期望结果

- `obstacle_nav_node` 不调用 `/navigate_to_pose`。
- 继续使用旧 TF 闭环 PD。
- 直接由 `obstacle_nav_node` 发布 `/quad/cmd_vel`。

## 8. 最小问题排查

### 编译找不到 `nav2_msgs`

现象：

```text
Could not find nav2_msgs
```

处理：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
colcon build --packages-select task_manager
```

### Nav2 action 不存在

检查：

```bash
ros2 action list | grep navigate_to_pose
```

期望：

```text
/navigate_to_pose
```

没有则说明 Nav2 bringup 没启动或 lifecycle 未激活。

### TF 不通

检查：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

期望：连续输出 transform。

没有则优先检查 `localization_bridge` 和 `base_link -> lidar` 静态 TF。

### 有目标但不发 Nav2 goal

检查：

```bash
ros2 topic echo /quad/nav_enable
ros2 topic echo /quad/obstacle_sequence/nav_target
ros2 param get /quad/obstacle_nav_node navigation_backend
```

期望：

```text
nav_enable=true
nav_target 有 [index,x,y,yaw]
navigation_backend=NAV2
```

### Nav2 成功但障碍赛不切状态

检查：

```bash
ros2 topic echo /quad/nav_status
```

期望：Nav2 action `SUCCEEDED` 后出现：

```text
data: true
```

如果没有，检查 `obstacle_nav_node` 日志中的 Nav2 result code。

## 9. 后续增强

最小链路跑通后，再逐步增加：

- `localization_bridge` 中加入 IMU yaw 平滑、跳变检测、定位超时保护。
- local costmap 加入 `/quad/perception/obstacle_points` 点云障碍层。
- 固定低矮障碍画进 `/map` 或接入 keepout mask。
- 引入 `cmd_vel_mux`，让 Nav2、视觉伺服、手柄速度统一仲裁。
- 对关键点增加末端精定位控制器。
