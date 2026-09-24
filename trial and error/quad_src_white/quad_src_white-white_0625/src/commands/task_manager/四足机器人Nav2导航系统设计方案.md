# 四足机器人 Nav2 导航系统设计方案

## 1. 设计目标

本方案面向 ROS2 Humble / Ubuntu 22.04 环境下的四足机器人导航系统。机器人底层运动控制已经完成，导航层只需要发布 `geometry_msgs/Twist` 到 `/quad/cmd_vel`，由底层执行线速度和角速度。

系统目标：

- 使用先验地图完成室内十几米到二十米级的大范围导航。
- 使用雷达定位提供全局位姿。
- 低矮障碍物采用先验地图禁行区为主、雷达实时检测为辅的策略，并全部绕开。
- 普通导航目标稳定到达 10-20cm 范围。
- 少数实地标定关键点通过末端精定位达到小于 5cm 的位置误差，并满足目标 yaw。
- 对上层任务节点提供统一导航接口，上层不需要直接操作 Nav2 内部模块。

总体架构采用：

```text
Nav2 + 传感器适配层 + 禁行区地图 + 低矮障碍点云补充检测 + 导航适配节点 + 末端精定位控制器
```

## 2. 总体架构

```text
上层任务节点
  |
  | /quad/nav/navigate 或 /quad/nav/goal
  v
quad_nav_adapter
  |
  | nav2_msgs/action/NavigateToPose
  v
Nav2
  |-- map_server
  |-- planner_server
  |-- controller_server
  |-- global_costmap
  |-- local_costmap
  |-- behavior_server
  |
  | /quad/nav2_cmd_vel
  v
cmd_vel_mux 或 quad_nav_adapter 仲裁
  |
  | /quad/cmd_vel
  v
四足底层控制器
```

精定位目标的数据流：

```text
上层任务节点
  |
  | precise=true
  v
quad_nav_adapter
  |
  | 先调用 Nav2 到目标附近
  v
Nav2
  |
  | 到达 0.15-0.20m 范围
  v
quad_precise_controller
  |
  | 低速 /quad/cmd_vel
  v
四足底层控制器
```

推荐让 Nav2 输出到 `/quad/nav2_cmd_vel`，末端精定位输出到 `/quad/precise_cmd_vel`，最后由 `cmd_vel_mux` 或导航适配节点统一仲裁到 `/quad/cmd_vel`。这样可以避免 Nav2 和精定位控制器同时抢占底层速度话题。

## 3. 传感器与数据需求

### 3.1 必需数据

| 数据 | 推荐话题 | ROS2 类型 | 作用 |
|---|---|---|---|
| 雷达定位位姿 | `/quad/localization/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 提供机器人在先验地图中的全局位姿 |
| 3D 点云 | `/quad/lidar/points` | `sensor_msgs/msg/PointCloud2` | 补充检测雷达可见范围内的低矮障碍物 |
| IMU | `/quad/imu/data` | `sensor_msgs/msg/Imu` | 与雷达定位融合生成平滑 odom |
| 2D 地图 | `/map` | `nav_msgs/msg/OccupancyGrid` | Nav2 全局规划地图，包含固定障碍和预标禁行区 |
| TF | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 维护坐标系关系 |

### 3.2 推荐补充数据

| 数据 | 推荐话题 | ROS2 类型 | 作用 |
|---|---|---|---|
| 雷达/IMU 融合里程计 | `/quad/odom` | `nav_msgs/msg/Odometry` | 给 Nav2 提供短时连续位姿 |
| 低矮障碍点云 | `/quad/perception/obstacle_points` | `sensor_msgs/msg/PointCloud2` | 点云滤波后的障碍物输入 |
| 低矮障碍 Scan | `/quad/perception/obstacle_scan` | `sensor_msgs/msg/LaserScan` | 可选，给 Nav2 obstacle layer 使用 |
| 禁行区掩膜 | `/quad/map/keepout_mask` | `nav_msgs/msg/OccupancyGrid` | 可选，独立维护预输入禁止区 |
| 禁行区过滤信息 | `/quad/map/keepout_filter_info` | `nav2_msgs/msg/CostmapFilterInfo` | 可选，给 Nav2 keepout filter 使用 |
| 导航状态 | `/quad/nav/status` | `quad_nav_msgs/msg/QuadNavStatus` | 给上层任务节点反馈状态 |
| 导航诊断 | `/quad/nav/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 调试定位、规划、避障异常 |

### 3.3 低矮障碍物处理策略

低矮障碍物高度范围按 5-20cm 设计。由于雷达安装较高，近距离低矮障碍可能处在雷达盲区，因此不能单独依赖实时检测。推荐采用两层策略：

```text
第一层：先验禁行区
  在地图中预先输入低矮障碍物、危险区域、不可通行区域。
  Nav2 全局规划和局部规划都必须把这些区域视为不可通行。

第二层：雷达实时补充
  只检测雷达实际可见范围内的低矮障碍物。
  用于处理位置偏移、新增障碍或地图误差。
  不作为近距离低矮障碍的唯一安全来源。
```

禁行区有两种实现方式：

```text
方式 A：直接画进 /map
  把已知低矮障碍、禁行区域画成 occupied cell。
  实现最简单，适合比赛场地或固定场景。

方式 B：独立 keepout mask
  保留原始 /map，同时发布 /quad/map/keepout_mask。
  Nav2 使用 keepout filter 把 mask 区域加入代价地图。
  适合频繁修改禁行区、不希望重生成主地图的场景。
```

点云补充检测建议保留高度区间：

```text
min_obstacle_height: 0.03 m
max_obstacle_height: 0.35 m
```

保留到 0.35m 是为了覆盖传感器噪声、地面起伏和障碍物实际高度误差。低于 0.03m 的点通常更接近地面噪声，先过滤掉。

如果雷达安装过高，近距离低矮障碍可能有盲区。此时禁止把“实时点云未检测到障碍”理解为“前方一定可通行”。对近场低矮障碍，应优先依赖预输入禁行区；若现场障碍可能变化，则建议补充以下任意一种低位近场传感器：

- 低位 2D 激光雷达；
- 前向深度相机；
- 前向 ToF / 超声阵列；
- 低位短距 3D 雷达。

否则系统只能保证绕开已标注禁行区和雷达可见障碍，不能保证发现所有近距离低矮障碍物。

## 4. 坐标系设计

Nav2 使用标准 TF 链：

```text
map -> odom -> base_link -> lidar
```

各坐标系含义：

| 坐标系 | 含义 |
|---|---|
| `map` | 先验地图坐标系，全局稳定 |
| `odom` | 短时间连续坐标系，允许缓慢漂移 |
| `base_link` | 机器人机体中心 |
| `lidar` | 雷达安装坐标系 |

### 4.1 推荐 TF 发布关系

| TF | 发布者 | 说明 |
|---|---|---|
| `map -> odom` | `localization_bridge` | 雷达定位对 odom 的全局校正 |
| `odom -> base_link` | `localization_bridge` | 雷达/IMU 融合后的短时连续机器人位姿 |
| `base_link -> lidar` | `robot_state_publisher` 或 static tf | 雷达外参 |

### 4.2 雷达/IMU 融合 odom

本方案固定采用雷达定位和 IMU 融合生成 `/quad/odom`。`localization_bridge` 是导航系统的必需模块，不再依赖底层提供真实 odom。

输入：

```text
/quad/localization/pose
/quad/imu/data
```

输出：

```text
/quad/odom
map -> odom
odom -> base_link
```

融合策略：

```text
雷达定位提供 map 下的全局 x/y/yaw。
IMU 提供短时角速度、姿态变化和 yaw 变化约束。
localization_bridge 对雷达位姿做滤波、跳变检测和时间同步。
输出连续 /quad/odom，并发布 map -> odom 与 odom -> base_link。
```

实现时必须加入：

- 位姿低通滤波；
- IMU yaw/角速度辅助平滑；
- 速度限幅；
- 定位跳变检测；
- 定位超时保护；
- 静止状态速度归零；
- 协方差输出。

跳变保护建议：

```text
单帧平移跳变 > 0.30m：拒绝该帧
单帧 yaw 跳变 > 0.50rad：拒绝该帧
定位超时 > 0.50s：停止导航并上报定位异常
```

## 5. 软件模块设计

### 5.1 `localization_bridge`

职责：

- 接收雷达定位结果。
- 融合雷达定位和 IMU，生成连续 `/quad/odom`。
- 发布 Nav2 所需 TF 链。
- 检测定位超时、跳变、协方差异常。

订阅：

| 话题 | 类型 |
|---|---|
| `/quad/localization/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| `/quad/imu/data` | `sensor_msgs/msg/Imu` |

发布：

| 话题 / TF | 类型 |
|---|---|
| `/quad/odom` | `nav_msgs/msg/Odometry` |
| `map -> odom` | TF |
| `odom -> base_link` | TF |
| `/quad/nav/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |

### 5.2 `pointcloud_obstacle_filter`

职责：

- 接收原始点云。
- 坐标变换到 `base_link` 或 `map`。
- 过滤地面和过高点。
- 保留雷达可见范围内的 5-20cm 低矮障碍物。
- 输出给 Nav2 local costmap 使用，作为先验禁行区之外的补充障碍层。
- 当点云无数据或检测不到近场低矮障碍时，不清除先验禁行区。

订阅：

| 话题 | 类型 |
|---|---|
| `/quad/lidar/points` | `sensor_msgs/msg/PointCloud2` |
| `/tf` | `tf2_msgs/msg/TFMessage` |

发布：

| 话题 | 类型 |
|---|---|
| `/quad/perception/obstacle_points` | `sensor_msgs/msg/PointCloud2` |
| `/quad/perception/obstacle_scan` | `sensor_msgs/msg/LaserScan`，可选 |

推荐参数：

```yaml
pointcloud_obstacle_filter:
  ros__parameters:
    input_topic: "/quad/lidar/points"
    output_points_topic: "/quad/perception/obstacle_points"
    output_scan_topic: "/quad/perception/obstacle_scan"
    target_frame: "base_link"
    min_height: 0.03
    max_height: 0.35
    min_range: 0.15
    max_range: 5.0
    voxel_leaf_size: 0.03
    publish_scan: false
```

### 5.3 Nav2 核心模块

使用 Nav2 标准模块：

| 模块 | 职责 |
|---|---|
| `map_server` | 加载先验 2D 栅格地图 |
| `amcl` | 本方案默认不用，定位由雷达定位系统提供 |
| `planner_server` | 全局路径规划 |
| `controller_server` | 局部路径跟踪和避障 |
| `global_costmap` | 全局代价地图，包含静态地图和预标禁行区 |
| `local_costmap` | 局部代价地图，融合预标禁行区和实时可见障碍层 |
| `behavior_server` | 恢复行为 |
| `bt_navigator` | 执行 NavigateToPose 行为树 |
| `lifecycle_manager` | 管理 Nav2 节点生命周期 |

推荐全局规划器：

```text
SmacPlanner2D
```

推荐局部控制器：

```text
DWB Controller
```

四足机器人支持全向运动，因此 DWB 配置允许：

```text
linear.x
linear.y
angular.z
```

### 5.4 `quad_nav_adapter`

职责：

- 对上层提供统一 Action 和话题接口。
- 将上层目标转换为 Nav2 `NavigateToPose` Action。
- 管理普通导航、精定位导航、取消、失败恢复。
- 把 Nav2 反馈转换为上层易读状态。
- 精定位目标在 Nav2 到附近后切换到 `quad_precise_controller`。

订阅：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/quad/nav/goal` | `geometry_msgs/msg/PoseStamped` | 普通目标 |
| `/quad/nav/precise_goal` | `geometry_msgs/msg/PoseStamped` | 精定位目标 |
| `/quad/nav/cancel` | `std_msgs/msg/Bool` | 取消当前导航 |
| `/quad/precise/status` | `quad_nav_msgs/msg/QuadNavStatus` | 精定位状态 |

Action Server：

```text
/quad/nav/navigate
quad_nav_msgs/action/QuadNavigate
```

Action Client：

```text
/navigate_to_pose
nav2_msgs/action/NavigateToPose
```

发布：

| 话题 | 类型 |
|---|---|
| `/quad/nav/status` | `quad_nav_msgs/msg/QuadNavStatus` |
| `/quad/nav/precise_target` | `geometry_msgs/msg/PoseStamped` |
| `/quad/nav/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |

### 5.5 `quad_precise_controller`

职责：

- 在关键目标点附近接管速度控制。
- 低速修正 x、y、yaw。
- 连续稳定后返回成功。
- 异常时发布零速度并上报失败。

订阅：

| 话题 | 类型 |
|---|---|
| `/quad/nav/precise_target` | `geometry_msgs/msg/PoseStamped` |
| `/tf` | `tf2_msgs/msg/TFMessage` |
| `/quad/nav/precise_enable` | `std_msgs/msg/Bool` |

发布：

| 话题 | 类型 |
|---|---|
| `/quad/precise_cmd_vel` | `geometry_msgs/msg/Twist` |
| `/quad/precise/status` | `quad_nav_msgs/msg/QuadNavStatus` |

控制阶段：

```text
IDLE：未启用，输出零速度
ALIGN：优先对齐 yaw
DOCK：低速修正 x/y/yaw
HOLD：误差满足阈值后保持稳定
SUCCEEDED：上报成功
FAILED：上报失败并停车
```

推荐默认阈值：

```yaml
quad_precise_controller:
  ros__parameters:
    xy_tolerance: 0.05
    yaw_tolerance: 0.05
    stable_time: 0.5
    control_rate: 50.0
    max_vx: 0.05
    max_vy: 0.05
    max_wz: 0.20
    kp_x: 0.6
    kp_y: 0.6
    kp_yaw: 0.8
    cmd_timeout: 0.3
```

### 5.6 `cmd_vel_mux`

职责：

- 仲裁 Nav2 和精定位控制器的速度输出。
- 防止多个节点同时发布 `/quad/cmd_vel`。
- 在任何控制源超时后发布零速度。

输入：

| 话题 | 优先级 |
|---|---|
| `/quad/precise_cmd_vel` | 高 |
| `/quad/nav2_cmd_vel` | 中 |
| `/quad/manual_cmd_vel` | 可配置，通常最高 |

输出：

```text
/quad/cmd_vel
```

推荐策略：

```text
精定位启用时屏蔽 Nav2 cmd_vel
Nav2 普通导航时屏蔽精定位 cmd_vel
任何输入超时超过 0.3s 后输出零速度
失败、取消、定位异常时输出零速度
```

## 6. 自定义接口定义

### 6.1 `QuadNavigate.action`

```text
# Goal
geometry_msgs/PoseStamped target_pose
bool precise
float32 xy_tolerance
float32 yaw_tolerance
string goal_id
---
# Result
bool success
uint16 error_code
string message
---
# Feedback
string state
float32 distance_remaining
float32 yaw_error
float32 obstacle_distance
```

字段说明：

| 字段 | 说明 |
|---|---|
| `target_pose` | 目标位姿，frame 建议为 `map` |
| `precise` | 是否需要末端精定位 |
| `xy_tolerance` | 位置容差，0 表示使用默认值 |
| `yaw_tolerance` | yaw 容差，0 表示使用默认值 |
| `goal_id` | 上层任务 ID，便于日志追踪 |

### 6.2 `QuadNavStatus.msg`

```text
std_msgs/Header header
string state
string mode
string goal_id
float32 distance_to_goal
float32 yaw_error
float32 obstacle_distance
uint16 error_code
string message
```

推荐 `state` 枚举字符串：

```text
IDLE
NAVIGATING
RECOVERING
PRECISE_ALIGN
PRECISE_DOCK
PRECISE_HOLD
SUCCEEDED
CANCELED
FAILED
LOCALIZATION_LOST
OBSTACLE_BLOCKED
```

推荐 `mode` 枚举字符串：

```text
NAV2
PRECISE
MANUAL
STOPPED
```

推荐 `error_code`：

```text
0  OK
1  CANCELED
2  NAV2_FAILED
3  LOCALIZATION_TIMEOUT
4  LOCALIZATION_JUMP
5  OBSTACLE_BLOCKED
6  PRECISE_TIMEOUT
7  TF_UNAVAILABLE
8  CMD_VEL_TIMEOUT
```

## 7. Nav2 参数配置要点

### 7.1 基础参数

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /quad/odom

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    cmd_vel_topic: /quad/nav2_cmd_vel
```

### 7.2 机器人 footprint

机器人尺寸先按 `0.6m x 0.4m`，建议 footprint 留一定余量：

```yaml
footprint: "[[0.35, 0.25], [0.35, -0.25], [-0.35, -0.25], [-0.35, 0.25]]"
```

实际调试时根据腿部摆动、机身宽度、定位误差修正。

### 7.3 local costmap

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 5.0
      height: 5.0
      resolution: 0.05
      plugins: ["static_layer", "voxel_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        observation_sources: obstacle_points
        obstacle_points:
          topic: /quad/perception/obstacle_points
          data_type: PointCloud2
          marking: true
          clearing: true
          min_obstacle_height: 0.03
          max_obstacle_height: 0.35
          obstacle_max_range: 4.0
          raytrace_max_range: 5.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.30
        cost_scaling_factor: 3.0
```

这里的 `voxel_layer` 只作为实时补充障碍层使用。已知低矮障碍、近场盲区障碍和永久禁行区必须通过 `/map` 或 keepout mask 进入 costmap，不能依赖点云实时检测兜底。

### 7.4 global costmap

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.30
        cost_scaling_factor: 3.0
```

如果禁行区需要独立于主地图维护，可增加 Nav2 keepout filter：

```yaml
costmap_filter_info_server:
  ros__parameters:
    type: 0
    filter_info_topic: /quad/map/keepout_filter_info
    mask_topic: /quad/map/keepout_mask
    base: 0.0
    multiplier: 1.0
```

实际部署时二选一即可：固定场地优先直接画进 `/map`；需要频繁修改禁行区时，再使用 keepout mask。

### 7.5 DWB 全向控制

```yaml
controller_server:
  ros__parameters:
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
      vx_samples: 12
      vy_samples: 12
      vtheta_samples: 20
      sim_time: 1.2
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.15
      trans_stopped_velocity: 0.03
      short_circuit_trajectory_evaluation: true
      stateful: true
```

普通导航默认目标容差为：

```text
xy_goal_tolerance: 0.10-0.20m
yaw_goal_tolerance: 0.10-0.20rad
```

小于 5cm 的目标不要只依赖 DWB，到目标附近后交给 `quad_precise_controller`。

## 8. 导航状态机

`quad_nav_adapter` 推荐状态机：

```text
IDLE
  |
  | 收到目标
  v
SEND_NAV2_GOAL
  |
  v
NAVIGATING
  |
  | Nav2 成功 + precise=false
  v
SUCCEEDED

NAVIGATING
  |
  | Nav2 到附近 + precise=true
  v
START_PRECISE
  |
  v
PRECISE_RUNNING
  |
  | 精定位成功
  v
SUCCEEDED

NAVIGATING / PRECISE_RUNNING
  |
  | 取消
  v
CANCELED

NAVIGATING
  |
  | Nav2 失败
  v
RECOVERING
  |
  | 恢复成功
  v
NAVIGATING
  |
  | 恢复超过次数
  v
FAILED
```

恢复策略：

```text
1. 发布零速度
2. 清理局部 costmap
3. 等待 0.5s 让点云刷新
4. 重新发送 Nav2 goal
5. 最多重试 2 次
6. 仍失败则 FAILED
```

## 9. 安全策略

必须停车的情况：

- 定位超时；
- TF 不可用；
- 点云长时间无数据；
- Nav2 或精定位控制器失败；
- 上层取消任务；
- cmd_vel 输入源超时；
- 障碍物过近且无可行路径。

建议阈值：

```text
定位超时: 0.5s
点云超时: 0.5s
TF 查询超时: 0.2s
cmd_vel 源超时: 0.3s
近障停车距离: 0.20m
恢复重试次数: 2
```

所有失败和取消路径都必须最终发布一次或持续发布短时间零速度：

```text
linear.x = 0
linear.y = 0
angular.z = 0
```

## 10. 推荐包结构

```text
quad_nav_bringup/
  launch/
    quad_nav.launch.py
  config/
    nav2_params.yaml
    localization_bridge.yaml
    pointcloud_obstacle_filter.yaml
    precise_controller.yaml
    cmd_vel_mux.yaml
  maps/
    map.yaml
    map.pgm

quad_nav_adapter/
  src/
    quad_nav_adapter_node.cpp

quad_nav_perception/
  src/
    pointcloud_obstacle_filter_node.cpp

quad_nav_localization/
  src/
    localization_bridge_node.cpp

quad_precise_controller/
  src/
    quad_precise_controller_node.cpp

quad_nav_msgs/
  action/
    QuadNavigate.action
  msg/
    QuadNavStatus.msg
```

## 11. 推荐开发顺序

1. 实现 `localization_bridge`，用雷达定位和 IMU 融合发布 `/quad/odom`。
2. 建立 `map -> odom -> base_link -> lidar` TF 链。
3. 加载 `/map`，确认 RViz 中地图、机器人模型和雷达点云对齐。
4. 启动 Nav2，不接精定位，先用 RViz 的 `NavigateToPose` 测普通导航。
5. 在 `/map` 中预先标注低矮障碍和禁行区，或接入 `/quad/map/keepout_mask`。
6. 接入 `/quad/lidar/points`，完成可见低矮障碍的点云过滤。
7. 确认预标禁行区和 `/quad/perception/obstacle_points` 都能进入 costmap。
8. 调通 DWB 全向控制，确认 `/quad/nav2_cmd_vel` 合理。
9. 接入 `cmd_vel_mux`，输出 `/quad/cmd_vel`。
10. 实现 `quad_nav_adapter`，让上层通过 `/quad/nav/navigate` 或 `/quad/nav/goal` 发送目标。
11. 实现 `quad_precise_controller`，处理 `precise=true` 的关键点。
12. 增加恢复、超时、诊断和状态上报。
13. 做实机调参和验收测试。

## 12. 验收测试

### 12.1 TF 与地图

- RViz 中 `map`、`odom`、`base_link`、`lidar` 无断链。
- 机器人在地图中的位置和实际位置一致。
- 原地转向时 `base_link` 不应围绕错误中心大幅绕圈。
- `/quad/odom` 应连续平滑，雷达定位轻微抖动不应直接造成 odom 大幅跳变。
- 人为制造雷达定位跳变时，`localization_bridge` 应拒绝异常帧并上报诊断。

### 12.2 普通导航

- 发送 10m 级目标，机器人可稳定到达。
- 到点误差稳定在 10-20cm。
- 路径不会穿过静态地图障碍。
- `/quad/cmd_vel` 无明显突变。

### 12.3 低矮障碍

- 将已知 5-20cm 障碍物预先画入 `/map` 或 `/quad/map/keepout_mask`。
- 规划路径不得穿过预标禁行区，即使实时点云没有检测到障碍。
- 在雷达可见范围内新增低矮障碍，local costmap 中应出现补充障碍。
- 机器人能绕开预标禁行区和实时可见障碍；绕不开时停车并上报失败。
- 移除临时可见障碍后，清理 local costmap 能恢复规划；预标禁行区不能被实时点云清除。

### 12.4 精定位

- 关键点实地标定。
- `precise=true` 目标先由 Nav2 到附近，再由精定位控制器接管。
- 最终位置误差小于 5cm。
- yaw 误差小于 0.05rad 或任务允许阈值。
- 稳定保持超过 0.5s 后才上报成功。

### 12.5 异常

- 断开定位输入，系统应停车并上报 `LOCALIZATION_TIMEOUT`。
- 断开点云输入，系统应停车或进入保守模式。
- 阻塞路径，系统应有限恢复，超过次数后上报 `OBSTACLE_BLOCKED`。
- 取消目标，系统应取消 Nav2 Action 并发布零速度。

## 13. 关键默认参数

| 参数 | 默认值 |
|---|---|
| 机器人 footprint | `0.70m x 0.50m` 矩形包络 |
| 普通导航位置容差 | `0.15m` |
| 普通导航 yaw 容差 | `0.15rad` |
| 精定位位置容差 | `0.05m` |
| 精定位 yaw 容差 | `0.05rad` |
| 精定位稳定时间 | `0.5s` |
| 障碍膨胀半径 | `0.30m` |
| 最大 `vx` | `0.35m/s` |
| 最大 `vy` | `0.20m/s` |
| 最大 `wz` | `0.70rad/s` |
| 定位超时 | `0.5s` |
| 点云超时 | `0.5s` |
| 恢复重试次数 | `2` |

## 14. 关键假设

- 系统使用 ROS2 Humble / Ubuntu 22.04。
- 底层四足控制器可以稳定执行 `/quad/cmd_vel`。
- 机器人支持全向运动，即 `linear.x`、`linear.y`、`angular.z` 均可用。
- 场地为室内十几米到二十米级。
- 低矮障碍物高度约 5-20cm，并且策略是全部绕开。
- 低矮障碍采用预标禁行区为主、雷达实时检测为辅；不单独依赖实时点云发现近场低矮障碍。
- 先验地图为 2D 占据栅格地图。
- 雷达定位可以提供全局 pose，并与 IMU 通过 `localization_bridge` 融合生成 `/quad/odom`。
- 少数关键目标点会经过实地标定，否则无法保证小于 5cm 的绝对到点精度。
