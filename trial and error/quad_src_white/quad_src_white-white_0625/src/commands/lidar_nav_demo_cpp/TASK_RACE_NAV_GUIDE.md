# 任务赛导航指南

本文只说明任务赛物流导航的启动、进入自动导航方式和重点调参位置。机械臂动作、障碍赛动作和底层策略细节不在这里展开。

## 一键启动

任务赛导航依赖雷达定位、感知结果和任务赛主流程。推荐按下面三个终端启动。

### 1. 雷达定位

雷达定位需要提供 `/tf` 中的 `camera_init -> aft_mapped`。

```bash
cd ~/point_lio
source install/setup.bash
ros2 launch point_lio mapping_mid360.launch.py
```

### 2. 感知栈

感知栈负责相机、YOLO/eightboxes 和 OCR。该脚本不会启动雷达定位。

```bash
cd /home/cat/hitcrt_quad2026_ws
./scripts/start_perception_stack.sh
```

停止感知栈：

```bash
cd /home/cat/hitcrt_quad2026_ws
./scripts/stop_perception_stack.sh
```

### 3. 任务赛导航主流程

任务赛导航主入口是 `src/launch/task_race.launch.py`。

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch /home/cat/hitcrt_quad2026_ws/src/launch/task_race.launch.py
```

如果需要边跑边记录场地锚点：

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch /home/cat/hitcrt_quad2026_ws/src/launch/task_race.launch.py anchor_recorder_enable:=true
```

## 进入自动导航

启动后用手柄进入任务赛导航：

```text
LT + START  -> ENABLE
LB + A      -> 站立/趴下切换
LB + X      -> ENTER_RL
LT + A      -> ENTER_AUTO，进入任务赛物流导航
```

进入自动后，`task_manager_node` 会进入 logistics 流程，并通过 `/quad/logistics_nav_enable` 使能 `lidar_nav_demo_node`。

典型状态序列：

```text
LOGISTICS_INIT_MANIP
LOGISTICS_PRE_SCAN
LOGISTICS_LIFT_ARM
LOGISTICS_NAV
TASK_PICKUP_BLOCK / TASK_DROPOFF_BLOCK
LOGISTICS_NAV
LOGISTICS_DONE
IDLE
```

## 重点调参文件

任务赛导航主要看这四个文件：

```text
src/commands/lidar_nav_demo_cpp/config/waypoints.yaml
src/commands/lidar_nav_demo_cpp/config/waypoints_generated.yaml
src/commands/lidar_nav_demo_cpp/config/prepath.yaml
src/config/quad_run_cfg.yaml
```

`task_race.launch.py` 会启动任务赛节点，但日常调导航优先改上面这些配置。

### 按文件说明

`src/commands/lidar_nav_demo_cpp/config/waypoints.yaml`

这是任务赛物流导航的主配置模板，日常调导航优先看它。它负责点位来源开关、入口轨迹、A* 相关参数、导航控制器、到点阈值、取货顺序和放货映射等。若 `use_field_layout_generated_points=true`，这里的手写 `pickup_points` 和 `dropoff_points` 会被 `field_generated_points_file` 指向的生成文件覆盖，但控制器、到点阈值、入口轨迹、映射关系等仍主要从这个文件调。

`src/commands/lidar_nav_demo_cpp/config/waypoints_generated.yaml`

这是实场地点位生成配置，通常由下面命令从锚点生成：

```bash
python3 src/pythontool/generate_field_waypoints.py --viz
```

当 `waypoints.yaml` 中 `use_field_layout_generated_points=true` 时，导航节点会用它覆盖取货点和放货点。它适合保存当前场地标定后的 `pickup_points`、`dropoff_points` 和可视化相关信息。一般优先改 `example_point.txt` 锚点后重新生成，不建议长期手改；临场微调时可以直接改，但要知道下次生成会覆盖。

`src/commands/lidar_nav_demo_cpp/config/prepath.yaml`

这是开局预扫描配置，控制左右看、左右看之间的暂停、预扫描超时兜底、预扫描结束后等待，以及视觉失败时使用的默认物块优先级。只要问题发生在“开局一直看、看完不生成路径、没视觉时怎么兜底”，优先看这个文件。

`src/config/quad_run_cfg.yaml`

这是全局运行配置。任务赛导航里它主要提供 `mission_mode: logistics`、底层状态机/RL/RKNN/IMU 等通用运行参数，以及部分兼容默认参数。导航点位、路径生成、预扫描和控制器调参仍以 `waypoints.yaml`、`waypoints_generated.yaml`、`prepath.yaml` 为主。

下面按功能列出常改参数，文件职责以上一节为准。

### 点位来源与场地标定

主要在 `waypoints.yaml` 和 `waypoints_generated.yaml`。

```yaml
use_field_layout_generated_points: true
field_generated_points_file: "/home/cat/hitcrt_quad2026_ws/src/commands/lidar_nav_demo_cpp/config/waypoints_generated.yaml"
pickup_points: [...]
dropoff_points: [...]
```

- `use_field_layout_generated_points=true`：启动时从 `field_generated_points_file` 覆盖取放点。
- `waypoints_generated.yaml`：由场地锚点工具生成，通常是实跑使用的取放点来源。
- `pickup_points`：8 个取货点，格式为 `[x, y, yaw, is_front, ...]`。
- `dropoff_points`：8 个放货点，格式为 `[x, y, yaw, ...]`，通常 0~3 是低层，4~7 是高层。

生成点位和可视化：

```bash
cd /home/cat/hitcrt_quad2026_ws
python3 src/pythontool/generate_field_waypoints.py --viz
```

现场重新标取/放点推荐使用交互式标点脚本：

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
python3 src/pythontool/interactive_waypoint_recorder.py --mode field
```

详细流程见 `src/commands/lidar_nav_demo_cpp/TASK_RACE_WAYPOINT_MARKING_GUIDE.md`。

### 预扫描与兜底

主要在 `prepath.yaml`。

```yaml
pre_path_spin_target_yaws: [0.5, -0.5, -0.15]
pre_scan_timeout_s: 8.0
default_priorities: [0, 0, 1, 1, 2, 2, 3, 3]
```

- `pre_path_spin_target_yaws`：开局左右看的 yaw 目标。
- `pre_scan_timeout_s`：预扫描最长等待时间。
- `default_priorities`：没有拿到视觉结果时的默认物块优先级。

### 入口轨迹

主要在 `waypoints.yaml`。

```yaml
logistics_entry_trajectory_enable: true
logistics_entry_trajectory_path: "/home/cat/hitcrt_quad2026_ws/src/commands/lidar_nav_demo_cpp/config/trajectories/record_pass.txt"
```

- `logistics_entry_trajectory_enable=true`：预扫描后使用录制入口轨迹。
- `logistics_entry_trajectory_path`：入口轨迹文件，格式通常为 `time_sec x y yaw`。

### 导航控制

主要在 `waypoints.yaml`。

```yaml
logistics_controller_type: "heading_dock"
approach_max_vx: 1.0
align_max_vx: 0.45
dock_max_vx: 0.35
kp_forward: 1.1
kp_final_yaw: 1.5
```

- `logistics_controller_type="heading_dock"`：任务赛推荐控制器。
- `approach_*`：远距离接近限速。
- `align_*`：中距离对齐限速。
- `dock_*`：取放点最后贴近限速。
- `kp_forward`：前向误差增益。
- `kp_final_yaw`：最终航向增益。

### 到点判定

主要在 `waypoints.yaml`。

```yaml
transition_position_tolerance: 0.20
transition_dist_arrival_tolerance: 0.2
manip_position_tolerance: 0.05
manip_yaw_tolerance: 0.07
arrival_stable_time_s: 0.25
```

- `transition_*_tolerance`：中继点到点阈值。
- `manip_position_tolerance`：取货/放货点位置阈值。
- `manip_yaw_tolerance`：取货/放货点航向阈值。
- `arrival_stable_time_s`：连续满足阈值多久才算到点。

### 取货顺序与映射

主要在 `waypoints.yaml`。

```yaml
eightboxes_pickup_index_map: [4, 5, 6, 7, 3, 2, 1, 0]
dropoff_color_order: [0, 1, 2, 3]
startup_pickup_enable: true
startup_pickup_index: 2
```

- `eightboxes_pickup_index_map`：视觉字符串下标到物理取货点 index 的映射。
- `dropoff_color_order`：低层 4 个坑位从左到右对应的颜色 id。
- `startup_pickup_index`：开局固定优先取的物块 index。

## external_manipulation_enable

`external_manipulation_enable` 决定到达取货/放货点后，导航节点如何处理等待。

只调导航时可以用：

```yaml
external_manipulation_enable: false
wait_time: 2.0
```

含义是到达 `PICKUP` 或 `DROPOFF` 后，本地等待 `wait_time` 秒，然后继续下一个航点。这个模式适合不接机械臂、只调路径和控制。

跑完整取放任务时建议改成：

```yaml
external_manipulation_enable: true
```

含义是到点后发布 `/quad/logistics_nav_event`，暂停导航，交给 `task_manager_node` 和机械臂流程处理，等动作完成后再恢复导航。

## 常用检查话题

启动前确认定位和感知：

```bash
ros2 topic echo /tf --once
ros2 topic echo /eightboxes --once
ros2 topic echo /stable_arithmetic_result --once
```

运行中观察导航：

```bash
ros2 topic echo /quad/logistics_nav_enable
ros2 topic echo /quad/logistics_nav_event
ros2 topic echo /quad/current_pose
ros2 topic echo /quad/current_target
ros2 topic echo /quad/cmd_vel
```

观察上层状态：

```bash
ros2 topic echo /quad/current_state
```

## 可视化输出

导航路径和取货顺序的调试 SVG 会写到：

```text
/home/cat/hitcrt_quad2026_ws/maps/lidar_nav_path_viz
```

这里的 `maps` 是调试输出目录，不是 ROS 地图服务器的地图目录。
