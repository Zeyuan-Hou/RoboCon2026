# 任务赛代码移植指南

本文档用于把当前仓库中的任务赛物流导航和自动任务流程移植到另一套 ROS 2 工作区或同类四足机器人平台。重点说明需要搬运的包、关键文件职责、运行依赖、话题接口、配置文件、一键启动命令和联调检查方法。

当前任务赛物流流程默认保留 `/quad` 命名空间，使用 `lidar_nav_demo_cpp` 负责雷达物流导航，不让 `obstacle_nav_node` 同时接管 `/quad/cmd_vel`。

## 1. 两条流程不要混用

### 1.1 任务赛物流流程

任务赛取放物块使用：

```text
mission_mode: "logistics"
```

核心节点：

```text
task_manager_node + lidar_nav_demo_node + manipulator_manager_node
```

主流程：

```text
手柄 LT + A 进入自动模式
  -> task_manager_node 进入 LOGISTICS_INIT_MANIP / LOGISTICS_PRE_SCAN
  -> 发布 /quad/logistics_nav_enable=true
  -> lidar_nav_demo_node 预扫描、等待 /eightboxes 和 /stable_arithmetic_result、生成路径
  -> 进入 LOGISTICS_NAV，持续发布 /quad/cmd_vel
  -> 到 PICKUP / DROPOFF 点后发布 /quad/logistics_nav_event
  -> task_manager_node 暂停导航并触发 /manipulator/cmd
  -> /manipulator/result 返回后恢复 LOGISTICS_NAV
  -> 全部取放完成后进入 LOGISTICS_DONE / IDLE
```

### 1.2 障碍赛自动流程

障碍赛流程使用：

```text
AUTO_NAV -> QR_RECOGNITION -> task_done
```

核心节点：

```text
task_manager_node + obstacle_nav_node + task_executor_node
```

该流程用于障碍点导航、二维码视觉伺服、匍匐、木桥、上楼梯、绕杆等。它和任务赛物流导航都可能发布 `/quad/cmd_vel`，移植或联调任务赛时不要让 `obstacle_nav_node` 与 `lidar_nav_demo_node` 同时控制底盘。

## 2. 一键启动命令

### 2.1 编译

在工作区根目录执行：

```bash
cd /home/cat/hitcrt_quad2026_ws
colcon build --packages-select task_manager lidar_nav_demo_cpp joystick_input state_machine manipulator_manager_node
source install/setup.bash
```

整包编译：

```bash
cd /home/cat/hitcrt_quad2026_ws
colcon build
source install/setup.bash
```

### 2.2 启动雷达定位

任务赛导航依赖 `/tf` 中的 `camera_init -> aft_mapped`。雷达定位需要单独启动，例如：

```bash
cd ~/point_lio
source install/setup.bash
ros2 launch point_lio mapping_mid360.launch.py
```

### 2.3 启动感知栈

感知栈脚本启动 RealSense、YOLO/eightboxes 和 PaddleOCR。雷达定位不在该脚本内启动。

```bash
cd /home/cat/hitcrt_quad2026_ws
./scripts/start_perception_stack.sh
```

常用查看：

```bash
tmux attach -t perception_stack
tmux attach -t ocr_sys
```

停止：

```bash
./scripts/stop_perception_stack.sh
```

### 2.4 启动任务赛主流程

```bash
cd /home/cat/hitcrt_quad2026_ws
source install/setup.bash
ros2 launch /home/cat/hitcrt_quad2026_ws/src/launch/task_race.launch.py
```

手柄进入任务赛自动流程的顺序：

```text
LT + START  -> ENABLE
LB + A      -> UP_DOWN / 站立
LB + X      -> ENTER_RL
LT + A      -> ENTER_AUTO，进入 LOGISTICS 流程
```

## 3. 需要移植的包和文件

### 3.1 总启动与全局配置

| 文件 | 作用 |
| --- | --- |
| `src/launch/task_race.launch.py` | 任务赛总启动文件，启动 task_manager、lidar_nav、手柄、底层状态机、控制器、电机、IMU、机械臂和气泵等节点。 |
| `src/config/quad_run_cfg.yaml` | 主配置文件，包含 `mission_mode: logistics`、机械臂等待时间、障碍赛执行器参数、视觉伺服参数、障碍导航参数。 |
| `src/config/quad_parku.yaml` | 录制轨迹回放/障碍赛相关配置，任务赛物流移植时通常不作为主入口。 |
| `src/launch/run.launch.py` | 旧版/通用启动文件，偏障碍赛自动流程；当前任务赛优先使用 `task_race.launch.py`。 |
| `src/launch/lidar_pose_bridge.launch.py` | 只启动雷达位姿桥接节点，用于不启动任务赛时发布 `/quad/lidar_pose_xyyaw`。 |

移植时重点修改 `task_race.launch.py` 中的绝对路径，例如：

```text
/home/cat/hitcrt_quad2026_ws/src/config/quad_run_cfg.yaml
/home/cat/hitcrt_quad2026_ws/src/commands/lidar_nav_demo_cpp/config/waypoints.yaml
/home/cat/hitcrt_quad2026_ws/src/commands/lidar_nav_demo_cpp/config/prepath.yaml
```

### 3.2 上层任务状态机：`task_manager`

| 文件 | 作用 |
| --- | --- |
| `src/commands/task_manager/src/task_state_machine.cpp` | 上层离散状态机。任务赛中负责 `LOGISTICS_INIT_MANIP`、`LOGISTICS_PRE_SCAN`、`LOGISTICS_NAV`、取货、放货、结束状态；障碍赛中负责 `AUTO_NAV`、`QR_RECOGNITION` 和具体障碍状态切换。 |
| `src/commands/task_manager/include/task_manager/task_state_machine.hpp` | 状态枚举、底层事件枚举、机械臂阶段枚举和 ROS 接口声明。 |
| `src/commands/task_manager/src/task_executor_node.cpp` | 障碍赛执行器：二维码视觉伺服、雷达闭环匍匐、木桥直线、上楼梯、绕杆轨迹回放。任务赛物流主流程不依赖它做取放导航。 |
| `src/commands/task_manager/include/task_manager/task_executor_node.hpp` | 障碍赛执行器的数据结构、模式枚举和参数成员。 |
| `src/commands/task_manager/src/obstacle_nav_node.cpp` | 障碍赛自动导航节点，支持点序列、轨迹序列、录制轨迹回放。任务赛物流时不要和 `lidar_nav_demo_node` 同时发布速度。 |
| `src/commands/task_manager/include/task_manager/obstacle_nav_node.hpp` | 障碍导航参数、状态和工具函数声明。 |
| `src/commands/task_manager/src/lidar_pose_bridge_node.cpp` | 从 TF 读取 `camera_init -> aft_mapped`，独立发布 `/quad/lidar_pose_xyyaw`。 |
| `src/commands/task_manager/CMakeLists.txt` | 编译 `task_manager_node`、`task_executor_node`、`obstacle_nav_node`、`lidar_pose_bridge_node`。 |

任务赛物流状态机关键输出：

```text
/quad/current_state
/quad/logistics_nav_enable
/quad/cmd_evt
/manipulator/cmd
```

任务赛物流关键输入：

```text
/quad/state_array
/quad/logistics_nav_event
/manipulator/result
/manipulator/suction_detect
/quad/cmd_vel
```

### 3.3 雷达物流导航：`lidar_nav_demo_cpp`

| 文件 | 作用 |
| --- | --- |
| `src/commands/lidar_nav_demo_cpp/src/lidar_nav_demo_node.cpp` | 节点构造、参数声明与读取、话题创建、动态参数检查。 |
| `src/commands/lidar_nav_demo_cpp/src/lidar_nav_callbacks.cpp` | 处理 `/tf`、`/eightboxes`、`/stable_arithmetic_result`、`/lidar_nav_control` 回调；锁定 OCR 结果并触发路径生成。 |
| `src/commands/lidar_nav_demo_cpp/src/lidar_nav_path.cpp` | 根据 8 个物块类别、OCR 结果、取放点映射生成取货/放货任务顺序和完整航点序列。 |
| `src/commands/lidar_nav_demo_cpp/src/lidar_nav_control.cpp` | 主控制循环、预扫描、入口轨迹、到点判定、任务赛事件发布、物流速度输出。 |
| `src/commands/lidar_nav_demo_cpp/src/nav_controller.cpp` | `body_pd` 与 `heading_dock` 控制器实现。任务赛推荐 `heading_dock`。 |
| `src/commands/lidar_nav_demo_cpp/include/lidar_nav_demo_cpp/nav_controller.hpp` | 航点类型、控制阶段、控制器输入输出和参数结构。 |
| `src/commands/lidar_nav_demo_cpp/src/lidar_A_star.cpp` | A* 路径规划，用于取放点之间生成避障/走廊中继点。 |
| `src/commands/lidar_nav_demo_cpp/src/isolation_band.cpp` | 隔离带/走廊区域判定。 |
| `src/commands/lidar_nav_demo_cpp/src/lidar_nav_auto_parku.cpp` | AUTO_PARKU 录制轨迹回放模式。 |
| `src/commands/lidar_nav_demo_cpp/src/field_layout_loader.cpp` | 从生成的 YAML 中读取 `pickup_points` 和 `dropoff_points`。 |
| `src/commands/lidar_nav_demo_cpp/CMakeLists.txt` | 编译 `lidar_nav_demo_node` 并安装 `launch`、`config` 目录。 |

导航节点关键输入：

```text
/tf
/quad/logistics_nav_enable
/eightboxes
/stable_arithmetic_result
/lidar_nav_control
```

导航节点关键输出：

```text
/quad/cmd_vel
/quad/current_pose
/quad/current_target
/quad/lidar_pose_xyyaw
/quad/logistics_nav_event
boxsuc
```

### 3.4 底层状态机与手柄

| 文件 | 作用 |
| --- | --- |
| `src/commands/state_machine/state_machine/state_machine.py` | 底层模式/状态桥接。订阅 `cmd_evt` 和 `cmd_vel`，发布 `task_state_command`、`high_command`、`state_array`。 |
| `src/commands/joystick_input/joystick_input/joystick_handler.py` | 手柄按键映射和手动/自动互斥。手动模式发布速度，自动模式只允许退出自动、急停等少量事件。 |
| `src/commands/joystick_input/joystick_input/joystick_pub.py` | 原始手柄数据发布。 |

关键事件：

```text
ENABLE       = 0   # LT + START
UP_DOWN      = 2   # LB + A
ENTER_RL     = 4   # LB + X
ENTER_AUTO   = 12  # LT + A
ENTER_MANUAL = 13  # RT + A
DAMPING      = 3   # LB + B，急停/阻尼
```

### 3.5 机械臂与气泵

| 文件 | 作用 |
| --- | --- |
| `src/manipulator/manipulator_manager_node/src/manipulator_manager_node.cpp` | 机械臂动作管理：初始化、抓取、低层放置、高层放置、导航待机；发布任务结果和吸取检测结果。 |
| `src/manipulator/arm_node/src/arm_node_td.cpp` | 机械臂关节控制节点，接收状态命令并输出关节目标。 |
| `src/manipulator/pump_node/src/pump_node.cpp` | 气泵/阀门 GPIO 控制。 |
| `src/hardware/3508_motor_node/src/3508_motor_node.cpp` | 机械臂 3508 电机驱动。 |

机械臂命令：

```text
/manipulator/cmd: std_msgs/UInt8
0 = INIT_STANDBY
1 = GRAB
2 = PLACE_LOW
3 = PLACE_HIGH
4 = LIFT_STANDBY
```

移植时必须检查：

```text
pump_chip / pump_line / valve_chip / valve_line
机械臂电机 ID、零点、软限位
grab/lift/place_low/place_high 位置参数
吸取检测阈值
```

## 4. 核心配置说明

### 4.1 `src/config/quad_run_cfg.yaml`

必须确认：

```yaml
/quad/task_manager_node:
  ros__parameters:
    mission_mode: "logistics"
```

机械臂流程参数位于：

```yaml
/quad/task_manager_node:
  ros__parameters:
    manipulation:
      command_topic: "/manipulator/cmd"
      init_cmd: 0
      lift_standby_cmd: 4
      pickup_cmd: 1
      place_low_cmd: 2
      place_high_cmd: 3
      dropoff_high_layer_min_index: 4
```

障碍赛执行器参数也在该文件内，例如：

```text
/quad/task_executor_node
visual_servoing
tasks.crawl_*
tasks.stair_*
tasks.pole_*
```

### 4.2 `src/commands/lidar_nav_demo_cpp/config/waypoints.yaml`

这是任务赛物流导航的主配置。重点项：

```yaml
enable_control: false
enable_topic: "/quad/logistics_nav_enable"
auto_parku_enable: false
straight_line_enable: false
logistics_controller_type: "heading_dock"
```

取放点来源：

```yaml
use_field_layout_generated_points: true
field_generated_points_file: "/home/cat/hitcrt_quad2026_ws/src/commands/lidar_nav_demo_cpp/config/waypoints_generated.yaml"
```

手工取放点格式：

```text
pickup_points:  [x, y, yaw, is_front, ...]
dropoff_points: [x, y, yaw, ...]
```

感知与兜底：

```yaml
ocr_wait_after_eightboxes_sec: 1.0
path_fallback_timeout_sec: 5.0
default_priorities: [0, 0, 1, 1, 2, 2, 3, 3]
```

移植到新场地时通常要重新标定：

```text
transition_front / transition_back / transition_mid
pickup_points / dropoff_points
eightboxes_pickup_index_map
dropoff_color_order
startup_pickup_index
arrival/manip/dock 容差和速度参数
```

### 4.3 `src/commands/lidar_nav_demo_cpp/config/prepath.yaml`

预扫描参数：

```yaml
pre_path_spin_enable: true
pre_path_spin_target_yaws: [0.5, -0.5, -0.15]
pre_scan_pause_after_done: true
pre_scan_timeout_s: 8.0
```

该文件控制开局左右看、暂停、超时后是否继续生成路径。

### 4.4 `src/commands/lidar_nav_demo_cpp/config/waypoints_generated.yaml`

由场地锚点工具生成的取放点配置。移植时可以选择：

```text
1. 重新记录场地锚点
2. 生成新的 waypoints_generated.yaml
3. 在 waypoints.yaml 中保持 use_field_layout_generated_points=true
```

相关工具：

```text
src/pythontool/record_anchor_points.py
src/pythontool/generate_field_waypoints.py
src/pythontool/visualize_field_waypoints.py
```

## 5. 硬编码路径和平台相关项

移植前必须全文搜索并替换：

```bash
rg -n "/home/cat|hitcrt_quad2026_ws|point_lio|paddleocrDeploy|ros2_ws|gpiochip|\\.rknn" src scripts
```

重点替换：

| 类别 | 位置 |
| --- | --- |
| 工作区绝对路径 | `task_race.launch.py`、`waypoints.yaml`、`quad_run_cfg.yaml`、脚本目录 |
| 雷达 TF | `target_frame: camera_init`、`child_frame: aft_mapped` |
| 感知工作区 | `scripts/start_perception_stack.sh` 中的 RealSense、YOLO、OCR 路径 |
| RKNN 模型 | `src/inference/rknn_quad_node/models/*` 和对应配置 |
| GPIO | `task_race.launch.py` 中 `pump_chip`、`pump_line`、`valve_chip`、`valve_line` |
| 机械臂参数 | `arm_node` 参数、`manipulator_manager_node` 时间参数 |
| 场地点位 | `waypoints.yaml`、`waypoints_generated.yaml`、`example_point.txt` |

## 6. 联调检查命令

### 6.1 启动前检查

```bash
source /home/cat/hitcrt_quad2026_ws/install/setup.bash
ros2 topic echo /tf --once
ros2 topic echo /quad/state_array --once
ros2 topic echo /eightboxes --once
ros2 topic echo /stable_arithmetic_result --once
```

如果 `/tf` 没有 `camera_init -> aft_mapped`，导航不会获得位姿。

### 6.2 任务赛运行观察

```bash
ros2 topic echo /quad/current_state
ros2 topic echo /quad/logistics_nav_enable
ros2 topic echo /quad/logistics_nav_event
ros2 topic echo /manipulator/result
```

期望状态序列：

```text
LOGISTICS_INIT_MANIP
LOGISTICS_PRE_SCAN
LOGISTICS_LIFT_ARM
LOGISTICS_NAV
TASK_PICKUP_BLOCK
LOGISTICS_NAV
TASK_DROPOFF_BLOCK
LOGISTICS_NAV
LOGISTICS_DONE
IDLE
```

到点事件格式：

```text
/quad/logistics_nav_event: std_msgs/Int32MultiArray
[event_type, waypoint_index, seq, action_id]

0 = PRE_SCAN_DONE
1 = PICKUP_REACHED
2 = DROPOFF_REACHED
3 = LOGISTICS_FINISHED
```

### 6.3 导航调试话题

```bash
ros2 topic echo /quad/current_pose
ros2 topic echo /quad/current_target
ros2 topic echo /quad/cmd_vel
ros2 topic echo /quad/lidar_pose_xyyaw
```

### 6.4 机械臂单独测试

```bash
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "{data: 0}"
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "{data: 1}"
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "{data: 2}"
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "{data: 3}"
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "{data: 4}"
```

观察：

```bash
ros2 topic echo /manipulator/state
ros2 topic echo /manipulator/result
ros2 topic echo /manipulator/suction_detect
```

## 7. 验收标准

移植完成后至少满足：

```text
1. colcon build 通过。
2. /tf、/quad/state_array、/eightboxes、/stable_arithmetic_result 正常。
3. 手柄 LT + A 后进入 LOGISTICS_PRE_SCAN。
4. 预扫描完成后进入 LOGISTICS_LIFT_ARM，再进入 LOGISTICS_NAV。
5. lidar_nav_demo_node 能发布 /quad/cmd_vel，并能更新 /quad/current_pose、/quad/current_target。
6. 到 PICKUP 点后发布 PICKUP_REACHED，task_manager_node 触发 /manipulator/cmd=1。
7. 到 DROPOFF 点后发布 DROPOFF_REACHED，task_manager_node 根据 dropoff index 触发低层或高层放置。
8. /manipulator/result 返回后能恢复 LOGISTICS_NAV。
9. 全部任务完成后进入 LOGISTICS_DONE / IDLE。
```

## 8. 常见问题

### 8.1 机器人不动

检查：

```bash
ros2 topic echo /quad/logistics_nav_enable
ros2 topic echo /quad/current_state
ros2 topic echo /quad/state_array
ros2 topic echo /tf --once
```

常见原因：

```text
没有进入 AUTO 模式
底层不在 RL_MOVE
没有 TF
enable_topic 配错
obstacle_nav_node 与 lidar_nav_demo_node 抢 /quad/cmd_vel
```

### 8.2 一直不生成路径

检查：

```bash
ros2 topic echo /eightboxes --once
ros2 topic echo /stable_arithmetic_result --once
ros2 topic echo boxsuc
```

常见原因：

```text
eightboxes 未检测满 8 个区域
OCR JSON 不符合预期
path_fallback_timeout_sec 太长
pickup/dropoff 点数量或格式错误
```

### 8.3 到取放点后机械臂不动作

检查：

```bash
ros2 topic echo /quad/logistics_nav_event
ros2 topic echo /manipulator/cmd
ros2 topic echo /manipulator/result
```

常见原因：

```text
external_manipulation_enable 与预期不一致
manipulation.enable=false
机械臂节点未启动
/manipulator/cmd 类型或命名空间不一致
机械臂正在运行上一条命令，忽略新命令
```

### 8.4 位置明显不准

优先检查：

```text
TF 坐标系方向
雷达到机体偏移 lidar_offset_x / lidar_offset_y
场地点位是否按当前场地重新标定
pickup/dropoff yaw 是否正确
到点容差是否过紧
```

## 9. 最小移植清单

只迁移任务赛物流主流程时，至少搬运：

```text
src/launch/task_race.launch.py
src/config/quad_run_cfg.yaml
src/commands/task_manager
src/commands/lidar_nav_demo_cpp
src/commands/state_machine
src/commands/joystick_input
src/manipulator/manipulator_manager_node
src/manipulator/arm_node
src/manipulator/pump_node
src/hardware/3508_motor_node
scripts/start_perception_stack.sh
scripts/stop_perception_stack.sh
scripts/lib
```

同时保证自定义消息包可用：

```text
src/interface
src/quad
```

如果新平台已有自己的底层状态机、电机控制和机械臂控制，可以保留上层接口语义，替换底层实现，但需要兼容以下话题：

```text
/quad/cmd_evt
/quad/cmd_vel
/quad/high_command
/quad/state_array
/manipulator/cmd
/manipulator/result
```
