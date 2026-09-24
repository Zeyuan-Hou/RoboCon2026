# 任务赛调试指南

本文档用于任务赛现场联调，覆盖启动流程、需要调试的模块、关键参数、常用话题与典型问题。当前任务赛链路由 `task_manager`、`lidar_nav_demo_cpp`、底层 `state_machine`、RL 控制链和机械臂节点共同完成。

## 1. 启动与基本流程

启动：

```bash
cd /home/cat/hitcrt_quad_ws
source install/setup.bash
ros2 launch /home/cat/hitcrt_quad_ws/src/launch/task_race.launch.py
```

手柄进入任务赛：

```text
LT + START  -> ENABLE
LB + A      -> UP_DOWN，站起
LB + X      -> ENTER_RL，进入 RL_MOVE
LT + A      -> ENTER_AUTO，开始任务赛
```

任务赛状态流程：
            
```text
CMD_AUTO_NAV
  -> LOGISTICS_INIT_MANIP
     发布 init_cmd
     等待 init_to_place_low_wait_s
     发布 place_low_cmd
  -> LOGISTICS_NAV
     雷达导航到取/放点
  -> TASK_PICKUP_BLOCK / TASK_DROPOFF_BLOCK
     暂停导航
     pre_stand_wait_s
     切 FIXED_STAND
     stand_settle_s
     发布机械臂取/放命令
     等待 pickup_wait_s / dropoff_wait_s
     切回 RL_MOVE
     rl_settle_s
  -> LOGISTICS_NAV
```

急停/退出：

```text
RT + A  -> ENTER_MANUAL
LB + B  -> DAMPING
```

## 2. 需要调试的模块

### task_manager_node

职责：

- 识别任务赛模式 `mission_mode: logistics`
- 控制任务赛状态机
- 控制 `/quad/logistics_nav_enable`
- 收到导航到点事件后切站立、触发机械臂、再切回 RL
- 任务赛开始时发布机械臂初始化和低层待机命令

重点话题：

```bash
ros2 topic echo /quad/current_state
ros2 topic echo /quad/logistics_nav_enable
ros2 topic echo /quad/logistics_nav_event
ros2 topic echo /manipulator/cmd
ros2 topic echo /quad/cmd_evt
ros2 topic echo /quad/state_array
```

正常现象：

```text
ENTER_AUTO 后：
IDLE/MANUAL_NAV -> LOGISTICS_INIT_MANIP -> LOGISTICS_NAV

到取块点：
LOGISTICS_NAV -> TASK_PICKUP_BLOCK -> LOGISTICS_NAV

到放置点：
LOGISTICS_NAV -> TASK_DROPOFF_BLOCK -> LOGISTICS_NAV
```

### lidar_nav_demo_node

职责：

- 从 `waypoints.yaml` 加载取货点、放置点、过渡点
- 等待视觉优先级或超时走默认优先级
- 根据 TF 闭环发布 `/quad/cmd_vel`
- 到达 PICKUP / DROPOFF 后发布 `/quad/logistics_nav_event`

航点类型、`heading_dock` 分阶段控制和现场调参顺序见：

```text
src/commands/lidar_nav_demo_cpp/HEADING_DOCK_TUNING_GUIDE.md
```

重点话题：

```bash
ros2 topic echo /quad/cmd_vel
ros2 topic echo /quad/logistics_nav_event
ros2 topic echo /quad/logistics_nav_enable
ros2 topic echo /tf
```

重点参数检查：

```bash
ros2 param get /quad/lidar_nav_demo_node path_fallback_timeout_sec
ros2 param get /quad/lidar_nav_demo_node pre_path_spin_target_yaws
ros2 param get /quad/lidar_nav_demo_node lidar_offset_x
ros2 param get /quad/lidar_nav_demo_node enable_topic
```

期望加载源码配置时类似：

```text
path_fallback_timeout_sec: 1.0
pre_path_spin_target_yaws: [0.5, -0.5, 0.0]
enable_topic: /quad/logistics_nav_enable
```

如果参数还是默认值，检查 `waypoints.yaml` / `prepath.yaml` 顶层是否为：

```yaml
/quad/lidar_nav_demo_node:
  ros__parameters:
```

### state_machine / RL 控制链

职责：

- 手柄事件转换为底层状态事件
- 自动模式下接收 `/quad/cmd_vel`
- 发布 `/quad/high_command` 给下游控制

重点话题：

```bash
ros2 topic echo /quad/state_array
ros2 topic echo /quad/cmd_vel
ros2 topic echo /quad/high_command
```

判断方法：

- `/quad/cmd_vel` 非零但机器人不动：看 `/quad/high_command` 是否非零。
- `/quad/high_command` 非零但机器人不动：查 RL 控制、电机、底层状态是否在 `RL_MOVE`。
- 进入 AUTO 被拒绝：通常是还没有进入 `RL_MOVE`。

### manipulator_manager_node

职责：

- 订阅 `/manipulator/cmd`
- 将任务命令转换为机械臂状态和气泵动作
- 当前常用命令：

```text
0 = INIT，任务赛开始初始化
1 = GRAB / PICKUP
2 = PLACE_LOW
3 = PLACE_HIGH
```

重点话题：

```bash
ros2 topic echo /manipulator/cmd
ros2 topic echo /manipulator/state
ros2 topic echo /manipulator/result
ros2 topic echo /quad/arm/state_cmd
ros2 topic echo /quad/pump_cmd
```

重要检查：

当前 `manipulator_manager_node.cpp` 需要支持 `init_cmd=0`。如果日志出现：

```text
Unknown command: 0
```

说明初始化命令没有被机械臂管理节点处理，需要先补 `0=INIT` 分支，或者临时关闭：

```yaml
init_on_logistics_start: false
```

### arm_node

职责：

- 控制二自由度机械臂电机
- 根据 `/quad/arm/state_cmd` 切换目标姿态

重点话题：

```bash
ros2 topic echo /quad/arm/state_cmd
ros2 topic echo /quad/arm/status
ros2 topic echo /quad/arm/arrived
ros2 topic echo /quad/manipulator/motor_state
ros2 topic echo /quad/manipulator/motor_cmd
```

重点检查：

- 电机反馈是否正常更新。
- 软限位是否触发。
- `arm_interpolate_time` 是否过短导致动作冲击。
- `grab/place_low/place_high` 的目标角度是否机械上安全。

### pump_node

职责：

- 控制气泵和阀
- 根据 `/quad/pump_cmd` 执行吸合/释放

重点话题：

```bash
ros2 topic echo /quad/pump_cmd
```

重点检查：

- `PUMP_GRAB` 时是否吸住。
- `PUMP_RELEASE` 时是否释放干净。
- GPIO 芯片和线号是否正确。

## 3. 关键参数

### task_manager 参数

文件：

```text
src/config/quad_run_cfg.yaml
```

节点：

```yaml
/quad/task_manager_node:
  ros__parameters:
```

任务赛模式：

```yaml
mission_mode: "logistics"
```

机械臂命令参数：

| 参数 | 作用 | 调试建议 |
| --- | --- | --- |
| `manipulation.command_topic` | 机械臂命令话题 | 默认 `/manipulator/cmd`，要和 manager 订阅一致 |
| `init_on_logistics_start` | 任务赛开始是否发布初始化命令 | 调机械臂初始化时打开 |
| `init_cmd` | 初始化命令值 | 默认 `0` |
| `place_low_after_init` | 初始化后是否发布低层待机命令 | 默认打开 |
| `init_to_place_low_wait_s` | 初始化和低层待机命令间隔 | 初始化动作慢就调大 |
| `pickup_cmd` | 取块命令 | 默认 `1` |
| `place_low_cmd` | 低层放置命令 | 默认 `2` |
| `place_high_cmd` | 高层叠放命令 | 默认 `3` |

动作时序参数：

| 参数 | 作用 | 调试建议 |
| --- | --- | --- |
| `pre_stand_wait_s` | 到取/放点停导航后，切站立前等待 | 机身晃动大调大，如 `1.0` |
| `stand_settle_s` | 切到 `FIXED_STAND` 后等待 | 机械臂动作前站稳用 |
| `pickup_wait_s` | 发布取块命令后的等待时间 | 应大于机械臂取块总耗时 |
| `dropoff_wait_s` | 发布放块命令后的等待时间 | 应大于放置总耗时 |
| `rl_settle_s` | 切回 `RL_MOVE` 后恢复导航前等待 | 回 RL 抖动大调大 |
| `lower_state_timeout_s` | 等待底层状态切换超时 | 超时会重发 `UP_DOWN` / `ENTER_RL` |

### lidar_nav_demo_cpp 参数

文件：

```text
src/commands/lidar_nav_demo_cpp/config/waypoints.yaml
```

路径生成参数：

| 参数 | 作用 | 调试建议 |
| --- | --- | --- |
| `path_fallback_timeout_sec` | 没有视觉优先级时多久走默认路径 | 跳过视觉调导航设 `1.0` |
| `external_manipulation_enable` | 到点后是否交给 task_manager 取放 | 任务赛应为 `true` |
| `enable_topic` | 导航使能话题 | 任务赛用 `/quad/logistics_nav_enable` |
| `auto_parku_enable` | 轨迹回放模式 | 任务赛正常为 `false` |
| `straight_line_enable` | 直线模式 | 多点任务赛为 `false` |

雷达到机体中心偏置：

| 参数 | 作用 | 调试方法 |
| --- | --- | --- |
| `lidar_offset_x` | 雷达到机器人中心的前后偏置 | 到点总是前后偏，优先调 |
| `lidar_offset_y` | 雷达到机器人中心的左右偏置 | 到点总是横向偏，优先调 |

航点与场地参数：

| 参数 | 作用 |
| --- | --- |
| `transition_front` | 前排物资过渡点 |
| `transition_back` | 后排/放置区过渡点 |
| `transition_mid` | 前后排切换中间点 |
| `pickup_points` | 取货点，格式 `[x, y, yaw, is_front]` |
| `dropoff_points` | 放置点，格式 `[x, y, yaw]` |

注意：

- `dropoff_points` 中坐标和 yaw 完全相同的点会被认为是同一个物理放置点。
- 第二次到达同一个物理放置点会自动发布 `place_high_cmd`。

导航控制参数：

| 参数 | 作用 | 调试建议 |
| --- | --- | --- |
| `kp_x` | 前后误差 P 增益 | 前后收敛慢调大 |
| `kp_y` | 横向误差 P 增益 | 横向收敛慢调大 |
| `kp_yaw` | 航向误差 P 增益 | 航向慢调大，抖动调小 |
| `kd_x / kd_y / kd_yaw` | 微分抑制 | 抖动/超调时小幅增加 |
| `lin_vel_creep_min` | 末端最小线速度 | 卡在 10 到 20 cm 可调大 |
| `max_vx / max_vy / max_dyaw` | 速度限幅 | 初调保守，确认方向后放开 |

**纵向优先 TRANSITION**（`forward_only_transition_enable: true` 时）：

| 参数 | 作用 | 调试建议 |
| --- | --- | --- |
| `forward_only_transition_enable` | 普通 TRANSITION 用转向+vx、vy≈0 | `false` 恢复全体横向 trim |
| `forward_only_bearing_threshold` | 方位角大于此值先原地转向 | 扫弧过大可略增 |
| `forward_only_kp_bearing` | 对准目标方向的转向 P | 转太慢调大 |
| `forward_only_max_vx` | 纵向限速 | 走廊可略降 |
| `forward_only_forward_creep_min` | 纵向 creep | 末段卡距离可调 |

例外（仍用原 heading_dock 横向 trim / DOCK）：

- `manip_approach_arrival`：取/放货 A* 段末图点
- `post_manip_exit`：PICKUP/DROPOFF 后该 A* 段首个 TRANSITION（边转边走，不走 FORWARD_ONLY / yaw-first）
- `graph_yaw_align_after_arrival`：\|add_point yaw\|> `graph_reverse_yaw_threshold` 的图点；**FORWARD_ONLY 只追到位置**，到点后 `GRAPH_YAW_ALIGN` 对准图点 yaw（如 3.07），再进下一段
- `manip_approach` 末图点 target_yaw 用 **PICKUP/DROPOFF yaw**，不再用 0
- `PICKUP` / `DROPOFF` 航点

日志：`GRAPH_YAW_ALIGN` 出现在 (-0.65,3.7) 等到点之后；下一段 manip_approach 的 target yaw 应≈3.0 而非 0。

`lin_vel_creep_min` 说明：

```text
当误差还没消掉，但 P 算出来的速度太小时，强行给一个最小速度。
调大更容易进点，但可能过冲；调小更柔和，但可能卡住。
```

### prepath 参数

文件：

```text
src/commands/lidar_nav_demo_cpp/config/prepath.yaml
```

| 参数 | 作用 | 调试建议 |
| --- | --- | --- |
| `pre_path_spin_enable` | 路径生成前是否原地预扫描 | 跳过视觉调导航可设 `false` |
| `pre_path_spin_target_yaws` | 预扫描 yaw 序列 | 覆盖二维码/物资观察方向 |
| `pre_path_spin_pause_s` | 每个 yaw 到位后停顿 | 图像稳定不足调大 |
| `pre_path_spin_kp_yaw` | 预扫描转向 P | 转慢调大，过冲调小 |
| `pre_path_spin_max_wz` | 最大角速度 | 现场保守调 |
| `pre_path_spin_yaw_tol` | yaw 到位容差 | 太难到位可适当放宽 |

### 物流 PRE_SCAN 入口录轨迹（waypoints.yaml）

PRE_SCAN 阶段顺序（`logistics_entry_trajectory_enable: true` 时）：

```text
pre_path_spin → 回放 record_pass.txt（time x y yaw）→ OCR + eightboxes 就绪 → generate_path(当前 TF) → PRE_SCAN_DONE
```

| 参数 | 作用 | 调试建议 |
| --- | --- | --- |
| `logistics_entry_trajectory_enable` | PRE_SCAN 是否在 spin 后先走录轨迹 | 关闭则退回 spin 后直接 A* |
| `logistics_entry_trajectory_path` | 入口轨迹文件 | 与 TF 同坐标系；格式 `time x y yaw` |
| `entry_traj_align_position_tolerance` | spin 后对齐首点位置阈值 | 未对齐不开回放时钟 |
| `entry_traj_align_yaw_tolerance` | 对齐首点 yaw 阈值 | 与上配合，避免一上来猛追 |
| `entry_traj_pause_tracking_error` | 跟踪误差超该值暂停时间轴 | 跟丢时调小；关闭设 0 |
| `kp_x / kp_y / kp_yaw` | 录轨迹回放 PD（body_pd） | 跟踪偏差大时调大或调 `lin_vel_creep_min` |

日志关键字：`LOGISTICS_ENTRY: replay started`、`LOGISTICS_ENTRY: reached end ... pose(...)`，随后应出现 `generate_path` 且首段起点接近轨迹结束位姿。

### task_race.launch.py 中的机械臂参数

文件：

```text
src/launch/task_race.launch.py
```

机械臂电机位置：

| 参数 | 作用 |
| --- | --- |
| `grab_motor0_pos / grab_motor1_pos` | 抓取姿态 |
| `lift_motor0_pos / lift_motor1_pos` | 抬起/转场姿态 |
| `place_low_motor0_pos / place_low_motor1_pos` | 低层放置姿态 |
| `place_high_motor0_pos / place_high_motor1_pos` | 高层叠放姿态 |
| `arm_interpolate_time` | 姿态插值时间 |
| `motor*_Kp/Ki/Kd` | 电机 PID |
| `soft_pos_limit_enabled` | 是否启用软限位 |
| `soft_pos_limit_threshold` | 软限位阈值 |

机械臂 manager 时间：

| 参数 | 作用 |
| --- | --- |
| `grab_arm_time_ms` | 到 GRAB 姿态时间 |
| `grab_pump_time_ms` | 吸合等待时间 |
| `grab_lift_time_ms` | 抬起时间 |
| `place_low_arm_time_ms` | 到低层放置姿态时间 |
| `place_low_release_time_ms` | 低层释放时间 |
| `place_low_lift_time_ms` | 低层释放后抬起时间 |
| `place_high_arm_time_ms` | 到高层放置姿态时间 |
| `place_high_release_time_ms` | 高层释放时间 |
| `place_high_lift_time_ms` | 高层释放后抬起时间 |

气泵参数：

| 参数 | 作用 |
| --- | --- |
| `pump_chip / pump_line` | 气泵 GPIO |
| `valve_chip / valve_line` | 电磁阀 GPIO |
| `cmd_interval_ms` | GPIO 命令间隔 |

## 4. 推荐调试顺序

### 第一步：只确认参数加载

启动后执行：

```bash
ros2 param get /quad/task_manager_node mission_mode
ros2 param get /quad/task_manager_node manipulation.init_cmd
ros2 param get /quad/task_manager_node manipulation.place_low_after_init
ros2 param get /quad/lidar_nav_demo_node path_fallback_timeout_sec
ros2 param get /quad/lidar_nav_demo_node lidar_offset_x
ros2 param get /quad/lidar_nav_demo_node pre_path_spin_target_yaws
```

如果参数不对，先查 YAML 顶层节点名、launch 绝对路径、是否重新启动。

### 第二步：单独调机械臂

手动发布：

```bash
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "{data: 0}"
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "{data: 2}"
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "{data: 1}"
ros2 topic pub --once /manipulator/cmd std_msgs/msg/UInt8 "{data: 3}"
```

观察：

```bash
ros2 topic echo /manipulator/state
ros2 topic echo /quad/arm/state_cmd
ros2 topic echo /quad/pump_cmd
ros2 topic echo /quad/arm/status
```

先保证：

- `0` 初始化有效。
- `2` 能进入低层待机/放置姿态。
- `1` 能抓取并抬起。
- `3` 能高层释放并抬起。

### 第三步：跳过视觉调导航

推荐配置：

```yaml
path_fallback_timeout_sec: 1.0
pre_path_spin_enable: false
```

启动任务赛后应看到：

```text
No path after 1.0 s; enabling fallback priorities.
```

随后应进入 `WP[0]` 跟踪。

### 第四步：调导航方向和收敛

看日志：

```text
WP[i] pos(...) target(...) err_map(...) err_body(...)
```

判断：

- `err_body.x` 是前后误差。
- `err_body.y` 是横向误差。
- `err_yaw` 是航向误差。

看速度：

```bash
ros2 topic echo /quad/cmd_vel
```

如果误差变大，优先查坐标方向、雷达偏置、速度方向。  
如果误差变小但卡在 10 到 20 cm，调 `lin_vel_creep_min`、`kp_x`、`kp_y` 或放宽到点阈值。

### 第五步：联调取放流程

到取/放点后应看到：

```text
lidar_nav_demo_node: Reached waypoint ...
task_manager_node: Logistics waypoint ... PICKUP/DROPOFF
task_manager_node: State Switch: LOGISTICS_NAV -> TASK_PICKUP_BLOCK/TASK_DROPOFF_BLOCK
```

然后顺序检查：

```bash
ros2 topic echo /quad/logistics_nav_enable
ros2 topic echo /quad/cmd_evt
ros2 topic echo /quad/state_array
ros2 topic echo /manipulator/cmd
```

期望：

- `/quad/logistics_nav_enable=false`
- 发布 `UP_DOWN`
- `state_array` 进入 `FIXED_STAND`
- 发布机械臂命令 `1/2/3`
- 等待后发布 `ENTER_RL`
- 回到 `LOGISTICS_NAV`

## 5. 常见问题

### 一直 Waiting for material priorities

检查：

```bash
ros2 param get /quad/lidar_nav_demo_node path_fallback_timeout_sec
ros2 param get /quad/lidar_nav_demo_node pre_path_spin_target_yaws
```

如果还是默认值，说明 YAML 没加载。确认文件顶层是 `/quad/lidar_nav_demo_node`，并重新启动 launch。

### 进入 WP 后不动

检查：

```bash
ros2 topic echo /quad/cmd_vel
ros2 topic echo /quad/high_command
ros2 topic echo /quad/state_array
```

判断：

- `/quad/cmd_vel=0`：导航没使能或到点等待。
- `/quad/cmd_vel` 非零但 `/quad/high_command=0`：状态机没收到速度或不在正确状态。
- `/quad/high_command` 非零但不动：查 RL/电机链路。

### 到点附近卡住

现象：

```text
dist 约 0.12~0.18 m，迟迟不切下一个点
```

原因：

- 当前到点阈值为 `dist < 0.1` 且 `abs(yaw_err) < 0.1`。
- 末端速度不足或横向响应弱。

优先调：

```yaml
kp_x
kp_y
lin_vel_creep_min
lidar_offset_x
lidar_offset_y
```

### 到取/放点后马上切站立导致晃动

调大：

```yaml
pre_stand_wait_s: 1.0
stand_settle_s: 0.5
```

### 机械臂没有执行初始化

检查：

```bash
ros2 topic echo /manipulator/cmd
```

应在任务赛开始看到：

```text
data: 0
```

如果 manager 日志为 `Unknown command: 0`，说明机械臂管理节点还没实现初始化命令处理。

### 初始化后没有进入低层待机

检查：

```bash
ros2 param get /quad/task_manager_node manipulation.place_low_after_init
ros2 param get /quad/task_manager_node manipulation.init_to_place_low_wait_s
ros2 topic echo /manipulator/cmd
```

应先看到 `0`，等待后看到 `2`。

### 高层叠放没有触发

检查 `dropoff_points`：

- 同一个物理放置点需要在 `dropoff_points` 中使用完全相同的 `x, y, yaw`。
- 第二次到达同一物理点时才会发布 `place_high_cmd`。

## 6. 现场建议初值

导航调试：

```yaml
path_fallback_timeout_sec: 1.0
pre_path_spin_enable: false
kp_x: 1.2 ~ 2.0
kp_y: 1.2 ~ 2.0
kp_yaw: 0.8 ~ 1.5
lin_vel_creep_min: 0.16 ~ 0.22
max_vx: 0.5 ~ 0.7
max_vy: 0.45 ~ 0.65
```

机械臂时序：

```yaml
init_to_place_low_wait_s: 1.0 ~ 2.0
pre_stand_wait_s: 0.5 ~ 1.0
stand_settle_s: 0.5 ~ 1.0
pickup_wait_s: 机械臂取块总耗时 + 余量
dropoff_wait_s: 机械臂放块总耗时 + 余量
rl_settle_s: 0.3 ~ 0.8
```

## 7. 快速检查清单

```text
[ ] task_race.launch.py 启动，无 obstacle_nav_node 抢 /quad/cmd_vel
[ ] /quad/task_manager_node mission_mode = logistics
[ ] /quad/lidar_nav_demo_node 参数来自源码 waypoints/prepath
[ ] 手柄能进入 RL_MOVE，再 ENTER_AUTO
[ ] 任务赛开始发布 /manipulator/cmd: 0 -> 2
[ ] /quad/logistics_nav_enable 在 LOGISTICS_NAV 为 true
[ ] /quad/cmd_vel 有非零速度
[ ] /quad/high_command 有非零速度
[ ] 到 PICKUP 发布 /manipulator/cmd: 1
[ ] 到 DROPOFF 第一次发布 2，第二次同点发布 3
[ ] 机械臂动作时间小于 task_manager 等待时间
[ ] 取/放完成后能回到 LOGISTICS_NAV
```
