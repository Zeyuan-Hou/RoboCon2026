# 障碍赛顺序配置与调试指南

本文记录本次障碍赛流程改造的目标、代码改动、参数位置和现场调试方法。

## 改造目标

本次改造的核心目标是把障碍赛流程从代码里固定的顺序，改成由 YAML 配置表驱动。

改造后可以做到：

- 在 `obstacle_race_params.yaml` 中调整实际障碍执行顺序。
- 每个障碍独立配置导航起始点、二维码 ID、视觉伺服偏置和执行策略。
- 保留旧流程，默认不影响当前可运行障碍赛。
- 支持冗余点：只导航到点，不做二维码伺服，不执行障碍动作。
- 二维码调试和完整障碍赛共用同一份障碍赛参数，调好的视觉伺服参数可直接用于全流程。

目标障碍顺序为：

```text
1 绕杆 -> 2 沙坑 -> 3 矮杆 -> 4 斜坡 -> 5 木桥A -> 6 木桥B -> 7 上下楼梯 -> 8 高墙
```

## 关键文件

障碍赛专用参数：

```text
src/commands/task_manager/config/obstacle_race_params.yaml
```

通用基础参数：

```text
src/config/quad_run_cfg.yaml
```

完整障碍赛启动：

```text
src/launch/obstacle_race.launch.py
```

二维码伺服单独调试启动：

```text
src/launch/qr_servo_debug.launch.py
```

主要代码节点：

```text
src/commands/task_manager/src/task_state_machine.cpp
src/commands/task_manager/src/task_executor_node.cpp
src/commands/task_manager/src/obstacle_nav_node.cpp
```

## 本次主要改动

### 1. task_manager_node 增加 sequence 模式

`task_manager_node` 会读取：

```yaml
/quad/task_manager_node:
  ros__parameters:
    obstacle_sequence:
      enabled: false
      order: [pole, sand, crawl, slope, bridge_a, bridge_b, stairs, wall]
```

当 `enabled: false` 时，保持旧流程。

当 `enabled: true` 时，按 `order` 中的槽位顺序执行障碍赛。

每个槽位包含：

```yaml
pole:
  action: "path"
  expected_task_id: 1
  nav_target: [0.0, 0.0, 0.0]
  visual_offset: [158.89, 1067.05, 5.27]
```

字段含义：

- `action`：到点和二维码确认后执行的策略。
- `expected_task_id`：期望二维码任务 ID。
- `nav_target`：该障碍前的导航目标点 `[x, y, yaw]`。
- `visual_offset`：该障碍二维码伺服目标偏置 `[x_mm, y_mm, yaw_deg]`。

### 2. 支持冗余点 skip

如果某个槽位只需要导航到达，不需要伺服和执行障碍动作，可以配置：

```yaml
reserve_1:
  action: "skip"
  nav_target: [1.0, 2.0, 0.0]
```

并把它加入顺序：

```yaml
order: [pole, reserve_1, sand, crawl]
```

行为：

- 导航到 `reserve_1.nav_target`。
- 到点后立即进入下一个槽位。
- 不进入 `QR_RECOGNITION`。
- 不检查二维码。
- 不执行障碍动作。

### 3. 二维码 ID 校验

sequence 模式下，二维码伺服完成后会检查：

```text
实际 task_id == 当前槽位 expected_task_id
```

如果一致，执行该槽位 `action`。

如果不一致：

- 停止当前流程。
- 切回 `IDLE`。
- 不执行障碍动作。

当前假设：

```text
task_id = marker_id - 18
```

因此任务 7 对应二维码 `marker_id = 25`。

### 4. obstacle_nav_node 增加 EXTERNAL_TARGET 模式

新增导航模式：

```yaml
/quad/obstacle_nav_node:
  ros__parameters:
    navigation_mode: "EXTERNAL_TARGET"
```

该模式下不再按 `target_points` 固定顺序走，而是订阅：

```text
/quad/obstacle_sequence/nav_target
```

消息类型：

```text
std_msgs/Float32MultiArray
```

格式：

```text
[slot_index, x, y, yaw]
```

每次 `slot_index` 变化时，导航节点重置目标并导航到新目标。到点后仍发布：

```text
/quad/nav_status = true
```

旧模式仍保留：

```yaml
navigation_mode: "POINT_SEQUENCE"
navigation_mode: "TRAJECTORY_SEQUENCE"
```

### 5. task_executor_node 使用同一张障碍配置表

执行器新增订阅：

```text
/quad/obstacle_sequence/active_index
/quad/obstacle_sequence/active_segment
/quad/obstacle_sequence/current_slot
```

二维码视觉伺服的目标偏置只有一个来源：

```yaml
obstacle_sequence:
  pole:
    expected_task_id: 1
    visual_offset: [158.89, 1067.05, 5.27]
```

`EXTERNAL_TARGET` 下按当前槽位校验二维码 ID，`POINT_SEQUENCE` 下按二维码 `task_id` 反查槽位。ID 不匹配时执行器会先停车，不进入视觉伺服运动。

### 6. 执行策略 action

当前支持的 `action`：

```text
skip
path
stride
crawl
line
stair_combo
wall
```

含义：

- `skip`：只导航到点，直接进入下一槽位。
- `path`：使用路径跟踪，适合绕杆、沙坑、斜坡；每个槽位用 `path_profiles.<slot>` 区分轨迹和 PID。
- `stride`：使用现有跨步动作，当前沙坑已改为 path 时可不使用。
- `crawl`：切匍匐策略并雷达闭环前进，适合矮杆。
- `line`：雷达直线闭环，适合木桥 A、木桥 B。
- `stair_combo`：先上楼梯直线段，再下楼梯直线段。
- `wall`：执行高墙 JUMP 动作。

路径策略配置：

```yaml
obstacle_sequence:
  sand:
    action: "path"
    path_policy: "upstair"
    restore_policy: "trot"

path_profiles:
  pole:
    trajectory_file: "/absolute/path/to/pole_path.txt"
    kp_x: 0.9
    kp_y: 0.75
    kp_yaw: 0.8
    max_vx: 0.5
    max_vy: 0.45
    max_dyaw: 0.7

  sand:
    trajectory_file: "/absolute/path/to/sand_path.txt"
    kp_x: 0.8
    kp_y: 0.8
    kp_yaw: 0.8
    max_vx: 0.35
    max_vy: 0.35
    max_dyaw: 0.5

  slope:
    trajectory_file: "/absolute/path/to/slope_path.txt"
    kp_x: 0.8
    kp_y: 0.8
    kp_yaw: 0.8
    max_vx: 0.35
    max_vy: 0.35
    max_dyaw: 0.5
```

`path_policy` / `restore_policy` 只对 `action: "path"` 生效：

- `path_policy: "trot"`：默认行为，保持小跑/RL 策略并直接回放路径。
- `path_policy: "upstair"`：先切 `POLICY_UPSTAIR`，底层反馈进入上楼梯策略后再回放路径。
- `restore_policy: "none"`：路径结束后直接进入下一步。
- `restore_policy: "trot"`：路径结束后先切回 `POLICY_TROT`，再进入下一步。

当前沙坑使用：

```text
QR 对准 -> POLICY_UPSTAIR -> path_profiles.sand 轨迹回放 -> POLICY_TROT -> 下一障碍
```

直线策略配置：

```yaml
line_profiles:
  bridge_a:
    target_dist: 1.25
    max_speed: 0.5
    kp_y: 0.8
    kp_xy: 0.5
    kp_yaw: 1.0
    max_vy: 0.5
    max_dyaw: 0.5

  bridge_b:
    target_dist: 1.25
    max_speed: 0.5
    kp_y: 0.8
    kp_xy: 0.5
    kp_yaw: 1.0
    max_vy: 0.5
    max_dyaw: 0.5
```

楼梯两段配置：

```yaml
stair_combo:
  up_segment: "stairs_up"
  down_segment: "stairs_down"

line_profiles:
  stairs_up:
    target_dist: 4.0
    max_speed: 0.4
    kp_y: 1.35
    kp_yaw: 1.0

  stairs_down:
    target_dist: 4.0
    max_speed: 0.25
    kp_y: 1.0
    kp_yaw: 1.0
```

## 如何启用新 sequence 流程

默认配置保持旧流程：

```yaml
obstacle_sequence:
  enabled: false

navigation_mode: "POINT_SEQUENCE"
```

要启用 YAML 表驱动流程，需要同时修改两个地方。

第一处：

```yaml
/quad/task_manager_node:
  ros__parameters:
    obstacle_sequence:
      enabled: true
```

第二处：

```yaml
/quad/obstacle_nav_node:
  ros__parameters:
    navigation_mode: "EXTERNAL_TARGET"
```

然后启动完整障碍赛：

```bash
cd ~/hitcrt_quad2026_ws
colcon build --symlink-install --packages-select task_manager
source install/setup.bash
ros2 launch quad obstacle_race.launch.py
```

## 如何调整障碍顺序

只改：

```yaml
order: [pole, sand, crawl, slope, bridge_a, bridge_b, stairs, wall]
```

例如先跑木桥，再跑绕杆：

```yaml
order: [bridge_a, bridge_b, pole, sand, crawl, slope, stairs, wall]
```

如果要插入冗余点：

```yaml
order: [pole, reserve_1, sand, crawl, slope, bridge_a, bridge_b, stairs, wall]

reserve_1:
  action: "skip"
  nav_target: [1.2, 0.8, 0.0]
```

注意：`order` 中引用的名字必须在 `obstacle_sequence` 下有对应配置。

## 调试建议流程

### 1. 先调二维码伺服

使用二维码伺服调试 launch：

```bash
cd ~/hitcrt_quad2026_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch quad qr_servo_debug.launch.py
```

手柄流程：

```text
LT + START  使能底层
LB + A      站立/起身
LB + X      进入 RL_MOVE
LT + A      进入 AUTO
LT + Y      进入 QR_RECOGNITION，启动二维码伺服
RT + A      退出到手动，准备下一轮
```

调这些参数：

```yaml
visual_servoing:
  kp_x
  kp_y
  kp_yaw
  max_vx
  max_vy
  max_wz
  tolerance_x_m
  tolerance_y_m
  tolerance_yaw_rad

obstacle_sequence:
  <slot>.visual_offset
```

### 2. 再单独调导航点

先把某个障碍设成 `skip`，只验证导航到点：

```yaml
pole:
  action: "skip"
  nav_target: [实测x, 实测y, 实测yaw]
```

启动完整障碍赛后观察：

```bash
ros2 topic echo /quad/obstacle_sequence/current_slot
ros2 topic echo /quad/obstacle_sequence/nav_target
ros2 topic echo /quad/obstacle_nav/current_pose
ros2 topic echo /quad/obstacle_nav/current_target
ros2 topic echo /quad/nav_status
ros2 topic echo /quad/cmd_vel
```

到点后应直接进入下一个槽位，不进入二维码伺服。

### 3. 再打开二维码校验

把 `action` 改回真实动作：

```yaml
pole:
  action: "path"
  expected_task_id: 1
```

观察：

```bash
ros2 topic echo /quad/current_state
ros2 topic echo /quad/task_done
ros2 topic echo /quad/qr_detection_result
```

正确情况：

```text
AUTO_NAV -> QR_RECOGNITION -> TASK_POLE_AROUND -> AUTO_NAV
```

二维码 ID 不匹配时：

```text
QR_RECOGNITION -> IDLE
```

### 4. 最后调具体障碍策略

绕杆、沙坑、斜坡：

```yaml
path_profiles:
  pole:
    trajectory_file
    kp_x
    kp_y
    kp_yaw
    max_vx
    max_vy
    max_dyaw

  sand:
    trajectory_file
    kp_x
    kp_y
    kp_yaw
    max_vx
    max_vy
    max_dyaw

  slope:
    trajectory_file
    kp_x
    kp_y
    kp_yaw
    max_vx
    max_vy
    max_dyaw
```

木桥 A、木桥 B、上下楼梯：

```yaml
line_profiles:
  bridge_a
  bridge_b
  stairs_up
  stairs_down
```

矮杆：

```yaml
tasks:
  crawl_forward_speed
  crawl_kp_y
  crawl_kp_yaw
  crawl_max_vy
  crawl_max_dyaw
  crawl_target_dist
```

旧沙坑跨步动作：

```yaml
policy_timers
task_timers
```

现在 `sand` 已改为 `action: "path"`，正常调沙坑时看 `obstacle_sequence.sand.path_policy`、`obstacle_sequence.sand.restore_policy` 和 `path_profiles.sand`。

## 常用检查命令

检查参数是否加载：

```bash
ros2 param get /quad/task_manager_node obstacle_sequence.enabled
ros2 param get /quad/task_manager_node obstacle_sequence.order
ros2 param get /quad/obstacle_nav_node navigation_mode
ros2 param get /quad/task_manager_node obstacle_sequence.pole.visual_offset
ros2 param get /quad/task_executor_node line_profiles.bridge_a.target_dist
ros2 param get /quad/task_executor_node path_profiles.pole.trajectory_file
```

检查 sequence 运行状态：

```bash
ros2 topic echo /quad/obstacle_sequence/active_index
ros2 topic echo /quad/obstacle_sequence/active_segment
ros2 topic echo /quad/obstacle_sequence/current_slot
ros2 topic echo /quad/obstacle_sequence/nav_target
```

检查导航：

```bash
ros2 topic echo /quad/nav_status
ros2 topic echo /quad/obstacle_nav/current_pose
ros2 topic echo /quad/obstacle_nav/current_target
ros2 topic echo /quad/obstacle_nav/debug_error
```

检查二维码伺服：

```bash
ros2 topic echo /quad/qr_detection_result
ros2 topic echo /quad/visual_servoing/target
ros2 topic echo /quad/visual_servoing/debug_errors
ros2 topic echo /quad/task_done
```

检查速度输出：

```bash
ros2 topic echo /quad/cmd_vel
```

## 验证命令

修改代码或 YAML 后，建议先做：

```bash
cd ~/hitcrt_quad2026_ws
python3 -c 'import yaml; yaml.safe_load(open("src/commands/task_manager/config/obstacle_race_params.yaml")); print("yaml ok")'
python3 -m py_compile src/launch/obstacle_race.launch.py src/launch/qr_servo_debug.launch.py
colcon build --symlink-install --packages-select task_manager
```

## 推荐调试顺序

建议按这个顺序调，不容易把问题混在一起：

```text
1. sequence enabled=false，确认旧流程仍可运行
2. sequence enabled=true + EXTERNAL_TARGET，所有障碍先设 skip，只调导航点
3. 单独打开 pole，调绕杆 QR 偏置和 path profile
4. 单独打开 sand，调沙坑 QR 偏置、上楼梯策略切换和 path profile
5. 单独打开 crawl，调矮杆伺服和匍匐距离
6. 单独打开 slope，调斜坡轨迹
7. 单独打开 bridge_a / bridge_b，分别调直线距离和速度
8. 单独打开 stairs，调 stairs_up / stairs_down 两段
9. 单独打开 wall，调高墙二维码位置和 JUMP 时序
10. 恢复完整 order，跑全流程
```

## 注意事项

- `obstacle_sequence.enabled=true` 时，`obstacle_nav_node.navigation_mode` 应设为 `EXTERNAL_TARGET`。
- `enabled=false` 时，不要依赖 `order`，旧流程仍按原逻辑走。
- `nav_target` 目前是占位值，需要实测后填写。
- `expected_task_id` 必须和二维码识别结果一致，否则会停在 `IDLE`。
- `skip` 槽位不需要 `expected_task_id` 和 `visual_offset`。
- 二维码伺服调出来的参数位于同一个 `obstacle_race_params.yaml`，完整障碍赛会直接使用。
