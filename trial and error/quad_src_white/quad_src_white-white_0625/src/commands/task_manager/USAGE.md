# Task Manager 使用说明

本文档说明如何配置、启动、联调 `task_manager` 障碍赛任务系统。

当前 0522 调试安排：

- 已完成或需回归确认：任务机、视觉相机节点、视觉伺服控制、任务切换策略、矮杆障碍、木桥。
- 待重点调试：雷达导航到障碍点、雷达绕杆、上楼梯任务。

---

## 1. 快速启动

在完整 ROS 2 工作区中编译：

```bash
colcon build
source install/setup.bash
```

推荐使用总启动文件，所有核心节点会进入 `/quad` 命名空间：

```bash
ros2 launch quad run.launch.py
```

若需要拆节点调试，可以单独启动三个核心节点：

```bash
ros2 run task_manager task_manager_node \
  --ros-args -r __ns:=/quad
```

```bash
ros2 run task_manager obstacle_nav_node \
  --ros-args -r __ns:=/quad --params-file src/config/quad_run_cfg.yaml
```

```bash
ros2 run task_manager task_executor_node \
  --ros-args -r __ns:=/quad --params-file src/config/quad_run_cfg.yaml
```

相机节点单独启动：

```bash
ros2 run aruco aruco_processor_node \
  --ros-args -r __ns:=/quad --params-file src/config/quad_run_cfg.yaml
```

---

## 2. 运行前检查

启动前确认：

1. 底层状态机已启动，并发布：

   ```text
   /quad/state_array
   ```

2. 雷达定位 TF 正常：

   ```text
   /tf: camera_init -> aft_mapped
   ```

3. 二维码识别正常发布：

   ```text
   /quad/qr_detection_result
   ```

4. 底盘监听的速度话题与本包一致：

   ```text
   /quad/cmd_vel
   ```

5. 障碍点导航参数已配置：

   ```text
   src/config/quad_run_cfg.yaml 中 /quad/obstacle_nav_node
   ```

6. 若使用绕杆任务，绕杆轨迹文件已配置：

   ```text
   src/config/quad_run_cfg.yaml 中 /quad/task_executor_node / tasks.pole_trajectory_file
   ```

---

## 3. 三节点协作流程

```text
AUTO_NAV
  ↓ task_manager_node 发布 nav_enable=true
obstacle_nav_node 导航到当前障碍点
  ↓ 到达后发布 nav_status=true
QR_RECOGNITION
  ↓ task_executor_node 视觉伺服并发布 task_done(task_id)
对应障碍任务
  ↓ task_done(-1) 或状态机动作序列结束
AUTO_NAV
  ↓ obstacle_nav_node 进入下一个障碍点
...
全部障碍点完成
  ↓ obstacle_nav_node 发布 nav_finished=true
IDLE
```

关键语义：

| 信号 | 含义 |
| --- | --- |
| `nav_enable=true` | 状态机处于 `AUTO_NAV`，允许 `obstacle_nav_node` 发布速度 |
| `nav_status=true` | 当前障碍点导航完成，状态机切到 `QR_RECOGNITION` |
| `task_done(id > 0)` | 二维码识别并对准完成，`id` 是障碍任务类型 |
| `task_done(-1)` | 当前具体任务完成 |
| `nav_finished=true` | 障碍点序列全部完成，状态机切到 `IDLE` |

---

## 4. 坐标系

### 4.1 雷达系

来源：

```text
camera_init -> aft_mapped
```

约定：

```text
x 向右为正
y 向前为正
yaw 为雷达系绝对朝向
```

用于：

- 障碍点自动导航；
- 木桥；
- 上楼梯；
- 绕杆。

雷达系目标点格式：

```text
x y yaw
```

单位：

```text
m, m, rad
```

### 4.2 机体系

`cmd_vel` 使用机体系：

```text
linear.x   向前为正
linear.y   向左为正
angular.z  逆时针为正
```

雷达系误差变换到机体系：

```cpp
dx_world = target_x - current_x;
dy_world = target_y - current_y;

dx_body =  cos(yaw) * dy_world + sin(yaw) * dx_world;
dy_body = -cos(yaw) * dx_world + sin(yaw) * dy_world;
```

### 4.3 QR 视觉系

二维码识别输出约定：

```text
qr.x   向右为正，单位 mm
qr.y   向前为正，单位 mm
qr.yaw 顺时针为正，单位 deg
```

当前 `aruco_processor_node` 的字段来源：

```cpp
qr.x = tvec[0];      // 水平右向，单位 mm
qr.y = tvec[2];      // 相机光轴深度，单位 mm
qr.yaw = pitch;      // 角度，单位 deg
task_type = marker_id - 18;
```

注意：`QrResult.msg` 中有 `z` 字段，但当前视觉伺服不使用 `z`，相机节点也没有填充 `z`。

视觉误差到机体系：

```cpp
err_qr_x_mm = observed_qr_x - target_qr_x;
err_qr_y_mm = observed_qr_y - target_qr_y;
err_qr_yaw_deg = observed_qr_yaw - target_qr_yaw;

err_body_x_m =  err_qr_y_mm / 1000.0;
err_body_y_m = -err_qr_x_mm / 1000.0;
err_body_yaw_rad = yaw_error_sign * err_qr_yaw_deg * pi / 180.0;
```

参数单位关系：

| 参数 | 单位 | 对应误差 | 说明 |
| --- | --- | --- | --- |
| `task_N_offset[0]` | mm | `err_body_y_m` | QR 左右位置目标，QR 右为正，机体左为正时取负 |
| `task_N_offset[1]` | mm | `err_body_x_m` | QR 前后距离目标，除以 1000 后与 `tolerance_x_m` 比较 |
| `task_N_offset[2]` | deg | `err_body_yaw_rad` | 先算角度误差，再转弧度 |
| `tolerance_x_m` | m | `err_body_x_m` | 例如 `0.06` 表示前后误差小于 60 mm |
| `tolerance_y_m` | m | `err_body_y_m` | 例如 `0.06` 表示左右误差小于 60 mm |
| `tolerance_yaw_rad` | rad | `err_body_yaw_rad` | 例如 `0.08 rad` 约等于 4.6 deg |

---

## 5. 障碍点自动导航配置

配置文件：

```text
src/config/quad_run_cfg.yaml
```

节点：

```text
/quad/obstacle_nav_node
```

### 5.1 点序列模式

适合每个障碍点前只需要一个导航停车点。

```yaml
navigation_mode: "POINT_SEQUENCE"

target_points: [
  0.0, 1.5, 0.0,
  0.0, 3.0, 0.0
]
```

`target_points` 是扁平三元组：

```text
[x1, y1, yaw1, x2, y2, yaw2, ...]
```

每个点都在雷达系下。

### 5.2 轨迹序列模式

适合每个障碍点前需要沿采集轨迹进入。

```yaml
navigation_mode: "TRAJECTORY_SEQUENCE"

trajectory_files:
  - "/absolute/path/to/obstacle_1_nav.txt"
  - "/absolute/path/to/obstacle_2_nav.txt"
```

每个轨迹文件格式：

```txt
# x y yaw
0.0 0.5 0.0
0.0 1.0 0.0
0.1 1.5 0.1
```

节点会逐点跟踪当前轨迹文件。到达最后一点后，发布：

```text
nav_status=true
```

### 5.3 控制参数

| 参数 | 作用 |
| --- | --- |
| `position_tolerance` | 位置到达阈值 |
| `yaw_tolerance` | 朝向到达阈值 |
| `kp_x / kp_y / kp_yaw` | P 控制增益 |
| `kd_x / kd_y / kd_yaw` | D 控制增益 |
| `max_vx / max_vy / max_dyaw` | 速度限幅 |
| `final_yaw_align_distance` | 距目标点小于该距离后开始对齐最终 yaw |
| `approach_yaw_max_dyaw` | 远距离朝目标点转向时的角速度限幅 |
| `yaw_first_threshold` | 兼容旧配置保留，当前 LOCAL_PD 不再用它先原地转向 |
| `control_rate` | 控制频率 |

---

## 6. 任务执行器配置

配置文件：

```text
src/config/quad_run_cfg.yaml
```

### 6.1 视觉伺服偏置

障碍赛二维码目标偏置统一配置在：

```text
src/commands/task_manager/config/obstacle_race_params.yaml
```

每个障碍槽位通过 `expected_task_id` 绑定二维码 ID，通过 `visual_offset` 设置唯一目标偏置：

```yaml
obstacle_sequence:
  slope:
    expected_task_id: 4
    visual_offset: [0.0, 1200.0, 0.0]
```

格式：

```text
[x_mm, y_mm, yaw_deg]
```

`visual_servoing.default_offset` 仅作为异常兜底，不作为障碍赛正常调参入口。

视觉伺服闭环调试时，优先观察：

```bash
ros2 topic echo /quad/qr_detection_result
ros2 topic echo /quad/visual_servoing/debug_errors
ros2 topic echo /quad/cmd_vel
```

期望现象：

- 二维码稳定时，`qr_detection_result.task_type` 与障碍任务 ID 一致。
- `debug_errors.x/y/z` 分别逐渐收敛到 `0`，单位为 `m, m, rad`。
- 到达容差后 `/quad/cmd_vel` 输出零速度，并发布 `/quad/task_done`。

如果前后方向反了，检查 `qr.y = tvec[2]` 是否符合相机安装方向；如果旋转方向反了，先调 `visual_servoing.yaw_error_sign`。

### 6.2 木桥参数

| 参数 | 作用 |
| --- | --- |
| `straight_target_yaw` | 目标行进方向 |
| `straight_target_dist` | 目标前进距离 |
| `straight_max_speed` | 最大前进速度 |
| `straight_kp_y` | 横向纠偏增益 |
| `kp_xy` | 前向位置控制增益 |
| `kp_yaw` | 偏航控制增益 |
| `max_vy` | 最大横移速度 |
| `max_dyaw` | 最大角速度 |

### 6.3 上楼梯参数

| 参数 | 作用 |
| --- | --- |
| `tasks.stair_duration` | 上楼梯持续时间 |
| `tasks.stair_target_yaw` | 上楼梯目标方向 |
| `tasks.stair_forward_speed` | 固定前进速度 |
| `tasks.stair_kp_y` | 横向纠偏增益 |
| `tasks.stair_kp_yaw` | 偏航纠偏增益 |
| `tasks.stair_max_vy` | 最大横移速度 |
| `tasks.stair_max_dyaw` | 最大角速度 |
| `tasks.stair_max_distance` | 最大前进距离保护 |

完成条件：

```text
达到 stair_duration 或超过 stair_max_distance
```

### 6.4 绕杆参数

| 参数 | 作用 |
| --- | --- |
| `tasks.pole_trajectory_file` | 绕杆录制轨迹文件路径 |
| `tasks.pole_kp_x / pole_kp_y / pole_kp_yaw` | P 控制增益 |
| `tasks.pole_kd_x / pole_kd_y / pole_kd_yaw` | D 控制增益 |
| `tasks.pole_lin_vel_creep_min` | 平移误差未消除时的最小线速度，0 表示关闭 |
| `tasks.pole_lidar_offset_x / pole_lidar_offset_y` | 雷达到机体系的平移补偿 |
| `tasks.pole_max_vx / pole_max_vy / pole_max_dyaw` | 速度限幅 |

绕杆轨迹格式：

```txt
# time_sec x y yaw
0.00 1.20 0.35 0.00
0.10 1.45 0.62 0.30
0.20 1.70 0.80 0.65
```

---

## 7. 任务 ID 与流程

| ID | 任务 | 流程 |
| --- | --- | --- |
| `1,22` | 上楼梯 | `POLICY_CREEP/准备步态 -> TASK_STAIR_MOVING -> POLICY_TROT` |
| `2,23` | 木桥 | `ENTER_RL -> 雷达直线闭环 -> task_done(-1)` |
| `3,24` | 高墙 | `UP_DOWN -> JUMP -> UP_DOWN -> 恢复导航` |
| `4,25` | 沙坑 | `UP_DOWN -> STRIDE -> UP_DOWN -> 恢复导航` |
| `5,26` | 矮杆 | `POLICY_CREEP -> TASK_CRAWL_MOVING -> POLICY_TROT` |
| `6,27` | 绕杆 | `ENTER_RL -> 录制轨迹时间轴回放 -> task_done(-1)` |

---

## 8. 常用调试命令

查看话题：

```bash
ros2 topic list
```

查看状态机状态：

```bash
ros2 topic echo /quad/current_state
```

查看自动导航使能：

```bash
ros2 topic echo /quad/nav_enable
```

查看当前障碍点到达信号：

```bash
ros2 topic echo /quad/nav_status
```

查看全部导航完成信号：

```bash
ros2 topic echo /quad/nav_finished
```

查看任务完成信号：

```bash
ros2 topic echo /quad/task_done
```

查看导航调试误差：

```bash
ros2 topic echo /quad/obstacle_nav/debug_error
```

查看当前导航目标：

```bash
ros2 topic echo /quad/obstacle_nav/current_target
```

查看视觉识别结果：

```bash
ros2 topic echo /quad/qr_detection_result
```

查看视觉伺服误差：

```bash
ros2 topic echo /quad/visual_servoing/debug_errors
```

查看速度指令：

```bash
ros2 topic echo /quad/cmd_vel
```

---

## 9. 0522 调试安排

### 9.1 已完成项目回归

这些项目已经完成，联调前建议快速复测一遍，确认没有被参数或启动方式改坏。

| 项目 | 回归检查 |
| --- | --- |
| 任务机 | `/quad/current_state` 能随手柄或命令切换，`QR_RECOGNITION`、`AUTO_NAV`、各任务状态正常出现 |
| 视觉相机节点 | `/quad/qr_detection_result` 持续输出，`marker_id` 与实体二维码一致，`task_type = marker_id - 18` |
| 视觉伺服控制 | `/quad/visual_servoing/debug_errors` 收敛，进入容差后发布 `/quad/task_done` |
| 切换策略 | `AUTO_NAV -> QR_RECOGNITION -> TASK_* -> AUTO_NAV` 链路能闭合 |
| 矮杆障碍 | 识别 `task_id=5` 后进入爬行流程，完成后恢复 `POLICY_TROT` |
| 木桥 | 识别 `task_id=2` 后进入 `TASK_BRIDGE_CROSS`，雷达直线闭环结束后发布 `task_done(-1)` |

### 9.2 雷达导航到障碍点调试

目标：在 `AUTO_NAV` 中由 `obstacle_nav_node` 控制 `/quad/cmd_vel`，到达每个障碍点前停车位后发布 `/quad/nav_status=true`。

调试步骤：

1. 确认 `/tf` 中有 `camera_init -> aft_mapped`。
2. 在 `src/config/quad_run_cfg.yaml` 中先配置 1 个近距离 `target_points`，避免一次跑完整序列。
3. 进入 `AUTO_NAV` 后观察 `/quad/nav_enable` 是否为 `true`。
4. 同时观察 `/quad/obstacle_nav/current_pose`、`/quad/obstacle_nav/current_target`、`/quad/obstacle_nav/debug_error`。
5. 若误差逐渐变小，放开速度限幅；若方向反了，优先检查雷达系到机体系误差变换和 `cmd_vel` 方向约定。
6. 到点后确认 `/quad/nav_status=true`，状态机应切到 `QR_RECOGNITION`。

建议初始参数：

```yaml
position_tolerance: 0.10
yaw_tolerance: 0.10
max_vx: 0.20
max_vy: 0.15
max_dyaw: 0.30
```

### 9.3 雷达绕杆调试

目标：识别 `task_id=6` 后进入 `TASK_POLE_AROUND`，按录制轨迹 `time_sec x y yaw` 回放并闭环跟踪。

调试步骤：

1. 确认 `tasks.pole_trajectory_file` 是绝对路径，文件存在。
2. 检查轨迹每行是四列：`time_sec x y yaw`，时间戳严格递增。
3. 先把 `pole_max_vx/pole_max_vy/pole_max_dyaw` 设小，低速验证方向。
4. 观察 `/quad/cmd_vel` 是否连续，速度是否频繁打满限幅。
5. 若轨迹整体偏移，调整 `tasks.pole_lidar_offset_x/y` 或重新录制轨迹。
6. 若绕杆后段发散，优先降低 `pole_kp_*` 或放慢录制轨迹速度。

### 9.4 上楼梯任务调试

目标：识别 `task_id=1` 后进入上楼梯流程，切到对应状态后由 `execute_stair_up()` 控制固定前进并用雷达横向/yaw 纠偏。

调试步骤：

1. 确认状态机中 `task_id=1` 映射到 `TASK_STAIR_UP`。
2. 先短时间测试，把 `tasks.stair_duration` 设小，例如 `1.0` 到 `2.0` 秒。
3. 确认 `/tf` 稳定，因为上楼梯纠偏依赖 `camera_init -> aft_mapped`。
4. 初始降低 `tasks.stair_forward_speed`，观察 `/quad/cmd_vel` 的 `linear.x`、`linear.y`、`angular.z`。
5. 若偏航方向反，检查 `tasks.stair_target_yaw` 与雷达 yaw 正方向。
6. 达到 `stair_duration` 或 `stair_max_distance` 后应停车并发布 `task_done(-1)`。

---

## 10. 调试顺序建议

建议按下面顺序联调：

1. 确认 `/tf` 中存在 `camera_init -> aft_mapped`；
2. 单独启动 `task_manager_node`，确认 `current_state` 正常；
3. 启动 `obstacle_nav_node`，进入 `AUTO_NAV` 后确认 `nav_enable=true`；
4. 用较近的 `POINT_SEQUENCE` 测试自动导航是否能到点；
5. 确认到点后 `nav_status=true`，状态机切到 `QR_RECOGNITION`；
6. 调二维码视觉伺服，确认能发布 `task_done(task_id)`；
7. 单独调每个障碍任务；
8. 最后接完整障碍点序列。

---

## 11. 常见问题

### 11.1 `obstacle_nav_node` 不动

检查：

- `nav_enable` 是否为 `true`；
- `/tf` 是否有 `camera_init -> aft_mapped`；
- `target_points` 是否为空；
- `navigation_mode` 是否配置正确；
- 底盘是否监听同一个 `cmd_vel`。

### 11.2 到点后没有进入二维码识别

检查：

- `obstacle_nav_node` 是否发布了 `nav_status=true`；
- `task_manager_node` 当前是否处于 `AUTO_NAV`；
- `nav_status` 话题名是否一致。

### 11.3 最后一个障碍后没有结束

检查：

- `nav_finished` 是否发布；
- `task_manager_node` 是否处于 `AUTO_NAV`；
- `task_manager_node` 是否订阅到了同名 `nav_finished`。

### 11.4 视觉伺服方向反了

检查 QR 坐标定义：

```text
QR x 向右为正
QR y 向前为正
QR yaw 顺时针为正
```

当前代码按以下方式转换到机体系：

```cpp
body.x =  qr.y
body.y = -qr.x
body.yaw = -qr.yaw
```

如果实际相机输出定义不同，需要同步修改转换关系。

### 11.5 绕杆或轨迹跟踪不平滑

绕杆使用录制轨迹时间轴回放。优先检查：

- 轨迹文件是否是 `time_sec x y yaw` 四列；
- 时间戳是否严格递增；
- `yaw` 是否连续；
- `kp_x/kp_y/kp_yaw` 是否过大；
- 录制轨迹速度是否过快，超过 `pole_max_vx/pole_max_vy/pole_max_dyaw` 后会跟不上。
