# Task Manager：四足机器人障碍赛任务管理系统

`task_manager` 是一个面向四足机器人障碍赛的 ROS 2 功能包。它把整场比赛拆成三层：

1. **任务状态机**：决定当前处于导航、识别还是某个障碍任务；
2. **障碍赛自动导航**：在 `AUTO_NAV` 阶段基于雷达位姿导航到下一个障碍点；
3. **任务执行器**：完成二维码视觉伺服和具体障碍动作/闭环控制。

相关文档：

- 使用说明：[`USAGE.md`](USAGE.md)
- 状态机图：[`STATE_MACHINE_DIAGRAM.md`](STATE_MACHINE_DIAGRAM.md)
- PNG 状态图：[`assets/task_state_machine.png`](assets/task_state_machine.png)

---

## 1. 节点组成

| 节点 | 可执行文件 | 职责 |
| --- | --- | --- |
| `TaskStateMachine` | `task_manager_node` | 上层离散事件状态机，负责状态切换、底层事件下发、任务分派。 |
| `ObstacleNavNode` | `obstacle_nav_node` | 障碍赛雷达自动导航节点，只在 `AUTO_NAV` 阶段导航到下一个障碍点。 |
| `TaskExecutorNode` | `task_executor_node` | 任务执行器，负责二维码视觉伺服、匍匐、过桥、上楼梯、绕杆等具体控制。 |

核心思想：

```text
AUTO_NAV 阶段：        obstacle_nav_node 发布 cmd_vel
QR_RECOGNITION 阶段：  task_executor_node 发布 cmd_vel
具体障碍任务阶段：     task_executor_node 或 task_manager_node 执行
状态切换与底层事件：   task_manager_node 统一管理
```

---

## 2. 总体流程

```text
task_manager_node 进入 AUTO_NAV
  ↓ 发布 nav_enable=true
obstacle_nav_node 导航到当前障碍点
  ↓ 到达后发布 nav_status=true
task_manager_node: AUTO_NAV -> QR_RECOGNITION
  ↓ 发布 nav_enable=false
obstacle_nav_node 停车并等待
  ↓
task_executor_node 执行二维码视觉伺服
  ↓ 识别 task_id 并发布 task_done(task_id)
task_manager_node 根据 task_id 切入对应障碍任务
  ↓
障碍任务完成 task_done(-1) 或状态机动作序列结束
  ↓
task_manager_node 回到 AUTO_NAV
  ↓
obstacle_nav_node 自动导航到下一个障碍点
```

当障碍点序列全部完成：

```text
obstacle_nav_node 发布 nav_finished=true
  ↓
task_manager_node 在 AUTO_NAV 中收到 nav_finished
  ↓
切换到 IDLE
```

---

## 3. 任务 ID 分配

二维码视觉伺服完成后，`task_executor_node` 会通过 `task_done` 发布识别到的任务 ID。

| Task ID | 任务 | 状态 | 执行方式 |
| --- | --- | --- | --- |
| `1` | 矮杆 | `TASK_CRAWL_CROSS` / `TASK_CRAWL_MOVING` | 切匍匐步态 + 定速盲走 + 切回普通步态 |
| `2` | 木桥 | `TASK_BRIDGE_CROSS` | 雷达直线闭环控制 |
| `3` | 高墙 | `TASK_WALL_CROSS` | 状态机固定动作序列 `JUMP` |
| `4` | 沙坑 | `TASK_SAND_INOUT` | 状态机固定动作序列 `STRIDE` |
| `5` | 矮杆 | `TASK_CRAWL_CROSS` / `TASK_CRAWL_MOVING` | 切匍匐步态 + 定速盲走 + 切回普通步态 |
| `6` | 绕杆 | `TASK_POLE_AROUND` | 读取雷达系录制轨迹 TXT，按时间轴回放并闭环纠偏 |

---

## 4. 关键话题接口

### `task_manager_node`

| 方向 | 话题 | 类型 | 作用 |
| --- | --- | --- | --- |
| Sub | `task_state_command` | `quad/msg/StateCommand` | 外部状态命令 |
| Sub | `state_array` | `std_msgs/Int8MultiArray` | 底层状态反馈 |
| Sub | `nav_status` | `std_msgs/Bool` | 当前障碍点导航完成 |
| Sub | `nav_finished` | `std_msgs/Bool` | 全部障碍点导航完成 |
| Sub | `task_done` | `std_msgs/Int32` | 视觉识别结果或任务完成信号 |
| Pub | `current_state` | `std_msgs/String` | 当前上层状态 |
| Pub | `nav_enable` | `std_msgs/Bool` | 是否允许自动导航节点发布速度 |
| Pub | `cmd_evt` | `std_msgs/Int8` | 底层状态机事件 |

### `obstacle_nav_node`

| 方向 | 话题 | 类型 | 作用 |
| --- | --- | --- | --- |
| Sub | `nav_enable` | `std_msgs/Bool` | `AUTO_NAV` 使能 |
| Sub | `/tf` | `tf2_msgs/TFMessage` | 雷达定位位姿 |
| Pub | `cmd_vel` | `geometry_msgs/Twist` | 自动导航速度 |
| Pub | `nav_status` | `std_msgs/Bool` | 当前障碍点到达 |
| Pub | `nav_finished` | `std_msgs/Bool` | 障碍点序列结束 |
| Pub | `obstacle_nav/current_pose` | `std_msgs/Float32MultiArray` | 调试：当前雷达位姿 |
| Pub | `obstacle_nav/current_target` | `std_msgs/Float32MultiArray` | 调试：当前目标点 |
| Pub | `obstacle_nav/debug_error` | `geometry_msgs/Point` | 调试：机体系误差 |

### `task_executor_node`

| 方向 | 话题 | 类型 | 作用 |
| --- | --- | --- | --- |
| Sub | `current_state` | `std_msgs/String` | 根据状态切换执行模式 |
| Sub | `/tf` | `tf2_msgs/TFMessage` | 木桥、上楼梯、绕杆使用 |
| Sub | `qr_detection_result` | `quad/msg/QrResult` | 二维码视觉伺服输入 |
| Pub | `cmd_vel` | `geometry_msgs/Twist` | 视觉/障碍任务速度 |
| Pub | `task_done` | `std_msgs/Int32` | 任务 ID 或任务完成信号 |
| Pub | `lidar_pose_xyyaw` | `geometry_msgs/Point` | 通用雷达定位：x、y、yaw |

---

## 5. 坐标系约定

### 5.1 雷达系

雷达位姿来自：

```text
camera_init -> aft_mapped
```

约定：

```text
x 向右为正
y 向前为正
yaw 为雷达系下绝对朝向
```

`task_executor_node` 会将该位姿发布到 `lidar_pose_xyyaw`，消息类型为 `geometry_msgs/Point`：

```text
point.x = x
point.y = y
point.z = yaw
```

雷达系目标点和轨迹均使用：

```text
x y yaw
```

单位为：

```text
m, m, rad
```

### 5.2 机体系

`cmd_vel` 按机体系解释：

```text
linear.x   向前为正
linear.y   向左为正
angular.z  逆时针为正
```

雷达系误差到机体系误差的变换：

```cpp
dx_body =  cos(yaw) * dy_world - sin(yaw) * dx_world;
dy_body = -cos(yaw) * dx_world - sin(yaw) * dy_world;
```

### 5.3 QR 视觉系

二维码视觉坐标约定：

```text
x 向右为正，单位 mm
y 向前为正，单位 mm
yaw 顺时针为正，单位 deg
```

变换到机体系：

```cpp
err_body_x_m =  err_qr_y_mm / 1000.0;
err_body_y_m = -err_qr_x_mm / 1000.0;
err_body_yaw_rad = -err_qr_yaw_deg * pi / 180.0;
```

---

## 6. 配置文件

| 文件 | 作用 |
| --- | --- |
| `config/obstacle_nav_params.yaml` | 障碍点自动导航参数，支持点序列和轨迹序列 |
| `config/task_executor_params.yaml` | 视觉伺服、木桥、上楼梯、绕杆等执行器参数 |
| `config/obstacle_nav_trajectory_example.txt` | 障碍点导航轨迹示例 |
| `config/pole_trajectory_example.txt` | 绕杆任务轨迹示例 |
| `config/bridge_cross_params.yaml` | 早期过桥参数文件，当前建议以 `task_executor_params.yaml` 为主 |

---

## 7. 编译

在 ROS 2 工作区根目录执行：

```bash
colcon build --packages-select task_manager
source install/setup.bash
```

如果提示找不到 `quad`：

```text
Could not find a package configuration file provided by "quad"
```

说明当前环境中缺少自定义消息包 `quad`，需要先将其加入工作区或正确 source 对应环境。

---

## 8. 启动

分别启动三个节点：

```bash
ros2 run task_manager task_manager_node
```

```bash
ros2 run task_manager obstacle_nav_node \
  --ros-args --params-file src/commands/task_manager/config/obstacle_nav_params.yaml
```

```bash
ros2 run task_manager task_executor_node \
  --ros-args --params-file src/commands/task_manager/config/task_executor_params.yaml
```

实际部署时请根据安装路径或 launch 文件调整参数文件路径。

---

## 9. 当前实现边界

已实现：

- `AUTO_NAV` 阶段障碍点序列导航；
- 点序列 `POINT_SEQUENCE`；
- 轨迹序列 `TRAJECTORY_SEQUENCE`；
- 到达单个障碍点后触发 `QR_RECOGNITION`；
- 全部障碍点完成后 `nav_finished -> IDLE`；
- 视觉伺服独立二维码偏置参数；
- 木桥、上楼梯、绕杆闭环控制；
- 高墙、沙坑固定动作序列。

暂未实现或建议后续扩展：

- 前视点平滑轨迹跟踪；
- 运行时动态追加障碍点；
- 更细粒度的导航状态枚举消息；
- `debug_state_cmd` 对 `STAIR` / `POLE` 的直接调试入口。
