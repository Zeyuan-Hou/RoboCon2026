# Lidar Nav Demo Node (雷达导航任务节点)

本包提供了一个基于激光雷达定位 (`/tf`) 的自主导航和任务调度节点 (`LidarNavControl`)。该节点主要设计用于机器人完成复杂的物流搬运任务，同时提供了一个独立的直线运动调试模式，用于测试底盘的闭环运动精度。

任务赛取放物块的完整联调说明见 [`TASK_RACE_DEBUG_GUIDE.md`](TASK_RACE_DEBUG_GUIDE.md)。

## 🌟 核心特性与双模式架构

节点通过 YAML 参数在三种任务模式间切换（优先级：`auto_parku_enable` > `straight_line_enable` > 物流默认）。

### 1. 🎯 模式一：物流搬运主任务 (Logistics Task Mode)
**触发条件**: `auto_parku_enable: false` 且 `straight_line_enable: false` (默认)

这是一个集成了视觉识别和复杂路径规划的完整任务状态机：
- **原地预扫描 (Pre-Path Spin)**: 在未生成路径前，机器人可执行多段原地旋转，用于 SLAM 初始化或视觉标签扫描。
- **视觉结果锁定**: 
  - 监听算术题识别结果 (`/stable_arithmetic_result`)，连续多次稳定后锁定目标物资类别。
  - 监听场地物资分布 (`eightboxes`) 获取 8 个位置的物资类别。
- **动态路径规划**: 
  - 提取符合目标类别的物资点，并根据优先级降序排序。
  - 智能插入过渡点（前排过渡、后排过渡、中间过渡），规划出一条无碰撞的 取货 -> 放货 完整轨迹。
- **精准靠泊与等待**: 到达取放货点后自动停车，并等待指定的 `wait_time`（模拟机械臂抓取/放置）。
- **外部机械臂接管**: `external_manipulation_enable=true` 时，到达 `PICKUP` / `DROPOFF` 后发布 `/quad/logistics_nav_event`，由 `task_manager_node` 切站立、发布机械臂动作命令、等待完成后恢复 RL 继续导航。

### 2. 📼 模式二：AUTO_PARKU 轨迹回放 (Recorded Path Replay)
**触发条件**: `auto_parku_enable: true`

按录制好的 `.txt` 文件时间轴回放路径，用于复现先前行走轨迹：
- **轨迹格式**: 每行 `time_sec x y yaw`（空格分隔），`yaw` 为雷达系航向角；`#` 开头为注释。
- **时间轴**: 首行时间归一化为 0，回放使用墙钟 `elapsed` 在相邻点间线性插值。
- **闭环控制**: 与物流模式相同的 map→body 误差变换 + 机体系 PD（`kp_*` / `kd_*` / `lin_vel_creep_min`），输出 `/quad/cmd_vel`。
- **完成**: 轨迹时间结束后发零速，`nav_status=true`。

```yaml
auto_parku_enable: true
auto_parku_trajectory_path: "/path/to/recorded_path.txt"
```

示例文件见 `config/trajectories/example_path.txt`。

### 3. 📏 模式三：雷达纠正定航向直线模式 (Straight Line Mode)
**触发条件**: `auto_parku_enable: false` 且 `straight_line_enable: true`

专为底盘运动学测试和特定简单任务设计的闭环直线运动：
- **起点锁定**: 激活后记录当前坐标为起点。
- **解耦 PID 闭环控制**:
  - **航向恒定 (Yaw)**: 实时纠正车头朝向，保证其等于设定的 `straight_target_yaw`。
  - **前向驱动 (Forward)**: 驶向目标距离 `straight_target_dist`，末端平滑减速。
  - **横向抗漂移 (Cross-track)**: 计算车体偏离理想直线的垂直距离，施加侧向速度 (`linear.y`) 强行拉回，完美克服麦克纳姆轮/全向轮侧滑。

---

## 🔌 数据接口 (ROS 2 Topics)

### 📥 订阅的话题 (Subscribers)
- **`/tf`** (`tf2_msgs/msg/TFMessage`): 机器人的实时全局位姿反馈。
- **`/quad/logistics_nav_enable`** (`std_msgs/msg/Bool`): 任务赛导航使能开关，决定本节点是否输出控制指令。
- **`/stable_arithmetic_result`** (`std_msgs/msg/String`): OCR 算术题识别结果。
- **`eightboxes`** (`std_msgs/msg/String`): 场地物资分布字符串。
- **`/lidar_nav_control`** (`std_msgs/msg/Float32MultiArray`): 强制接管接口 `[enable, x, y, yaw]`。

### 📤 发布的话题 (Publishers)
- **`/quad/cmd_vel`** (`geometry_msgs/msg/Twist`): 底盘速度控制指令。
- **`nav_status`** (`std_msgs/msg/Bool`): 导航状态反馈。到达目标或任务完成时发送 `true` 通知上层。
- **`/quad/current_pose`** (`std_msgs/msg/Float32MultiArray`): 实时位姿 `[x, y, yaw]`。
- **`/quad/current_target`** (`std_msgs/msg/Float32MultiArray`): 当前追踪的目标航点 `[x, y, yaw]`。
- **`boxsuc`** (`std_msgs/msg/String`): 握手信号，路径生成成功后连发 10 次 "666"。
- **`/quad/logistics_nav_event`** (`std_msgs/msg/Int32MultiArray`): 任务赛到点事件 `[event_type, waypoint_index, seq, action_id]`，其中 `1=PICKUP_REACHED`、`2=DROPOFF_REACHED`、`3=LOGISTICS_FINISHED`。`DROPOFF` 的 `action_id` 表示物理放置点 ID。

---

## ⚙️ 核心参数配置 (Parameters)

所有的参数均在 `config/waypoints.yaml` 中配置。

### 基础与运动学参数
```yaml
kp_xy: 3.5            # 平移 PID
kp_yaw: 3.0           # 旋转 PID
max_vx: 0.5           # 最大前进速度 (m/s)
max_vy: 0.45          # 最大横向速度 (m/s)
max_dyaw: 0.8         # 最大旋转角速度 (rad/s)
enable_control: true  # 启动时是否默认使能
enable_topic: "/quad/logistics_nav_enable"
external_manipulation_enable: true
```

### AUTO_PARKU 参数
```yaml
auto_parku_enable: false
auto_parku_trajectory_path: ""   # 绝对路径或相对 launch 工作目录
```

### 直线模式参数 (Straight Line Mode)
```yaml
straight_line_enable: false    # 设置为 true 开启直线模式
straight_target_yaw: 0.0       # 期望保持的绝对航向角 (rad)
straight_target_dist: 2.0      # 期望前进的总距离 (m)
straight_max_speed: 0.5        # 直线模式下的最大前向速度 (m/s)
straight_kp_y: 2.0             # 横向漂移纠正 PID 参数
```

### 物流模式路径参数 (Logistics Mode)
```yaml
wait_time: 2.0        # 到达 取货/放货 点后的等待时间(秒)
transition_front: [0.0, 1.15, 0.0]  # 前排过渡点 [x, y, yaw]
transition_back:  [0.29, 4.00, 0.0] # 后排过渡点
transition_mid:   [0.08, 2.67, 0.0] # 中间过渡点
pickup_points: [...]  # 取货点列表，格式为 [x, y, yaw, is_front]
dropoff_points: [...] # 放货点列表，格式为 [x, y, yaw]
```

---

## 🚀 启动指南

1. **修改配置**: 编辑 `config/waypoints.yaml`，选择模式（`auto_parku_enable` / `straight_line_enable`）。
2. **编译与运行**:
   ```bash
   colcon build --packages-select lidar_nav_demo_cpp
   source install/setup.bash
   ros2 launch lidar_nav_demo_cpp lidar_nav.launch.py
   ```
3. **使能控制**: 任务赛中由 `task_manager_node` 通过 `/quad/logistics_nav_enable` 发送 `true`；单节点调试时也可以在 yaml 中临时开启 `enable_control: true`。

4. **机械臂流程**: 推荐让 `task_manager_node` 处理取/放物块。配置 `mission_mode: "logistics"` 后，`task_manager_node` 会在收到 `/quad/logistics_nav_event` 时切站立、向 `/manipulator/cmd` 发布一次命令，等待完成后切回 RL 并恢复导航。同一放置点第一次放低层，第二次及以后放高层。

git add -A          # 所有改动（慎用，先 git status 看一眼）
git status          # 确认没有不该提交的
git commit -m "feat: 任务赛完整流程"
git push origin dev
