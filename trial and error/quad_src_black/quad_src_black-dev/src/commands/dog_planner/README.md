# quadruped_planner_ros2 (dog_planner)

一个面向四足机器人的 ROS2 路径规划功能包。该系统采用分层架构设计，将规划任务拆分为全局路径规划（“去哪”）和局部路径规划及控制（“怎么走 + 避障 + 直接输出 cmd_vel”）。

本功能包同时提供了 **手写轻量级 DWA** 和 **ROS2 官方 Nav2 DWB 插件** 两种局部规划实现，你可以根据系统性能和需求灵活切换。

---

## 🎯 系统架构与目标

```text
SLAM (Map/Odom) → Global Planner (A*) → Local Planner (DWA/DWB) → cmd_vel (Twist)
```

本规划系统包含三个主要节点：
1. **Global Planner (`global_planner_node`)**
   - 算法：**A*** (栅格地图搜索)
   - 职责：基于 `OccupancyGrid` 和 `Odometry` 计算到达目标的全局路径。
   
2. **Local Planner - 手写轻量版 (`local_planner_node`)**
   - 算法：**DWA** (Dynamic Window Approach)
   - 职责：极简实现，不依赖外部 Nav2 栈。跟随全局路径，结合实时 `/scan` 避障，并在速度空间中采样模拟，直接输出 `geometry_msgs/Twist`。

3. **Local Planner - Nav2 官方版 (`local_planner_node_nav2`)**
   - 算法：**DWB** (`dwb_core::DWBLocalPlanner` 插件)
   - 职责：作为一个桥接节点，将 A* 计算的 `/global_path` 转发给官方的 `nav2_controller_server`。该方案提供了完整的 `Costmap2D` 避障层、评分函数和恢复机制，功能更全面。

---

## 📁 功能包结构

```text
dog_planner/
├── launch/
│   ├── planner.launch.py            # 启动手写极简版 A* + DWA
│   └── nav2_planner.launch.py       # 启动 A* + Nav2 官方 DWB
├── config/
│   ├── planner.yaml                 # 极简版全局和局部规划器参数
│   └── nav2_params.yaml             # Nav2 local_costmap 及 dwb_core 参数
├── src/
│   ├── global_planner_node.cpp      # A* 全局规划实现
│   ├── local_planner_node.cpp       # 手写轻量级 DWA 实现
│   └── local_planner_node_nav2.cpp  # Nav2 Action 桥接接口
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🔗 节点接口说明

### Topics

| Topic          | 类型                                | 方向      | 说明                       |
| -------------- | ----------------------------------- | --------- | -------------------------- |
| `/map`         | `nav_msgs/OccupancyGrid`            | Subscribe | 来自 SLAM 系统的全局代价地图 |
| `/odom`        | `nav_msgs/Odometry`                 | Subscribe | 机器人的当前位姿和速度估算   |
| `/scan`        | `sensor_msgs/LaserScan`             | Subscribe | 2D 激光雷达点云，用于局部避障 |
| `/goal`        | `geometry_msgs/PoseStamped`         | Subscribe | 用户下发的目标点位姿         |
| `/global_path` | `nav_msgs/Path`                     | Publish   | Global Planner 输出的全局路径 |
| `/cmd_vel`     | `geometry_msgs/Twist`               | Publish   | Local Planner 输出的底盘控制指令 |

### Actions (Nav2 模式下)
- `local_costmap/follow_path` (`nav2_msgs/FollowPath`): `local_planner_node_nav2` 调用的 Nav2 Action，触发 `nav2_controller_server` 运行 DWB。

---

## ⚙️ 关键参数配置

### 1. 手写轻量版配置 (`config/planner.yaml`)
- `resolution` & `heuristic_weight`: A* 的地图分辨率和启发式权重。
- `max_vel` / `max_omega`: DWA 的运动学最大速度约束。
- `w_path`, `w_obstacle`, `w_goal`, `w_speed`: 决定避障、寻路倾向的 DWA 评分项权重。

### 2. Nav2 官方版配置 (`config/nav2_params.yaml`)
- **`local_costmap`**: 配置了局部 `ObstacleLayer` 和 `InflationLayer`，负责融合 `/scan` 数据并膨胀障碍物。
- **`controller_server`**: 
  - `FollowPath` 插件指向 `dwb_core::DWBLocalPlanner`。
  - `critics`: `RotateToGoal`, `Oscillation`, `BaseObstacle`, `GoalAlign`, `PathAlign`, `PathDist`, `GoalDist`。可以根据四足特性精细调参。

---

## 🚀 编译与运行

### 1. 编译工作空间
**前置要求**：如果使用 Nav2 版本，需确保已安装 `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-dwb-core`。

```bash
cd ~/your_ws
colcon build --packages-select dog_planner
source install/setup.bash
```

### 2. 启动节点

你可以选择两种模式之一运行：

**模式一：手写轻量版 A* + DWA**
极简设计，资源占用低，适合快速测试。
```bash
ros2 launch dog_planner planner.launch.py
```

**模式二：A* + 官方 Nav2 DWB**
功能完善，支持复杂的 Costmap 配置和多插件评分。
```bash
ros2 launch dog_planner nav2_planner.launch.py
```

### 3. 测试流程
1. 确保上游的 SLAM 节点已经启动，并在 `/map` 和 `/odom` 发布数据。
2. 确保雷达数据在 `/scan` 上正常发布。
3. 在 RViz 中使用 **"2D Goal Pose"** 工具，发布目标点至 `/goal` 话题。
4. `global_planner_node` 将计算 `/global_path` 并在 RViz 中显示。
5. 选择的 Local Planner (DWA 或 DWB) 将开始计算速度指令，并输出 `/cmd_vel` 控制机器人移动。
