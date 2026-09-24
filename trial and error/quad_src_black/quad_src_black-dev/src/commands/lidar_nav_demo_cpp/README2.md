# Lidar Nav Demo Node (C++)

这是一个用于四足机器人自动导航的 ROS 2 节点（C++版本）。它依赖激光雷达提供的 TF 坐标变换树，根据用户配置的一系列取货与放货点，自动规划并执行一套状态机驱动的平滑移动路线。

---

## 🌟 核心特性

- **动态路径生成机制**：节点在启动时不会盲目跑向目标，而是订阅优先级的通信话题，依据外部视觉节点提供的“物资优先级”数组动态决定拿取物资的先后顺序。
- **安全的区域过渡逻辑**：机器人能够智能区分“前排(Front)”和“后排(Back)”。跨排移动时，必然会通过“中转站(Mid)”进行安全过渡，防止直接切区时碰撞场地障碍物。
- **雷达平移补偿**：如果雷达并非安装在底盘的旋转中心，可以配置 `lidar_offset_x` 和 `lidar_offset_y`。在进行角度纠正时，节点会自动补偿由偏心旋转带来的额外位置线速度需求，大幅提高对准精度。
- **闭环握手控制**：不独立抢占控制权，而是订阅 `/quad/nav_enable`，由上层的**总控任务状态机**来决定何时开启导航（`AUTO_NAV`），并在到达每个放货或取货点时发布 `/nav_status` 进行到达通知握手。

---

## 📡 话题接口 (Topics)

### 订阅 (Subscribers)
* **`/tf`** (`tf2_msgs/msg/TFMessage`)
  监听雷达输出的世界系到机器人基座的坐标系变换关系（默认为 `camera_init` -> `aft_mapped`）。
* **`/quad/nav_enable`** (`std_msgs/msg/Bool`)
  导航主开关，由外部的任务状态机发布 `true` 以激活本节点的 PID 控制循环。如果收到 `false`，则立刻交出控制权（不发布速度）。
* **`/quad/material_priorities`** (`std_msgs/msg/Int32MultiArray`)
  **[新增]** 物资拿取优先级触发器。传入一个代表各取货点优先级的整数数组（数值越大越先去拿）。接收到此消息后，系统才会正式生成路径规划序列。
* **`/lidar_nav_control`** (`std_msgs/msg/Float32MultiArray`)
  允许通过此话题向导航系统下发一个临时强制点 `[enable, x, y, yaw]`，用于覆盖当前整个列队任务进行紧急移动。

### 发布 (Publishers)
* **`/quad/cmd_vel`** (`geometry_msgs/msg/Twist`)
  输出的底盘速度控制指令（包含机体系下的 $v_x$, $v_y$ 线速度和 $w_z$ 角速度）。
* **`/quad/current_pose`** (`std_msgs/msg/Float32MultiArray`)
  调试用：向外发布当前计算得到的机器人位姿 `[x, y, yaw]`。
* **`/quad/current_target`** (`std_msgs/msg/Float32MultiArray`)
  调试用：向外发布机器人正在前往的当前局部目标点位姿 `[x, y, yaw]`。
* **`nav_status`** (`std_msgs/msg/Bool`)
  状态反馈握手：当机器人成功到达一个“取货”或“放货”点，或者全部任务序列执行结束时，会向该话题发布 `true`，通知上层状态机可以进行视觉微调或其它抓取操作了。

---

## ⚙️ 核心参数 (Parameters)

可以通过 launch 文件加载 YAML 参数或命令行直接赋值。

### 控制与限幅参数
* **`kp_xy`** (double, 默认: 0.5): X/Y 平面位置 PID 的比例系数。
* **`kp_yaw`** (double, 默认: 1.0): Yaw 角 PID 的比例系数。
* **`max_vx`, `max_vy`, `max_dyaw`** (double, 默认: 0.5): 最大允许下发的前后/左右线速度与角速度。
* **`control_rate`** (double, 默认: 50.0): `control_loop` 定时器的运行频率 (Hz)。
* **`wait_time`** (double, 默认: 2.0): 到达 `PICKUP`(取货) 或 `DROPOFF` (放货) 目标点后，强制停留等待秒数。

### 雷达安装参数
* **`lidar_offset_x`** (double, 默认: 0.0): 雷达物理中心相对机器人运动底盘中心的前向偏移量（米，向前为正）。
* **`lidar_offset_y`** (double, 默认: 0.0): 雷达物理中心相对机器人运动底盘中心的左向偏移量（米，向左为正）。

### 场地预设航点配置
* **`transition_front`** (double array): 前排过渡点坐标 `[x, y, yaw]`
* **`transition_mid`** (double array): 中间过渡点坐标 `[x, y, yaw]`
* **`transition_back`** (double array): 后排过渡点坐标 `[x, y, yaw]`
* **`pickup_points`** (double array): 所有的取货点坐标及其前后排属性。格式为扁平数组，每4个元素代表一个点：`[x, y, yaw, is_front(>0.5为前排)]`。
* **`dropoff_points`** (double array): 所有的放货坑位坐标。格式为扁平数组，每3个元素代表一个点：`[x, y, yaw]`。

---

## 🚀 运行与测试方法

1. 启动节点及参数加载（假设你编写好了 `launch` 脚本）：
   ```bash
   ros2 launch lidar_nav_demo_cpp lidar_nav.launch.py
   ```
   *此时由于没有路径规划和上层使能，节点会处于闲置监听状态并提示 `Waiting for material priorities to generate path...`。*

2. **发送优先级数组（触发路径生成）**：
   假如配置了 3 个物资取货点，想要按照顺序 `点2 -> 点3 -> 点1` 的顺序拿取，可以给第 2 个点最高优先级：
   ```bash
   ros2 topic pub /quad/material_priorities std_msgs/msg/Int32MultiArray "{data: [10, 50, 30]}" -1
   ```
   *此时节点终端将输出 `Path generated! Sorted indices: [1, 2, 0]`。*

3. **使能导航（下发控制权）**：
   通过 `task_state_machine` 节点自动发送，或者在调试时手动发送：
   ```bash
   ros2 topic pub /quad/nav_enable std_msgs/msg/Bool "{data: true}" -1
   ```
   *机器人将立即开始沿计算好的路径移动。*
