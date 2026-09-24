# NavController 输入输出与模块交互说明

## 1. 目标

`NavController` 是任务赛物流导航中抽出来的纯控制器接口。

它的目标是把“速度怎么计算”从 ROS 节点里拆出来，方便之后替换控制器。

当前已有两个实现：

```text
BodyPdNavController
  旧版机体系 PD 控制器。

HeadingDockNavController
  当前默认的 APPROACH / ALIGN / DOCK 分阶段控制器。
```

相关文件：

```text
接口定义:
  src/commands/lidar_nav_demo_cpp/include/lidar_nav_demo_cpp/nav_controller.hpp

实现:
  src/commands/lidar_nav_demo_cpp/src/nav_controller.cpp

节点接入:
  src/commands/lidar_nav_demo_cpp/src/lidar_nav_control.cpp
  src/commands/lidar_nav_demo_cpp/src/lidar_nav_demo_node.cpp

参数:
  src/commands/lidar_nav_demo_cpp/config/waypoints.yaml
```

## 2. 职责边界

`NavController` 只负责一件事：

```text
输入当前目标误差
  ↓
计算 cmd_vel
  ↓
返回 Twist 和调试信息
```

它不负责：

```text
订阅 /tf
发布 /quad/cmd_vel
读取 YAML 参数
生成 waypoint
判断是否到点
发布 PICKUP_REACHED / DROPOFF_REACHED 事件
控制 task_manager 或机械臂
```

这些仍由 `LidarNavControl` 和 `task_manager` 负责。

## 3. 输入 NavControllerInput

控制器输入结构定义在 `nav_controller.hpp`：

```cpp
struct NavControllerInput {
    double err_x_body;
    double err_y_body;
    double err_yaw;
    double dist;
    double dt;
    std::size_t track_id;
    WaypointType waypoint_type;
    bool enabled;
};
```

字段含义：

```text
err_x_body
  目标在机体系前后方向的误差，单位 m。
  > 0 表示目标在前方，通常对应正 vx。

err_y_body
  目标在机体系左右方向的误差，单位 m。
  > 0 表示目标在左侧。

err_yaw
  目标 yaw 与当前 yaw 的差值，单位 rad。
  已经被归一化到 [-pi, pi]。

dist
  平面距离误差，单位 m。
  当前由 LidarNavControl 计算:
    hypot(err_x_body, err_y_body)

dt
  本次控制周期距离上一控制周期的时间，单位 s。
  用于 D 项或后续速度平滑。

track_id
  当前跟踪目标 ID。
  物流导航中使用 current_wp_idx_。
  换 waypoint 时 track_id 会变化，控制器会重置上一帧误差，避免 D 项串扰。

waypoint_type
  当前 waypoint 类型:
    START
    TRANSITION
    PICKUP
    DROPOFF
  heading_dock 在 DOCK 阶段会根据 PICKUP/DROPOFF 使用更明确的末端 yaw 增益。

enabled
  当前导航是否使能。
  false 时控制器返回零速度并 reset 内部状态。
```

输入来源：

```text
LidarNavControl::control_loop()
  从当前 waypoint 和 TF 位姿计算 map 误差
  调用 map_error_to_body() 得到 err_x_body / err_y_body
  计算 dist / dt
  填充 NavControllerInput
  调用 logistics_controller_->compute(input)
```

## 4. 输出 NavControllerOutput

控制器输出结构：

```cpp
struct NavControllerOutput {
    geometry_msgs::msg::Twist cmd;
    NavControlStage stage;
    double bearing_error;
};
```

字段含义：

```text
cmd
  速度控制输出。
  LidarNavControl 会直接发布到 /quad/cmd_vel。

  cmd.linear.x:
    机体系前后速度 vx，单位 m/s。

  cmd.linear.y:
    机体系左右速度 vy，单位 m/s。

  cmd.angular.z:
    yaw 角速度 wz，单位 rad/s。

stage
  当前控制阶段。

  BodyPdNavController:
    BODY_PD

  HeadingDockNavController:
    APPROACH
    ALIGN
    DOCK

  LidarNavControl 会把 stage 打进日志，便于调参。

bearing_error
  目标方位角误差，单位 rad。
  当前主要由 heading_dock 计算:
    atan2(err_y_body, err_x_body)
  用于调试目标在机器人前方/侧方的程度。
```

输出去向：

```text
NavControllerOutput
  ↓
LidarNavControl::control_loop()
  ↓
cmd_vel_pub_->publish(output.cmd)
  ↓
/quad/cmd_vel
  ↓
底盘/RL 速度接口执行
```

## 5. 配置 NavControllerConfig

控制器配置结构：

```cpp
struct NavControllerConfig {
    // body_pd 参数
    kp_x, kp_y, kp_yaw;
    kd_x, kd_y, kd_yaw;
    lin_vel_creep_min;
    max_vx, max_vy, max_wz;
    yaw_first_threshold;

    // heading_dock 阶段距离
    approach_to_align_dist;
    dock_start_dist;

    // heading_dock 限速
    approach_max_vx, approach_max_vy, approach_max_wz;
    align_max_vx, align_max_vy, align_max_wz;
    dock_max_vx, dock_max_vy, dock_max_wz;

    // heading_dock 增益与 creep
    kp_forward;
    kp_bearing;
    kp_final_yaw;
    kp_lateral_to_yaw;
    kp_dock_x;
    kp_dock_y;
    kp_dock_yaw;
    lateral_trim_gain;
    max_vy_trim;
    approach_creep_min;
    dock_creep_min;
    approach_final_yaw_weight;
};
```

配置来源：

```text
waypoints.yaml
  ↓
LidarNavControl::LidarNavControl()
  declare_parameter()
  get_parameter()
  填充 logistics_controller_config_
  ↓
make_nav_controller(logistics_controller_type_, logistics_controller_config_)
```

注意：

```text
NavController 本身不读取 ROS 参数。
这样做是为了保持控制器纯净，方便单独测试和替换。
```

## 6. 控制器创建与切换

创建函数：

```cpp
std::unique_ptr<NavController> make_nav_controller(
    const std::string& controller_type,
    const NavControllerConfig& config);
```

当前逻辑：

```text
controller_type == "body_pd"
  -> 创建 BodyPdNavController

其他情况
  -> 创建 HeadingDockNavController
```

YAML 切换方式：

```yaml
logistics_controller_type: "heading_dock"
```

或：

```yaml
logistics_controller_type: "body_pd"
```

如果填了未知字符串，`LidarNavControl` 会打印 warning，并回退到 `heading_dock`。

## 7. 内部状态

`NavController` 不是完全无状态，它保存上一帧误差：

```cpp
bool initialized_;
std::size_t track_id_;
double prev_err_x_body_;
double prev_err_y_body_;
double prev_err_yaw_;
```

用途：

```text
计算 D 项:
  dx/dt
  dy/dt
  dyaw/dt

在 waypoint 切换时重置上一帧误差:
  防止上一目标的误差影响新目标。
```

重置触发：

```text
input.enabled == false
track_id 改变
LidarNavControl 主动调用 logistics_controller_->reset()
```

当前 `LidarNavControl` 会在这些场景主动 reset：

```text
导航未使能
到达 waypoint
重新生成物流路径
收到 /lidar_nav_control 临时目标
```

## 8. 和 LidarNavControl 的交互

`LidarNavControl` 是控制器的直接调用者。

调用位置：

```text
src/commands/lidar_nav_demo_cpp/src/lidar_nav_control.cpp
  LidarNavControl::control_loop()
```

交互流程：

```text
1. LidarNavControl 选择当前 waypoint。
2. LidarNavControl 根据 TF 位姿计算 err_x_map / err_y_map / err_yaw。
3. LidarNavControl 调用 map_error_to_body() 得到 err_x_body / err_y_body。
4. LidarNavControl 先做 update_arrival_debounce()。
5. 如果未到点，LidarNavControl 填充 NavControllerInput。
6. 调用 logistics_controller_->compute(input)。
7. LidarNavControl 发布 output.cmd 到 /quad/cmd_vel。
8. LidarNavControl 把 output.stage 打进日志。
```

关键边界：

```text
到点判定在 LidarNavControl，不在 NavController。
cmd_vel 发布在 LidarNavControl，不在 NavController。
物流事件发布在 LidarNavControl，不在 NavController。
```

## 9. 和 waypoints.yaml 的交互

`waypoints.yaml` 不直接被控制器读取，而是被 `LidarNavControl` 读取。

相关参数分组：

```text
物流航点控制器总开关:
  logistics_controller_type
  control_rate

body_pd 纯 PD 控制参数:
  kp_x / kp_y / kp_yaw
  kd_x / kd_y / kd_yaw
  lin_vel_creep_min
  max_vx / max_vy / max_dyaw
  yaw_first_threshold

heading_dock 分阶段控制参数:
  approach_to_align_dist
  dock_start_dist
  approach_max_*
  align_max_*
  dock_max_*
  kp_forward
  kp_bearing
  kp_final_yaw
  kp_lateral_to_yaw
  kp_dock_*
  lateral_trim_gain
  max_vy_trim
  approach_creep_min
  dock_creep_min
  approach_final_yaw_weight
```

注意：

```text
到点阈值参数不属于 NavController。
它们属于 LidarNavControl 的到点判定层。
```

到点阈值包括：

```text
transition_position_tolerance
transition_yaw_tolerance
manip_position_tolerance
manip_yaw_tolerance
arrival_stable_time_s
```

## 10. 和 task_manager 的交互

`NavController` 不直接和 `task_manager` 交互。

实际交互链路是：

```text
NavController
  -> 输出 cmd_vel
  -> LidarNavControl 发布 /quad/cmd_vel
  -> 机器人移动
  -> LidarNavControl 判断到达 PICKUP/DROPOFF
  -> LidarNavControl 发布 /quad/logistics_nav_event
  -> task_manager 接收事件
  -> task_manager 切站立并触发机械臂
```

因此：

```text
控制器只影响机器人如何到点。
task_manager 只关心是否已经到点，以及到的是 PICKUP 还是 DROPOFF。
```

事件格式仍保持：

```text
[event_type, waypoint_index, seq, action_id]
```

## 11. 新增控制器时怎么接入

如果后续要新增控制器，例如 `mpc_dock`：

```text
1. 在 nav_controller.hpp 中新增类:
   class MpcDockNavController : public NavController

2. 在 nav_controller.cpp 中实现:
   NavControllerOutput MpcDockNavController::compute(...)

3. 在 make_nav_controller() 中注册:
   if (controller_type == "mpc_dock") return std::make_unique<MpcDockNavController>(config);

4. 在 waypoints.yaml 中增加:
   logistics_controller_type: "mpc_dock"

5. 如需新参数:
   在 NavControllerConfig 增加字段
   在 LidarNavControl::LidarNavControl() 中 declare/get 参数
   在 waypoints.yaml 写参数和注释
```

只要仍遵守 `NavControllerInput -> NavControllerOutput` 接口，`LidarNavControl` 的 ROS I/O、到点判定和任务事件逻辑就不需要改。
