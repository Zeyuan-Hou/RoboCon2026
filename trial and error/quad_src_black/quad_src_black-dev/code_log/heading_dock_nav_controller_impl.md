# heading_dock 分阶段导航控制器实现说明

## 1. 文件与作用范围

当前任务赛物流导航有两个可选控制器：

```yaml
logistics_controller_type: "heading_dock"
```

可选值：

```text
heading_dock: 当前默认，分阶段少横移控制。
body_pd: 旧版机体系 PD 控制，用于快速回退或对比。
```

相关文件：

```text
控制器接口:
  src/commands/lidar_nav_demo_cpp/include/lidar_nav_demo_cpp/nav_controller.hpp

控制器实现:
  src/commands/lidar_nav_demo_cpp/src/nav_controller.cpp

ROS 节点接入:
  src/commands/lidar_nav_demo_cpp/src/lidar_nav_control.cpp
  src/commands/lidar_nav_demo_cpp/src/lidar_nav_demo_node.cpp

参数:
  src/commands/lidar_nav_demo_cpp/config/waypoints.yaml
```

作用范围：

```text
只影响物流多航点模式:
  TRANSITION
  PICKUP
  DROPOFF

不影响:
  AUTO_PARKU 轨迹回放
  straight_line_enable 直线模式
  task_manager 状态机事件协议
```

## 2. 总体数据流

物流导航主循环仍在 `LidarNavControl::control_loop()` 中。节点负责 ROS 相关工作，控制器只负责速度计算。

完整数据流可以按 8 层理解。每一层都标明处理主体和对应文件：

```text
1. ROS 输入层
   处理主体:
     LidarNavControl 节点

   对应文件:
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_callbacks.cpp
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_demo_node.cpp

   /tf
     -> tf_callback()
     -> 更新当前位姿 x_, y_, yaw_, got_tf_

   /quad/logistics_nav_enable
     -> enable_sub_ 回调
     -> 更新 enable_control_

   /eightboxes + /stable_arithmetic_result
     -> priority_callback() / ocr_result_callback()
     -> generate_path()
     -> 生成 waypoints_

2. 主循环入口
   处理主体:
     LidarNavControl 节点

   对应文件:
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_control.cpp

   LidarNavControl::control_loop()
     -> 先发布 /quad/current_pose
     -> 检查 enable_control_
     -> 检查预扫描/路径生成状态
     -> 检查 got_tf_
     -> 检查 current_wp_idx_ 是否越界

3. 当前目标选择
   处理主体:
     LidarNavControl 节点

   对应文件:
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_control.cpp
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_path.cpp

   current_wp = waypoints_[current_wp_idx_]

   current_wp 包含:
     x
     y
     yaw
     type: TRANSITION / PICKUP / DROPOFF
     action_id: DROPOFF 时用于告诉 task_manager 放置点下标

   节点同时发布:
     /quad/current_target = [target_x, target_y, target_yaw]

4. map 系误差计算
   处理主体:
     LidarNavControl 节点

   对应文件:
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_control.cpp

   err_x_map = target_x - x_
   err_y_map = target_y - y_
   err_yaw   = normalize_angle(target_yaw - yaw_)

   这里的 x_/y_/yaw_ 来自 TF。
   target_x/target_y/target_yaw 来自当前 waypoint。

5. map 误差转机体系误差
   处理主体:
     LidarNavControl 节点

   对应文件:
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_auto_parku.cpp

   map_error_to_body(err_x_map, err_y_map, err_x_body, err_y_body)

   输出:
     err_x_body: 目标在机器人前后方向的误差
     err_y_body: 目标在机器人左右方向的误差

   然后计算:
     dist = hypot(err_x_body, err_y_body)

   注意:
     map_error_to_body() 内部会叠加 lidar_offset_x / lidar_offset_y。
     因此控制器看到的是补偿雷达安装偏置后的机体系误差。

6. 到点判定层
   处理主体:
     LidarNavControl 节点

   对应文件:
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_control.cpp

   get_arrival_tolerances(current_wp.type, position_tolerance, yaw_tolerance)

   如果是 TRANSITION:
     使用 transition_position_tolerance
     使用 transition_yaw_tolerance

   如果是 PICKUP / DROPOFF:
     使用 manip_position_tolerance
     使用 manip_yaw_tolerance

   update_arrival_debounce(dist, err_yaw, current_wp_idx_, position_tolerance, yaw_tolerance)
     -> dist 和 yaw 连续满足 arrival_stable_time_s 后才算到点

7. 到点后的状态输出
   处理主体:
     LidarNavControl 节点

   对应文件:
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_control.cpp
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_path.cpp
     src/commands/task_manager/src/task_state_machine.cpp

   如果已经到点:
     发布零 /quad/cmd_vel
     current_wp_idx_++
     reset_arrival_debounce()
     logistics_controller_->reset()

   如果当前点是 TRANSITION:
     不发物流取放事件，直接继续下一个 waypoint

   如果当前点是 PICKUP:
     发布 /quad/logistics_nav_event = PICKUP_REACHED
     external_manipulation_enable=true 时暂停导航，交给 task_manager
     task_manager 收到后进入 TASK_PICKUP_BLOCK

   如果当前点是 DROPOFF:
     发布 /quad/logistics_nav_event = DROPOFF_REACHED
     action_id 随事件一起发给 task_manager
     task_manager 收到后进入 TASK_DROPOFF_BLOCK

8. 未到点时的控制输出
   处理主体:
     LidarNavControl 负责组装输入和发布输出
     NavController / HeadingDockNavController 负责计算速度

   对应文件:
     src/commands/lidar_nav_demo_cpp/src/lidar_nav_control.cpp
     src/commands/lidar_nav_demo_cpp/include/lidar_nav_demo_cpp/nav_controller.hpp
     src/commands/lidar_nav_demo_cpp/src/nav_controller.cpp

   组装 NavControllerInput:
     err_x_body
     err_y_body
     err_yaw
     dist
     dt
     track_id = current_wp_idx_
     waypoint_type = current_wp.type
     enabled = enable_control_

   调用:
     logistics_controller_->compute(controller_input)

   得到 NavControllerOutput:
     cmd
     stage
     bearing_error

   发布:
     /quad/cmd_vel = cmd

   打印日志:
     WP[i] stage=... dist=... tol=... err_body=... cmd=...
```

同一条数据流按“处理主体”重新归纳如下：

```text
LidarNavControl 节点
  文件:
    lidar_nav_demo_node.cpp
    lidar_nav_callbacks.cpp
    lidar_nav_path.cpp
    lidar_nav_control.cpp
    lidar_nav_auto_parku.cpp

  负责:
    读取参数
    订阅 TF / enable / OCR / eightboxes
    生成 waypoints_
    选择 current_wp
    发布 current_pose/current_target
    计算 map 误差
    调用 map_error_to_body()
    做到点判定和消抖
    调用 NavController::compute()
    发布 /quad/cmd_vel
    发布 /quad/logistics_nav_event

NavController 抽象层
  文件:
    nav_controller.hpp
    nav_controller.cpp

  负责:
    定义控制器统一输入 NavControllerInput
    定义控制器统一输出 NavControllerOutput
    提供 reset / clamp / normalize_angle / apply_creep 等工具
    维护跨帧误差状态，支持 D 项和换 waypoint 重置

HeadingDockNavController
  文件:
    nav_controller.cpp

  负责:
    根据 err_x_body / err_y_body / err_yaw / dist 选择 APPROACH、ALIGN、DOCK
    计算 vx / vy / wz
    执行各阶段限幅
    输出 NavControllerOutput

BodyPdNavController
  文件:
    nav_controller.cpp

  负责:
    复刻旧版机体系 PD 控制
    用于 logistics_controller_type="body_pd" 回退

task_manager
  文件:
    src/commands/task_manager/src/task_state_machine.cpp

  负责:
    接收 /quad/logistics_nav_event
    到 PICKUP/DROPOFF 后暂停物流导航
    切站立
    触发机械臂
    恢复 RL 和 LOGISTICS_NAV
```

把上面压缩成一条主线，就是：

```text
TF 位姿 + 当前 waypoint
  -> map 误差
  -> 机体系误差
  -> 到点判定
  -> 未到点则控制器算 cmd_vel
  -> 到点则发布事件/切下一个 waypoint
```

各模块职责边界：

```text
LidarNavControl:
  负责 ROS I/O、TF、路径生成、当前 waypoint、到点判定、任务事件、cmd_vel 发布。

NavController:
  只负责从机体系误差计算速度 cmd，不知道 ROS topic，也不发布事件。

task_manager:
  只接收 /quad/logistics_nav_event，负责切站立、机械臂动作、恢复导航。
```

控制器输入结构：

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

控制器输出结构：

```cpp
struct NavControllerOutput {
    geometry_msgs::msg::Twist cmd;
    NavControlStage stage;
    double bearing_error;
};
```

其中 `stage` 会被日志打印：

```text
WP[i] stage=APPROACH/ALIGN/DOCK dist=... err_body=... cmd=...
```

## 3. 坐标与误差约定

控制器接收的是已经转换好的机体系误差，不直接处理 TF 和雷达外参。

当前约定：

```text
err_x_body > 0:
  目标在机器人前方，需要正 vx 前进。

err_y_body > 0:
  目标在机器人左侧，需要左向修正或左转修正。

err_yaw > 0:
  目标 yaw 相对当前 yaw 为正，需要正 wz 转向。
```

`dist` 使用：

```cpp
dist = hypot(err_x_body, err_y_body);
```

注意：`err_x_body / err_y_body` 已经包含 `lidar_offset_x / lidar_offset_y` 的影响，因此到点判定和控制器看到的是“补偿雷达偏置后的机体系目标误差”。

## 4. 控制器基类

`NavController` 是抽象基类，负责通用工具：

```text
reset()
  重置控制器内部状态。

update_derivative()
  根据当前误差和上一帧误差计算 D 项。

normalize_angle()
  将角度归一化到 [-pi, pi]。

clamp()
  限幅。

apply_creep()
  在误差未消除且速度太小时，补一个最小速度以克服低速死区。
```

当前 `heading_dock` 会调用 `update_derivative()` 来维护换点状态，但本身还没有使用 D 项。D 项主要由 `body_pd` 使用。

`track_id` 是当前 waypoint index。换 waypoint 时：

```text
track_id 改变
  ↓
控制器重置上一帧误差
  ↓
避免 D 项或内部状态跨 waypoint 串扰
```

节点在这些场景也会 reset 控制器：

```text
导航未使能
到达 waypoint
重新生成物流路径
收到 /lidar_nav_control 临时目标
```

## 5. body_pd 回退控制器

`body_pd` 复刻旧逻辑，便于对比和快速回退。

启用方式：

```yaml
logistics_controller_type: "body_pd"
```

控制形式：

```text
若 abs(err_yaw) > yaw_first_threshold:
  vx = 0
  vy = 0
  wz = kp_yaw * err_yaw + kd_yaw * d(err_yaw)

否则:
  vx = kp_x * err_x_body + kd_x * d(err_x_body)
  vy = kp_y * err_y_body + kd_y * d(err_y_body)
  wz = kp_yaw * err_yaw + kd_yaw * d(err_yaw)
```

然后分别限幅：

```text
vx ∈ [-max_vx, max_vx]
vy ∈ [-max_vy, max_vy]
wz ∈ [-max_dyaw, max_dyaw]
```

如果 `lin_vel_creep_min > 0`，并且某个方向误差仍大于 `5e-3 m`，但速度小于最小速度，则强制补到：

```text
vx = sign(err_x_body) * lin_vel_creep_min
vy = sign(err_y_body) * lin_vel_creep_min
```

这个控制器的特点：

```text
优点:
  行为简单，和旧版本一致，适合快速回退。

缺点:
  会直接用 vy 修横向误差。
  末端 lin_vel_creep_min 过大时容易过冲。
  取/放物块时横向移动可能导致机身不稳。
```

## 6. heading_dock 分阶段控制器

`heading_dock` 的核心目标是：

```text
远距离:
  像“开车”一样靠 vx + wz 接近目标，尽量少横移。

近距离:
  先修横向误差和 yaw，让机器人姿态适合进点。

末端:
  低速、小幅、允许极小 vy 补偿，稳定进入取/放点。
```

阶段枚举：

```cpp
enum class NavControlStage {
    BODY_PD,
    APPROACH,
    ALIGN,
    DOCK
};
```

阶段切换完全由距离决定：

```text
if dist > approach_to_align_dist:
  APPROACH

else if dist > dock_start_dist:
  ALIGN

else:
  DOCK
```

当前参数：

```yaml
approach_to_align_dist: 0.60
dock_start_dist: 0.25
```

## 7. APPROACH 阶段

适用条件：

```text
dist > approach_to_align_dist
```

目标：

```text
远距离快速接近目标。
主要使用 vx 和 wz。
强烈限制 vy。
目标在侧方或身后时降低前进速度，避免横着冲。
```

首先计算目标方位角：

```cpp
bearing_error = atan2(err_y_body, err_x_body);
```

含义：

```text
bearing_error = 0:
  目标正前方。

bearing_error > 0:
  目标在左前/左侧，需要正 wz 转向。

bearing_error < 0:
  目标在右前/右侧，需要负 wz 转向。
```

前进速度缩放：

```cpp
heading_scale = clamp(cos(bearing_error), 0.0, 1.0);
vx = max(0.0, kp_forward * err_x_body) * heading_scale;
```

这个设计的效果：

```text
目标正前方:
  cos(0)=1，允许正常前进。

目标侧方:
  cos(±90deg)=0，基本不前进，优先转向。

目标在身后:
  cos 接近负数，被 clamp 到 0，不倒着冲。
```

横向速度：

```cpp
vy = lateral_trim_gain * err_y_body;
vy = clamp(vy, -approach_max_vy, approach_max_vy);
```

角速度：

```cpp
wz =
  kp_bearing * bearing_error +
  approach_final_yaw_weight * kp_final_yaw * err_yaw;
```

这里有两个来源：

```text
kp_bearing * bearing_error:
  主要项，让机器人朝目标点方向转。

approach_final_yaw_weight * kp_final_yaw * err_yaw:
  弱项，远距离时稍微考虑最终目标 yaw，但不强求太早对准。
```

APPROACH 限幅：

```yaml
approach_max_vx: 0.45
approach_max_vy: 0.06
approach_max_wz: 0.60
```

最小前进速度：

```cpp
if heading_scale > 0.2:
  apply_creep(err_x_body, approach_creep_min, 5e-3, vx);
```

含义：

```text
只有目标大致在前方时才补最小前进速度。
如果目标在侧方或身后，不强行 creep，避免侧向/背向冲目标。
```

## 8. ALIGN 阶段

适用条件：

```text
dock_start_dist < dist <= approach_to_align_dist
```

目标：

```text
近距离先对准。
尽量不用 vy 硬横移。
把横向误差转化为转向修正，让机器人用姿态和前进方向消除侧偏。
```

横向误差转航向：

```cpp
forward_for_bearing = max(err_x_body, 0.05);
lateral_heading = atan2(err_y_body, forward_for_bearing);
```

这里使用 `0.05` 作为最小前向距离，是为了避免 `err_x_body` 很小或为负时，`atan2(err_y_body, err_x_body)` 突然变得过激。

速度计算：

```cpp
vx = kp_forward * err_x_body;
vy = lateral_trim_gain * err_y_body;
wz = kp_lateral_to_yaw * lateral_heading + kp_final_yaw * err_yaw;
```

解释：

```text
vx:
  继续小速度靠近目标。

vy:
  仍保留很小的横向微调，但不作为主要纠偏手段。

wz:
  lateral_heading 修横向误差；
  err_yaw 修最终姿态。
```

ALIGN 阶段允许轻微倒退：

```cpp
vx = clamp(vx, -0.5 * align_max_vx, align_max_vx);
```

即：

```text
最大前进速度:
  align_max_vx

最大后退速度:
  0.5 * align_max_vx
```

这样如果目标已经略在身后，可以小幅后退修正，但不会高速倒退。

横向限幅：

```cpp
max_trim_vy = min(align_max_vy, max_vy_trim);
vy = clamp(vy, -max_trim_vy, max_trim_vy);
```

ALIGN 限幅：

```yaml
align_max_vx: 0.20
align_max_vy: 0.04
align_max_wz: 0.45
max_vy_trim: 0.04
```

最小速度：

```cpp
if err_x_body > 0:
  apply_creep(err_x_body, dock_creep_min, 5e-3, vx);
```

注意 ALIGN 使用的是 `dock_creep_min`，不是 `approach_creep_min`。这样近距离不会再用太大的最小速度。

## 9. DOCK 阶段

适用条件：

```text
dist <= dock_start_dist
```

目标：

```text
末端低速精定位。
进入取/放点 3~5 cm 范围。
减少速度突变和横向大动作。
```

速度计算：

```cpp
vx = kp_dock_x * err_x_body;
vy = kp_dock_y * err_y_body;
```

航向增益按 waypoint 类型选择：

```cpp
if waypoint_type == PICKUP or DROPOFF:
  yaw_gain = kp_dock_yaw;
else:
  yaw_gain = kp_final_yaw;

wz = yaw_gain * err_yaw;
```

含义：

```text
PICKUP/DROPOFF:
  使用 kp_dock_yaw，末端更认真对准机械臂取放姿态。

TRANSITION:
  使用 kp_final_yaw，姿态要求相对温和。
```

DOCK 限幅：

```yaml
dock_max_vx: 0.08
dock_max_vy: 0.03
dock_max_wz: 0.20
```

最小速度：

```cpp
apply_creep(err_x_body, dock_creep_min, 5e-3, vx);

if abs(err_y_body) > 0.02:
  apply_creep(err_y_body, 0.5 * dock_creep_min, 5e-3, vy);
```

含义：

```text
前后方向:
  允许 dock_creep_min 克服小速度死区。

横向方向:
  只有横向误差超过 2 cm 时才补最小横移速度；
  且横向 creep 只有 dock_creep_min 的一半。
```

这样做是为了避免末端在横向误差很小时仍来回横移抖动。

## 10. 到点判定不在控制器内

控制器只输出速度，不判断任务完成。

到点判定仍在 `LidarNavControl` 中：

```text
dist < position_tolerance
abs(err_yaw) < yaw_tolerance
并持续 arrival_stable_time_s
```

当前按 waypoint 类型区分阈值：

```yaml
transition_position_tolerance: 0.12
transition_yaw_tolerance: 0.12

manip_position_tolerance: 0.05
manip_yaw_tolerance: 0.06

arrival_stable_time_s: 0.20
```

含义：

```text
TRANSITION:
  过渡点要求宽松，减少不必要停顿。

PICKUP/DROPOFF:
  取放点要求严格，保证机械臂动作前的位置和姿态。
```

到点后节点会：

```text
发布零速度
reset 控制器
PICKUP/DROPOFF 发布 logistics_nav_event
TRANSITION 直接切下一个 waypoint
```

## 11. 参数调试建议

### 11.1 远距离绕不过来或转向慢

优先调：

```yaml
kp_bearing
approach_max_wz
```

现象：

```text
bearing_error 长时间较大。
vx 有输出但机器人朝目标方向转得慢。
```

处理：

```text
增大 kp_bearing 或 approach_max_wz。
如果转向来回摆，先降 kp_bearing。
```

### 11.2 远距离仍然横移太多

优先调：

```yaml
approach_max_vy
lateral_trim_gain
```

处理：

```text
降低 approach_max_vy。
降低 lateral_trim_gain。
```

任务赛取放物块时，建议 `approach_max_vy` 明显小于 `approach_max_vx`。

### 11.3 近距离横向误差消得慢

优先调：

```yaml
kp_lateral_to_yaw
align_max_wz
max_vy_trim
```

处理：

```text
如果机器人不愿意转向修横向误差，增大 kp_lateral_to_yaw。
如果角速度打满仍慢，适当增大 align_max_wz。
如果确实需要一点横向补偿，略增 max_vy_trim，但不建议太大。
```

### 11.4 DOCK 阶段过冲

优先调：

```yaml
dock_max_vx
dock_max_vy
dock_creep_min
kp_dock_x
kp_dock_y
```

处理：

```text
降低 dock_max_vx / dock_max_vy。
降低 dock_creep_min。
降低 kp_dock_x / kp_dock_y。
```

特别注意：

```text
dock_creep_min 太大时，接近 3~5 cm 会继续给最小速度，容易越过目标点。
```

### 11.5 到点附近来回抖动但不确认到点

优先看日志：

```text
dist
abs(err_yaw)
tol=(position_tolerance,yaw_tolerance)
stage
cmd
```

可能处理：

```yaml
arrival_stable_time_s: 0.20 -> 0.30
manip_position_tolerance: 0.05 -> 0.06
manip_yaw_tolerance: 0.06 -> 0.08
dock_creep_min: 0.03 -> 0.02
```

如果定位噪声大于阈值本身，控制器会反复追噪声，表现为一直无法稳定确认到点。

## 12. 日志怎么看

当前物流控制日志格式：

```text
WP[i] stage=... dist=... tol=(pos,yaw)
pos(...)
target(...)
err_map(...)
err_body(...)
cmd(vx,vy,wz)
```

重点看：

```text
stage:
  当前处于 APPROACH / ALIGN / DOCK 哪个阶段。

dist:
  是否按预期在 0.60m 和 0.25m 附近切阶段。

err_body.y:
  横向误差是否主要通过 wz 变小，而不是长期靠大 vy。

cmd.linear.y:
  APPROACH/ALIGN 下应明显小于 cmd.linear.x。

cmd.angular.z:
  横向误差大时应该有明显转向修正。
```

## 13. 当前实现的限制

当前 `heading_dock` 仍然是速度接口层面的控制器，它不能单独解决所有厘米级误差来源。

主要限制：

```text
雷达定位噪声和漂移。
雷达到机体中心外参误差。
TF 延迟。
RL 步态小速度死区。
足端滑移。
机身停稳后的晃动。
机械臂/吸盘自身容差。
```

因此当前建议目标仍是：

```text
工程目标:
  稳定进入 3~5 cm，取放动作可靠。

挑战目标:
  静态条件下接近 1~2 cm，但需要定位、标定、低速响应和机械结构共同保证。
```

## 14. 后续可扩展方向

如果继续提高精度，可以考虑：

```text
1. 加速度/角加速度斜率限制，减少 cmd_vel 突变。
2. 对 TF 位姿做低通滤波和异常点剔除。
3. 对 PICKUP/DROPOFF 单独配置目标点补偿。
4. 对不同 waypoint 类型使用不同 dock 参数。
5. 将 AUTO_PARKU 迁移到统一 NavController 接口。
6. 引入视觉/机械臂末端二次对准。
```

当前控制器接口已经把 ROS I/O 与速度计算拆开，后续新增控制器时只需要：

```text
继承 NavController
实现 compute()
在 make_nav_controller() 中注册新 controller_type
在 waypoints.yaml 中新增对应参数
```
