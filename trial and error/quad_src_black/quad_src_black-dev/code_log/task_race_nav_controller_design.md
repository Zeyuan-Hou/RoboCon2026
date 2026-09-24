# 任务赛导航控制器设计方案

## 1. 背景与目标

当前任务赛导航由 `lidar_nav_demo_cpp` 直接根据当前 waypoint 误差输出 `/quad/cmd_vel`。控制器目前不是单纯 P，而是：

```text
map 误差 -> body 误差
vx = kp_x * err_x_body + kd_x * d(err_x_body)
vy = kp_y * err_y_body + kd_y * d(err_y_body)
wz = kp_yaw * err_yaw + kd_yaw * d(err_yaw)
再叠加 lin_vel_creep_min 和速度限幅
```

存在的问题：

```text
1. 实机到点精度约 10 cm，取放物块希望进一步提高。
2. 横向速度 vy 会带来重心不稳，任务赛中应尽量使用前进 vx 和转向 wz。
3. 单层点到点控制缺少“接近目标前的姿态规划”和“末端精定位”。
4. 雷达定位、机体系偏置、RL 步态响应、地面滑移都会限制最终精度。
```

目标分级：

```text
工程可达目标：
  稳定进入 3~5 cm 范围，取放点动作稳定。

挑战目标：
  静态条件下接近 1~2 cm，但需要定位、标定、低速控制和机械臂容差共同保证。

不建议承诺：
  仅靠当前雷达闭环 + RL 速度接口稳定达到 1 cm。1 cm 对 TF 噪声、雷达外参、地面滑移和步态死区都非常苛刻。
```

## 2. 当前控制器问题分析

### 2.1 单层点到点控制

当前控制器直接将目标点误差转换为机体系速度：

```text
err_body.x -> vx
err_body.y -> vy
err_yaw    -> wz
```

这样会导致：

```text
目标在侧前方时，机器人会一边前进一边横移。
横向误差较大时，vy 可能长期存在。
到点附近 vx/vy/wz 同时小幅变化，容易抖动或卡在阈值附近。
```

### 2.2 横向移动不适合任务赛

四足机器人虽然可以横移，但任务赛取放物块时更希望姿态稳定。横向速度带来的问题：

```text
重心摆动更明显。
足端侧向摩擦和滑移更难控。
机械臂或吸盘携带物块时风险更高。
末端靠近货架/物块时横向微动容易碰撞。
```

### 2.3 1 cm 精度的瓶颈

即使控制器很精细，最终精度还受这些因素限制：

```text
雷达定位噪声和漂移。
雷达到机器人中心的外参误差 lidar_offset_x/y。
TF 时间延迟。
RL 控制器对小速度的响应死区。
足端滑移。
机器人停稳后的机身晃动。
机械臂取放容差。
```

因此控制方案不应只靠增大 P，而应分阶段降低速度、减少横移、增加稳定判定和末端补偿。

## 3. 推荐总体架构

建议将任务赛导航控制器改为多阶段控制：

```text
阶段 A：远距离航向导引
  目标：主要用前进 vx 和转向 wz 接近目标，限制 vy。

阶段 B：近距离对准
  目标：把机器人朝向调整到适合进点的方向，把横向误差转化为航向修正。

阶段 C：末端精定位
  目标：低速、小幅、带滤波和稳定判定地进入取/放点。

阶段 D：到点保持
  目标：确认到点后持续发布零速/保持一小段时间，再交给 task_manager 切站立。
```

推荐新增控制模式：

```text
APPROACH
ALIGN
DOCK
HOLD
```

## 4. 控制策略设计

### 4.1 APPROACH：远距离前进导引

适用条件：

```text
dist > approach_to_align_dist
```

目标：

```text
尽量使用 vx + wz，弱化 vy。
让机器人像“开车”一样朝目标点靠近，而不是斜着平移。
```

核心思路：

```text
desired_heading = atan2(err_y_map, err_x_map) 转换到当前坐标约定后得到目标朝向
heading_error = desired_heading - current_yaw

vx = f(dist) * cos(heading_error)
wz = kp_heading * heading_error
vy = 0 或极小横向补偿
```

实际实现时，因为当前工程坐标系里 body.x 表示前进方向，可以直接使用 `err_body`：

```text
bearing_error = atan2(err_y_body, err_x_body)
vx = kp_forward * err_x_body
wz = kp_bearing * bearing_error + kp_yaw * err_yaw_soft
vy = lateral_trim_gain * err_y_body
```

其中：

```text
lateral_trim_gain 建议很小，甚至为 0。
```

速度限制：

```text
vx: 0 ~ approach_max_vx
vy: [-approach_max_vy, approach_max_vy]，建议很小，如 0.05~0.10
wz: [-approach_max_wz, approach_max_wz]
```

重要逻辑：

```text
如果 |bearing_error| 很大，先降 vx，避免横着冲。
如果目标在身后，先转向，不前进。
```

### 4.2 ALIGN：近距离姿态对准

适用条件：

```text
dist <= approach_to_align_dist
且 dist > dock_start_dist
```

目标：

```text
不急着进点，先把横向误差和目标 yaw 调整好。
```

控制策略：

```text
vx = 小速度前进，或根据 err_x_body 控制
wz = kp_lateral_to_yaw * atan2(err_y_body, max(err_x_body, eps))
   + kp_final_yaw * err_yaw
vy = 0 或很小
```

解释：

```text
横向误差不直接用 vy 修，而是让机器人转向，把横向误差变成前进方向误差。
这比直接横移更稳。
```

切换到 DOCK 的条件：

```text
abs(err_y_body) < dock_lateral_entry_tol
abs(err_yaw) < dock_yaw_entry_tol
dist < dock_start_dist
```

如果横向误差始终较大，可以允许极小 vy：

```text
vy = clamp(kp_y_dock * err_y_body, -max_vy_trim, max_vy_trim)
max_vy_trim 建议 0.03~0.08 m/s
```

### 4.3 DOCK：末端精定位

适用条件：

```text
dist <= dock_start_dist
```

目标：

```text
低速进入目标点，优先稳定，不追求快速。
```

推荐控制：

```text
vx = clamp(kp_dock_x * err_x_body, -dock_max_vx_back, dock_max_vx)
vy = clamp(kp_dock_y * err_y_body, -dock_max_vy, dock_max_vy)
wz = clamp(kp_dock_yaw * err_yaw, -dock_max_wz, dock_max_wz)
```

但为了减少横移：

```text
dock_max_vy << dock_max_vx
```

建议：

```text
dock_max_vx: 0.06~0.12 m/s
dock_max_vy: 0.02~0.05 m/s
dock_max_wz: 0.15~0.30 rad/s
```

末端不建议继续使用较大的 `lin_vel_creep_min`。原因：

```text
lin_vel_creep_min 可以克服死区，但接近 1~3 cm 时会导致过冲。
```

建议改为分阶段最小速度：

```text
APPROACH:
  creep_min 可较大，如 0.12~0.20

DOCK:
  creep_min 降低，如 0.02~0.05

进入 arrival_tolerance 内：
  creep_min = 0，允许真正停下
```

### 4.4 HOLD：到点保持

当前已经加入到点消抖：

```yaml
arrival_position_tolerance
arrival_yaw_tolerance
arrival_stable_time_s
```

建议在确认到点后增加一个短暂停稳阶段：

```text
publish zero cmd_vel
hold 0.2~0.5 s
再发布 PICKUP_REACHED / DROPOFF_REACHED
```

原因：

```text
即使连续满足到点阈值，机器人可能仍有速度或机身晃动。
先停稳再切站立，有利于机械臂动作稳定。
```

这个 HOLD 可以在导航节点内部实现，也可以由 `task_manager` 的 `pre_stand_wait_s` 承担。当前已有：

```yaml
pre_stand_wait_s
```

因此第一版可以不新增 HOLD，只调大 `pre_stand_wait_s`。

## 5. 精度目标设计

### 5.1 分级到点阈值

建议把“导航到点”和“取放精定位”分开：

```text
普通过渡点：
  position_tolerance = 0.10~0.15 m
  yaw_tolerance = 0.10~0.15 rad

取块点 / 放置点：
  position_tolerance = 0.03~0.06 m
  yaw_tolerance = 0.03~0.08 rad
```

不要要求所有 waypoint 都 1 cm。过渡点追求 1 cm 会拖慢任务，还可能引入抖动。

### 5.2 1 cm 目标的现实条件

如果确实要追求 1 cm，需要满足：

```text
雷达定位静态噪声小于 5 mm。
雷达到机体中心外参误差小于 5 mm。
目标点坐标标定误差小于 5 mm。
RL 控制对 0.01~0.03 m/s 小速度有稳定响应。
到点后机身晃动小于 1 cm。
```

如果上述条件不满足，控制器调得再细也会被观测误差和执行误差吞掉。

建议工程目标：

```text
导航闭环做到 3~5 cm。
机械臂吸盘/夹具容差覆盖剩余 2~4 cm。
必要时在取块点增加视觉/触觉/机械限位二次对准。
```

## 6. 参数设计建议

### 6.1 新增控制模式参数

建议未来在 `waypoints.yaml` 增加：

```yaml
controller_mode: "heading_dock"

approach_to_align_dist: 0.60
dock_start_dist: 0.25

approach_max_vx: 0.45
approach_max_vy: 0.06
approach_max_wz: 0.60

align_max_vx: 0.20
align_max_vy: 0.04
align_max_wz: 0.45

dock_max_vx: 0.08
dock_max_vy: 0.03
dock_max_wz: 0.20

kp_forward: 0.8
kp_bearing: 1.2
kp_final_yaw: 0.8
kp_dock_x: 0.6
kp_dock_y: 0.4
kp_dock_yaw: 0.8

lateral_trim_gain: 0.1
max_vy_trim: 0.04
```

### 6.2 到点参数

当前已有：

```yaml
arrival_position_tolerance: 0.10
arrival_yaw_tolerance: 0.10
arrival_stable_time_s: 0.20
```

建议改为按 waypoint 类型区分：

```yaml
transition_position_tolerance: 0.12
transition_yaw_tolerance: 0.12

manip_position_tolerance: 0.04
manip_yaw_tolerance: 0.06

arrival_stable_time_s: 0.30
```

### 6.3 速度平滑参数

建议增加速度斜率限制：

```yaml
max_acc_vx: 0.4
max_acc_vy: 0.15
max_acc_wz: 0.8
```

作用：

```text
避免速度指令突变导致机身晃动。
对机械臂携带物块时尤其重要。
```

### 6.4 TF 滤波参数

建议对 `x/y/yaw` 做轻量滤波：

```yaml
pose_filter_enable: true
pose_filter_alpha: 0.3
pose_outlier_dist: 0.20
pose_outlier_yaw: 0.30
```

作用：

```text
降低雷达 TF 瞬时跳变对到点判定和速度输出的影响。
```

## 7. 推荐实施路径

### 第一阶段：少横移控制，不追求 1 cm

目标：

```text
让机器人主要靠前进和转向到点。
把横向速度限制到很小。
稳定进入 5~8 cm。
```

修改：

```text
1. 增加 controller_mode。
2. APPROACH 阶段用 bearing_error 控制 wz。
3. 限制 approach/align 阶段 max_vy。
4. 保留当前到点消抖。
```

验证：

```text
观察 /quad/cmd_vel：
  linear.y 应明显小于 linear.x。

观察日志：
  err_body.y 应主要通过 wz 修正，而不是长期靠 vy 横移。
```

### 第二阶段：末端 DOCK

目标：

```text
取/放点进入 3~5 cm。
```

修改：

```text
1. 增加 DOCK 阶段。
2. DOCK 中降低 max_vx/max_vy/max_wz。
3. 降低末端 creep_min。
4. PICKUP/DROPOFF 使用更严格到点阈值。
```

验证：

```text
取/放点日志稳定进入：
  dist < 0.05
  yaw < 0.06
```

### 第三阶段：高精度增强

目标：

```text
在定位和执行条件允许时接近 1~2 cm。
```

修改：

```text
1. 加 TF 滤波和异常点剔除。
2. 加速度斜率限制。
3. 标定 lidar_offset_x/y。
4. 对取/放点单独做目标点补偿。
5. 如有条件，引入视觉/机械臂末端二次对准。
```

## 8. 调试方法

### 8.1 看误差

导航日志：

```text
WP[i] pos(...) target(...) err_map(...) err_body(...)
```

判断：

```text
err_body.x：前后误差。
err_body.y：横向误差。
err_yaw：航向误差。
```

目标：

```text
APPROACH 阶段：
  err_body.y 可以较大，但 linear.y 应较小，主要靠 wz 修正。

DOCK 阶段：
  err_body.x/y/yaw 都应缓慢收敛。
```

### 8.2 看速度

```bash
ros2 topic echo /quad/cmd_vel
```

建议：

```text
linear.y 不应长期接近 max_vy。
接近取/放点时 linear.x/y/wz 应明显降速。
到点候选阶段速度应接近 0。
```

### 8.3 看底层响应

```bash
ros2 topic echo /quad/high_command
```

如果 `/quad/cmd_vel` 很小但 `/quad/high_command` 也很小、机器人不动，说明进入了 RL 小速度死区。需要：

```text
适当提高 dock_min_speed，或让 DOCK 阶段使用短脉冲式微动。
```

## 9. 风险与取舍

### 9.1 减少横移会增加路径长度

用转向代替横移后，机器人可能需要绕一点小弧线，不如直接 `vy` 快。但稳定性会更好。

### 9.2 过高精度会拖慢任务

所有点都追求 1 cm 会显著降低速度。建议只对 PICKUP/DROPOFF 使用高精度。

### 9.3 小速度死区需要实测

如果 RL 对小速度不响应，DOCK 阶段要设计最小速度或短脉冲，不然会卡住。

### 9.4 机械臂容差应参与设计

导航不一定独自承担 1 cm。更好的工程方案是：

```text
导航稳定到 3~5 cm
机械臂/吸盘/物块结构容差覆盖剩余误差
```

## 10. 推荐结论

建议下一版控制器采用：

```text
heading-based approach + low-vy align + slow dock + arrival debounce
```

控制优先级：

```text
1. 先减少横移，提高机身稳定。
2. 再做取/放点专用 DOCK。
3. 最后再追求 1~2 cm 高精度。
```

不建议直接靠增大 `kp_x/kp_y` 追求 1 cm。那样通常会带来：

```text
过冲
横向晃动
到点抖动
机械臂动作不稳
```

更稳妥的方向是：分阶段控制、限制横向速度、末端慢速精定位、到点消抖和外参/目标点精标定。
