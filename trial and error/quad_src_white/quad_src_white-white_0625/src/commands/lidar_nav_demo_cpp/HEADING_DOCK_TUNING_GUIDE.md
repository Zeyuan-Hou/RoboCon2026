# heading_dock 航点与局部分阶段控制调试指南

本文档只聚焦任务赛雷达导航的航点类型、阶段切换和 `heading_dock` 参数调试。视觉、任务状态机和机械臂流程见 `TASK_RACE_DEBUG_GUIDE.md`。

## 1. 调试时先看什么

`lidar_nav_demo_node` 每 25 个控制周期会打印当前航点和控制阶段：

```text
WP[12] stage=ALIGN cross_band=false tight_lat=false manip_appr=true direct_near=false post_exit=false yaw_free=false
dist=0.532 tol=(0.060,0.060) dist_tol=0.200
pos(...) target(...) err_map(...) err_body(...)
raw_cmd(vx,vy,wz) cmd(vx,vy,wz)
```

优先看这些字段：

| 字段 | 含义 | 调试用途 |
|---|---|---|
| `WP[i]` | 当前航点序号 | 判断是否卡在某个航点 |
| `stage` | 局部控制阶段 | 判断正在远距离接近、对齐还是靠泊 |
| `cross_band` | 是否处于隔离带/窄通道相关段 | 决定使用跨隔离带限速和横向到点逻辑 |
| `tight_lat` | 是否处于横向较窄区域 | 决定横向限速/横向 creep |
| `manip_appr` | 取/放货前最后一个 A* 图点 | 通常先到这个点，再精确到 `PICKUP/DROPOFF` |
| `direct_near` | 直达最近图点/目标点 | 普通情况下走 `FORWARD_ONLY`，近取放货点时切 heading_dock |
| `post_exit` | 从取/放货点离开后的第一个过渡点 | 不走 `FORWARD_ONLY`，用 heading_dock 脱离 |
| `yaw_free` | 后排走廊内是否抑制 yaw | 抑制时只修位置，避免在窄区频繁扭头 |
| `err_body(x,y,yaw)` | 机体系误差 | `x` 对应前后，`y` 对应横向，`yaw` 对应航向 |
| `raw_cmd/cmd` | 限幅前/限幅后的速度 | 如果两者差很多，优先看 slew 或最大速度限制 |

常用观测命令：

```bash
ros2 topic echo /quad/current_pose
ros2 topic echo /quad/current_target
ros2 topic echo /quad/cmd_vel
ros2 param get /quad/lidar_nav_demo_node logistics_controller_type
ros2 param get /quad/lidar_nav_demo_node approach_to_align_dist
ros2 param get /quad/lidar_nav_demo_node dock_start_dist
```

运行中可调大多数导航参数：

```bash
ros2 param set /quad/lidar_nav_demo_node align_max_vy 0.50
ros2 param set /quad/lidar_nav_demo_node manip_position_tolerance 0.07
```

注意：`logistics_controller_type` 和 `control_rate` 是启动时参数，改它们需要重启节点。

## 2. 阶段和参数总览

先按当前日志里的 `stage` 定位问题，再调对应参数。

| 阶段 | 主要用途 | 进入条件 | 优先调的参数 |
|---|---|---|---|
| `FORWARD_ONLY` | 普通过渡点快速追点，只前进和转向，不横移 | 普通 `TRANSITION` 或 `direct_nearest_approach` | `forward_only_max_vx`, `forward_only_kp_bearing`, `forward_only_yaw_creep_min`, `transition_dist_arrival_tolerance` |
| `APPROACH` | 远距离接近目标，速度较快 | `dist > approach_to_align_dist` | `approach_to_align_dist`, `approach_max_vx`, `approach_max_vy`, `approach_max_wz`, `kp_forward`, `lateral_trim_gain` |
| `ALIGN` | 中距离收横向误差和 yaw，准备靠泊 | `dock_start_dist < dist <= approach_to_align_dist` | `dock_start_dist`, `align_max_vx`, `align_max_vy`, `align_max_wz`, `align_yaw_priority_threshold`, `align_*_creep_min` |
| `DOCK_YAW` | 取/放货近距离 yaw 未对齐时先收 yaw | `PICKUP/DROPOFF` 且 `dist <= dock_start_dist` 且 yaw 超阈值 | `dock_position_only_yaw_threshold`, `dock_yaw_priority_threshold`, `dock_max_wz`, `kp_final_yaw` |
| `DOCK` | 取/放货最终精靠泊 | `PICKUP/DROPOFF` 且 `dist <= dock_start_dist` 且 yaw 已进阈值 | `dock_max_vx`, `dock_max_vy`, `kp_dock_x`, `kp_dock_y`, `dock_*_creep_min`, `manip_position_tolerance`, `manip_yaw_tolerance` |
| `GRAPH_YAW_ALIGN` | 图点到达后单独原地对准下一段 yaw | `graph_yaw_align_after_arrival=true` 且 yaw 误差超过阈值 | `transition_yaw_tolerance`, `direct_nearest_yaw_align_threshold`, `align_max_wz`, `align_yaw_creep_min` |

阶段切换的核心距离阈值：

```text
dist > approach_to_align_dist          -> APPROACH
dock_start_dist < dist <= approach_to_align_dist -> ALIGN
dist <= dock_start_dist                -> DOCK_YAW / DOCK，仅 PICKUP/DROPOFF
```

普通 `TRANSITION` 默认会绕过 `APPROACH/ALIGN/DOCK`，直接用 `FORWARD_ONLY`。取/放货前的 `manip_approach`、取/放货点本身、取/放货后撤出点，才会更多使用 `heading_dock` 的接近和靠泊阶段。

## 3. 航点类型和标志

路径生成后，控制器只认 `TargetPoint` 列表。每个航点有一个主类型和若干标志。

### TRANSITION

中继/过渡点，主要用于 A* 路径跟踪、跨隔离带走廊、从取放点撤出。

常见控制方式：

- 普通 `TRANSITION` 默认走 `FORWARD_ONLY`：只给前向速度和转向，不给横向速度。
- `manip_approach_arrival=true` 的 `TRANSITION` 使用 heading_dock，作为取/放货前的最后一个图点。
- `post_manip_exit=true` 的 `TRANSITION` 使用 heading_dock，避免刚离开取/放货点时直接纵向冲出去。
- `corridor_lateral_only_arrival=true` 的跨隔离带中间点，只看横向误差到点，忽略 x 和 yaw。

### PICKUP / DROPOFF

精确取货/放货点。一定要求位置和 yaw 同时满足阈值：

```text
dist < manip_position_tolerance
abs(err_yaw) < manip_yaw_tolerance
连续稳定 manip_arrival_stable_time_s
```

导航调试时只需要关注是否满足上述到点条件。到点后上层可能暂停导航并处理任务动作，但这不影响 `heading_dock` 本身的参数判断。

### 关键标志

| 标志 | 来源/用途 | 调参关注 |
|---|---|---|
| `cross_isolation_band` | 航点在隔离带区域或路径穿越隔离带 | 使用 `cross_band_*` 限速和 creep |
| `tight_lateral_zone` | y 处于窄横向区域 | 横向速度和横向 creep 更关键 |
| `corridor_lateral_only_arrival` | 跨隔离带中间点 | 到点只看 `err_y_body` |
| `manip_approach_arrival` | 取/放货前最后图点 | 到点用 `manip_approach_arrival_tolerance`，yaw 不强制 |
| `manip_to_nearest_arrival` | 放货点撤回最近图点 | 到点要求距离和 yaw |
| `direct_nearest_approach` | 直达最近点或直达取/放货 | 远处 `FORWARD_ONLY`，近处可切入靠泊 |
| `graph_yaw_align_after_arrival` | 到达图点后再原地对准 yaw | 主循环执行 `GRAPH_YAW_ALIGN`，日志不会显示为 `stage=ALIGN_YAW` |

## 4. heading_dock 阶段

`heading_dock` 的主要入口在 `NavControllerInput`，使用机体系误差：

```text
err_x_body: 目标在车体前后方向的误差，正数表示目标在前方
err_y_body: 目标在车体左右方向的误差，正数表示目标在左侧
err_yaw:    目标 yaw 与当前 yaw 的差
dist:       hypot(err_x_body, err_y_body)
```

### FORWARD_ONLY

适用：

- 普通 `TRANSITION`
- `direct_nearest_approach`
- 但不包括 `manip_approach_arrival`、`manip_to_nearest_arrival`、`post_manip_exit`
- 如果直达 `PICKUP/DROPOFF` 且 `dist <= direct_manip_dock_handoff_dist`，会退出 `FORWARD_ONLY`，转入 `APPROACH/ALIGN/DOCK`

动作：

```text
vx = kp_forward * max(0, err_x_body)
vy = 0
wz = forward_only_kp_bearing * atan2(err_y_body, err_x_body)
```

主要参数：

| 参数 | 作用 | 调试建议 |
|---|---|---|
| `forward_only_max_vx` | 普通过渡点最大前进速度 | 过渡段太慢就增大；容易冲过就减小 |
| `direct_nearest_max_vx` | direct-nearest 段最大前进速度 | 直达目标太猛就减小 |
| `forward_only_kp_bearing` | 对准目标方向的转向增益 | 朝向跟不上就增大；左右摆头就减小 |
| `forward_only_yaw_creep_min` | 小 yaw 指令的最小转速 | 小角度不动就增大；细碎抖动就减小 |
| `forward_only_forward_creep_min` | 小前向指令的最小速度 | 低速死区导致不走就增大；末端冲过就减小 |
| `direct_manip_dock_handoff_dist` | 直达取/放货时切入靠泊的距离 | 取放点前太晚精调就增大；过早慢下来就减小 |

### APPROACH

适用：

```text
dist > approach_to_align_dist
```

动作：

- 远距离快速接近目标。
- `vx` 主要由 `kp_forward * err_x_body` 给出。
- `vy` 由 `lateral_trim_gain * err_y_body` 给出。
- 对 `PICKUP/DROPOFF`，远距离横向误差会部分转成 yaw 补偿，减少纯横移依赖。

主要参数：

| 参数 | 作用 | 调试建议 |
|---|---|---|
| `approach_to_align_dist` | APPROACH 到 ALIGN 的切换距离 | 离目标还很偏就减速太晚：增大；太早慢下来：减小 |
| `approach_max_vx` | APPROACH 最大前进速度 | 远距离太慢增大；冲击太大减小 |
| `approach_max_vy` | APPROACH 最大横向速度 | 远距离横向追不上增大；横向摆动减小 |
| `approach_max_wz` | APPROACH 最大角速度 | 转向跟不上增大；摆头减小 |
| `kp_forward` | 前向比例增益 | 整体前向响应慢增大；过冲减小 |
| `lateral_trim_gain` | 横向比例增益 | 横向误差消不掉增大；横向来回摆减小 |
| `approach_creep_min` | APPROACH 最小前向速度 | 远处小误差不走增大；末段冲过减小 |
| `approach_lateral_yaw_comp_gain` | 横向误差转 yaw 的比例 | 取/放货远处横向靠不进去可增大；绕弯太多减小 |
| `approach_lateral_yaw_comp_max` | 横向 yaw 补偿上限 | 转向被补偿拉得过大就减小 |
| `approach_lateral_vy_scale` | 取/放点 APPROACH 横向速度缩放 | 取/放点远处横移不足就增大；侧向动作过多就减小 |
| `approach_final_yaw_weight` | APPROACH 阶段最终 yaw 权重 | 远处 yaw 收得慢增大；太早扭头减小 |

跨隔离带时，`approach_max_vx` 会被 `cross_band_approach_max_vx` 替代。

### ALIGN

适用：

```text
dock_start_dist < dist <= approach_to_align_dist
```

动作：

- 中距离对齐位置和 yaw。
- 速度比 APPROACH 慢，横向修正更重要。
- yaw 误差大时会降低线速度，避免边大幅转向边冲向取/放点。

主要参数：

| 参数 | 作用 | 调试建议 |
|---|---|---|
| `dock_start_dist` | ALIGN 到 DOCK 的切换距离 | 还没对准就进 DOCK：减小；迟迟不进 DOCK：增大 |
| `align_max_vx` | ALIGN 最大前进速度 | 中段太慢增大；过冲减小 |
| `align_max_vy` | ALIGN 最大横向速度 | 横向靠不进去增大；横摆减小 |
| `align_max_wz` | ALIGN 最大角速度 | yaw 收不住增大；摆头减小 |
| `max_vy_trim` | 横向修正总上限 | 横向修正被限住就增大 |
| `align_yaw_priority_threshold` | yaw 大时线速度缩放阈值 | yaw 大还冲得太快就减小；过度保守就增大 |
| `align_forward_creep_min` | ALIGN 最小前向速度 | 小误差不走增大；点前冲过减小 |
| `align_lateral_creep_min` | ALIGN 最小横向速度 | 横向死区不动增大；横向震荡减小 |

跨隔离带或窄区时会使用：

```text
cross_band_align_max_vx
cross_band_align_max_vy
cross_band_align_forward_creep_min
cross_band_align_lateral_creep_min
precise_lateral_align_lateral_creep_min
rear_exit_forward_creep_min
rear_exit_lateral_creep_min
```

### DOCK_YAW

适用：

```text
PICKUP/DROPOFF
dist <= dock_start_dist
abs(err_yaw) > dock_position_only_yaw_threshold
dock_position_only_disable_yaw = true
```

动作：

- 近距离还没对准 yaw 时，低速修位置并收 yaw。
- yaw 误差越大，线速度缩放越明显。

主要参数：

| 参数 | 作用 | 调试建议 |
|---|---|---|
| `dock_position_only_disable_yaw` | yaw 对齐后是否关闭 yaw，仅修位置 | 通常保持 `true`，减少近距离抖头 |
| `dock_position_only_yaw_threshold` | 从 DOCK_YAW 切到 DOCK 的 yaw 阈值 | 近处总抖头可增大；最终 yaw 不准可减小 |
| `dock_yaw_priority_threshold` | yaw 大时线速度缩放阈值 | 近处边转边冲就减小；太保守就增大 |
| `dock_max_wz` | DOCK/DOCK_YAW 最大角速度 | yaw 收不住增大；摆头减小 |

注意：代码里有 `ALIGN_YAW` 枚举，但当前 `heading_dock` 基本不输出 `stage=ALIGN_YAW`。近距离偏航修正看 `stage=DOCK_YAW`；图点到达后的原地对准看日志 `GRAPH_YAW_ALIGN`。

### DOCK

适用：

```text
PICKUP/DROPOFF
dist <= dock_start_dist
abs(err_yaw) <= dock_position_only_yaw_threshold
```

动作：

- 最终精确靠泊。
- `vx = kp_dock_x * err_x_body`
- `vy = kp_dock_y * err_y_body`
- 如果 yaw 已在阈值内且 `dock_position_only_disable_yaw=true`，则 `wz=0`，只修位置。

主要参数：

| 参数 | 作用 | 调试建议 |
|---|---|---|
| `dock_max_vx` | 最终前向限速 | 靠泊慢增大；容易顶过/撞上减小 |
| `dock_max_vy` | 最终横向限速 | 横向对不准增大；横向抖动减小 |
| `kp_dock_x` | 最终前向增益 | 前后误差消不掉增大；前后过冲减小 |
| `kp_dock_y` | 最终横向增益 | 横向误差消不掉增大；横向来回摆减小 |
| `kp_dock_yaw` | 最终 yaw 增益 | yaw 仍参与 DOCK 时生效 |
| `dock_forward_creep_min` | 最终前向最小速度 | 靠泊死区不动增大；最后冲过减小 |
| `dock_lateral_creep_min` | 最终横向最小速度 | 横向死区不动增大；横向抖动减小 |
| `manip_position_tolerance` | 取/放货位置到点阈值 | 明显到位但不触发可增大；精度不够可减小 |
| `manip_yaw_tolerance` | 取/放货 yaw 到点阈值 | yaw 不触发可增大；姿态要求更严可减小 |
| `manip_arrival_stable_time_s` | 取/放货到点消抖时间 | 误触发就增大；到点后等待太久就减小 |

## 5. 到点判定参数

调控制器前先确认是不是“控制没问题，但到点判定太严/太松”。

| 场景 | 到点条件 | 相关参数 |
|---|---|---|
| 普通 `TRANSITION` | `dist < transition_dist_arrival_tolerance` | `transition_dist_arrival_tolerance`, `transition_arrival_stable_time_s` |
| 跨隔离带中间点 | `abs(err_y_body) < cross_band_transition_position_tolerance_y` | `cross_band_transition_position_tolerance_y`, `cross_band_transition_arrival_stable_time_s` |
| 精细横向区 | 更严格的 y-only 阈值 | `precise_lateral_arrival_tolerance_y`, `precise_lateral_arrival_y_zone1_min/max` |
| 取/放货前图点 | `dist < manip_approach_arrival_tolerance` | `manip_approach_arrival_tolerance` |
| `PICKUP/DROPOFF` | `dist < manip_position_tolerance && abs(err_yaw) < manip_yaw_tolerance` | `manip_position_tolerance`, `manip_yaw_tolerance`, `manip_arrival_stable_time_s` |

## 6. 推荐调参顺序

### 第一步：确认坐标方向和误差符号

让机器人朝一个固定目标走，观察日志：

```text
err_body.x > 0 -> 目标在前，cmd.linear.x 应为正
err_body.y > 0 -> 目标在左，cmd.linear.y 应为正
err_yaw > 0    -> 目标 yaw 在正方向，cmd.angular.z 应为正
```

如果方向反了，先查 TF、雷达偏移和坐标转换，不要先调增益。

相关参数：

```text
lidar_offset_x
lidar_offset_y
```

### 第二步：调普通过渡点

目标是中继点不乱横移、不蛇形、不卡点。

先调：

```text
forward_only_max_vx
forward_only_kp_bearing
forward_only_yaw_creep_min
transition_dist_arrival_tolerance
```

典型判断：

- 路上摇头：降低 `forward_only_kp_bearing` 或 `forward_only_yaw_creep_min`。
- 朝向跟不上、弯道切不过来：提高 `forward_only_kp_bearing` 或降低 `forward_only_max_vx`。
- 到点附近磨蹭：增大 `transition_dist_arrival_tolerance` 或 `forward_only_forward_creep_min`。

### 第三步：调取/放货前的 APPROACH/ALIGN

目标是进取/放点前不带大横向误差、不带大 yaw 误差。

先调：

```text
approach_to_align_dist
align_max_vy
lateral_trim_gain
align_lateral_creep_min
align_yaw_priority_threshold
```

典型判断：

- 离目标 0.5m 还有很大横向误差：增大 `approach_to_align_dist`、`align_max_vy` 或 `lateral_trim_gain`。
- 中段横向来回摆：减小 `lateral_trim_gain`、`align_lateral_creep_min` 或 `align_max_vy`。
- yaw 没对好就冲进去：减小 `align_yaw_priority_threshold` 或增大 `approach_to_align_dist`。

### 第四步：调最终 DOCK

目标是稳定满足 `PICKUP/DROPOFF` 的最终到点条件，且最后位置不抖、不冲过。

先调：

```text
dock_start_dist
kp_dock_x
kp_dock_y
dock_forward_creep_min
dock_lateral_creep_min
manip_position_tolerance
manip_yaw_tolerance
```

典型判断：

- 最后 5cm 内不动：增大对应方向的 `dock_*_creep_min`。
- 最后持续左右抖：减小 `kp_dock_y` 或 `dock_lateral_creep_min`，必要时增大 `manip_position_tolerance`。
- yaw 一直差一点不触发：先看机械允许精度；可增大 `manip_yaw_tolerance` 或 `dock_position_only_yaw_threshold`。
- 进入 DOCK 太早导致斜着贴近：减小 `dock_start_dist`。

### 第五步：只在最后调 slew

`logistics_cmd_slew_enable=true` 会限制速度变化率。它能让动作更柔，但会掩盖真实控制输出。

如果日志里 `raw_cmd` 明显大于 `cmd`，说明 slew 正在限制：

```text
logistics_cmd_slew_vx_rate
logistics_cmd_slew_wz_rate
logistics_cmd_slew_vy_enable
```

调参时可以临时关闭：

```bash
ros2 param set /quad/lidar_nav_demo_node logistics_cmd_slew_enable false
```

确认阶段和增益正确后再打开。

## 7. 症状反推表

| 症状 | 优先检查 | 常调参数 |
|---|---|---|
| 普通过渡点蛇形 | `stage=FORWARD_ONLY`, `bearing_error`, `cmd.angular.z` | 降 `forward_only_kp_bearing`, 降 `forward_only_yaw_creep_min`, 降 `forward_only_max_vx` |
| 过渡点到不了下一点 | `err_body.x` 是否正、`cmd.linear.x` 是否被限 | 增 `forward_only_forward_creep_min`, 增 `transition_dist_arrival_tolerance` |
| 跨隔离带中间点卡住 | `corridor_lateral_only_arrival=true`, `err_body.y` | 增 `cross_band_transition_position_tolerance_y`, 增 `cross_band_align_lateral_creep_min` |
| 取/放货前横向误差大 | `stage=ALIGN`, `err_body.y`, `cmd.linear.y` | 增 `align_max_vy`, 增 `lateral_trim_gain`, 增 `align_lateral_creep_min` |
| 取/放货前 yaw 没收住 | `err_yaw`, `stage=ALIGN/DOCK_YAW` | 增 `kp_final_yaw`, 增 `align_max_wz`, 减 `align_yaw_priority_threshold` |
| 近距离冲过目标 | `stage=DOCK`, `cmd.linear.x/y` | 降 `dock_max_vx/vy`, 降 `kp_dock_x/y`, 降 `dock_*_creep_min` |
| 近距离不动 | `raw_cmd` 是否小于死区 | 增 `dock_forward_creep_min`, 增 `dock_lateral_creep_min` |
| 到位但不发事件 | `Arrival candidate` 日志 | 增 `manip_position_tolerance`, 增 `manip_yaw_tolerance`, 降 `manip_arrival_stable_time_s` |
| 发事件太早 | 到点阈值是否太松 | 降 `manip_position_tolerance`, 降 `transition_dist_arrival_tolerance`, 增稳定时间 |
| `raw_cmd` 正常但 `cmd` 很小 | slew 限制 | 增 `logistics_cmd_slew_vx_rate/wz_rate` 或临时关闭 slew |

## 8. 常用参数快照

查看当前核心参数：

```bash
ros2 param get /quad/lidar_nav_demo_node forward_only_max_vx
ros2 param get /quad/lidar_nav_demo_node forward_only_kp_bearing
ros2 param get /quad/lidar_nav_demo_node approach_to_align_dist
ros2 param get /quad/lidar_nav_demo_node dock_start_dist
ros2 param get /quad/lidar_nav_demo_node align_max_vy
ros2 param get /quad/lidar_nav_demo_node lateral_trim_gain
ros2 param get /quad/lidar_nav_demo_node kp_dock_x
ros2 param get /quad/lidar_nav_demo_node kp_dock_y
ros2 param get /quad/lidar_nav_demo_node manip_position_tolerance
ros2 param get /quad/lidar_nav_demo_node manip_yaw_tolerance
```

现场微调示例：

```bash
# 过渡点减少蛇形
ros2 param set /quad/lidar_nav_demo_node forward_only_kp_bearing 0.9
ros2 param set /quad/lidar_nav_demo_node forward_only_yaw_creep_min 0.35

# 取/放货前更早进入对齐
ros2 param set /quad/lidar_nav_demo_node approach_to_align_dist 1.4

# 增强中距离横向修正
ros2 param set /quad/lidar_nav_demo_node align_max_vy 0.70
ros2 param set /quad/lidar_nav_demo_node lateral_trim_gain 0.90

# 最终靠泊更柔
ros2 param set /quad/lidar_nav_demo_node dock_max_vx 0.25
ros2 param set /quad/lidar_nav_demo_node kp_dock_x 0.6

# 到位但不触发时放宽一点
ros2 param set /quad/lidar_nav_demo_node manip_position_tolerance 0.075
ros2 param set /quad/lidar_nav_demo_node manip_yaw_tolerance 0.10
```

## 9. 调参纪律

- 一次只改一到两个参数，并记录修改前后的 `stage/err_body/raw_cmd/cmd`。
- 先调到点阈值和阶段切换，再调速度上限，最后调 creep 和 slew。
- `creep_min` 是克服死区用的，不是越大越好；过大最容易造成最后一脚冲过。
- `manip_position_tolerance` 和 `manip_yaw_tolerance` 是任务成功阈值，不要用它们掩盖明显的靠泊误差。
- 如果 `raw_cmd` 方向不符合 `err_body` 符号，优先查坐标/TF/雷达偏移。
