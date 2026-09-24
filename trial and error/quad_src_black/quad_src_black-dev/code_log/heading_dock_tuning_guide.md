# heading_dock 实机调参指导

## 1. 调参目标

任务赛物流导航当前默认控制器：

```yaml
logistics_controller_type: "heading_dock"
```

调参目标不是让所有 waypoint 都厘米级精确，而是分类型处理：

```text
TRANSITION 过渡点:
  快速、稳定通过，不要卡很久。
  推荐 0.15~0.25 m 到点阈值。

PICKUP / DROPOFF 取放点:
  稳定进入机械臂可工作的范围。
  推荐 0.04~0.08 m 到点阈值。
```

当前控制器分三段：

```text
APPROACH:
  远距离接近，主要用 vx + wz。

ALIGN:
  近距离对准，主要把横向误差转成转向修正。

DOCK:
  末端精定位，低速但必须超过实机速度死区。
```

## 2. 先看日志

控制日志格式：

```text
WP[i] stage=... dist=... tol=(pos_tol,yaw_tol)
pos(...)
target(...)
err_map(...)
err_body(...)
cmd(vx,vy,wz)
```

重点字段：

```text
WP[i]
  当前 waypoint 下标。

stage
  APPROACH / ALIGN / DOCK。

dist
  当前机体系平面距离误差。

tol=(pos_tol,yaw_tol)
  当前 waypoint 使用的到点阈值。

err_body(x,y,yaw)
  控制器实际看到的机体系误差。

cmd(vx,vy,wz)
  输出到 /quad/cmd_vel 的速度指令。
```

判断是否已经应该到点：

```text
dist < pos_tol
abs(err_yaw) < yaw_tol
并且连续满足 arrival_stable_time_s
```

如果 `dist` 一直大于 `pos_tol`，不是消抖问题，是没进阈值。

如果 `dist` 和 `yaw` 都进了阈值但仍没切点，再看 `arrival_stable_time_s` 和 TF 噪声。

## 3. 推荐调参顺序

不要一上来同时乱改所有参数。建议顺序：

```text
1. 先调到点阈值
2. 再调阶段切换距离
3. 再调速度限幅
4. 再调最小速度 creep
5. 最后调 P 增益
```

原因：

```text
阈值决定是否会卡点。
阶段距离决定什么时候开始慢下来/对准。
限幅决定最大能动多快。
creep 决定小误差时是否能克服死区。
P 增益决定误差到速度的比例。
```

## 4. 到点阈值怎么调

参数：

```yaml
transition_position_tolerance
transition_yaw_tolerance
manip_position_tolerance
manip_yaw_tolerance
arrival_stable_time_s
```

### 4.1 过渡点卡住

现象：

```text
WP 是 TRANSITION。
dist 长时间在 0.15~0.25 m。
机器人已经基本到附近，但没有切下一个点。
```

处理：

```yaml
transition_position_tolerance: 0.20
transition_yaw_tolerance: 0.18
```

如果任务允许过渡点更粗：

```yaml
transition_position_tolerance: 0.25
transition_yaw_tolerance: 0.25
```

不要为了过渡点去大幅提高 DOCK 速度，否则取放点也会受影响。

### 4.2 取放点到不了很小阈值

现象：

```text
PICKUP / DROPOFF。
dist 在 0.06~0.10 m 附近抖动。
机械臂其实可以取放，但导航一直不确认。
```

处理：

```yaml
manip_position_tolerance: 0.06
manip_yaw_tolerance: 0.08
```

如果机械臂容差足够，可放宽到：

```yaml
manip_position_tolerance: 0.08
manip_yaw_tolerance: 0.10
```

### 4.3 已进阈值但不稳定

现象：

```text
dist 偶尔小于 tol，又马上跳出去。
Arrival candidate 反复重置。
```

处理：

```yaml
arrival_stable_time_s: 0.20 -> 0.30
```

如果 TF 噪声明显，不要把阈值设得小于定位噪声。

## 5. 阶段切换距离怎么调

参数：

```yaml
approach_to_align_dist
dock_start_dist
```

当前推荐：

```yaml
approach_to_align_dist: 0.70
dock_start_dist: 0.30
```

### 5.1 进点前姿态来不及对准

现象：

```text
快到目标才进入 ALIGN/DOCK。
err_yaw 或 err_body.y 在末端还很大。
```

处理：

```yaml
approach_to_align_dist: 0.70 -> 0.90
dock_start_dist: 0.30 -> 0.40
```

### 5.2 太早降速，任务变慢

现象：

```text
机器人离目标还很远就慢下来。
APPROACH 很短，ALIGN 很长。
```

处理：

```yaml
approach_to_align_dist: 0.70 -> 0.55
dock_start_dist: 0.30 -> 0.22
```

## 6. 速度限幅怎么调

参数：

```yaml
approach_max_vx
approach_max_vy
approach_max_wz

align_max_vx
align_max_vy
align_max_wz

dock_max_vx
dock_max_vy
dock_max_wz
```

### 6.1 APPROACH 太慢

现象：

```text
远距离 cmd.linear.x 经常打满 approach_max_vx。
机器人稳定但耗时。
```

处理：

```yaml
approach_max_vx: 0.45 -> 0.55~0.65
```

如果转向跟不上，不要只加 vx，同时加：

```yaml
approach_max_wz: 0.60 -> 0.70~0.80
```

### 6.2 横移太多

现象：

```text
cmd.linear.y 长时间较大。
机器人侧向晃，取放物块不稳。
```

处理：

```yaml
approach_max_vy: 0.06 -> 0.03
align_max_vy: 0.06 -> 0.04
dock_max_vy: 0.10 -> 0.05
lateral_trim_gain: 0.12 -> 0.05
```

任务赛一般建议：

```text
dock_max_vy 明显小于 dock_max_vx
```

### 6.3 DOCK 阶段 cmd 太小，机器人不动

现象：

```text
stage=DOCK。
dist 一直不下降。
cmd(vx,vy,wz) 很小，或小于实机能响应的速度。
```

处理：

```yaml
dock_max_vx: 0.20 -> 0.30~0.45
dock_max_wz: 0.30 -> 0.45~0.60
```

如果 cmd 已经达到限幅但机器人仍不动，优先检查底盘/RL 是否接收速度、当前 gait 是否能响应小速度。

## 7. creep 最小速度怎么调

参数：

```yaml
approach_creep_min
dock_creep_min
```

含义：

```text
当误差还没消掉，但 P 算出来的速度太小时，
强制给一个最小速度，避免落入实机死区。
```

### 7.1 DOCK 阶段有误差但不动

现象：

```text
err_body.x 还有 0.05~0.20 m。
cmd.linear.x 很小。
机器人不动或动一下停一下。
```

处理：

```yaml
dock_creep_min: 0.08 -> 0.12 -> 0.15
```

如果实机死区特别大，可以短时间试：

```yaml
dock_creep_min: 0.18
```

但不建议一开始设到 `0.5`。因为到 5 cm 以内仍会强制给很大速度，容易直接越过目标。

### 7.2 DOCK 阶段过冲

现象：

```text
dist 变小后又变大。
机器人穿过目标点。
cmd.linear.x 在近点仍然很大。
```

处理：

```yaml
dock_creep_min: 0.15 -> 0.10 -> 0.08
dock_max_vx: 0.45 -> 0.30
kp_dock_x: 1.8 -> 1.2
```

## 8. P 增益怎么调

参数：

```yaml
kp_forward
kp_bearing
kp_final_yaw
kp_lateral_to_yaw
kp_dock_x
kp_dock_y
kp_dock_yaw
```

### 8.1 远距离不朝目标转

现象：

```text
APPROACH 阶段 err_body.y 较大。
wz 不够大。
机器人像斜着走，而不是转向目标。
```

处理：

```yaml
kp_bearing: 1.2 -> 1.4 -> 1.6
approach_max_wz: 0.60 -> 0.70
```

### 8.2 ALIGN 横向误差消得慢

现象：

```text
ALIGN 阶段 err_body.y 长时间不下降。
cmd.angular.z 不明显。
```

处理：

```yaml
kp_lateral_to_yaw: 1.0 -> 1.3 -> 1.6
align_max_wz: 0.45 -> 0.65
```

### 8.3 DOCK 前后误差消得慢

现象：

```text
DOCK 阶段 err_body.x 还比较大。
cmd.linear.x 没打到 dock_max_vx。
机器人动得慢。
```

处理：

```yaml
kp_dock_x: 1.0 -> 1.4 -> 1.8
```

如果 `cmd.linear.x` 已经打到 `dock_max_vx`，再增大 `kp_dock_x` 没用，应调 `dock_max_vx` 或 `dock_creep_min`。

### 8.4 DOCK 航向对不准

现象：

```text
dist 已接近阈值。
abs(err_yaw) 一直大于 yaw_tol。
wz 偏小。
```

处理：

```yaml
kp_dock_yaw: 1.2 -> 1.6
dock_max_wz: 0.45 -> 0.60
```

## 9. 常见现象对照表

```text
现象:
  WP[0] TRANSITION 卡在 dist=0.18~0.22
处理:
  transition_position_tolerance 调到 0.20~0.25

现象:
  DOCK dist 卡住，cmd 很小
处理:
  提高 dock_creep_min、kp_dock_x、dock_max_vx

现象:
  DOCK dist 卡住，cmd 已经打满
处理:
  提高 dock_max_vx，或检查底盘/RL 是否执行

现象:
  横向晃动明显
处理:
  降低 dock_max_vy、align_max_vy、lateral_trim_gain

现象:
  到点附近来回越过
处理:
  降低 dock_creep_min、dock_max_vx、kp_dock_x

现象:
  yaw 迟迟进不了阈值
处理:
  提高 kp_dock_yaw、dock_max_wz，或放宽 manip_yaw_tolerance

现象:
  已经可以机械臂取放，但导航不确认
处理:
  放宽 manip_position_tolerance / manip_yaw_tolerance
```

## 10. 建议初始参数

适合“实机速度死区明显，但不想末端乱冲”的一版：

```yaml
transition_position_tolerance: 0.20
transition_yaw_tolerance: 0.18
manip_position_tolerance: 0.06
manip_yaw_tolerance: 0.08
arrival_stable_time_s: 0.25

approach_to_align_dist: 0.70
dock_start_dist: 0.30

approach_max_vx: 0.55
approach_max_vy: 0.06
approach_max_wz: 0.70

align_max_vx: 0.30
align_max_vy: 0.06
align_max_wz: 0.65

dock_max_vx: 0.30
dock_max_vy: 0.10
dock_max_wz: 0.45

kp_forward: 0.9
kp_bearing: 1.4
kp_final_yaw: 1.0
kp_lateral_to_yaw: 1.3
lateral_trim_gain: 0.12
max_vy_trim: 0.06
approach_final_yaw_weight: 0.2

kp_dock_x: 1.4
kp_dock_y: 0.8
kp_dock_yaw: 1.2
approach_creep_min: 0.18
dock_creep_min: 0.12
```

## 11. 每次试车记录什么

建议每次只改 1~3 个参数，并记录：

```text
1. 哪个 waypoint 卡住或过冲。
2. 卡住时 stage 是 APPROACH / ALIGN / DOCK 哪个。
3. 卡住时 dist 和 tol 是多少。
4. 卡住时 err_body 是多少。
5. 卡住时 cmd 是多少。
6. 机器人实际是否在动。
7. 改了哪些参数。
```

最有用的判断句：

```text
cmd 小且不动:
  加 creep / 加 kp。

cmd 大且不动:
  查底盘/RL 执行，或加限幅。

cmd 大且过冲:
  降 creep / 降限幅 / 降 kp。

dist 已经够但不切点:
  看 yaw 是否够，看 stable_time 是否被 TF 噪声打断。
```
