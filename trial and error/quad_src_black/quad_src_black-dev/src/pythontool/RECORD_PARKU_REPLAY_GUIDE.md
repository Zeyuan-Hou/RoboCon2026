# RECORD_PARKU_REPLAY 使用与调参说明

本文说明 `record_parku` 离线工具和 `task_manager/obstacle_nav_node` 的
`RECORD_PARKU_REPLAY` 回放模式。

## 1. 快速使用

### 1.1 录制一条任务赛路线

```bash
source /opt/ros/humble/setup.bash
source /home/cat/hitcrt_quad_ws/install/setup.bash

python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/record_session.py \
  --output-dir /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test012 \
  --duration 40

python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/prepare_replay_session.py test011


python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/prepare_replay_session.py test010
\
  --patch-session test011

```
python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/prepare_replay_session.py test08 \
  --patch-session test09
录制内容包括：

- `/tf` 中的 `camera_init -> aft_mapped` 位姿，输出 `raw_pose.csv`
- 原始手柄，输出 `raw_joy.csv`
- 高层事件 `/quad/cmd_evt`，输出 `raw_cmd_evt.csv`
- 底层状态、任务状态、速度命令等辅助日志

### 1.2 提取事件表

```bash
python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/extract_events.py \
  --session /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010 \
  --yaw-weight 0.25
```

输出：

- `events.csv`：事件码绑定到轨迹弧长 `s` 和当时 `x,y,yaw`
- `action_sequences.csv`：同一位置附近的 STAND/JUMP/ENTER_RL 等动作组合，并按状态机转换表压缩无效或冗余切换
- `raw_pose_with_s.csv`：带弧长进度的原始位姿

默认动作组合事件码为 `2,4,5,6,7,8,21`。动作事件会按录制时间顺序处理；只要后续动作仍在本组首个动作位置的 `0.20m` 半径内，就会被合并到同一组 `action_sequences.csv`。`POLICY_KNEEL_CRAWL=21` 会跟附近动作一起进组合，被组合覆盖后不会再作为普通事件即时触发；`POLICY_TROT/CREEP/UPSTAIR` 仍按位置即时触发。

生成动作组合时会参考 `state_machine.py` 的 lower FSM 转换表做一次简化：如果原始组合里有多余的蹲站切换，且可以用更短的合法事件序列达到同样的关键动作和最终 lower_state，就会写入更短的 `steps_json`。`POLICY_KNEEL_CRAWL=21` 是策略切换事件，不要求先进入 `FIXED_DOWN`；在 `RL_MOVE` 下也可以直接切换并继续走。简化后事件间隔不再直接使用录制时间差，而使用动作耗时表：`STAND/UP_DOWN(2)=1.0s`，`ENTER_RL(4)=0.3s`，`STRIDE(5)=1.0s`，`SMALL_JUMP(6)=3.0s`，`JUMP(7)=5.5s`，`KNEEL_CRAWL(8)=1.0s`，`POLICY_KNEEL_CRAWL(21)=0.3s`。需要调整时可在 `extract_events.py` 后加 `--action-event-durations 2:1.2,4:0.3,6:3.5,7:5.5,21:0.3`。

回放动作组合结束后的恢复由 `replay_action_finish_enter_rl` 控制。为 `false` 时不会额外补发 `ENTER_RL`；为 `true` 时会在组合结束后补一次 `ENTER_RL`，帮助回到 `RL_MOVE`。

默认不会把全局控制/任务事件写入 `events.csv`，避免回放时把系统切停或切回手动。过滤事件码包括 `ENABLE(0)`, `DISABLE(1)`, `DAMPING(3)`, `ENTER_AUTO(12)`, `ENTER_MANUAL(13)`, `SAVE/DELETE(19/20)` 等。若确实需要改这个列表，可用 `extract_events.py --exclude-event-codes ...`。

### 1.3 生成回放轨迹

推荐直接使用一键处理脚本。它会按顺序执行：

```text
extract_events.py
-> build_replay_path.py
-> 可选 splice_replay_path.py
-> process_replay_path.py
-> 把最终结果覆盖回 replay_path.csv / events.csv / action_sequences.csv / route.svg
```

单个 session：

```bash
python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/prepare_replay_session.py test010
```

用 session B 替换/补入 session A 的局部路径，例如把 `test09` 拼进 `test08`：

```bash
python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/prepare_replay_session.py test08 \
  --patch-session test09
```

脚本会先备份目标 session 里已有的标准输出到类似：

```text
backup_before_prepare_YYYYMMDD_HHMMSS/
```

最终仍然输出为固定文件名：

```text
replay_path.csv
events.csv
action_sequences.csv
route.svg
```

一键脚本默认使用 `--yaw-mode recorded`：离线修整只改路径的 `x,y,s`，最终
`replay_path.csv` 的 `yaw`、`events.csv` 的 `yaw` 和
`action_sequences.csv` 的 `trigger_yaw` 都会按原始录制姿态回填，不会改成路径切线方向。
若需要临时恢复旧的切线方向调试，可显式加 `--yaw-mode tangent`。

因此 `quad_parku.yaml` 里只需要改 session 编号，不需要改文件名：

```yaml
record_parku_replay_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test08/replay_path.csv"
record_parku_events_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test08/events.csv"
record_parku_action_sequences_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test08/action_sequences.csv"
```

下面是手动分步命令，只有需要单独调某一步参数时才需要使用。

```bash
python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/build_replay_path.py \
  --session /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010 \
  --sample-step 0.02 \
  --yaw-weight 0.25 \
  --min-yaw-step 0.03 \
  --remove-small-loops \
  --loop-close-xy 0.08 \
  --loop-max-span-s 0.35 \
  --xy-median-window 0.06 \
  --xy-smooth-window 0.10 \
  --xy-smooth-passes 1

python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/process_replay_path.py \
  --session /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010 \
  --circle-mode repel \
  --repel-safe-radius 0.55 \
  --repel-margin 0.10 \
  --repel-smooth-passes 25 \
  --repel-smooth-alpha 0.10 \
  --post-smooth-window-s 0.16 \
  --post-smooth-passes 3 \
  --yaw-mode recorded
```

输出：

- `replay_path.csv`：回放用连续线路，格式 `s,x,y,yaw`
- `route.svg`：二维路线图，可检查路线和事件位置

`s` 不是时间。现在默认同时包含 xy 位移和 yaw 变化：

```text
ds = hypot(dxy, yaw_weight * dyaw)
```

默认 `yaw_weight=0.25`，表示转 1 rad 大约相当于 0.25 m 的轨迹进度。
这样原地转圈、小范围转 yaw 不会被压成同一个 `s` 点。

`build_replay_path.py` 默认会对输出轨迹的 `x,y` 做轻微平滑：

- `--remove-small-loops`：删除无事件保护的小圈/小回环，默认开启。
- `--loop-close-xy`：小圈首尾距离阈值，越大删得越多。
- `--loop-min-span-s` / `--loop-max-span-s`：只删除这个 `s` 长度范围内的小圈。
- `--loop-protect-margin-s`：事件和动作组合前后保护范围，保护区内的小圈不删。
- `--xy-median-window`：先用小窗口中值滤波去掉孤立位置刺。
- `--xy-smooth-window`：再用小窗口移动平均让路线更顺。
- `--xy-smooth-passes`：平滑次数，通常 1 次即可。

小圈删除会保留首尾端点的 `s`，把中间小圈替换成直连段，因此 `events.csv` 和 `action_sequences.csv` 的 `s` 不会整体错位。这一步只修改 `replay_path.csv` 的 `x,y`，不会平滑 `yaw`。因此原地转圈段若在动作/事件保护区内，会被保留。

### 1.4 启用 C++ 回放模式

回放参数文件在：

```text
/home/cat/hitcrt_quad_ws/src/config/quad_parku.yaml
```

核心配置：

```yaml
/quad/obstacle_nav_node:
  ros__parameters:
    navigation_mode: "RECORD_PARKU_REPLAY"
    record_parku_replay_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test02/replay_path.csv"
    record_parku_events_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test02/events.csv"
    record_parku_action_sequences_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test02/action_sequences.csv"
```

回放跨过 `action_sequences.csv` 的 `trigger_s` 后，会暂停轨迹跟踪，按 `steps_json` 里的 `delay_s` 依次发布 `/quad/cmd_evt`。组合结束后的恢复行为由 `replay_action_finish_enter_rl` 控制：为 `false` 时不额外补发 `ENTER_RL`，为 `true` 时会补发 `ENTER_RL` 并等待 `replay_action_finish_wait_s`，再恢复路径跟踪。

当前 `src/launch/task_race.launch.py` 使用两层参数：

```python
base_config = '/home/cat/hitcrt_quad_ws/src/config/quad_run_cfg.yaml'
parku_config = '/home/cat/hitcrt_quad_ws/src/config/quad_parku.yaml'
```

`quad_run_cfg.yaml` 保留整车、RL、pos、rknn、task_executor 等基础参数。
`quad_parku.yaml` 只覆盖 `task_manager_node` 和 `obstacle_nav_node` 的障碍赛回放参数。

不要把所有节点都只改成读取 `quad_parku.yaml`，否则 RL_MOVE、pos control 等节点会丢失
`quad_run_cfg.yaml` 中的原参数。

也可以单独启动回放节点做调试：

```bash
ros2 run task_manager obstacle_nav_node \
  --ros-args \
  -r __ns:=/quad \
  --params-file /home/cat/hitcrt_quad_ws/src/config/quad_parku.yaml
```

回放开始依赖任务状态机发布 `/quad/nav_enable=true`。单独调试时可手动发布：

```bash
ros2 topic pub --once /quad/nav_enable std_msgs/msg/Bool "{data: true}"
```

### 1.5 当前快速启动方式

当前已经把 `src/launch/task_race.launch.py` 改成：

```python
base_config = '/home/cat/hitcrt_quad_ws/src/config/quad_run_cfg.yaml'
parku_config = '/home/cat/hitcrt_quad_ws/src/config/quad_parku.yaml'
```

其中：

- `task_manager_node` 读取 `[base_config, parku_config, waypoints_file, {...}]`
- `obstacle_nav_node` 读取 `[base_config, parku_config]`
- `rl_control_node`、`pos_control_node`、`task_executor_node`、`rknn_quad_node` 仍只读取 `base_config`

这样 RL_MOVE 相关参数仍来自 `quad_run_cfg.yaml`，只有障碍赛入口和回放参数由
`quad_parku.yaml` 覆盖。测试这套 parku/障碍赛回放时，只需要开这一个 launch：

```bash
source /opt/ros/humble/setup.bash
source /home/cat/hitcrt_quad_ws/install/setup.bash

ros2 launch /home/cat/hitcrt_quad_ws/src/launch/task_race.launch.py
```

启动后流程是：

```text
task_race.launch.py
-> 大部分节点读取 quad_run_cfg.yaml
-> task_manager_node 叠加 quad_parku.yaml，使用 mission_mode: obstacle
-> obstacle_nav_node 叠加 quad_parku.yaml，使用 navigation_mode: RECORD_PARKU_REPLAY
-> 遥控器切自动后，任务状态机进入 AUTO_NAV
-> task_manager_node 发布 /quad/nav_enable=true
-> obstacle_nav_node 开始跟踪 replay_path.csv 并按 events.csv 触发事件
```

此时不需要手动发布 `/quad/nav_enable=true`。只有单独启动 `obstacle_nav_node`、
没有启动完整任务状态机时，才需要手动发布这个话题。

### 1.6 之后改回任务赛

如果之后要把 `task_race.launch.py` 改回普通任务赛/物流赛：

1. 可以保留：

```python
base_config = '/home/cat/hitcrt_quad_ws/src/config/quad_run_cfg.yaml'
```

删除或不再使用：

```python
parku_config = '/home/cat/hitcrt_quad_ws/src/config/quad_parku.yaml'
```

2. 把 `task_manager_node` 的参数改回只用任务赛配置，并加回 logistics 覆盖：

```python
parameters=[
    base_config,
    LaunchConfiguration('waypoints_file'),
    {
        'mission_mode': 'logistics',
        'manipulation.command_topic': '/manipulator/cmd',
    },
]
```

也就是不要再给 `task_manager_node` 叠加 `parku_config`。

3. 普通任务赛不需要 `obstacle_nav_node`，可以把它的 Node 定义和 `LaunchDescription`
里的 `obstacle_nav_node` 注释掉或删掉。

## 2. 代码结构简述

### 2.1 Python 离线工具

路径：

```text
src/pythontool/record_parku/
```

主要文件：

- `record_session.py`：录制 `x,y,yaw`、手柄、事件、底层状态、速度命令
- `extract_events.py`：从录制数据中提取 `events.csv`
- `build_replay_path.py`：生成 `replay_path.csv` 和 `route.svg`
- `common.py`：CSV 读写、弧长 `s`、重采样、事件命名等公共函数

### 2.2 C++ 回放节点

路径：

```text
src/commands/task_manager/src/obstacle_nav_node.cpp
src/commands/task_manager/include/task_manager/obstacle_nav_node.hpp
```

新增模式：

```text
RECORD_PARKU_REPLAY
```

运行逻辑：

- 读取 `replay_path.csv`，得到线路点 `s,x,y,yaw`
- 读取 `events.csv`，得到事件触发位置
- 当前雷达位置投影到线路，得到当前进度 `current_s`
- 以狗当前位置为圆心，用 `replay_lookahead_radius` 找前方轨迹交点
- 使用现有 `lidar_nav_demo_cpp` 同款雷达到机体转换方式计算控制误差
- 输出 `/quad/cmd_vel`
- 正向跨过事件 `s` 时发布 `/quad/cmd_evt`
- 卡住时倒退 `rewind_distance`，倒退阶段不触发事件

### 2.3 坐标转换

回放模式使用和 `lidar_nav_demo_cpp` 一致的转换：

```cpp
err_x_lidar = cos(yaw) * err_y_map - sin(yaw) * err_x_map;
err_y_lidar = -sin(yaw) * err_y_map - cos(yaw) * err_x_map;

err_x_body = err_x_lidar + lidar_offset_x;
err_y_body = err_y_lidar + lidar_offset_y;
```

对应参数在 `quad_parku.yaml`：

```yaml
lidar_offset_x: 0.05
lidar_offset_y: 0.0
```

## 3. 调参使用说明

建议按下面顺序调，不要一次改很多参数。

### 3.1 先确认数据文件

先看 `route.svg`：

- 路线是否连续
- 起点终点是否正确
- `JUMP` 等事件是否落在预期位置
- 有没有明显跳点或大折线

如果路线本身不干净，优先重新录制或调整：

```bash
python3 build_replay_path.py --sample-step 0.02 --min-step 0.005 --max-jump 0.50
```

### 3.2 跟踪精度优先调 lookahead

```yaml
replay_lookahead_radius: 0.25
replay_intersection_search_dist: 1.0
replay_fallback_lookahead: 0.18
replay_projection_yaw_weight: 0.25
```

- 轨迹切弯、贴不住路线：减小 `replay_lookahead_radius`
- 速度发抖、目标点太近：增大 `replay_lookahead_radius`
- 找不到圆线交点时经常 fallback：增大 `replay_intersection_search_dist`
- fallback 时目标太远：减小 `replay_fallback_lookahead`
- 原地转圈时 `current_s` 不前进：重新生成 `events.csv/replay_path.csv`，并确认
  `build_replay_path.py --yaw-weight` 与 `quad_parku.yaml` 的
  `replay_projection_yaw_weight` 一致

### 3.3 再调速度和增益

默认参考 `waypoints.yaml` 的 ALIGN 段：

```yaml
replay_kp_forward: 1.1
replay_lateral_trim_gain: 0.75
replay_kp_yaw: 1.5
replay_max_vx: 0.45
replay_max_vy: 0.60
replay_max_wz: 0.70
```

- 前后方向跟不上：增大 `replay_kp_forward`
- 前后方向来回冲：减小 `replay_kp_forward` 或 `replay_max_vx`
- 横向贴线慢：增大 `replay_lateral_trim_gain`
- 横向摆动：减小 `replay_lateral_trim_gain` 或 `replay_max_vy`
- yaw 对不准：增大 `replay_kp_yaw`
- yaw 抖动：减小 `replay_kp_yaw` 或 `replay_max_wz`

### 3.4 死区补偿

```yaml
replay_min_vx: 0.20
replay_min_vy: 0.35
replay_deadzone_error_x: 0.04
replay_deadzone_error_y: 0.04
```

含义：

- 误差超过 deadzone 时，速度会至少补到 `replay_min_vx/vy`
- 误差小于 deadzone 时，不做最小速度补偿，避免到目标附近抖动

调参建议：

- 狗明明有误差但不动：增大 `replay_min_vx` 或 `replay_min_vy`
- 到线附近抖动：增大 `replay_deadzone_error_x/y` 或减小最小速度
- 横向死区通常更大，所以 `replay_min_vy` 默认比 `replay_min_vx` 大

### 3.5 位置事件与可选等待

```yaml
replay_event_min_interval_s: 0.25
replay_event_pause_codes: [4, 5, 6, 7, 8, 9, 10, 11]
replay_event_pause_durations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

对应事件：

```text
4 ENTER_RL
5 STRIDE
6 SMALL_JUMP
7 JUMP
8 KNEEL_CRAWL
9 POLICY_TROT
10 POLICY_CREEP
11 POLICY_UPSTAIR
```

默认是根据轨迹位置 `s` 触发事件，然后继续流畅追轨迹，不停车等待。
`replay_event_min_interval_s` 只用于避免多个事件过密时被 `state_machine` 消抖吞掉。

只有某个动作必须阻塞回放时，才把对应的 `replay_event_pause_durations` 从 `0.0`
调大。普通策略切换建议保持 `0.0`，让它边走边切。

### 3.6 卡住倒退

```yaml
stuck_window_s: 1.5
stuck_progress_epsilon: 0.03
rewind_distance: 0.3
rewind_cooldown_s: 1.0
```

- 卡住很久才倒退：减小 `stuck_window_s`
- 正常慢速也误判卡住：增大 `stuck_progress_epsilon` 或 `stuck_window_s`
- 倒退不够脱困：增大 `rewind_distance`
- 频繁倒退：增大 `rewind_cooldown_s`

倒退阶段不会触发事件。倒退完成后，倒退区间内
`repeat_on_rewind=true` 的事件会被重置，正向重新经过时会再次触发。

### 3.7 结束判定

```yaml
finish_s_tolerance: 0.08
finish_xy_tolerance: 0.10
finish_yaw_tolerance: 0.20
```

- 终点附近不结束：适当放宽 `finish_xy_tolerance` 或 `finish_yaw_tolerance`
- 还没到终点就结束：减小这些阈值

## 4. 常见问题

### 4.1 不触发事件

检查：

- `events.csv` 是否存在
- `event.s` 是否在 `replay_path.csv` 的 `s` 范围内
- 是否处于倒退阶段，倒退阶段不会触发事件
- 是否已经触发过，普通正向经过只触发一次

### 4.2 一启用就提示路径未加载

检查 `quad_parku.yaml` 中路径是否存在：

```bash
ls /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test02/replay_path.csv
ls /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test02/events.csv
ls /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test02/action_sequences.csv
```

### 4.3 起点终点接近导致进度跳变

回放节点已经使用滑动窗口投影，正常不会因为空间接近直接跳到终点。
如果仍然跳变，通常是线路中间存在大段重复路线，建议重新录制或把路线拆成更单纯的一圈。

### 4.4 最后一步：离线修整 L 形段和绕圆段

如果录制路线整体可用，但第一段楼梯策略段不够规整，或者起点到第一段楼梯前的绕圆轨迹有抖动，可以在生成
`replay_path.csv`、`events.csv`、`action_sequences.csv` 之后，再跑一次离线修整脚本：

```bash
python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/process_replay_path.py \
  --session /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010 \
  --circle-mode repel \
  --repel-safe-radius 0.55 \
  --repel-margin 0.10 \
  --repel-smooth-passes 25 \
  --repel-smooth-alpha 0.10 \
  --post-smooth-window-s 0.16 \
  --post-smooth-passes 3 \
  --yaw-mode recorded
```

这一步会：

- 把第一段 `POLICY_UPSTAIR -> POLICY_TROT` 楼梯段修整成近似 L 形：直线、1/4 圆、直线。
- 对第二段 `POLICY_UPSTAIR -> POLICY_TROT` 楼梯段做温和平滑，端点附近保留，避免接头突兀。
- 起点到第一段楼梯前使用 `repel` 模式处理绕圆段：只把轨迹从圆心安全半径内推开，不强行吸到某个中点。
- 自动检测长度超过 `2.0m` 的近似直线段，把局部小抖动修成平滑直线。
- 默认不对 `POLICY_CREEP` 和 `POLICY_KNEEL_CRAWL` 之间的轨迹做长直线去抖。
- 处理完所有轨迹后重新计算全局 `s`，默认只按新的 `x,y` 路径长度算。
- 同步更新 `events.csv` 和 `action_sequences.csv` 对应的新 `s,x,y,yaw`；默认 `--yaw-mode recorded` 会保留原始录制姿态 yaw，不会改成路径切线方向。
- 输出带坐标网格、圆心、绕圆安全圈、事件点和 yaw 箭头的可视化图。

输出文件在同一个 session 目录下：

```text
replay_path_processed.csv
events_processed.csv
action_sequences_processed.csv
route_processed.svg
route_processed_debug.svg
routenew.svg
routenew.png
```

优先打开 `routenew.svg` 或 `routenew.png` 检查效果。图中黑线是处理后的轨迹，灰线是原始轨迹，蓝色圆是估计的圆心和安全半径，青色小箭头是每隔 `0.3m` 画出的 yaw 方向；箭头会从轨迹点向车头左侧偏移一点，避免压住黑线。

如果绕圆段仍然离圆太近，可以小幅增大：

```bash
--repel-safe-radius 0.58
```

如果轨迹安全但抖动偏多，优先增大后处理平滑：

```bash
--post-smooth-window-s 0.20
--post-smooth-passes 4
```

如果平滑后贴圆或变形明显，就把这两个值调回小一点。

第二段楼梯平滑默认开启，主要参数是：

```bash
--second-upstairs-smooth-window-s 0.18
--second-upstairs-smooth-passes 2
--second-upstairs-smooth-guard-s 0.20
--second-upstairs-smooth-blend 0.75
```

如果第二段楼梯仍然抖，可以适当增大 `--second-upstairs-smooth-window-s` 或
`--second-upstairs-smooth-passes`。如果接头附近被拉变形，就增大
`--second-upstairs-smooth-guard-s`，或者用 `--no-smooth-second-upstairs` 关闭这一步。

长直线去抖默认开启，主要参数是：

```bash
--straighten-min-length 2.0
--straighten-max-deviation 0.14
--straighten-max-mean-deviation 0.06
--straighten-max-heading-deg 35
--straighten-max-path-ratio 1.08
```

如果直线段仍然有小波纹，可以适当增大 `--straighten-max-deviation` 或
`--straighten-max-heading-deg`。如果发现弯道被误拉直，就减小这些值，或者临时加
`--no-straighten-long-lines` 关闭这一步。

`POLICY_CREEP` 和 `POLICY_KNEEL_CRAWL` 之间默认会从长直线处理里排除；如果确实想让这段也参与直线去抖，可以加：

```bash
--no-straighten-protect-kneel-creep
```

确认 `routenew.svg/png` 看起来满意后，把 `quad_parku.yaml` 中的回放文件改为 processed 版本：

```yaml
record_parku_replay_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010/replay_path_processed.csv"
record_parku_events_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010/events_processed.csv"
record_parku_action_sequences_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010/action_sequences_processed.csv"
```

### 4.5 用路径 B 替换完整路径 A 的一段

如果已经有一条完整路线 A，同时又单独录制或处理出一段更好的局部路线 B，可以用
`splice_replay_path.py` 把 B 融合进 A。脚本会：

- 找到 A 中距离 B 起点最近的点。
- 找到 A 中距离 B 终点最近的点。
- 优先选择和 B 长度相近的局部区间，避免用很短的 B 误替换 A 里的大段路线。
- 用整段 B 替换 A 中这两个点之间的区间；如果 B 连接的是 A 的尾部和头部，也支持替换跨起终点的空白/闭环接缝。
- 在两个接缝附近做局部平滑过渡。
- 重新计算整条新路径的 `s` 和 `yaw`。
- 删除 A 被替换段内的 `events.csv` 和 `action_sequences.csv` 动作。
- 删除距离 B 起点/终点 `0.3m` 内的 A 状态/策略切换，避免接缝附近重复切状态。
- 把 B 的 `events.csv` 和 `action_sequences.csv` 按 B 在新路径中的位置重新映射进去。
- 合并 A 剩余动作和 B 动作，按新的 `s` 排序并重新编号。
- 生成新的路线图用于检查。

常用命令：

```bash
python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/splice_replay_path.py \
  --session /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010 \
  --path-b /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/patch01/replay_path.csv \
  --output-prefix spliced
```

默认 A 为当前 session 下的：

```text
replay_path.csv
events.csv
action_sequences.csv
```

B 的动作表默认从 `--path-b` 所在目录读取：

```text
events.csv
action_sequences.csv
```

也可以显式指定：

```bash
--events-b /path/to/session_B/events.csv
--actions-b /path/to/session_B/action_sequences.csv
```

输出在同一个 session 目录下：

```text
replay_path_spliced.csv
events_spliced.csv
action_sequences_spliced.csv
route_spliced.svg
```

如果 A 不在 session 默认位置，也可以显式指定：

```bash
python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/splice_replay_path.py \
  --path-a /path/to/replay_path_A.csv \
  --path-b /path/to/replay_path_B.csv \
  --events /path/to/events_A.csv \
  --actions /path/to/action_sequences_A.csv \
  --output-dir /path/to/output \
  --output-prefix spliced
```

接缝融合参数：

```bash
--seam-window-s 0.35
--seam-smooth-passes 3
--seam-blend 0.85
```

如果接缝还有折角，可以适当增大 `--seam-window-s` 或
`--seam-smooth-passes`。如果接缝附近被拉得太多，就减小 `--seam-blend`。

局部匹配保护参数：

```bash
--max-replacement-span-ratio 2.0
--max-replacement-extra-s 2.0
```

这两个参数会阻止“4m 的 B 替换 A 里几十米路线”这种误匹配。若脚本提示找不到局部替换区间，先用下面两个参数把搜索限制在 A 的目标区域：

```bash
--a-search-start-s 20.0
--a-search-end-s 28.0
```

如果 B 是用来补 A 的尾部到头部的空白/闭环接缝，默认 `--allow-wrap` 会允许这种跨起终点替换。若只想替换 A 内部连续区间，可以加：

```bash
--no-allow-wrap
```

接缝附近 A 状态切换删除半径默认是：

```bash
--drop-a-seam-state-radius 0.30
```

半径内只删除 A 的状态/策略切换类事件，B 的动作会按新路径插入。

脚本默认允许自动反转 B 的方向：如果 B 的终点更接近 A 中靠前的位置、B 的起点更接近 A 中靠后的位置，会自动用反向 B 替换。若不希望自动反转，可以加：

```bash
--no-allow-reverse-b
```

检查 `route_spliced.svg` 满意后，把 `quad_parku.yaml` 指向 spliced 版本：

```yaml
record_parku_replay_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010/replay_path_spliced.csv"
record_parku_events_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010/events_spliced.csv"
record_parku_action_sequences_path: "/home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test010/action_sequences_spliced.csv"
```
