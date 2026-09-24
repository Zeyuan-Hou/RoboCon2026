# record_parku

`record_parku` 是后续“按轨迹位置回放高层事件”的离线数据工具。当前阶段只做录制和离线处理，不改现有障碍赛/任务赛 C++ 逻辑。

## 1. 录制

```bash
source /opt/ros/humble/setup.bash
source /home/cat/hitcrt_quad_ws/install/setup.bash

python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/record_session.py \
  --output-dir /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test02 \
  --duration 60
```

输出：                                                                              
```text
metadata.json
raw_pose.csv
raw_joy.csv
raw_cmd_evt.csv
raw_state_array.csv
raw_task_state_command.csv
raw_current_state.csv
raw_cmd_vel.csv
```

`raw_pose.csv` 来自 `/tf` 的 `camera_init -> aft_mapped`。

## 2. 提取事件

```bash
python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/extract_events.py \
  --session /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test02
```

输出：

```text
events.csv
action_sequences.csv
raw_pose_with_s.csv
```

`events.csv` 把 `/quad/cmd_evt` 中的高层事件绑定到轨迹弧长 `s` 和当时的 `x,y,yaw`。
`action_sequences.csv` 会把同一 `0.20m` 半径位置附近连续出现的 `STAND/JUMP/ENTER_RL` 等动作事件合成一组，回放时按录制相对时间完整执行。回放不会自动补发 `ENTER_RL`，只执行录制/提取出的动作步骤。

`extract_events.py` 默认过滤全局控制/任务事件，不让 `DISABLE(RT+START)`, `DAMPING`, `ENTER_AUTO`, `ENTER_MANUAL`, `SAVE/DELETE` 等进入回放事件表。这样录制开始/结束或急停按键不会在回放过程中把状态机切停。

## 3. 生成回放线路和可视化

```bash
python3 /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/build_replay_path.py \
  --session /home/cat/hitcrt_quad_ws/src/pythontool/record_parku/sessions/test02 \
  --sample-step 0.02 \
  --loop-close-xy 0.08 \
  --loop-max-span-s 0.80 \
  --loop-protect-margin-s 0.03 \
  --remove-small-loops \
  --xy-median-window 0.06 \
  --xy-smooth-window 0.10 \
  --xy-smooth-passes 1
```

输出：

```text
replay_path.csv
route.svg
```

`replay_path.csv` 格式：

```text
s,x,y,yaw
```

其中 `s` 是沿录制轨迹从起点累计的路径长度，不是时间，也不是回放时实际走过的总距离。

默认只对输出线路的 `x,y` 做轻微平滑和去刺，`yaw` 保持原始插值结果，避免把原地转圈段抹掉。需要关闭平滑时，把 `--xy-median-window` 和 `--xy-smooth-window` 都设成 `0`。

默认会删除无事件保护的小圈。常用参数：

- `--loop-close-xy 0.08`：小圈首尾距离小于 8cm 才删除。
- `--loop-max-span-s 0.80`：只删除长度小于 0.8m 的小圈。
- `--loop-protect-margin-s 0.20`：事件和动作组合前后 0.2m 不删。
- `--no-remove-small-loops`：关闭小圈删除。

## 4. 后续回放设计约定

- 正向回放时用 `events.csv` 的 `s` 触发事件。
- 倒退脱困时不触发事件。
- 倒退区间覆盖到 `repeat_on_rewind=true` 的事件时，后续正向再次经过可重新触发。
- 轨迹跟踪用 `replay_path.csv`，后续可实现小半径 pure pursuit：以当前狗位置为圆心找前方轨迹交点。
