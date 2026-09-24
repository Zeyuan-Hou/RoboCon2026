# 任务赛生命周期脚本说明

`run_task_race_lifecycle.sh` 是任务赛的一键生命周期入口，用来统一启动任务赛视觉栈、启动任务赛主流程，并在识别阶段完成后自动关闭视觉栈。

## 它管理什么

这个脚本管理两部分：

- 任务赛视觉栈
- 任务赛主流程 launch

视觉栈启动使用：

```bash
/home/cat/start_perception_stack.sh
```

视觉栈关闭使用：

```bash
/home/cat/stop_perception_stack.sh
```

生命周期脚本本身放在：

```bash
/home/cat/hitcrt_quad2026_ws/scripts/run_task_race_lifecycle.sh
```

脚本放在工作区 `scripts/` 目录里，不会影响视觉栈管理。真正启动和关闭视觉的文件路径已经在 `run_task_race_lifecycle.sh` 里显式配置为 `/home/cat/start_perception_stack.sh` 和 `/home/cat/stop_perception_stack.sh`。

## 它不管理什么

这个脚本不启动、不关闭 point_lio。

运行生命周期脚本前，需要先在另一个终端启动雷达定位，并确认存在下面这个 TF：

```text
camera_init -> aft_mapped
```

生命周期脚本只会等待这个 TF 就绪。正常退出或清理时，它不会关闭 point_lio。

## 生命周期流程

1. 等待 `/tf` 话题出现。
2. 等待 `camera_init -> aft_mapped` 这组 TF 就绪。
3. 启动 `/home/cat/start_perception_stack.sh`。
4. 等待 `POST_PERCEPTION_DELAY_SEC` 秒，让相机、YOLO、OCR、eightboxes 稳定。
5. 后台启动 watcher，监听 `/quad/logistics_nav_event`。
6. 前台启动任务赛主流程：

```bash
ros2 launch /home/cat/hitcrt_quad2026_ws/src/launch/task_race.launch.py
```

7. 当 watcher 收到 `event_type=0`，也就是 `PRE_SCAN_DONE` 时，自动执行：

```bash
/home/cat/stop_perception_stack.sh
```

8. 视觉栈关闭后，任务赛主流程继续运行，进入取块、放块等后续流程。

## 使用方法

先在另一个终端启动 point_lio。

然后运行：

```bash
cd /home/cat/hitcrt_quad2026_ws
./scripts/run_task_race_lifecycle.sh
```

这个终端需要保持打开。任务赛主流程会在前台运行，方便直接观察日志。

如果需要中止任务赛，在这个终端按：

```text
Ctrl+C
```

脚本会停止任务赛主流程，并清理由本脚本启动过的视觉栈。

## 常用参数

可以在命令前临时覆盖默认参数。

调整视觉栈启动后等待时间：

```bash
POST_PERCEPTION_DELAY_SEC=8 ./scripts/run_task_race_lifecycle.sh
```

调整等待 TF 的超时时间：

```bash
TF_WAIT_TIMEOUT_SEC=180 ./scripts/run_task_race_lifecycle.sh
```

调整 TF 名称：

```bash
TF_PARENT=camera_init TF_CHILD=aft_mapped ./scripts/run_task_race_lifecycle.sh
```

默认触发事件是：

```bash
TRIGGER_EVENT=0
```

`0` 表示 `PRE_SCAN_DONE`，含义是预扫描和路径生成已经完成。此时关闭视觉栈，可以释放 RealSense、YOLO、eightboxes、OCR 占用的资源。

## 手动测试自动关闭视觉

如果想不跑完整任务赛，只测试 watcher 是否能自动关闭视觉，可以发布一条测试事件：

```bash
ros2 topic pub --once /quad/logistics_nav_event std_msgs/msg/Int32MultiArray "{data: [0, -1, 999, -1]}"
```

收到这条消息后，生命周期脚本里的 watcher 应该会调用：

```bash
/home/cat/stop_perception_stack.sh
```

## 注意事项

- 生命周期脚本调用 `/home/cat/start_perception_stack.sh` 时，会传入 `HITCRT_WS=/home/cat/hitcrt_quad2026_ws`，所以视觉日志会写到任务赛工作区下。
- 生命周期脚本停止视觉栈时不会使用 `--force`，所以默认不会关闭 point_lio。
- 如果 point_lio 没有启动，或者没有 `camera_init -> aft_mapped`，生命周期脚本会在启动视觉栈和任务赛主流程之前退出。
- 当前有两套视觉脚本：`/home/cat/start_perception_stack.sh` 和 `/home/cat/hitcrt_quad2026_ws/scripts/start_perception_stack.sh`。生命周期脚本默认使用 `/home/cat/` 下这一套。
