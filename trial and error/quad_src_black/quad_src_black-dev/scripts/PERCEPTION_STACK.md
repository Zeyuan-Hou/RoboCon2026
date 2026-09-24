# 任务赛感知栈一键启动

本目录脚本用于按依赖顺序启动：**RealSense → YOLOv8 → eightboxes → PaddleOCR**（**雷达定位需手动先开**），供 `lidar_nav_demo_cpp` 任务赛使用。

## 快速开始

```bash
cd /home/cat/hitcrt_quad_ws
chmod +x scripts/*.sh scripts/lib/*.sh

# 启动（默认 tmux 会话 perception_stack）
./scripts/start_perception_stack.sh

# 查看各节点日志窗格
tmux attach -t perception_stack   # 相机 + 物块识别
tmux attach -t ocr_sys            # 智力题 OCR（默认一并启动）

# 停止
./scripts/stop_perception_stack.sh
# 若节点未退出，可强制：
./scripts/stop_perception_stack.sh --force
```

启动完成后再开主控栈：

```bash
source install/setup.bash
ros2 launch /home/cat/hitcrt_quad_ws/src/launch/task_race.launch.py
```

## 启动顺序

**第 0 步（手动，另开终端）— 雷达定位：**

```bash
cd ~/point_lio   # 或你的 point_lio 路径
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch point_lio mapping_mid360.launch.py
```

**本脚本自动启动：**

| 顺序 | 节点 | 等待条件 |
|------|------|----------|
| 1 | `realsense_node` | `/color/image_raw` 有数据 |
| 2 | **PaddleOCR**（`ppocr_node` + `ocr_result_subscriber_node`） | RealSense 就绪后启动；发布 `/stable_arithmetic_result` |
| 3 | `yolov8_node` | 相机就绪后 warmup，可选 `/target_3d_position` |
| 4 | `eightboxes_node` | 等待 `/tf` 与 `camera_init` → `aft_mapped` 后启动 |

## 路径配置（环境变量）

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `REALSENSE_WS` | `/home/cat/realsense` | RealSense 工作区 |
| `ROS2_WS` | `/home/cat/ros2_ws` | YOLOv8 / eightboxes 工作区 |
| `PADDLEOCR_WS` | `/home/cat/paddleocrDeploy_rk3588` | OCR 工作区 |
| `WAIT_TIMEOUT_SEC` | `120` | 各等待步骤超时（秒） |
| `REALSENSE_FPS` | `15` | 深度相机帧率 |
| `START_OCR` | `1` | 设为 `0` 可跳过 OCR |
| `START_OCR_USBCAM` | `0` | 设为 `1` 额外启 USB 相机（发布 `/image_raw`，默认用 RealSense 的 `/color/image_raw`） |
| `OCR_SESSION` | `ocr_sys` | OCR 专用 tmux 会话 |
| `OCR_WARMUP_SEC` | `5` | OCR 节点启动后等待秒数 |
| `TMUX_SESSION` | `perception_stack` | 主 tmux 会话名 |

示例：

```bash
WAIT_TIMEOUT_SEC=180 ./scripts/start_perception_stack.sh
```

## OCR（智力题）说明

默认 **已包含在** `start_perception_stack.sh` 中（`START_OCR=1`），独立 tmux 会话 `ocr_sys`：

| 节点 | 作用 | 话题 |
|------|------|------|
| `ppocr_node` | 识别题目 | 订阅 `/color/image_raw`，发布 `/ocr_results` |
| `ocr_result_subscriber_node` | 稳定算术结果 | 发布 `/stable_arithmetic_result`（`lidar_nav` 订阅） |

图像来自 **RealSense** 的 `/color/image_raw`，与 `start_ocr_system.sh` 里可选的 USB 相机（`/image_raw`）不同。若必须用 USB 相机：

```bash
START_OCR_USBCAM=1 ./scripts/start_perception_stack.sh
```

单独手动启动（与仓库原脚本一致）：

```bash
cd /home/cat/paddleocrDeploy_rk3588
./start_ocr_system.sh
tmux attach -t ocr_sys
```

## RealSense 单独使用

**与脚本一起（推荐）**：`start_perception_stack.sh` 会在窗格 `realsense` 中执行：

```bash
ros2 run realsense realsense_node --fps 15
```

**手动启动**：

```bash
cd /home/cat/realsense
source /opt/ros/humble/setup.bash
source install/setup.bash

# 常用：15Hz，无窗口
ros2 run realsense realsense_node --fps 15

# 本地预览（按 q 退出窗口）
ros2 run realsense realsense_node --fps 30 --visualize

# 等价 launch（在 launch 文件里改 fps/visualize）
ros2 launch realsense realsense.launch.py
```

发布话题：

- `/color/image_raw`、`/depth/image_raw`
- `/color/camera_info`、`/depth/camera_info`

`yolov8_node` 默认订阅上述话题，**必须先启动 RealSense，再启动 yolov8**。

## 与任务赛导航的对接

| 话题 | 发布者 | 订阅者 |
|------|--------|--------|
| `/eightboxes` | `eightboxes_node` | `lidar_nav_demo_node` |
| `/stable_arithmetic_result` | PaddleOCR | `lidar_nav_demo_node` |
| `/tf` (`camera_init`→`aft_mapped`) | `point_lio` | `eightboxes_node`、导航栈 |

## 验证命令

```bash
source /opt/ros/humble/setup.bash

# 定位 TF
ros2 run tf2_ros tf2_echo camera_init aft_mapped

# 相机
ros2 topic echo /color/image_raw --once

# 物块检测链
ros2 topic echo /target_3d_position --once
ros2 topic echo /eightboxes --once

# 智力题 OCR
ros2 topic echo /stable_arithmetic_result --once
```

## 日志

各窗格输出追加到：

```text
/home/cat/hitcrt_quad_ws/logs/perception/
  realsense.log
  yolov8.log
  eightboxes.log
  ppocr.log
  ocr_result.log
  usbcam.log          # 仅 START_OCR_USBCAM=1 时
```

## tmux 操作

```text
tmux attach -t perception_stack   # 进入主会话
Ctrl+b 方向键                      # 切换窗格
Ctrl+b d                          # 分离（后台继续跑）
tmux attach -t ocr_sys            # OCR（ppocr + ocr_result）
```

## 文件说明

| 文件 | 作用 |
|------|------|
| `start_perception_stack.sh` | 主启动脚本 |
| `stop_perception_stack.sh` | 停止 tmux 与可选进程清理 |
| `lib/wait_ros_ready.sh` | topic / TF 等待工具函数 |

## 常见问题

1. **`AMENT_TRACE_SETUP_FILES: 未绑定的变量`**  
   旧版脚本使用了 `set -u`，与 ROS `setup.bash` 不兼容。请更新到最新 `start_perception_stack.sh`（已改为 `set -eo pipefail`）。

2. **手动 tf2_echo 有 Translation，脚本仍说收不到**  
   - 原因常为：探测时间太短（启动负载高时前几秒只有 `Invalid frame ID`，后面才有 `Translation`）。  
   - 已改为等待首条 `- Translation:` 最多 15s。  
   - 注意：若在 OCR 启动处按了 **Ctrl+C**，脚本会中断，根本还没跑到 TF 检测步骤。  
   - 自检与脚本相同：`timeout 15 ros2 run tf2_ros tf2_echo camera_init aft_mapped 2>&1 | grep -m1 Translation`  
   - 临时跳过：`SKIP_TF_WAIT=1 ./scripts/start_perception_stack.sh`

3. **`/eightboxes` 一直不出现**  
   需要 YOLO 检测到物块且 8 个区域都识别完；查看 `logs/perception/yolov8.log` 与 `eightboxes.log`。

4. **RealSense 打不开**  
   检查 USB、是否已有进程占用相机：`stop_perception_stack.sh --force` 后重试。

5. **OCR 未启动**  
   确认 `PADDLEOCR_WS/start_ocr_system.sh` 可执行，且已安装 `tmux`、`uv`。
