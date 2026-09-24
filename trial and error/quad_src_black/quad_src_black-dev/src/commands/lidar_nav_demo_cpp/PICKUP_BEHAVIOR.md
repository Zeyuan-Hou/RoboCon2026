# 取块行为与参数说明

简要描述任务赛从启动到取物块、再放物的整体流程，以及主要可调参数所在位置。

**坐标系**：雷达/地图系，`x` 向右，`y` 向前，`yaw` 逆时针为正（rad）。

**主配置文件**：

| 文件 | 作用 |
|------|------|
| [`config/waypoints.yaml`](config/waypoints.yaml) | 取/放点坐标、导航控制、到点阈值 |
| [`config/prepath.yaml`](config/prepath.yaml) | 赛前左右看预扫描 |
| [`config/add_point.txt`](config/add_point.txt) | A* 走廊节点（跨隔离带路径） |
| [`src/config/quad_run_cfg.yaml`](../../config/quad_run_cfg.yaml) | `task_manager` 机械臂时序与命令值 |
| [`config/waypoints.yaml` 末尾 `task_manager_node`](config/waypoints.yaml) | 机械臂一键开关 `manipulation.enable` |

**启动**：`ros2 launch src/launch/task_race.launch.py`，手柄 `LT+A` 进入任务赛。

---

## 1. 整体流程

```text
ENTER_AUTO
  -> LOGISTICS_INIT_MANIP     发布 init_cmd，等待 init_to_place_low_wait_s
  -> LOGISTICS_PRE_SCAN       导航 enable；原地左右看；等 eightboxes/OCR
  -> LOGISTICS_LIFT_ARM       路径生成后 PRE_SCAN_DONE；发布 lift_standby_cmd
  -> LOGISTICS_NAV            按路径依次：过渡点 -> 取货点 -> 放货点 -> ...
       |
       +-- 到达 PICKUP 点
       |     -> TASK_PICKUP_BLOCK
       |        暂停导航 -> pre_stand_wait_s -> 机体停稳 -> pickup_cmd
       |        -> 等 result / pickup_wait_s -> 恢复 LOGISTICS_NAV
       |
       +-- 到达 DROPOFF 点
             -> TASK_DROPOFF_BLOCK（低层走路放 / 高层切站立放，见放货文档）
```

每个「取一块」任务在路径上为三段 A*：

```text
当前位置 --[TRANSITION...]--> staging 中继点 --[TRANSITION...]--> PICKUP --[TRANSITION...]--> DROPOFF
```

- **staging**：在 `add_point.txt` 图里，按前排/后排选最近节点（前排 `y ≤ pickup_front_staging_y_max`，后排 `y > pickup_rear_staging_y_min`）。
- **PICKUP**：精确坐标 + 目标 yaw（取货朝向）。
- 中间 **TRANSITION** 点目标 yaw 固定为 `0`（后排走廊内 TRANSITION 不控 yaw，见下文）。

---

## 2. 路径与取货顺序

### 2.1 输入

| 来源 | 话题/参数 | 含义 |
|------|-----------|------|
| eightboxes | `/eightboxes` (`String`，8 位) | 8 个货位上的物块类别 `0~3`（绿/灰/蓝/红） |
| OCR | 内部 `ocr_result` | 若锁定，**同类物块先取** |
| 兜底 | `default_priorities` | 无 eightboxes 时用固定优先级生成路径 |

### 2.2 映射关系

- **`eightboxes_pickup_index_map`**：字符串下标 `0~7` → `pickup_points` 物理下标。默认 `[4,5,6,7,0,1,2,3]`（前 4 位后排，后 4 位前排）。
- **`pickup_points`**：每点 4 元组 `[x, y, yaw, is_front]`，`is_front=1` 前排，`0` 后排。
- **`dropoff_color_order`** + **`dropoff_layer_stride`**：物块类别 → 放货坑位（第一层 index 0~3，第二层 +4）。

### 2.3 时序参数（导航节点）

| 参数 | 默认 | 说明 |
|------|------|------|
| `ocr_wait_after_eightboxes_sec` | 8.0 s | 收到 eightboxes 后等 OCR；超时则按左→右顺序 |
| `path_fallback_timeout_sec` | 5.0 s | 未收到 eightboxes 的兜底超时 |
| `pre_scan_timeout_s` | 8.0 s | 左右看最长等待（`prepath.yaml`） |
| `pre_path_spin_target_yaws` | [0.5,-0.5,-0.15] | 预扫描转向序列 |

路径生成完成后发布 **`PRE_SCAN_DONE`**，`task_manager` 抬臂并进入正式导航。

---

## 3. 导航到取货点

控制器：`logistics_controller_type: "heading_dock"`（推荐）。

### 3.1 分阶段接近（距目标欧氏距离 `dist`）

| 阶段 | 条件 | 行为 |
|------|------|------|
| **APPROACH** | `dist > approach_to_align_dist` (0.70 m) | 远距离，偏纵向 + bearing 转向 |
| **ALIGN** | `dock_start_dist < dist ≤ 0.70 m` | 近距离对准，可横移 |
| **DOCK** | `dist ≤ dock_start_dist` (0.10 m) | 低速精调：`kp_dock_x/y`，creep |

TRANSITION / PICKUP / DROPOFF **共用**上述距离切换；取货点进入 PICKUP 类型后仍会做 yaw 精对准。

### 3.2 特殊区域

| 机制 | 参数 | 作用 |
|------|------|------|
| 跨隔离带 | `isolation_band_y_min/max` (0.9~3.8) | 路径穿过隔离带时限速；Y 到点可放宽 |
| 精确定位 Y 段 | `precise_lateral_arrival_y_zone{1,2}_*` | 指定 Y 段 TRANSITION 横向到点更严 |
| 后排 yaw 禁用 | `pickup_rear_staging_y_min` ~ `rear_corridor_yaw_free_y_max` (3.5~4.5) | 该区间 **TRANSITION 不控 yaw**，避免对 target_yaw=0 猛转 |
| 取货点 | 类型 `PICKUP` | **始终控 yaw**，用 `manip_yaw_tolerance` 到点 |

### 3.3 到点判定（PICKUP）

| 参数 | 默认 | 说明 |
|------|------|------|
| `manip_position_tolerance` | 0.06 m | 平面距离阈值 |
| `manip_yaw_tolerance` | 0.08 rad | 航向阈值 |
| `manip_arrival_stable_time_s` | 0.25 s | 连续满足后才确认到点 |

到点后发布 **`PICKUP_REACHED`**，并暂停导航（`external_manipulation_enable=true` 时由 task_manager 接管；`false` 时本地 `wait_time` 等待）。

---

## 4. 取块动作（task_manager）

到达 PICKUP 后，`TASK_PICKUP_BLOCK` 执行（**走路模式，不切站立**）：

```text
STOP_NAV
  -> PRE_STAND_WAIT          pre_stand_wait_s
  -> ARM_CMD_SETTLE          等 cmd_vel 归零或 arm_cmd_settle_s
  -> 发布 pickup_cmd 一次    默认 1 (GRAB)
  -> WAIT_ARM                等 /manipulator/result 或 pickup_wait_s
  -> DONE                    恢复 LOGISTICS_NAV
```

### 4.1 机械臂参数（`quad_run_cfg.yaml` → `manipulation.*`）

| 参数 | 默认 | 说明 |
|------|------|------|
| `pickup_cmd` | 1 | 取物命令值 |
| `pickup_wait_s` | 4.0 s | 等 result 超时兜底 |
| `pre_stand_wait_s` | 0.4 s | 到点后静止再动臂 |
| `arm_cmd_settle_s` | 0.4 s | 等机体停稳上限 |
| `init_cmd` | 0 | 任务赛开始初始化 |
| `lift_standby_cmd` | 4 | 预扫描完成后抬臂待机 |

### 4.2 机械臂总开关（`waypoints.yaml` 末尾）

```yaml
/quad/task_manager_node:
  ros__parameters:
    manipulation.enable: true              # false：不发 /manipulator/cmd，流程仍走
    manipulation.disabled_result_wait_s: 0.5
```

`external_manipulation_enable`（导航节点）：`true` 时到点交 task_manager；`false` 时仅本地 `wait_time`（2.0 s）模拟等待。

---

## 5. 单块任务路径示意

```text
[当前] --A*--> [staging] --A*--> [PICKUP 点] --A*--> [DROPOFF 点] --下一任务-->
```

- 前排取货：staging 在 `y ≤ 1.5` 的图节点附近。
- 后排取货：staging 在 `y > 3.5` 的图节点；走廊内 TRANSITION 保持当前朝向。
- 取货完成后同路径继续导航至对应 **DROPOFF**（低层/高层逻辑见 `dropoff_high_layer_min_index`）。

---

## 6. 常用调试

```bash
ros2 topic echo /quad/current_state
ros2 topic echo /quad/logistics_nav_enable
ros2 topic echo /quad/logistics_nav_event    # 0=PRE_SCAN_DONE 1=PICKUP 2=DROPOFF 3=FINISHED
ros2 topic echo /eightboxes
ros2 topic echo /manipulator/cmd
```

导航周期日志（`lidar_nav_demo_node`）：关注 `stage=`（APPROACH/ALIGN/DOCK）、`yaw_free=`、`dist=`、`tol=`。

更完整的联调说明见 [`TASK_RACE_DEBUG_GUIDE.md`](TASK_RACE_DEBUG_GUIDE.md)。
