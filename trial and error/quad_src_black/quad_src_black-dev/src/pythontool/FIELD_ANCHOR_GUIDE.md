# 赛场取放货标点与点阵生成 — 全流程说明

本文介绍如何用遥控器在雷达地图上标 **四个锚点**（A1、B4、C1、C4），自动生成全部 8 个取货点 + 8 个放货点，并让 `lidar_nav_demo_node` 在任务赛中使用这些坐标。

---

## 1. 流程总览

```
手操走到 A1 → LB+RB 存点
手操走到 B4 → LB+RB 存点
手操走到 C1 → LB+RB 存点
手操走到 C4 → LB+RB 存点
（可选）LT+RT 删错最后一行
运行 generate_field_waypoints.py --viz  → 生成全据点 + 可视化
waypoints.yaml 里 use_field_layout_generated_points: true
重启 lidar_nav_demo_node → 路径规划使用生成坐标
```

| 阶段 | 工具 / 文件 |
|------|-------------|
| 遥控器标点 | `record_anchor_points.py` → `example_point.txt` |
| 离线计算 | `generate_field_waypoints.py` → `manip_points_generated.yaml` + `waypoints_generated.yaml` |
| 可视化检查 | `field_waypoints_viz.svg` |
| 导航加载 | `waypoints.yaml` 标志位 + `lidar_nav_demo_node` |

---

## 2. 四个锚点分别是什么

在地图上标定 **4 个物理位置**（每行 `x y yaw`）。生成脚本取 **最后 4 行**：

| 文件位置 | 锚点 | 含义 |
|----------|------|------|
| 倒数第 5 行 | **Startup** | 开局先取的货位（覆盖 `pickup_points[startup_pickup_index]`） |
| 倒数第 4 行 | **A1** | 前排取货 1（pickup index 0），AB 区原点 |
| 倒数第 3 行 | **B4** | 后排取货 4（pickup index 7），AB 区标定终点 |
| 倒数第 2 行 | **C1** | 底层放货 1（dropoff index 0） |
| 倒数第 1 行 | **C4** | 底层放货 4（dropoff index 3） |

前面可有多行试标点；生成前用 LT+RT 删掉多余行。末 5 行推荐顺序：**Startup → A1 → B4 → C1 → C4**。`startup_pickup_index` 在 `waypoints.yaml` 里配置（默认 2 = A3）。

### 坐标系

雷达系：**x 向右，y 向前**（yaw=0 时）。局部表 `(dx, dy)` 中 dx=横向，dy=纵向。

### 几何与生成方式（三行平行）

以 **A1 为原点** 建立场局部系：+x 沿货位**行方向**，+y 指向场地内侧（与 +x 正交）。A、B、C 三行共用同一行方向 `theta_row` 与缩放 `scale_ab`，因此**必然平行**。

| 行 | 相对 A1 的局部坐标 |
|----|-------------------|
| A 行 | (0,0), (0.85,0), (1.70,0), (2.55,0) |
| B 行 | 同上 dx，**dy = 1.61 m**（AB 纵向间距固定） |
| C 行 | dx = 0.075, 0.875, 1.675, 2.475；**dy = d_c**（由 C1、C4 锚点反投影求平均） |

**行方向 `theta_row`**：`theta_ab`（A1+B4 解出场坐标系 +x）与 `theta_c`（C1→C4 实测横向）的向量平均。

**缩放 `scale_ab`**：`|B4 − A1| / hypot(2.55, 1.61)`，由 AB 区定标。

**C 行纵向 `d_c`**：将 C1、C4 反投影到 `theta_row` 局部系，取纵向分量平均；C 行不再沿 C1→C4 弦插值，而与 AB 行平行。

生成时 A/B/C 均用 `local_to_map(A1, (dx, dy), theta_row, scale_ab)`：

- **A1**：精确（原点）
- **B4、C1、C4**：在平行约束下近似；若锚点与平行假设略冲突，终端会打印残差警告

**Yaw**：

- 放货 `forward_yaw`：C1、C4 记录 yaw 的平均
- 取货 `reverse_yaw`：A1、B4 记录 yaw 的平均
- 高层放货：低层 y + **0.025 m**，x/yaw 同低层

---

## 3. 相关文件路径

| 文件 | 路径 |
|------|------|
| 锚点记录 | `src/commands/lidar_nav_demo_cpp/config/trajectories/example_point.txt` |
| 生成结果 | `src/commands/lidar_nav_demo_cpp/config/manip_points_generated.yaml` |
| 可视化图 | `src/commands/lidar_nav_demo_cpp/config/field_waypoints_viz.svg` |
| 导航总配置 | `src/commands/lidar_nav_demo_cpp/config/waypoints.yaml` |
| 标点节点 | `src/pythontool/record_anchor_points.py` |
| 生成脚本 | `src/pythontool/generate_field_waypoints.py` |
| 几何库 | `src/pythontool/field_waypoint_math.py` |
| 可视化 | `src/pythontool/visualize_field_waypoints.py` |

`example_point.txt` 示例：

```
# 末 4 行: A1, B4, C1, C4
1.4500 2.4000 3.0800
1.5150 3.2550 3.0500
-0.8500 4.7400 -0.0500
1.5400 4.5400 -0.0500
```

---

## 4. 遥控器标点（实机）

### 4.1 前置条件

1. TF 正常：`camera_init` → `aft_mapped`。
2. **手操**（MANUAL）。
3. **RL 行走**：`LB + X`。
4. `task_race.launch.py` 默认启动 `record_anchor_points.py`（`anchor_recorder_enable:=false` 可关）。

### 4.2 按键

| 组合 | 作用 |
|------|------|
| **LB + RB** | 追加一行 `x y yaw` |
| **LT + RT** | 删除最后一行有效数据 |

仅在 **MANUAL + RL_MOVE** 下生效；按住不连发。

### 4.3 推荐走点顺序

1. 开局先取货位（**Startup**，走录制路径前那块）→ LB+RB  
2. 前排取货 1（**A1**）→ LB+RB  
3. 后排取货 4（**B4**）→ LB+RB  
4. 底层放货 1（**C1**）→ LB+RB  
5. 底层放货 4（**C4**）→ LB+RB  

确认 `example_point.txt` 末 5 行对应上述五点（末 4 行仍为 A1/B4/C1/C4）。

---

## 5. 生成全据点与可视化

```bash
cd /home/cat/hitcrt_quad_ws
python3 src/pythontool/generate_field_waypoints.py --viz
```

终端会打印 `theta_ab`、`theta_c`、`theta_row`、`d_c_row`、`scale_ab` 及 A1/B4/C1/C4 自检误差。A1 应 ≈ 0；B4/C1/C4 在平行约束下可能有毫米级残差。

`waypoints_generated.yaml` 每次从 **`waypoints.yaml` 复制全部参数**，仅替换 `pickup_points` / `dropoff_points` 为单行数组（C++ loader 可读）。其它逻辑参数（`startup_pickup_index`、`eightboxes_pickup_index_map` 等）与 `waypoints.yaml` 保持一致。

打开 `field_waypoints_viz.svg`：

- 蓝圆 **0~7**：取货点  
- 红方 **0~7**：放货点（0~3 低层，4~7 高层）  
- 星标：**A1**（绿）、**B4**（紫）、**C1**（棕）、**C4**（橙）  
- 虚线：A/B/C 三行延长线，用于目视检查平行性

---

## 6. 让导航使用生成点

`waypoints.yaml`：

```yaml
use_field_layout_generated_points: true
field_generated_points_file: ".../waypoints_generated.yaml"
```

改 `use_field_layout_generated_points: true` 后重启节点，启动时从 `waypoints_generated.yaml`（或 `manip_points_generated.yaml`）覆盖取放点坐标；其余导航参数仍读 `waypoints.yaml`。

---

## 7. 常见问题

**生成报 Need at least 4 anchor points**  
末 4 行有效数据不足；按 A1→B4→C1→C4 补全。

**图上 AB 横向间距不对**  
检查 A1、B4 是否标在正确坑位；横向间距应为约 0.85 m × scale_ab。

**C1/C4 与星标略有偏差**  
正常；三行平行约束下 C 行由 `d_c` 拟合，不再沿 C1→C4 弦强行对齐。残差 > 1 cm 时重新标 C1、C4。

**三行不平行**  
检查 `theta_ab-c delta`；若 > 2° 说明 AB 与 C 锚点横向方向不一致，需重标锚点。

**LB+RB 无反应**  
确认 MANUAL + RL_MOVE、`record_anchor_points.py` 在运行、TF 有数据。

---

## 8. 检查清单

- [ ] 末 4 行：A1、B4、C1、C4  
- [ ] `generate_field_waypoints.py --viz` 成功，A1 自检 ≈ 0，B4/C1/C4 残差可接受  
- [ ] SVG 中 A/B/C 虚线延长线视觉平行
- [ ] `use_field_layout_generated_points: true` 且重启后日志有 Loaded field-layout generated manip points
