# 取货顺序规划与可视化

本文记录 `lidar_nav_demo_node` 当前的取货任务排序规则，以及如何查看生成后的二维顺序图。

## 输入含义

- `/eightboxes`：8 位字符串，例如 `02313102`。每一位是该槽位的货物类别。
- OCR：稳定锁定后得到 `ocr_result_`，表示优先取的货物类别。
- `eightboxes_pickup_index_map`：把 eightboxes 字符串下标映射到物理取货点。当前配置为 `[4, 5, 6, 7, 3, 2, 1, 0]`。
- 生成点阵中 `pickup_points` 的物理下标为 `0~3=A1~A4`，`4~7=B1~B4`。

收到 `/eightboxes` 后，节点会保留原始 8 位字符串用于图上显示；实际参与任务规划前，会仅将 A 区对应的后四位类别反转一次。这样不改变既有 `eightboxes_pickup_index_map`，只修正 A 区颜色左右相反的问题。

## 排序规则

排序在 `LidarNavControl::build_tasks()` 中完成。

1. 开局固定点先取：如果 `startup_pickup_enable=true`，先取 `startup_pickup_index`。当前配置 `startup_pickup_index=2`，对应 A3。
2. OCR 优先阶段：如果 OCR 已锁定，先处理所有 `classes[i] == ocr_result_` 的货物。
3. 优先货物在 B 行时，直接取该 B 点。
4. 优先货物在 A 行时，先检查同列 B 点；如果同列 B 点还没取且是有效货物类别，则先取 B，再取 A。例如 A1 优先时顺序为 B1 -> A1。
5. 如果 A3 已被开局固定点取走，则不会因为 A3 是优先颜色而额外取同列 B3；但如果 B3 本身也是优先颜色，B3 仍会在 OCR 优先阶段被取。
6. OCR 优先阶段结束后，剩余取货点按距离贪心：以上一次放货点的 xy 坐标为起点，选择最近的未取 pickup。
7. 剩余阶段仍遵守前后排依赖：如果某个 A 点同列 B 点还没取，A 点暂不作为候选，先取 B 点。

## 查看可视化

每次成功生成路径后，会自动输出：

```bash
/home/cat/hitcrt_quad2026_ws/maps/lidar_nav_path_viz/pickup_order.svg
```

图中元素：

- 圆形：8 个取货点。
- 圆内数字 `1~8`：真实生成的取货顺序。
- `-`：该点未进入任务，通常是无效类别或任务数不足。
- 圆下方标签：物理点名、eightboxes 槽位、颜色简写。
- 方块 `D0~D7`：放货点，用于观察“从哪个放货区去最近 pickup”。
- 黑色折线：按取货顺序连接 pickup 点，方便快速检查整体顺序。

终端日志也会打印每个任务：

```text
Route order append startup: ...
Route order append ocr_front_dependency_rear: ...
Route order append ocr_priority: ...
Route order append nearest_remaining: ...
Task[0]: pickup[...] -> dropoff[...]
```

其中 `Route order append` 的 reason 能看出每个取货点是由开局固定、OCR 前置依赖、OCR 优先，还是最近邻补全加入的。
