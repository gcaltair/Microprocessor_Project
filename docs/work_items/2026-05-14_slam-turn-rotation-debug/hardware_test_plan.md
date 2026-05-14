# SLAM 转向后地图旋转问题硬件测试计划

## 测试目标

确认“转向后地图也会转向”的直接来源：

- ICP 没有接受匹配
- ICP 接受但 yaw 修正错误
- 建图 gate 过早恢复写图
- LiDAR 角度/安装方向存在系统偏差

## 前提条件

- 使用当前控制基线固件。
- 小车放在空旷、墙体轮廓清晰的位置。
- 先不要进入导航命令 `Jx,y`。
- 若现场接近墙体，先保证转向不会撞墙。

## 命令序列

### 1. 静止基线

```text
R0
Z
M
```

等待 3 到 5 秒，让静止地图形成第一版参考。

```text
P
G
O
X4
```

记录：

- `LOC init/updates/accept/reject/odom/mode/pts/inliers/fit_mm`
- `MAP loc mode/inliers/fit_mm`
- `MAP gate`
- `ODOM th`
- `EST th`
- `POSE pred/corr`
- `X4` 地图截图或文本

### 2. 原地转向 90 度

```text
A90
```

等待动作完全结束，再额外等待 2 秒。

```text
P
G
O
X4
```

记录同上。

### 3. 反向转回

```text
A-90
```

等待动作完全结束，再额外等待 2 秒。

```text
P
G
O
X4
```

记录同上。

### 4. 小角度转向对照

```text
R0
Z
M
```

等待 3 到 5 秒。

```text
A20
```

等待动作结束后 2 秒。

```text
P
G
O
X4
```

## 预期结果

理想状态：

- 静止时 `LOC mode` 多数为 accepted 或稳定初始化。
- 转向结束后，短暂出现 `MAP gate=paused(settle)` 是正常的。
- settle 后恢复写图时，应至少看到一次质量较好的 ICP accepted。
- `EST th` 和 `ODOM th` 不应长期大幅分离。
- `X4` 中已有墙体不应整体旋转出第二套轮廓。

## 失败判据

任一情况都说明需要改 SLAM 链路：

- 转向后 `LOC reject` 或 `odom-only` 增加明显，但 `MAP gate=active` 继续写图。
- `LOC accepted`，但 `fit_mm` 偏大或 `inliers` 很低，地图仍被写入。
- `ODOM th` 与 `EST th` 看似合理，但地图仍旋转，优先怀疑 LiDAR 角度方向或投影公式。
- `MAP gate` 在转向结束后过快恢复 active，且第一批 active 扫描造成地图重影。

## 记录格式

每轮请保存：

- 命令序列
- `P` 输出
- `G` 输出
- `O` 输出
- `X4` 输出或上位机截图
- 肉眼观察：地图是否出现第二套旋转墙体、机器人红点是否合理

## 安全停止

```text
S
N
```

必要时直接断电。
