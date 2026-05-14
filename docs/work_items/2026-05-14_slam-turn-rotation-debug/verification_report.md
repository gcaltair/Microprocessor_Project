# SLAM 转向后地图旋转问题验证记录

## 本轮验证范围

本轮只做文档整理和代码静态诊断，没有修改固件行为。

## 已检查内容

- `Core/Src/localization_task.c`
- `Core/Src/mapping_task.c`
- `Core/Src/hc04.c`
- `Core/Inc/localization_task.h`
- `Core/Inc/mapping_task.h`
- `docs/development_plan/slam_icp_progress_status.md`
- `docs/development_plan/README.md`
- `docs/freertos_further_development_plan.md`

## 静态诊断结论

当前实现已经有基础保护：

- 转向期间暂停建图。
- 转向结束后 `200 ms` settle 窗口暂停建图。
- 单帧 odom 位移/角度过大时跳过建图。
- scan usable point 太少时跳过建图。

但当前保护仍不足以保证转向后的地图不会被错误姿态污染：

- ICP rejected 或 odometry-only 后，只要 mapping gate 通过，仍可能写图。
- 当前参考扫描是局部连续参考，不是全局 map matching。
- 现有 `P/G/O` 输出能看到 mode、inliers、fitness、gate，但还不能直接看到每次 ICP 的 `delta_theta`。

## 构建/测试结果

- 固件构建：未运行。本轮没有改固件代码。
- Host app 测试：未运行。本轮没有改 host app 代码。

## 最高验证等级

`Level 0: static review`

## 待硬件验证

执行同目录下 `hardware_test_plan.md`，确认：

- 转向后第一批 active scan 是否在 ICP accepted 前写入地图。
- `LOC mode / inliers / fit_mm` 与地图旋转的对应关系。
- `ODOM th / EST th / POSE pred/corr` 是否出现 yaw 不一致。
