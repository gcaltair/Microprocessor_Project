# 终点与转向抖动手动测试记录

## 测试引用

- 日期：2026-05-13
- 测试者：待填写
- 相关测试计划：`docs/work_items/2026-05-13_control-jitter-debug/hardware_test_plan.md`
- 固件构建：待填写
- 环境：待填写

## 执行

- 是否严格按计划执行：待填写
- 如未严格执行，变化：待填写

## 观察

- 串口 / 日志观察：待填写
- 机器人行为观察：待填写
- 地图 / 定位观察：待填写

## 重点记录字段

每轮测试请尽量保留：

```text
ODOM ...
MOVE ...
CTRLDBG pos_err=... ang_err=... turn=... l_sp=... r_sp=... pwm=(...)
LCTRL pos_err=... ang_err=... base=... turn=... l_sp=... r_sp=... pwm=(...)
TCTRL ang_err=... base=... turn=... l_sp=... r_sp=... pwm=(...)
ENC ...
LMOVE ...
```

判断重点：

- 终点附近 `turn` 是否被压低。
- 终点附近 `l_sp/r_sp` 是否避免频繁一正一负。
- 动作结束后 `LCTRL` 是否显示最后一轮驱动控制仍平稳。
- 转角结束后 `TCTRL` 是否显示目标角附近仍有过大转向输出。
- `A90 / A-90` 目标角附近是否仍反复抖动。

## 结果

- 状态：blocked
- 主要原因：等待实车硬件验证

## 偏差

- 预期：终点和转角抖动下降。
- 实际：待填写。

## 后续

- 必要修复：根据 `CTRLDBG` 和实体表现决定是否继续调低角度 Kp / 转向限幅 / 电机死区补偿。
- 是否需要复测：是。
- 相关文件：
  - `Core/Src/pid.c`
  - `Core/Src/hc04.c`
  - `Core/Src/system.h`
