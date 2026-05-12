# 手工测试记录

## 测试引用信息

- 日期：
- 测试者：
- 对应测试计划：`hardware_test_plan.md`
- 固件构建版本：
- 测试环境：

## 执行情况

- 是否严格按计划执行？
- 如果没有，改动了什么？

## 分步骤回填

### 步骤 1：`R0` 之后的基线状态

- 已发送命令：R0
- 原始 `O` 输出：
```DATA
OODOM x=0.000 y=0.000 th=0.00 ls=0.000 rs=0.000 base=0.000 ang_sp=0.00 mode=MANUAL move=IDLE
MOVE cmd=(0.000,0.000) target=0.000 progress=0.000 remain=0.000
ENC  dl=0.000 dr=0.000 cntL=0 cntR=0 cal=(1.000,1.000,1.000,1.000)
EST  x=0.000 y=0.000 th=0.00 CTRL=(0.000,0.000,0.00)
POSE pred=(0.000,0.000,0.00) corr=(0.000,0.000,0.00) slow=(0.000,0.000,0.00)

```
- `ODOM` 是否接近零：
- `MOVE` 是否接近零：
- `EST / CTRL` 是否接近零：

### 步骤 2：手动转向检查

- 已发送命令：R0 L 
- `L` 的行为观察：卡顿的转向,存在来回的卡顿（可能是超调）
- `L` 之后原始 `O` 输出：
 ```
 OODOM x=-0.027 y=0.002 th=-10.22 ls=0.000 rs=0.000 base=0.000 ang_sp=-10.00 mode=MANUAL move=IDLE
  MOVE cmd=(0.000,0.000) target=0.000 progress=0.000 remain=0.000
  ENC  dl=0.021 dr=-0.075 cntL=56 cntR=-401 cal=(1.000,1.000,1.000,1.000)
  EST  x=-0.027 y=0.002 th=-10.22 CTRL=(-0.027,0.002,-10.22)
  POSE pred=(0.000,0.000,0.00) corr=(0.000,0.000,0.00) slow=(0.000,0.000,0.00)
```
- `R` 的行为观察：
- `R` 之后原始 `O` 输出：卡顿的转向,存在来回的卡顿（可能是超调）
```data
OODOM x=-0.025 y=0.002 th=-0.88 ls=0.000 rs=0.000 base=0.000 ang_sp=-0.22 mode=MANUAL move=IDLE
MOVE cmd=(0.000,0.000) target=0.000 progress=0.000 remain=0.000
ENC  dl=0.054 dr=-0.104 cntL=133 cntR=-651 cal=(1.000,1.000,1.000,1.000)
EST  x=-0.025 y=0.002 th=-0.88 CTRL=(-0.025,0.002,-0.88)
POSE pred=(0.000,0.000,0.00) corr=(0.000,0.000,0.00) slow=(0.000,0.000,0.00)
```
- `L / R` 是否表现为相对转向：是相对转向

### 步骤 3：前进 1.0 m 测试

- 已发送命令：P1,0
- 原始 `K` 输出：
  KKENC cal lf=1.000 lr=1.000 rf=1.000 rr=1.000

- 原始 `O` 输出：
```DATA
OOODOM x=0.984 y=-0.011 th=-0.68 ls=0.000 rs=0.000 base=0.000 ang_sp=-1.23 mode=MANUAL move=IDLE
MOVE cmd=(1.000,0.000) target=1.000 progress=0.957 remain=0.000
ENC  dl=0.793 dr=1.176 cntL=1688 cntR=2678 cal=(1.000,1.000,1.000,1.000)
EST  x=0.984 y=-0.011 th=-0.68 CTRL=(0.984,-0.011,-0.68)
POSE pred=(0.000,0.000,0.00) corr=(0.000,0.000,0.00) slow=(0.000,0.000,0.00)
```
- 实际停车距离：0.82m
- 航向漂移观察：基本不漂
- `MOVE target/progress/remain`：
- `ENC dl/dr/cntL/cntR`：

### 步骤 4：倒车 1.0 m 测试

- 已发送命令:P-1,0
- 原始 `O` 输出：
```data
OOODOM x=0.046 y=0.001 th=-1.29 ls=0.000 rs=0.000 base=0.000 ang_sp=-1.46 mode=MANUAL move=IDLE
MOVE cmd=(-1.000,0.000) target=1.000 progress=0.950 remain=0.000
ENC  dl=-0.337 dr=0.430 cntL=-408 cntR=1258 cal=(1.000,1.000,1.000,1.000)
EST  x=0.046 y=0.001 th=-1.29 CTRL=(0.046,0.001,-1.29)
POSE pred=(0.000,0.000,0.00) corr=(0.000,0.000,0.00) slow=(0.000,0.000,0.00)
```
- 实际停车距离：-1.1m
- 航向漂移观察：基本不漂移
- `MOVE target/progress/remain`：
- `ENC dl/dr/cntL/cntR`：

### 步骤 5：标定辅助命令（如使用）

- 已发送命令：
- 原始 `D...` 输出：
  D0.82D0.82CAL suggest forward: left=1.034 right=0.697 from actual=0.820 dl=0.793 dr=1.176
  Use: K1.034,1.000,0.697,1.000
- 建议的 `K...`：
- 是否应用了建议的 `K...`：

## 观察记录

- 串口/日志观察：
- 前进阶段的 `MOVE target/progress/remain`：
- 前进阶段的 `ENC dl/dr/cntL/cntR`：
- 倒车阶段的 `ENC dl/dr/cntL/cntR`：
- 执行 `P1.0,0.0` 时相对位移冲过头的现象：
- `D...` 建议输出：
- 机器人行为观察：
- 前进 1.0 m 的实体结果：
- 倒车 1.0 m 的实体结果：

## 测试结果

- 状态：阻塞
- 主要原因：硬件测试无法由代理执行，必须由人工测试者完成

## 偏差说明

- 预期：
- 实际：

## 后续处理

- 需要修复的问题：
- 是否需要复测：是
- 根据判读矩阵推测的主要原因：
- 建议的下一条命令或系数调整：
- 相关文件：
  - `Core/Src/encoder.c`
  - `Core/Src/encoder.h`
  - `Core/Src/hc04.c`
  - `Core/Src/pid.c`
