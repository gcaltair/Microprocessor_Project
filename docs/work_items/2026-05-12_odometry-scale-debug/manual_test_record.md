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

## 第二轮隔离测试：新版 `LMOVE / CAL diag`

- 日期：2026-05-13
- 固件特征：`O` 已包含 `LMOVE`，`D...` 已包含 `CAL source` 和 `CAL diag`
- 当前标定系数：`cal=(1.000,1.000,1.000,1.000)`

### 基线复位

- 已发送命令：`R0`
- 原始 `O` 输出：

```text
ODOM x=0.000 y=0.000 th=0.00 ls=0.000 rs=0.000 base=0.000 ang_sp=0.00 mode=MANUAL move=IDLE
MOVE cmd=(0.000,0.000) target=0.000 progress=0.000 remain=0.000
ENC  dl=0.000 dr=0.000 cntL=0 cntR=0 cal=(1.000,1.000,1.000,1.000)
LMOVE valid=0 cmd=0.000 progress=0.000 dl=0.000 dr=0.000
EST  x=0.000 y=0.000 th=0.00 CTRL=(0.000,0.000,0.00)
POSE pred=(0.000,0.000,0.00) corr=(0.000,0.000,0.00) slow=(0.000,0.000,0.00)
```

### 前进 1.0 m 单段

- 已发送命令：`P1,0`
- 实体实际距离：`0.83 m`
- 原始 `O` 输出：

```text
ODOM x=0.959 y=-0.017 th=-1.12 ls=0.000 rs=0.000 base=0.000 ang_sp=-0.30 mode=MANUAL move=IDLE
MOVE cmd=(1.000,0.000) target=1.000 progress=0.952 remain=0.000
ENC  dl=0.824 dr=1.097 cntL=1670 cntR=2130 cal=(1.000,1.000,1.000,1.000)
LMOVE valid=1 cmd=1.000 progress=0.952 dl=0.838 dr=1.069
EST  x=0.959 y=-0.017 th=-1.12 CTRL=(0.959,-0.017,-1.12)
POSE pred=(0.000,0.000,0.00) corr=(0.000,0.000,0.00) slow=(0.000,0.000,0.00)
```

- 原始 `D...` 输出：

```text
CAL source=last_move cmd=1.000 progress=0.952 dl=0.838 dr=1.069
CAL diag left_err=1.0% right_err=28.8% asym=24.2%
CAL suggest forward: left=0.990 right=0.776 from actual=0.830 dl=0.838 dr=1.069
Use: K0.990,1.000,0.776,1.000
```

### 倒车 1.0 m 单段

- 已发送命令：`P-1,0`
- 实体实际距离：`-1.12 m`
- 原始 `D...` 输出：

```text
CAL source=last_move cmd=-1.000 progress=0.952 dl=-1.048 dr=-0.857
CAL diag left_err=-6.4% right_err=-23.5% asym=20.1%
CAL suggest reverse: left=1.069 right=1.308 from actual=-1.120 dl=-1.048 dr=-0.857
Use: K1.000,1.069,1.000,1.308
```

### 第二轮判读

- 前进方向：左轮误差 `+1.0%`，基本可接受；右轮误差 `+28.8%`，右轮前进里程明显记大
- 倒车方向：左轮误差 `-6.4%`，右轮误差 `-23.5%`，两侧倒车里程都记小，右轮倒车偏差更大
- 左右不对称：前进 `asym=24.2%`，倒车 `asym=20.1%`，左右差异仍是主问题
- 合并建议命令：`K0.990,1.069,0.776,1.308`
- 下一步：应用合并建议命令后，重新执行前进和倒车单段测试，确认误差是否压到 `5%` 附近

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

- 状态：部分完成，等待应用新系数后的复测
- 主要原因：已经获得新版 `LMOVE / CAL diag` 单段数据，但尚未验证 `K0.990,1.069,0.776,1.308` 应用后的效果

## 偏差说明

- 预期：前进和倒车单段轮程应接近实体距离，左右轮误差最好控制在 `5%` 内
- 实际：前进右轮偏大 `28.8%`；倒车左轮偏小 `6.4%`、右轮偏小 `23.5%`

## 后续处理

- 需要修复的问题：回写左右轮正反向比例系数，并复测误差是否收敛
- 是否需要复测：是
- 根据判读矩阵推测的主要原因：左右轮正反向比例不对称，当前不是 `MOVE progress` 主导
- 建议的下一条命令或系数调整：`K0.990,1.069,0.776,1.308`
- 相关文件：
  - `Core/Src/encoder.c`
  - `Core/Src/encoder.h`
  - `Core/Src/hc04.c`
  - `Core/Src/pid.c`
