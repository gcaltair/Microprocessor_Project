# SLAM turn debug run 20260514_202957

- Port: `COM15`
- Baud: `921600`
- Raw log: `20260514_202957_slam_turn_debug_serial.log`

## Extracted Lines

### loc

- `SLAM loc init=1 updates=64 mode=1 accept=48 reject=13 odom=3 pts=58/58 inliers=55 fit_mm=46.7`
- `SLAM loc init=1 updates=165 mode=1 accept=119 reject=43 odom=3 pts=59/59 inliers=58 fit_mm=60.0`

### gate

- `SLAM gate allowed=1 reason=active turning=0 recovery=0 odom_dxy_mm=0.0 odom_dth=0.00`
- `SLAM gate allowed=1 reason=active turning=0 recovery=0 odom_dxy_mm=0.0 odom_dth=0.00`

### map

- `SLAM map updates=64 written=116 gate=active rec=0 skip=(0,0,0,0) pose=(-0.001,-0.004,1.29)`
- `SLAM map updates=165 written=117 gate=active rec=0 skip=(0,0,11,0) pose=(0.008,-0.001,90.75)`

### odom

- `ODOM x=0.000 y=0.000 th=0.00 ls=0.000 rs=0.000 base=0.000 ang_sp=0.00 mode=MANUAL move=IDLE`
- `ODOM x=-0.001 y=0.000 th=17.09 ls=-0.111 rs=0.081 base=0.000 ang_sp=90.00 mode=MANUAL move=IDLE`
- `ODOM x=0.001 y=-0.001 th=91.07 ls=0.000 rs=0.000 base=0.000 ang_sp=90.00 mode=MANUAL move=IDLE`
- `ODOM x=0.001 y=0.003 th=58.97 ls=0.130 rs=-0.123 base=0.000 ang_sp=1.07 mode=MANUAL move=IDLE`

### est

- `EST  x=0.000 y=0.000 th=0.00 CTRL=(0.000,0.000,0.00)`
- `EST  x=-0.001 y=0.000 th=17.09 CTRL=(-0.001,0.000,17.09)`
- `EST  x=0.001 y=-0.001 th=91.07 CTRL=(0.001,-0.001,91.07)`
- `EST  x=-0.035 y=-0.046 th=47.96 CTRL=(0.001,0.003,58.97)`

### map_gate

- `MAP gate=active dxy_mm=0.0 dth=0.00 skip turn=0 settle=0 quality=0 recovery=0`
- `MAP gate=active dxy_mm=0.0 dth=0.00 skip turn=0 settle=0 quality=11 recovery=0`
- `MAP gate=active dxy_mm=0.0 dth=0.00 skip turn=0 settle=0 quality=24 recovery=0`

