# 2026-05-14 SLAM Turn Recovery Hardware Summary

## Setup

- Port: `COM15`
- Baud: `961200`
- Tool: `python -m host_app.tools.pid_tuner_cli`
- Log: `20260514_slam_turn_recovery_cli.jsonl`
- Test: static mapping, `A90`, wait, diagnostics, `A-90`, wait, diagnostics, then reverse order.

## Confirmed

- The host CLI can keep debugging through the turn tests.
- RTOS counters remained healthy during the run:
  - `overrun=0`
  - `cmd drop=0`
  - `dma_drop=0`
  - `tx busy=0`
  - `err=0`
  - `wait=0`
- Heap remained stable at `16200` bytes free.
- After `A90` and `A-90`, mapping gate prevented bad scans from being written:
  - `SLAM gate allowed=0 reason=paused(recovery)`
  - `SLAM map ... written=0`

## Key Observation

When the robot stays at a new `+90 deg` or `-90 deg` heading, ICP/recovery does not reliably reacquire the existing map frame:

- `A90` sample:
  - `pose pred=(0.002,0.002,90.44)`
  - `reason=paused(recovery)`
  - recovery skip increased while `written=0`
- `A-90` sample:
  - `pose pred=(0.005,0.007,-90.42)`
  - `reason=paused(recovery)`
  - recovery skip increased while `written=0`

When the robot turns back near the original heading, recovery exits and mapping resumes:

- after return to approximately `0 deg`:
  - `SLAM gate allowed=1 reason=active recovery=0`
  - `MAP gate=active`

## Interpretation

The first fix is working as a map-protection gate: it blocks turn-corrupted scan writes.

The remaining problem is relocalization after large in-place turns. The most likely next checks are:

1. LiDAR angle sign / coordinate convention.
2. Weak local reference strategy: matching only against the last accepted reference scan is not enough after a 90 degree view change.
3. ICP capture range and correspondence quality after large heading changes.

## Next Firmware Test

The next build adds runtime angle convention switching:

- `SG`: show current convention.
- `SG+`: use `pose + scan_angle`.
- `SG-`: use `pose - scan_angle`.

Run the same `A90/A-90` recovery test under both signs. If one sign exits recovery at `+/-90 deg` and produces a coherent map, the angle convention was the root cause. If both signs fail, the next fix should target global/local reference strategy rather than gate timing.
