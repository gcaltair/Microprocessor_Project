# 2026-05-14 SLAM Angle Sign A/B Summary

## Setup

- Port: `COM15`
- Baud: `961200`
- Tool: `python -m host_app.tools.pid_tuner_cli`
- Log: `20260514_slam_angle_sign_ab_cli.jsonl`

## SG+ Result

`SG+` uses `pose + scan_angle`.

Static mapping was acceptable, but after `A90` the robot stayed in recovery:

- `SLAM cfg angle=+`
- pose after turn: about `90.28 deg`
- `SLAM gate allowed=0 reason=paused(recovery) recovery=1`
- `MAP gate=paused(recovery)`
- ICP correspondences collapsed, with `inliers=0` in the sampled post-turn frames.
- Mapping did not write bad scans: `written=0`.

Turning back near the original heading restored active mapping:

- `SLAM gate allowed=1 reason=active recovery=0`

## SG- Result

`SG-` uses `pose - scan_angle`.

Static mapping was better and after `A90` the robot stayed active:

- `SLAM cfg angle=-`
- pose after turn: about `88.85 deg` to `92.05 deg`
- `SLAM gate allowed=1 reason=active recovery=0`
- `MAP gate=active`
- ICP remained accepted:
  - first sampled post-turn frame: `accept=389 reject=2`
  - later return-to-start sample: `accept=987 reject=3`
- Mapping continued to write scans without entering recovery.

After returning with `A-90`, mapping remained active:

- pose around `0 deg`
- `SLAM gate allowed=1 reason=active recovery=0`
- `MAP gate=active`

## Decision

The LiDAR scan angle sign was the primary cause of the turn-after-map-rotation behavior. The firmware default should be `pose - scan_angle`.

The recovery gate remains useful as a protection layer, but with the corrected sign it no longer triggers during normal `A90` and return tests.
