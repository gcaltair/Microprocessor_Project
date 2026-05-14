# SLAM Angle Sign A/B Test Plan

## Purpose

The current gate fix prevents bad scans from being written after in-place turns, but hardware testing shows recovery does not reliably exit while the robot remains at `+90 deg` or `-90 deg`.

This test checks whether the LiDAR scan angle convention is the root cause.

## New Commands

```text
SG
SG+
SG-
```

- `SG`: show current angle convention.
- `SG+`: use `pose + scan_angle`.
- `SG-`: use `pose - scan_angle`.
- `SG+` and `SG-` reset localization, grid, and navigation state.

## Test Sequence

Run the same sequence once with `SG+`, then once with `SG-`.

```text
S
N
SG+
R0
M
```

Wait 3 to 5 seconds.

```text
SL
G
P
X4
A90
```

Wait until the turn is complete, then wait another 5 seconds.

```text
SL
G
P
X4
A-90
```

Wait until the turn is complete, then wait another 5 seconds.

```text
SL
G
P
X4
S
```

Repeat the full sequence with `SG-`.

## Decision Rule

- If one sign exits `paused(recovery)` at `+/-90 deg` and the map remains coherent, use that sign as the firmware default.
- If both signs remain stuck in `paused(recovery)` at `+/-90 deg`, the next fix should target reference strategy or ICP capture range, not gate timing.
- If either sign writes a rotated duplicate map, stop using that sign and keep recovery gate enabled.

## 2026-05-14 Result

`SG-` won the A/B test.

- `SG+` entered `paused(recovery)` after `A90`; sampled post-turn inliers dropped to `0`.
- `SG-` stayed `active` after `A90` and after `A-90`; ICP remained mostly accepted.

Firmware default is now `pose - scan_angle`.
