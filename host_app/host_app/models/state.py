from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque

from host_app.protocol.pid_tuning import ControlDebugSample, PidLoopId, PidTuning


@dataclass(slots=True)
class RobotPose:
    x_m: float = 0.0
    y_m: float = 0.0
    theta_deg: float = 0.0
    timestamp_ms: int = 0


@dataclass(slots=True)
class RuntimeStats:
    control_cycles: int = 0
    cmd_rx: int = 0
    cmd_drop: int = 0
    dma_drop: int = 0
    scan_count: int = 0
    heap_free: int = 0
    heap_min: int = 0
    localization_inliers: int = 0
    localization_fitness_m: float = 0.0
    control_mode: int = 0
    move_state: int = 0
    localization_mode: int = 0
    navigation_state: int = 0
    base_speed_mps: float = 0.0
    map_update_active: bool = True
    map_last_skip_reason: int = 0
    mapping_skipped_turning_count: int = 0
    mapping_skipped_settle_count: int = 0
    mapping_skipped_quality_count: int = 0
    lidar_active: bool = False
    lidar_binary_enabled: bool = False
    telemetry_enabled: bool = False
    scan_stream_enabled: bool = False


@dataclass(slots=True)
class MapMeta:
    width_cells: int = 0
    height_cells: int = 0
    resolution_m_per_cell: float = 0.05
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0


@dataclass(slots=True)
class MapSnapshot:
    meta: MapMeta = field(default_factory=MapMeta)
    cells: list[int] = field(default_factory=list)
    revision: int = 0

    def ensure_size(self) -> None:
        expected = self.meta.width_cells * self.meta.height_cells
        if expected <= 0:
            self.cells = []
        elif len(self.cells) != expected:
            self.cells = [0] * expected


@dataclass(slots=True)
class NavigationState:
    state: int = 0
    goal_cell: tuple[int, int] = (-1, -1)
    current_waypoint_index: int = 0
    current_waypoint_cell: tuple[int, int] = (-1, -1)
    path_cells: list[tuple[int, int]] = field(default_factory=list)
    last_waypoint_distance_m: float = 0.0


@dataclass(slots=True)
class LidarScan:
    seq: int = 0
    pose: RobotPose = field(default_factory=RobotPose)
    corrected_pose: RobotPose = field(default_factory=RobotPose)
    points: list[tuple[float, float]] = field(default_factory=list)


@dataclass(slots=True)
class CommandAck:
    ok: bool
    command: str
    detail: str


@dataclass(slots=True)
class ConnectionInfo:
    connected: bool = False
    port: str = ""
    baud_rate: int = 921600
    protocol_mode: str = "structured"
    recording: bool = False


@dataclass(slots=True)
class SessionState:
    connection: ConnectionInfo = field(default_factory=ConnectionInfo)
    odom_pose: RobotPose = field(default_factory=RobotPose)
    estimated_pose: RobotPose = field(default_factory=RobotPose)
    control_pose: RobotPose = field(default_factory=RobotPose)
    runtime: RuntimeStats = field(default_factory=RuntimeStats)
    map_snapshot: MapSnapshot = field(default_factory=MapSnapshot)
    navigation: NavigationState = field(default_factory=NavigationState)
    latest_scan: LidarScan = field(default_factory=LidarScan)
    pid_tunings: dict[PidLoopId, PidTuning] = field(default_factory=dict)
    control_debug_samples: Deque[ControlDebugSample] = field(default_factory=lambda: deque(maxlen=500))
    acks: Deque[CommandAck] = field(default_factory=lambda: deque(maxlen=50))
    logs: Deque[str] = field(default_factory=lambda: deque(maxlen=500))
