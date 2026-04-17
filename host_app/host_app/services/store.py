from __future__ import annotations

import struct

from host_app.models.state import (
    CommandAck,
    LidarScan,
    MapMeta,
    NavigationState,
    RobotPose,
    RuntimeStats,
    SessionState,
)
from host_app.protocol.telemetry import (
    FRAME_TYPE_ACK_V2,
    FRAME_TYPE_ERR_V2,
    FRAME_TYPE_MAP_DATA_V2,
    FRAME_TYPE_MAP_META_V2,
    FRAME_TYPE_PATH_V2,
    FRAME_TYPE_SCAN_V2,
    FRAME_TYPE_STATUS_V2,
    TelemetryFrame,
    TextLine,
    unpack_map_data_header,
    unpack_map_meta_payload,
    unpack_path_header,
    unpack_scan_header,
    unpack_status_payload,
)


class SessionStore:
    def __init__(self) -> None:
        self.state = SessionState()

    def append_log(self, message: str) -> None:
        self.state.logs.append(message)

    def apply_event(self, event: TelemetryFrame | TextLine) -> None:
        if isinstance(event, TextLine):
            self.append_log(event.text)
            return

        if event.frame_type == FRAME_TYPE_STATUS_V2:
            self._apply_status(event)
        elif event.frame_type == FRAME_TYPE_MAP_META_V2:
            self._apply_map_meta(event)
        elif event.frame_type == FRAME_TYPE_MAP_DATA_V2:
            self._apply_map_data(event)
        elif event.frame_type == FRAME_TYPE_PATH_V2:
            self._apply_path(event)
        elif event.frame_type == FRAME_TYPE_SCAN_V2:
            self._apply_scan(event)
        elif event.frame_type in {FRAME_TYPE_ACK_V2, FRAME_TYPE_ERR_V2}:
            self._apply_ack(event)

    def _apply_status(self, event: TelemetryFrame) -> None:
        values = unpack_status_payload(event.payload)
        (
            timestamp_ms,
            odom_x,
            odom_y,
            odom_th,
            est_x,
            est_y,
            est_th,
            ctrl_x,
            ctrl_y,
            ctrl_th,
            control_mode,
            move_state,
            localization_mode,
            navigation_state,
            base_speed_mps,
            _nav_path_length,
            nav_current_waypoint_index,
            nav_goal_x,
            nav_goal_y,
            nav_last_distance_m,
            control_cycles,
            cmd_rx_count,
            cmd_drop_count,
            lidar_scan_complete_count,
            lidar_dma_drop_count,
            free_heap_bytes,
            min_ever_free_heap_bytes,
            localization_inliers,
            localization_fitness_m,
            map_update_active,
            map_last_skip_reason,
            _reserved0,
            _reserved1,
            mapping_skipped_turning_count,
            mapping_skipped_settle_count,
            mapping_skipped_quality_count,
            lidar_active,
            lidar_binary_enabled,
            telemetry_enabled,
            scan_stream_enabled,
        ) = values

        self.state.odom_pose = RobotPose(odom_x, odom_y, odom_th, timestamp_ms)
        self.state.estimated_pose = RobotPose(est_x, est_y, est_th, timestamp_ms)
        self.state.control_pose = RobotPose(ctrl_x, ctrl_y, ctrl_th, timestamp_ms)
        self.state.runtime = RuntimeStats(
            control_cycles=control_cycles,
            cmd_rx=cmd_rx_count,
            cmd_drop=cmd_drop_count,
            dma_drop=lidar_dma_drop_count,
            scan_count=lidar_scan_complete_count,
            heap_free=free_heap_bytes,
            heap_min=min_ever_free_heap_bytes,
            localization_inliers=localization_inliers,
            localization_fitness_m=localization_fitness_m,
            control_mode=control_mode,
            move_state=move_state,
            localization_mode=localization_mode,
            navigation_state=navigation_state,
            base_speed_mps=base_speed_mps,
            map_update_active=bool(map_update_active),
            map_last_skip_reason=map_last_skip_reason,
            mapping_skipped_turning_count=mapping_skipped_turning_count,
            mapping_skipped_settle_count=mapping_skipped_settle_count,
            mapping_skipped_quality_count=mapping_skipped_quality_count,
            lidar_active=bool(lidar_active),
            lidar_binary_enabled=bool(lidar_binary_enabled),
            telemetry_enabled=bool(telemetry_enabled),
            scan_stream_enabled=bool(scan_stream_enabled),
        )
        self.state.navigation.state = navigation_state
        self.state.navigation.goal_cell = (nav_goal_x, nav_goal_y)
        self.state.navigation.current_waypoint_index = nav_current_waypoint_index
        self.state.navigation.last_waypoint_distance_m = nav_last_distance_m

    def _apply_map_meta(self, event: TelemetryFrame) -> None:
        width, height, resolution, origin_x, origin_y = unpack_map_meta_payload(event.payload)
        self.state.map_snapshot.meta = MapMeta(width, height, resolution, origin_x, origin_y)
        self.state.map_snapshot.ensure_size()

    def _apply_map_data(self, event: TelemetryFrame) -> None:
        width, height, row_offset, row_count = unpack_map_data_header(event.payload)
        self.state.map_snapshot.meta.width_cells = width
        self.state.map_snapshot.meta.height_cells = height
        self.state.map_snapshot.ensure_size()
        cell_count = width * row_count
        rows = struct.unpack_from(f"<{cell_count}b", event.payload, struct.calcsize("<HHHH"))
        start = row_offset * width
        self.state.map_snapshot.cells[start : start + cell_count] = list(rows)
        self.state.map_snapshot.revision += 1

    def _apply_path(self, event: TelemetryFrame) -> None:
        header = unpack_path_header(event.payload)
        nav_state, _reserved, current_waypoint_index, path_length, goal_x, goal_y, current_x, current_y, distance = header
        offset = struct.calcsize("<BBHHhhhhf")
        path_cells = []
        for _ in range(path_length):
            path_cells.append(struct.unpack_from("<hh", event.payload, offset))
            offset += struct.calcsize("<hh")

        self.state.navigation = NavigationState(
            state=nav_state,
            goal_cell=(goal_x, goal_y),
            current_waypoint_index=current_waypoint_index,
            current_waypoint_cell=(current_x, current_y),
            path_cells=path_cells,
            last_waypoint_distance_m=distance,
        )

    def _apply_scan(self, event: TelemetryFrame) -> None:
        sequence, pose_x, pose_y, pose_th, corr_x, corr_y, corr_th, point_count = unpack_scan_header(event.payload)
        offset = struct.calcsize("<I3f3fH")
        points = []
        for _ in range(point_count):
            angle_x100, distance_mm = struct.unpack_from("<HH", event.payload, offset)
            points.append((angle_x100 / 100.0, float(distance_mm)))
            offset += struct.calcsize("<HH")

        self.state.latest_scan = LidarScan(
            seq=sequence,
            pose=RobotPose(pose_x, pose_y, pose_th, self.state.odom_pose.timestamp_ms),
            corrected_pose=RobotPose(corr_x, corr_y, corr_th, self.state.odom_pose.timestamp_ms),
            points=points,
        )

    def _apply_ack(self, event: TelemetryFrame) -> None:
        ok = event.frame_type == FRAME_TYPE_ACK_V2
        command_len = event.payload[1] if len(event.payload) >= 2 else 0
        detail_len = int.from_bytes(event.payload[2:4], "little") if len(event.payload) >= 4 else 0
        command = event.payload[4 : 4 + command_len].decode("utf-8", errors="ignore")
        detail_start = 4 + command_len
        detail = event.payload[detail_start : detail_start + detail_len].decode("utf-8", errors="ignore")
        self.state.acks.append(CommandAck(ok=ok, command=command, detail=detail))
        self.append_log(f"{'ACK' if ok else 'ERR'} {command} {detail}".strip())
