from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import List

import numpy as np

FRAME_MAGIC = b"\xC3\x3C"
PROTOCOL_VERSION = 6
FRAME_TYPE_MAP_GRID = 1
FREE_THRESHOLD = -10
OCCUPIED_THRESHOLD = 10

_HEADER_STRUCT = struct.Struct("<2sBBHH")
_SUPPORTED_PROTOCOL_VERSIONS = {PROTOCOL_VERSION}


@dataclass(slots=True)
class MapFrame:
    sequence: int
    width: int
    height: int
    resolution_m_per_cell: float
    origin_x_m: float
    origin_y_m: float
    update_count: int
    last_scan_sequence: int
    tick_ms: int
    usable_points: int
    endpoints_written: int
    grid_initialized: int
    robot_inside_grid: int
    map_update_active: int
    last_skip_reason: int
    last_localization_mode: int
    robot_cell_x: int
    robot_cell_y: int
    odom_delta_theta_deg: float
    odom_delta_translation_m: float
    pose_x_m: float
    pose_y_m: float
    pose_theta_deg: float
    skipped_turning_count: int
    skipped_settle_count: int
    skipped_quality_count: int
    nav_goal_valid: int
    nav_target_valid: int
    nav_status: int
    nav_update_count: int
    nav_fail_count: int
    nav_raw_path_len: int
    nav_smooth_path_len: int
    nav_distance_to_goal_m: float
    nav_goal_x_m: float
    nav_goal_y_m: float
    nav_target_x_m: float
    nav_target_y_m: float
    nav_path_points: tuple[tuple[float, float], ...]
    cells: np.ndarray
    control_mode: int = 0
    relative_move_state: int = 0
    left_motor_direction: int = 2
    right_motor_direction: int = 2
    left_pwm: int = 0
    right_pwm: int = 0
    left_speed_pid_output: int = 0
    right_speed_pid_output: int = 0
    position_pid_output_mps: float = 0.0
    angle_pid_output_mps: float = 0.0
    angle_error_deg: float = 0.0
    position_error_m: float = 0.0
    base_speed_mps: float = 0.0
    left_speed_setpoint_mps: float = 0.0
    right_speed_setpoint_mps: float = 0.0
    left_speed_feedback_mps: float = 0.0
    right_speed_feedback_mps: float = 0.0


class TelemetryParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> List[MapFrame]:
        frames: List[MapFrame] = []
        if not chunk:
            return frames

        self._buffer.extend(chunk)

        while True:
            magic_index = self._buffer.find(FRAME_MAGIC)
            if magic_index < 0:
                if len(self._buffer) > 1:
                    del self._buffer[:-1]
                break

            if magic_index > 0:
                del self._buffer[:magic_index]

            if len(self._buffer) < _HEADER_STRUCT.size:
                break

            magic, version, frame_type, sequence, payload_len = _HEADER_STRUCT.unpack_from(self._buffer)
            total_len = _HEADER_STRUCT.size + payload_len + 2
            if len(self._buffer) < total_len:
                break

            frame_bytes = bytes(self._buffer[:total_len])
            crc_expected = struct.unpack_from("<H", frame_bytes, total_len - 2)[0]
            crc_actual = _crc16_ccitt(frame_bytes[:-2])
            if crc_expected != crc_actual or magic != FRAME_MAGIC:
                del self._buffer[0]
                continue

            if version in _SUPPORTED_PROTOCOL_VERSIONS and frame_type == FRAME_TYPE_MAP_GRID:
                frames.append(_parse_map_frame(sequence, frame_bytes[_HEADER_STRUCT.size:-2]))

            del self._buffer[:total_len]

        return frames


def _crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _parse_map_frame(sequence: int, payload: bytes) -> MapFrame:
    offset = 0

    def take(fmt: str):
        nonlocal offset
        value = struct.unpack_from(fmt, payload, offset)
        offset += struct.calcsize(fmt)
        return value[0] if len(value) == 1 else value

    width = take("<H")
    height = take("<H")
    resolution = take("<f")
    origin_x = take("<f")
    origin_y = take("<f")
    update_count = take("<I")
    last_scan_sequence = take("<I")
    tick_ms = take("<I")
    usable_points = take("<H")
    endpoints_written = take("<H")
    grid_initialized = take("<B")
    robot_inside_grid = take("<B")
    map_update_active = take("<B")
    last_skip_reason = take("<B")
    last_localization_mode = take("<B")
    robot_cell_x = take("<h")
    robot_cell_y = take("<h")
    odom_delta_theta = take("<f")
    odom_delta_translation = take("<f")
    pose_x = take("<f")
    pose_y = take("<f")
    pose_theta = take("<f")
    skipped_turning_count = take("<I")
    skipped_settle_count = take("<I")
    skipped_quality_count = take("<I")
    nav_goal_valid = take("<B")
    nav_target_valid = take("<B")
    nav_status = take("<B")
    _nav_reserved = take("<B")
    nav_update_count = take("<I")
    nav_fail_count = take("<I")
    nav_raw_path_len = take("<H")
    nav_smooth_path_len = take("<H")
    nav_distance_to_goal = take("<f")
    nav_goal_x = take("<f")
    nav_goal_y = take("<f")
    nav_target_x = take("<f")
    nav_target_y = take("<f")
    nav_path_point_count = take("<H")
    nav_path_points = tuple((take("<f"), take("<f")) for _ in range(nav_path_point_count))
    control_mode = 0
    relative_move_state = 0
    left_motor_direction = 2
    right_motor_direction = 2
    left_pwm = 0
    right_pwm = 0
    left_speed_pid_output = 0
    right_speed_pid_output = 0
    position_pid_output = 0.0
    angle_pid_output = 0.0
    angle_error = 0.0
    position_error = 0.0
    base_speed = 0.0
    left_speed_setpoint = 0.0
    right_speed_setpoint = 0.0
    left_speed_feedback = 0.0
    right_speed_feedback = 0.0
    cell_count = width * height
    if len(payload) - offset >= cell_count + 48:
        control_mode = take("<B")
        relative_move_state = take("<B")
        left_motor_direction = take("<B")
        right_motor_direction = take("<B")
        left_pwm = take("<H")
        right_pwm = take("<H")
        left_speed_pid_output = take("<h")
        right_speed_pid_output = take("<h")
        position_pid_output = take("<f")
        angle_pid_output = take("<f")
        angle_error = take("<f")
        position_error = take("<f")
        base_speed = take("<f")
        left_speed_setpoint = take("<f")
        right_speed_setpoint = take("<f")
        left_speed_feedback = take("<f")
        right_speed_feedback = take("<f")

    cells = np.frombuffer(payload, dtype=np.int8, count=cell_count, offset=offset).copy()
    cells = cells.reshape((height, width))

    return MapFrame(
        sequence=sequence,
        width=width,
        height=height,
        resolution_m_per_cell=resolution,
        origin_x_m=origin_x,
        origin_y_m=origin_y,
        update_count=update_count,
        last_scan_sequence=last_scan_sequence,
        tick_ms=tick_ms,
        usable_points=usable_points,
        endpoints_written=endpoints_written,
        grid_initialized=grid_initialized,
        robot_inside_grid=robot_inside_grid,
        map_update_active=map_update_active,
        last_skip_reason=last_skip_reason,
        last_localization_mode=last_localization_mode,
        robot_cell_x=robot_cell_x,
        robot_cell_y=robot_cell_y,
        odom_delta_theta_deg=odom_delta_theta,
        odom_delta_translation_m=odom_delta_translation,
        pose_x_m=pose_x,
        pose_y_m=pose_y,
        pose_theta_deg=pose_theta,
        skipped_turning_count=skipped_turning_count,
        skipped_settle_count=skipped_settle_count,
        skipped_quality_count=skipped_quality_count,
        nav_goal_valid=nav_goal_valid,
        nav_target_valid=nav_target_valid,
        nav_status=nav_status,
        nav_update_count=nav_update_count,
        nav_fail_count=nav_fail_count,
        nav_raw_path_len=nav_raw_path_len,
        nav_smooth_path_len=nav_smooth_path_len,
        nav_distance_to_goal_m=nav_distance_to_goal,
        nav_goal_x_m=nav_goal_x,
        nav_goal_y_m=nav_goal_y,
        nav_target_x_m=nav_target_x,
        nav_target_y_m=nav_target_y,
        nav_path_points=nav_path_points,
        cells=cells,
        control_mode=control_mode,
        relative_move_state=relative_move_state,
        left_motor_direction=left_motor_direction,
        right_motor_direction=right_motor_direction,
        left_pwm=left_pwm,
        right_pwm=right_pwm,
        left_speed_pid_output=left_speed_pid_output,
        right_speed_pid_output=right_speed_pid_output,
        position_pid_output_mps=position_pid_output,
        angle_pid_output_mps=angle_pid_output,
        angle_error_deg=angle_error,
        position_error_m=position_error,
        base_speed_mps=base_speed,
        left_speed_setpoint_mps=left_speed_setpoint,
        right_speed_setpoint_mps=right_speed_setpoint,
        left_speed_feedback_mps=left_speed_feedback,
        right_speed_feedback_mps=right_speed_feedback,
    )
