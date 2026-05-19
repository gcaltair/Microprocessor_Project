from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import List

import numpy as np

FRAME_MAGIC = b"\xC3\x3C"
PROTOCOL_VERSION = 1
FRAME_TYPE_MAP_GRID = 1
FREE_THRESHOLD = -10
OCCUPIED_THRESHOLD = 10

_HEADER_STRUCT = struct.Struct("<2sBBHH")


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
    localization_inliers: int
    grid_initialized: int
    robot_inside_grid: int
    map_update_active: int
    last_skip_reason: int
    last_localization_mode: int
    robot_cell_x: int
    robot_cell_y: int
    localization_fitness_m: float
    odom_delta_theta_deg: float
    odom_delta_translation_m: float
    pose_x_m: float
    pose_y_m: float
    pose_theta_deg: float
    skipped_turning_count: int
    skipped_settle_count: int
    skipped_quality_count: int
    cells: np.ndarray


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

            if version == PROTOCOL_VERSION and frame_type == FRAME_TYPE_MAP_GRID:
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
    localization_inliers = take("<H")
    grid_initialized = take("<B")
    robot_inside_grid = take("<B")
    map_update_active = take("<B")
    last_skip_reason = take("<B")
    last_localization_mode = take("<B")
    robot_cell_x = take("<h")
    robot_cell_y = take("<h")
    localization_fitness = take("<f")
    odom_delta_theta = take("<f")
    odom_delta_translation = take("<f")
    pose_x = take("<f")
    pose_y = take("<f")
    pose_theta = take("<f")
    skipped_turning_count = take("<I")
    skipped_settle_count = take("<I")
    skipped_quality_count = take("<I")

    cell_count = width * height
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
        localization_inliers=localization_inliers,
        grid_initialized=grid_initialized,
        robot_inside_grid=robot_inside_grid,
        map_update_active=map_update_active,
        last_skip_reason=last_skip_reason,
        last_localization_mode=last_localization_mode,
        robot_cell_x=robot_cell_x,
        robot_cell_y=robot_cell_y,
        localization_fitness_m=localization_fitness,
        odom_delta_theta_deg=odom_delta_theta,
        odom_delta_translation_m=odom_delta_translation,
        pose_x_m=pose_x,
        pose_y_m=pose_y,
        pose_theta_deg=pose_theta,
        skipped_turning_count=skipped_turning_count,
        skipped_settle_count=skipped_settle_count,
        skipped_quality_count=skipped_quality_count,
        cells=cells,
    )
