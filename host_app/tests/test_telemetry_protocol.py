from __future__ import annotations

import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from telemetry_protocol import FRAME_MAGIC, TelemetryParser


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


def test_parser_accepts_firmware_compatible_map_frame_and_ignores_noise() -> None:
    width = 4
    height = 3
    cells = bytes([0, 12, 0xF4, 0, 20, 0, 0xEC, 0, 0, 0, 0, 0xF0])

    payload = b"".join(
        [
            struct.pack("<H", width),
            struct.pack("<H", height),
            struct.pack("<f", 0.05),
            struct.pack("<f", -2.4),
            struct.pack("<f", -2.4),
            struct.pack("<I", 11),
            struct.pack("<I", 77),
            struct.pack("<I", 1234),
            struct.pack("<H", 88),
            struct.pack("<H", 22),
            struct.pack("<B", 1),
            struct.pack("<B", 1),
            struct.pack("<B", 1),
            struct.pack("<B", 0),
            struct.pack("<B", 1),
            struct.pack("<h", 1),
            struct.pack("<h", 2),
            struct.pack("<f", 1.5),
            struct.pack("<f", 0.02),
            struct.pack("<f", 0.25),
            struct.pack("<f", -0.10),
            struct.pack("<f", 90.0),
            struct.pack("<I", 3),
            struct.pack("<I", 4),
            struct.pack("<I", 5),
            struct.pack("<B", 1),
            struct.pack("<B", 1),
            struct.pack("<B", 4),
            struct.pack("<B", 0),
            struct.pack("<I", 6),
            struct.pack("<I", 7),
            struct.pack("<H", 8),
            struct.pack("<H", 9),
            struct.pack("<f", 0.35),
            struct.pack("<f", 0.5),
            struct.pack("<f", -0.25),
            struct.pack("<f", 0.4),
            struct.pack("<f", -0.1),
            struct.pack("<H", 0),
            cells,
        ]
    )
    header = struct.pack("<2sBBHH", FRAME_MAGIC, 6, 1, 9, len(payload))
    frame_without_crc = header + payload
    frame = frame_without_crc + struct.pack("<H", _crc16_ccitt(frame_without_crc))

    parser = TelemetryParser()
    parsed = parser.feed(b"debug-noise" + frame[:19] + frame[19:41] + frame[41:] + b"text-tail")

    assert len(parsed) == 1
    result = parsed[0]
    assert result.width == 4
    assert result.height == 3
    assert result.update_count == 11
    assert result.last_scan_sequence == 77
    assert result.robot_cell_x == 1
    assert result.robot_cell_y == 2
    assert result.nav_status == 4
    assert result.nav_raw_path_len == 8
    assert result.nav_smooth_path_len == 9
    assert result.nav_path_points == ()
    assert int(result.cells[0, 1]) == 12
    assert int(result.cells[1, 2]) == -20
    assert int(result.cells[2, 3]) == -16


def test_parser_reads_control_diagnostics_from_v6_map_frame() -> None:
    width = 2
    height = 2
    cells = bytes([0, 10, 0xF6, 20])

    payload = b"".join(
        [
            struct.pack("<H", width),
            struct.pack("<H", height),
            struct.pack("<f", 0.05),
            struct.pack("<f", -0.5),
            struct.pack("<f", -0.5),
            struct.pack("<I", 22),
            struct.pack("<I", 88),
            struct.pack("<I", 2345),
            struct.pack("<H", 90),
            struct.pack("<H", 24),
            struct.pack("<B", 1),
            struct.pack("<B", 1),
            struct.pack("<B", 1),
            struct.pack("<B", 0),
            struct.pack("<B", 1),
            struct.pack("<h", 3),
            struct.pack("<h", 4),
            struct.pack("<f", 0.1),
            struct.pack("<f", 0.02),
            struct.pack("<f", 0.30),
            struct.pack("<f", -0.20),
            struct.pack("<f", 10.0),
            struct.pack("<I", 0),
            struct.pack("<I", 0),
            struct.pack("<I", 0),
            struct.pack("<B", 0),
            struct.pack("<B", 0),
            struct.pack("<B", 0),
            struct.pack("<B", 0),
            struct.pack("<I", 0),
            struct.pack("<I", 0),
            struct.pack("<H", 0),
            struct.pack("<H", 0),
            struct.pack("<f", 0.0),
            struct.pack("<f", 0.0),
            struct.pack("<f", 0.0),
            struct.pack("<f", 0.0),
            struct.pack("<f", 0.0),
            struct.pack("<H", 2),
            struct.pack("<f", 0.10),
            struct.pack("<f", 0.20),
            struct.pack("<f", 0.30),
            struct.pack("<f", 0.40),
            struct.pack("<B", 1),
            struct.pack("<B", 2),
            struct.pack("<B", 0),
            struct.pack("<B", 1),
            struct.pack("<H", 1200),
            struct.pack("<H", 1300),
            struct.pack("<h", 514),
            struct.pack("<h", -615),
            struct.pack("<f", 0.12),
            struct.pack("<f", -0.03),
            struct.pack("<f", 2.5),
            struct.pack("<f", 0.18),
            struct.pack("<f", 0.10),
            struct.pack("<f", 0.07),
            struct.pack("<f", 0.13),
            struct.pack("<f", 0.06),
            struct.pack("<f", 0.11),
            cells,
        ]
    )
    header = struct.pack("<2sBBHH", FRAME_MAGIC, 6, 1, 11, len(payload))
    frame_without_crc = header + payload
    frame = frame_without_crc + struct.pack("<H", _crc16_ccitt(frame_without_crc))

    parsed = TelemetryParser().feed(frame)

    assert len(parsed) == 1
    result = parsed[0]
    assert result.control_mode == 1
    assert result.relative_move_state == 2
    assert result.left_motor_direction == 0
    assert result.right_motor_direction == 1
    assert result.left_pwm == 1200
    assert result.right_pwm == 1300
    assert result.left_speed_pid_output == 514
    assert result.right_speed_pid_output == -615
    assert result.position_pid_output_mps == pytest.approx(0.12)
    assert result.angle_pid_output_mps == pytest.approx(-0.03)
    assert result.angle_error_deg == pytest.approx(2.5)
    assert result.position_error_m == pytest.approx(0.18)
    assert result.base_speed_mps == pytest.approx(0.10)
    assert result.left_speed_setpoint_mps == pytest.approx(0.07)
    assert result.right_speed_setpoint_mps == pytest.approx(0.13)
    assert result.left_speed_feedback_mps == pytest.approx(0.06)
    assert result.right_speed_feedback_mps == pytest.approx(0.11)
    assert result.nav_path_points[0] == pytest.approx((0.10, 0.20))
    assert result.nav_path_points[1] == pytest.approx((0.30, 0.40))
    assert int(result.cells[1, 0]) == -10


if __name__ == "__main__":
    test_parser_accepts_firmware_compatible_map_frame_and_ignores_noise()
    test_parser_reads_control_diagnostics_from_v6_map_frame()
    print("protocol_smoke_ok")
