from __future__ import annotations

import struct
import sys
from pathlib import Path

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
            struct.pack("<H", 15),
            struct.pack("<B", 1),
            struct.pack("<B", 1),
            struct.pack("<B", 1),
            struct.pack("<B", 0),
            struct.pack("<B", 1),
            struct.pack("<h", 1),
            struct.pack("<h", 2),
            struct.pack("<f", 0.0125),
            struct.pack("<f", 1.5),
            struct.pack("<f", 0.02),
            struct.pack("<f", 0.25),
            struct.pack("<f", -0.10),
            struct.pack("<f", 90.0),
            struct.pack("<I", 3),
            struct.pack("<I", 4),
            struct.pack("<I", 5),
            struct.pack("<I", 6),
            cells,
        ]
    )
    header = struct.pack("<2sBBHH", FRAME_MAGIC, 1, 1, 9, len(payload))
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
    assert int(result.cells[0, 1]) == 12
    assert int(result.cells[1, 2]) == -20
    assert int(result.cells[2, 3]) == -16


if __name__ == "__main__":
    test_parser_accepts_firmware_compatible_map_frame_and_ignores_noise()
    print("protocol_smoke_ok")
