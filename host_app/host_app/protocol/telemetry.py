from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Union

MAGIC = b"\xC3\x3C"
VERSION = 1

FRAME_TYPE_ACK_V2 = 1
FRAME_TYPE_ERR_V2 = 2
FRAME_TYPE_STATUS_V2 = 16
FRAME_TYPE_MAP_META_V2 = 17
FRAME_TYPE_MAP_DATA_V2 = 18
FRAME_TYPE_PATH_V2 = 19
FRAME_TYPE_SCAN_V2 = 20


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass(slots=True)
class TelemetryFrame:
    frame_type: int
    sequence: int
    payload: bytes


@dataclass(slots=True)
class TextLine:
    text: str


ProtocolEvent = Union[TelemetryFrame, TextLine]


class TelemetryStreamParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    @staticmethod
    def _looks_like_text(text: str) -> bool:
        if not text:
            return False
        printable = sum(1 for char in text if char.isprintable() or char in "\t ")
        if printable < max(1, int(len(text) * 0.8)):
            return False
        has_signal = any(char.isalnum() or "\u4e00" <= char <= "\u9fff" for char in text)
        return has_signal

    def feed(self, data: bytes) -> list[ProtocolEvent]:
        self._buffer.extend(data)
        events: list[ProtocolEvent] = []

        while self._buffer:
            if self._buffer.startswith(MAGIC):
                if len(self._buffer) < 8:
                    break

                version = self._buffer[2]
                if version != VERSION:
                    del self._buffer[0]
                    continue

                payload_len = int.from_bytes(self._buffer[6:8], "little")
                frame_len = 8 + payload_len + 2
                if len(self._buffer) < frame_len:
                    break

                body = bytes(self._buffer[: frame_len - 2])
                expected_crc = int.from_bytes(self._buffer[frame_len - 2 : frame_len], "little")
                if crc16_ccitt(body) != expected_crc:
                    del self._buffer[0]
                    continue

                events.append(
                    TelemetryFrame(
                        frame_type=self._buffer[3],
                        sequence=int.from_bytes(self._buffer[4:6], "little"),
                        payload=bytes(self._buffer[8 : 8 + payload_len]),
                    )
                )
                del self._buffer[:frame_len]
                continue

            newline_index = self._buffer.find(b"\n")
            magic_index = self._buffer.find(MAGIC)
            if newline_index == -1 and magic_index == -1:
                break

            split_index = None
            include_newline = False
            if magic_index != -1 and (newline_index == -1 or magic_index < newline_index):
                split_index = magic_index
            elif newline_index != -1:
                split_index = newline_index
                include_newline = True

            if split_index is None:
                break

            raw = bytes(self._buffer[:split_index])
            del self._buffer[: split_index + (1 if include_newline else 0)]
            text = raw.decode("utf-8", errors="ignore").replace("\r", "").strip()
            if self._looks_like_text(text):
                events.append(TextLine(text=text))

        return events


def unpack_status_payload(payload: bytes) -> tuple:
    return struct.unpack("<I9f4BfHHhhf7IHf4B", payload)


def unpack_map_meta_payload(payload: bytes) -> tuple:
    return struct.unpack("<HHfff", payload)


def unpack_map_data_header(payload: bytes) -> tuple:
    return struct.unpack_from("<HHHH", payload, 0)


def unpack_path_header(payload: bytes) -> tuple:
    return struct.unpack_from("<BBHHhhhhf", payload, 0)


def unpack_scan_header(payload: bytes) -> tuple:
    return struct.unpack_from("<I3f3fH", payload, 0)
