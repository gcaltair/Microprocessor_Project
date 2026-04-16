from __future__ import annotations

from pathlib import Path
from typing import Optional

import serial
from serial.tools import list_ports


class SerialTransport:
    def __init__(self) -> None:
        self._serial: Optional[serial.Serial] = None
        self._record_handle = None
        self._port = ""
        self._baud_rate = 115200

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def port(self) -> str:
        return self._port

    @property
    def baud_rate(self) -> int:
        return self._baud_rate

    @staticmethod
    def available_ports() -> list[str]:
        return [port.device for port in list_ports.comports()]

    def open(self, port: str, baud_rate: int, timeout: float = 0.02) -> None:
        self.close()
        self._serial = serial.Serial(port=port, baudrate=baud_rate, timeout=timeout, write_timeout=timeout)
        self._port = port
        self._baud_rate = baud_rate

    def close(self) -> None:
        if self._record_handle is not None:
            self._record_handle.close()
            self._record_handle = None
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def enable_recording(self, file_path: str | Path) -> None:
        path = Path(file_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._record_handle = path.open("ab")

    def disable_recording(self) -> None:
        if self._record_handle is not None:
            self._record_handle.close()
            self._record_handle = None

    def write_line(self, command: str) -> None:
        if not self.is_open:
            raise RuntimeError("serial port is not connected")
        self._serial.write(command.encode("utf-8") + b"\r\n")

    def poll(self) -> bytes:
        if not self.is_open:
            return b""
        waiting = self._serial.in_waiting
        if waiting <= 0:
            return b""
        data = self._serial.read(waiting)
        if self._record_handle is not None and data:
            self._record_handle.write(data)
            self._record_handle.flush()
        return data
