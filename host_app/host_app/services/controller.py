from __future__ import annotations

from pathlib import Path
import time

from PySide6.QtCore import QObject, QTimer, Signal

from host_app.protocol import DeviceCommandBuilder, TelemetryStreamParser
from host_app.services.store import SessionStore
from host_app.transport import SerialTransport


class HostSessionController(QObject):
    state_changed = Signal(object)
    ports_changed = Signal(list)
    connection_changed = Signal(bool, str)
    log_message = Signal(str)
    error = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.transport = SerialTransport()
        self.parser = TelemetryStreamParser()
        self.store = SessionStore()
        self._auto_reconnect = False
        self._last_reconnect_attempt = 0.0

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(20)
        self._poll_timer.timeout.connect(self._poll_serial)
        self._poll_timer.start()

        self._ports_timer = QTimer(self)
        self._ports_timer.setInterval(1500)
        self._ports_timer.timeout.connect(self.refresh_ports)
        self._ports_timer.start()
        self.refresh_ports()

    def refresh_ports(self) -> None:
        self.ports_changed.emit(self.transport.available_ports())

    def connect_port(self, port: str, baud_rate: int, protocol_mode: str) -> None:
        try:
            self.transport.open(port, baud_rate)
        except Exception as exc:
            self.error.emit(f"串口连接失败: {exc}")
            self.connection_changed.emit(False, "")
            return

        self.store.state.connection.connected = True
        self.store.state.connection.port = port
        self.store.state.connection.baud_rate = baud_rate
        self.store.state.connection.protocol_mode = protocol_mode
        self._auto_reconnect = True
        self.connection_changed.emit(True, port)
        self.log_message.emit(f"已连接 {port} @ {baud_rate}")
        self._send_handshake(protocol_mode)
        self.state_changed.emit(self.store.state)

    def disconnect_port(self) -> None:
        self._auto_reconnect = False
        self.transport.close()
        self.store.state.connection.connected = False
        self.connection_changed.emit(False, "")
        self.state_changed.emit(self.store.state)

    def set_recording(self, enabled: bool) -> None:
        if enabled:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            file_path = Path("logs") / f"session_{timestamp}.bin"
            self.transport.enable_recording(file_path)
            self.store.state.connection.recording = True
            self.log_message.emit(f"开始录包: {file_path}")
        else:
            self.transport.disable_recording()
            self.store.state.connection.recording = False
            self.log_message.emit("录包已停止")
        self.state_changed.emit(self.store.state)

    def send_command(self, command: str) -> None:
        try:
            self.transport.write_line(command)
        except Exception as exc:
            self.error.emit(f"发送失败: {exc}")
            return
        self.log_message.emit(f"> {command}")

    def send_manual(self, command: str) -> None:
        self.send_command(DeviceCommandBuilder.manual(command))

    def set_speed(self, value: float) -> None:
        self.send_command(DeviceCommandBuilder.set_speed(value))

    def send_goal_cell(self, cell_x: int, cell_y: int) -> None:
        self.send_command(DeviceCommandBuilder.goto_cell(cell_x, cell_y))

    def set_lidar_enabled(self, enabled: bool) -> None:
        self.send_command(DeviceCommandBuilder.lidar(enabled))

    def set_scan_stream(self, enabled: bool) -> None:
        self.send_command(DeviceCommandBuilder.scan_stream(enabled))

    def switch_baud(self, baud_rate: int) -> None:
        self.send_command(DeviceCommandBuilder.baud(baud_rate))

    def show_pid_tunings(self, loop: str | None = None) -> None:
        self.send_command(DeviceCommandBuilder.pid_show(loop))

    def set_pid_tuning(self, loop: str, kp: float, ki: float, kd: float) -> None:
        self.send_command(DeviceCommandBuilder.pid_set(loop, kp, ki, kd))

    def replay_log(self, file_path: str) -> None:
        data = Path(file_path).read_bytes()
        for event in self.parser.feed(data):
            self.store.apply_event(event)
        self.state_changed.emit(self.store.state)
        self.log_message.emit(f"已回放 {file_path}")

    def _send_handshake(self, protocol_mode: str) -> None:
        if protocol_mode == "structured":
            self.send_command(DeviceCommandBuilder.telemetry(True))
            self.send_command(DeviceCommandBuilder.scan_stream(False))
        else:
            self.send_command(DeviceCommandBuilder.telemetry(False))
        self.send_command("U0")
        self.send_command("O")
        self.send_command("P")
        self.send_command("G")

    def _poll_serial(self) -> None:
        if not self.transport.is_open:
            if self._auto_reconnect and self.store.state.connection.port:
                now = time.time()
                if now - self._last_reconnect_attempt >= 1.0:
                    self._last_reconnect_attempt = now
                    self.connect_port(
                        self.store.state.connection.port,
                        self.store.state.connection.baud_rate,
                        self.store.state.connection.protocol_mode,
                    )
            return

        try:
            data = self.transport.poll()
        except Exception as exc:
            self.error.emit(f"串口读取失败: {exc}")
            self.transport.close()
            self.store.state.connection.connected = False
            self.connection_changed.emit(False, "")
            self.state_changed.emit(self.store.state)
            return

        if not data:
            return

        for event in self.parser.feed(data):
            self.store.apply_event(event)
            if hasattr(event, "text"):
                self.log_message.emit(event.text)

        self.state_changed.emit(self.store.state)
