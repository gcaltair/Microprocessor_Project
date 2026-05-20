from __future__ import annotations

import argparse
import sys

import numpy as np
import serial
from serial.tools import list_ports
from PySide6 import QtCore, QtGui, QtWidgets

from telemetry_protocol import (
    FREE_THRESHOLD,
    OCCUPIED_THRESHOLD,
    MapFrame,
    TelemetryParser,
)
from navigation_protocol import build_clear_goal_command, build_debug_command_line, build_plan_command, build_set_goal_command


SKIP_REASON_LABELS = {
    0: "none",
    1: "turning",
    2: "settle",
    3: "quality",
}

LOCALIZATION_MODE_LABELS = {
    0: "odometry",
}

NAVIGATION_STATUS_LABELS = {
    0: "idle",
    1: "ok",
    2: "reached",
    3: "failed",
    4: "busy",
}

CONTROL_MODE_LABELS = {
    0: "manual",
    1: "position",
    2: "speed_test",
}

RELATIVE_MOVE_STATE_LABELS = {
    0: "idle",
    1: "turning",
    2: "driving",
}

MOTOR_DIRECTION_LABELS = {
    0: "forward",
    1: "backward",
    2: "stop",
}


class MapView(QtWidgets.QLabel):
    goalPicked = QtCore.Signal(float, float)

    def __init__(self) -> None:
        super().__init__()
        self._pixmap: QtGui.QPixmap | None = None
        self._last_frame: MapFrame | None = None
        self._goal: tuple[float, float] | None = None
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(520, 520)
        self.setStyleSheet("background:#14181d; border:1px solid #2f3944;")

    def set_frame(self, frame: MapFrame) -> None:
        self._last_frame = frame
        self._pixmap = self._build_pixmap(frame)
        self._refresh_pixmap()

    def set_goal(self, goal: tuple[float, float] | None) -> None:
        self._goal = goal
        if self._last_frame is not None:
            self._pixmap = self._build_pixmap(self._last_frame)
            self._refresh_pixmap()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        self._refresh_pixmap()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if self._last_frame is None:
            return

        world = self._event_to_world(event.position())
        if world is not None:
            self.goalPicked.emit(world[0], world[1])

    def _refresh_pixmap(self) -> None:
        if self._pixmap is None:
            return
        scaled = self._pixmap.scaled(
            self.size(),
            QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.FastTransformation,
        )
        self.setPixmap(scaled)

    def _build_pixmap(self, frame: MapFrame) -> QtGui.QPixmap:
        cells = frame.cells
        height, width = cells.shape

        image_data = np.zeros((height, width, 3), dtype=np.uint8)
        image_data[:, :] = (120, 132, 146)

        occupied = cells >= OCCUPIED_THRESHOLD
        free = cells <= FREE_THRESHOLD
        weak_occ = (cells > 0) & ~occupied
        weak_free = (cells < 0) & ~free

        image_data[occupied] = (18, 23, 28)
        image_data[free] = (240, 244, 248)
        image_data[weak_occ] = (70, 84, 98)
        image_data[weak_free] = (182, 196, 210)

        image_data = np.ascontiguousarray(np.flipud(image_data))
        qimage = QtGui.QImage(
            image_data.data,
            width,
            height,
            width * 3,
            QtGui.QImage.Format.Format_RGB888,
        ).copy()

        for path_x_m, path_y_m in frame.nav_path_points:
            path_cell_x = int((path_x_m - frame.origin_x_m) / frame.resolution_m_per_cell)
            path_cell_y = int((path_y_m - frame.origin_y_m) / frame.resolution_m_per_cell)
            path_y_screen = height - 1 - path_cell_y
            if 0 <= path_cell_x < width and 0 <= path_y_screen < height:
                qimage.setPixelColor(path_cell_x, path_y_screen, QtGui.QColor("#2fbf71"))

        if self._goal is not None:
            goal_x_m, goal_y_m = self._goal
            goal_cell_x = int((goal_x_m - frame.origin_x_m) / frame.resolution_m_per_cell)
            goal_cell_y = int((goal_y_m - frame.origin_y_m) / frame.resolution_m_per_cell)
            goal_y_screen = height - 1 - goal_cell_y
            if 0 <= goal_cell_x < width and 0 <= goal_y_screen < height:
                qimage.setPixelColor(goal_cell_x, goal_y_screen, QtGui.QColor("#2f80ed"))

        if frame.robot_inside_grid:
            robot_x = frame.robot_cell_x
            robot_y = height - 1 - frame.robot_cell_y
            if 0 <= robot_x < width and 0 <= robot_y < height:
                qimage.setPixelColor(robot_x, robot_y, QtGui.QColor("#f25f5c"))

        return QtGui.QPixmap.fromImage(qimage)

    def _event_to_world(self, position: QtCore.QPointF) -> tuple[float, float] | None:
        frame = self._last_frame
        if frame is None:
            return None

        label_w = self.width()
        label_h = self.height()
        scale = min(label_w / frame.width, label_h / frame.height)
        image_w = frame.width * scale
        image_h = frame.height * scale
        left = (label_w - image_w) * 0.5
        top = (label_h - image_h) * 0.5
        image_x = (position.x() - left) / scale
        image_y = (position.y() - top) / scale

        if image_x < 0 or image_y < 0 or image_x >= frame.width or image_y >= frame.height:
            return None

        cell_x = int(image_x)
        cell_y = frame.height - 1 - int(image_y)
        world_x = frame.origin_x_m + (cell_x + 0.5) * frame.resolution_m_per_cell
        world_y = frame.origin_y_m + (cell_y + 0.5) * frame.resolution_m_per_cell
        return world_x, world_y


class MainWindow(QtWidgets.QWidget):
    def __init__(self, default_port: str | None, baudrate: int) -> None:
        super().__init__()
        self._baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._parser = TelemetryParser()
        self._last_frame: MapFrame | None = None

        self.setWindowTitle("Car Mapping Host App")
        self.resize(1180, 760)

        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        self.refresh_button = QtWidgets.QPushButton("Refresh")
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.status_banner = QtWidgets.QLabel("Disconnected")
        self.status_banner.setStyleSheet("color:#f4d35e; font-weight:600;")

        self.map_view = MapView()
        self.frame_info = QtWidgets.QLabel("No telemetry yet")
        self.frame_info.setWordWrap(True)
        self.frame_info.setStyleSheet("font-family:Consolas, monospace; color:#dce3ea;")

        self.goal_x_spin = QtWidgets.QDoubleSpinBox()
        self.goal_x_spin.setRange(-10.0, 10.0)
        self.goal_x_spin.setDecimals(3)
        self.goal_x_spin.setSingleStep(0.05)
        self.goal_x_spin.setSuffix(" m")
        self.goal_y_spin = QtWidgets.QDoubleSpinBox()
        self.goal_y_spin.setRange(-10.0, 10.0)
        self.goal_y_spin.setDecimals(3)
        self.goal_y_spin.setSingleStep(0.05)
        self.goal_y_spin.setSuffix(" m")
        self.send_goal_button = QtWidgets.QPushButton("Send Goal")
        self.plan_button = QtWidgets.QPushButton("Plan")
        self.clear_goal_button = QtWidgets.QPushButton("Clear Goal")
        self.nav_status = QtWidgets.QLabel("Navigation idle")
        self.nav_status.setWordWrap(True)
        self.nav_status.setStyleSheet("color:#9fb3c8;")
        self.nav_telemetry = QtWidgets.QLabel("Navigation telemetry: no frame")
        self.nav_telemetry.setWordWrap(True)
        self.nav_telemetry.setStyleSheet("font-family:Consolas, monospace; color:#dce3ea;")
        self.control_telemetry = QtWidgets.QLabel("Control telemetry: no frame")
        self.control_telemetry.setWordWrap(True)
        self.control_telemetry.setStyleSheet("font-family:Consolas, monospace; color:#dce3ea;")

        self.debug_command_edit = QtWidgets.QLineEdit("P1,0")
        self.debug_command_edit.setPlaceholderText("ASCII command, e.g. P1,0")
        self.send_debug_command_button = QtWidgets.QPushButton("Send Debug")
        self.preset_p10_button = QtWidgets.QPushButton("P1,0")
        self.debug_status = QtWidgets.QLabel("Debug command ready")
        self.debug_status.setWordWrap(True)
        self.debug_status.setStyleSheet("color:#9fb3c8;")

        self.stats_box = QtWidgets.QPlainTextEdit()
        self.stats_box.setReadOnly(True)
        self.stats_box.setStyleSheet(
            "background:#0e1216; color:#dce3ea; font-family:Consolas, monospace; border:1px solid #2f3944;"
        )

        self._build_layout()
        self._wire_events()
        self._refresh_ports(default_port)

        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.setInterval(25)
        self._poll_timer.timeout.connect(self._poll_serial)
        self._poll_timer.start()

        if default_port:
            self._toggle_connection()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._close_serial()
        super().closeEvent(event)

    def _build_layout(self) -> None:
        self.setStyleSheet("background:#101419; color:#f1f5f9;")

        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addWidget(QtWidgets.QLabel("Port"))
        top_bar.addWidget(self.port_combo, 1)
        top_bar.addWidget(QtWidgets.QLabel(f"Baud {self._baudrate}"))
        top_bar.addWidget(self.refresh_button)
        top_bar.addWidget(self.connect_button)
        top_bar.addWidget(self.status_banner)

        side_panel = QtWidgets.QVBoxLayout()
        side_panel.addWidget(QtWidgets.QLabel("Latest Frame"))
        side_panel.addWidget(self.frame_info)
        side_panel.addWidget(self._build_navigation_panel())
        side_panel.addWidget(self._build_debug_panel())
        side_panel.addWidget(QtWidgets.QLabel("Telemetry Details"))
        side_panel.addWidget(self.stats_box, 1)

        main_layout = QtWidgets.QHBoxLayout()
        main_layout.addWidget(self.map_view, 3)

        side_widget = QtWidgets.QWidget()
        side_widget.setLayout(side_panel)
        side_widget.setMinimumWidth(340)
        main_layout.addWidget(side_widget, 2)

        root = QtWidgets.QVBoxLayout(self)
        root.addLayout(top_bar)
        root.addLayout(main_layout, 1)

    def _wire_events(self) -> None:
        self.refresh_button.clicked.connect(lambda: self._refresh_ports(self.port_combo.currentText().strip()))
        self.connect_button.clicked.connect(self._toggle_connection)
        self.send_goal_button.clicked.connect(self._send_navigation_goal)
        self.plan_button.clicked.connect(self._send_plan_goal)
        self.clear_goal_button.clicked.connect(self._clear_navigation_goal)
        self.send_debug_command_button.clicked.connect(self._send_debug_command)
        self.preset_p10_button.clicked.connect(lambda: self._set_debug_command("P1,0"))
        self.debug_command_edit.returnPressed.connect(self._send_debug_command)
        self.map_view.goalPicked.connect(self._set_goal_from_map)

    def _build_navigation_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Navigation")
        group.setStyleSheet("QGroupBox{border:1px solid #2f3944; margin-top:8px; padding:8px;}")

        form = QtWidgets.QFormLayout()
        form.addRow("Goal X", self.goal_x_spin)
        form.addRow("Goal Y", self.goal_y_spin)

        buttons = QtWidgets.QHBoxLayout()
        buttons.addWidget(self.send_goal_button)
        buttons.addWidget(self.plan_button)
        buttons.addWidget(self.clear_goal_button)

        layout = QtWidgets.QVBoxLayout(group)
        layout.addLayout(form)
        layout.addLayout(buttons)
        layout.addWidget(self.nav_status)
        layout.addWidget(self.nav_telemetry)
        layout.addWidget(self.control_telemetry)
        layout.addWidget(QtWidgets.QLabel("Tip: click the map to fill goal coordinates."))
        return group

    def _build_debug_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Debug Command")
        group.setStyleSheet("QGroupBox{border:1px solid #2f3944; margin-top:8px; padding:8px;}")

        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.debug_command_edit, 1)
        row.addWidget(self.send_debug_command_button)

        presets = QtWidgets.QHBoxLayout()
        presets.addWidget(QtWidgets.QLabel("Preset"))
        presets.addWidget(self.preset_p10_button)
        presets.addStretch(1)

        layout = QtWidgets.QVBoxLayout(group)
        layout.addLayout(row)
        layout.addLayout(presets)
        layout.addWidget(self.debug_status)
        return group

    def _refresh_ports(self, preferred: str | None) -> None:
        ports = [port.device for port in list_ports.comports()]
        current_text = preferred or self.port_combo.currentText().strip()

        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current_text:
            if current_text not in ports:
                self.port_combo.addItem(current_text)
            self.port_combo.setCurrentText(current_text)
        self.port_combo.blockSignals(False)

    def _toggle_connection(self) -> None:
        if self._serial is not None and self._serial.is_open:
            self._close_serial()
            return

        port = self.port_combo.currentText().strip()
        if not port:
            self.status_banner.setText("Select a serial port")
            self.status_banner.setStyleSheet("color:#ff7b72; font-weight:600;")
            return

        try:
            self._serial = serial.Serial(port=port, baudrate=self._baudrate, timeout=0)
        except serial.SerialException as exc:
            self._serial = None
            self.status_banner.setText(f"Open failed: {exc}")
            self.status_banner.setStyleSheet("color:#ff7b72; font-weight:600;")
            return

        self.connect_button.setText("Disconnect")
        self.status_banner.setText(f"Connected to {port}")
        self.status_banner.setStyleSheet("color:#7bd389; font-weight:600;")
        self._parser = TelemetryParser()

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            finally:
                self._serial = None

        self.connect_button.setText("Connect")
        self.status_banner.setText("Disconnected")
        self.status_banner.setStyleSheet("color:#f4d35e; font-weight:600;")

    def _write_command(self, command: bytes) -> bool:
        if self._serial is None or not self._serial.is_open:
            self.nav_status.setText("Navigation command not sent: serial is disconnected")
            self.nav_status.setStyleSheet("color:#ff7b72;")
            return False

        try:
            self._serial.write(command)
        except serial.SerialException as exc:
            self.nav_status.setText(f"Navigation command failed: {exc}")
            self.nav_status.setStyleSheet("color:#ff7b72;")
            self._close_serial()
            return False

        return True

    def _send_navigation_goal(self) -> None:
        goal_x = self.goal_x_spin.value()
        goal_y = self.goal_y_spin.value()
        if self._write_command(build_set_goal_command(goal_x, goal_y)):
            self.map_view.set_goal((goal_x, goal_y))
            self.nav_status.setText(f"Goal sent: x={goal_x:.3f} m, y={goal_y:.3f} m")
            self.nav_status.setStyleSheet("color:#7bd389;")

    def _send_plan_goal(self) -> None:
        goal_x = self.goal_x_spin.value()
        goal_y = self.goal_y_spin.value()
        if self._write_command(build_plan_command(goal_x, goal_y)):
            self.map_view.set_goal((goal_x, goal_y))
            self.nav_status.setText(f"Plan requested: x={goal_x:.3f} m, y={goal_y:.3f} m")
            self.nav_status.setStyleSheet("color:#7bd389;")

    def _clear_navigation_goal(self) -> None:
        if self._write_command(build_clear_goal_command()):
            self.map_view.set_goal(None)
            self.nav_status.setText("Goal cleared")
            self.nav_status.setStyleSheet("color:#9fb3c8;")

    def _set_debug_command(self, command: str) -> None:
        self.debug_command_edit.setText(command)
        self.debug_command_edit.setFocus()

    def _send_debug_command(self) -> None:
        command_text = self.debug_command_edit.text()
        try:
            command = build_debug_command_line(command_text)
        except (UnicodeEncodeError, ValueError) as exc:
            self.debug_status.setText(f"Debug command invalid: {exc}")
            self.debug_status.setStyleSheet("color:#ff7b72;")
            return

        if self._write_command(command):
            sent_text = command.decode("ascii").rstrip("\n")
            self.debug_status.setText(f"Debug command sent: {sent_text}")
            self.debug_status.setStyleSheet("color:#7bd389;")

    def _set_goal_from_map(self, x_m: float, y_m: float) -> None:
        self.goal_x_spin.setValue(x_m)
        self.goal_y_spin.setValue(y_m)
        self.map_view.set_goal((x_m, y_m))
        self.nav_status.setText(f"Goal selected from map: x={x_m:.3f} m, y={y_m:.3f} m")
        self.nav_status.setStyleSheet("color:#9fb3c8;")

    def _poll_serial(self) -> None:
        if self._serial is None:
            return

        try:
            waiting = self._serial.in_waiting
            chunk = self._serial.read(waiting or 1)
        except serial.SerialException as exc:
            self.status_banner.setText(f"Serial error: {exc}")
            self.status_banner.setStyleSheet("color:#ff7b72; font-weight:600;")
            self._close_serial()
            return

        for frame in self._parser.feed(chunk):
            self._apply_frame(frame)

    def _apply_frame(self, frame: MapFrame) -> None:
        self._last_frame = frame
        self.map_view.set_frame(frame)

        skip_reason = SKIP_REASON_LABELS.get(frame.last_skip_reason, str(frame.last_skip_reason))
        loc_mode = LOCALIZATION_MODE_LABELS.get(frame.last_localization_mode, str(frame.last_localization_mode))
        nav_status = NAVIGATION_STATUS_LABELS.get(frame.nav_status, str(frame.nav_status))
        control_mode = CONTROL_MODE_LABELS.get(frame.control_mode, str(frame.control_mode))
        move_state = RELATIVE_MOVE_STATE_LABELS.get(frame.relative_move_state, str(frame.relative_move_state))
        left_direction = MOTOR_DIRECTION_LABELS.get(frame.left_motor_direction, str(frame.left_motor_direction))
        right_direction = MOTOR_DIRECTION_LABELS.get(frame.right_motor_direction, str(frame.right_motor_direction))

        self.nav_telemetry.setText(
            f"fw_status={nav_status} goal_valid={frame.nav_goal_valid} target_valid={frame.nav_target_valid}\n"
            f"goal=({frame.nav_goal_x_m:.3f}, {frame.nav_goal_y_m:.3f}) "
            f"target=({frame.nav_target_x_m:.3f}, {frame.nav_target_y_m:.3f})\n"
            f"distance={frame.nav_distance_to_goal_m:.3f} m "
            f"path raw/smooth={frame.nav_raw_path_len}/{frame.nav_smooth_path_len} "
            f"points={len(frame.nav_path_points)} fail={frame.nav_fail_count}"
        )
        self.control_telemetry.setText(
            f"mode={control_mode} move={move_state}\n"
            f"PWM L/R={frame.left_pwm:5d}/{frame.right_pwm:5d} "
            f"dir={left_direction}/{right_direction}\n"
            f"PID pos={frame.position_pid_output_mps:+.3f} m/s "
            f"angle={frame.angle_pid_output_mps:+.3f} m/s\n"
            f"PID speed L/R={frame.left_speed_pid_output:+6d}/{frame.right_speed_pid_output:+6d}"
        )

        self.frame_info.setText(
            f"frame={frame.sequence} update={frame.update_count} scan={frame.last_scan_sequence}\n"
            f"pose=({frame.pose_x_m:.3f}, {frame.pose_y_m:.3f}, {frame.pose_theta_deg:.2f} deg)\n"
            f"grid={frame.width}x{frame.height} res={frame.resolution_m_per_cell:.3f} m/cell"
        )

        self.stats_box.setPlainText(
            "\n".join(
                [
                    f"tick_ms                 : {frame.tick_ms}",
                    f"usable_points           : {frame.usable_points}",
                    f"endpoints_written       : {frame.endpoints_written}",
                    f"localization_mode       : {loc_mode}",
                    f"map_update_active       : {frame.map_update_active}",
                    f"skip_reason             : {skip_reason}",
                    f"robot_inside_grid       : {frame.robot_inside_grid}",
                    f"robot_cell              : ({frame.robot_cell_x}, {frame.robot_cell_y})",
                    f"odom_delta_theta_deg    : {frame.odom_delta_theta_deg:.3f}",
                    f"odom_delta_translation  : {frame.odom_delta_translation_m:.4f}",
                    f"skip_turning_count      : {frame.skipped_turning_count}",
                    f"skip_settle_count       : {frame.skipped_settle_count}",
                    f"skip_quality_count      : {frame.skipped_quality_count}",
                    f"nav_status              : {nav_status}",
                    f"nav_goal_valid          : {frame.nav_goal_valid}",
                    f"nav_target_valid        : {frame.nav_target_valid}",
                    f"nav_update_count        : {frame.nav_update_count}",
                    f"nav_fail_count          : {frame.nav_fail_count}",
                    f"nav_path_raw_smooth     : {frame.nav_raw_path_len}/{frame.nav_smooth_path_len}",
                    f"nav_path_points         : {len(frame.nav_path_points)}",
                    f"nav_distance_to_goal_m  : {frame.nav_distance_to_goal_m:.3f}",
                    f"nav_goal                : ({frame.nav_goal_x_m:.3f}, {frame.nav_goal_y_m:.3f})",
                    f"nav_target              : ({frame.nav_target_x_m:.3f}, {frame.nav_target_y_m:.3f})",
                    f"control_mode            : {control_mode}",
                    f"relative_move_state     : {move_state}",
                    f"pwm_left_right          : {frame.left_pwm} / {frame.right_pwm}",
                    f"motor_dir_left_right    : {left_direction} / {right_direction}",
                    f"pid_position_output     : {frame.position_pid_output_mps:+.4f} m/s",
                    f"pid_position_error      : {frame.position_error_m:.4f} m",
                    f"pid_angle_output        : {frame.angle_pid_output_mps:+.4f} m/s",
                    f"pid_angle_error         : {frame.angle_error_deg:+.3f} deg",
                    f"pid_speed_output_l_r    : {frame.left_speed_pid_output:+d} / {frame.right_speed_pid_output:+d}",
                    f"base_speed              : {frame.base_speed_mps:+.4f} m/s",
                    f"wheel_setpoint_l_r      : {frame.left_speed_setpoint_mps:+.4f} / {frame.right_speed_setpoint_mps:+.4f} m/s",
                    f"wheel_feedback_l_r      : {frame.left_speed_feedback_mps:+.4f} / {frame.right_speed_feedback_mps:+.4f} m/s",
                    "",
                    "Legend:",
                    "dark   = occupied",
                    "light  = free",
                    "gray   = unknown / weak evidence",
                    "red    = robot pose",
                    "blue   = navigation goal",
                    "green  = firmware path points",
                ]
            )
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Display live occupancy-grid telemetry from UART5.")
    parser.add_argument("--port", help="Serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(default_port=args.port, baudrate=args.baud)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
