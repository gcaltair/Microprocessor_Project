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
    MAZE_NODE_COUNT,
    MAZE_DIR_COUNT,
    MapFrame,
    TelemetryParser,
)
from navigation_protocol import (
    build_clear_goal_command,
    build_debug_command_line,
    build_set_start_cell_command,
)

# ── Label maps ──────────────────────────────────────────────────────────────

SKIP_REASON_LABELS = {0: "none", 1: "turning", 2: "settle", 3: "quality"}
LOCALIZATION_MODE_LABELS = {0: "odometry"}
NAVIGATION_STATUS_LABELS = {0: "idle", 1: "ok", 2: "reached", 3: "failed", 4: "busy"}
CONTROL_MODE_LABELS = {0: "manual", 1: "position", 2: "speed_test"}
RELATIVE_MOVE_STATE_LABELS = {0: "idle", 1: "turning", 2: "driving"}
MOTOR_DIRECTION_LABELS = {0: "forward", 1: "backward", 2: "stop"}
MAZE_EXPLORE_STATE_LABELS = {
    0: "IDLE", 1: "SCAN", 2: "CHOOSE", 3: "MOVING",
    4: "BACKTRACK", 5: "RETURN", 6: "DONE",
}
EDGE_STATE_LABELS = {0: "unknown", 1: "open", 2: "wall"}

# ── Color palette ───────────────────────────────────────────────────────────

CLR_BG = "#0e1216"
CLR_PANEL = "#14181d"
CLR_BORDER = "#2f3944"
CLR_TEXT = "#dce3ea"
CLR_TEXT_DIM = "#9fb3c8"
CLR_ACCENT = "#58a6ff"
CLR_SUCCESS = "#7bd389"
CLR_ERROR = "#ff7b72"
CLR_WARN = "#f4d35e"

# Maze colors
CLR_CELL_UNVISITED = "#1c2128"
CLR_CELL_VISITED = "#1f3a5f"
CLR_CELL_CURRENT = "#58a6ff"
CLR_CELL_START = "#f0883e"
CLR_EDGE_UNKNOWN = "#2f3944"
CLR_EDGE_OPEN = "#7bd389"
CLR_EDGE_WALL = "#ff7b72"
CLR_NODE_DOT = "#8b949e"


# ── Maze Graph View ────────────────────────────────────────────────────────

class MazeGraphView(QtWidgets.QWidget):
    """Interactive 5×5 maze graph visualization with walls and robot position."""

    cellClicked = QtCore.Signal(int, int)  # col, row

    CELL_PX = 80
    WALL_PX = 6
    PAD = 24

    def __init__(self) -> None:
        super().__init__()
        self._frame: MapFrame | None = None
        total = self.PAD * 2 + self.CELL_PX * 5
        self.setMinimumSize(total, total)
        self.setMaximumSize(total + 40, total + 40)

    def set_frame(self, frame: MapFrame) -> None:
        self._frame = frame
        self.update()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        col, row = self._pos_to_cell(event.position())
        if 0 <= col < 5 and 0 <= row < 5:
            self.cellClicked.emit(col, row)

    def _pos_to_cell(self, pos: QtCore.QPointF) -> tuple[int, int]:
        x = pos.x() - self.PAD
        y = pos.y() - self.PAD
        col = int(x / self.CELL_PX)
        # Y axis: screen top = row 4 (north), bottom = row 0 (south)
        row = 4 - int(y / self.CELL_PX)
        return col, row

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        # Background
        painter.fillRect(self.rect(), QtGui.QColor(CLR_BG))

        frame = self._frame

        for row in range(5):
            for col in range(5):
                node_id = row * 5 + col
                sx = self.PAD + col * self.CELL_PX
                sy = self.PAD + (4 - row) * self.CELL_PX  # flip Y

                # Cell fill
                if frame and frame.maze_current_node == node_id:
                    color = CLR_CELL_CURRENT
                elif frame and frame.maze_start_node == node_id and frame.maze_start_node >= 0:
                    color = CLR_CELL_START
                elif frame and frame.maze_visited[node_id]:
                    color = CLR_CELL_VISITED
                else:
                    color = CLR_CELL_UNVISITED

                painter.setBrush(QtGui.QColor(color))
                painter.setPen(QtCore.Qt.PenStyle.NoPen)
                painter.drawRoundedRect(sx + 2, sy + 2, self.CELL_PX - 4, self.CELL_PX - 4, 6, 6)

                # Node label
                painter.setPen(QtGui.QColor(CLR_TEXT))
                painter.setFont(QtGui.QFont("Consolas", 9))
                painter.drawText(
                    QtCore.QRectF(sx, sy, self.CELL_PX, self.CELL_PX),
                    QtCore.Qt.AlignmentFlag.AlignCenter,
                    f"({col},{row})",
                )

        # Draw edges (walls / open / unknown)
        if frame:
            pen = QtGui.QPen()
            pen.setWidth(self.WALL_PX)
            pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)

            for row in range(5):
                for col in range(5):
                    node_id = row * 5 + col
                    sx = self.PAD + col * self.CELL_PX
                    sy = self.PAD + (4 - row) * self.CELL_PX

                    edges = frame.maze_edges[node_id]

                    # EAST edge (right side)
                    if col < 4:
                        edge_state = edges[0]
                        pen.setColor(QtGui.QColor(self._edge_color(edge_state)))
                        painter.setPen(pen)
                        x = sx + self.CELL_PX
                        painter.drawLine(x, sy + 8, x, sy + self.CELL_PX - 8)

                    # NORTH edge (top side in world = top on screen since we flip)
                    if row < 4:
                        edge_state = edges[1]
                        pen.setColor(QtGui.QColor(self._edge_color(edge_state)))
                        painter.setPen(pen)
                        y = sy
                        painter.drawLine(sx + 8, y, sx + self.CELL_PX - 8, y)

                    # WEST edge (left side) — only draw for col=0 boundary
                    if col == 0:
                        edge_state = edges[2]
                        pen.setColor(QtGui.QColor(self._edge_color(edge_state)))
                        painter.setPen(pen)
                        painter.drawLine(sx, sy + 8, sx, sy + self.CELL_PX - 8)

                    # SOUTH edge (bottom side) — only draw for row=0 boundary
                    if row == 0:
                        edge_state = edges[3]
                        pen.setColor(QtGui.QColor(self._edge_color(edge_state)))
                        painter.setPen(pen)
                        y = sy + self.CELL_PX
                        painter.drawLine(sx + 8, y, sx + self.CELL_PX - 8, y)

        # Border
        painter.setPen(QtGui.QColor(CLR_BORDER))
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))
        painter.end()

    @staticmethod
    def _edge_color(state: int) -> str:
        if state == 1:
            return CLR_EDGE_OPEN
        if state == 2:
            return CLR_EDGE_WALL
        return CLR_EDGE_UNKNOWN


# ── OGM Map View ───────────────────────────────────────────────────────────

class MapView(QtWidgets.QLabel):
    def __init__(self) -> None:
        super().__init__()
        self._pixmap: QtGui.QPixmap | None = None
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(400, 400)
        self.setStyleSheet(f"background:{CLR_BG}; border:1px solid {CLR_BORDER};")

    def set_frame(self, frame: MapFrame) -> None:
        self._pixmap = self._build_pixmap(frame)
        self._refresh_pixmap()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        self._refresh_pixmap()

    def _refresh_pixmap(self) -> None:
        if self._pixmap is None:
            return
        scaled = self._pixmap.scaled(
            self.size(),
            QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.FastTransformation,
        )
        self.setPixmap(scaled)

    @staticmethod
    def _build_pixmap(frame: MapFrame) -> QtGui.QPixmap:
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
            image_data.data, width, height, width * 3,
            QtGui.QImage.Format.Format_RGB888,
        ).copy()

        # Robot dot
        if frame.robot_inside_grid:
            rx = frame.robot_cell_x
            ry = height - 1 - frame.robot_cell_y
            if 0 <= rx < width and 0 <= ry < height:
                qimage.setPixelColor(rx, ry, QtGui.QColor("#f25f5c"))

        return QtGui.QPixmap.fromImage(qimage)


# ── Main Window ─────────────────────────────────────────────────────────────

class MainWindow(QtWidgets.QWidget):
    def __init__(self, default_port: str | None, baudrate: int) -> None:
        super().__init__()
        self._baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._parser = TelemetryParser()
        self._last_frame: MapFrame | None = None

        self.setWindowTitle("Maze Explorer — 5×5 Graph Navigation")
        self.resize(1280, 800)
        self.setStyleSheet(f"background:{CLR_PANEL}; color:{CLR_TEXT};")

        self._create_widgets()
        self._build_layout()
        self._wire_events()
        self._refresh_ports(default_port)

        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.setInterval(25)
        self._poll_timer.timeout.connect(self._poll_serial)
        self._poll_timer.start()

        if default_port:
            self._toggle_connection()

    # ── Widget Creation ──

    def _create_widgets(self) -> None:
        # Connection bar
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        self.refresh_btn = QtWidgets.QPushButton("⟳ Refresh")
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.status_banner = QtWidgets.QLabel("Disconnected")
        self.status_banner.setStyleSheet(f"color:{CLR_WARN}; font-weight:600;")

        # Maze graph view
        self.maze_view = MazeGraphView()

        # OGM map view
        self.ogm_view = MapView()

        # Explore control panel
        self.start_col_spin = QtWidgets.QSpinBox()
        self.start_col_spin.setRange(0, 4)
        self.start_col_spin.setValue(0)
        self.start_row_spin = QtWidgets.QSpinBox()
        self.start_row_spin.setRange(0, 4)
        self.start_row_spin.setValue(0)
        self.start_explore_btn = QtWidgets.QPushButton("▶ Start Explore")
        self.start_explore_btn.setStyleSheet(
            f"background:{CLR_ACCENT}; color:#fff; font-weight:bold; padding:6px 12px; border-radius:4px;"
        )
        self.stop_btn = QtWidgets.QPushButton("■ Stop")
        self.stop_btn.setStyleSheet(
            f"background:{CLR_ERROR}; color:#fff; font-weight:bold; padding:6px 12px; border-radius:4px;"
        )

        # Explore status
        self.explore_state_label = QtWidgets.QLabel("State: IDLE")
        self.explore_state_label.setStyleSheet(f"font-size:14px; font-weight:bold; color:{CLR_ACCENT};")
        self.explore_info_label = QtWidgets.QLabel("Waiting for start command")
        self.explore_info_label.setWordWrap(True)
        self.explore_info_label.setStyleSheet(f"color:{CLR_TEXT_DIM};")

        # Debug command
        self.debug_edit = QtWidgets.QLineEdit()
        self.debug_edit.setPlaceholderText("ASCII command, e.g. P0.70,0")
        self.debug_send_btn = QtWidgets.QPushButton("Send")

        # Telemetry details
        self.telemetry_box = QtWidgets.QPlainTextEdit()
        self.telemetry_box.setReadOnly(True)
        self.telemetry_box.setStyleSheet(
            f"background:{CLR_BG}; color:{CLR_TEXT}; font-family:Consolas,monospace; "
            f"border:1px solid {CLR_BORDER}; font-size:11px;"
        )
        self.telemetry_box.setMaximumHeight(260)

    # ── Layout ──

    def _build_layout(self) -> None:
        # Top bar
        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addWidget(QtWidgets.QLabel("Port"))
        top_bar.addWidget(self.port_combo, 1)
        top_bar.addWidget(QtWidgets.QLabel(f"@ {self._baudrate}"))
        top_bar.addWidget(self.refresh_btn)
        top_bar.addWidget(self.connect_btn)
        top_bar.addWidget(self.status_banner)

        # Left: Maze + OGM stacked
        left_col = QtWidgets.QVBoxLayout()
        left_col.addWidget(self._section_label("5×5 Maze Graph"))
        left_col.addWidget(self.maze_view)
        left_col.addWidget(self._section_label("Occupancy Grid Map"))
        left_col.addWidget(self.ogm_view, 1)

        # Right: Controls + telemetry
        right_col = QtWidgets.QVBoxLayout()
        right_col.addWidget(self._build_explore_panel())
        right_col.addWidget(self._build_debug_panel())
        right_col.addWidget(self._section_label("Telemetry"))
        right_col.addWidget(self.telemetry_box, 1)

        # Combine
        body = QtWidgets.QHBoxLayout()
        body.addLayout(left_col, 3)
        right_widget = QtWidgets.QWidget()
        right_widget.setLayout(right_col)
        right_widget.setMinimumWidth(360)
        body.addWidget(right_widget, 2)

        root = QtWidgets.QVBoxLayout(self)
        root.addLayout(top_bar)
        root.addLayout(body, 1)

    def _build_explore_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Maze Exploration")
        group.setStyleSheet(
            f"QGroupBox{{border:1px solid {CLR_BORDER}; margin-top:10px; padding:12px; border-radius:4px;}}"
            f"QGroupBox::title{{color:{CLR_ACCENT};}}"
        )

        form = QtWidgets.QHBoxLayout()
        form.addWidget(QtWidgets.QLabel("Start Col:"))
        form.addWidget(self.start_col_spin)
        form.addWidget(QtWidgets.QLabel("Row:"))
        form.addWidget(self.start_row_spin)

        buttons = QtWidgets.QHBoxLayout()
        buttons.addWidget(self.start_explore_btn)
        buttons.addWidget(self.stop_btn)

        layout = QtWidgets.QVBoxLayout(group)
        layout.addLayout(form)
        layout.addLayout(buttons)
        layout.addWidget(self.explore_state_label)
        layout.addWidget(self.explore_info_label)
        layout.addWidget(QtWidgets.QLabel("💡 Click a cell on the maze to set start position"))
        return group

    def _build_debug_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Debug Command")
        group.setStyleSheet(
            f"QGroupBox{{border:1px solid {CLR_BORDER}; margin-top:10px; padding:12px; border-radius:4px;}}"
            f"QGroupBox::title{{color:{CLR_TEXT_DIM};}}"
        )
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.debug_edit, 1)
        row.addWidget(self.debug_send_btn)
        layout = QtWidgets.QVBoxLayout(group)
        layout.addLayout(row)
        return group

    @staticmethod
    def _section_label(text: str) -> QtWidgets.QLabel:
        lbl = QtWidgets.QLabel(text)
        lbl.setStyleSheet(f"color:{CLR_ACCENT}; font-weight:bold; font-size:12px; margin-top:4px;")
        return lbl

    # ── Events ──

    def _wire_events(self) -> None:
        self.refresh_btn.clicked.connect(lambda: self._refresh_ports(self.port_combo.currentText().strip()))
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.start_explore_btn.clicked.connect(self._send_start_explore)
        self.stop_btn.clicked.connect(self._send_stop)
        self.debug_send_btn.clicked.connect(self._send_debug)
        self.debug_edit.returnPressed.connect(self._send_debug)
        self.maze_view.cellClicked.connect(self._on_maze_cell_clicked)

    def _on_maze_cell_clicked(self, col: int, row: int) -> None:
        self.start_col_spin.setValue(col)
        self.start_row_spin.setValue(row)
        self.explore_info_label.setText(f"Start cell set to ({col}, {row})")
        self.explore_info_label.setStyleSheet(f"color:{CLR_ACCENT};")

    # ── Serial ──

    def _refresh_ports(self, preferred: str | None) -> None:
        ports = [p.device for p in list_ports.comports()]
        current = preferred or self.port_combo.currentText().strip()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current:
            if current not in ports:
                self.port_combo.addItem(current)
            self.port_combo.setCurrentText(current)
        self.port_combo.blockSignals(False)

    def _toggle_connection(self) -> None:
        if self._serial and self._serial.is_open:
            self._close_serial()
            return
        port = self.port_combo.currentText().strip()
        if not port:
            self._set_status("Select a serial port", CLR_ERROR)
            return
        try:
            self._serial = serial.Serial(port=port, baudrate=self._baudrate, timeout=0)
        except serial.SerialException as exc:
            self._serial = None
            self._set_status(f"Open failed: {exc}", CLR_ERROR)
            return
        self.connect_btn.setText("Disconnect")
        self._set_status(f"Connected to {port}", CLR_SUCCESS)
        self._parser = TelemetryParser()

    def _close_serial(self) -> None:
        if self._serial:
            try:
                if self._serial.is_open:
                    self._serial.close()
            finally:
                self._serial = None
        self.connect_btn.setText("Connect")
        self._set_status("Disconnected", CLR_WARN)

    def _set_status(self, text: str, color: str) -> None:
        self.status_banner.setText(text)
        self.status_banner.setStyleSheet(f"color:{color}; font-weight:600;")

    def _write_cmd(self, cmd: bytes) -> bool:
        if not self._serial or not self._serial.is_open:
            self.explore_info_label.setText("Not connected")
            self.explore_info_label.setStyleSheet(f"color:{CLR_ERROR};")
            return False
        try:
            self._serial.write(cmd)
            return True
        except serial.SerialException as exc:
            self.explore_info_label.setText(f"Send failed: {exc}")
            self.explore_info_label.setStyleSheet(f"color:{CLR_ERROR};")
            self._close_serial()
            return False

    # ── Commands ──

    def _send_start_explore(self) -> None:
        col = self.start_col_spin.value()
        row = self.start_row_spin.value()
        if self._write_cmd(build_set_start_cell_command(col, row)):
            self.explore_info_label.setText(f"Sent: CELL {col} {row}")
            self.explore_info_label.setStyleSheet(f"color:{CLR_SUCCESS};")

    def _send_stop(self) -> None:
        if self._write_cmd(build_clear_goal_command()):
            self.explore_info_label.setText("Exploration stopped")
            self.explore_info_label.setStyleSheet(f"color:{CLR_WARN};")

    def _send_debug(self) -> None:
        text = self.debug_edit.text()
        try:
            cmd = build_debug_command_line(text)
        except (UnicodeEncodeError, ValueError):
            return
        if self._write_cmd(cmd):
            self.explore_info_label.setText(f"Debug sent: {text}")
            self.explore_info_label.setStyleSheet(f"color:{CLR_SUCCESS};")

    # ── Polling ──

    def _poll_serial(self) -> None:
        if not self._serial:
            return
        try:
            waiting = self._serial.in_waiting
            chunk = self._serial.read(waiting or 1)
        except serial.SerialException as exc:
            self._set_status(f"Serial error: {exc}", CLR_ERROR)
            self._close_serial()
            return
        for frame in self._parser.feed(chunk):
            self._apply_frame(frame)

    def _apply_frame(self, frame: MapFrame) -> None:
        self._last_frame = frame

        # Update maze graph view
        self.maze_view.set_frame(frame)

        # Update OGM
        self.ogm_view.set_frame(frame)

        # Update explore state
        state_name = MAZE_EXPLORE_STATE_LABELS.get(frame.maze_explore_state, str(frame.maze_explore_state))
        self.explore_state_label.setText(f"State: {state_name}")

        visited_count = sum(frame.maze_visited)
        current = frame.maze_current_node
        cur_col = current % 5 if current >= 0 else -1
        cur_row = current // 5 if current >= 0 else -1
        self.explore_info_label.setText(
            f"Visited: {visited_count}/25 | "
            f"Current: ({cur_col},{cur_row}) | "
            f"Mode: {CONTROL_MODE_LABELS.get(frame.control_mode, '?')} "
            f"Move: {RELATIVE_MOVE_STATE_LABELS.get(frame.relative_move_state, '?')}"
        )
        self.explore_info_label.setStyleSheet(f"color:{CLR_TEXT_DIM};")

        # Color the state label
        if frame.maze_explore_state == 6:  # DONE
            self.explore_state_label.setStyleSheet(f"font-size:14px; font-weight:bold; color:{CLR_SUCCESS};")
        elif frame.maze_explore_state >= 1:  # active
            self.explore_state_label.setStyleSheet(f"font-size:14px; font-weight:bold; color:{CLR_ACCENT};")
        else:
            self.explore_state_label.setStyleSheet(f"font-size:14px; font-weight:bold; color:{CLR_TEXT_DIM};")

        # Telemetry details
        skip_reason = SKIP_REASON_LABELS.get(frame.last_skip_reason, str(frame.last_skip_reason))
        nav_status = NAVIGATION_STATUS_LABELS.get(frame.nav_status, str(frame.nav_status))
        ctrl_mode = CONTROL_MODE_LABELS.get(frame.control_mode, str(frame.control_mode))
        move_st = RELATIVE_MOVE_STATE_LABELS.get(frame.relative_move_state, str(frame.relative_move_state))

        self.telemetry_box.setPlainText("\n".join([
            f"─── Pose & Grid ───",
            f"pose          : ({frame.pose_x_m:.3f}, {frame.pose_y_m:.3f}, {frame.pose_theta_deg:.1f}°)",
            f"grid          : {frame.width}×{frame.height} @ {frame.resolution_m_per_cell:.3f} m/cell",
            f"robot_cell    : ({frame.robot_cell_x}, {frame.robot_cell_y})",
            f"skip_reason   : {skip_reason}",
            f"tick_ms       : {frame.tick_ms}",
            f"",
            f"─── Control ───",
            f"mode          : {ctrl_mode}",
            f"move_state    : {move_st}",
            f"base_speed    : {frame.base_speed_mps:+.4f} m/s",
            f"angle_error   : {frame.angle_error_deg:+.2f}°",
            f"pos_error     : {frame.position_error_m:.4f} m",
            f"PWM L/R       : {frame.left_pwm}/{frame.right_pwm}",
            f"",
            f"─── Navigation ───",
            f"nav_status    : {nav_status}",
            f"update_count  : {frame.nav_update_count}",
            f"",
            f"─── Maze Graph ───",
            f"explore_state : {MAZE_EXPLORE_STATE_LABELS.get(frame.maze_explore_state, '?')}",
            f"current_node  : {frame.maze_current_node}",
            f"start_node    : {frame.maze_start_node}",
            f"visited       : {visited_count}/25",
        ]))

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._close_serial()
        super().closeEvent(event)


# ── Entry Point ─────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description="5×5 Maze Graph Explorer Host App")
    parser.add_argument("--port", help="Serial port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(default_port=args.port, baudrate=args.baud)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
