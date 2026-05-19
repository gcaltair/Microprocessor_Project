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


SKIP_REASON_LABELS = {
    0: "none",
    1: "turning",
    2: "settle",
    3: "quality",
}

LOCALIZATION_MODE_LABELS = {
    0: "odometry",
}


class MapView(QtWidgets.QLabel):
    def __init__(self) -> None:
        super().__init__()
        self._pixmap: QtGui.QPixmap | None = None
        self.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(520, 520)
        self.setStyleSheet("background:#14181d; border:1px solid #2f3944;")

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

        painter = QtGui.QPainter(qimage)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)

        if frame.robot_inside_grid:
            robot_x = frame.robot_cell_x
            robot_y = height - 1 - frame.robot_cell_y
            if 0 <= robot_x < width and 0 <= robot_y < height:
                center = QtCore.QPointF(robot_x + 0.5, robot_y + 0.5)
                heading_len = max(2.0, min(width, height) * 0.03)
                angle_rad = np.deg2rad(frame.pose_theta_deg)
                end = QtCore.QPointF(
                    center.x() + float(np.cos(angle_rad)) * heading_len,
                    center.y() - float(np.sin(angle_rad)) * heading_len,
                )

                painter.setPen(QtGui.QPen(QtGui.QColor("#f25f5c"), 0.7))
                painter.setBrush(QtGui.QBrush(QtGui.QColor("#f25f5c")))
                painter.drawEllipse(center, 1.6, 1.6)
                painter.drawLine(center, end)

        painter.end()
        return QtGui.QPixmap.fromImage(qimage)


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
                    f"localization_inliers    : {frame.localization_inliers}",
                    f"localization_mode       : {loc_mode}",
                    f"localization_fitness_m  : {frame.localization_fitness_m:.4f}",
                    f"map_update_active       : {frame.map_update_active}",
                    f"skip_reason             : {skip_reason}",
                    f"robot_inside_grid       : {frame.robot_inside_grid}",
                    f"robot_cell              : ({frame.robot_cell_x}, {frame.robot_cell_y})",
                    f"odom_delta_theta_deg    : {frame.odom_delta_theta_deg:.3f}",
                    f"odom_delta_translation  : {frame.odom_delta_translation_m:.4f}",
                    f"skip_turning_count      : {frame.skipped_turning_count}",
                    f"skip_settle_count       : {frame.skipped_settle_count}",
                    f"skip_quality_count      : {frame.skipped_quality_count}",
                    "",
                    "Legend:",
                    "dark   = occupied",
                    "light  = free",
                    "gray   = unknown / weak evidence",
                    "red    = robot pose",
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
