from __future__ import annotations

from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from host_app.services.controller import HostSessionController
from host_app.ui.map_view import MapCanvas
from host_app.ui.pid_tuning_panel import PidTuningPanel
from host_app.ui.status_panel import StatusPanel


class MainWindow(QMainWindow):
    def __init__(self, controller: HostSessionController) -> None:
        super().__init__()
        self.controller = controller
        self.pressed_keys: set[int] = set()
        self.setWindowTitle("小车上位机 V1")
        self.resize(1480, 920)

        central = QWidget(self)
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)

        top_row = QHBoxLayout()
        top_row.addWidget(self._build_connection_box(), stretch=2)
        top_row.addWidget(self._build_control_box(), stretch=2)
        top_row.addWidget(self._build_status_box(), stretch=3)
        root_layout.addLayout(top_row)

        content_splitter = QSplitter(Qt.Horizontal)
        content_splitter.addWidget(self._build_map_box())
        content_splitter.addWidget(self._build_scan_log_splitter())
        content_splitter.setStretchFactor(0, 3)
        content_splitter.setStretchFactor(1, 2)
        root_layout.addWidget(content_splitter, stretch=1)

        self.controller.state_changed.connect(self._on_state_changed)
        self.controller.ports_changed.connect(self._on_ports_changed)
        self.controller.connection_changed.connect(self._on_connection_changed)
        self.controller.log_message.connect(self._append_log)
        self.controller.error.connect(self._append_error)

    def _build_connection_box(self) -> QWidget:
        box = QGroupBox("连接")
        layout = QGridLayout(box)

        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["921600", "115200", "9600"])
        self.protocol_combo = QComboBox()
        self.protocol_combo.addItem("结构化 telemetry", userData="structured")
        self.protocol_combo.addItem("文本兼容", userData="text")
        self.connect_button = QPushButton("连接")
        self.disconnect_button = QPushButton("断开")
        self.refresh_ports_button = QPushButton("刷新串口")
        self.record_checkbox = QCheckBox("录包")
        self.replay_button = QPushButton("回放录包")
        self.connection_label = QLabel("未连接")

        layout.addWidget(QLabel("串口"), 0, 0)
        layout.addWidget(self.port_combo, 0, 1)
        layout.addWidget(self.refresh_ports_button, 0, 2)
        layout.addWidget(QLabel("波特率"), 1, 0)
        layout.addWidget(self.baud_combo, 1, 1)
        layout.addWidget(QLabel("模式"), 2, 0)
        layout.addWidget(self.protocol_combo, 2, 1)
        layout.addWidget(self.connect_button, 3, 0)
        layout.addWidget(self.disconnect_button, 3, 1)
        layout.addWidget(self.record_checkbox, 4, 0)
        layout.addWidget(self.replay_button, 4, 1)
        layout.addWidget(self.connection_label, 5, 0, 1, 3)

        self.refresh_ports_button.clicked.connect(self.controller.refresh_ports)
        self.connect_button.clicked.connect(self._connect_port)
        self.disconnect_button.clicked.connect(self.controller.disconnect_port)
        self.record_checkbox.toggled.connect(self.controller.set_recording)
        self.replay_button.clicked.connect(self._replay_log)
        return box

    def _build_control_box(self) -> QWidget:
        box = QGroupBox("控制")
        layout = QGridLayout(box)

        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.0, 10.0)
        self.speed_spin.setSingleStep(0.05)
        self.speed_spin.setValue(0.25)
        self.turn_angle_spin = QDoubleSpinBox()
        self.turn_angle_spin.setRange(1.0, 180.0)
        self.turn_angle_spin.setSingleStep(5.0)
        self.turn_angle_spin.setValue(15.0)

        self.speed_button = QPushButton("发送速度")
        self.forward_button = QPushButton("前进 (W)")
        self.backward_button = QPushButton("后退 (S)")
        self.left_button = QPushButton("左转角 (A)")
        self.right_button = QPushButton("右转角 (D)")
        self.stop_button = QPushButton("停止")
        self.lidar_on_button = QPushButton("启动 LiDAR")
        self.lidar_off_button = QPushButton("停止 LiDAR")
        self.cancel_nav_button = QPushButton("取消导航")
        self.scan_stream_checkbox = QCheckBox("点云流")

        layout.addWidget(QLabel("基础速度"), 0, 0)
        layout.addWidget(self.speed_spin, 0, 1)
        layout.addWidget(self.speed_button, 0, 2)
        layout.addWidget(QLabel("转角"), 1, 0)
        layout.addWidget(self.turn_angle_spin, 1, 1)
        layout.addWidget(self.forward_button, 2, 1)
        layout.addWidget(self.left_button, 3, 0)
        layout.addWidget(self.stop_button, 3, 1)
        layout.addWidget(self.right_button, 3, 2)
        layout.addWidget(self.backward_button, 4, 1)
        layout.addWidget(self.lidar_on_button, 5, 0)
        layout.addWidget(self.lidar_off_button, 5, 1)
        layout.addWidget(self.cancel_nav_button, 5, 2)
        layout.addWidget(self.scan_stream_checkbox, 6, 0, 1, 2)

        self.speed_button.clicked.connect(lambda: self.controller.set_speed(self.speed_spin.value()))
        self.forward_button.clicked.connect(lambda: self.controller.send_manual("F"))
        self.backward_button.clicked.connect(lambda: self.controller.send_manual("B"))
        self.left_button.clicked.connect(lambda: self.controller.send_command(f"A-{self.turn_angle_spin.value():.1f}"))
        self.right_button.clicked.connect(lambda: self.controller.send_command(f"A{self.turn_angle_spin.value():.1f}"))
        self.stop_button.clicked.connect(lambda: self.controller.send_manual("S"))
        self.lidar_on_button.clicked.connect(lambda: self.controller.set_lidar_enabled(True))
        self.lidar_off_button.clicked.connect(lambda: self.controller.set_lidar_enabled(False))
        self.cancel_nav_button.clicked.connect(lambda: self.controller.send_command("C"))
        self.scan_stream_checkbox.toggled.connect(self.controller.set_scan_stream)
        return box

    def _build_status_box(self) -> QWidget:
        box = QGroupBox("状态")
        layout = QVBoxLayout(box)
        self.status_panel = StatusPanel()
        layout.addWidget(self.status_panel)
        return box

    def _build_map_box(self) -> QWidget:
        box = QGroupBox("地图与路径")
        layout = QVBoxLayout(box)
        self.map_canvas = MapCanvas()
        self.map_canvas.goal_requested.connect(self.controller.send_goal_cell)
        layout.addWidget(self.map_canvas, stretch=1)
        layout.addWidget(QLabel("点击地图直接发送 Jx,y 导航目标"))
        return box

    def _build_scan_log_splitter(self) -> QWidget:
        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(self._build_scan_box())
        splitter.addWidget(self._build_pid_box())
        splitter.addWidget(self._build_log_box())
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 1)
        return splitter

    def _build_scan_box(self) -> QWidget:
        box = QGroupBox("点云")
        layout = QVBoxLayout(box)
        self.scan_plot = pg.PlotWidget()
        self.scan_plot.showGrid(x=True, y=True, alpha=0.25)
        self.scan_plot.setAspectLocked(True)
        self.scan_scatter = pg.ScatterPlotItem(size=5, brush=pg.mkBrush("#2563eb"))
        self.scan_plot.addItem(self.scan_scatter)
        layout.addWidget(self.scan_plot)
        return box

    def _build_pid_box(self) -> QWidget:
        box = QGroupBox("PID")
        layout = QVBoxLayout(box)
        self.pid_panel = PidTuningPanel(self.controller)
        layout.addWidget(self.pid_panel)
        return box

    def _build_log_box(self) -> QWidget:
        box = QGroupBox("日志")
        layout = QVBoxLayout(box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        layout.addWidget(self.log_view)
        return box

    def _connect_port(self) -> None:
        port = self.port_combo.currentText().strip()
        if not port:
            self._append_error("没有可用串口")
            return
        self.controller.connect_port(port, int(self.baud_combo.currentText()), self.protocol_combo.currentData())

    def _replay_log(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(self, "选择录包文件", str(Path.cwd()), "Binary log (*.bin)")
        if file_path:
            self.controller.replay_log(file_path)

    def _on_ports_changed(self, ports: list[str]) -> None:
        current = self.port_combo.currentText()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)
        self.port_combo.blockSignals(False)

    def _on_connection_changed(self, connected: bool, port: str) -> None:
        self.connection_label.setText(f"已连接: {port}" if connected else "未连接")

    def _append_log(self, text: str) -> None:
        self.log_view.appendPlainText(text)

    def _append_error(self, text: str) -> None:
        self.log_view.appendPlainText(f"[ERROR] {text}")

    def _on_state_changed(self, state) -> None:
        self.status_panel.update_state(state)
        self.pid_panel.update_state(state)
        pose = state.control_pose
        if pose.timestamp_ms == 0:
            pose = state.estimated_pose if state.estimated_pose.timestamp_ms != 0 else state.odom_pose
        self.map_canvas.update_state(state.map_snapshot, pose, state.navigation)
        self._update_scan_plot(state.latest_scan)

    def _update_scan_plot(self, scan) -> None:
        if not scan.points:
            self.scan_scatter.setData([], [])
            return

        angles_rad = np.deg2rad([point[0] + scan.corrected_pose.theta_deg for point in scan.points])
        distances_m = np.array([point[1] / 1000.0 for point in scan.points])
        xs = scan.corrected_pose.x_m + np.cos(angles_rad) * distances_m
        ys = scan.corrected_pose.y_m + np.sin(angles_rad) * distances_m
        self.scan_scatter.setData(xs, ys)

    def keyPressEvent(self, event) -> None:
        if event.isAutoRepeat() or event.key() in self.pressed_keys:
            return
        self.pressed_keys.add(event.key())
        mapping = {Qt.Key_W: "F", Qt.Key_S: "B"}
        command = mapping.get(event.key())
        if command:
            self.controller.send_manual(command)
        elif event.key() == Qt.Key_A:
            self.controller.send_command(f"A-{self.turn_angle_spin.value():.1f}")
        elif event.key() == Qt.Key_D:
            self.controller.send_command(f"A{self.turn_angle_spin.value():.1f}")
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        if event.isAutoRepeat():
            return
        self.pressed_keys.discard(event.key())
        if event.key() in {Qt.Key_W, Qt.Key_S}:
            self.controller.send_manual("S")
        super().keyReleaseEvent(event)
