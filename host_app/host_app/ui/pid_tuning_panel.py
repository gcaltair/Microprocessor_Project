from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from host_app.models.state import SessionState
from host_app.services.controller import HostSessionController


class PidTuningPanel(QWidget):
    def __init__(self, controller: HostSessionController, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.controller = controller
        self._latest_state: SessionState | None = None
        self._sample_index = 0

        self.live_timer = QTimer(self)
        self.live_timer.setInterval(250)
        self.live_timer.timeout.connect(self.controller.request_control_snapshot)

        layout = QVBoxLayout(self)
        layout.addLayout(self._build_controls())
        layout.addWidget(self._build_plots(), stretch=1)

    def _build_controls(self):
        root = QVBoxLayout()
        row = QHBoxLayout()

        self.loop_combo = QComboBox()
        self.loop_combo.addItem("angle", "A")
        self.loop_combo.addItem("left_speed", "L")
        self.loop_combo.addItem("right_speed", "R")
        self.loop_combo.addItem("position", "P")
        self.loop_combo.currentIndexChanged.connect(self._load_current_loop)

        self.show_all_button = QPushButton("Read All")
        self.show_one_button = QPushButton("Read Loop")
        self.apply_button = QPushButton("Apply RAM")
        self.load_button = QPushButton("Load")
        self.clear_button = QPushButton("Clear")

        self.show_all_button.clicked.connect(lambda: self.controller.show_pid_tunings())
        self.show_one_button.clicked.connect(lambda: self.controller.show_pid_tunings(self.current_loop))
        self.apply_button.clicked.connect(self._apply_tuning)
        self.load_button.clicked.connect(self._load_current_loop)
        self.clear_button.clicked.connect(self.controller.clear_pid_samples)

        row.addWidget(QLabel("Loop"))
        row.addWidget(self.loop_combo)
        row.addWidget(self.show_all_button)
        row.addWidget(self.show_one_button)
        row.addWidget(self.load_button)
        row.addWidget(self.apply_button)
        row.addWidget(self.clear_button)
        root.addLayout(row)

        form = QFormLayout()
        self.kp_spin = self._make_gain_spin()
        self.ki_spin = self._make_gain_spin()
        self.kd_spin = self._make_gain_spin()
        self.current_label = QLabel("-")
        self.current_label.setWordWrap(True)

        form.addRow("Kp", self.kp_spin)
        form.addRow("Ki", self.ki_spin)
        form.addRow("Kd", self.kd_spin)
        form.addRow("Current", self.current_label)
        root.addLayout(form)

        live = QHBoxLayout()
        self.live_checkbox = QCheckBox("Live O")
        self.live_checkbox.toggled.connect(self._set_live_polling)
        self.interval_spin = QSpinBox()
        self.interval_spin.setRange(100, 5000)
        self.interval_spin.setSingleStep(50)
        self.interval_spin.setValue(250)
        self.interval_spin.valueChanged.connect(self.live_timer.setInterval)
        self.test_angle_spin = QDoubleSpinBox()
        self.test_angle_spin.setRange(-360.0, 360.0)
        self.test_angle_spin.setSingleStep(5.0)
        self.test_angle_spin.setValue(90.0)
        self.angle_test_button = QPushButton("A Test")
        self.stop_button = QPushButton("Stop")
        self.angle_test_button.clicked.connect(self._run_angle_test)
        self.stop_button.clicked.connect(lambda: self.controller.send_manual("S"))

        live.addWidget(self.live_checkbox)
        live.addWidget(QLabel("ms"))
        live.addWidget(self.interval_spin)
        live.addWidget(QLabel("Angle"))
        live.addWidget(self.test_angle_spin)
        live.addWidget(self.angle_test_button)
        live.addWidget(self.stop_button)
        root.addLayout(live)

        stats = QGridLayout()
        self.sample_label = QLabel("samples=0")
        self.last_sample_label = QLabel("-")
        self.last_sample_label.setWordWrap(True)
        stats.addWidget(self.sample_label, 0, 0)
        stats.addWidget(self.last_sample_label, 1, 0)
        root.addLayout(stats)
        return root

    def _build_plots(self) -> QTabWidget:
        tabs = QTabWidget()

        self.angle_plot = pg.PlotWidget()
        self.angle_plot.showGrid(x=True, y=True, alpha=0.25)
        self.angle_plot.addLegend(offset=(8, 8))
        self.angle_error_curve = self.angle_plot.plot(pen=pg.mkPen("#ef4444", width=2), name="angle error deg")
        self.angle_actual_curve = self.angle_plot.plot(pen=pg.mkPen("#0f766e", width=2), name="theta deg")
        self.angle_setpoint_curve = self.angle_plot.plot(pen=pg.mkPen("#7c3aed", width=2), name="angle target deg")
        self.turn_curve = self.angle_plot.plot(pen=pg.mkPen("#2563eb", width=2), name="turn m/s x100")
        tabs.addTab(self.angle_plot, "Angle")

        self.speed_plot = pg.PlotWidget()
        self.speed_plot.showGrid(x=True, y=True, alpha=0.25)
        self.speed_plot.addLegend(offset=(8, 8))
        self.left_setpoint_curve = self.speed_plot.plot(pen=pg.mkPen("#16a34a", width=2), name="left target")
        self.right_setpoint_curve = self.speed_plot.plot(pen=pg.mkPen("#f59e0b", width=2), name="right target")
        self.left_actual_curve = self.speed_plot.plot(pen=pg.mkPen("#14532d", width=2, style=Qt.DashLine), name="left actual")
        self.right_actual_curve = self.speed_plot.plot(pen=pg.mkPen("#92400e", width=2, style=Qt.DashLine), name="right actual")
        tabs.addTab(self.speed_plot, "Speed")

        self.pwm_plot = pg.PlotWidget()
        self.pwm_plot.showGrid(x=True, y=True, alpha=0.25)
        self.pwm_plot.addLegend(offset=(8, 8))
        self.left_pwm_curve = self.pwm_plot.plot(pen=pg.mkPen("#16a34a", width=2), name="left pwm")
        self.right_pwm_curve = self.pwm_plot.plot(pen=pg.mkPen("#f59e0b", width=2), name="right pwm")
        tabs.addTab(self.pwm_plot, "PWM")

        return tabs

    @staticmethod
    def _make_gain_spin() -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 1000000.0)
        spin.setDecimals(6)
        spin.setSingleStep(0.001)
        return spin

    @property
    def current_loop(self) -> str:
        return str(self.loop_combo.currentData())

    def update_state(self, state: SessionState) -> None:
        self._latest_state = state
        self._update_current_label()
        self._update_plot(state)

    def _load_current_loop(self) -> None:
        if self._latest_state is None:
            return
        tuning = self._latest_state.pid_tunings.get(self.current_loop)
        if tuning is None:
            return
        self.kp_spin.setValue(tuning.kp)
        self.ki_spin.setValue(tuning.ki)
        self.kd_spin.setValue(tuning.kd)
        self._update_current_label()

    def _apply_tuning(self) -> None:
        self.controller.set_pid_tuning(
            self.current_loop,
            self.kp_spin.value(),
            self.ki_spin.value(),
            self.kd_spin.value(),
        )

    def _set_live_polling(self, enabled: bool) -> None:
        if enabled:
            self.controller.request_control_snapshot()
            self.live_timer.setInterval(self.interval_spin.value())
            self.live_timer.start()
        else:
            self.live_timer.stop()

    def _run_angle_test(self) -> None:
        self.controller.clear_pid_samples()
        self.controller.send_command(f"A{self.test_angle_spin.value():.1f}")
        if self.live_checkbox.isChecked():
            return
        self.live_checkbox.setChecked(True)

    def _update_current_label(self) -> None:
        if self._latest_state is None:
            self.current_label.setText("-")
            return
        tuning = self._latest_state.pid_tunings.get(self.current_loop)
        if tuning is None:
            self.current_label.setText("-")
            return
        self.current_label.setText(f"{tuning.name}: Kp={tuning.kp:.6g}, Ki={tuning.ki:.6g}, Kd={tuning.kd:.6g}")

    def _update_plot(self, state: SessionState) -> None:
        control_samples = list(state.control_debug_samples)
        response_samples = list(state.control_response_samples)
        self.sample_label.setText(f"control={len(control_samples)} response={len(response_samples)}")
        if not control_samples and not response_samples:
            self.angle_error_curve.setData([], [])
            self.turn_curve.setData([], [])
            self.angle_actual_curve.setData([], [])
            self.angle_setpoint_curve.setData([], [])
            self.left_setpoint_curve.setData([], [])
            self.right_setpoint_curve.setData([], [])
            self.left_actual_curve.setData([], [])
            self.right_actual_curve.setData([], [])
            self.left_pwm_curve.setData([], [])
            self.right_pwm_curve.setData([], [])
            self.last_sample_label.setText("-")
            return

        control_xs = list(range(len(control_samples)))
        response_xs = list(range(len(response_samples)))
        self.angle_error_curve.setData(control_xs, [sample.angle_error_deg for sample in control_samples])
        self.turn_curve.setData(control_xs, [sample.turn_output_mps * 100.0 for sample in control_samples])
        self.left_setpoint_curve.setData(control_xs, [sample.left_speed_setpoint_mps for sample in control_samples])
        self.right_setpoint_curve.setData(control_xs, [sample.right_speed_setpoint_mps for sample in control_samples])
        self.left_pwm_curve.setData(control_xs, [sample.pwm_left for sample in control_samples])
        self.right_pwm_curve.setData(control_xs, [sample.pwm_right for sample in control_samples])

        self.angle_actual_curve.setData(response_xs, [sample.theta_deg for sample in response_samples])
        self.angle_setpoint_curve.setData(response_xs, [sample.angle_setpoint_deg for sample in response_samples])
        self.left_actual_curve.setData(response_xs, [sample.left_speed_mps for sample in response_samples])
        self.right_actual_curve.setData(response_xs, [sample.right_speed_mps for sample in response_samples])

        parts = []
        if control_samples:
            last = control_samples[-1]
            parts.append(
                f"{last.source}: err={last.angle_error_deg:.2f}, turn={last.turn_output_mps:.3f}, "
                f"target=({last.left_speed_setpoint_mps:.3f},{last.right_speed_setpoint_mps:.3f}), "
                f"pwm=({last.pwm_left},{last.pwm_right})"
            )
        if response_samples:
            last_response = response_samples[-1]
            parts.append(
                f"ODOM: th={last_response.theta_deg:.2f}/{last_response.angle_setpoint_deg:.2f}, "
                f"actual=({last_response.left_speed_mps:.3f},{last_response.right_speed_mps:.3f})"
            )
        self.last_sample_label.setText(" | ".join(parts))
