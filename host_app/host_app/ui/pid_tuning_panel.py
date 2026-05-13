from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
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

        layout = QVBoxLayout(self)
        layout.addLayout(self._build_controls())

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend(offset=(8, 8))
        self.angle_curve = self.plot.plot(pen=pg.mkPen("#ef4444", width=2), name="ang_err")
        self.turn_curve = self.plot.plot(pen=pg.mkPen("#2563eb", width=2), name="turn x100")
        self.left_curve = self.plot.plot(pen=pg.mkPen("#16a34a", width=2), name="left_sp x100")
        self.right_curve = self.plot.plot(pen=pg.mkPen("#f59e0b", width=2), name="right_sp x100")
        layout.addWidget(self.plot, stretch=1)

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

        self.show_all_button.clicked.connect(lambda: self.controller.show_pid_tunings())
        self.show_one_button.clicked.connect(lambda: self.controller.show_pid_tunings(self.current_loop))
        self.apply_button.clicked.connect(self._apply_tuning)
        self.load_button.clicked.connect(self._load_current_loop)

        row.addWidget(QLabel("Loop"))
        row.addWidget(self.loop_combo)
        row.addWidget(self.show_all_button)
        row.addWidget(self.show_one_button)
        row.addWidget(self.load_button)
        row.addWidget(self.apply_button)
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

        stats = QGridLayout()
        self.sample_label = QLabel("samples=0")
        self.last_sample_label = QLabel("-")
        self.last_sample_label.setWordWrap(True)
        stats.addWidget(self.sample_label, 0, 0)
        stats.addWidget(self.last_sample_label, 1, 0)
        root.addLayout(stats)
        return root

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
        samples = list(state.control_debug_samples)
        self.sample_label.setText(f"samples={len(samples)}")
        if not samples:
            self.angle_curve.setData([], [])
            self.turn_curve.setData([], [])
            self.left_curve.setData([], [])
            self.right_curve.setData([], [])
            self.last_sample_label.setText("-")
            return

        xs = list(range(len(samples)))
        self.angle_curve.setData(xs, [sample.angle_error_deg for sample in samples])
        self.turn_curve.setData(xs, [sample.turn_output_mps * 100.0 for sample in samples])
        self.left_curve.setData(xs, [sample.left_speed_setpoint_mps * 100.0 for sample in samples])
        self.right_curve.setData(xs, [sample.right_speed_setpoint_mps * 100.0 for sample in samples])

        last = samples[-1]
        self.last_sample_label.setText(
            f"{last.source}: ang={last.angle_error_deg:.2f}, turn={last.turn_output_mps:.3f}, "
            f"l={last.left_speed_setpoint_mps:.3f}, r={last.right_speed_setpoint_mps:.3f}, "
            f"pwm=({last.pwm_left},{last.pwm_right})"
        )
