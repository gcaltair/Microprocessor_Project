from __future__ import annotations

from PySide6.QtWidgets import QFormLayout, QLabel, QWidget

from host_app.models.state import SessionState


class StatusPanel(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.labels: dict[str, QLabel] = {}
        layout = QFormLayout(self)

        for key, title in [
            ("odom", "ODOM"),
            ("est", "EST"),
            ("ctrl", "CTRL"),
            ("runtime", "Runtime"),
            ("localization", "Localization"),
            ("navigation", "Navigation"),
            ("mapping", "Map"),
            ("link", "Link"),
        ]:
            label = QLabel("-")
            label.setWordWrap(True)
            self.labels[key] = label
            layout.addRow(title, label)

    @staticmethod
    def _map_status_text(state: SessionState) -> str:
        if state.runtime.map_update_active:
            status = "active"
        else:
            status = {
                1: "paused(turning)",
                2: "paused(settle)",
                3: "paused(quality)",
            }.get(state.runtime.map_last_skip_reason, "paused")

        return (
            f"{status}, skip={state.runtime.mapping_skipped_turning_count}/"
            f"{state.runtime.mapping_skipped_settle_count}/"
            f"{state.runtime.mapping_skipped_quality_count}"
        )

    def update_state(self, state: SessionState) -> None:
        self.labels["odom"].setText(
            f"x={state.odom_pose.x_m:.3f}, y={state.odom_pose.y_m:.3f}, th={state.odom_pose.theta_deg:.2f}"
        )
        self.labels["est"].setText(
            f"x={state.estimated_pose.x_m:.3f}, y={state.estimated_pose.y_m:.3f}, th={state.estimated_pose.theta_deg:.2f}"
        )
        self.labels["ctrl"].setText(
            f"x={state.control_pose.x_m:.3f}, y={state.control_pose.y_m:.3f}, th={state.control_pose.theta_deg:.2f}"
        )
        self.labels["runtime"].setText(
            f"ctrl={state.runtime.control_cycles}, cmd={state.runtime.cmd_rx}/{state.runtime.cmd_drop}, "
            f"scan={state.runtime.scan_count}, dma_drop={state.runtime.dma_drop}, "
            f"heap={state.runtime.heap_free}/{state.runtime.heap_min}"
        )
        self.labels["localization"].setText(
            f"mode={state.runtime.localization_mode}, inliers={state.runtime.localization_inliers}, "
            f"fit={state.runtime.localization_fitness_m * 1000.0:.1f} mm"
        )
        self.labels["navigation"].setText(
            f"state={state.navigation.state}, goal={state.navigation.goal_cell}, "
            f"step={state.navigation.current_waypoint_index}/{len(state.navigation.path_cells)}, "
            f"dist={state.navigation.last_waypoint_distance_m:.2f} m"
        )
        self.labels["mapping"].setText(self._map_status_text(state))
        self.labels["link"].setText(
            f"lidar={'on' if state.runtime.lidar_active else 'off'}, "
            f"telemetry={'on' if state.runtime.telemetry_enabled else 'off'}, "
            f"scan={'on' if state.runtime.scan_stream_enabled else 'off'}"
        )
