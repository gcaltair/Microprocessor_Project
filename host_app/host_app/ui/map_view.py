from __future__ import annotations

from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

from host_app.models.state import MapSnapshot, NavigationState, RobotPose


class MapCanvas(QWidget):
    goal_requested = Signal(int, int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(320, 320)
        self.map_snapshot = MapSnapshot()
        self.robot_pose = RobotPose()
        self.navigation = NavigationState()

    def update_state(self, map_snapshot: MapSnapshot, robot_pose: RobotPose, navigation: NavigationState) -> None:
        self.map_snapshot = map_snapshot
        self.robot_pose = robot_pose
        self.navigation = navigation
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#111827"))

        meta = self.map_snapshot.meta
        if meta.width_cells <= 0 or meta.height_cells <= 0 or not self.map_snapshot.cells:
            painter.setPen(QColor("#d1d5db"))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待地图数据")
            return

        cell_w = self.width() / meta.width_cells
        cell_h = self.height() / meta.height_cells

        for cell_y in range(meta.height_cells):
            for cell_x in range(meta.width_cells):
                value = self.map_snapshot.cells[cell_y * meta.width_cells + cell_x]
                color = QColor("#374151")
                if value <= -10:
                    color = QColor("#e5e7eb")
                elif value >= 10:
                    color = QColor("#111827")

                painter.fillRect(
                    int(cell_x * cell_w),
                    int(self.height() - (cell_y + 1) * cell_h),
                    int(cell_w + 1),
                    int(cell_h + 1),
                    color,
                )

        painter.setPen(QPen(QColor("#10b981"), max(2, int(min(cell_w, cell_h) * 0.18))))
        for cell_x, cell_y in self.navigation.path_cells:
            painter.drawPoint(QPointF((cell_x + 0.5) * cell_w, self.height() - (cell_y + 0.5) * cell_h))

        robot_cell = self._world_to_cell(self.robot_pose.x_m, self.robot_pose.y_m)
        if robot_cell is not None:
            center_x = (robot_cell[0] + 0.5) * cell_w
            center_y = self.height() - (robot_cell[1] + 0.5) * cell_h
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor("#ef4444"))
            radius = max(4, int(min(cell_w, cell_h) * 0.35))
            painter.drawEllipse(QPointF(center_x, center_y), radius, radius)

    def mousePressEvent(self, event) -> None:
        meta = self.map_snapshot.meta
        if meta.width_cells <= 0 or meta.height_cells <= 0:
            return

        cell_w = self.width() / meta.width_cells
        cell_h = self.height() / meta.height_cells
        cell_x = int(event.position().x() / cell_w)
        cell_y = int((self.height() - event.position().y()) / cell_h)
        if 0 <= cell_x < meta.width_cells and 0 <= cell_y < meta.height_cells:
            self.goal_requested.emit(cell_x, cell_y)

    def _world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        meta = self.map_snapshot.meta
        if meta.resolution_m_per_cell <= 0:
            return None
        cell_x = int((x_m - meta.origin_x_m) / meta.resolution_m_per_cell)
        cell_y = int((y_m - meta.origin_y_m) / meta.resolution_m_per_cell)
        if 0 <= cell_x < meta.width_cells and 0 <= cell_y < meta.height_cells:
            return cell_x, cell_y
        return None
