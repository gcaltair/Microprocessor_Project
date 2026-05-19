from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
from PySide6 import QtWidgets

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from main import MainWindow
from telemetry_protocol import MapFrame


def test_main_window_refreshes_when_new_frames_arrive() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MainWindow(default_port=None, baudrate=921600)

    frame1 = MapFrame(
        sequence=1,
        width=4,
        height=4,
        resolution_m_per_cell=0.05,
        origin_x_m=-2.4,
        origin_y_m=-2.4,
        update_count=10,
        last_scan_sequence=50,
        tick_ms=1000,
        usable_points=40,
        endpoints_written=12,
        localization_inliers=18,
        grid_initialized=1,
        robot_inside_grid=1,
        map_update_active=1,
        last_skip_reason=0,
        last_localization_mode=0,
        robot_cell_x=1,
        robot_cell_y=1,
        localization_fitness_m=0.01,
        odom_delta_theta_deg=0.5,
        odom_delta_translation_m=0.02,
        pose_x_m=0.1,
        pose_y_m=0.2,
        pose_theta_deg=45.0,
        skipped_turning_count=0,
        skipped_settle_count=0,
        skipped_quality_count=0,
        cells=np.array(
            [[0, 10, -10, 0], [0, 0, 0, 0], [0, 0, 20, 0], [-15, 0, 0, 0]],
            dtype=np.int8,
        ),
    )
    frame2 = MapFrame(
        sequence=2,
        width=4,
        height=4,
        resolution_m_per_cell=0.05,
        origin_x_m=-2.4,
        origin_y_m=-2.4,
        update_count=11,
        last_scan_sequence=51,
        tick_ms=1250,
        usable_points=41,
        endpoints_written=13,
        localization_inliers=19,
        grid_initialized=1,
        robot_inside_grid=1,
        map_update_active=1,
        last_skip_reason=3,
        last_localization_mode=0,
        robot_cell_x=2,
        robot_cell_y=3,
        localization_fitness_m=0.02,
        odom_delta_theta_deg=0.6,
        odom_delta_translation_m=0.03,
        pose_x_m=0.3,
        pose_y_m=0.4,
        pose_theta_deg=90.0,
        skipped_turning_count=1,
        skipped_settle_count=2,
        skipped_quality_count=3,
        cells=np.array(
            [[0, 0, 0, 0], [0, 20, 0, 0], [0, 0, -15, 0], [0, 0, 0, 0]],
            dtype=np.int8,
        ),
    )

    window._apply_frame(frame1)
    first_text = window.frame_info.text()
    window._apply_frame(frame2)
    second_text = window.frame_info.text()
    stats_text = window.stats_box.toPlainText()

    assert "frame=1 update=10 scan=50" in first_text
    assert "frame=2 update=11 scan=51" in second_text
    assert "skip_reason             : quality" in stats_text
    assert "localization_mode       : odometry" in stats_text
    assert window.map_view._pixmap is not None

    window.close()
    app.quit()


if __name__ == "__main__":
    test_main_window_refreshes_when_new_frames_arrive()
    print("ui_smoke_ok")
