from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import pytest

QtWidgets = pytest.importorskip("PySide6.QtWidgets")

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
        grid_initialized=1,
        robot_inside_grid=1,
        map_update_active=1,
        last_skip_reason=0,
        last_localization_mode=0,
        robot_cell_x=1,
        robot_cell_y=1,
        odom_delta_theta_deg=0.5,
        odom_delta_translation_m=0.02,
        pose_x_m=0.1,
        pose_y_m=0.2,
        pose_theta_deg=45.0,
        skipped_turning_count=0,
        skipped_settle_count=0,
        skipped_quality_count=0,
        nav_goal_valid=1,
        nav_target_valid=1,
        nav_status=1,
        nav_phase=1,
        nav_update_count=3,
        nav_fail_count=0,
        nav_raw_path_len=4,
        nav_smooth_path_len=2,
        nav_distance_to_goal_m=0.5,
        nav_goal_x_m=0.6,
        nav_goal_y_m=0.7,
        nav_target_x_m=0.2,
        nav_target_y_m=0.3,
        nav_path_points=((0.1, 0.2), (0.2, 0.3)),
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
        grid_initialized=1,
        robot_inside_grid=1,
        map_update_active=1,
        last_skip_reason=3,
        last_localization_mode=0,
        robot_cell_x=2,
        robot_cell_y=3,
        odom_delta_theta_deg=0.6,
        odom_delta_translation_m=0.03,
        pose_x_m=0.3,
        pose_y_m=0.4,
        pose_theta_deg=90.0,
        skipped_turning_count=1,
        skipped_settle_count=2,
        skipped_quality_count=3,
        nav_goal_valid=1,
        nav_target_valid=1,
        nav_status=4,
        nav_phase=3,
        nav_update_count=4,
        nav_fail_count=1,
        nav_raw_path_len=5,
        nav_smooth_path_len=3,
        nav_distance_to_goal_m=0.4,
        nav_goal_x_m=0.6,
        nav_goal_y_m=0.7,
        nav_target_x_m=0.35,
        nav_target_y_m=0.45,
        nav_path_points=((0.3, 0.4), (0.35, 0.45), (0.6, 0.7)),
        control_mode=1,
        relative_move_state=2,
        left_motor_direction=0,
        right_motor_direction=1,
        left_pwm=1200,
        right_pwm=1300,
        left_speed_pid_output=514,
        right_speed_pid_output=-615,
        position_pid_output_mps=0.12,
        angle_pid_output_mps=-0.03,
        angle_error_deg=2.5,
        position_error_m=0.18,
        base_speed_mps=0.10,
        left_speed_setpoint_mps=0.07,
        right_speed_setpoint_mps=0.13,
        left_speed_feedback_mps=0.06,
        right_speed_feedback_mps=0.11,
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
    assert "nav_status              : busy" in stats_text
    assert "nav_phase               : driving" in stats_text
    assert "nav_path_points         : 3" in stats_text
    assert "control_mode            : position" in stats_text
    assert "relative_move_state     : driving" in stats_text
    assert "pwm_left_right          : 1200 / 1300" in stats_text
    assert "pid_speed_output_l_r    : +514 / -615" in stats_text
    assert "PWM L/R= 1200/ 1300" in window.control_telemetry.text()
    assert "fw_status=driving" in window.nav_telemetry.text()
    assert window.send_goal_button.text() == "Send Goal"
    assert window.plan_button.text() == "Plan"
    assert window.clear_goal_button.text() == "Clear Goal"
    assert window.send_debug_command_button.text() == "Send Debug"
    assert window.debug_command_edit.text() == "P1,0"
    assert window.record_button.text() == "Start Recording"
    assert window.save_button.text() == "Save CSV"
    assert window.map_view._pixmap is not None

    window.close()
    app.quit()


if __name__ == "__main__":
    test_main_window_refreshes_when_new_frames_arrive()
    print("ui_smoke_ok")
