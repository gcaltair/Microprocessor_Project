from host_app.protocol.pid_tuning import build_pid_set_command, build_pid_show_command, parse_pid_text_line
from host_app.protocol.telemetry import TextLine
from host_app.services.store import SessionStore
from host_app.tools.pid_tuner_cli import build_parser


def test_pid_command_builders() -> None:
    assert build_pid_show_command() == "PK"
    assert build_pid_show_command("angle") == "PKA"
    assert build_pid_set_command("left_speed", 8000.0, 33705.0, 33.0) == "PKL,8000,33705,33"


def test_parse_pid_tuning_line() -> None:
    event = parse_pid_text_line("PID loop=A name=angle kp=0.0575 ki=0 kd=0.001")

    assert event is not None
    assert event.loop == "A"
    assert event.name == "angle"
    assert event.kp == 0.0575
    assert event.ki == 0.0
    assert event.kd == 0.001


def test_parse_pid_set_line() -> None:
    event = parse_pid_text_line("PID set loop=A name=angle kp=0.05 ki=0 kd=0.001")

    assert event is not None
    assert event.loop == "A"
    assert event.kp == 0.05


def test_parse_line_with_command_echo_prefix() -> None:
    pid = parse_pid_text_line("PKPID loop=A name=angle kp=0.0575 ki=0 kd=0.001")
    odom = parse_pid_text_line(
        "OODOM x=0.948 y=-0.001 th=-0.32 ls=0.123 rs=0.145 base=0.100 ang_sp=90.00 mode=MANUAL move=IDLE"
    )

    assert pid is not None
    assert pid.loop == "A"
    assert odom is not None
    assert odom.theta_deg == -0.32


def test_parse_control_debug_lines() -> None:
    ctrl = parse_pid_text_line("CTRLDBG pos_err=0.040 ang_err=-0.50 turn=0.010 l_sp=0.020 r_sp=0.040 pwm=(120,130)")
    lctrl = parse_pid_text_line("LCTRL pos_err=0.052 ang_err=1.20 base=0.020 turn=0.008 l_sp=0.012 r_sp=0.028 pwm=(90,110)")
    tctrl = parse_pid_text_line("TCTRL ang_err=1.50 base=0.000 turn=0.050 l_sp=-0.050 r_sp=0.050 pwm=(-200,210)")

    assert ctrl is not None and ctrl.source == "CTRLDBG"
    assert ctrl.position_error_m == 0.04
    assert ctrl.base_speed_mps is None
    assert lctrl is not None and lctrl.source == "LCTRL"
    assert lctrl.base_speed_mps == 0.02
    assert tctrl is not None and tctrl.source == "TCTRL"
    assert tctrl.left_speed_setpoint_mps == -0.05


def test_parse_odometry_response_line() -> None:
    event = parse_pid_text_line(
        "ODOM x=0.948 y=-0.001 th=-0.32 ls=0.123 rs=0.145 base=0.100 ang_sp=90.00 mode=MANUAL move=IDLE"
    )

    assert event is not None
    assert event.theta_deg == -0.32
    assert event.left_speed_mps == 0.123
    assert event.right_speed_mps == 0.145
    assert event.angle_setpoint_deg == 90.0


def test_parse_move_progress_line() -> None:
    event = parse_pid_text_line("MOVE cmd=(0.300,0.000) target=0.300 progress=0.239 remain=0.061")

    assert event is not None
    assert event.target_distance_m == 0.3
    assert event.progress_m == 0.239
    assert event.remaining_m == 0.061


def test_store_tracks_pid_tuning_and_control_samples() -> None:
    store = SessionStore()

    store.apply_event(TextLine("PID loop=A name=angle kp=0.0575 ki=0 kd=0.001"))
    store.apply_event(
        TextLine("CTRLDBG pos_err=0.040 ang_err=-0.50 turn=0.010 l_sp=0.020 r_sp=0.040 pwm=(120,130)")
    )
    store.apply_event(
        TextLine("ODOM x=0.948 y=-0.001 th=-0.32 ls=0.123 rs=0.145 base=0.100 ang_sp=90.00 mode=MANUAL move=IDLE")
    )
    store.apply_event(TextLine("MOVE cmd=(0.300,0.000) target=0.300 progress=0.239 remain=0.061"))

    assert store.state.pid_tunings["A"].kp == 0.0575
    assert len(store.state.control_debug_samples) == 1
    assert store.state.control_debug_samples[-1].source == "CTRLDBG"
    assert len(store.state.control_response_samples) == 1
    assert store.state.control_response_samples[-1].left_speed_mps == 0.123
    assert len(store.state.move_progress_samples) == 1
    assert store.state.move_progress_samples[-1].remaining_m == 0.061


def test_store_ignores_historical_control_snapshots_for_live_plot() -> None:
    store = SessionStore()

    store.apply_event(TextLine("TCTRL ang_err=1.50 base=0.000 turn=0.050 l_sp=-0.050 r_sp=0.050 pwm=(-200,210)"))
    store.apply_event(TextLine("LCTRL pos_err=0.052 ang_err=1.20 base=0.020 turn=0.008 l_sp=0.012 r_sp=0.028 pwm=(90,110)"))

    assert len(store.state.control_debug_samples) == 0


def test_store_clears_pid_samples() -> None:
    store = SessionStore()

    store.apply_event(TextLine("TCTRL ang_err=1.50 base=0.000 turn=0.050 l_sp=-0.050 r_sp=0.050 pwm=(-200,210)"))
    store.apply_event(
        TextLine("ODOM x=0.948 y=-0.001 th=-0.32 ls=0.123 rs=0.145 base=0.100 ang_sp=90.00 mode=MANUAL move=IDLE")
    )
    store.apply_event(TextLine("MOVE cmd=(0.300,0.000) target=0.300 progress=0.239 remain=0.061"))
    store.clear_pid_samples()

    assert len(store.state.control_debug_samples) == 0
    assert len(store.state.control_response_samples) == 0
    assert len(store.state.move_progress_samples) == 0


def test_cli_accepts_options_after_subcommand() -> None:
    args = build_parser().parse_args(
        ["send", "R0", "A90", "O", "--port", "COM7", "--duration", "3", "--jsonl", "logs/pid_angle.jsonl"]
    )

    assert args.command_name == "send"
    assert args.port == "COM7"
    assert args.duration == 3.0
    assert str(args.jsonl) == "logs\\pid_angle.jsonl" or str(args.jsonl) == "logs/pid_angle.jsonl"
