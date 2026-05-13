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


def test_store_tracks_pid_tuning_and_control_samples() -> None:
    store = SessionStore()

    store.apply_event(TextLine("PID loop=A name=angle kp=0.0575 ki=0 kd=0.001"))
    store.apply_event(TextLine("TCTRL ang_err=1.50 base=0.000 turn=0.050 l_sp=-0.050 r_sp=0.050 pwm=(-200,210)"))

    assert store.state.pid_tunings["A"].kp == 0.0575
    assert len(store.state.control_debug_samples) == 1
    assert store.state.control_debug_samples[-1].source == "TCTRL"


def test_cli_accepts_options_after_subcommand() -> None:
    args = build_parser().parse_args(
        ["send", "R0", "A90", "O", "--port", "COM7", "--duration", "3", "--jsonl", "logs/pid_angle.jsonl"]
    )

    assert args.command_name == "send"
    assert args.port == "COM7"
    assert args.duration == 3.0
    assert str(args.jsonl) == "logs\\pid_angle.jsonl" or str(args.jsonl) == "logs/pid_angle.jsonl"
