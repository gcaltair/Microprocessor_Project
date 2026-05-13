from __future__ import annotations

from dataclasses import asdict, dataclass
import re
from typing import Literal


PidLoopId = Literal["A", "L", "R", "P"]

LOOP_ALIASES: dict[str, PidLoopId] = {
    "A": "A",
    "ANGLE": "A",
    "L": "L",
    "LEFT": "L",
    "LEFT_SPEED": "L",
    "R": "R",
    "RIGHT": "R",
    "RIGHT_SPEED": "R",
    "P": "P",
    "POSITION": "P",
}

LOOP_NAMES: dict[PidLoopId, str] = {
    "A": "angle",
    "L": "left_speed",
    "R": "right_speed",
    "P": "position",
}


@dataclass(slots=True)
class PidTuning:
    loop: PidLoopId
    name: str
    kp: float
    ki: float
    kd: float

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(slots=True)
class ControlDebugSample:
    source: Literal["CTRLDBG", "LCTRL", "TCTRL"]
    position_error_m: float | None
    angle_error_deg: float
    base_speed_mps: float | None
    turn_output_mps: float
    left_speed_setpoint_mps: float
    right_speed_setpoint_mps: float
    pwm_left: int
    pwm_right: int
    raw: str

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(slots=True)
class ControlResponseSample:
    x_m: float
    y_m: float
    theta_deg: float
    left_speed_mps: float
    right_speed_mps: float
    base_speed_mps: float
    angle_setpoint_deg: float
    raw: str

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


PidTextEvent = PidTuning | ControlDebugSample | ControlResponseSample

_PID_LINE_RE = re.compile(
    r"^PID(?:\s+set)?\s+loop=(?P<loop>[ALRP])\s+name=(?P<name>\S+)\s+"
    r"kp=(?P<kp>[-+0-9.eE]+)\s+ki=(?P<ki>[-+0-9.eE]+)\s+kd=(?P<kd>[-+0-9.eE]+)$"
)
_CTRLDBG_RE = re.compile(
    r"^CTRLDBG\s+pos_err=(?P<pos>[-+0-9.eE]+)\s+ang_err=(?P<ang>[-+0-9.eE]+)\s+"
    r"turn=(?P<turn>[-+0-9.eE]+)\s+l_sp=(?P<left>[-+0-9.eE]+)\s+"
    r"r_sp=(?P<right>[-+0-9.eE]+)\s+pwm=\((?P<pwm_left>-?\d+),(?P<pwm_right>-?\d+)\)$"
)
_LCTRL_RE = re.compile(
    r"^LCTRL\s+pos_err=(?P<pos>[-+0-9.eE]+)\s+ang_err=(?P<ang>[-+0-9.eE]+)\s+"
    r"base=(?P<base>[-+0-9.eE]+)\s+turn=(?P<turn>[-+0-9.eE]+)\s+"
    r"l_sp=(?P<left>[-+0-9.eE]+)\s+r_sp=(?P<right>[-+0-9.eE]+)\s+"
    r"pwm=\((?P<pwm_left>-?\d+),(?P<pwm_right>-?\d+)\)$"
)
_TCTRL_RE = re.compile(
    r"^TCTRL\s+ang_err=(?P<ang>[-+0-9.eE]+)\s+base=(?P<base>[-+0-9.eE]+)\s+"
    r"turn=(?P<turn>[-+0-9.eE]+)\s+l_sp=(?P<left>[-+0-9.eE]+)\s+"
    r"r_sp=(?P<right>[-+0-9.eE]+)\s+pwm=\((?P<pwm_left>-?\d+),(?P<pwm_right>-?\d+)\)$"
)
_ODOM_RE = re.compile(
    r"^ODOM\s+x=(?P<x>[-+0-9.eE]+)\s+y=(?P<y>[-+0-9.eE]+)\s+th=(?P<th>[-+0-9.eE]+)\s+"
    r"ls=(?P<left>[-+0-9.eE]+)\s+rs=(?P<right>[-+0-9.eE]+)\s+"
    r"base=(?P<base>[-+0-9.eE]+)\s+ang_sp=(?P<ang_sp>[-+0-9.eE]+)"
)


def normalize_loop_id(loop: str) -> PidLoopId:
    key = loop.strip().upper().replace("-", "_")
    try:
        return LOOP_ALIASES[key]
    except KeyError as exc:
        raise ValueError("loop must be one of A/angle, L/left_speed, R/right_speed, P/position") from exc


def build_pid_show_command(loop: str | None = None) -> str:
    if loop is None:
        return "PK"
    return f"PK{normalize_loop_id(loop)}"


def build_pid_set_command(loop: str, kp: float, ki: float, kd: float) -> str:
    loop_id = normalize_loop_id(loop)
    return f"PK{loop_id},{kp:.6g},{ki:.6g},{kd:.6g}"


def parse_pid_text_line(text: str) -> PidTextEvent | None:
    stripped = text.strip()
    pid_match = _PID_LINE_RE.match(stripped)
    if pid_match is not None:
        loop = normalize_loop_id(pid_match.group("loop"))
        return PidTuning(
            loop=loop,
            name=pid_match.group("name"),
            kp=float(pid_match.group("kp")),
            ki=float(pid_match.group("ki")),
            kd=float(pid_match.group("kd")),
        )

    ctrl_match = _CTRLDBG_RE.match(stripped)
    if ctrl_match is not None:
        return _control_sample_from_match("CTRLDBG", ctrl_match, stripped)

    lctrl_match = _LCTRL_RE.match(stripped)
    if lctrl_match is not None:
        return _control_sample_from_match("LCTRL", lctrl_match, stripped)

    tctrl_match = _TCTRL_RE.match(stripped)
    if tctrl_match is not None:
        return _control_sample_from_match("TCTRL", tctrl_match, stripped)

    odom_match = _ODOM_RE.match(stripped)
    if odom_match is not None:
        return ControlResponseSample(
            x_m=float(odom_match.group("x")),
            y_m=float(odom_match.group("y")),
            theta_deg=float(odom_match.group("th")),
            left_speed_mps=float(odom_match.group("left")),
            right_speed_mps=float(odom_match.group("right")),
            base_speed_mps=float(odom_match.group("base")),
            angle_setpoint_deg=float(odom_match.group("ang_sp")),
            raw=stripped,
        )

    return None


def _control_sample_from_match(
    source: Literal["CTRLDBG", "LCTRL", "TCTRL"], match: re.Match[str], raw: str
) -> ControlDebugSample:
    position_error = float(match.group("pos")) if "pos" in match.groupdict() else None
    base_speed = float(match.group("base")) if "base" in match.groupdict() else None
    return ControlDebugSample(
        source=source,
        position_error_m=position_error,
        angle_error_deg=float(match.group("ang")),
        base_speed_mps=base_speed,
        turn_output_mps=float(match.group("turn")),
        left_speed_setpoint_mps=float(match.group("left")),
        right_speed_setpoint_mps=float(match.group("right")),
        pwm_left=int(match.group("pwm_left")),
        pwm_right=int(match.group("pwm_right")),
        raw=raw,
    )
