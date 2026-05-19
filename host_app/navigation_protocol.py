from __future__ import annotations


def build_set_goal_command(x_m: float, y_m: float) -> bytes:
    return f"NAV {x_m:.3f} {y_m:.3f}\n".encode("ascii")


def build_clear_goal_command() -> bytes:
    return b"NAVC\n"


def build_debug_command_line(command: str) -> bytes:
    line = command.strip()
    if not line:
        raise ValueError("debug command is empty")

    return f"{line}\n".encode("ascii")
