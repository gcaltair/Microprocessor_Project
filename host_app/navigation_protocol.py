from __future__ import annotations


def build_set_goal_command(x_m: float, y_m: float) -> bytes:
    return f"NAV {x_m:.3f} {y_m:.3f}\n".encode("ascii")


def build_clear_goal_command() -> bytes:
    return b"NAVC\n"
