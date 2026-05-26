from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pytest

from navigation_protocol import build_clear_goal_command, build_debug_command_line, build_plan_command, build_set_goal_command


def test_navigation_commands_are_ascii_lines() -> None:
    assert build_set_goal_command(0.1254, -1.2) == b"NAV 0.125 -1.200\n"
    assert build_plan_command(0.1254, -1.2) == b"PLAN 0.125 -1.200\n"
    assert build_clear_goal_command() == b"NAVC\n"


def test_debug_command_lines_are_ascii_lines() -> None:
    assert build_debug_command_line("P1,0") == b"P1,0\n"
    assert build_debug_command_line("  P1,0\n") == b"P1,0\n"

    with pytest.raises(ValueError):
        build_debug_command_line("  ")

    with pytest.raises(UnicodeEncodeError):
        build_debug_command_line("调试")
