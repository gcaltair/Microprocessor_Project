from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from navigation_protocol import build_clear_goal_command, build_set_goal_command


def test_navigation_commands_are_ascii_lines() -> None:
    assert build_set_goal_command(0.1254, -1.2) == b"NAV 0.125 -1.200\n"
    assert build_clear_goal_command() == b"NAVC\n"
