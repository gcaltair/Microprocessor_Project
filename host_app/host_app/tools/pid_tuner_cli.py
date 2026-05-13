from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Iterable

from host_app.protocol import TelemetryStreamParser, TextLine
from host_app.protocol.pid_tuning import (
    build_pid_set_command,
    build_pid_show_command,
    parse_pid_text_line,
)


def _serial_transport_class():
    from host_app.transport import SerialTransport

    return SerialTransport


class PidTunerClient:
    def __init__(self, port: str, baud_rate: int, timeout: float = 0.02) -> None:
        self.transport = _serial_transport_class()()
        self.parser = TelemetryStreamParser()
        self.transport.open(port, baud_rate, timeout=timeout)

    def close(self) -> None:
        self.transport.close()

    def send(self, command: str) -> None:
        self.transport.write_line(command)

    def collect_text_lines(self, duration_s: float) -> list[str]:
        deadline = time.monotonic() + duration_s
        lines: list[str] = []
        while time.monotonic() < deadline:
            data = self.transport.poll()
            if data:
                for event in self.parser.feed(data):
                    if isinstance(event, TextLine):
                        lines.append(event.text)
            else:
                time.sleep(0.01)
        return lines


def _print_lines(lines: Iterable[str], json_output: bool, jsonl_path: Path | None = None) -> None:
    if jsonl_path is not None:
        jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    jsonl_handle = jsonl_path.open("a", encoding="utf-8") if jsonl_path is not None else None
    try:
        for line in lines:
            parsed = parse_pid_text_line(line)
            if json_output:
                payload: dict[str, object]
                if parsed is None:
                    payload = {"type": "text", "raw": line}
                else:
                    payload = {"type": type(parsed).__name__, **parsed.to_dict()}
                encoded = json.dumps(payload, ensure_ascii=False)
                print(encoded)
                if jsonl_handle is not None:
                    jsonl_handle.write(encoded + "\n")
            else:
                print(line)
                if jsonl_handle is not None:
                    parsed_payload = {"type": "text", "raw": line} if parsed is None else {
                        "type": type(parsed).__name__,
                        **parsed.to_dict(),
                    }
                    jsonl_handle.write(json.dumps(parsed_payload, ensure_ascii=False) + "\n")
    finally:
        if jsonl_handle is not None:
            jsonl_handle.close()


def _with_client(args: argparse.Namespace) -> PidTunerClient:
    return PidTunerClient(args.port, args.baud, timeout=args.timeout)


def command_list_ports(_: argparse.Namespace) -> int:
    for port in _serial_transport_class().available_ports():
        print(port)
    return 0


def command_show(args: argparse.Namespace) -> int:
    client = _with_client(args)
    try:
        client.send(build_pid_show_command(args.loop))
        lines = client.collect_text_lines(args.duration)
        _print_lines(lines, args.json, args.jsonl)
    finally:
        client.close()
    return 0


def command_set(args: argparse.Namespace) -> int:
    command = build_pid_set_command(args.loop, args.kp, args.ki, args.kd)
    client = _with_client(args)
    try:
        client.send(command)
        lines = client.collect_text_lines(args.duration)
        _print_lines(lines, args.json, args.jsonl)
    finally:
        client.close()
    return 0


def command_send(args: argparse.Namespace) -> int:
    client = _with_client(args)
    try:
        for command in args.command:
            client.send(command)
            time.sleep(args.command_gap)
        lines = client.collect_text_lines(args.duration)
        _print_lines(lines, args.json, args.jsonl)
    finally:
        client.close()
    return 0


def command_monitor(args: argparse.Namespace) -> int:
    client = _with_client(args)
    try:
        lines = client.collect_text_lines(args.duration)
        _print_lines(lines, args.json, args.jsonl)
    finally:
        client.close()
    return 0


def _add_connection_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", default=argparse.SUPPRESS, help="Serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=argparse.SUPPRESS, help="Serial baud rate")
    parser.add_argument("--timeout", type=float, default=argparse.SUPPRESS, help="Serial read/write timeout in seconds")
    parser.add_argument("--duration", type=float, default=argparse.SUPPRESS, help="Seconds to collect responses")
    parser.add_argument("--json", action="store_true", default=argparse.SUPPRESS, help="Print parsed events as JSON lines")
    parser.add_argument("--jsonl", type=Path, default=argparse.SUPPRESS, help="Append parsed events to a JSONL file")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PID tuning CLI for the STM32 robot car.")
    parser.add_argument("--port", help="Serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--timeout", type=float, default=0.02, help="Serial read/write timeout in seconds")
    parser.add_argument("--duration", type=float, default=1.0, help="Seconds to collect responses")
    parser.add_argument("--json", action="store_true", help="Print parsed events as JSON lines")
    parser.add_argument("--jsonl", type=Path, help="Append parsed events to a JSONL file")

    subparsers = parser.add_subparsers(dest="command_name", required=True)

    list_ports = subparsers.add_parser("list-ports", help="List available serial ports")
    list_ports.set_defaults(func=command_list_ports)

    show = subparsers.add_parser("show", help="Show PID tuning values")
    _add_connection_options(show)
    show.add_argument("--loop", help="Optional loop: A/angle, L/left_speed, R/right_speed, P/position")
    show.set_defaults(func=command_show)

    set_cmd = subparsers.add_parser("set", help="Set PID tuning values in firmware RAM")
    _add_connection_options(set_cmd)
    set_cmd.add_argument("loop", help="Loop: A/angle, L/left_speed, R/right_speed, P/position")
    set_cmd.add_argument("kp", type=float)
    set_cmd.add_argument("ki", type=float)
    set_cmd.add_argument("kd", type=float)
    set_cmd.set_defaults(func=command_set)

    send = subparsers.add_parser("send", help="Send raw robot command(s) then collect output")
    _add_connection_options(send)
    send.add_argument("command", nargs="+", help="Command(s), e.g. R0 A90 O")
    send.add_argument("--command-gap", type=float, default=0.2, help="Seconds between commands")
    send.set_defaults(func=command_send)

    monitor = subparsers.add_parser("monitor", help="Collect serial output without sending commands")
    _add_connection_options(monitor)
    monitor.set_defaults(func=command_monitor)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.command_name != "list-ports" and not args.port:
        parser.error("--port is required unless using list-ports")
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
