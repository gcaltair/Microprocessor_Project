from __future__ import annotations

import argparse

from host_app.protocol.telemetry import TelemetryStreamParser


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a recorded serial log and print decoded events.")
    parser.add_argument("file", help="Path to the recorded binary log")
    args = parser.parse_args()

    stream_parser = TelemetryStreamParser()
    data = open(args.file, "rb").read()
    for event in stream_parser.feed(data):
        print(event)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
