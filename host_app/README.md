# Mapping Host App

This host app reads binary telemetry frames from `UART5` and renders the live occupancy grid.

## Firmware side

- Telemetry port: `UART5`
- Baud rate: `921600`
- Frame format: `magic(2) + version(1) + type(1) + seq(2) + payload_len(2) + payload + crc16(2)`
- Current frame type:
  - `0x01`: full occupancy-grid snapshot plus mapping statistics

## Run

Use the project virtual environment:

```powershell
.\.venv\Scripts\python.exe .\host_app\main.py --port COM5
```

If you do not pass `--port`, the UI will let you choose one manually.

## Notes

- `USART6` remains dedicated to the lidar.
- The host app ignores unrelated `UART5` debug text and only parses valid telemetry frames.
- Grid colors:
  - dark: occupied
  - light: free
  - gray: unknown / weak evidence
  - red: robot pose
