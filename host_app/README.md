# Mapping Host App

This host app reads binary telemetry frames from `UART5` and renders the live occupancy grid.
It can also send simple navigation commands back to the firmware on the same serial port.

## Firmware side

- Telemetry port: `UART5`
- Baud rate: `921600`
- Frame format: `magic(2) + version(1) + type(1) + seq(2) + payload_len(2) + payload + crc16(2)`
- Current frame type:
  - `0x01`: full occupancy-grid snapshot plus mapping and navigation statistics
- Host-to-firmware navigation commands:
  - `NAV <x_m> <y_m>\n`: set a navigation goal in map/world coordinates, meters
  - `NAVC\n`: clear the current navigation goal
- The `Debug Command` panel sends one raw ASCII line, with `\n` appended automatically.
  It includes a `P1,0` preset; firmware interprets `P<dx_m>,<dy_m>` as a direct
  relative move command for the base position loop, bypassing A* navigation.

## Run

Use the project virtual environment:

```powershell
.\.venv\Scripts\python.exe .\host_app\main.py --port COM5
```

If you do not pass `--port`, the UI will let you choose one manually.

## Notes

- `USART6` remains dedicated to the lidar.
- The host app ignores unrelated `UART5` debug text and only parses valid telemetry frames.
- Click the map to fill the navigation goal coordinates, then press `Send Goal`.
- Grid colors:
  - dark: occupied
  - light: free
  - gray: unknown / weak evidence
  - red: robot pose
  - blue: navigation goal
