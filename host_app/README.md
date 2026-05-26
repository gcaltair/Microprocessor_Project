# Mapping Host App

This desktop application reads the UART5 binary telemetry stream, renders the live
occupancy grid, sends navigation/debug commands, and records benchmark CSV evidence.

## Firmware Interface

- Port: UART5 Bluetooth link, `921600` baud
- Frame envelope: `magic(2) + version(1) + type(1) + seq(2) + payload_len(2) + payload + crc16(2)`
- Accepted protocol versions: v7 and v8
- Frame type `0x01`: occupancy-grid snapshot plus map, navigation, and control stats
- Protocol v8 additional status: battery, speed limit, safety/stop state, LiDAR
  activity/recovery count, control heartbeat age, and benchmark elapsed/exit/return
  times

## Run

Install host dependencies in the project environment, then launch:

```powershell
python .\host_app\main.py --port COM5
```

Without `--port`, select the COM port in the application.

## Operation

- Click the map to fill goal coordinates, then select **Send Goal** to send
  `NAV x,y`. A driving goal starts timing and firmware automatically returns home
  after reaching it.
- Select **Plan** for a non-driving plan request and **Clear Goal** to send `NAVC`.
- Use **Start Recording** before a timed run, then **Stop Recording** and **Save CSV**.
  Output is stored under `host_app/logs/`.
- The debug command field appends `\n` automatically. Useful commands are:

| Command | Function |
| --- | --- |
| `P` / `Q` | Enable / disable periodic binary telemetry |
| `P1,0` | Direct relative move, bypassing A* navigation |
| `E` or `STOP` | Emergency stop |
| `C` | Clear a permitted stop latch |
| `G` or `HOME` | Return home |
| `START` | Clear permitted stop state, reset map, and restart LiDAR |

## Map Colors

- dark: occupied
- light: free
- gray: unknown or weak evidence
- red: current robot pose
- blue: navigation goal

`USART6` remains dedicated to the LiDAR; UART5 telemetry frames use DMA on the
firmware side to avoid delaying the real-time motor-control task.
