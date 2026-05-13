from __future__ import annotations


class DeviceCommandBuilder:
    @staticmethod
    def manual(command: str) -> str:
        return command.strip().upper()

    @staticmethod
    def set_speed(value: float) -> str:
        return f"V{value:.2f}"

    @staticmethod
    def goto_cell(x: int, y: int) -> str:
        return f"J{x},{y}"

    @staticmethod
    def relative_move(dx: float, dy: float) -> str:
        return f"P{dx:.3f},{dy:.3f}"

    @staticmethod
    def telemetry(enabled: bool) -> str:
        return "Y1" if enabled else "Y0"

    @staticmethod
    def scan_stream(enabled: bool) -> str:
        return "Q1" if enabled else "Q0"

    @staticmethod
    def lidar(enabled: bool) -> str:
        return "M" if enabled else "N"

    @staticmethod
    def baud(baud_rate: int) -> str:
        return f"B{baud_rate}"

    @staticmethod
    def pid_show(loop: str | None = None) -> str:
        from host_app.protocol.pid_tuning import build_pid_show_command

        return build_pid_show_command(loop)

    @staticmethod
    def pid_set(loop: str, kp: float, ki: float, kd: float) -> str:
        from host_app.protocol.pid_tuning import build_pid_set_command

        return build_pid_set_command(loop, kp, ki, kd)
