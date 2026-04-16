from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from host_app.services.controller import HostSessionController
from host_app.ui.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("小车上位机 V1")
    controller = HostSessionController()
    window = MainWindow(controller)
    window.show()
    return app.exec()
