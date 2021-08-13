import sys

from PyQt5.QtWidgets import QApplication

import shared
from dashboard import Dashboard


def app_start():
    shared.teleop_keyboard.start()
    app = QApplication([])
    widget = Dashboard()
    widget.showMaximized()
    widget.setWindowTitle("mrl-dashboard")
    sys.exit(app.exec_())
