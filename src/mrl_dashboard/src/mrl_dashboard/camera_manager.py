import math
import os

from PyQt5.QtCore import QDir, QFile
from PyQt5.QtWidgets import QMainWindow, QGridLayout
from PyQt5 import uic

import shared
from robot_camera import RobotCamera


class CameraManager(QMainWindow):
    def __init__(self):
        super(CameraManager, self).__init__()
        self.load_ui()
        self.grid_layout = self.findChild(QGridLayout, 'main_layout')

    def close(self):
        for item in shared.robot_widget_list:
            item.clse()

    def launch(self):
        shared.robot_widget_list = []

        n_rows = math.ceil(math.sqrt(len(shared.robot_model_list)))
        for i in range(0, len(shared.robot_model_list)):
            new_camera = RobotCamera(shared.robot_model_list[i])
            shared.robot_widget_list.append(new_camera)
            self.grid_layout.addWidget(new_camera, math.floor(i / n_rows), math.floor(i % n_rows))
            new_camera.connect()

    def load_ui(self):
        root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
        ui_path = str(root_dir.path()) + "/ui/camera_manager.ui"
        uic.loadUi(ui_path, self)