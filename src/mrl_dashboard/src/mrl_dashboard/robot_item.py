import os

from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QWidget, QPushButton, QLabel, QCheckBox
from PyQt5.QtCore import QDir
from PyQt5 import uic

import shared
from add_robot import AddRobot


class RobotItem(QWidget):
    def __init__(self, model, parent):
        super(RobotItem, self).__init__()
        self.load_ui()
        self.model = model
        self.parent = parent

        # events
        self.edit_button = self.findChild(QPushButton, 'edit_button')
        self.edit_button.clicked.connect(self.edit_btn_click)

        self.remove_button = self.findChild(QPushButton, 'remove_button')
        self.remove_button.clicked.connect(self.remove_btn_click)

        self.bot_name_label = self.findChild(QLabel, 'bot_name_label')
        self.bot_strategy_label = self.findChild(QLabel, 'bot_strategy_label')
        self.bot_active_checkbox = self.findChild(QCheckBox, 'robot_active_checkbox')
        self.bot_image_label = self.findChild(QLabel, 'bot_image_label')

        self.update_widget()

    def edit_btn_click(self):
        widget = AddRobot(self.model)
        widget.setWindowTitle('edit robot')
        widget.show()
        widget.exec_()
        self.parent.update_list()

    def remove_btn_click(self):
        shared.robot_model_list.remove(self.model)
        self.parent.update_list()

    def update_widget(self):
        if self.model is not None:
            self.bot_active_checkbox.setChecked(self.model.status)
            self.bot_name_label.setText(self.model.name)
            self.bot_strategy_label.setText(self.model.nav_strategy)
            root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))

            if self.model.type == 'Pioneer3D':
                pixmap = QPixmap(str(root_dir.path()) + "/resource/icons/p3at.png")
                self.bot_image_label.setPixmap(pixmap)
            elif self.model.type == 'Quadrator':
                pixmap = QPixmap(str(root_dir.path()) + "/resource/icons/quadrator.png")
                self.bot_image_label.setPixmap(pixmap)

    def load_ui(self):
        root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
        ui_path = str(root_dir.path()) + "/ui/robot_item.ui"
        uic.loadUi(ui_path, self)
