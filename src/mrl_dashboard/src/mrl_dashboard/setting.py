import os
import pickle
from PyQt5.QtGui import QTextLine

from PyQt5.QtWidgets import QDialog, QPushButton, QListWidget, QListWidgetItem, QRadioButton, QLineEdit
from PyQt5.QtCore import QDir
from PyQt5 import uic

import shared
from add_robot import AddRobot
from mrl_dashboard.global_config import GlobalConfig
from robot_item import RobotItem


def save_robot_config():
    root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
    conf_path = str(root_dir.path()) + '/config/robot_config.dat'
    with open(conf_path, "wb") as f:
        pickle.dump(shared.robot_model_list, f)


class Setting(QDialog):
    def __init__(self):
        super(Setting, self).__init__()
        self.load_ui()

        # ui binding
        self.new_button = self.findChild(QPushButton, 'new_button')
        self.exit_button = self.findChild(QPushButton, 'exit_button')

        self.robot_list_widget = self.findChild(QListWidget, 'robot_list_widget')

        self.logitech_radio = self.findChild(QRadioButton, 'logitech_joy_radio')
        self.xbox_radio = self.findChild(QRadioButton, 'xbox_joy_radio')
        self.joy_off_radio = self.findChild(QRadioButton, 'joy_off_radio')

        self.victim_on_radio = self.findChild(QRadioButton, 'victim_on_radio')
        self.victim_off_radio = self.findChild(QRadioButton, 'victim_off_radio')

        self.nav_fn_radio = self.findChild(QRadioButton, 'nav_fn_radio')
        self.nav_teb_radio = self.findChild(QRadioButton, 'nav_teb_radio')

        self.explore_lite_radio = self.findChild(QRadioButton, 'explore_lite_radio')
        self.frontier_explore_radio = self.findChild(QRadioButton, 'frontier_explore_radio')
        self.random_walk_radio = self.findChild(QRadioButton, 'random_walk_radio')

        self.map_width_text = self.findChild(QLineEdit, 'map_width_text')
        self.map_height_text = self.findChild(QLineEdit, 'map_height_text')
        self.scan_topic_text = self.findChild(QLineEdit, 'scan_topic_text')

        # events
        self.new_button.clicked.connect(self.new_btn_click)
        self.update_list()

    def new_btn_click(self):
        widget = AddRobot(None)
        widget.setWindowTitle('new robot')
        widget.show()
        widget.exec_()
        self.update_list()

    def save_global_config(self):
        new_config = GlobalConfig()
        if self.logitech_radio.isChecked():
            new_config.joy_type = 'logitech'
        elif self.xbox_radio.isChecked():
            new_config.joy_type = 'xbox'
        elif self.joy_off_radio.isChecked():
            new_config.joy_type = 'off'

        if self.victim_on_radio.isChecked():
            new_config.victim_detection = True
        elif self.victim_off_radio.isChecked():
            new_config.victim_detection = False

        if self.nav_teb_radio.isChecked():
            new_config.nav_method = 'teb'
        elif self.nav_fn_radio.isChecked():
            new_config.nav_method = 'navfn'

        if self.explore_lite_radio.isChecked():
            new_config.exp_method = 'explore_lite'
        elif self.frontier_explore_radio.isChecked():
            new_config.exp_method = 'frontier_explore'
        elif self.random_walk_radio.isChecked():
            new_config.exp_method = 'random_walk'

        new_config.map_width = str(self.map_width_text.text())
        new_config.map_height = str(self.map_height_text.text())
        new_config.scan_topic = str(self.scan_topic_text.text())
        shared.global_config = new_config
        root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
        conf_path = str(root_dir.path()) + '/config/global_config.dat'
        with open(conf_path, "wb") as f:
            pickle.dump(new_config, f)

    def update_list(self):
        self.robot_list_widget.clear()
        for item in shared.robot_model_list:
            w_item = QListWidgetItem(self.robot_list_widget)
            new_item = RobotItem(item, self)
            self.robot_list_widget.setItemWidget(w_item, new_item)
            w_item.setSizeHint(new_item.size())

        if shared.global_config.joy_type.__contains__('logitech'):
            self.logitech_radio.setChecked(True)
        elif shared.global_config.joy_type.__contains__('xbox'):
            self.xbox_radio.setChecked(True)
        else:
            self.joy_off_radio.setChecked(True)

        if shared.global_config.victim_detection:
            self.victim_on_radio.setChecked(True)
        else:
            self.victim_off_radio.setChecked(True)

        if shared.global_config.nav_method.__contains__('teb'):
            self.nav_teb_radio.setChecked(True)
        elif shared.global_config.nav_method.__contains__('navfn'):
            self.nav_teb_radio.setChecked(True)
        else:
            self.nav_huskey_radio.setChecked(True)

        if shared.global_config.exp_method.__contains__('explore_lite'):
            self.explore_lite_radio.setChecked(True)
        elif shared.global_config.nav_method.__contains__('frontier_explore'):
            self.explore_lite_radio.setChecked(True)
        else:
            self.random_walk_radio.setChecked(True)

        self.map_width_text.setText(shared.global_config.map_width)
        self.map_height_text.setText(shared.global_config.map_height)
        self.scan_topic_text.setText(shared.global_config.scan_topic)

        save_robot_config()

    def load_ui(self):
        root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
        ui_path = str(root_dir.path()) + "/ui/setting.ui"
        uic.loadUi(ui_path, self)
