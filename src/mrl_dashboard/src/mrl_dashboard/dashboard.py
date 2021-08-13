import os
import pickle
import threading
from time import sleep
import rviz

from PyQt5.QtWidgets import QPushButton, QGridLayout, QWidget
from PyQt5.QtCore import QDir, QProcess
from PyQt5 import QtWidgets, uic

from os import path

import global_config
import shared
from camera_manager import CameraManager
from setting import Setting


def setting_btn_click():
    setting_dialog = Setting()
    setting_dialog.setWindowTitle("mrl-settings")
    setting_dialog.show()
    setting_dialog.exec_()
    setting_dialog.save_global_config()


def load_robot_config():
    root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
    conf_path = str(root_dir.path()) + '/config/robot_config.dat'
    if path.exists(conf_path):
        with open(conf_path) as f:
            data = pickle.load(f)
        shared.robot_model_list = data
    else:
        shared.robot_widget_list = []


def load_global_config():
    root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
    conf_path = str(root_dir.path()) + '/config/global_config.dat'
    if path.exists(conf_path):
        with open(conf_path) as f:
            data = pickle.load(f)
        shared.global_config = data
    else:
        shared.global_config = global_config.GlobalConfig()


class MyViz(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
        ui_file = str(root_dir.path()) + "/rviz/map_merge_" + str(len(shared.robot_model_list)) + ".rviz"

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, ui_file)

        self.frame.load(config)
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)
        self.layout = QGridLayout()
        self.layout.addWidget(self.frame)

        self.setLayout(self.layout)


class MapSpawn(threading.Thread):
    def __init__(self, process):
        threading.Thread.__init__(self)
        self.is_stop = False
        self.map_merge_process = process

    def run(self):
        spawned = False
        try_time = 5
        try_count = 0
        import rospy

        while try_count < try_time and not self.is_stop:
            sleep(2)
            topics_list = rospy.get_published_topics()
            for item in shared.robot_model_list:
                spawned = False
                for topic in topics_list:
                    if str(topic[0]).__contains__(item.name) and str(topic[0]).__contains__('map'):
                        spawned = True
                        continue
                if not spawned:
                    break
            if spawned:
                self.map_merge_process.start(
                    "gnome-terminal  --tab -e \"roslaunch mrl_navigation mrl_map_merge.launch\"")

                break
            try_count += 1

    def stop(self):
        self.is_stop = True
        self.join(2000)


class Dashboard(QtWidgets.QMainWindow):
    def __init__(self):
        super(Dashboard, self).__init__()
        self.map_merge_process = QProcess()
        self.map_spawn = MapSpawn(self.map_merge_process)
        self.joy_process = QProcess()
        self.spawn_process = QProcess()
        self.load_ui()
        load_robot_config()
        load_global_config()

        self.camera_manager = CameraManager()

        # ui bindings
        self.setting_btn = self.findChild(QPushButton, 'settingButton')
        self.exit_btn = self.findChild(QPushButton, 'exit_button')
        self.camera_btn = self.findChild(QPushButton, 'camera_button')
        self.spawn_btn = self.findChild(QPushButton, 'spawn_button')
        self.autonomous_btn = self.findChild(QPushButton, 'autonomous_button')
        self.joystick_btn = self.findChild(QPushButton, 'joystick_button')
        self.save_btn = self.findChild(QPushButton, 'save_button')

        self.rviz_grid = self.findChild(QGridLayout, 'rviz_grid')

        # events
        self.camera_btn.clicked.connect(self.camera_btn_click)
        self.setting_btn.clicked.connect(setting_btn_click)
        self.exit_btn.clicked.connect(self.exit_btn_click)
        self.spawn_btn.clicked.connect(self.spawn_btn_click)
        self.save_btn.clicked.connect(self.save_btn_click)
        setting_btn_click()
        self.bring_up()

    def bring_up(self):
        if not shared.global_config.joy_type.__contains__('off'):
            self.joy_process.start("gnome-terminal  --tab -e \"roslaunch mrl_dashboard bring_up.launch\"")

        self.my_viz = MyViz()
        self.rviz_grid.addWidget(self.my_viz, 0, 1, 9, 9)

    def spawn_robots(self):
        cmd = 'gnome-terminal'
        for item in shared.robot_model_list:
            if item.type.__contains__('Pioneer3D'):
                robot_cmd = "roslaunch mrl_dashboard p3at_configure.launch joy_type:={0} " \
                            "robot_name:={1} init_x:={2} init_y:={3} " \
                            "scan_topic:={4} map_width:={5} map_height:={6} " \
                            "nav_method:={7} exp_method:={8}"
                robot_cmd = robot_cmd.format(shared.global_config.joy_type, item.name, item.init_pos_x, item.init_pos_y,
                                             shared.global_config.scan_topic, shared.global_config.map_width,
                                             shared.global_config.map_height, shared.global_config.nav_method,
                                             shared.global_config.exp_method)
                cmd += " --tab  -e \"" + robot_cmd + "\""

            elif item.type.__contains__('Quadrator'):
                cmd += " --tab  -e \"roslaunch mrl_dashboard quadrator_configure.launch joy_type:=" + shared.global_config.joy_type + " robot_name:=" + item.name + "\""
        self.spawn_process.start(cmd)

        self.map_spawn.start()

    def save_btn_click(self):
        print 'save button is not implemented'

    def spawn_btn_click(self):
        self.spawn_robots()
        self.camera_btn.setEnabled(True)
        self.autonomous_btn.setEnabled(True)
        self.joystick_btn.setEnabled(True)
        self.save_btn.setEnabled(True)
        self.setting_btn.setEnabled(False)

    def camera_btn_click(self):
        self.camera_manager.setWindowTitle("camera-manager")
        self.camera_manager.showMaximized()
        self.camera_manager.launch()

    def exit_btn_click(self):
        shared.teleop_keyboard.stop()
        self.close()

    def load_ui(self):
        root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
        os.chdir(str(root_dir.path()))
        ui_path = str(root_dir.path()) + "/ui/dashboard.ui"
        uic.loadUi(ui_path, self)  # Load the .ui file
