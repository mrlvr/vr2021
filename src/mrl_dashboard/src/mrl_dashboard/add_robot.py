import rospy
import os
from PyQt5.QtWidgets import QLineEdit, QComboBox, QPushButton, QDialogButtonBox, QRadioButton, QDialog
from PyQt5.QtCore import QDir
from PyQt5 import uic

import shared
from robot_model import RobotModel


class AddRobot(QDialog):
    def __init__(self, robot_model):
        super(AddRobot, self).__init__()
        self.load_ui()

        self.model = robot_model

        # ui binding
        self.name_line_edit = self.findChild(QLineEdit, 'name_line_edit')
        self.type_combo_box = self.findChild(QComboBox, 'type_combo_box')

        self.init_pos_x_line_edit = self.findChild(QLineEdit, 'init_pos_x_line_edit')
        self.init_pos_y_line_edit = self.findChild(QLineEdit, 'init_pos_y_line_edit')
        self.init_pos_z_line_edit = self.findChild(QLineEdit, 'init_pos_z_line_edit')

        self.init_rot_x_line_edit = self.findChild(QLineEdit, 'init_rot_x_line_edit')
        self.init_rot_y_line_edit = self.findChild(QLineEdit, 'init_rot_y_line_edit')
        self.init_rot_z_line_edit = self.findChild(QLineEdit, 'init_rot_z_line_edit')

        self.get_pos_button = self.findChild(QPushButton, 'get_pos_button')

        self.button_box = self.findChild(QDialogButtonBox, 'button_box')

        self.rgb_camera_combobox = self.findChild(QComboBox, 'rgb_camera_combobox')
        self.thermal_camera_combobox = self.findChild(QComboBox, 'thermal_camera_combobox')

        self.nav_indoor_radio = self.findChild(QRadioButton, 'nav_indoor_radio')
        self.nav_outdoor_radio = self.findChild(QRadioButton, 'nav_outdoor_radio')

        # events
        self.button_box.accepted.connect(self.box_button_accepted)
        self.button_box.rejected.connect(self.exit_btn_rejected)

        topics_list = rospy.get_published_topics()
        sorted_list = []

        for item in topics_list:
            if str(item[1]).__contains__('Image'):
                sorted_list.append(str(item[0]))

        sorted_list.sort()
        self.rgb_camera_combobox.addItems(sorted_list)
        self.thermal_camera_combobox.addItems(sorted_list)

        if self.model is None:
            self.new_model = True
        else:
            self.new_model = False
            self.update_widget()

    def load_ui(self):
        root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
        ui_path = str(root_dir.path()) + "/ui/add_robot.ui"
        uic.loadUi(ui_path, self)  # Load the .ui file

    def box_button_accepted(self):
        self.save_model()
        if self.new_model:
            shared.robot_model_list.append(self.model)

        self.close()

    def exit_btn_rejected(self):
        self.close()

    def save_model(self):
        if self.model is None:
            self.model = RobotModel()
        self.model.status = 1
        self.model.name = str(self.name_line_edit.text())
        self.model.type = str(self.type_combo_box.currentText())
        self.model.init_pos_x = str(self.init_pos_x_line_edit.text())
        self.model.init_pos_y = str(self.init_pos_y_line_edit.text())
        self.model.init_pos_z = str(self.init_pos_z_line_edit.text())
        self.model.init_rot_x = str(self.init_rot_x_line_edit.text())
        self.model.init_rot_y = str(self.init_rot_y_line_edit.text())
        self.model.init_rot_z = str(self.init_rot_z_line_edit.text())
        self.model.rgb_camera_topic = str(self.rgb_camera_combobox.currentText())
        self.model.thermal_camera_topic = str(self.thermal_camera_combobox.currentText())
        if self.nav_indoor_radio.isChecked():
            self.model.nav_strategy = 'indoor'
        else:
            self.model.nav_strategy = 'outdoor'

    def update_widget(self):
        if self.model is not None:
            self.name_line_edit.setText(self.model.name)
            self.type_combo_box.setCurrentText(self.model.type)
            self.init_pos_x_line_edit.setText(self.model.init_pos_x)
            self.init_pos_y_line_edit.setText(self.model.init_pos_y)
            self.init_pos_z_line_edit.setText(self.model.init_pos_z)
            self.init_rot_x_line_edit.setText(self.model.init_rot_x)
            self.init_rot_y_line_edit.setText(self.model.init_rot_y)
            self.init_rot_z_line_edit.setText(self.model.init_rot_z)
            self.rgb_camera_combobox.setCurrentText(self.model.rgb_camera_topic)
            self.thermal_camera_combobox.setCurrentText(self.model.thermal_camera_topic)
            if self.model.nav_strategy.__eq__('indoor'):
                self.nav_indoor_radio.setChecked(True)
            else:
                self.nav_outdoor_radio.setChecked(True)
