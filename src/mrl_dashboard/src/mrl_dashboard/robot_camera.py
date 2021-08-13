import os
import cv2
import threading
from time import sleep

import rospy
from PyQt5 import QtCore
from PyQt5.QtGui import QPixmap
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry

import numpy as np

import qimage2ndarray
from PyQt5.QtWidgets import QMainWindow, QPushButton, QLabel
from PyQt5.QtCore import QDir
from PyQt5 import uic

import shared
from mrl_dashboard.robot_model import RobotModel


def check_list(clist, pose):
    cx = pose.position.x
    cy = pose.position.y

    exist = False
    for p in clist:
        sx = p[0]
        sy = p[1]
        dist = np.sqrt(np.power(sx - cx, 2) + np.power(sy - cy, 2))
        if dist < 3:
            exist = True
            break

    return exist

class VictimDetection(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.victim_data = []
        self.is_stop = False
        self.image = None
        self.is_free = True
        self.is_updated = False
        self.th_image = None

    def rgb_update(self, image):
        if self.is_free:
            self.is_updated = True
            self.is_free = False
            self.image = image

    def thermal_update(self, image):
        self.th_image = image

    def run(self):
        import cv2
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        while not self.is_stop:
            if self.is_updated:
                image = cv2.resize(self.image, (240, 240))
                img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                rects, weights = hog.detectMultiScale(img_gray, winStride=(2, 2), padding=(10, 10), scale=1.02)
                self.victim_data = []
                data = []

                if len(rects) > 0 and self.th_image is not None:
                    th_image = cv2.resize(self.th_image, (240, 240))

                for i, (x, y, w, h) in enumerate(rects):
                    if weights[i] < 0.13:
                        continue
                    p_white_pix = 0

                    if self.th_image is not None:
                        c_image = th_image[y:y + h, x:x + w]
                        # (thresh, bw_image) = cv2.threshold(c_image, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                        n_white_pix = np.sum(c_image > 127)

                        p_white_pix = ((n_white_pix * 100) / (w * h))

                    data.append([weights[i], x, y, w, h, p_white_pix])

                self.victim_data = data

                self.is_updated = False
                self.is_free = True

    def stop(self):
        self.is_stop = True
        self.join(2000)


class RobotCamera(QMainWindow):

    def __init__(self, robot_model):
        super(RobotCamera, self).__init__()
        self.load_ui()
        self.model = robot_model
        self.bridge = CvBridge()
        self.state = RobotModel.IDLE

        # ui bindings
        self.manual_btn = self.findChild(QPushButton, 'manual_button')
        self.autonomous_btn = self.findChild(QPushButton, 'autonomous_button')
        self.alive_btn = self.findChild(QPushButton, 'alive_button')
        self.dead_btn = self.findChild(QPushButton, 'dead_button')

        self.rgb_view = self.findChild(QLabel, 'rgb_view')
        self.thermal_view = self.findChild(QLabel, 'thermal_view')
        self.bot_name_label = self.findChild(QLabel, 'bot_name_label')

        # events
        self.manual_btn.clicked.connect(self.manual_button_click)
        self.autonomous_btn.clicked.connect(self.autonomous_button_click)
        self.alive_btn.clicked.connect(self.alive_button_click)
        self.dead_btn.clicked.connect(self.dead_button_click)

        # ros publishers
        self.joy_publisher = rospy.Publisher(self.model.name + '/joy_control', String, queue_size=1)
        self.victim_publisher = rospy.Publisher(self.model.name + '/victim_marker', Int32, queue_size=1)
        self.explore_publisher = rospy.Publisher(self.model.name + '/explore/control', String, queue_size=1)

        self.update_widget()
        self.rgb_pub = None
        self.thermal_pub = None
        self.pose = None
        self.victim_detection = VictimDetection()
        self.victim_detection.start()

    def close(self):
        if self.rgb_pub is not None:
            self.rgb_pub.unregister()
        if self.thermal_pub is not None:
            self.thermal_pub.unregister()
        if self.cmd_publisher is not None:
            self.cmd_publisher.unregister()
        if self.victim_detection is None:
            self.victim_detection.stop()

    def connect(self):
        if self.model.rgb_camera_topic is not None:
            self.rgb_pub = rospy.Subscriber(self.model.rgb_camera_topic, Image, self.rgb_callback, queue_size=1)
        if self.model.thermal_camera_topic is not None:
            self.thermal_pub = rospy.Subscriber(self.model.thermal_camera_topic, Image, self.thermal_callback,
                                                queue_size=1)

        self.odom_sub = rospy.Subscriber(self.model.name + '/' + 'odom', Odometry, self.odom_callback,
                                         queue_size=1)

        self.cmd_publisher = rospy.Publisher(self.model.name + '/' + 'cmd_vel', Twist, queue_size=1)

    def manual_button_click(self):
        self.manual_control(True)

        for widget in shared.robot_widget_list:
            if widget is not self:
                if widget.state == RobotModel.IDLE or widget.state == RobotModel.MANUAL:
                    widget.joy_publisher.publish(String("stop"))
                    widget.state = RobotModel.IDLE
                    widget.setStyleSheet('background-color: rgb(211, 215, 207);')

    def manual_control(self, state):
        if state:
            shared.teleop_keyboard.set_publisher(self.cmd_publisher)

            self.joy_publisher.publish(String("start"))
            self.explore_publisher.publish(String("stop"))
            self.state = RobotModel.MANUAL
            self.setStyleSheet('background-color: rgb(115, 210, 22);')
        else:
            shared.teleop_keyboard.set_publisher(None)
            self.joy_publisher.publish(String("stop"))
            self.explore_publisher.publish(String("stop"))
            self.state = RobotModel.IDLE
            self.setStyleSheet('background-color: rgb(211, 215, 207);')

    def autonomous_control(self, state):
        if state:
            self.explore_publisher.publish(String("start"))
            self.setStyleSheet('background-color: rgb(40, 54, 253);')

            if self.state == RobotModel.MANUAL:
                shared.teleop_keyboard.set_publisher(None)
                self.joy_publisher.publish(String("stop"))

            self.state = RobotModel.AUTONOMOUS
        else:
            self.explore_publisher.publish(String("stop"))
            self.state = RobotModel.IDLE
            self.setStyleSheet('background-color: rgb(211, 215, 207);')

    def autonomous_button_click(self):
        self.autonomous_control(True)

    def alive_button_click(self):
        if self.victim_publisher is not None:
            self.victim_publisher.publish(1)
        if self.pose is not None:
            shared.alive_list.append([self.pose.position.x, self.pose.position.y])

    def dead_button_click(self):
        if self.victim_publisher is not None:
            self.victim_publisher.publish(2)

        if self.pose is not None:
            shared.dead_list.append([self.pose.position.x, self.pose.position.y])

    def update_widget(self):
        self.bot_name_label.setText(self.model.name)

    def odom_callback(self, odom_data):
        self.pose = odom_data.pose.pose  # the x,y,z pose and quaternion orientation

    def rgb_callback(self, data):
        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(data, 'rgb8')
            if self.victim_detection is not None:
                self.victim_detection.rgb_update(cv2_img)

                height, width = cv2_img.shape[:2]

                if len(self.victim_detection.victim_data) > 0:
                    for item in self.victim_detection.victim_data:
                        x = (item[1] * width) / 240
                        y = (item[2] * height) / 240
                        w = (item[3] * width) / 240
                        h = (item[4] * height) / 240
                        pv = item[5]
                        if 0.3 > item[0] > 0.13:
                            cv2.rectangle(cv2_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        elif 0.7 > item[0] > 0.3:
                            cv2.rectangle(cv2_img, (x, y), (x + w, y + h), (50, 122, 255), 2)
                        elif item[0] > 0.7:
                            cv2.rectangle(cv2_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        if pv <= 5:
                            status = "dead"
                            cv2.putText(cv2_img, status,
                                        (x, y + h), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                            if self.pose is not None:
                                if not check_list(shared.dead_list, self.pose):
                                    self.dead_button_click()
                        else:
                            status = "alive"
                            cv2.putText(cv2_img, status,
                                        (x, y + h), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                            if self.pose is not None:
                                if not check_list(shared.alive_list, self.pose):
                                    self.alive_button_click()

            image1 = qimage2ndarray.array2qimage(cv2_img)  # SOLUTION FOR MEMORY LEAK
            self.rgb_view.setPixmap(QPixmap.fromImage(image1.scaled(self.rgb_view.width(), self.rgb_view.height(),
                                                                    QtCore.Qt.KeepAspectRatio)))

        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def thermal_callback(self, data):
        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(data, 'mono8')
            self.victim_detection.thermal_update(cv2_img)

            image2 = qimage2ndarray.array2qimage(cv2_img)  # SOLUTION FOR MEMORY LEAK
            self.thermal_view.setPixmap(
                QPixmap.fromImage(image2.scaled(self.thermal_view.width(), self.thermal_view.height(),
                                                QtCore.Qt.KeepAspectRatio)))
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def load_ui(self):
        root_dir = QDir(os.path.dirname(os.path.realpath(__file__)))
        ui_path = str(root_dir.path()) + "/ui/robot_camera.ui"
        uic.loadUi(ui_path, self)
