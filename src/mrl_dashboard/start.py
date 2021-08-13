#!/usr/bin/env python
import rospy
from mrl_dashboard.main import app_start
import sys
import random

if __name__ == '__main__':
    rospy.init_node('mrl_dashboard_node')
    app_start()

