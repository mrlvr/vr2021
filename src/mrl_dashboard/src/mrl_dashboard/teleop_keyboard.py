#!/usr/bin/env python

import sys
import threading
from time import sleep

from input_listener import InputListener

if sys.version_info.major < 3:
    import glib
else:
    from gi.repository import GLib as glib

from geometry_msgs.msg import Twist

moveBindings = {
    'Up': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'Left': (0, 0, 0, 1),
    'Right': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'n': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    'Down': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'Prior': (1.1, 1.1),
    'Next': (.9, .9),
    'q': (1.1, 1),
    'a': (.9, 1),
    'w': (1, 1.1),
    's': (1, .9),
}


class TeleopKeyboard(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.speed = 0
        self.turn = 0
        self.status = 0
        self.pub = None
        self.loop = True

    def set_publisher(self, pub):
        self.pub = pub

    def callback(self, data):
        if self.pub:
            pressed = False
            repeated = False
            key = ''
            for k in dir(data):
                if k[0] == '_': continue
                if k == 'pressed':
                    pressed = True
                if k == 'repeated':
                    repeated = True
                if k == 'symbol':
                    key = str(getattr(data, k))

            if pressed and repeated and len(key) > 0:
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.y = moveBindings[key][1]
                    self.z = moveBindings[key][2]
                    self.th = moveBindings[key][3]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]

                    if self.speed > 1:
                        self.speed = 1
                    if self.speed < 0:
                        self.speed = 0
                    if self.turn > 1:
                        self.turn = 1
                    if self.turn < 0:
                        self.turn = 0
                else:
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.th = 0

                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = self.th * self.turn

                self.pub.publish(twist)

    def run(self):
        self.speed = 0.5
        self.turn = 0.5
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0

        glib.threads_init()
        self.kl = InputListener(self.callback)
        try:
            # keep running only while the listener is alive
            self.kl.start()

            while self.kl.is_alive() and self.loop:
                sleep(1)

        except KeyboardInterrupt:
            pass

        # check if the thread terminated unexpectedly
        if self.kl.is_alive():
            self.kl.stop()
            self.kl.join()
        elif self.kl.error:
            print("initialization error: {}".format(self.kl.error))
            if '__traceback__' in dir(self.kl.error):
                import traceback

                traceback.print_tb(self.kl.error.__traceback__)

            exit(1)

    def stop(self):
        self.loop = False
        self.kl.stop()
        self.kl.join(1000)
        self.join(1000)


