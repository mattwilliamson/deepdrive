#!/usr/bin/env python

# Copyright (C) 2012 Jon Stephan.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import sys
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import inspect

from PyQt5 import QtWidgets, QtCore


class NodeMixin(Node):
    def __init__(self, parent=None):
        super(NodeMixin, self).__init__('virtual_joystick')


class MainWindow(NodeMixin, QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        super(QtWidgets.QMainWindow, self).__init__(*args, **kwargs)

        self.get_logger().info("virtual_joystick")

        self.x_min = self.declare_parameter("x_min", -0.20).value
        self.x_max = self.declare_parameter("x_max", 0.20).value
        self.r_min = self.declare_parameter("r_min", -1.0).value
        self.r_max = self.declare_parameter("r_max", 1.0).value

        self.timer_rate = self.declare_parameter('publish_rate', 50).value
        self.twist_publisher = self.create_publisher(Twist, 'twist', 10)

        self.init_ui()

    def init_ui(self):
        img_path = os.path.dirname(
            os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/../../images/crosshair.jpg"
        self.get_logger().info("init_ui img_path: %s" % img_path)

        self.statusBar()

        self.setStyleSheet("QMainWindow { border-image: url(%s); }" % img_path)

        self.setGeometry(0, 600, 200, 200)
        self.setWindowTitle('Virtual Joystick')
        self.show()
        self.timer = QtCore.QBasicTimer()

        self.statusBar().showMessage('started')

    def mousePressEvent(self, event):
        self.statusBar().showMessage('mouse clicked')
        self.timer.start(self.timer_rate, self)
        self.get_position(event)

    def mouseReleaseEvent(self, event):
        self.statusBar().showMessage('mouse released')
        self.timer.stop()

    def mouseMoveEvent(self, event):
        self.get_position(event)

    def get_position(self, event):
        s = self.size()
        s_w = s.width()
        s_h = s.height()
        pos = event.pos()
        self.x = 1.0 * pos.x() / s_w
        self.y = 1.0 * pos.y() / s_h

        self.statusBar().showMessage('point (%0.2f, %0.2f)' % (self.x, self.y))

    def timerEvent(self, event):
        # self.statusBar().showMessage("timer tick")
        self.pub_twist()

    def pub_twist(self):
        self.get_logger().info("publishing twist from (%0.3f,%0.3f)" % (self.x, self.y))
        self.twist = Twist()
        self.twist.linear.x = (1 - self.y) * (self.x_max - self.x_min) + self.x_min
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = (1 - self.x) * (self.r_max - self.r_min) + self.r_min

        if self.twist.linear.x > self.x_max:
            self.twist.linear.x = self.x_max
        if self.twist.linear.x < self.x_min:
            self.twist.linear.x = self.x_min
        if self.twist.angular.z > self.r_max:
            self.twist.angular.z = self.r_max
        if self.twist.angular.z < self.r_min:
            self.twist.angular.z = self.r_min

        self.twist_publisher.publish(self.twist)


def main(args=None):
    try:
        rclpy.init(args=args)

        app = QtWidgets.QApplication(sys.argv)
        ex = MainWindow()
        ex.show()
        sys.exit(app.exec_())

    except rclpy.exceptions.ROSInterruptException:
        pass

    ex.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
