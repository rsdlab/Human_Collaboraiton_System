#!/usr/bin/env python3
# coding: UTF-8

import rclpy
import time
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Point


class WorkSpaceRecogniseNode(Node):
    def __init__(self):
        super().__init__('ws_recognise')
        self.pub = self.create_publisher(Float32, 'recg_data', 10)

    def send_ws_recognise(self):
        rate_hz = 10.0
        period = 1.0 / rate_hz
        while rclpy.ok():
            sensor = Float32()
            sensor.data = 1.0
            self.pub.publish(sensor)
            time.sleep(period)


def main():
    try:
        rclpy.init()
        node = WorkSpaceRecogniseNode()
        node.send_ws_recognise()
    except Exception as e:
        pass


if __name__ == '__main__':
    main()
