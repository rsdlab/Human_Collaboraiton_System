#!/usr/bin/env python3
# coding: UTF-8

import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time as RosTime


class CollaborationTool(Node):
    classnode = None

    def __init__(self):
        super().__init__('tool_node')

    @classmethod
    def ros_init(cls):
        rclpy.init()
        if cls.classnode is None:
            cls.classnode = CollaborationTool()

    @classmethod
    def signal_shutdown(cls, reason):
        if cls.classnode is None:
            cls.classnode = CollaborationTool()
        cls.classnode.get_logger().info(f'Shutdown reason: {reason}')
        rclpy.shutdown()

    @classmethod
    def loginfo(cls, message, *args, **kwargs):
        if(None == cls.classnode):
            cls.classnode = CollaborationTool()
        cls.classnode.get_logger().info(str(message), *args, **kwargs)

    @classmethod
    def logwarn(cls, message, *args, **kwargs):
        if(None == cls.classnode):
            cls.classnode = CollaborationTool()
        cls.classnode.get_logger().warn(str(message), *args, **kwargs)

    @classmethod
    def wait_time(cls, time_second):
        time.sleep(time_second)

    @classmethod
    def create_time(cls, time):
        return RosTime(seconds=time)

    @classmethod
    def create_duration(cls, num):
        return Duration(seconds=num)
    
    @classmethod
    def is_shutdown(cls):
        return not rclpy.ok()