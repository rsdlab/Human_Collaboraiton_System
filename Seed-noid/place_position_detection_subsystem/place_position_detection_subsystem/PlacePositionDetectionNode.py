#!/usr/bin/env python3
# coding: UTF-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from collaboration_manipulation_message.srv import DetectPlacePosition


class Place_PositionDetectNode(Node):
    def __init__(self):
        super().__init__('place_position_detect_node')

    def response_data(self, req, res):
        print(req)
        res.place_position_detection_result.task_command_id = 1

        # リストを受信する.

        # 排出位置候補から排出場所決める.

        print(res)
        return res

    def dis_pos_detect_server(self):
        self.create_service(DetectPlacePosition, 'place_position_detect_service', self.response_data)


def main():
    rclpy.init()
    node = Place_PositionDetectNode()
    node.dis_pos_detect_server()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
