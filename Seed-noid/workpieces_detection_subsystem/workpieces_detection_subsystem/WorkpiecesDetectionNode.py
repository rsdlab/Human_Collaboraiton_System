#!/usr/bin/env python3
# coding: UTF-8

import rclpy
import yaml

from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from collaboration_manipulation_message.msg import WorkDetectionResult
from collaboration_manipulation_message.srv import DetectWorkpieces
from rclpy.node import Node


class WorkDetect(Node):
    def __init__(self):
        super().__init__('work_detection_node')
        self.setup_req = self.create_service(DetectWorkpieces, 'detect_workpieces_service', self.pose_request)
        print("Initialization done")

    def pose_request(self, req_, res):
        # ワーク検知処理.
        # ここから.

        # リクエストの確認のみ
        print("")
        print("Request Data:")
        print(req_)
        print("")

        set_data = WorkDetectionResult()
        set_data.work_type_id = req_.work_type_id
        res.work_detection_results.work_detection_results.append(set_data)
        return res
        # ここまで.


def main():
    rclpy.init()
    wd = WorkDetect()
    rclpy.spin(wd)


if __name__ == "__main__":
    main()
