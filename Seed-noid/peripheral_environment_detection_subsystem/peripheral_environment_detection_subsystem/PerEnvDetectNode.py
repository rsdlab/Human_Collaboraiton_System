#!/usr/bin/env python3
# coding: UTF-8

import rclpy
import time
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import Point
from collaboration_manipulation_message.srv import SetMonitoringArea
from collaboration_manipulation_message.msg import MonitoringAreaList
from rclpy.node import Node


class AreaIntrusionDetect(Node):
    def __init__(self):
        super().__init__("peripheral_environment_detection_node")
        self.result_pub = self.create_publisher(Int16, '/intrusion_result', 10)
        self.length_sub = self.create_subscription(Float32, '/len_topic', self.UrgCallback, 10)
        self.setup_req = self.create_service(SetMonitoringArea, 'per_env_det_service', self.setup_request)
        self.setup_data = MonitoringAreaList()
        self.result_data = 0
        self.urg_data = Float32()
        self.basis_length = 0.3  # [m]

        print("Initialization done")

    def setup_request(self, req_, res):
        self.setup_data.monitoring_areas = req_.monitoring_areas
        if self.setup_data.monitoring_areas:
            print("Setup succeeded")
            res.response = 1
            print("Setup Data")
            print(self.setup_data)
            print("AreaIntrusionDetection start!")
        else:
            print("Setup fail")
            res.response = 0
        return res

    def UrgCallback(self, urg_):
        self.urg_data = urg_.data

    def mainloop(self):
        rate_hz = 10.0
        period = 1.0 / rate_hz
        while rclpy.ok():
            if self.setup_data.monitoring_areas:
                msg = Int16()
                msg.data = self.result_data
                self.result_pub.publish(msg)
                self.setup_data = MonitoringAreaList()

            time.sleep(period)


def main():
    rclpy.init()
    aid = AreaIntrusionDetect()
    aid.mainloop()


if __name__ == "__main__":
    main()
