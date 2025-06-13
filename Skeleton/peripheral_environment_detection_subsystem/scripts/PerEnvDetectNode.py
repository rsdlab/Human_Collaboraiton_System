#!/usr/bin/env python3
# coding: UTF-8

#####################################################################################################################
#このノードは，周辺環境計測モジュールを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

#python,TFのライブラリを使用
import rospy
from rospy.topics import Publisher, Subscriber

from std_msgs.msg import *
from geometry_msgs.msg import *
from peripheral_environment_detection_subsystem.srv import *
from collaboration_manipulation_message.msg import MonitoringAreaList

class AreaIntrusionDetect:
    def __init__(self):
        self.result_pub = Publisher('/intrusion_result', int, queue_size=10)
        self.length_sub = Subscriber('/len_topic', Float32, self.UrgCallback)
        self.setup_req = rospy.Service('per_env_det_service', SetMonitoringArea, self.setup_request)
        self.setup_data = MonitoringAreaList()
        self.result_data = 0
        self.urg_data = Float32()
        self.basis_length = 0.3 #[m]

        print("Initialization done")

    def setup_request(self, req_):
        srv = SetMonitoringAreaResponse()
        self.setup_data.monitoring_areas = req_.monitoring_areas
        if self.setup_data.monitoring_areas:
            print("Setup succeeded")
            srv.response = 1
            print("Setup Data")
            print(self.setup_data)
            print("AreaIntrusionDetection start!")
        else:
            print("Setup fail")
            srv.response= 0
        return srv

    def UrgCallback(self, urg_):
        self.urg_data = urg_.data

    def mainloop(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.setup_data.monitoring_areas:
                self.result_pub.publish(self.result_data)
                self.setup_data = MonitoringAreaList()
                #TODO サブスクライバー作成.

            r.sleep()

if __name__ == "__main__":
    rospy.init_node("peripheral_environment_detection_node")
    aid = AreaIntrusionDetect()
    aid.mainloop()
    
