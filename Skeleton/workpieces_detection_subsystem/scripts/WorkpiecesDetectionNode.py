#!/usr/bin/env python3
# coding: UTF-8
#####################################################################################################################
#このノードは，ワーク検出モジュールを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

import rospy
import yaml
import rospkg
from rospy.topics import Subscriber

from std_msgs.msg import *
from geometry_msgs.msg import *
from aruco_msgs.msg import *
from collaboration_manipulation_message.msg import WorkDetectionResult
from workpieces_detection_subsystem.srv import *

class WorkDetect:
    def __init__(self):
        self.length_sub = Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.ArucoCallback)
        self.setup_req = rospy.Service('detect_workpieces_service', DetectWorkpieces, self.pose_request)
        self.aruco_data = MarkerArray()
        print("Initialization done")

    def ArucoCallback(self, aruco_):
        self.aruco_data = aruco_.markers

    def pose_request(self, req_):
        #ワーク検知処理.
        #ここから.
        
        return None
        #ここまで.

if __name__ == "__main__":
    rospy.init_node("work_detection_node")
    wd = WorkDetect()
    rospy.spin()