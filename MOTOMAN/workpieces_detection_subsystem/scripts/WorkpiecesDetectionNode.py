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
        
        # リクエストの確認
        print("")
        print("Request Data:")
        print(req_)
        print("")
        print("Arco Data")
        print(self.aruco_data)
        print("")

        rospack = rospkg.RosPack()
        rospack.list() 
        srv = DetectWorkpiecesResponse()
        set_data = WorkDetectionResult()
        if req_.task_command_id == 1:
            if req_.work_type_id == 1:
                path = rospack.get_path('workpieces_detection_subsystem')
                with open(path + '/config/sandwich_id.yaml') as f:
                    config = yaml.safe_load(f)

            if req_.work_type_id == 2:
                path = rospack.get_path('workpieces_detection_subsystem')
                with open(path + '/config/drink_id.yaml') as f:
                    config = yaml.safe_load(f)

            #該当データ検索.
            self.SerchData(config, req_, set_data)
            print(set_data.pose)
            set_data.work_type_id = req_.work_type_id
            srv.work_detection_results.work_detection_results.append(set_data)
            print(srv.work_detection_results.work_detection_results)
        elif req_.task_command_id == 2:
            # 未実装
            set_data.work_type_id = req_.work_type_id
            srv.work_detection_results.work_detection_results.append(set_data)
        else:
            # 定義されていない作業指令IDの場合
            set_data.work_type_id = req_.work_type_id
            srv.work_detection_results.work_detection_results.append(set_data)  
        return srv
        #ここまで.
    
    #範囲内のワーク検索.
    def SerchData(self, config, req, set_data):
        if config and self.aruco_data:
                for i in range(len(self.aruco_data)):
                    if self.aruco_data[i].id == config[i]:
                        if req.target_area.start_point.x < self.aruco_data[i].pose.pose.position.x < req.target_area.end_point.x and\
                           req.target_area.start_point.y < self.aruco_data[i].pose.pose.position.y < req.target_area.end_point.y and\
                           req.target_area.start_point.z < self.aruco_data[i].pose.pose.position.z < req.target_area.end_point.z:
                                set_data.pose.position.x = self.aruco_data[i].pose.pose.position.x
                                set_data.pose.position.y = self.aruco_data[i].pose.pose.position.y
                                set_data.pose.position.z = self.aruco_data[i].pose.pose.position.z
                                set_data.pose.orientation.x = self.aruco_data[i].pose.pose.orientation.x
                                set_data.pose.orientation.y = self.aruco_data[i].pose.pose.orientation.y
                                set_data.pose.orientation.z = self.aruco_data[i].pose.pose.orientation.z
                                set_data.pose.orientation.w = self.aruco_data[i].pose.pose.orientation.w
                                print(set_data.pose)
                                return None

if __name__ == "__main__":
    rospy.init_node("work_detection_node")
    wd = WorkDetect()
    rospy.spin()
  
