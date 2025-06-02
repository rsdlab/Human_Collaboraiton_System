#!/usr/bin/env python3
# coding: UTF-8

#####################################################################################################################
#このノードは，上位アプリを実装するために作成したコードです．　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　 
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#10/05 test_server.pyと単体検証を行うために，クラス部分をコメントアウト→統合試験でも成功
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

from abc import ABCMeta
from abc import abstractmethod
import rospy
from collaboration_manipulation_message.msg import TargetArea, TaskCommandList, TaskCommand, PlaceAreaCandidatesList
from collaboration_manipulation_module.srv import SendTaskCommand, SendTaskCommandRequest
from collaboration_manipulation_module.srv import TerminateSystem, TerminateSystemRequest
from management_system.srv import *
from std_msgs.msg import Empty
import sys
import os

#TestMover
import time
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import String

PICK_COMMAND = 0
DOWN_COMMAND = 1
PLACE_COMMAND = 2

# 相対パスでhuman_collaboration/scriptsのパスを追加
sys.path.append(os.path.join(os.path.dirname(__file__), '../../collaboration_manipulation_module/scripts'))
# インポート
from EnumerateModule import EnumCommandReceiveState

class SystemManagementBase(metaclass=ABCMeta):
    """
    Base class that SystemManagement communication
    Define machine-dependent packages
    """
    @abstractmethod
    def execute(self, data):
        pass


class SystemManagementPublisher(SystemManagementBase):
    def __init__(self, topic_name, class_type):
        self.pub = rospy.Publisher(topic_name, class_type, queue_size = 10)

    def execute(self, data):
        self.pub.publish(data)

class SystemManagementClient(SystemManagementBase):
    def __init__(self, service_name, service_class):
        self.proxy = rospy.ServiceProxy(service_name, service_class)

    def execute(self, data):
        try :
            result = self.proxy(data)
            print(result)
            return result
        except rospy.ServiceException as e:
            rospy.loginfo("ServiceException : %s" % e)
            return False

class SystemManagementServer(SystemManagementBase):
    def __init__(self, service_name, service_class, callback_impl):
        self.server = rospy.Service(service_name, service_class, callback_impl)

    def service_delete(self, msg=''):
        self.server.shutdown(msg)

    def execute(self, object):
        pass

#作業結果コマンド受信クラス.
class TaskResultServer(SystemManagementServer):
    def __init__(self):
        super().__init__('notify_task_result', NotifyTaskResult, self.recv_task_result)
        self.task_failed = 0

    def service_delete(self):
        super().service_delete('notify_task_result deleted')

    #作業結果コマンド受信.
    def recv_task_result(self, data):
        print("TaskResult:", data.task_result.task_result)
        if(False == data.task_result.task_result):
            self.task_failed = 1

        return_value = NotifyTaskResultResponse()
        return_value.return_code = 1
        return return_value
    
    def get_task_failed(self):
        return self.task_failed
    
    def reset_task_result(self):
        self.task_failed = 0

#作業完了コマンド受信クラス.
class TaskCompleteServer(SystemManagementServer):
    def __init__(self):
        super().__init__('notify_task_completion', NotifyTaskCompletion, self.recv_task_complete)
        self.task_complete = 0

    def service_delete(self):
        super().service_delete('notify_task_completion deleted')

    #作業完了コマンド受信.
    def recv_task_complete(self, data):
        print("TaskComplete")
        self.task_complete = 1
        return_value = NotifyTaskCompletionResponse()
        return_value.return_code = 1
        return return_value
    
    def get_task_complete(self):
        return self.task_complete
    
    def reset_task_complete(self):
        self.task_complete = 0

#システム終了指令.
class TerminateSystemClient(SystemManagementClient):
    def __init__(self,):
        super().__init__('terminate_system', TerminateSystem)

    def execute(self):
        msg = TerminateSystemRequest()
        msg.empty = Empty()
        super().execute(msg)

#作業開始指令.
class SendTaskCommandClient(SystemManagementClient):
    def __init__(self):
        super().__init__('sned_command_service', SendTaskCommand)

    def execute(self, command_id):
        print('{} start'.format(self.__class__.__name__))

        ###作業コマンド作成 ここから.

        #作業コマンドリスト作成.
        set_task_command_list = TaskCommandList()
        set_task_command = TaskCommand()
        set_task_command.task_command_id = command_id
        set_task_command.work_type_id = 1
        set_task_command.picking_count= 1
        
        #ワーク検知エリア設定.
        set_task_command.work_presence_area = TargetArea()
        set_task_command.work_presence_area.start_point.x = -2.0
        set_task_command.work_presence_area.start_point.y = -2.0
        set_task_command.work_presence_area.start_point.z = 0.0
        set_task_command.work_presence_area.end_point.x = 2.0
        set_task_command.work_presence_area.end_point.y = 2.0
        set_task_command.work_presence_area.end_point.z = 2.0

        #廃棄候補エリアリスト作成.
        set_task_command.place_area_candidates = PlaceAreaCandidatesList()

        #廃棄候補エリア作成.
        dis_area = TargetArea()
        dis_area.start_point.x = -2.0
        dis_area.start_point.y = -2.0
        dis_area.start_point.z = 0.0
        dis_area.end_point.x = 2.0
        dis_area.end_point.y = 2.0
        dis_area.end_point.z = 2.0
        set_task_command.place_area_candidates.place_area_candidates.append(dis_area)

        print(set_task_command)
        set_task_command_list.task_commands.append(set_task_command)

        #ここまで.
        srv = SendTaskCommandRequest()
        srv.task_commands = set_task_command_list
        ret = super().execute(srv)
        print('reslt ', ret)
        return ret

class TestMover():
    def __init__(self):
        ################################### 初期値設定 ####################################

        self.rate = rospy.Rate(10)  # 10hz

        ########## PI制御のパラメータ初期化（偏差、時間、偏差の積分） ##########
        self.e_pre_x = 0
        self.T_pre_x = time.time()
        self.ie_x = 0

        self.e_pre_y = 0
        self.T_pre_y = time.time()
        self.ie_y = 0

        self.e_pre_z = 0
        self.T_pre_z = time.time()
        self.ie_z = 0

        ########## 状態遷移のための変数 ##########
        # 連続して同じ処理をしないための変数（/signをsubscribeした時）
        self.init = ''

        # コマンドによる割り込み処理用の変数
        self.cancel = False # True:マーカー補正可能 False:マーカー補正不可（コマンド処理実行時）

        # フィードバック用の変数（/cmd_velをpublishした時）
        self.bool = True # True:成功 False:失敗

        ########## マーカー関連の変数 ##########
        # マーカー補正したときのマーカーID（後退する量を決めるための変数）
        self.pre_id = 0

        # マーカー補正を繰り返ししないための変数（マーカー補正終了時）
        self.adjust = False

        # yaw→x→yの順にマーカー補正するための変数
        self.void_x = 0
        self.void_y = 0
        self.void_z = 0

        # マーカーの2段階補正を切り替えるための変数
        self.change = False

        # 姿勢分布中心の計算のための変数
        self.elr_ttl = 0.0
        self.cnt_z = 0
        self.centerdeg_z = 0.0

        ##########################################################################

        ########################### パラメーター設定 ###############################

        ########## 台形加速 ##########
        # 目標速度
        self.tar_vel_x = 0.3 # [m]RECEIVED
        self.acc_time_x = 1 # [s]
        self.acc_time_y = 1

        # cmd_velのpublish
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        #######################################################################

    # 前後移動速度指定付き
    def pub_x_vel(self, data, vel):
        self.tar_vel_x = vel
        self.pub_x(data)
        self.tar_vel_x = 0.3

    # 前後移動
    def pub_x(self, data):
        # 目的の距離と速度を設定
        dist_x = data # [m]
        
        # Twist 型のデータ
        t = Twist()
        t.angular.z = 0

        # 等速時間の計算
        tar_time = (abs(dist_x) - self.tar_vel_x) / self.tar_vel_x

        # 速度の向き設定
        if (dist_x > 0):
            self.tar_vel_x = self.tar_vel_x
        if (dist_x < 0):
            self.tar_vel_x = -self.tar_vel_x

        # 開始の時刻を保存
        start_time = time.time()
        # 経過した時刻を取得
        end_time = time.time()

        # 等速時間があるとき
        if (tar_time >= 0):
            # 加速
            while (end_time - start_time <= self.acc_time_x):
                t.linear.x = (self.tar_vel_x / self.acc_time_x) * (end_time - start_time)
                self.cmd_pub.publish(t)
                end_time = time.time()

            # 等速
            while (self.acc_time_x <= end_time - start_time <= tar_time + self.acc_time_x):
                t.linear.x = self.tar_vel_x
                self.cmd_pub.publish(t)
                end_time = time.time()

            # 減速
            while (end_time - start_time <= tar_time + (self.acc_time_x + self.acc_time_x)):
                t.linear.x = (self.tar_vel_x / self.acc_time_x) * ((tar_time + (self.acc_time_x + self.acc_time_x)) - (end_time - start_time))
                self.cmd_pub.publish(t)
                end_time = time.time()
        
        # 等速時間がないとき
        if (tar_time < 0):
            acc_time = dist_x / (self.tar_vel_x / self.acc_time_x)
            # 加速
            while (end_time - start_time <= acc_time):
                t.linear.x = (self.tar_vel_x / self.acc_time_x) * (end_time - start_time)
                self.cmd_pub.publish(t)
                end_time = time.time()

            # 減速
            while (end_time - start_time <= acc_time + acc_time):
                t.linear.x = (self.tar_vel_x / self.acc_time_x) * ((acc_time + acc_time) - (end_time - start_time))
                self.cmd_pub.publish(t)
                end_time = time.time()

        self.tar_vel_x = abs(self.tar_vel_x)
        t.linear.x = 0

    def rotate(self, angle, speed, duration):
        vel_msg = Twist()
        
        # Set the angular velocity in the z-axis
        vel_msg.angular.z = speed

        # Rotate for a certain duration to achieve the desired angle
        end_time = rospy.Time.now() + rospy.Duration(duration)
        
        while rospy.Time.now() < end_time:
            self.cmd_pub.publish(vel_msg)
            self.rate.sleep()

        # Stop the robot after rotation
        vel_msg.angular.z = 0
        self.cmd_pub.publish(vel_msg)

def halt():
    cmd = TerminateSystemClient()
    #終了処理
    print("halt")
    rospy.sleep(0.1)
    cmd.execute()
    rospy.sleep(0.1)

def main(data = None):
    cmd = SendTaskCommandClient()
    result = TaskResultServer()
    complete = TaskCompleteServer()
    testmover = TestMover()

    #ピック
    print("pick")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(PICK_COMMAND).response):
        print("command_retry")
        rospy.sleep(1)
    
    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        result.service_delete()
        complete.service_delete()
        return Empty()

     #机までの移動
    testmover.pub_x_vel(-0.4, 0.2)

    #ダウン
    print("down")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(DOWN_COMMAND).response):
        print("command_retry")
        rospy.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        result.service_delete()
        complete.service_delete()
        return Empty()

    rospy.sleep(1)
    testmover.pub_x_vel(-0.8, 0.2)
    rospy.sleep(1)
    testmover.rotate(1.57, 0.2, (5.23)*1.51)
    rospy.sleep(1)
    testmover.pub_x_vel(1.97, 0.2)
    rospy.sleep(1)
    testmover.rotate(-1.57, -0.2, (5.23)*1.51)
    rospy.sleep(1)
    testmover.pub_x_vel(0.2551, 0.2)

    #リリース
    print("Place")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(PLACE_COMMAND).response):
        print("command_retry")
        rospy.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        result.service_delete()
        complete.service_delete()
        return Empty()

    #戻る
    testmover.pub_x_vel(-0.2551, 0.3)
    testmover.rotate(-1.57, -0.3, 5.23)
    testmover.pub_x_vel(1.975, 0.3)
    testmover.rotate(1.57, 0.3, 5.23)
    testmover.pub_x_vel(1.250, 0.3)
    
    result.service_delete()
    complete.service_delete()
    return Empty()

def wait(result:TaskResultServer, comp:TaskCompleteServer):
    while ((False == result.get_task_failed()) and (False == comp.get_task_complete()) and (not rospy.is_shutdown())):
        rospy.sleep(0.05)
    return 


if __name__ == '__main__':
    rospy.init_node("system_management_node")
    args = sys.argv
    print("args:", args)
    try:
        if 2 == len(args):
            #システム終了指令.
            if 'halt' == args[1]:
                halt()
            else:
                main()
        else:
            main()
    except rospy.ROSInterruptException:
        pass