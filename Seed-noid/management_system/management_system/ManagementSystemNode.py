#!/usr/bin/env python3
# coding: UTF-8

#####################################################################################################################
#このノードは，上位アプリを実装するために作成したコードです．
#===================================================================================================================#
#バージョン管理
#===================================================================================================================#
#ver. 0.1:  基本実装（Linux版）　　　2023/06/28
#10/05 test_server.pyと単体検証を行うために，クラス部分をコメントアウト→統合試験でも成功
#ver. 1.0:  ROS2 Jazzy対応
#===================================================================================================================#
#依存ノード
#===================================================================================================================#
#このノードはLinuxでのみ利用可能です．
#===================================================================================================================#

from abc import ABCMeta
from abc import abstractmethod
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from collaboration_manipulation_message.msg import TargetArea, TaskCommandList, TaskCommand, PlaceAreaCandidatesList
from collaboration_manipulation_message.srv import SendTaskCommand, TerminateSystem
from collaboration_manipulation_message.srv import NotifyTaskResult, NotifyTaskCompletion, MoveCommunication
from std_msgs.msg import Empty
import sys
import time
import threading
import math

#TestMover
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import String


PICK_COMMAND = 0
DOWN_COMMAND = 1
PLACE_COMMAND = 2
RELEASE_COMMAND = 3
SOLVE_COMMAND = 4
APPROACH_COMMAND = 5

# ROS2パッケージとしてインポート
from collaboration_manipulation_module.EnumerateModule import EnumCommandReceiveState

from waypoint_management.waypoint_management_node import *

# グローバルノード・エグゼキュータ
_node: Node = None
_executor: MultiThreadedExecutor = None
_callback_group: ReentrantCallbackGroup = None


def get_node() -> Node:
    return _node


def get_callback_group() -> ReentrantCallbackGroup:
    return _callback_group


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
        self.pub = get_node().create_publisher(class_type, topic_name, 10)

    def execute(self, data):
        self.pub.publish(data)

class SystemManagementClient(SystemManagementBase):
    def __init__(self, service_name, service_class):
        self.proxy = get_node().create_client(
            service_class, service_name, callback_group=get_callback_group())

    def execute(self, data):
        try:
            while not self.proxy.wait_for_service(timeout_sec=1.0):
                get_node().get_logger().warn('Service not available, waiting: ' + str(self.proxy.srv_name))
            future = self.proxy.call_async(data)
            while not future.done():
                time.sleep(0.01)
            result = future.result()
            print(result)
            return result
        except Exception as e:
            get_node().get_logger().info("ServiceException : %s" % e)
            return False

class SystemManagementServer(SystemManagementBase):
    def __init__(self, service_name, service_class, callback_impl):
        self.server = get_node().create_service(
            service_class, service_name, callback_impl,
            callback_group=get_callback_group())

    def service_delete(self):
        get_node().destroy_service(self.server)

    def execute(self, object):
        pass

#作業結果コマンド受信クラス.
class TaskResultServer(SystemManagementServer):
    def __init__(self):
        super().__init__('notify_task_result', NotifyTaskResult, self.recv_task_result)
        self.task_failed = 0

    def service_delete(self):
        super().service_delete()

    #作業結果コマンド受信.
    def recv_task_result(self, request, response):
        print("TaskResult:", request.task_result.task_result)
        if(False == request.task_result.task_result):
            self.task_failed = 1

        response.return_code = 1
        return response

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
        super().service_delete()

    #作業完了コマンド受信.
    def recv_task_complete(self, _, response):
        print("TaskComplete")
        self.task_complete = 1
        response.return_code = 1
        return response

    def get_task_complete(self):
        return self.task_complete

    def reset_task_complete(self):
        self.task_complete = 0

#システム終了指令.
class TerminateSystemClient(SystemManagementClient):
    def __init__(self,):
        super().__init__('terminate_system', TerminateSystem)

    def execute(self):
        msg = TerminateSystem.Request()
        msg.empty = Empty()
        super().execute(msg)

#作業開始指令.
class SendTaskCommandClient(SystemManagementClient):
    def __init__(self):
        super().__init__('send_command_service', SendTaskCommand)

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
        srv = SendTaskCommand.Request()
        srv.task_commands = set_task_command_list
        ret = super().execute(srv)
        print('reslt ', ret)
        return ret

def main_callback(_, response):
    _run_task()
    return response

def communication():
    get_node().create_service(
        MoveCommunication, 'move_seed_noid', main_callback,
        callback_group=get_callback_group())
    _executor.spin()

def halt():
    cmd = TerminateSystemClient()
    #終了処理
    print("halt")
    time.sleep(0.1)
    cmd.execute()
    time.sleep(0.1)

def _run_task():
    cmd = SendTaskCommandClient()
    result = TaskResultServer()
    complete = TaskCompleteServer()
    waypoint_manager = WaypointManagementNode(get_node())

    #ピック

    print("pick")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(PICK_COMMAND).response):
        print("command_retry")
        time.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        result.service_delete()
        complete.service_delete()
        return Empty()

     #障害物検知外の位置まで後退
    waypoint_manager.back_off_if_needed(10.0)

    #ダウン
    print("down")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(DOWN_COMMAND).response):
        print("command_retry")
        time.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        result.service_delete()
        complete.service_delete()
        return Empty()

    # 旋回(180°)
    waypoint_manager.rotate_in_place(180, speed=0.3)
    time.sleep(1)

    # 机までの移動
    waypoint_manager.move_to_goal(message_id="move_to_table", x=-2.67, y=0.06, theta=1.57)
    time.sleep(1)

    # 机に接近
    waypoint_manager.adjust_pose_if_needed(16.0)

    #リリース
    print("Place")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(PLACE_COMMAND).response):
        print("command_retry")
        time.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        result.service_delete()
        complete.service_delete()
        return Empty()

    time.sleep(1)
    waypoint_manager.back_off_if_needed(8.0)

    waypoint_manager.rotate_in_place(-90, speed=0.3)  # 右旋回

    time.sleep(1)
    waypoint_manager.move_to_goal(message_id="move_to_table", x=-0.86, y=0.07, theta=0.00)
    time.sleep(1)
    waypoint_manager.adjust_pose_if_needed(17.0)

    result.service_delete()
    complete.service_delete()
    return Empty()

def wait(result:TaskResultServer, comp:TaskCompleteServer):
    while ((False == result.get_task_failed()) and (False == comp.get_task_complete()) and rclpy.ok()):
        time.sleep(0.05)
    return

def release(data = None):
    cmd = SendTaskCommandClient()
    result = TaskResultServer()
    complete = TaskCompleteServer()

    #ピック
    print("release")
    result.reset_task_result()
    complete.reset_task_complete()
    #送信完了まで繰り返す.
    while (EnumCommandReceiveState.e_received() != cmd.execute(RELEASE_COMMAND).response):
        print("command_retry")
        time.sleep(1)

    #動作完了まで待つ.
    wait(result, complete)
    if(True == result.get_task_failed()):
        result.service_delete()
        complete.service_delete()
        return Empty()

    result.service_delete()
    complete.service_delete()
    return Empty()


def main(args=None):
    global _node, _callback_group, _executor
    rclpy.init(args=args)
    _node = Node("system_management_node")
    _callback_group = ReentrantCallbackGroup()
    _executor = MultiThreadedExecutor()
    _executor.add_node(_node)

    argv = sys.argv
    print("args:", argv)
    try:
        if 2 == len(argv):
            #システム終了指令.
            if 'halt' == argv[1]:
                _executor_thread = threading.Thread(target=_executor.spin, daemon=True)
                _executor_thread.start()
                halt()
            elif 'comm' == argv[1]:
                # commモードはexecutor.spin()をブロッキングで使用
                communication()
            elif 'release' == argv[1]:
                _executor_thread = threading.Thread(target=_executor.spin, daemon=True)
                _executor_thread.start()
                release()
            else:
                _executor_thread = threading.Thread(target=_executor.spin, daemon=True)
                _executor_thread.start()
                _run_task()
        else:
            _executor_thread = threading.Thread(target=_executor.spin, daemon=True)
            _executor_thread.start()
            _run_task()
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
