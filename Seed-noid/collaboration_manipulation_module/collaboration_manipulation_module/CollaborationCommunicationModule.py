#!/usr/bin/env python3
# coding: UTF-8

import threading
import time
from abc import ABCMeta, abstractmethod

import rclpy
from collaboration_manipulation_message.msg import (
    MonitoringArea,
    MonitoringAreaList,
    TargetArea,
    WorkDetectionResultList,
)
from collaboration_manipulation_message.srv import (
    CancelTask,
    DetectPlacePosition,
    DetectWorkpieces,
    FinishTask,
    GetStatus,
    NotifyTaskCompletion,
    NotifyTaskResult,
    PauseTask,
    SendTaskCommand,
    SetMonitoringArea,
    TerminateSystem,
)
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

from .CollaborationEventModule import (
    CollaborationCandidatePlaceLocationArea,
    CollaborationEventPublisher,
    CollaborationPlaceAreaCandidatesList,
    CollaborationTaskCommand,
    CollaborationTaskCommandList,
    CollaborationWorkPresenceArea,
)
from .CollaborationToolModule import CollaborationTool
from .EnumerateModule import EnumCommandReceiveState, EnumEvent, EnumState


#協働通信クラスの抽象クラス.
class CollaborationCommunication(Node, metaclass=ABCMeta):
    """
    Base class that HumanCollaboration communication
    Define machine-dependent packages
    """
    def __init__(self, node_name):
        super().__init__(node_name)

    @abstractmethod
    def execute(self, object):
        pass

#協働パブリッシャークラスの抽象クラス.
class CollaborationPublisher(CollaborationCommunication):
    """
    Base class that HumanCollaboration communication for Publisher
    Define machine-dependent packages
    """
    def __init__(self, topic_name, datatype, node_name):
        super().__init__(node_name)
        self.pub = self.create_publisher(datatype, topic_name, 10)

    def execute(self, object):
        pass

#協働サブスクライバークラスの抽象クラス.
class CollaborationSubscriver(CollaborationCommunication):
    """
    Base class that HumanCollaboration communication for Subscriver
    Define machine-dependent packages
    """
    def __init__(self, topic_name, datatype, callback, node_name):
        super().__init__(node_name)
        self.sub = self.create_subscription(datatype, topic_name, callback, 10)
    def execute(self, object):
        pass

#協働クライアントクラスの抽象クラス.
class CollaborationClient(CollaborationCommunication):
    """
    Base class that HumanCollaboration communication for request
    Define machine-dependent packages
    """
    def __init__(self, service_name, service_class, node_name):
        super().__init__(node_name)
        self.proxy = self.create_client(service_class, service_name)

    def wait_for_service(self, timeout_sec = None):
        self.proxy.wait_for_service(timeout_sec)

    def execute(self, object):
        pass

#協働サーバークラスの抽象クラス.
class CollaborationServer(CollaborationCommunication):
    """
    Base class that HumanCollaboration communication for response
    Define machine-dependent packages
    """
    def __init__(self, service_name, service_class, callback_impl, node_name):
        super().__init__(node_name)
        self.server = self.create_service(service_class, service_name, callback_impl)

    def service_delete(self, msg = None):
        self.server.destroy()

    def execute(self, object):
        pass

    def wait(self, time_count, condition):
        self.condition = condition
        while self.condition:
            time.sleep(time_count)

    def set_wait_condition(self, condition):
        self.condition = condition

#システム終了コマンド受信クラス.
class SystemTerminate(CollaborationServer):
    def __init__(self):
        super().__init__('terminate_system', TerminateSystem, self.receive_system_halt, 'terminate_system_server_node')
        CollaborationTool.loginfo('{} start '.format(self.__class__.__name__))

    #システム終了コマンド受信.
    def receive_system_halt(self, data, res):
        CollaborationTool.loginfo('receive_terminate_system {}'.format(data))
        CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_end()), data)
        res.return_code = True
        return res

    def service_delete(self):
        super().service_delete('terminate_system deleted')

    def execute(self, object):
        pass

#システム一時停止コマンド受信クラス.
class SystemPause(CollaborationServer):
    def __init__(self):
        super().__init__('pause_system', PauseTask, self.receive_system_pause, 'pause_system_server_node')
        CollaborationTool.loginfo('{} start '.format(self.__class__.__name__))
    
    #システム一時停止コマンド受信.
    def receive_system_pause(self, data, res):
        if data.data:
            CollaborationTool.loginfo('receive_pause_system {}'.format(data))
            CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_pause()), data)
        else:
            CollaborationTool.loginfo('receive_pause_system {}'.format(data))
            CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_pausecancel()), data)
        res.return_code = True
        return res

    def execute(self, object):
        pass

#協働作業開始コマンド受信クラス.
class StartManipulationTaskServer(CollaborationServer):
    def __init__(self):
        super().__init__('start_manip_task', Bool, self.receive_start_manip_task, 'start_manip_task_server_node')
        CollaborationTool.loginfo('{} start'.format(self.__class__.__name__))

    #協働作業コマンド受信.
    def receive_start_manip_task(self, data, res):
            CollaborationTool.loginfo('receive_start_manip_task {}'.format(data))
            CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_collabo()), data)
            res.return_code = True
            return res

    def service_delete(self):
        super().service_delete('start_manip_task deleted')

    def execute(self, object):
        pass

#協働作業終了コマンド受信クラス.
class StopManipulationTaskServer(CollaborationServer):
    def __init__(self):
        super().__init__('stop_manip_task', Bool, self.receive_stop_manip_task, 'stop_manip_task_server_node')
        CollaborationTool.loginfo('{} start'.format(self.__class__.__name__))

    #協働作業コマンド受信.
    def receive_stop_manip_task(self, data, res):
        if data.data:
            CollaborationTool.loginfo('receive_stop_manip_task {}'.format(data))
            CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_collaboend()), data)
            res.return_code = True
            return res

    def service_delete(self):
        super().service_delete('stop_manip_task deleted')

    def execute(self, object):
        pass

#協働サブスクライバースレッド.
class CollaborationSubscriverThread(threading.Thread):
    def __init__(self):
        super(CollaborationSubscriverThread, self).__init__()

    def run(self):
        rclpy.spin()

#作業開始コマンド受信クラス.
class TaskCommandServer(CollaborationServer):
    def __init__(self):
        super().__init__('send_command_service', SendTaskCommand, self.recv_task_command, 'send_command_server_node')
        CollaborationTool.loginfo('{} start '.format(self.__class__.__name__))
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('sys_manage_servece deleted')

    #作業開始コマンド受信.
    def recv_task_command(self, data, res):
        CollaborationTool.loginfo("TaskCommandServer::recv_task_command")

        #待機中以外で受信した場合はBUSYを返却する.
        from .CollaborationStateHeader import CollaborationCurrentData
        state = CollaborationCurrentData.GetCurrentState()
        if( EnumState.e_standby() != state.get_state()):
            print(state.get_state())
            res.response = EnumCommandReceiveState.e_busy()
            return res
        
        #受信データ変換処理.
        command_list = CollaborationTaskCommandList()
        for exchange_task in data.task_commands.task_commands :
            command = CollaborationTaskCommand()
            command.set_member(exchange_task.task_command_id, exchange_task.work_type_id, exchange_task.picking_count)
            work_presence = CollaborationWorkPresenceArea()
            work_presence.set_member(exchange_task.work_presence_area.start_point.x,
                                     exchange_task.work_presence_area.start_point.y,
                                     exchange_task.work_presence_area.start_point.z,
                                     exchange_task.work_presence_area.end_point.x,
                                     exchange_task.work_presence_area.end_point.y,
                                     exchange_task.work_presence_area.end_point.z)
            command.set_WorkPresenceArea(work_presence)
            discharge_list = CollaborationPlaceAreaCandidatesList()

            for rcv_dis_area in exchange_task.place_area_candidates.place_area_candidates:
                dis_area = CollaborationCandidatePlaceLocationArea()
                dis_area.set_member(rcv_dis_area.start_point.x,
                                    rcv_dis_area.start_point.y,
                                    rcv_dis_area.start_point.z,
                                    rcv_dis_area.end_point.x,
                                    rcv_dis_area.end_point.y,
                                    rcv_dis_area.end_point.z)
                discharge_list.add(dis_area)

            command.set_CandidatePlaceLocationAreaList(discharge_list)
            command_list.add(command)

        #受信内容をオブザーバ経由で通知する.
        CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_workstart()), command_list)
        
        res.response = EnumCommandReceiveState.e_received()
        return res

    #作業完了時に返信オブジェクトを記憶し、処理完了とする.
    def execute(self, object, task_command_id, number_of_items_picked, task_result):
        self.rsp = object
        self.is_rsp = True
        self.task_command_id = task_command_id
        self.number_of_items_picked = number_of_items_picked
        self.task_result = task_result
        self.set_wait_condition(self.is_rsp is False)
        return True

#作業中断受信クラス.
class TaskCancelServer(CollaborationServer):
    def __init__(self):
        super().__init__('task_cancel_server', CancelTask, self.recv_task_cancel, 'task_cancel_server_node')
        self.task_comand_id = 0
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('task_cancel_server deleted')

    #作業中断コマンド受信.
    def recv_task_cancel(self, data, res):
        CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_worksuspend()), data.task_command_id)
        self.task_comand_id = data.task_command_id
        self.wait(0.1, self.is_rsp is False)
        self.is_rsp = False
        return self.rsp

    def execute(self, object):
        rsp = CancelTask.Response()
        rsp.return_code = 1
        self.rsp = rsp
        self.is_rsp = True
        self.set_wait_condition(self.is_rsp is False)
        return True

#作業終了コマンド受信クラス.
class TaskFinishServer(CollaborationServer):
    def __init__(self):
        super().__init__('task_fin_service', FinishTask, self.recv_task_finish, 'task_fin_service_node')
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('task_fin_service deleted')

    #作業終了コマンド.
    def recv_task_finish(self, data, res):
        CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_workend()), data)
        self.wait(0.1, self.is_rsp is False)
        self.is_rsp = False
        return self.rsp
    
    def execute(self, object):
        rsp = FinishTask.Response()
        rsp.return_code = 1
        self.rsp = rsp
        self.is_rsp = True
        self.set_wait_condition(self.is_rsp is False)
        return True

#状態確認コマンド受信クラス.
class GetStatusServer(CollaborationServer):
    def __init__(self):
        super().__init__('get_status_server', GetStatus, self.recv_get_state, 'get_status_server_node')
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('get_status_server deleted')

    #状態確認コマンド受信.
    def recv_get_state(self, data, res):
        CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_getstate()), data.req)
        self.wait(0.1, self.is_rsp is False)
        res.status = self.rsp
        self.is_rsp = False
        return res
    
    def execute(self, object):
        self.rsp = object
        self.is_rsp = True
        self.set_wait_condition(self.is_rsp is False)
        return True
    
#サーバーに作業結果を送信するクラス.
class NotifyTaskResultClient(CollaborationClient):
    def __init__(self):
        super().__init__('notify_task_result', NotifyTaskResult, 'notify_task_result_client_node')

    def execute(self, object, task_command_id, number_of_items_picked, task_result):
        if not self.proxy.wait_for_service(timeout_sec = 10.0):
            CollaborationTool.loginfo("Service 'notify_task_result' not available after 10s.")
            return False
        
        res = NotifyTaskResult.Request()
        res.task_result.task_command_id = task_command_id
        res.task_result.picked_number = number_of_items_picked
        res.task_result.task_result = task_result
        try:
            future = self.proxy.call_async(res)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            CollaborationTool.loginfo("Task Result")
            return True
        except Exception as e:
            CollaborationTool.loginfo("ServiceException : %s" % e)
            return False

#サーバーに作業完了を送信するクラス.
class NotifyTaskCompleteClient(CollaborationClient):
    def __init__(self):
        super().__init__('notify_task_completion', NotifyTaskCompletion, 'notify_task_completion_client_node')

    def execute(self, object):
        if not self.proxy.wait_for_service(timeout_sec = 10.0):
            CollaborationTool.loginfo("Service 'notify_task_completion' not available after 10s.")
            return False
        
        try:
            req = NotifyTaskCompletion.Request()
            future = self.proxy.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            CollaborationTool.loginfo("Task Complete")
            return True
        except Exception as e:
            CollaborationTool.loginfo("ServiceException : %s" % e)
            return False
            

#サーバーにワーク認識指令を出すクラス.
class WorkDetectionClient(CollaborationClient):
    def __init__(self):
        super().__init__('detect_workpieces_service', DetectWorkpieces, 'detect_workpieces_client_node')
        self.work_detect_result = WorkDetectionResultList()

    def set_presence_area(self, req_task_id, req_work_id, sx, sy, sz, ex, ey, ez):
        self.req_task_id = req_task_id
        self.req_work_id = req_work_id
        set_area = TargetArea()
        set_area.start_point.x = sx
        set_area.start_point.y = sy
        set_area.start_point.z = sz
        set_area.end_point.x = ex
        set_area.end_point.y = ey
        set_area.end_point.z = ez
        self.req_target_area = set_area

    def execute(self, object):
        try:
            info = DetectWorkpieces.Request()
            info.task_command_id = self.req_task_id
            info.work_type_id = self.req_work_id
            info.target_area = self.req_target_area
            future = self.proxy.call_async(info)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            CollaborationTool.loginfo("Detection Result")
            CollaborationTool.loginfo(result.work_detection_results)
            self.work_detect_result = result.work_detection_results
            return True
        except Exception as e:
            CollaborationTool.loginfo("ServiceException : %s" % e)
            return False

#サーバーに排出場所検出指令を出すクラス.
class PlacePositionClient(CollaborationClient):
    def __init__(self):
        super().__init__('place_position_detect_service', DetectPlacePosition, 'place_position_detect_server_node')
        self.taskid = 0
    
    def set_place_area(self, sx, sy, sz, ex, ey, ez):
        self.req_place_position = DetectPlacePosition.Request()
        place_position = TargetArea()
        place_position.start_point.x = sx
        place_position.start_point.y = sy
        place_position.start_point.z = sz
        place_position.end_point.x = ex
        place_position.end_point.y = ey
        place_position.end_point.z = ez
        self.req_place_position.discharge_area = [place_position]

    def execute(self, object):
        try:
            future = self.proxy.call_async(self.req_place_position)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            CollaborationTool.loginfo("Detection Result")
            CollaborationTool.loginfo(result.place_position_detection_result.task_command_id)
            self.task_id = result.place_position_detection_result.task_command_id
            return True
        except Exception as e:
            CollaborationTool.loginfo("ServiceException : %s" % e)
            return False

#サーバーに周辺環境セット指令を出すクラス.
class PeripheralEnvironmentAreaSetClient(CollaborationClient):
    def __init__(self):
        super().__init__('per_env_det_service', SetMonitoringArea, 'per_env_det_server_node')
        self.req_list = MonitoringAreaList()
        self.area_setup_result = 0
    
    def set_peripheral_area(self, sx, sy, sz, ex, ey, ez, division):
        set_data = MonitoringArea()
        set_data.target_area.start_point.x = sx
        set_data.target_area.start_point.y = sy
        set_data.target_area.start_point.z = sz
        set_data.target_area.end_point.x = ex
        set_data.target_area.end_point.y = ey
        set_data.target_area.end_point.z = ez
        set_data.area_type = division
        self.req_list.monitoring_areas.append(set_data)

    def execute(self, object):
        try:
            future = self.proxy.call_async(self.req_list)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            CollaborationTool.loginfo("Area Setup Result")
            CollaborationTool.loginfo(result.setup_result)
            self.area_setup_result = result.setup_result
            return True
        except Exception as e:
            CollaborationTool.loginfo("ServiceException : %s" % e)
            return False

#サービスサーバーノードのspin用クラス.
class ServiceServerExecutor:
    def __init__(self):
        self.executor = SingleThreadedExecutor()
        self.node_list = []

    def add_service_server(self, node):
        self.executor.add_node(node)
        self.node_list.append(node)

    def execute(self):
        # サブスレッドでexecutorをspin.
        self.executor_thread = threading.Thread(target=self.spin_executor, args=(), daemon=True)
        self.executor_thread.start()

    def spin_executor(self):
        # サブスレッドでノードのspinを継続
        try:
            self.executor.spin()
        except ExternalShutdownException:
                print("Executor spin stopped by external shutdown.")
        except Exception as e:
                print(f"Executor spin error: {e}")
        finally:
                print("Executor stopped")
    
    def shutdown(self):
        self.executor.shutdown()
        if self.executor_thread is not None:
            self.executor_thread.join()
        for node in self.node_list:
            node.service_delete()
            node.destroy_node()