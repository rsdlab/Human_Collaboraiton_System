#!/usr/bin/env python3
# coding: UTF-8

from abc import ABCMeta
from abc import abstractmethod
import rospy, tf2_ros
from collaboration_manipulation_message.msg import TargetArea, WorkDetectionResultList, MonitoringAreaList, MonitoringArea
from collaboration_manipulation_module.srv import *
from peripheral_environment_detection_subsystem.srv import *
from workpieces_detection_subsystem.srv import *
from management_system.srv import * 
from place_position_detection_subsystem.srv import *
from CollaborationEventModule import CollaborationEventPublisher, CollaborationTaskCommand, CollaborationWorkPresenceArea
from CollaborationEventModule import CollaborationPlaceAreaCandidatesList, CollaborationCandidatePlaceLocationArea, CollaborationTaskCommandList
from CollaborationToolModule import CollaborationTool
from EnumerateModule import EnumEvent
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Point
import threading
from EnumerateModule import EnumState, EnumCommandReceiveState


#協働通信クラスの抽象クラス.
class CollaborationCommunication(metaclass=ABCMeta):
    """
    Base class that HumanCollaboration communication
    Define machine-dependent packages
    """
    @abstractmethod
    def execute(self, object):
        pass

#協働パブリッシャークラスの抽象クラス.
class CollaborationPublisher(CollaborationCommunication):
    """
    Base class that HumanCollaboration communication for Publisher
    Define machine-dependent packages
    """
    def __init__(self, topic_name, datatype):
        self.pub = rospy.Publisher(topic_name, datatype, queue_size=10)

    def execute(self, object):
        pass

#協働サブスクライバークラスの抽象クラス.
class CollaborationSubscriver(CollaborationCommunication):
    """
    Base class that HumanCollaboration communication for Subscriver
    Define machine-dependent packages
    """
    def __init__(self, topic_name, datatype, callback):
        self.sub = rospy.Subscriber(topic_name, datatype, callback)
    def execute(self, object):
        pass

#協働クライアントクラスの抽象クラス.
class CollaborationClient(CollaborationCommunication):
    """
    Base class that HumanCollaboration communication for request
    Define machine-dependent packages
    """
    def __init__(self, service_name, service_class):
        self.proxy = rospy.ServiceProxy(service_name, service_class)

    def execute(self, object):
        pass

#協働サーバークラスの抽象クラス.
class CollaborationServer(CollaborationCommunication):
    """
    Base class that HumanCollaboration communication for response
    Define machine-dependent packages
    """
    def __init__(self, service_name, service_class, callback_impl):
        self.server = rospy.Service(service_name, service_class, callback_impl)

    def service_delete(self, msg=''):
        self.server.shutdown(msg)

    def execute(self, object):
        pass

    def wait(self, time, condition):
        self.condition = condition
        while self.condition:
            rospy.sleep(time)

    def set_wait_condition(self, condition):
        self.condition = condition

#システム終了コマンド受信クラス.
class SystemTerminate(CollaborationServer):
    def __init__(self):
        super().__init__("terminate_system", TerminateSystem, self.receive_system_halt)
        CollaborationTool.loginfo('{} start '.format(self.__class__.__name__))

    #システム終了コマンド受信.
    def receive_system_halt(self, data):
        CollaborationTool.loginfo('receive_terminate_system {}'.format(data))
        CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_end()), data)
        return True

    def service_delete(self):
        super().service_delete('terminate_system deleted')

    def execute(self, object):
        pass

#システム一時停止コマンド受信クラス.
class SystemPause(CollaborationServer):
    def __init__(self):
        super().__init__("pause_system", Bool, self.receive_system_pause)
        CollaborationTool.loginfo('{} start '.format(self.__class__.__name__))
    
    #システム一時停止コマンド受信.
    def receive_system_pause(self, data):
        if data.data:
            CollaborationTool.loginfo('receive_pause_system {}'.format(data))
            CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_pause()), data)
        else:
            CollaborationTool.loginfo('receive_pause_system {}'.format(data))
            CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_pausecancel()), data)

    def execute(self, object):
        pass

#協働作業開始コマンド受信クラス.
class StartManipulationTaskServer(CollaborationServer):
    def __init__(self):
        super().__init__("start_manip_task", Bool, self.receive_start_manip_task)
        CollaborationTool.loginfo('{} start'.format(self.__class__.__name__))

    #協働作業コマンド受信.
    def receive_start_manip_task(self, data):
            CollaborationTool.loginfo('receive_start_manip_task {}'.format(data))
            CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_collabo()), data)

    def service_delete(self):
        super().service_delete('start_manip_task deleted')

    def execute(self, object):
        pass

#協働作業終了コマンド受信クラス.
class StopManipulationTaskServer(CollaborationServer):
    def __init__(self):
        super().__init__("stop_manip_task", Bool, self.receive_stop_manip_task)
        CollaborationTool.loginfo('{} start'.format(self.__class__.__name__))

    #協働作業コマンド受信.
    def receive_stop_manip_task(self, data):
        if data.data:
            CollaborationTool.loginfo('receive_stop_manip_task {}'.format(data))
            CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_collaboend()), data)

    def service_delete(self):
        super().service_delete('stop_manip_task deleted')

    def execute(self, object):
        pass

#協働サブスクライバースレッド.
class CollaborationSubscriverThread(threading.Thread):
    def __init__(self):
        super(CollaborationSubscriverThread, self).__init__()

    def run(self):
        rospy.spin()

#作業開始コマンド受信クラス.
class TaskCommandServer(CollaborationServer):
    def __init__(self):
        super().__init__('sned_command_service', SendTaskCommand, self.recv_task_command)
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('sys_manage_servece deleted')

    #作業開始コマンド受信.
    def recv_task_command(self, data):
        CollaborationTool.loginfo("TaskCommandServer::recv_task_command")

        #待機中以外で受信した場合はBUSYを返却する.
        from CollaborationStateHeader import CollaborationCurrentData
        state = CollaborationCurrentData.GetCurrentState()
        if( EnumState.e_standby() != state.get_state()):
            print(state.get_state())
            ret = SendTaskCommandResponse()
            ret.response = EnumCommandReceiveState.e_busy()
            return ret
        
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
        
        ret = SendTaskCommandResponse()
        ret.response = EnumCommandReceiveState.e_received()
        return ret

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
        super().__init__('task_cancel_server', CancelTask, self.recv_task_cancel)
        self.task_comand_id = 0
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('task_cancel_server deleted')

    #作業中断コマンド受信.
    def recv_task_cancel(self, data):
        CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_worksuspend()), data.task_command_id)
        self.task_comand_id = data.task_command_id
        self.wait(0.1, self.is_rsp is False)
        self.is_rsp = False
        return self.rsp

    def execute(self, object):
        rsp = CancelTaskResponse()
        rsp.return_code = 1
        self.rsp = rsp
        self.is_rsp = True
        self.set_wait_condition(self.is_rsp is False)
        return True

#作業終了コマンド受信クラス.
class TaskFinishServer(CollaborationServer):
    def __init__(self):
        super().__init__('task_fin_service', FinishTask, self.recv_task_finish)
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('task_fin_service deleted')

    #作業終了コマンド.
    def recv_task_finish(self, data):
        CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_workend()), data)
        self.wait(0.1, self.is_rsp is False)
        self.is_rsp = False
        return self.rsp
    
    def execute(self, object):
        rsp = FinishTaskResponse()
        rsp.return_code = 1
        self.rsp = rsp
        self.is_rsp = True
        self.set_wait_condition(self.is_rsp is False)
        return True

#状態確認コマンド受信クラス.
class GetStatusServer(CollaborationServer):
    def __init__(self):
        super().__init__('get_status_server', GetStatus, self.recv_get_state)
        self.rsp = None
        self.is_rsp = False

    def service_delete(self):
        super().service_delete('get_status_server deleted')

    #状態確認コマンド受信.
    def recv_get_state(self, data):
        CollaborationEventPublisher.notify(EnumEvent(EnumEvent.e_getstate()), data.req)
        self.wait(0.1, self.is_rsp is False)
        rsp = GetStatusResponse()
        rsp.res = self.rsp
        self.is_rsp = False
        return rsp
    
    def execute(self, object):
        self.rsp = object
        self.is_rsp = True
        self.set_wait_condition(self.is_rsp is False)
        return True
    
#サーバーに作業結果を送信するクラス.
class NotifyTaskResultClient(CollaborationClient):
    def __init__(self):
        super().__init__('notify_task_result', NotifyTaskResult)

    def execute(self, object, task_command_id, number_of_items_picked, task_result):
        res = NotifyTaskResultRequest()
        res.task_result.task_command_id = task_command_id
        res.task_result.picked_number = number_of_items_picked
        res.task_result.task_result = task_result
        try:
            rospy.wait_for_service('notify_task_result', timeout=0.01)
            try:
                self.proxy(res)
                CollaborationTool.loginfo("Task Result")
                return True
            except CollaborationTool.ServiceException as e:
                CollaborationTool.loginfo("ServiceException : %s" % e)
                return False
        except rospy.ROSException:
            # サービスが起動していない → 通信せずスキップ.
            CollaborationTool.loginfo("Service 'notify_task_result' not available. Skipping service call.")
            return True

#サーバーに作業完了を送信するクラス.
class NotifyTaskCompleteClient(CollaborationClient):
    def __init__(self):
        super().__init__('notify_task_completion', NotifyTaskCompletion)

    def execute(self, object):
        try:
            rospy.wait_for_service('notify_task_completion', timeout=0.01)
            try:
                req = NotifyTaskCompletionRequest()
                self.proxy(req)
                CollaborationTool.loginfo("Task Complete")
                return True
            except CollaborationTool.ServiceException as e:
                CollaborationTool.loginfo("ServiceException : %s" % e)
                return False
        except rospy.ROSException:
            # サービスが起動していない → 通信せずスキップ.
            CollaborationTool.loginfo("Service 'notify_task_completion' not available. Skipping service call.")
            return True

#サーバーにワーク認識指令を出すクラス.
class WorkDetectionClient(CollaborationClient):
    def __init__(self):
        super().__init__('detect_workpieces_service', DetectWorkpieces)
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
            info = DetectWorkpiecesRequest()
            info.task_command_id = self.req_task_id
            info.work_type_id = self.req_work_id
            info.target_area = self.req_target_area
            result = self.proxy(info)
            CollaborationTool.loginfo("Detection Result")
            CollaborationTool.loginfo(result.work_detection_results)
            self.work_detect_result = result.work_detection_results
            return True
        except CollaborationTool.ServiceException as e:
            CollaborationTool.loginfo("ServiceException : %s" % e)
            return False

#サーバーに排出場所検出指令を出すクラス.
class PlacePositionClient(CollaborationClient):
    def __init__(self):
        super().__init__('place_position_detect_service', DetectPlacePosition)
        self.taskid = 0
    
    def set_place_area(self, sx, sy, sz, ex, ey, ez):
        place_position = TargetArea()
        place_position.start_point.x = sx
        place_position.start_point.y = sy
        place_position.start_point.z = sz
        place_position.end_point.x = ex
        place_position.end_point.y = ey
        place_position.end_point.z = ez
        self.req_place_position = [place_position]

    def execute(self, object):
        try:
            result = self.proxy(self.req_place_position)
            CollaborationTool.loginfo("Detection Result")
            CollaborationTool.loginfo(result.place_position_detection_result.task_command_id)
            self.task_id = result.place_position_detection_result.task_command_id
            return True
        except CollaborationTool.ServiceException as e:
            CollaborationTool.loginfo("ServiceException : %s" % e)
            return False

#サーバーに周辺環境セット指令を出すクラス.
class PeripheralEnvironmentAreaSetClient(CollaborationClient):
    def __init__(self):
        super().__init__('per_env_det_service', SetMonitoringArea)
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
            result = self.proxy(self.req_list)
            CollaborationTool.loginfo("Area Setup Result")
            CollaborationTool.loginfo(result.setup_result)
            self.area_setup_result = result.setup_result
            return True
        except CollaborationTool.ServiceException as e:
            CollaborationTool.loginfo("ServiceException : %s" % e)
            return False
