#!/usr/bin/env python3
# coding: UTF-8

import rospy
from rospy.topics import Publisher, Subscriber
from std_msgs.msg import String
from CollaborationStateHeader import *
from TransitionModule import Transition
from CollaborationToolModule import CollaborationTool
from CollaborationCommunicationModule import *
from geometry_msgs.msg import TransformStamped
from MovePlannerModule import MovePlanner

#定数の定義
DEFAULT_VELOCITY = 1.0
RETRY_COUNT_MAX = 3
PREPAR_WAIT = 3
PAUSED_WAIT = 0.5
NON_OPERATING_WAIT = 1

PICK_COMMAND = 0
DOWN_COMMAND = 1
PLACE_COMMAND = 2
READY_APPROACH_VAL_01 = 0.0
READY_APPROACH_VAL_02 = 0.0
READY_APPROACH_VAL_03 = 0.0
READY_APPROACH_VAL_04 = -52.0
READY_APPROACH_VAL_05 = 12.0
READY_APPROACH_VAL_06 = -4.0
READY_APPROACH_VAL_07 = -70.0
READY_APPROACH_VAL_08 = 0.0
READY_APPROACH_VAL_09 = 0.0
READY_APPROACH_VAL_10 = 0.0
READY_APPROACH_VAL_11 = 0.0
READY_APPROACH_VAL_12 = 0.0
READY_APPROACH_VAL_13 = 0.0
READY_APPROACH_VAL_14 = 0.0
READY_APPROACH_VAL_15 = 0.0
READY_APPROACH_VAL_16 = 0.0
READY_APPROACH_VAL_17 =-52.0
READY_APPROACH_VAL_18 = -12.0
READY_APPROACH_VAL_19 = 4.0
READY_APPROACH_VAL_20 = -70.0
READY_APPROACH_VAL_21 = 0.0
READY_APPROACH_VAL_22 = 0.0
READY_APPROACH_VAL_23 = 0.0
READY_APPROACH_VAL_24 = 0.0
READY_APPROACH_VAL_25 = 0.0
READY_APPROACH_VAL_26 = 0.0
READY_APPROACH_VEL = 1.0
APPROACH_VAL_01 = 0.0
APPROACH_VAL_02 = -9.0
APPROACH_VAL_03 = 0.0
APPROACH_VAL_04 = -52.0
APPROACH_VAL_05 = 10.0
APPROACH_VAL_06 = -1.0
APPROACH_VAL_07 = -33.0
APPROACH_VAL_08 = 32.0
APPROACH_VAL_09 = -19.0
APPROACH_VAL_10 = -13.0
APPROACH_VAL_11 = 2.0
APPROACH_VAL_12 = -1.0
APPROACH_VAL_13 = -18.0
APPROACH_VAL_14 = 0.0
APPROACH_VAL_15 = 0.0
APPROACH_VAL_16 = 0.0
APPROACH_VAL_17 = -52.0
APPROACH_VAL_18 = -10.0
APPROACH_VAL_19 = 0.0
APPROACH_VAL_20 = -30.0
APPROACH_VAL_21 = 30.0
APPROACH_VAL_22 = -18.0
APPROACH_VAL_23 = -12.0
APPROACH_VAL_24 = -2.0
APPROACH_VAL_25 = -5.0
APPROACH_VAL_26 = 19.0
APPROACH_VEL = 1.0
PICK_VAL_01 = 0.0
PICK_VAL_02 = -9.15112
PICK_VAL_03 = 0.0
PICK_VAL_04 = -46.86233
PICK_VAL_05 = 2.74952
PICK_VAL_06 = -0.95000
PICK_VAL_07 = -37.39286
PICK_VAL_08 = 37.39286
PICK_VAL_09 = -22.10748
PICK_VAL_10 = -15.28538
PICK_VAL_11 = -1.69000
PICK_VAL_12 = 1.38875
PICK_VAL_13 = -9.68590
PICK_VAL_14 = 0.0
PICK_VAL_15 = 0.0
PICK_VAL_16 = 0.0
PICK_VAL_17 = -46.52467
PICK_VAL_18 = -2.86513
PICK_VAL_19 = 0.01000
PICK_VAL_20 = -38.00977
PICK_VAL_21 = 38.00977
PICK_VAL_22 = -22.47221
PICK_VAL_23 = -15.53756
PICK_VAL_24 = 0.97000
PICK_VAL_25 = -1.07731
PICK_VAL_26 = 9.75000
PICK_VEL = 1.0
UP_VAL_01 = 0.0
UP_VAL_02 = -9.2
UP_VAL_03 = 0.0
UP_VAL_04 = -47.0
UP_VAL_05 = 1.0
UP_VAL_06 = -1.0
UP_VAL_07 = -43.0
UP_VAL_08 = 43.0
UP_VAL_09 = -25.0
UP_VAL_10 = -18.0
UP_VAL_11 = 2.0
UP_VAL_12 = 6.0
UP_VAL_13 = -10.0
UP_VAL_14 = 0.0
UP_VAL_15 = 0.0
UP_VAL_16 = 0.0
UP_VAL_17 = -47.0
UP_VAL_18 = -2.0
UP_VAL_19 = 0.0
UP_VAL_20 = -43.0
UP_VAL_21 = 43.0
UP_VAL_22 = 25.0
UP_VAL_23 = -18.0
UP_VAL_24 = 2.0
UP_VAL_25 = 6.0
UP_VAL_26 = 10.0
UP_VEL = 1.0
DOWN_LIFTER_VAL_01 = 55.0
DOWN_LIFTER_VAL_02 = 55.0
DOWN_LIFTER_VAL_03 = -55.0
DOWN_LIFTER_VAL_04 = -55.0
DOWN_LIFTER_VEL = 1.0
SET_LIFTER_VAL_01 = 63.0
SET_LIFTER_VAL_02 =63.0
SET_LIFTER_VAL_03 =-63.0
SET_LIFTER_VAL_04 =-63.0
SET_LIFTER_VEL =0.1

###########################################
#MovePlanner共有クラス.                   #
###########################################
class ShareMovePlanner:
    move_plannner = MovePlanner()
    @classmethod
    def get_move_planner(cls):
        return cls.move_plannner
    
###########################################
#初期処理クラス.                          #
###########################################
class INITIALISE(CollaborationState):
    def __init__(self,
                 label,
                 taskfin:TaskFinishServer,
                 tran:Transition,
                 outcomes):
        super().__init__(label, tran, outcomes)
        self.planner = ShareMovePlanner.get_move_planner()
        self.taskfin = taskfin
        self.counter = 0

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        
        if self.tran.get_event().is_workend():
            task_command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = task_command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        if self.preempt('INIITIALISE is being preempted!!!'):
            return 'preempted'
        
        #初期処理を行う.
        return self.initialise()

    def initialise(self):
        ###ロボットごとの初期動作.
        #ここから.

        while not CollaborationTool.is_shutdown():
          #初期位置移動.
          ret = self.init_pose_upper_body()
          if 'succeeded' == ret:
                break
          elif 'retry' == ret:
            continue
          else:
            return ret
        
        while not CollaborationTool.is_shutdown():
          #初期位置移動.
          ret = self.init_pose_lifter()
          if 'succeeded' == ret:
                break
          elif 'retry' == ret:
            continue
          else:
            return ret

        #ここまで.
        return 'succeeded'

    #初期位置移動.
    def init_pose_upper_body(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('Initialise')
            rc = self.planner.initial_pose_upper_body()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
        
    def init_pose_lifter(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('Initialise')
            rc = self.planner.initial_pose_lifter()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'

###########################################
#スタート前処理クラス.                    #
###########################################
class BEFORE_START(CollaborationState):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        if self.tran.get_event().is_pause():
            CollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        CollaborationTool.loginfo('Before Start ...')
        if self.preempt('State BEFORE_START is being preempted!!!'):
            return 'preempted'
      
        return self.before_start()

    def before_start(self):
        ###ロボットごとのスタート前動作.
        #ここから.

        #ここまで.
        return 'succeeded'

###########################################
#準備中処理クラス.                        #
###########################################
class PREPAR_WORK(CollaborationState):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition, area, disc, workd, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin
        self.area = area
        self.disc = disc
        self.workd = workd

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)

        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
      
        CollaborationTool.loginfo('Preparing ...')
        if self.preempt('State Prepar is being preempted!!!'):
            return 'preempted'
        
        #準備処理.
        return self.prepar()

    def prepar(self):
        ###ロボットごとの準備動作.
        #ここから.

        self.counter = 0

        #周辺環境範囲設定.
        while not CollaborationTool.is_shutdown():
            ret = self.PeripheralEnvironmentAreaSet()
            if 'succeeded' == ret:
                break
            elif 'retry' == ret:
                continue
            else:
                return ret
          
        #ワーク検知.
        while not CollaborationTool.is_shutdown():
            ret = self.WorkDetect()
            if 'succeeded' == ret:
                    break
            elif 'retry' == ret:
                continue
            else:
                return ret
          
        #排出位置検出.
        while not CollaborationTool.is_shutdown():
            ret = self.PlaceDetect()
            if 'succeeded' == ret:
                    break
            elif 'retry' == ret:
             continue
            else:
                return ret
        
        #ここまで.
        return 'succeeded'
    
    #周辺環境範囲を設定する.
    def PeripheralEnvironmentAreaSet(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        return 'succeeded'

    #排出位置を検出する.
    def PlaceDetect(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        CollaborationTool.loginfo("waiting place_position")
        CollaborationTool.wait_for_service('place_position_detect_service')
        CollaborationTool.loginfo("place_position comes up")

        event = self.check_event()
        if(None != event ):
            return event
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State PlaceDetect is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            #排出位置を探索.
            self.set_place_area()
            rc = self.disc.execute(None)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc) 
        else:
            return 'aborted'

    #ワークを検知する.
    def WorkDetect(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
    
        CollaborationTool.loginfo("waiting work_det_service")
        CollaborationTool.wait_for_service('detect_workpieces_service')
        CollaborationTool.loginfo("work_det_service comes up")

        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State Work Detect is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            #ワーク検知範囲の設定.
            self.set_presence_area()
            rc = self.workd.execute(None)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc) 
        else:
            return 'aborted'

    #排出位置を設定する.
    def set_place_area(self):
        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        task_command = command_list.get(0)
        place_area_list = task_command.get_PlaceLocationAreaList()
        print(place_area_list)
        place_area = place_area_list[0]
        print(place_area)
        self.disc.set_place_area(place_area.start_point.x,
                                     place_area.start_point.y,
                                     place_area.start_point.z,
                                     place_area.end_point.x,
                                     place_area.end_point.y,
                                     place_area.end_point.z,)

    #ワーク検知範囲を設定する.
    def set_presence_area(self):
        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        task_command = command_list.get(0)
        presence_area = task_command.work_presence_area
        self.workd.set_presence_area(task_command.task_command_id,
                                     task_command.work_type_id,
                                     presence_area.start_point.x,
                                     presence_area.start_point.y,
                                     presence_area.start_point.z,
                                     presence_area.end_point.x,
                                     presence_area.end_point.y,
                                     presence_area.end_point.z,)
    
    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_pause():
            CollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None
        
###########################################
#自律動作処理クラス.                      #
###########################################
class OPERATING_WORK(CollaborationState):
    def __init__(self,
                 label,
                 taskfin:TaskFinishServer,
                 tran:Transition,
                 outcomes):
        super().__init__(label, tran, outcomes)
        self.planner = ShareMovePlanner.get_move_planner()
        self.taskfin = taskfin
        self.counter = 0
        self.work_label = None

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        CollaborationTool.loginfo(command_list.get(0).task_command_id)
        
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        #自律動作.
        return self.operating_work()

    #自律動作処理.
    def operating_work(self):
        ###ロボットごとの自律動作.
        #ここから.

        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        command_id = command_list.get(0).task_command_id

        ret = ''
        if(PICK_COMMAND == command_id):
            ret = self.pick_move()
        elif(DOWN_COMMAND == command_id):
            ret = self.down_move()
        elif(PLACE_COMMAND == command_id):
            ret = self.place_move()
        
        #ここまで.
        self.work_label = None
        return 'succeeded'
    
    def pick_move(self):
        #上半身の初期位置1.
        if (None == self.work_label) or ('UpperInit' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'TfManip'
                ret = self.init_pose_upper_body()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #リフターの初期位置.
        if (None == self.work_label) or ('CurrentManip' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'CurrentManip1'
                ret = self.init_pose_lifter()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #アプローチ準備位置.
        if (None == self.work_label) or ('ReadyApproach' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'ReadyApproach'
                ret = self.joint_manip_upper(READY_APPROACH_VAL_01, READY_APPROACH_VAL_02, READY_APPROACH_VAL_03, READY_APPROACH_VAL_04, READY_APPROACH_VAL_05, READY_APPROACH_VAL_06, READY_APPROACH_VAL_07,
                                             READY_APPROACH_VAL_08, READY_APPROACH_VAL_09, READY_APPROACH_VAL_10, READY_APPROACH_VAL_11, READY_APPROACH_VAL_12, READY_APPROACH_VAL_13, READY_APPROACH_VAL_14,
                                             READY_APPROACH_VAL_15, READY_APPROACH_VAL_16, READY_APPROACH_VAL_17, READY_APPROACH_VAL_18, READY_APPROACH_VAL_19, READY_APPROACH_VAL_20, READY_APPROACH_VAL_21,
                                             READY_APPROACH_VAL_22, READY_APPROACH_VAL_23, READY_APPROACH_VAL_24, READY_APPROACH_VAL_25, READY_APPROACH_VAL_26, READY_APPROACH_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #アプローチ位置.
        if (None == self.work_label) or ('Approach' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'Approach'
                ret = self.joint_manip_upper(APPROACH_VAL_01, APPROACH_VAL_02, APPROACH_VAL_03, APPROACH_VAL_04, APPROACH_VAL_05, APPROACH_VAL_06, APPROACH_VAL_07,
                                             APPROACH_VAL_08, APPROACH_VAL_09, APPROACH_VAL_10, APPROACH_VAL_11, APPROACH_VAL_12, APPROACH_VAL_13, APPROACH_VAL_14,
                                             APPROACH_VAL_15, APPROACH_VAL_16, APPROACH_VAL_17, APPROACH_VAL_18, APPROACH_VAL_19, APPROACH_VAL_20, APPROACH_VAL_21,
                                             APPROACH_VAL_22, APPROACH_VAL_23, APPROACH_VAL_24, APPROACH_VAL_25, APPROACH_VAL_26, APPROACH_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #つかみ位置.
        if (None == self.work_label) or ('Pick' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'Pick'
                ret = self.joint_manip_upper(PICK_VAL_01, PICK_VAL_02, PICK_VAL_03, PICK_VAL_04, PICK_VAL_05, PICK_VAL_06, PICK_VAL_07,
                                             PICK_VAL_08, PICK_VAL_09, PICK_VAL_10, PICK_VAL_11, PICK_VAL_12, PICK_VAL_13, PICK_VAL_14,
                                             PICK_VAL_15, PICK_VAL_16, PICK_VAL_17, PICK_VAL_18, PICK_VAL_19, PICK_VAL_20, PICK_VAL_21,
                                             PICK_VAL_22, PICK_VAL_23, PICK_VAL_24, PICK_VAL_25, PICK_VAL_26, PICK_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #上昇位置.
        if (None == self.work_label) or ('Up' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'Up'
                ret = self.joint_manip_upper(UP_VAL_01, UP_VAL_02, UP_VAL_03, UP_VAL_04, UP_VAL_05, UP_VAL_06, UP_VAL_07,
                                             UP_VAL_08, UP_VAL_09, UP_VAL_10, UP_VAL_11, UP_VAL_12, UP_VAL_13, UP_VAL_14,
                                             UP_VAL_15, UP_VAL_16, UP_VAL_17, UP_VAL_18, UP_VAL_19, UP_VAL_20, UP_VAL_21,
                                             UP_VAL_22, UP_VAL_23, UP_VAL_24, UP_VAL_25, UP_VAL_26, UP_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret

        self.work_label = None
        return 'succeeded'

    def down_move(self):
        #リフターを下げる.
        if (None == self.work_label) or ('DownLifetr' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'DownLifetr'
                ret = self.joint_manip_lifter(DOWN_LIFTER_VAL_01, DOWN_LIFTER_VAL_02, DOWN_LIFTER_VAL_03, DOWN_LIFTER_VAL_04, DOWN_LIFTER_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret

        self.work_label = None
        return 'succeeded'

    def place_move(self):
        #リフターをセット位置にする.
        if (None == self.work_label) or ('SetLifetr' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'SetLifetr'
                ret = self.joint_manip_lifter(SET_LIFTER_VAL_01, SET_LIFTER_VAL_02, SET_LIFTER_VAL_03, SET_LIFTER_VAL_04, SET_LIFTER_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
    
        #上昇位置.
        if (None == self.work_label) or ('Place1' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'Place1'
                ret = self.joint_manip_upper(UP_VAL_01, UP_VAL_02, UP_VAL_03, UP_VAL_04, UP_VAL_05, UP_VAL_06, UP_VAL_07,
                                             UP_VAL_08, UP_VAL_09, UP_VAL_10, UP_VAL_11, UP_VAL_12, UP_VAL_13, UP_VAL_14,
                                             UP_VAL_15, UP_VAL_16, UP_VAL_17, UP_VAL_18, UP_VAL_19, UP_VAL_20, UP_VAL_21,
                                             UP_VAL_22, UP_VAL_23, UP_VAL_24, UP_VAL_25, UP_VAL_26, UP_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #つかみ位置.
        if (None == self.work_label) or ('Place2' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'Place2'
                ret = self.joint_manip_upper(PICK_VAL_01, PICK_VAL_02, PICK_VAL_03, PICK_VAL_04, PICK_VAL_05, PICK_VAL_06, PICK_VAL_07,
                                             PICK_VAL_08, PICK_VAL_09, PICK_VAL_10, PICK_VAL_11, PICK_VAL_12, PICK_VAL_13, PICK_VAL_14,
                                             PICK_VAL_15, PICK_VAL_16, PICK_VAL_17, PICK_VAL_18, PICK_VAL_19, PICK_VAL_20, PICK_VAL_21,
                                             PICK_VAL_22, PICK_VAL_23, PICK_VAL_24, PICK_VAL_25, PICK_VAL_26, PICK_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #リリース位置.
        if (None == self.work_label) or ('Release' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'Release'
                ret = self.joint_manip_upper(APPROACH_VAL_01, APPROACH_VAL_02, APPROACH_VAL_03, APPROACH_VAL_04, APPROACH_VAL_05, APPROACH_VAL_06, APPROACH_VAL_07,
                                             APPROACH_VAL_08, APPROACH_VAL_09, APPROACH_VAL_10, APPROACH_VAL_11, APPROACH_VAL_12, APPROACH_VAL_13, APPROACH_VAL_14,
                                             APPROACH_VAL_15, APPROACH_VAL_16, APPROACH_VAL_17, APPROACH_VAL_18, APPROACH_VAL_19, APPROACH_VAL_20, APPROACH_VAL_21,
                                             APPROACH_VAL_22, APPROACH_VAL_23, APPROACH_VAL_24, APPROACH_VAL_25, APPROACH_VAL_26, APPROACH_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret

        #初期化準備位置.
        if (None == self.work_label) or ('ReadyInit' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'ReadyApproach'
                ret = self.joint_manip_upper(READY_APPROACH_VAL_01, READY_APPROACH_VAL_02, READY_APPROACH_VAL_03, READY_APPROACH_VAL_04, READY_APPROACH_VAL_05, READY_APPROACH_VAL_06, READY_APPROACH_VAL_07,
                                             READY_APPROACH_VAL_08, READY_APPROACH_VAL_09, READY_APPROACH_VAL_10, READY_APPROACH_VAL_11, READY_APPROACH_VAL_12, READY_APPROACH_VAL_13, READY_APPROACH_VAL_14,
                                             READY_APPROACH_VAL_15, READY_APPROACH_VAL_16, READY_APPROACH_VAL_17, READY_APPROACH_VAL_18, READY_APPROACH_VAL_19, READY_APPROACH_VAL_20, READY_APPROACH_VAL_21,
                                             READY_APPROACH_VAL_22, READY_APPROACH_VAL_23, READY_APPROACH_VAL_24, READY_APPROACH_VAL_25, READY_APPROACH_VAL_26, READY_APPROACH_VEL)
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret

        #上半身の初期位置.
        if (None == self.work_label) or ('UpperInit' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'TfManip'
                ret = self.init_pose_upper_body()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #リフターの初期位置.
        if (None == self.work_label) or ('CurrentManip' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'CurrentManip1'
                ret = self.init_pose_lifter()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret

        self.work_label = None
        return 'succeeded'

    #初期位置移動.
    def init_pose_upper_body(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('Initialise')
            rc = self.planner.initial_pose_upper_body()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
        
    def init_pose_lifter(self):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('Initialise')
            rc = self.planner.initial_pose_lifter()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
        
    def joint_manip_upper(self, val_1, val_2, val_3, val_4, val_5, val_6, val_7, val_8, val_9, val_10, val_11, val_12, val_13, val_14, val_15,
                                          val_16, val_17, val_18, val_19, val_20, val_21, val_22, val_23, val_24, val_25, val_26, vel):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State NORMAL_MANIP is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            rc = self.planner.joint_value_upper_body(val_1, val_2, val_3, val_4, val_5, val_6, val_7, val_8, val_9, val_10, val_11, val_12, val_13, val_14, val_15,
                                          val_16, val_17, val_18, val_19, val_20, val_21, val_22, val_23, val_24, val_25, val_26, vel)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
    
    def joint_manip_lifter(self, val_1, val_2, val_3, val_4, vel):
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event):
            return event
        
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            if self.preempt('State NORMAL_MANIP is being preempted!!!'):
                return 'preempted'
            self.counter += 1
            rc = self.planner.joint_value_lifter(val_1, val_2, val_3, val_4, vel)
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'

    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_pause():
            CollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        elif self.tran.get_event().is_workend():
            command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None


###########################################
#非自律動作処理クラス.                    #
###########################################
class NON_OPERATING_WORK(CollaborationState):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin
        self.planner = ShareMovePlanner.get_move_planner()

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        CollaborationTool.loginfo(command_list.get(0).task_command_id)
        
        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        #非自律動作処理.
        return self.non_operating_work()
    
    #非自律動作処理.
    def non_operating_work(self):
        ###ロボットごとの非自律動作.
        #ここから.

        CollaborationTool.wait_time(NON_OPERATING_WAIT)

        #初期位置に戻る.
        if (None == self.work_label) or ('InitPose1' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'InitPose1'
                ret = self.init_pose_upper_body()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #初期位置に戻る.
        if (None == self.work_label) or ('InitPose2' == self.work_label):
            while not CollaborationTool.is_shutdown():
                self.work_label = 'InitPose1'
                ret = self.init_pose_lifter()
                if 'succeeded' == ret:
                    self.work_label = None
                    break
                elif 'retry' == ret:
                    continue
                else:
                    return ret
        
        #ここまで.
        self.work_label = None
        return 'succeeded'
    
    #初期位置移動.
    def init_pose_upper_body(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('Initialise')
            rc = self.planner.initial_pose_upper_body()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
        
    def init_pose_lifter(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('Initialise')
            rc = self.planner.initial_pose_lifter()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'

    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_pause():
            CollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        elif self.tran.get_event().is_workend():
            command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None

###########################################
#一時停止中処理クラス.                    #
###########################################
class PAUSED_WORK(CollaborationState):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin
        self.counter = 0
        self.planner = ShareMovePlanner.get_move_planner()

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)

        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event

        CollaborationTool.loginfo('Preparing ...')
        if self.preempt('State Prepar is being preempted!!!'):
            return 'preempted'
        
        #待機処理.
        return self.paused()

    def paused(self):
        ###ロボットごとの待機動作.
        #ここから.

        CollaborationTool.wait_time(PAUSED_WAIT)

        #作業途中で状態遷移するイベントの確認.
        event = self.check_event()
        if(None != event ):
            return event
        
        #初期位置移動.
        while not CollaborationTool.is_shutdown():
            ret = self.init_pose()
            if 'succeeded' == ret:
                    break
            elif 'retry' == ret:
                continue
            else:
                return ret


        #ここまで.
        return 'succeeded'

    #初期位置移動.
    def init_pose_upper_body(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('Initialise')
            rc = self.planner.initial_pose_upper_body()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
        
    def init_pose_lifter(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('Initialise')
            rc = self.planner.initial_pose_lifter()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'
    #作業途中で状態遷移するイベントの確認.
    def check_event(self):
        if self.tran.get_event().is_workend():
            command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return None

###########################################
#終了処理クラス.                          #
###########################################        
class FINALISE(CollaborationState):
    def __init__(self,
                 label,
                 taskfin:TaskFinishServer,
                 tran:Transition,
                 outcomes):
        super().__init__(label, tran, outcomes)
        self.planner = ShareMovePlanner.get_move_planner()
        self.taskfin = taskfin
        self.counter = 0

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)

        if self.tran.get_event().is_workend():
            task_command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = task_command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        if self.preempt('INIITIALISE_POSE is being preempted!!!'):
            return 'preempted'
        
        #終了処理を行う.
        return self.finalise()

    #終了処理.
    def finalise(self):
        ###ロボットごとの終了動作.
        #ここから.

        #トレイを持っているときに軸を動かすと事故につながるので何もせず終了.

        #終了する.
        CollaborationTool.signal_shutdown("sys_manage_halt")

        #ここまで.
        return 'succeeded'
    
    def init_pose(self):
        #3回以上連続で失敗したら異常終了する.
        if self.counter < RETRY_COUNT_MAX:
            self.counter += 1
            CollaborationTool.loginfo('finalise')
            rc = self.planner.initial_pose()
            #正常完了の場合はエラーカウンターをリセット.
            if True == rc:
                self.counter = 0
            return self.return_btos(rc)
        else:
            return 'aborted'