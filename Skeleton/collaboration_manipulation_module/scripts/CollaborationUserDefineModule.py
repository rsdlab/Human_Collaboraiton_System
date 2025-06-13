#!/usr/bin/env python3
# coding: UTF-8

from CollaborationStateHeader import *
from TransitionModule import Transition
from CollaborationToolModule import CollaborationTool
from CollaborationCommunicationModule import *

#定数の定義
    
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

        #ここまで.
        return 'succeeded'

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
        
        #ここまで.
        return 'succeeded'
    
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
        self.taskfin = taskfin
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
        
        #ここまで.
        self.work_label = None
        return 'succeeded'

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
        
        #ここまで.
        self.work_label = None
        return 'succeeded'

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
#待機中処理クラス.                        #
###########################################
class PAUSED_WORK(CollaborationState):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition, outcomes):
        super().__init__(label, tran, outcomes)
        self.taskfin = taskfin

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

        #ここまで.
        return 'succeeded'

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
        self.taskfin = taskfin

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

        #ここまで.
        return 'succeeded'