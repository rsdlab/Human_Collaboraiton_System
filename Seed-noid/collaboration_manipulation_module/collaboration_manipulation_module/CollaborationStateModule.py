#!/usr/bin/env python3
# coding: UTF-8

import smach_ros
from smach import Concurrence, StateMachine
from std_msgs.msg import Int16

from .CollaborationCommunicationModule import *
from .CollaborationEventModule import (
    CollaborationEventSubscriver,
    CollaborationEventSubscriverWorkStart,
)
from .CollaborationStateHeader import *
from .CollaborationToolModule import CollaborationTool
from .CollaborationUserDefineModule import *
from .EnumerateModule import EnumEvent, EnumState
from .TransitionModule import (
    FinalizingTran,
    InitializingTran,
    NonOperatingTran,
    OperatingTran,
    PausedTran,
    PreparingTran,
    RunningTran,
    StandbyTran,
    Transition,
    WorkingTran,
)


#コマンド取得.
class GET_COMMAND(CollaborationState):
    def __init__(self, label, tran:Transition, workstartevt, outcomes):
        super().__init__(label, tran, outcomes)
        self.workstartevt = workstartevt

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate

        if self.preempt('State GET_COMMAND is being preempted!!!'):
            return 'preempted'

        #キューからデータを取得
        CollaborationTool.loginfo("Attempting to dequeue...")
        dequeued_obj = self.workstartevt.dequeue()
        CollaborationTool.loginfo(f"Dequeued object: {dequeued_obj}")

        #現在のコマンドリストを記憶.
        CollaborationCurrentData.SetCurrentCommandList(dequeued_obj)
        return 'succeeded'

#作業開始.
class WORKING_START(CollaborationState):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition):
        super().__init__(label, tran,
            ['succeeded','suspended','workend','paused','end','Operating','NonOperating'])
        self.taskfin = taskfin

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        #現在のコマンドリストを取得.
        command_list = CollaborationCurrentData.GetCurrentCommandList()
        CollaborationTool.loginfo(command_list.get(0).task_command_id)
        
        # setting history state
        if self.tran.get_event().is_pause():
            CollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())
        elif self.tran.get_event().e_workend():
            task_command = command_list.get(0)
            self.taskfin.execute(task_command)

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        history_state = EnumState(self.tran.get_history_state())
        if history_state.is_operating() :
            self.tran.reset_history_state()             
            return "Operating"
        elif history_state.is_nonoperating() :
            self.tran.reset_history_state()             
            return "NonOperating"
        
        return 'succeeded'

#作業開始イベント待ち.
class WAIT_START(CollaborationState):
    def __init__(self, label, tran:Transition, workstartevt, outcomes):
        super().__init__(label, tran, outcomes)
        self.workstartevt = workstartevt

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        if self.preempt('State SYSTEM_CALL is being preempted!!!'):
            return 'preempted'
        
        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        
        CollaborationTool.loginfo('Wait starting ...')
        while not CollaborationTool.is_shutdown():
            if(0 < self.workstartevt.getcount()):
                CollaborationTool.loginfo('Wait compreted ...')
                return 'succeeded'
            elif(self.tran.get_event().is_end()):
                CollaborationTool.loginfo('Wait end ...')
                return 'end'
            else:
                CollaborationTool.wait_time(0.01)
        CollaborationTool.loginfo('Wait shutdown ...')
        return 'aborted'

class MonitorState(smach_ros.MonitorState, Node):
    def __init__(self, label, taskfin:TaskFinishServer, tran:Transition, topic, msg_type, cond_cb, node_name):
        Node.__init__(self, node_name)
        smach_ros.MonitorState.__init__(self, self, topic, msg_type, cond_cb, -1)
        self.tran = tran
        self.label = label
        self.taskfin = taskfin
        
    def set_event(self, event:EnumEvent):
        self.tran.set_event(event)

    def service_delete(self):
        # ServiceServerExecutor.shutdown() は登録された全ノードに service_delete() を
        # 呼ぶ. MonitorState はサービスを持たないため no-op を用意しておく
        # （これが無いと終了時に AttributeError になる）.
        pass

    def execute(self, userdata):
        if self.tran.get_event().is_workend():
            #現在のコマンドリストを取得.
            task_command_list = CollaborationCurrentData.GetCurrentCommandList()
            task_command = task_command_list.get(0)
            self.taskfin.execute(task_command)
        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate
        return super().execute(userdata)
    
    def get_state(self):
        return self.tran.get_state()

#作業結果の送信
class RESULT_RECEIVE(CollaborationState):
    def __init__(self,
                 label,
                 taskresult:NotifyTaskResultClient,
                 taskcomp:NotifyTaskCompleteClient,
                 taskspd:TaskCancelServer,
                 taskfin:TaskFinishServer,
                 tran:Transition):
        super().__init__(label, tran, ['succeeded','aborted'])
        self.taskresult = taskresult
        self.taskcomp = taskcomp
        self.taskspd = taskspd
        self.taskfin = taskfin

    def execute(self, userdata):
        CollaborationCurrentData.SetCurrentState(self)
        # setting history state
        if self.tran.get_event().is_pause():
            CollaborationState.set_history_label(self.label)
            self.tran.set_history_state(self.tran.get_state())

        if self.tran.is_tran() :
            nextstate = self.tran.transition()
            self.tran.reset_event()
            return nextstate

        #現在のコマンドリストを取得.
        task_command_list = CollaborationCurrentData.GetCurrentCommandList()
        task_command = task_command_list.get(0)
        if self.tran.get_event().is_workstart():
            self.taskresult.execute(task_command, task_command.task_command_id, task_command.picking_count , True)
            self.taskcomp.execute(task_command)
        elif self.tran.get_event().is_worksuspend():
            self.taskspd.execute(task_command)
        elif self.tran.get_event().is_workend():
            self.taskfin.execute(task_command)

        return 'succeeded'

#状態遷移の具象クラス.
class HumanCollaborationStateMachine(CollaborationEventSubscriver):
    """
    Base class for HumanCollaboration statemachine.

    Attributes
    ----------
    statemachine : StateMachine
        state machine class
    """
    def __init__(self, 
            taskresult:NotifyTaskResultClient,
            taskcomp:NotifyTaskCompleteClient,
            tasksuspend:TaskCancelServer,
            taskfinal:TaskFinishServer,
            taskgetstate:GetStatusServer,
            area:PeripheralEnvironmentAreaSetClient,
            disc:PlacePositionClient,
            workd:WorkDetectionClient,
            workstartevt:CollaborationEventSubscriverWorkStart
            ):
        self.taskresult     = taskresult
        self.taskcomp       = taskcomp
        self.tasksuspend    = tasksuspend
        self.taskfinal      = taskfinal
        self.taskgetstate   = taskgetstate
        self.workstartevt   = workstartevt
        self.area           = area
        self.disc           = disc
        self.workd          = workd
        # MonitorState のノードは executor で spin しないと購読が動かないため、
        # ここに集めておき、呼び出し側で executor に追加する.
        self.monitor_nodes = []
        self.statemachine = self.set_statemachine()

        #smach_viwerによって状態遷移を可視化する.
        self.sis = smach_ros.IntrospectionServer(
            'smach_server', self.statemachine, '/SM_ROOT')

    def execute(self):
        self.sis.start()
        self.statemachine.execute()
        self.sis.stop()

    #コマンド受信.
    def update(self, event:EnumEvent, data):
        state = CollaborationCurrentData.GetCurrentState()
        state.tran.set_event(event)
        if event.is_getstate():
            self.taskgetstate.execute(state.get_state())
        CollaborationTool.loginfo(state.tran.event)

    def set_statemachine(self):
        """helper function

        Returns
        -------
        module_play : StateMachine
        state machine instanse
        """

        # Operating(自律動作中)
        Operating = StateMachine(outcomes=['succeeded','aborted','preempted','suspended','workend','paused','collabo','end'])
        with Operating:
            Operating.add('OPERATING_WORK', OPERATING_WORK('OPERATING_WORK', self.taskfinal, OperatingTran(),
                                                           ['succeeded','aborted','preempted','suspended','workend','paused','collabo','end']), 
                transitions={'succeeded':'succeeded',
                             'aborted':'aborted',
                             'preempted':'preempted',
                             'suspended':'suspended',
                             'workend':'workend',
                             'paused':'paused',
                             'collabo':'collabo',
                             'end':'end'})

        # NonOperating(非自律動作中)
        NonOperating = StateMachine(outcomes=['succeeded','aborted','preempted','suspended','paused','workend','collaboend','end'])
        with NonOperating:
            NonOperating.add('NONOPERATING_START', NON_OPERATING_WORK('NONOPERATING_START', self.taskfinal, NonOperatingTran(),
                             ['succeeded','suspended','paused','workend','collaboend','end']),
                transitions={'succeeded':'succeeded',
                             'suspended':'suspended',
                             'paused':'paused',
                             'workend':'workend',
                             'collaboend':'collaboend',
                             'end':'end'})

        # Woring(作業進行中)
        Working = StateMachine(outcomes=['succeeded','aborted','preempted','suspended','workend','paused','end'])
        with Working:
            Working.add('WorkingStart', WORKING_START('WorkingStart', self.taskfinal, WorkingTran()), 
                transitions={'succeeded':'Operating',
                             'Operating':'Operating',      # history state
                             'NonOperating':'NonOperating', # history state
                             'suspended':'suspended',
                             'workend':'workend',
                             'paused':'paused',
                             'end':'end'})
            Working.add('Operating',  Operating, 
                transitions={'succeeded':'RESULT RECEIVE',
                             'aborted':'aborted',
                             'suspended':'suspended',
                             'workend':'workend',
                             'paused':'paused',
                             'collabo':'NonOperating',
                             'end':'end'})
            Working.add('NonOperating',  NonOperating, 
                transitions={'succeeded':'RESULT RECEIVE',
                             'aborted':'aborted',
                             'suspended':'suspended',
                             'workend':'workend',
                             'paused':'paused',
                             'collaboend':'Operating',
                             'end':'end'})

        #作業結果の送信
            Working.add('RESULT RECEIVE', 
                RESULT_RECEIVE('RESULT RECEIVE', self.taskresult, self.taskcomp, self.tasksuspend, self.taskfinal, WorkingTran()), 
                transitions={'succeeded':'succeeded','aborted':'aborted'})

        # 一時停止中
        Paused = StateMachine(outcomes=['succeeded','aborted','preempted','suspended','workend','pausecancel','end'])
        with Paused:
            wait_monitor = MonitorState(label='WAIT', taskfin=self.taskfinal, tran=PausedTran(), topic="/stop_judge", msg_type=String, cond_cb=resumemove_cb, node_name = 'wait_monitor_node')
            self.monitor_nodes.append(wait_monitor)
            Paused.add('WAIT', wait_monitor,
                transitions={'invalid':'WAIT',
                             'valid':'PAUSED WORK',
                             'preempted':'WAIT'})
            Paused.add('PAUSED WORK', PAUSED_WORK('PAUSED WORK', self.taskfinal, PausedTran(),
                            ['succeeded',
                             'aborted',
                             'preempted',
                             'suspended',
                             'workend',
                             'pausecancel',
                             'end']),
                transitions={'succeeded':'succeeded',
                             'aborted':'aborted',
                             'preempted':'preempted',
                             'suspended':'suspended',
                             'workend':'workend',
                             'pausecancel':'pausecancel',
                             'end':'end'})
        
#################################################################
#　　　　　　　　　　　 準備中動作               　　           #
#################################################################
        Preparing = StateMachine(outcomes=['succeeded','aborted','preempted','suspended','workend','end'])
        
        with Preparing:
            Preparing.add('PREPAR', PREPAR_WORK('PREPAR', self.taskfinal, PreparingTran(), self.area, self.disc, self.workd,
                            ['succeeded','aborted','preempted','suspended','workend','end']),
                transitions={'succeeded':'succeeded',
                             'aborted':'aborted',
                             'suspended':'suspended',
                             'workend':'workend',
                             'end':'end'})

#################################################################
#　　　　　　　　　　　 実行中動作               　　           #
#################################################################
        Running = StateMachine(outcomes=['succeeded','aborted','preempted','suspended','workend','end'])
        with Running:
            Running.add('Preparing', Preparing, 
                transitions={'succeeded':'Working',
                             'preempted':'preempted',
                             'aborted':'aborted',
                             'suspended':'suspended',
                             'workend':'workend',
                             'end':'end'})
            Running.add('Working', Working, 
                transitions={'succeeded':'succeeded',
                             'preempted':'preempted',
                             'aborted':'aborted',
                             'suspended':'suspended',
                             'workend':'workend',
                             'paused':'Paused',
                             'end':'end'})
        #一時停止
            Running.add('Paused', Paused,
                transitions={'succeeded':'Working',
                             'aborted':'aborted',
                             'preempted':'preempted',
                             'suspended':'suspended',
                             'workend':'workend',
                             'pausecancel':'Working',
                             'end':'end'})

#################################################################
#　　　　　　　　　　　 待機中動作               　　           #
#################################################################
        Standby = StateMachine(outcomes=['succeeded','aborted','preempted','end'])
        with Standby:
            #作業開始コマンドを待つ.  
            Standby.add('WAIT_START',  WAIT_START('WAIT_START', StandbyTran(), self.workstartevt,
                            ['succeeded',
                             'aborted',
                             'preempted',
                             'workstart',
                             'end']),
                transitions={'succeeded':'BEFORE START',
                             'aborted':'aborted',
                             'preempted':'preempted',
                             'workstart':'BEFORE START',
                             'end':'end'})
            Standby.add('BEFORE START', BEFORE_START('BEFORE START', self.taskfinal, StandbyTran(),
                            ['succeeded',
                             'aborted',
                             'preempted',
                             'workstart',
                             'end']),
                transitions={'succeeded':'GET COMMAND',
                             'aborted':'aborted',
                             'preempted':'preempted',
                             'workstart':'GET COMMAND',
                             'end':'end'})
            Standby.add('GET COMMAND', GET_COMMAND('GET COMMAND', StandbyTran(), self.workstartevt,
                                                   ['succeeded','preempted','end']), 
                transitions={'succeeded':'succeeded',
                             'preempted':'preempted',
                             'end':'end'})

#################################################################
#　　　　　　　　　　　 並列処理内容　                          #
#################################################################
        concur = Concurrence(outcomes=['succeeded','aborted','preempted','suspended','workend','end','motion_stop'],
                            default_outcome='succeeded',
                            child_termination_cb=child_manip_cb,
                            outcome_cb=out_manip_cb)
        with concur:
            concur.add('Running', Running)
            human_detect = MonitorState('HUMAN DETECT', self.taskfinal, RunningTran(),'/intrusion_result', Int16, humandetect_cb, node_name = 'human_detect_node')
            self.monitor_nodes.append(human_detect)
            concur.add('HUMAN DETECT', human_detect)

##################################################################
#　　　　　　　　　　　 メイン処理内容                           #
##################################################################
        module_play = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with module_play:
            #初期化動作
            module_play.add('Initializing',
                INITIALISE('Initializing', self.taskfinal, InitializingTran(),
                   outcomes=['succeeded',
                             'retry',
                             'aborted',
                             'preempted']), 
                transitions={'succeeded':'Standby','retry':'Initializing','aborted':'aborted'})
            #待機中
            module_play.add('Standby',  Standby, 
                transitions={'succeeded':'Running',
                             'preempted':'preempted',
                             'aborted':'aborted',
                             'end':'Finalizing'})
            #実行中（人検知の並行監視つき：concur = Running + HUMAN DETECT）
            module_play.add('Running', concur,
                transitions={'succeeded':'Standby',
                             'aborted':'aborted',
                             'preempted':'preempted',
                             'suspended':'Standby',
                             'workend':'Standby',
                             'motion_stop':'Standby',
                             'end':'Finalizing'})
            #終了処理中
            module_play.add('Finalizing', 
                FINALISE('Finalizing', self.taskfinal, FinalizingTran(),
                   outcomes=['succeeded',
                             'retry',
                             'aborted',
                             'preempted']), 
                transitions={'succeeded':'succeeded','retry':'Finalizing','aborted':'aborted','preempted':'preempted'})
        return module_play
    
 #人検知実行
def child_manip_cb(outcome_map):
    # 人を検知したら(HUMAN DETECT が invalid)、並行処理を終了させて停止する.
    if outcome_map['HUMAN DETECT'] == 'invalid':
        return True
    # Running 側が何らかの終了状態に達したら、並行処理を終了する.
    if outcome_map['Running'] is not None:
        return True
    return False

def out_manip_cb(outcome_map):
    # 人検知で止まった場合は motion_stop を返す.
    if outcome_map['HUMAN DETECT'] == 'invalid':
        return 'motion_stop'
    # それ以外は Running の終了種別(succeeded/workend/suspended/end など)をそのまま返す.
    if outcome_map['Running'] is not None:
        return outcome_map['Running']
    return 'succeeded'
    
def humandetect_cb(ud, msg):
    # 人検知時の安全停止は MovePlanner 側のゲート（_wait_while_human_present）に一本化した.
    # ここで invalid を返すと旧来の「motion_stop → Standby（作業破棄してやり直し）」に戻るため、
    # 常に True を返して HUMAN DETECT では状態を止めない（＝一時停止＆途中再開を優先する）.
    return True

def resumemove_cb(ud, msg):
    if msg.data == False:
      return False
    else:
      return True
    