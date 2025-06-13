#!/usr/bin/env python3
# coding: UTF-8

import moveit_commander
import tf
from CollaborationToolModule import CollaborationTool
from geometry_msgs.msg import PoseStamped
from CollaborationUserDefineModule import *
import math

MOVE_GROUP_COMMAND_UPPER_BODY = 'upper_body'
MOVE_GROUP_COMMAND_LIFTER = 'lifter'
MOVE_FRAME_NAME = 'base_link'
MOVE_LINK_NAME = 'head_link'
MOVE_PLANNER_ID = 'RRTConnectkConfigDefault'
MOVE_PLANNER_VALUE = True
MOVE_PLANNER_TIMEOUT = 7
MAX_VELOCITY = 1.0

#MovePlannerクラス.
#MovePlannerクラス.
class MovePlanner:
    """
    Classes that plan and control movement
    Define machine-dependent packages
    """
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.upper_body_group = moveit_commander.MoveGroupCommander(MOVE_GROUP_COMMAND_UPPER_BODY)
        self.upper_body_group.set_pose_reference_frame(MOVE_FRAME_NAME)
        self.upper_body_group.set_planner_id(MOVE_PLANNER_ID)
        self.upper_body_group.allow_replanning(MOVE_PLANNER_VALUE)
        self.upper_body_group.set_planning_time(MOVE_PLANNER_TIMEOUT)

        self.lifter_group = moveit_commander.MoveGroupCommander(MOVE_GROUP_COMMAND_LIFTER)
        self.lifter_group.set_pose_reference_frame(MOVE_FRAME_NAME)
        self.lifter_group.set_planner_id(MOVE_PLANNER_ID)
        self.lifter_group.allow_replanning(MOVE_PLANNER_VALUE)
        self.lifter_group.set_planning_time(MOVE_PLANNER_TIMEOUT)

    def grasp_position_upper_body_group(self, x, y, z, ox, oy, oz, ow, velocity):
        pose = self.target_pose(x, y, z, ox, oy, oz, ow)
        print("")
        print(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        print(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        print("")
        plan = self.pose_plan(self.upper_body_group, pose, velocity)
        return self.plan_exec(self.upper_body_group, plan, "IK can't be solved")
    
    def grasp_position_lifter(self, x, y, z, ox, oy, oz, ow, velocity):
        pose = self.target_pose(x, y, z, ox, oy, oz, ow)
        print("")
        print(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        print(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        print("")
        plan = self.pose_plan(self.lifter_group, pose, velocity)
        return self.plan_exec(self.lifter_group, plan, "IK can't be solved")

    def current_position_upper_body_group(self, offset_x, offset_y, offset_z,  offset_ox, offset_oy, offset_oz, offset_ow, velocity):
        x = self.upper_body_group.get_current_pose().pose.position.x + offset_x
        y = self.upper_body_group.get_current_pose().pose.position.y + offset_y
        z = self.upper_body_group.get_current_pose().pose.position.z + offset_z
        ox = self.upper_body_group.get_current_pose().pose.orientation.x + offset_ox
        oy = self.upper_body_group.get_current_pose().pose.orientation.y + offset_oy
        oz = self.upper_body_group.get_current_pose().pose.orientation.z + offset_oz
        ow = self.upper_body_group.get_current_pose().pose.orientation.w + offset_ow
        pose = self.target_pose(x, y, z, ox, oy, oz, ow)
        print("")
        print(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        print(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        print("")
        plan = self.pose_plan(self.torso_group, pose, velocity)
        return self.plan_exec(self.torso_group, plan, "IK can't be solved")

    def current_position_lifter(self, offset_x, offset_y, offset_z,  offset_ox, offset_oy, offset_oz, offset_ow, velocity):
        x = self.lifter_group.get_current_pose().pose.position.x + offset_x
        y = self.lifter_group.get_current_pose().pose.position.y + offset_y
        z = self.lifter_group.get_current_pose().pose.position.z + offset_z
        ox = self.lifter_group.get_current_pose().pose.orientation.x + offset_ox
        oy = self.lifter_group.get_current_pose().pose.orientation.y + offset_oy
        oz = self.lifter_group.get_current_pose().pose.orientation.z + offset_oz
        ow = self.lifter_group.get_current_pose().pose.orientation.w + offset_ow
        pose = self.target_pose(x, y, z, ox, oy, oz, ow)
        print("")
        print(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        print(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        print("")
        plan = self.pose_plan(self.lifter_group, pose, velocity)
        return self.plan_exec(self.lifter_group, plan, "IK can't be solved")

    def tf_position_upper_body(self, offset_x, offset_y, offset_z, offset_ox, offset_oy, offset_oz, offset_ow, velocity, tf_pose):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform("/base_link", tf_pose, CollaborationTool.create_time(0), CollaborationTool.create_duration(4.0))
            (trans,rot) = listener.lookupTransform('/base_link', tf_pose, CollaborationTool.create_time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            CollaborationTool.loginfo("Not found frame...")
            return False
        pose = self.target_pose(trans[0] + offset_x, trans[1] + offset_y, offset_z, offset_ox, offset_oy, offset_oz, offset_ow)
        plan = self.pose_plan(self.upper_body_group, pose, velocity)
        return self.plan_exec(self.upper_body_group, plan, "IK can't be solved")
    
    def tf_position_lifter(self, offset_x, offset_y, offset_z, offset_ox, offset_oy, offset_oz, offset_ow, velocity, tf_pose):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform("/base_link", tf_pose, CollaborationTool.create_time(0), CollaborationTool.create_duration(4.0))
            (trans,rot) = listener.lookupTransform('/base_link', tf_pose, CollaborationTool.create_time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            CollaborationTool.loginfo("Not found frame...")
            return False
        pose = self.target_pose(trans[0] + offset_x, trans[1] + offset_y, offset_z, offset_ox, offset_oy, offset_oz, offset_ow)
        plan = self.pose_plan(self.lifter_group, pose, velocity)
        return self.plan_exec(self.lifter_group, plan, "IK can't be solved")

    def joint_value_upper_body(self, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7, joint_8, joint_9, joint_10,
                               joint_11, joint_12, joint_13, joint_14, joint_15, joint_16, joint_17, joint_18, joint_19, joint_20,
                               joint_21, joint_22, joint_23, joint_24, joint_25, joint_26,  vel):
        joint_goal = self.upper_body_group.get_current_joint_values()
        joint_goal[0] = math.radians(joint_1) #waist_y
        joint_goal[1] = math.radians(joint_2) #waist_p
        joint_goal[2] = math.radians(joint_3) #waist_r
        joint_goal[3] = math.radians(joint_4) #l_shoulder_p
        joint_goal[4] = math.radians(joint_5) #l_shoulder_r
        joint_goal[5] = math.radians(joint_6) #l_shoulder_y
        joint_goal[6] = math.radians(joint_7) #l_elbow
        joint_goal[7] = math.radians(joint_8) #l_elbow_joint_mimic
        joint_goal[8] = math.radians(joint_9) #l_elbow_middle_joint
        joint_goal[9] = math.radians(joint_10) #l_elbow_middle_joint_mimic
        joint_goal[10] = math.radians(joint_11) #l_wrist_y
        joint_goal[11] = math.radians(joint_12) #l_wrist_p
        joint_goal[12] = math.radians(joint_13) #l_wrist_r
        joint_goal[13] = math.radians(joint_14) #neck_y
        joint_goal[14] = math.radians(joint_15) #neck_p
        joint_goal[15] = math.radians(joint_16) #neck_r
        joint_goal[16] = math.radians(joint_17) #r_shoulder_p
        joint_goal[17] = math.radians(joint_18) #r_shoulder_r
        joint_goal[18] = math.radians(joint_19) #r_shoulder_y
        joint_goal[19] = math.radians(joint_20) #r_elbow
        joint_goal[20] = math.radians(joint_21) #r_elbow_joint_mimic
        joint_goal[21] = math.radians(joint_22) #r_elbow_middle_joint
        joint_goal[22] = math.radians(joint_23) #r_elbow_middle_joint_mimic
        joint_goal[23] = math.radians(joint_24) #r_wrist_y
        joint_goal[24] = math.radians(joint_25) #r_wrist_p
        joint_goal[25] = math.radians(joint_26) #r_wrist_r
        print(joint_goal)
        self.upper_body_group.set_joint_value_target(joint_goal)       
        self.upper_body_group.set_max_velocity_scaling_factor(vel)
        plan = self.plan(self.upper_body_group)
        return self.plan_exec(self.upper_body_group, plan,"can't be solved lifter ik")
    
    def joint_value_lifter(self, joint_1, joint_2, joint_3, joint_4, vel):
        joint_goal = self.lifter_group.get_current_joint_values()
        joint_goal[0] = math.radians(joint_1) #ankle_joint
        joint_goal[1] = math.radians(joint_2) #ankle_joint_mimic
        joint_goal[2] = math.radians(joint_3) #knee_joint
        joint_goal[3] = math.radians(joint_4) #knee_joint_mimic
        print(joint_goal)
        self.lifter_group.set_joint_value_target(joint_goal)       
        self.lifter_group.set_max_velocity_scaling_factor(vel)
        plan = self.plan(self.lifter_group)
        return self.plan_exec(self.lifter_group, plan,"can't be solved lifter ik")

    def initial_pose_upper_body(self):
        joint_goal = self.upper_body_group.get_current_joint_values()
        for i in range(0,len(joint_goal)):
            joint_goal[i] = math.radians(0)
        joint_goal[6] = math.radians(-170)
        joint_goal[19] = math.radians(-170)
        print(joint_goal)
        self.upper_body_group.set_joint_value_target(joint_goal)
        plan = self.plan(self.upper_body_group)
        return self.plan_exec(self.upper_body_group, plan,"IK can't be solved")
    
    def initial_pose_lifter(self):
        joint_goal = self.lifter_group.get_current_joint_values()
        for i in range(0,len(joint_goal)):
            joint_goal[i] = math.radians(0)
        print(joint_goal)
        self.upper_body_group.set_max_velocity_scaling_factor(MAX_VELOCITY)
        self.lifter_group.set_joint_value_target(joint_goal)
        plan = self.plan(self.lifter_group)
        return self.plan_exec(self.lifter_group, plan,"IK can't be solved")
        
    def target_pose(self, x, y, z, ox, oy, oz, ow):
        """Internal
        Helper method
        """
        #quat = tf.transformations.quaternion_from_euler(0, -3.14, -1.57)
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.orientation.x = ox
        pose.pose.orientation.y = oy
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose

    def pose_plan(self, group, pose, velocity):
        """Internal
        Helper method
        """
        group.set_pose_target(pose)
        group.set_max_velocity_scaling_factor(velocity)
        return self.plan(group)
    
    
    def plan(self, group):
        """Internal
        Helper method
        """
        plan = group.plan()
        if type(plan) is tuple:
            plan = plan[1]
        return plan

    def plan_exec(self, group, plan, warning):
        """Internal
        Helper method
        """
        if len(plan.joint_trajectory.points) == 0:
            CollaborationTool.logwarn(warning)
            group.clear_pose_targets()
            return False
        else:
            group.execute(plan)
            return True