#!/usr/bin/env python3
# coding: UTF-8

import moveit_commander
import tf
from CollaborationToolModule import CollaborationTool
from geometry_msgs.msg import PoseStamped
from CollaborationUserDefineModule import *

MOVE_GROUP_COMMAND_NAME = 'motoman_gp8'
MOVE_FRAME_NAME = 'base_link'
MOVE_LINK_NAME = 'grasp_point'
MOVE_PLANNER_ID = 'RRTConnectkConfigDefault'
MOVE_PLANNER_VALUE = True

#MovePlannerクラス.
class MovePlanner:
    """
    Classes that plan and control movement
    Define machine-dependent packages
    """
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(MOVE_GROUP_COMMAND_NAME)
        self.group.set_pose_reference_frame(MOVE_FRAME_NAME)
        self.group.set_end_effector_link(MOVE_LINK_NAME)
        self.group.set_planner_id(MOVE_PLANNER_ID)
        self.group.allow_replanning(MOVE_PLANNER_VALUE)


    def grasp_position(self, x, y, z, velocity):
        pose = self.target_pose(x, y, z)
        plan = self.pose_plan(pose, velocity)
        return self.plan_exec(plan, "IK can't be solved")


    def current_position(self, offset_x, offset_y, offset_z, velocity):
        x = self.group.get_current_pose().pose.position.x + offset_x
        y = self.group.get_current_pose().pose.position.y + offset_y
        z = self.group.get_current_pose().pose.position.z + offset_z
        pose = self.target_pose(x, y, z)
        plan = self.pose_plan(pose, velocity)
        return self.plan_exec(plan, "IK can't be solved")


    def tf_position(self, offset_x, offset_y, offset_z, velocity, tf_pose):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform("/base_link", tf_pose, CollaborationTool.create_time(0), CollaborationTool.create_duration(4.0))
            (trans,rot) = listener.lookupTransform('/base_link', tf_pose, CollaborationTool.create_time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            CollaborationTool.loginfo("Not found frame...")
            return False
        pose = self.target_pose(trans[0] + offset_x, trans[1] + offset_y, offset_z)
        plan = self.pose_plan(pose, velocity)
        return self.plan_exec(plan, "IK can't be solved")


    def joint_value(self, joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t, vel):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = joint_1_s
        joint_goal[1] = joint_2_l
        joint_goal[2] = joint_3_u
        joint_goal[3] = joint_4_r
        joint_goal[4] = joint_5_b
        joint_goal[5] = joint_6_t
        self.group.set_joint_value_target(joint_goal)       
        self.group.set_max_velocity_scaling_factor(vel)
        plan = self.plan()
        return self.plan_exec(plan,"can't be solved lifter ik")


    def initial_pose(self):
        joint_goal = self.group.get_current_joint_values()
        for i in range(0,len(joint_goal)):
            joint_goal[i] = 0
        joint_goal[4] = -1.57
        self.group.set_joint_value_target(joint_goal)
        plan = self.plan()
        return self.plan_exec(plan,"IK can't be solved")
        
    def target_pose(self, x, y, z):
        """Internal
        Helper method
        """
        quat = tf.transformations.quaternion_from_euler(0, -3.14, -1.57)
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose

    def pose_plan(self, pose, velocity):
        """Internal
        Helper method
        """
        self.group.set_pose_target(pose)
        self.group.set_max_velocity_scaling_factor(velocity)
        return self.plan()
    
    
    def plan(self):
        """Internal
        Helper method
        """
        plan = self.group.plan()
        if type(plan) is tuple:
            plan = plan[1]
        return plan

    def plan_exec(self, plan, warning):
        """Internal
        Helper method
        """
        if len(plan.joint_trajectory.points) == 0:
            CollaborationTool.logwarn(warning)
            self.group.clear_pose_targets()
            return False
        else:
            self.group.execute(plan)
            return True
