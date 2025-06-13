#!/usr/bin/env python3
# coding: UTF-8

import moveit_commander
import tf
from CollaborationToolModule import CollaborationTool
from geometry_msgs.msg import PoseStamped
from CollaborationUserDefineModule import *
import math

MOVE_GROUP_COMMAND_NAME = 'arm'
MOVE_FRAME_NAME = 'base_link'
MOVE_LINK_NAME = 'crane_x7_gripper_base_link'
MOVE_PLANNER_ID = 'RRTConnectkConfigDefault'
MOVE_PLANNER_VALUE = True
MOVE_PLANNER_TIMEOUT = 7

GRIPPER_GROUP_COMMAND_NAME = 'gripper'

INIT_VALUE_1 = -80.0
INIT_VALUE_2 = 22.0
INIT_VALUE_3 = 0.0
INIT_VALUE_4 = -131.0
INIT_VALUE_5 = -3.0
INIT_VALUE_6 = 27.0
INIT_VALUE_7 = 0.0
INIT_VALUE_VEL = 1.0

HAND_OPEN_VALUE= 40.0
HAND_CLOSE_VALUE= 0.3
HAND_VEL= 0.01

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
        self.group.set_planning_time(MOVE_PLANNER_TIMEOUT)

        self.gripper = moveit_commander.MoveGroupCommander(GRIPPER_GROUP_COMMAND_NAME)
        self.gripper.set_pose_reference_frame(MOVE_FRAME_NAME)
        self.gripper.set_planner_id(MOVE_PLANNER_ID)
        self.gripper.allow_replanning(MOVE_PLANNER_VALUE)
        self.gripper.set_planning_time(MOVE_PLANNER_TIMEOUT)


    def grasp_position(self, x, y, z, ox, oy, oz, ow, velocity):
        pose = self.target_pose(x, y, z, ox, oy, oz, ow)
        print("")
        print(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        print(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        print("")
        plan = self.pose_plan(self.group, pose, velocity)
        return self.plan_exec(self.group, plan, "IK can't be solved")


    def current_position(self, offset_x, offset_y, offset_z,  offset_ox, offset_oy, offset_oz, offset_ow, velocity):
        x = self.group.get_current_pose().pose.position.x + offset_x
        y = self.group.get_current_pose().pose.position.y + offset_y
        z = self.group.get_current_pose().pose.position.z + offset_z
        ox = self.group.get_current_pose().pose.orientation.x + offset_ox
        oy = self.group.get_current_pose().pose.orientation.y + offset_oy
        oz = self.group.get_current_pose().pose.orientation.z + offset_oz
        ow = self.group.get_current_pose().pose.orientation.w + offset_ow
        pose = self.target_pose(x, y, z, ox, oy, oz, ow)
        print("")
        print(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        print(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        print("")
        plan = self.pose_plan(self.group, pose, velocity)
        return self.plan_exec(self.group, plan, "IK can't be solved")

    def tf_position(self, offset_x, offset_y, offset_z, offset_ox, offset_oy, offset_oz, offset_ow, velocity, tf_pose):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform("/base_link", tf_pose, CollaborationTool.create_time(0), CollaborationTool.create_duration(4.0))
            (trans,rot) = listener.lookupTransform('/base_link', tf_pose, CollaborationTool.create_time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            CollaborationTool.loginfo("Not found frame...")
            return False
        self.n = 0
        for i in trans:
            trans[self.n] = round(i, 5)
            self.n = self.n + 1

        print(trans)
        print(self.group.get_goal_orientation_tolerance())
        pose = self.target_pose(trans[0] + offset_x, trans[1] + offset_y, offset_z, offset_ox, offset_oy, offset_oz, offset_ow)
        plan = self.pose_plan(self.group, pose, velocity)
        print(pose)
        return self.plan_exec(self.group, plan, "IK can't be solved")

    def joint_value(self, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7, vel):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = math.radians(joint_1) #crane_x7_shoulder_arm_fixed_part_joint
        joint_goal[1] = math.radians(joint_2) #crane_x7_shoulder_arm_revolute_part_joint
        joint_goal[2] = math.radians(joint_3) #crane_x7_upper_arm_fixed_part_joint
        joint_goal[3] = math.radians(joint_4) #crane_x7_upper_arm_revolute_part_joint
        joint_goal[4] = math.radians(joint_5) #crane_x7_lower_arm_fixed_part_joint
        joint_goal[5] = math.radians(joint_6) #crane_x7_lower_arm_revolute_part_joint
        joint_goal[6] = math.radians(joint_7) #crane_x7_wrist_joint
        self.group.set_joint_value_target(joint_goal)
        self.group.set_max_velocity_scaling_factor(vel)
        print(joint_goal)
        plan = self.plan(self.group)
        return self.plan_exec(self.group, plan,"can't be solved lifter ik")
    
    def gripper_value(self, joint_1, joint_2, vel):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[0] = math.radians(joint_1) #crane_x7_gripper_finger_a_joint
        joint_goal[1] = math.radians(joint_2) #crane_x7_gripper_finger_b_joint
        self.gripper.set_joint_value_target(joint_goal)
        self.gripper.set_max_velocity_scaling_factor(vel)
        print(joint_goal)
        plan = self.plan(self.gripper)
        return self.plan_exec(self.gripper, plan,"can't be solved lifter ik")
    
    def gripper_open(self):
        return self.gripper_value(HAND_OPEN_VALUE, HAND_OPEN_VALUE, HAND_VEL)
    
    def gripper_close(self):
        return self.gripper_value(HAND_CLOSE_VALUE, HAND_CLOSE_VALUE, HAND_VEL)

    def initial_pose(self):
        return self.joint_value(INIT_VALUE_1, INIT_VALUE_2, INIT_VALUE_3, INIT_VALUE_4, INIT_VALUE_5, INIT_VALUE_6, INIT_VALUE_7, INIT_VALUE_VEL)
        
    def target_pose(self, x, y, z, ox, oy, oz, ow):
        """Internal
        Helper method
        """
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

    def target_position(self, x, y, z):
        """Internal
        Helper method
        """
        position = [x, y, z]
        return position

    def pose_plan(self, group, pose, velocity):
        """Internal
        Helper method
        """
        group.clear_pose_targets()
        group.set_pose_target(pose)
        group.set_max_velocity_scaling_factor(velocity)
        return self.plan(group)
    
    def position_plan(self, group, xyz, velocity):
        """Internal
        Helper method
        """
        self.group.clear_pose_targets()
        self.group.set_position_target(xyz)
        self.group.set_max_velocity_scaling_factor(velocity)
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