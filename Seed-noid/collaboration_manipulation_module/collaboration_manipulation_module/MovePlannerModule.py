#!/usr/bin/env python3
# coding: UTF-8

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from collaboration_manipulation_module.CollaborationToolModule import CollaborationTool

MOVE_GROUP_UPPER_BODY = 'upper_body'
MOVE_GROUP_LIFTER = 'lifter'
MOVE_FRAME_NAME = 'base_link'
MOVE_LINK_NAME = 'head_link'
MOVE_PLANNER_TIMEOUT = 7.0

INITIALIZE_UPPER_BODY_ELBOW_L = -170.0
INITIALIZE_UPPER_BODY_ELBOW_R = -170.0
UPPER_BODY_ELBOW_L_INDEX = 6   # joint_goal[6] = l_elbow
UPPER_BODY_ELBOW_R_INDEX = 16  # joint_goal[16] = r_elbow
INITIALIZE_VELOCITY = 0.5

INITIALIZE_LIFTER_JOINTS = 0.0


class MovePlanner(Node):
    def __init__(self):
        super().__init__('move_planner')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Waiting for move_action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Connected to move_action server.')

        # 人検知（安全適合監視停止 SMS）.
        # /intrusion_result が 1 (人あり) の間は、次の動作ゴールを出さずに待つ.
        # 人がいなくなれば、止まった所から続きの動作を再開する（タスクは破棄しない）.
        self.human_present = False
        self.create_subscription(Int16, '/intrusion_result', self._intrusion_cb, 10)

    # 人検知トピックのコールバック. 1(人あり)で human_present を立てる.
    def _intrusion_cb(self, msg):
        self.human_present = (msg.data != 0)

    # 人がいる間は次の動作を出さずに待機する（動作の境目で一時停止し、人が去れば再開）.
    def _wait_while_human_present(self):
        if self.human_present:
            self.get_logger().warn('Human detected (/intrusion_result=1). Pausing before next motion...')
            while self.human_present and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info('Human cleared. Resuming motion.')

    def joint_value_upper_body(self,
                               joint_1,  joint_2,  joint_3,  joint_4,  joint_5,
                               joint_6,  joint_7,  joint_8,  joint_9,  joint_10,
                               joint_11, joint_12, joint_13, joint_14, joint_15,
                               joint_16, joint_17, joint_18, joint_19, joint_20, vel=1.0):
        joint_names = [
            'waist_y_joint', 'waist_p_joint', 'waist_r_joint',
            'l_shoulder_p_joint', 'l_shoulder_r_joint', 'l_shoulder_y_joint',
            'l_elbow_joint', 'l_wrist_y_joint', 'l_wrist_p_joint', 'l_wrist_r_joint',
            'neck_y_joint', 'neck_p_joint', 'neck_r_joint',
            'r_shoulder_p_joint', 'r_shoulder_r_joint', 'r_shoulder_y_joint',
            'r_elbow_joint', 'r_wrist_y_joint', 'r_wrist_p_joint', 'r_wrist_r_joint',
        ]
        values_deg = [
            joint_1,  joint_2,  joint_3,  joint_4,  joint_5,
            joint_6,  joint_7,  joint_8,  joint_9,  joint_10,
            joint_11, joint_12, joint_13, joint_14, joint_15,
            joint_16, joint_17, joint_18, joint_19, joint_20,
        ]
        joint_goal = {name: math.radians(val) for name, val in zip(joint_names, values_deg)}
        return self.send_goal_fk(joint_goal, MOVE_GROUP_UPPER_BODY, vel, vel)

    def joint_value_lifter(self, joint_1, joint_2, vel=1.0):
        # ankle_joint, knee_joint (mimic除く)
        joint_names = ['ankle_joint', 'knee_joint']
        values_deg = [joint_1, joint_2]
        joint_goal = {name: math.radians(val) for name, val in zip(joint_names, values_deg)}
        return self.send_goal_fk(joint_goal, MOVE_GROUP_LIFTER, vel, vel)

    def initial_pose_upper_body(self):
        joint_names = [
            'waist_y_joint', 'waist_p_joint', 'waist_r_joint',
            'l_shoulder_p_joint', 'l_shoulder_r_joint', 'l_shoulder_y_joint',
            'l_elbow_joint', 'l_wrist_y_joint', 'l_wrist_p_joint', 'l_wrist_r_joint',
            'neck_y_joint', 'neck_p_joint', 'neck_r_joint',
            'r_shoulder_p_joint', 'r_shoulder_r_joint', 'r_shoulder_y_joint',
            'r_elbow_joint', 'r_wrist_y_joint', 'r_wrist_p_joint', 'r_wrist_r_joint',
        ]
        joint_goal = {name: 0.0 for name in joint_names}
        joint_goal['l_elbow_joint'] = math.radians(INITIALIZE_UPPER_BODY_ELBOW_L)
        joint_goal['r_elbow_joint'] = math.radians(INITIALIZE_UPPER_BODY_ELBOW_R)
        return self.send_goal_fk(joint_goal, MOVE_GROUP_UPPER_BODY, INITIALIZE_VELOCITY, INITIALIZE_VELOCITY)

    def initial_pose_lifter(self):
        joint_names = ['ankle_joint', 'knee_joint']
        joint_goal = {name: math.radians(INITIALIZE_LIFTER_JOINTS) for name in joint_names}
        return self.send_goal_fk(joint_goal, MOVE_GROUP_LIFTER, INITIALIZE_VELOCITY, INITIALIZE_VELOCITY)

    def send_goal_fk(self, joint_goal: dict, group_name: str, vel: float, acc: float,
                     pipeline_id: str = '', planner_id: str = ''):
        # 人がいる間はここで待機（動作の境目で一時停止）.
        self._wait_while_human_present()

        self.get_logger().info(f"Moving {group_name} to goal: {joint_goal}")

        goal_msg = MoveGroup.Goal()
        goal_msg.planning_options.plan_only = False

        joint_constraints = []
        for joint_name, position in joint_goal.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            joint_constraints.append(jc)

        constraints = Constraints()
        constraints.joint_constraints = joint_constraints

        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = group_name
        motion_plan_request.goal_constraints.append(constraints)
        motion_plan_request.max_velocity_scaling_factor = vel
        motion_plan_request.max_acceleration_scaling_factor = acc
        motion_plan_request.allowed_planning_time = MOVE_PLANNER_TIMEOUT
        motion_plan_request.pipeline_id = pipeline_id
        motion_plan_request.planner_id = planner_id
        goal_msg.request = motion_plan_request

        self.get_logger().info('Sending goal...')
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.result.error_code.val == 1:
            self.get_logger().info('Motion succeeded!')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code {result.result.error_code.val}')
            return False

    def wait_until_joint_goal_reached(self, target_joints: dict, tolerance=0.05, timeout=60.0):
        current_joint_states = {}

        def joint_state_callback(msg):
            for name, position in zip(msg.name, msg.position):
                current_joint_states[name] = position

        sub = self.create_subscription(JointState, '/joint_states', joint_state_callback, 10)

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            if not all(j in current_joint_states for j in target_joints):
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            all_within_tolerance = True
            for joint_name, target_pos in target_joints.items():
                current_pos = current_joint_states[joint_name]
                if abs(current_pos - target_pos) > tolerance:
                    all_within_tolerance = False
                    break

            if all_within_tolerance:
                self.get_logger().info("Target joint positions reached.")
                self.destroy_subscription(sub)
                return True

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn("Timeout waiting for joints to reach target.")
        self.destroy_subscription(sub)
        return False

    def tool_pose(self, x, y, z, ox, oy, oz, ow, vel=1.0, acc=1.0):
        target_pose = PoseStamped()
        target_pose.header.frame_id = MOVE_FRAME_NAME
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = ox
        target_pose.pose.orientation.y = oy
        target_pose.pose.orientation.z = oz
        target_pose.pose.orientation.w = ow
        return self.send_goal_ik(target_pose, MOVE_GROUP_UPPER_BODY, vel, acc)

    def send_goal_ik(self, target_pose: PoseStamped, group_name: str, vel: float, acc: float):
        # 人がいる間はここで待機（動作の境目で一時停止）.
        self._wait_while_human_present()

        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.goal_constraints.append(self._create_goal_constraints_ik(target_pose))
        goal.request.allowed_planning_time = MOVE_PLANNER_TIMEOUT
        goal.planning_options.plan_only = False
        goal.request.max_velocity_scaling_factor = vel
        goal.request.max_acceleration_scaling_factor = acc

        self.get_logger().info('Sending IK goal to move_action...')
        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if result.result.error_code.val == 1:
                self.get_logger().info('IK Motion succeeded!')
                return self.wait_until_tool_pose_reached(target_pose)
            else:
                self.get_logger().error(f'IK Motion failed with error code {result.result.error_code.val}')
                return False
        else:
            self.get_logger().error(f'IK Motion did not succeed. Status: {result.status}')
            return False

    def _create_goal_constraints_ik(self, target_pose: PoseStamped) -> Constraints:
        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = MOVE_LINK_NAME
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        constraints.position_constraints.append(pos_constraint)

        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = MOVE_LINK_NAME
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0
        constraints.orientation_constraints.append(ori_constraint)

        return constraints

    def get_tool_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                MOVE_FRAME_NAME, MOVE_LINK_NAME, now,
                timeout=rclpy.duration.Duration(seconds=1.0))
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f'Could not get tool pose: {e}')
            return None

    def pose_distance(self, pose1, pose2):
        import math
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def orientation_distance(self, pose1, pose2):
        import numpy as np
        q1 = pose1.orientation
        q2 = pose2.orientation
        dot_product = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w
        dot_product = max(min(dot_product, 1.0), -1.0)
        return 2 * math.acos(abs(dot_product))

    def wait_until_tool_pose_reached(self, target_pose_stamped,
                                     pos_threshold=0.015, ori_threshold=0.15, timeout=10.0):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            current_pose = self.get_tool_pose()
            if current_pose is None:
                time.sleep(0.1)
                continue
            pos_err = self.pose_distance(current_pose.pose, target_pose_stamped.pose)
            ori_err = self.orientation_distance(current_pose.pose, target_pose_stamped.pose)
            if pos_err < pos_threshold and ori_err < ori_threshold:
                self.get_logger().info('Target pose reached.')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn('Timeout waiting for tool pose to reach target.')
        return False
