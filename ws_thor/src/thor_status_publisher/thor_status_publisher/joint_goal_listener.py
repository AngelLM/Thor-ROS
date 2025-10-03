#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient

class JointGoalListener(Node):
    def __init__(self):
        super().__init__('joint_goal_listener')
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'gripperbase_to_armgearright'
        ]
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint_group_position_controller/command',
            self.command_callback,
            10
        )
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Nodo joint_goal_listener iniciado.')

    def command_callback(self, msg):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().error('Longitud del mensaje no coincide con las articulaciones.')
            return

        self.get_logger().info('Recibido comando, enviando a MoveIt2...')

        # Crear la solicitud de planificación de movimiento del arm_group
        goal_constraints = Constraints()
        for i, joint_name in enumerate(self.joint_names[:-1]):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = msg.data[i]
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name = 'arm_group'  # ← Asegúrate de que este sea el grupo de planificación correcto
        request.goal_constraints.append(goal_constraints)

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True

        self.move_group_client.wait_for_server()
        self.future = self.move_group_client.send_goal_async(goal_msg)
        self.get_logger().info('Solicitud de arm_group enviada a MoveIt2.')

        # Solicitud para gripper_group
        gripper_constraints = Constraints()
        jc = JointConstraint()
        jc.joint_name = self.joint_names[-1]  # joint_gripper
        jc.position = msg.data[-1]
        jc.weight = 1.0
        gripper_constraints.joint_constraints.append(jc)

        gripper_request = MotionPlanRequest()
        gripper_request.group_name = 'gripper_group'
        gripper_request.goal_constraints.append(gripper_constraints)

        gripper_goal_msg = MoveGroup.Goal()
        gripper_goal_msg.request = gripper_request
        gripper_goal_msg.planning_options.plan_only = False
        gripper_goal_msg.planning_options.look_around = False
        gripper_goal_msg.planning_options.replan = True

        self.move_group_client.wait_for_server()
        self.future = self.move_group_client.send_goal_async(gripper_goal_msg)
        self.get_logger().info('Solicitud de gripper_group enviada a MoveIt2.')

def main(args=None):
    rclpy.init(args=args)
    node = JointGoalListener()
    rclpy.spin(node)
    rclpy.shutdown()
