#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
)
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class RandomCommander(Node):
    def __init__(self):
        super().__init__('random_position_commander')
        self.get_logger().info("Nodo iniciado. Enviando posición aleatoria...")

        self.action_client = ActionClient(self, MoveGroup, '/move_action')

        # Define 5 posibles posiciones manualmente
        self.predefined_positions = [
            {'joint_1': 0.0, 'joint_2': -0.5, 'joint_3': 0.5, 'joint_4': 0.0, 'joint_5': 0.0, 'joint_6': 0.0},
            {'joint_1': 1.0, 'joint_2': 0.5, 'joint_3': -0.5, 'joint_4': 0.3, 'joint_5': -0.2, 'joint_6': 0.0},
            {'joint_1': -1.2, 'joint_2': 0.4, 'joint_3': 0.0, 'joint_4': -0.4, 'joint_5': 0.6, 'joint_6': 0.0},
            {'joint_1': 0.7, 'joint_2': -0.8, 'joint_3': 0.2, 'joint_4': 0.1, 'joint_5': -0.1, 'joint_6': 0.0},
            {'joint_1': 0.0, 'joint_2': 0.0, 'joint_3': 0.0, 'joint_4': 0.0, 'joint_5': 0.0, 'joint_6': 0.0}
        ]

        self.send_random_goal()

    def send_random_goal(self):
        # Espera a que el servidor de acciones esté listo
        self.action_client.wait_for_server()

        # Elige una posición aleatoria
        goal_position = random.choice(self.predefined_positions)
        self.get_logger().info(f"Objetivo seleccionado: {goal_position}")

        # Construye las constraints
        joint_constraints = []
        for joint_name, position in goal_position.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        constraints = Constraints()
        constraints.joint_constraints = joint_constraints

        # Construir el mensaje MoveGroup
        request = MotionPlanRequest()
        request.group_name = "arm_group"
        request.goal_constraints = [constraints]
        request.allowed_planning_time = 5.0

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        # Enviar la acción
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ La meta fue rechazada por el servidor de acciones.')
            rclpy.shutdown()
            return

        self.get_logger().info('✅ Meta aceptada, esperando resultado...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🎯 Movimiento finalizado con estado: {future.result().status}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RandomCommander()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
