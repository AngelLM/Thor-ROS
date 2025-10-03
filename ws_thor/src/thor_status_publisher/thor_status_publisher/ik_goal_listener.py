import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from moveit_msgs.srv import GetPositionIK
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from thor_msgs.msg import IKGoal
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from std_msgs.msg import String
import json

class IKGoalListener(Node):
    def __init__(self):
        super().__init__('ik_goal_listener')
        self.subscription = self.create_subscription(
            IKGoal,
            '/ik_goal',
            self.ik_goal_callback,
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.status_pub = self.create_publisher(String, '/ik_goal_status', 10)
        self.current_joint_state = None
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info('Nodo ik_goal_listener iniciado.')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    
    def is_pose_reachable(self, pose_stamped, callback=None):
        self.get_logger().info('Entrando en is_pose_reachable')
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Servicio /compute_ik no disponible')
            if callback:
                callback(False, None)
            return False
        self.get_logger().info('Servicio /compute_ik disponible, preparando petición')
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm_group'
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 0
        req.ik_request.ik_link_name = 'gripper_mid_point'

        # --- Constraints para solución aproximada ---
        approx_constraints = Constraints()
        # Tolerancia de posición (±2mm)
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = pose_stamped.header.frame_id
        pos_constraint.link_name = 'gripper_mid_point'
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]  # 4mm box (±2mm)
        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        pos_constraint.weight = 1.0
        approx_constraints.position_constraints.append(pos_constraint)
        # Tolerancia de orientación (±0.05 rad)
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = pose_stamped.header.frame_id
        ori_constraint.link_name = 'gripper_mid_point'
        ori_constraint.orientation = pose_stamped.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.001
        ori_constraint.absolute_y_axis_tolerance = 0.001
        ori_constraint.absolute_z_axis_tolerance = 0.001
        ori_constraint.weight = 1.0
        approx_constraints.orientation_constraints.append(ori_constraint)
        req.ik_request.constraints = approx_constraints
        # --- Fin constraints aproximados ---

        self.get_logger().info('Enviando robot_state vacío (sin current_joint_state)')
        self.get_logger().info(f'Enviando petición a /compute_ik: group={req.ik_request.group_name}, link={req.ik_request.ik_link_name}, timeout={req.ik_request.timeout.sec}, pose=({pose_stamped.pose.position.x}, {pose_stamped.pose.position.y}, {pose_stamped.pose.position.z})')
        future = self.ik_client.call_async(req)
        self.get_logger().info('Esperando respuesta del servicio /compute_ik...')
        def done_cb(fut):
            result = fut.result()
            if result:
                self.get_logger().info(f"Respuesta compute_ik: error_code={result.error_code.val}, solución encontrada={result.error_code.val == 1}")
                # Intentar distinguir solución exacta vs aproximada (si el servicio lo soporta)
                solution_type = None
                if hasattr(result, 'solution_type'):
                    solution_type = result.solution_type  # Solo si existe este campo
                # Si no existe, usar solo success/fail
                if callback:
                    callback(result.error_code.val == 1, solution_type)
            else:
                self.get_logger().warn('No se recibió respuesta del servicio compute_ik')
                if callback:
                    callback(False, None)
        future.add_done_callback(done_cb)
        # No bloquear, el resultado se maneja en el callback
        return None

    def publish_status(self, status, detail=""):
        msg = String()
        msg.data = json.dumps({"status": status, "detail": detail})
        self.status_pub.publish(msg)

    def ik_goal_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = msg.pose
        self.get_logger().info(f"Pose objetivo recibida para IK: frame_id={pose_stamped.header.frame_id}, "
                               f"x={pose_stamped.pose.position.x:.3f}, y={pose_stamped.pose.position.y:.3f}, z={pose_stamped.pose.position.z:.3f}, "
                               f"qx={pose_stamped.pose.orientation.x:.3f}, qy={pose_stamped.pose.orientation.y:.3f}, "
                               f"qz={pose_stamped.pose.orientation.z:.3f}, qw={pose_stamped.pose.orientation.w:.3f}")
        def after_ik(is_reachable, solution_type):
            if not is_reachable:
                self.get_logger().warn('La pose objetivo NO es alcanzable por el robot. No se enviará el goal.')
                self.publish_status("unreachable", "La pose objetivo no es alcanzable por el robot.")
                return
            # Si se puede distinguir solución exacta vs aproximada, reportar
            if solution_type is not None:
                if solution_type == 0:
                    self.publish_status("reachable_exact", "La pose objetivo es alcanzable EXACTAMENTE. Planificando movimiento.")
                else:
                    self.publish_status("reachable_approx", "La pose objetivo es alcanzable APROXIMADAMENTE (usando tolerancias). Planificando movimiento.")
            else:
                self.publish_status("reachable", "La pose objetivo es alcanzable. Planificando movimiento.")
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = "base_link"
            pos_constraint.link_name = "gripper_mid_point"
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.001, 0.001, 0.001]
            pos_constraint.constraint_region.primitives.append(box)
            pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
            pos_constraint.weight = 1.0
            ori_constraint = OrientationConstraint()
            ori_constraint.header.frame_id = "base_link"
            ori_constraint.link_name = "gripper_mid_point"
            ori_constraint.orientation = msg.pose.orientation
            ori_constraint.absolute_x_axis_tolerance = 0.001
            ori_constraint.absolute_y_axis_tolerance = 0.001
            ori_constraint.absolute_z_axis_tolerance = 0.001
            ori_constraint.weight = 1.0
            constraints = Constraints()
            constraints.position_constraints.append(pos_constraint)
            constraints.orientation_constraints.append(ori_constraint)
            joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
            joint_pref = {}
            if self.current_joint_state:
                for name, pos in zip(self.current_joint_state.name, self.current_joint_state.position):
                    joint_pref[name] = pos
            else:
                for name in joint_names:
                    joint_pref[name] = 0.0
            
            request = MotionPlanRequest()
            request.group_name = 'arm_group'
            request.goal_constraints.append(constraints)
            request.allowed_planning_time = 5.0
            goal_msg = MoveGroup.Goal()
            goal_msg.request = request
            goal_msg.planning_options.plan_only = False
            goal_msg.planning_options.look_around = False
            goal_msg.planning_options.replan = True
            self.move_group_client.wait_for_server()
            self.future = self.move_group_client.send_goal_async(goal_msg)
            self.get_logger().info('Solicitud IK enviada a MoveIt2.')

            # Solicitud para gripper_group
            gripper_constraints = Constraints()
            jc = JointConstraint()
            jc.joint_name = "gripperbase_to_armgearright"  # Nombre de la joint del gripper
            jc.position = msg.gripperbase_to_armgearright  # Usar el valor del mensaje IKGoal
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
        self.is_pose_reachable(pose_stamped, callback=after_ik)

        

def main(args=None):
    rclpy.init(args=args)
    node = IKGoalListener()
    rclpy.spin(node)
    rclpy.shutdown()