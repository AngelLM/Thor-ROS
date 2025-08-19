import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import RobotState, RobotTrajectory, MotionPlanRequest, Constraints, JointConstraint
from thor_msgs.msg import CartesianGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.action import ActionClient
import json

class CartesianGoalListener(Node):
    def __init__(self):
        super().__init__('cartesian_goal_listener')
        
        # Subscriptions
        self.subscription = self.create_subscription(
            CartesianGoal,
            '/cartesian_goal',
            self.cartesian_goal_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/cartesian_goal_status', 10)
        self.waypoints_pub = self.create_publisher(String, '/cartesian_waypoints', 10)
        
        # Service clients
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        # Action clients
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # State variables
        self.current_joint_state = None
        self.current_cartesian_goal = None  # Para almacenar el goal actual
        
        self.get_logger().info('Nodo cartesian_goal_listener iniciado.')

    def joint_state_callback(self, msg):
        """Callback para actualizar el estado actual de las articulaciones"""
        self.current_joint_state = msg

    def cartesian_goal_callback(self, msg):
        """Callback principal para procesar goals cartesianos"""
        self.get_logger().info('Recibido CartesianGoal')
        
        # Almacenar el goal actual para usarlo después
        self.current_cartesian_goal = msg
        
        # Publicar status de inicio
        status_msg = String()
        status_msg.data = json.dumps({
            'status': 'processing',
            'message': 'Computing cartesian path...'
        })
        self.status_pub.publish(status_msg)
        
        # Computar trayectoria cartesiana
        self.compute_cartesian_path(msg)

    def compute_cartesian_path(self, goal):
        """Computa la trayectoria cartesiana usando el servicio de MoveIt"""
        if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Servicio /compute_cartesian_path no disponible')
            self.publish_error_status('Cartesian path service not available')
            return
        
        self.get_logger().info('Servicio /compute_cartesian_path disponible, preparando petición')
        
        # Preparar la petición
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()
        
        # Configurar el estado inicial del robot
        req.start_state = RobotState()
        if self.current_joint_state is not None:
            req.start_state.joint_state = self.current_joint_state
        req.start_state.is_diff = False
        
        # Configurar parámetros del grupo
        req.group_name = 'arm_group'
        req.link_name = 'gripper_mid_point'
        
        # Configurar waypoints (solo el punto final)
        waypoint = goal.end_pose
        req.waypoints = [waypoint]
        
        # Configurar parámetros de la trayectoria
        req.max_step = goal.max_step if goal.max_step > 0 else 0.01
        req.jump_threshold = goal.jump_threshold if goal.jump_threshold > 0 else 0.0
        req.avoid_collisions = goal.avoid_collisions
        
        # Constraints vacías por defecto
        req.path_constraints.name = ''
        req.path_constraints.joint_constraints = []
        req.path_constraints.position_constraints = []
        req.path_constraints.orientation_constraints = []
        req.path_constraints.visibility_constraints = []
        
        self.get_logger().info('Enviando petición de trayectoria cartesiana')
        
        # Llamar al servicio
        future = self.cartesian_path_client.call_async(req)
        future.add_done_callback(lambda f: self.cartesian_path_response_callback(f, goal))

    def cartesian_path_response_callback(self, future, goal):
        """Callback para procesar la respuesta del servicio de trayectoria cartesiana"""
        try:
            response = future.result()
            
            self.get_logger().info(f'Respuesta recibida. Fracción: {response.fraction}')
            
            if response.fraction > 0.8:  # Umbral de éxito para ejecutar trayectoria
                self.get_logger().info('Trayectoria cartesiana calculada exitosamente')
                
                # Extraer waypoints de la trayectoria
                waypoints_data = self.extract_waypoints_from_trajectory(response.solution)
                
                # Publicar waypoints
                waypoints_msg = String()
                waypoints_msg.data = json.dumps({
                    'waypoints': waypoints_data,
                    'fraction': response.fraction
                })
                self.waypoints_pub.publish(waypoints_msg)
                
                # Publicar status de éxito
                status_msg = String()
                status_msg.data = json.dumps({
                    'status': 'path_computed',
                    'message': f'Cartesian path computed successfully. Fraction: {response.fraction}',
                    'fraction': response.fraction,
                    'waypoints_count': len(waypoints_data)
                })
                self.status_pub.publish(status_msg)
                
                # Ejecutar la trayectoria
                self.execute_trajectory(response.solution, goal)
                
            else:
                # Fracción insuficiente: tratar como error de planificación y no ejecutar
                # ni la trayectoria cartesiana ni el movimiento del gripper.
                self.get_logger().error(f'No cartesian solution found. Fraction: {response.fraction}')
                self.publish_error_status(f'No cartesian solution found. Fraction: {response.fraction}')
                
        except Exception as e:
            self.get_logger().error(f'Error en cartesian_path_response_callback: {str(e)}')
            self.publish_error_status(f'Error computing cartesian path: {str(e)}')

    def extract_waypoints_from_trajectory(self, trajectory):
        """Extrae los waypoints de la trayectoria"""
        waypoints = []
        
        if trajectory.joint_trajectory and trajectory.joint_trajectory.points:
            joint_names = trajectory.joint_trajectory.joint_names
            
            for i, point in enumerate(trajectory.joint_trajectory.points):
                waypoint = {
                    'index': i,
                    'joints': {},
                    'time': {
                        'sec': point.time_from_start.sec,
                        'nanosec': point.time_from_start.nanosec
                    }
                }
                
                # Mapear nombres de joints con posiciones
                for j, name in enumerate(joint_names):
                    if j < len(point.positions):
                        waypoint['joints'][name] = point.positions[j]
                
                waypoints.append(waypoint)
                
        return waypoints

    def execute_trajectory(self, trajectory, goal):
        """Ejecuta la trayectoria usando el action server de MoveIt"""
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server /execute_trajectory no disponible')
            self.publish_error_status('Execute trajectory action server not available')
            return
        
        self.get_logger().info('Ejecutando trayectoria cartesiana')
        
        # Crear el goal para el action
        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = trajectory
        
        # Publicar status de ejecución
        status_msg = String()
        status_msg.data = json.dumps({
            'status': 'executing',
            'message': 'Executing cartesian trajectory...'
        })
        self.status_pub.publish(status_msg)
        
        # Enviar el goal
        send_goal_future = self.execute_trajectory_client.send_goal_async(
            execute_goal,
            feedback_callback=self.execute_feedback_callback
        )
        send_goal_future.add_done_callback(self.execute_goal_response_callback)

    def execute_feedback_callback(self, feedback_msg):
        """Callback para feedback de la ejecución"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback de ejecución: {feedback.state}')

    def execute_goal_response_callback(self, future):
        """Callback para la respuesta del goal de ejecución"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal de ejecución rechazado')
            self.publish_error_status('Trajectory execution goal rejected')
            return
        
        self.get_logger().info('Goal de ejecución aceptado')
        
        # Esperar resultado
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.execute_result_callback)

    def execute_result_callback(self, future):
        """Callback para el resultado de la ejecución"""
        result = future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Trayectoria cartesiana ejecutada exitosamente')
            status_msg = String()
            status_msg.data = json.dumps({
                'status': 'cartesian_completed',
                'message': 'Cartesian trajectory executed successfully, moving gripper...'
            })
            self.status_pub.publish(status_msg)
            
            # Ahora ejecutar el movimiento del gripper
            self.execute_gripper_movement()
        else:
            self.get_logger().error(f'Error en ejecución: {result.error_code.val}')
            self.publish_error_status(f'Trajectory execution failed. Error code: {result.error_code.val}')

    def execute_gripper_movement(self):
        """Ejecuta el movimiento del gripper como un movimiento de joint independiente"""
        if self.current_cartesian_goal is None:
            self.get_logger().error('No hay goal cartesiano disponible para el gripper')
            self.publish_error_status('No cartesian goal available for gripper movement')
            return
            
        self.get_logger().info(f'Ejecutando movimiento del gripper a posición: {self.current_cartesian_goal.gripperbase_to_armgearright}')
        
        # Crear constraints para el gripper
        gripper_constraints = Constraints()
        jc = JointConstraint()
        jc.joint_name = 'gripperbase_to_armgearright'
        jc.position = self.current_cartesian_goal.gripperbase_to_armgearright
        jc.weight = 1.0
        gripper_constraints.joint_constraints.append(jc)

        # Crear la solicitud de planificación de movimiento del gripper_group
        gripper_request = MotionPlanRequest()
        gripper_request.group_name = 'gripper_group'
        gripper_request.goal_constraints.append(gripper_constraints)

        gripper_goal_msg = MoveGroup.Goal()
        gripper_goal_msg.request = gripper_request
        gripper_goal_msg.planning_options.plan_only = False
        gripper_goal_msg.planning_options.look_around = False
        gripper_goal_msg.planning_options.replan = True

        # Esperar que el action server esté disponible
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server move_action no disponible para gripper')
            self.publish_error_status('MoveGroup action server not available for gripper movement')
            return

        # Enviar el goal para el gripper
        future = self.move_group_client.send_goal_async(gripper_goal_msg)
        future.add_done_callback(self.gripper_goal_response_callback)
        self.get_logger().info('Solicitud de gripper_group enviada a MoveIt2.')

    def gripper_goal_response_callback(self, future):
        """Callback para la respuesta del goal del gripper"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal del gripper rechazado')
            self.publish_error_status('Gripper movement goal rejected')
            return
        
        self.get_logger().info('Goal del gripper aceptado')
        
        # Esperar resultado
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future):
        """Callback para el resultado del movimiento del gripper"""
        result = future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Movimiento del gripper ejecutado exitosamente')
            status_msg = String()
            status_msg.data = json.dumps({
                'status': 'completed',
                'message': 'Cartesian trajectory and gripper movement completed successfully'
            })
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().error(f'Error en movimiento del gripper: {result.error_code.val}')
            self.publish_error_status(f'Gripper movement failed. Error code: {result.error_code.val}')

    def publish_error_status(self, error_message):
        """Publica un mensaje de error en el topic de status"""
        status_msg = String()
        status_msg.data = json.dumps({
            'status': 'error',
            'message': error_message
        })
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    cartesian_goal_listener = CartesianGoalListener()
    
    try:
        rclpy.spin(cartesian_goal_listener)
    except KeyboardInterrupt:
        pass
    finally:
        cartesian_goal_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
