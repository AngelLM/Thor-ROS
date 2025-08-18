import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState, RobotTrajectory
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
        
        # State variables
        self.current_joint_state = None
        
        self.get_logger().info('Nodo cartesian_goal_listener iniciado.')

    def joint_state_callback(self, msg):
        """Callback para actualizar el estado actual de las articulaciones"""
        self.current_joint_state = msg

    def cartesian_goal_callback(self, msg):
        """Callback principal para procesar goals cartesianos"""
        self.get_logger().info('Recibido CartesianGoal')
        
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
            
            if response.fraction > 0.8:  # Umbral de éxito
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
                self.get_logger().warn(f'Trayectoria cartesiana fallida. Fracción: {response.fraction}')
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
                'status': 'completed',
                'message': 'Cartesian trajectory executed successfully'
            })
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().error(f'Error en ejecución: {result.error_code.val}')
            self.publish_error_status(f'Trajectory execution failed. Error code: {result.error_code.val}')

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
