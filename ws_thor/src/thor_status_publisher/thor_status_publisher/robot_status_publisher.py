import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from std_msgs.msg import String
import json

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')

        # Publishers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # Joint states
        self.joint_states = []
        self.joint_names = []
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to publish every 0.5 seconds
        self.create_timer(0.5, self.publish_status)

    def joint_state_callback(self, msg):
        self.joint_states = msg.position
        self.joint_names = msg.name

    def publish_status(self):
        try:
            transform = self.tf_buffer.lookup_transform('world', 'gripper_base', rclpy.time.Time())
            pos = transform.transform.translation
            ori = transform.transform.rotation
            r, p, y = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        except Exception as e:
            self.get_logger().warn(f"TF not available: {e}")
            return

        joint_degrees = [round(p * 180 / 3.14159, 2) for p in self.joint_states]
        ee_pos = {
            'x': round(pos.x * 1000, 2),
            'y': round(pos.y * 1000, 2),
            'z': round(pos.z * 1000, 2),
            'roll': round(r, 3),
            'pitch': round(p, 3),
            'yaw': round(y, 3),
        }

        data = {
            'joints': dict(zip(self.joint_names, joint_degrees)),
            'end_effector': ee_pos
        }

        msg = String()
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
