import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
      
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )

    joy_pose = Node(
        package="thor_joystick",
        executable="joystick_pose_controller",
        name="joystick_pose_controller",
        output="screen"
    )

    return LaunchDescription([
        joy_node,
        joy_pose,
    ])