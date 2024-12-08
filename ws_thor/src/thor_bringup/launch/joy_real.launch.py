import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("thor_controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("moveit_thor"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )

    joy_test = Node(
        package="thor_examples",
        executable="joy_test",
        name="joy_test",
        output="screen"
    )

    return LaunchDescription([
        controller,
        moveit,
        joy_node,
        joy_test,
    ])