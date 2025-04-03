import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
                get_package_share_directory("thor_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    task_server = Node(
        package="thor_server",
        executable="task_server_node",
        name="task_server_node",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )
    
    return LaunchDescription([
        controller,
        moveit,
        task_server,
    ])