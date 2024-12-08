import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("urdf_thor"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thor_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True", "use_sim_time": "True"}.items()
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("moveit_thor"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True", "use_sim_time": "True"}.items()
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    joy_test = Node(
        package="thor_examples",
        executable="joy_test",
        name="joy_test",
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        joy_node,
        joy_test,
    ])
