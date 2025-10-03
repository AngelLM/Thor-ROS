import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thor_urdf"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thor_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thor_moveit"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    task_server = Node(
        package="thor_server",
        executable="task_server_node",
        name="task_server_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    rosbridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thor_bringup"),
                "launch",
                "rosbridge.launch.py"
            )
        )
    )

    rosapi = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    status_publisher = Node(
        package="thor_status_publisher",
        executable="robot_status_publisher",
        name="robot_status_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    joint_goal_listener = Node(
        package="thor_status_publisher",
        executable="joint_goal_listener",
        name="joint_goal_listener",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    ik_goal_listener = Node(
        package="thor_status_publisher",
        executable="ik_goal_listener",
        name="ik_goal_listener",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    cartesian_goal_listener = Node(
        package="thor_status_publisher",
        executable="cartesian_goal_listener",
        name="cartesian_goal_listener",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        task_server,
        rosbridge,
        rosapi,
        status_publisher,
        joint_goal_listener,
        ik_goal_listener,
        cartesian_goal_listener
    ])