import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Set the robot description parameter
    print(">>>>  ", os.getcwd())
    urdf_file = os.path.join(
        os.path.dirname(__file__),
        os.getcwd() +'/src/urdf_thor/urdf/thor.urdf'
    )

    robot_description = {'robot_description': open(urdf_file).read()}

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    rviz_config_file_arg = DeclareLaunchArgument('rviz_config_file', default_value='')
    
    # Launch joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Launch robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )

    # Launch MoveIt2
    moveit_controller_manager_parameter = {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
    moveit_manage_controllers_node = Node(
        package='moveit_controller_manager',
        executable='moveit_simple_controller_manager',
        name='moveit_manage_controllers',
        output='screen',
        parameters=[moveit_controller_manager_parameter]
    )

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[robot_description],
        arguments=['-d', LaunchConfiguration('rviz_config_file')]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_file_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        moveit_manage_controllers_node,
        rviz_node
    ])
