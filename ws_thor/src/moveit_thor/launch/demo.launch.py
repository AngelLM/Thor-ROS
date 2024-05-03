from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("THOR_arm", package_name="moveit_thor").to_moveit_configs()
    return generate_demo_launch(moveit_config)
