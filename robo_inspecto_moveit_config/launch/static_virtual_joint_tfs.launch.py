from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    robo_inspecto_moveit_config = MoveItConfigsBuilder("macro", package_name="robo_inspecto_moveit_config").to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(robo_inspecto_moveit_config)
