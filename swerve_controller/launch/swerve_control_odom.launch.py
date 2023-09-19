import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


# launch LaunchDescription define
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("swerve_controller"),
        "config",
        "robot_geometry.yaml",  # get parameters from yaml file
    )

    # load robot swerve controller
    node_swerve_commander = Node(
        package="swerve_controller",
        name="swerve_commander",
        executable="wheel_cmd_publisher",
        parameters=[config],
    )
    # load robot odom calculator
    node_swerve_odom = Node(
        package="swerve_controller",
        name="swerve_odometer",
        executable="wheel_odometry",
        parameters=[config],
    )

    ld.add_action(node_swerve_commander)
    ld.add_action(node_swerve_odom)
    return ld
