import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("delivery_bringup"),
        "config",
        "robot_geometry.yaml",
    )

    node_swerve_commander = Node(
        package="delivery_bringup",
        name="swerve_commander",
        executable="wheel_cmd_publisher",
        parameters=[config],
    )

    node_swerve_odom = Node(
        package="delivery_bringup",
        name="swerve_odometer",
        executable="wheel_odometry",
        parameters=[config],
    )

    ld.add_action(node_swerve_commander)
    ld.add_action(node_swerve_odom)
    return ld
