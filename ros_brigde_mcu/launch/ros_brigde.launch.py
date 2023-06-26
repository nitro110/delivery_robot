import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

#    node_read_serial = Node(
#        package="ros_brigde_mcu",
#       name="ros_read_serial",
#        executable="sensor_publisher",
#    )

    node_send_serial = Node(
        package="ros_brigde_mcu",
        name="ros_send_serial",
        executable="serial_com",
    )

    ld.add_action(node_send_serial)
#    ld.add_action(node_read_serial)
    return ld
