#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.actions



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world = os.path.join(get_package_share_directory('robot_swerve_description'),'worlds', '4ws_world.model')
    launch_file_dir = os.path.join(get_package_share_directory('robot_swerve_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_four_ws_control = get_package_share_directory('four_ws_control')
    rviz_config_file = os.path.join(get_package_share_directory('robot_swerve_description'), 'rviz', 'test.rviz')

    gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items(),
            )

    gzclient = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_four_ws_control, 'launch', 'four_ws_control.launch.py')]
            ),
        )

    robot_excute_process = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        output='screen')

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),

        )

    forward_position_controller = ExecuteProcess( 
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'forward_position_controller'], output='screen'
        )

    forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'forward_velocity_controller'], output='screen'
        )

    joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'configure', 'joint_state_broadcaster'], output='screen'
        )
        
    joy_node = Node(
        package = "joy",
        executable = "joy_node"
        )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
        )


    return LaunchDescription([
        gzserver,
        gzclient, 
        robot_excute_process,
        robot_state_publisher,
        joy_node,
        controller,
        forward_position_controller,
        forward_velocity_controller,
        joint_state_broadcaster,
        rviz_node
    ])