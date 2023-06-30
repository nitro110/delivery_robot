#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.actions



def generate_launch_description():

    launch_file_dir = os.path.join(get_package_share_directory('robot_swerve_description'), 'launch')
    pkg_swerve_controller = get_package_share_directory('swerve_controller')
    controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_swerve_controller, 'launch', 'swerve_control_odom.launch.py')]
            ),
        )

    teleop_twist_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/teleop_joy.launch.py']),
    )

    
    # Spawn robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "robot_swerve",
            "-z",
            "0",
        ],
        output="screen",
    )

    # Joint State Controller
    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # Wheel Rotation Controllers
    load_fr_wheel_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "wheel_front_right",
        ],
        output="screen",
    )

    load_fl_wheel_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "wheel_front_left",
        ],
        output="screen",
    )

    load_rl_wheel_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "wheel_rear_left",
        ],
        output="screen",
    )

    load_rr_wheel_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "wheel_rear_right",
        ],
        output="screen",
    )

    # Wheel Steering Controllers
    load_fr_steering_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "steering_front_right",
        ],
        output="screen",
    )

    load_fl_steering_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "steering_front_left",
        ],
        output="screen",
    )

    load_rl_steering_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "steering_rear_left",
        ],
        output="screen",
    )

    load_rr_steering_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "steering_rear_right",
        ],
        output="screen",
    )

    eh_joint_state_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )

    eh_fr_steer_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_fr_steering_controller],
        )
    )

    eh_fl_steer_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_fr_steering_controller,
            on_exit=[load_fl_steering_controller],
        )
    )

    eh_rl_steer_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_fl_steering_controller,
            on_exit=[load_rl_steering_controller],
        )
    )

    eh_rr_steer_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_rl_steering_controller,
            on_exit=[load_rr_steering_controller],
        )
    )

    eh_fr_wheel_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_rr_steering_controller,
            on_exit=[load_fr_wheel_controller],
        )
    )

    eh_fl_wheel_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_fr_wheel_controller,
            on_exit=[load_fl_wheel_controller],
        )
    )

    eh_rl_wheel_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_fl_wheel_controller,
            on_exit=[load_rl_wheel_controller],
        )
    )

    eh_rr_wheel_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_rl_wheel_controller,
            on_exit=[load_rr_wheel_controller],
        )
    )


    return LaunchDescription([
        controller,
        teleop_twist_robot,
        load_fr_wheel_controller,
        load_fl_wheel_controller,
        load_rl_wheel_controller,
        load_rr_wheel_controller,
        load_fr_steering_controller,
        load_fl_steering_controller,
        load_rl_steering_controller,
        load_rr_steering_controller,
        eh_joint_state_ctrl,
        eh_fr_steer_ctrl,
        eh_fl_steer_ctrl,
        eh_rl_steer_ctrl,
        eh_rr_steer_ctrl,
        eh_fr_wheel_ctrl,
        eh_fl_wheel_ctrl,
        eh_rl_wheel_ctrl,
        eh_rr_wheel_ctrl

    ])