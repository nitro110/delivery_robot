from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_sfr = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'steering_front_right'],
        output='screen'
    )

    load_sfl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'steering_front_left'],
        output='screen'
    )

    load_srr = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'steering_rear_right'],
        output='screen'
    )

    load_srl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'steering_rear_left'],
        output='screen'
    )

    load_wfr = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'wheel_front_right'],
        output='screen'
    )

    load_wfl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'wheel_front_left'],
        output='screen'
    )

    load_wrr = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'wheel_rear_right'],
        output='screen'
    )

    load_wrl = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'wheel_rear_left'],
        output='screen'
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_controller_sfr = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["steering_front_right"],
        output="screen",
    )
    spawn_controller_sfl = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["steering_front_left"],
        output="screen",
    )
    spawn_controller_srr = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["steering_rear_right"],
        output="screen",
    )
    spawn_controller_srl = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["steering_rear_left"],
        output="screen",
    )
    spawn_controller_wfr = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["wheel_front_right"],
        output="screen",
    )
    spawn_controller_wfl = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["wheel_front_left"],
        output="screen",
    )
    spawn_controller_wrr = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["wheel_rear_right"],
        output="screen",
    )
    spawn_controller_wrl = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["wheel_rear_left"],
        output="screen",
    )

    return LaunchDescription([        
        spawn_controller,
        spawn_controller_srr,
        spawn_controller_sfl,
        spawn_controller_sfr,
        spawn_controller_srl,
        spawn_controller_wfl,
        spawn_controller_wfr,
        spawn_controller_wrl,
        spawn_controller_wrr
    ])
