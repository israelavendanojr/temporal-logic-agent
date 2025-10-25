#!/usr/bin/env python3
"""
Launch file for UAV LTL Planner mission execution system.

This launch file starts the mission executor and state monitor nodes
for the UAV LTL translation and execution pipeline.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate the launch description for the UAV LTL Planner system.
    """

    # --- FINAL FIX: Use prefix to force VENV Python interpreter ---
    # The path confirmed by the 'find' command:
    python_exec_path = '/home/avendai/Desktop/crazyflie_ws/ros2_ws/venv/bin/python'
    python_prefix = [python_exec_path]
    # -----------------------------------------------------------

    # Get package share directory
    package_share_dir = get_package_share_directory('uav_ltl_planner')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_share_dir, 'config', 'environment.yaml'),
        description='Path to the environment configuration file'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the nodes (debug, info, warn, error)'
    )

    # Mission Executor Node
    mission_executor_node = Node(
        package='uav_ltl_planner',
        executable='mission_executor',
        name='mission_executor',
        output='screen',
        # --- FIX: Use prefix to run the node through VENV Python ---
        prefix=python_prefix,
        # --------------------------------------------------------
        parameters=[
            {'config_file': LaunchConfiguration('config_file')},
            {'log_level': LaunchConfiguration('log_level')}
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    # State Monitor Node
    state_monitor_node = Node(
        package='uav_ltl_planner',
        executable='state_monitor',
        name='state_monitor',
        output='screen',
        # --- FIX: Use prefix to run the node through VENV Python ---
        prefix=python_prefix,
        # --------------------------------------------------------
        parameters=[
            {'log_level': LaunchConfiguration('log_level')}
        ]
    )

    # Log startup information
    startup_info = LogInfo(
        msg=[
            'Starting UAV LTL Planner system...\n',
            '  - Mission Executor Node: /mission_command -> /ltl_formula, /mission_status\n',
            '  - State Monitor Node: monitoring mission execution\n',
            '  - Configuration: ', LaunchConfiguration('config_file'), '\n',
            '  - Log Level: ', LaunchConfiguration('log_level'), '\n',
            '\n',
            'Usage:\n',
            "  ros2 topic pub /mission_command std_msgs/String \"data: 'go to waypoint_a'\"\n",
            '  ros2 topic echo /ltl_formula\n',
            '  ros2 topic echo /mission_status\n'
        ]
    )

    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        startup_info,
        mission_executor_node,
        state_monitor_node,
    ])