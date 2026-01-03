#!/usr/bin/env python3
"""
Launch file for Franka User Control system.

Starts the C++ bridge node with robot configuration.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction # type: ignore
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution # type: ignore
from launch_ros.actions import Node # type: ignore
from launch_ros.substitutions import FindPackageShare # type: ignore
from ament_index_python.packages import get_package_share_directory # type: ignore


def generate_launch_description():
    """Generate launch description for Franka user control."""
    
    # Package name
    pkg_name = 'franka_user_control'
    
    # Get package share directory
    pkg_share = FindPackageShare(pkg_name)
    
    # Configuration file paths
    robot_config_file = PathJoinSubstitution([
        pkg_share, 'config', 'robot_config.yaml'
    ])
    
    safety_config_file = PathJoinSubstitution([
        pkg_share, 'config', 'safety_limits.yaml'
    ])
    
    impedance_config_file = PathJoinSubstitution([
        pkg_share, 'config', 'impedance_params.yaml'
    ])
    
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.100',
        description='IP address of the Franka robot'
    )
    
    use_conservative_limits_arg = DeclareLaunchArgument(
        'conservative_limits',
        default_value='false',
        description='Use conservative safety limits for testing'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Franka bridge node
    franka_bridge_node = Node(
        package=pkg_name,
        executable='franka_bridge_node',
        name='franka_bridge_node',
        output='screen',
        parameters=[
            robot_config_file,
            safety_config_file,
            impedance_config_file,
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'conservative_limits': LaunchConfiguration('conservative_limits'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Launch arguments
        robot_ip_arg,
        use_conservative_limits_arg,
        log_level_arg,
        
        # Nodes
        franka_bridge_node,
    ])
