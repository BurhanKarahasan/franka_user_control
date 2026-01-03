#!/usr/bin/env python3
"""
Simulation launch file for Franka User Control system.

Starts the system in simulation mode without requiring a real robot.
Uses fake hardware interface for testing and development.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction # type: ignore
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution # type: ignore
from launch.conditions import IfCondition # type: ignore
from launch_ros.actions import Node # type: ignore
from launch_ros.substitutions import FindPackageShare # type: ignore
from ament_index_python.packages import get_package_share_directory # type: ignore
import launch.conditions # type: ignore


def generate_launch_description():
    """Generate launch description for simulation mode."""
    
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
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )
    
    use_conservative_limits_arg = DeclareLaunchArgument(
        'conservative_limits',
        default_value='true',
        description='Use conservative safety limits for testing'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100',
        description='State publishing rate in Hz'
    )
    
    # Fake robot state publisher for simulation
    # This node simulates the robot state without actual hardware
    fake_robot_state_node = Node(
        package=pkg_name,
        executable='fake_robot_state_publisher.py',  # We'll create this
        name='fake_robot_state_publisher',
        output='screen',
        parameters=[
            {
                'publish_rate': LaunchConfiguration('publish_rate'),
                'simulate_motion': True,
                'add_noise': False,  # Set to true for more realistic simulation
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    
    # Robot state publisher (publishes TF transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            # In a real implementation, this would load the URDF
            # For now, we'll use a placeholder
        ],
    )
    
    # Joint state publisher for manual control (optional, useful for testing)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('use_rviz')
        ),
    )
    
    # RViz for visualization
    rviz_config_file = PathJoinSubstitution([
        pkg_share, 'rviz', 'simulation.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(
            LaunchConfiguration('use_rviz')
        ),
    )
    
    # Simulation bridge node (simplified version of franka_bridge_node)
    sim_bridge_node = Node(
        package=pkg_name,
        executable='simulation_bridge_node.py',  # We'll create this
        name='simulation_bridge_node',
        output='screen',
        parameters=[
            robot_config_file,
            safety_config_file,
            impedance_config_file,
            {
                'simulation_mode': True,
                'conservative_limits': LaunchConfiguration('conservative_limits'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        use_conservative_limits_arg,
        log_level_arg,
        publish_rate_arg,
        
        # Nodes
        fake_robot_state_node,
        robot_state_publisher,
        sim_bridge_node,
        rviz_node,
    ])