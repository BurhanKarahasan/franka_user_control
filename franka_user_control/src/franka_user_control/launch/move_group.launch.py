#!/usr/bin/env python3
"""
Minimal MoveIt move_group launch for Franka simulation.
Provides move_group service and action server for motion planning.
Uses franka_fr3_moveit_config for kinematics and planning pipeline.
"""

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def _load_yaml_if_exists(path):
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():
    # Resolve package share dirs at launch-generation time
    pkg_share = get_package_share_directory('franka_user_control')
    moveit_share = None
    try:
        moveit_share = get_package_share_directory('franka_fr3_moveit_config')
    except Exception:
        moveit_share = None

    # URDF / SRDF (rendered as strings)
    robot_description_cmd = Command(['xacro ', os.path.join(pkg_share, 'urdf', 'fr3.urdf.xacro')])
    robot_description_semantic_cmd = Command(['cat ', os.path.join(pkg_share, 'urdf', 'fr3.srdf')])

    # Prepare ros__parameters dict for move_group
    ros_params = {}

    # robot description strings (use ParameterValue so substitutions stay lazy)
    ros_params['robot_description'] = ParameterValue(robot_description_cmd, value_type=str)
    ros_params['robot_description_semantic'] = ParameterValue(robot_description_semantic_cmd, value_type=str)

    # Load MoveIt config YAMLs if available and merge under ros__parameters
    if moveit_share:
        kinematics_path = os.path.join(moveit_share, 'config', 'kinematics.yaml')
        ompl_path = os.path.join(moveit_share, 'config', 'ompl_planning.yaml')
        controllers_path = os.path.join(moveit_share, 'config', 'controllers.yaml')

        kinematics = _load_yaml_if_exists(kinematics_path)
        ompl = _load_yaml_if_exists(ompl_path)
        controllers = _load_yaml_if_exists(controllers_path)

        # Merge common keys into ros__parameters. OMPL/kinematics files usually contain
        # top-level mappings suitable for inclusion in the move_group node namespace.
        for d in (kinematics, ompl, controllers):
            if isinstance(d, dict):
                ros_params.update(d)

    # Set MoveIt runtime/trajectory execution parameters to use the simple controller manager
    ros_params.setdefault('allow_trajectory_execution', True)
    ros_params.setdefault('use_sim_time', True)
    ros_params.setdefault('moveit_controller_manager', 'moveit_simple_controller_manager/MoveItSimpleControllerManager')
    ros_params.setdefault('trajectory_execution', {
        'manage_controllers': True,
        'allow_nonzero_velocity_at_trajectory_start': True,
    })

    # Wrap under ros__parameters as required by rcl (Node accepts a dict here)
    params = [{'ros__parameters': ros_params}]

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=params,
    )

    return LaunchDescription([move_group])
