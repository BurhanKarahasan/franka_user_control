"""
Franka User Control - Python Interface

A user-friendly Python interface for controlling the Franka Research 3 robot.
Provides simple APIs for velocity control, position control, and safety monitoring.

Example usage:
    from franka_python_interface import FrankaInterface
    
    robot = FrankaInterface()
    robot.connect()
    
    # Velocity control
    robot.set_cartesian_velocity(vx=0.05, vy=0.0, vz=0.0)
    robot.stop()
    
    # Position control
    robot.move_relative(dx=0.1, dy=0.0, dz=0.05)
"""

__version__ = "0.1.0"
__author__ = "Franka User Control"

from .robot_interface import FrankaInterface
from .control_modes import ControlMode, ImpedanceMode
from .safety_monitor import SafetyMonitor
from .utils import pose_to_list, list_to_pose, euler_to_quaternion, quaternion_to_euler

__all__ = [
    'FrankaInterface',
    'ControlMode',
    'ImpedanceMode',
    'SafetyMonitor',
    'pose_to_list',
    'list_to_pose',
    'euler_to_quaternion',
    'quaternion_to_euler',
]
