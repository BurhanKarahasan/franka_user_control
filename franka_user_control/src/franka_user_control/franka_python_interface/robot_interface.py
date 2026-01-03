"""
Main Python interface for controlling the Franka Research 3 robot.

This module provides a high-level API for robot control that communicates
with the C++ bridge node via ROS2 services.
"""

import time
from typing import Optional, List, Tuple
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Pose # type: ignore
from sensor_msgs.msg import JointState # type: ignore

from .control_modes import ControlMode, ImpedanceMode, SafetyLimits, VelocityLimits
from .safety_monitor import SafetyMonitor
from .utils import (
    pose_to_list, list_to_pose, euler_to_quaternion, 
    quaternion_to_euler, print_pose
)

# Note: After building, these imports would use the generated service types
# from franka_user_control.srv import (
#     SetCartesianVelocity, MoveToPose, MoveRelative,
#     SetControlMode, EmergencyStop
# )
# from franka_user_control.msg import RobotStatus, SafetyStatus


class FrankaInterface:
    """
    High-level Python interface for Franka robot control.
    
    This class provides simple methods for controlling the robot in both
    velocity and position modes, with built-in safety monitoring.
    
    Example:
        robot = FrankaInterface()
        robot.connect()
        
        # Velocity control
        robot.set_cartesian_velocity(vx=0.05, vy=0.0, vz=0.0)
        time.sleep(2.0)
        robot.stop()
        
        # Position control
        robot.move_relative(dx=0.1, dy=0.0, dz=0.05)
    """
    
    def __init__(self, 
                 node_name: str = 'franka_python_client',
                 safety_limits: Optional[SafetyLimits] = None):
        """
        Initialize the Franka interface.
        
        Args:
            node_name: Name for the ROS2 node
            safety_limits: Custom safety limits. If None, uses defaults.
        """
        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Create ROS2 node
        self.node = Node(node_name)
        
        # Safety monitor
        self.safety_monitor = SafetyMonitor(safety_limits)
        
        # State tracking
        self.current_mode = ControlMode.IDLE
        self.is_connected = False
        self.latest_joint_state: Optional[JointState] = None
        self.latest_pose: Optional[Pose] = None
        
        # Service clients (will be created in connect())
        self.srv_set_velocity = None
        self.srv_move_to_pose = None
        self.srv_move_relative = None
        self.srv_set_control_mode = None
        self.srv_emergency_stop = None
        
        # Subscribers
        self.sub_joint_states = None
        
        self.node.get_logger().info("Franka Interface initialized")
    
    def connect(self, timeout: float = 5.0) -> bool:
        """
        Connect to the robot control services.
        
        Args:
            timeout: Connection timeout in seconds
            
        Returns:
            True if connection successful, False otherwise
        """
        self.node.get_logger().info("Connecting to robot services...")
        
        try:
            # Create service clients
            # Note: In actual implementation, these would use the generated service types
            # For now, using placeholder names
            
            # self.srv_set_velocity = self.node.create_client(
            #     SetCartesianVelocity, 'set_cartesian_velocity')
            # self.srv_move_to_pose = self.node.create_client(
            #     MoveToPose, 'move_to_pose')
            # self.srv_move_relative = self.node.create_client(
            #     MoveRelative, 'move_relative')
            # self.srv_set_control_mode = self.node.create_client(
            #     SetControlMode, 'set_control_mode')
            # self.srv_emergency_stop = self.node.create_client(
            #     EmergencyStop, 'emergency_stop')
            
            # Create subscribers
            self.sub_joint_states = self.node.create_subscription(
                JointState,
                'joint_states',
                self._joint_state_callback,
                10
            )
            
            # Wait for services to become available
            # start_time = time.time()
            # while time.time() - start_time < timeout:
            #     if (self.srv_set_velocity.service_is_ready() and
            #         self.srv_move_to_pose.service_is_ready()):
            #         self.is_connected = True
            #         self.node.get_logger().info("Successfully connected to robot")
            #         return True
            #     time.sleep(0.1)
            
            # For now, just mark as connected
            self.is_connected = True
            self.node.get_logger().info("Interface ready (services will be connected after build)")
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from robot and cleanup resources."""
        self.node.get_logger().info("Disconnecting from robot...")
        
        # Stop any ongoing motion
        self.stop()
        
        # Destroy service clients
        if self.srv_set_velocity:
            self.node.destroy_client(self.srv_set_velocity)
        if self.srv_move_to_pose:
            self.node.destroy_client(self.srv_move_to_pose)
        if self.srv_move_relative:
            self.node.destroy_client(self.srv_move_relative)
        if self.srv_set_control_mode:
            self.node.destroy_client(self.srv_set_control_mode)
        if self.srv_emergency_stop:
            self.node.destroy_client(self.srv_emergency_stop)
        
        # Destroy subscribers
        if self.sub_joint_states:
            self.node.destroy_subscription(self.sub_joint_states)
        
        self.is_connected = False
    
    def _joint_state_callback(self, msg: JointState):
        """Callback for joint state updates."""
        self.latest_joint_state = msg
        # TODO: Update latest_pose from forward kinematics
    
    def _check_connection(self):
        """Check if connected to robot, raise exception if not."""
        if not self.is_connected:
            raise RuntimeError("Not connected to robot. Call connect() first.")
    
    def _spin_once(self):
        """Process one iteration of ROS callbacks."""
        rclpy.spin_once(self.node, timeout_sec=0.01)
    
    # Velocity Control Methods
    
    def set_cartesian_velocity(self,
                              vx: float = 0.0,
                              vy: float = 0.0,
                              vz: float = 0.0,
                              wx: float = 0.0,
                              wy: float = 0.0,
                              wz: float = 0.0,
                              duration: float = 0.0) -> bool:
        """
        Command Cartesian velocity.
        
        Args:
            vx, vy, vz: Linear velocities in m/s
            wx, wy, wz: Angular velocities in rad/s
            duration: Duration to maintain velocity (0 = continuous)
            
        Returns:
            True if command was accepted, False otherwise
        """
        self._check_connection()
        
        # Safety check
        violation = self.safety_monitor.check_velocity_command(vx, vy, vz, wx, wy, wz)
        if violation:
            self.node.get_logger().error(f"Velocity command rejected: {violation}")
            return False
        
        self.node.get_logger().info(
            f"Setting velocity: linear=[{vx:.3f}, {vy:.3f}, {vz:.3f}], "
            f"angular=[{wx:.3f}, {wy:.3f}, {wz:.3f}]"
        )
        
        # TODO: Call service after build
        # request = SetCartesianVelocity.Request()
        # request.vx = vx
        # request.vy = vy
        # request.vz = vz
        # request.wx = wx
        # request.wy = wy
        # request.wz = wz
        # request.duration = duration
        
        # future = self.srv_set_velocity.call_async(request)
        # rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        
        # if future.result() is not None:
        #     response = future.result()
        #     if response.success:
        #         self.current_mode = ControlMode.VELOCITY
        #         return True
        #     else:
        #         self.node.get_logger().error(f"Velocity command failed: {response.message}")
        #         return False
        
        self.current_mode = ControlMode.VELOCITY
        return True
    
    def stop(self) -> bool:
        """
        Stop all robot motion immediately.
        
        Returns:
            True if stop was successful
        """
        self.node.get_logger().info("Stopping robot motion...")
        return self.set_cartesian_velocity(0, 0, 0, 0, 0, 0)

    # Position Control Methods
    
    def move_to_pose(self,
                    target_pose: List[float],
                    max_velocity: float = 0.1,
                    max_acceleration: float = 1.0,
                    impedance_mode: ImpedanceMode = ImpedanceMode.MEDIUM,
                    wait: bool = True) -> bool:
        """
        Move to an absolute Cartesian pose.
        
        Args:
            target_pose: Target pose [x, y, z, roll, pitch, yaw] or [x, y, z, qx, qy, qz, qw]
            max_velocity: Maximum velocity (m/s)
            max_acceleration: Maximum acceleration (m/s²)
            impedance_mode: Impedance mode for motion
            wait: If True, block until motion completes
            
        Returns:
            True if motion started successfully
        """
        self._check_connection()
        
        # Convert to ROS Pose message
        if len(target_pose) == 6:
            # [x, y, z, roll, pitch, yaw]
            qx, qy, qz, qw = euler_to_quaternion(target_pose[3], target_pose[4], target_pose[5])
            pose = list_to_pose([target_pose[0], target_pose[1], target_pose[2], qx, qy, qz, qw])
        elif len(target_pose) == 7:
            # [x, y, z, qx, qy, qz, qw]
            pose = list_to_pose(target_pose)
        else:
            self.node.get_logger().error("target_pose must be length 6 or 7")
            return False
        
        # Safety check - verify target is within workspace
        violation = self.safety_monitor.check_position(pose.position.x, pose.position.y, pose.position.z)
        if violation:
            self.node.get_logger().error(f"Target pose rejected: {violation}")
            return False
        
        self.node.get_logger().info(f"Moving to pose: [{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]")
        
        # TODO: Call service after build
        # request = MoveToPose.Request()
        # request.target_pose = pose
        # request.max_velocity = max_velocity
        # request.max_acceleration = max_acceleration
        # request.impedance_mode = str(impedance_mode)
        # request.wait_for_completion = wait
        
        # future = self.srv_move_to_pose.call_async(request)
        # rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        # if future.result() is not None:
        #     response = future.result()
        #     if response.success:
        #         self.current_mode = ControlMode.POSITION
        #         self.node.get_logger().info(f"Motion started. Estimated duration: {response.estimated_duration:.2f}s")
        #         return True
        #     else:
        #         self.node.get_logger().error(f"Move to pose failed: {response.message}")
        #         return False
        
        self.current_mode = ControlMode.POSITION
        return True
    
    def move_relative(self,
                     dx: float = 0.0,
                     dy: float = 0.0,
                     dz: float = 0.0,
                     droll: float = 0.0,
                     dpitch: float = 0.0,
                     dyaw: float = 0.0,
                     max_velocity: float = 0.1,
                     impedance_mode: ImpedanceMode = ImpedanceMode.MEDIUM,
                     wait: bool = True) -> bool:
        """
        Move relative to current pose.
        
        Args:
            dx, dy, dz: Translation deltas (m)
            droll, dpitch, dyaw: Rotation deltas (radians)
            max_velocity: Maximum velocity (m/s)
            impedance_mode: Impedance mode for motion
            wait: If True, block until motion completes
            
        Returns:
            True if motion started successfully
        """
        self._check_connection()
        
        # Get current position for safety check
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.node.get_logger().error("Cannot get current pose for relative motion")
            return False
        
        current_pos = (current_pose.position.x, current_pose.position.y, current_pose.position.z)
        
        # Safety check
        violation = self.safety_monitor.check_relative_motion(current_pos, dx, dy, dz)
        if violation:
            self.node.get_logger().error(f"Relative motion rejected: {violation}")
            return False
        
        self.node.get_logger().info(f"Moving relative: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
        
        # TODO: Call service after build
        # request = MoveRelative.Request()
        # request.dx = dx
        # request.dy = dy
        # request.dz = dz
        # request.droll = droll
        # request.dpitch = dpitch
        # request.dyaw = dyaw
        # request.max_velocity = max_velocity
        # request.impedance_mode = str(impedance_mode)
        # request.wait_for_completion = wait
        
        # future = self.srv_move_relative.call_async(request)
        # rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        # if future.result() is not None:
        #     response = future.result()
        #     if response.success:
        #         self.current_mode = ControlMode.POSITION
        #         return True
        #     else:
        #         self.node.get_logger().error(f"Relative move failed: {response.message}")
        #         return False
        
        self.current_mode = ControlMode.POSITION
        return True
    
    # State Query Methods
    
    def get_current_pose(self) -> Optional[Pose]:
        """
        Get current end-effector pose.
        
        Returns:
            Current pose, or None if not available
        """
        # Spin to update latest data
        self._spin_once()
        return self.latest_pose
    
    def get_joint_positions(self) -> Optional[List[float]]:
        """
        Get current joint positions.
        
        Returns:
            List of 7 joint positions in radians, or None if not available
        """
        self._spin_once()
        if self.latest_joint_state:
            return list(self.latest_joint_state.position)
        return None
    
    def get_joint_velocities(self) -> Optional[List[float]]:
        """
        Get current joint velocities.
        
        Returns:
            List of 7 joint velocities in rad/s, or None if not available
        """
        self._spin_once()
        if self.latest_joint_state:
            return list(self.latest_joint_state.velocity)
        return None
    
    def is_moving(self) -> bool:
        """
        Check if robot is currently moving.
        
        Returns:
            True if robot is in motion
        """
        return self.current_mode != ControlMode.IDLE
    
    def get_current_mode(self) -> ControlMode:
        """
        Get current control mode.
        
        Returns:
            Current control mode
        """
        return self.current_mode
    
    # Control Mode Methods
    
    def set_control_mode(self, mode: ControlMode) -> bool:
        """
        Switch control mode.
        
        Args:
            mode: Desired control mode
            
        Returns:
            True if mode switch successful
        """
        self._check_connection()
        
        self.node.get_logger().info(f"Switching to {mode} mode")
        
        # TODO: Call service after build
        # request = SetControlMode.Request()
        # request.mode = str(mode)
        
        # future = self.srv_set_control_mode.call_async(request)
        # rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        
        # if future.result() is not None:
        #     response = future.result()
        #     if response.success:
        #         self.current_mode = mode
        #         return True
        
        self.current_mode = mode
        return True
    
    # Emergency Stop
    
    def emergency_stop(self) -> bool:
        """
        Trigger emergency stop.
        
        Returns:
            True if emergency stop was triggered
        """
        self.node.get_logger().warn("EMERGENCY STOP TRIGGERED")
        
        # TODO: Call service after build
        # request = EmergencyStop.Request()
        # request.stop = True
        
        # future = self.srv_emergency_stop.call_async(request)
        # rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        
        self.current_mode = ControlMode.IDLE
        return True
    
    def reset_emergency_stop(self) -> bool:
        """
        Reset emergency stop.
        
        Returns:
            True if emergency stop was reset
        """
        self.node.get_logger().info("Resetting emergency stop")
        
        # TODO: Call service after build
        # request = EmergencyStop.Request()
        # request.stop = False
        
        # future = self.srv_emergency_stop.call_async(request)
        # rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        
        return True
    
    # Safety Configuration
    
    def set_velocity_limits(self, max_linear: float, max_angular: float):
        """
        Update velocity limits.
        
        Args:
            max_linear: Maximum linear velocity (m/s)
            max_angular: Maximum angular velocity (rad/s)
        """
        self.safety_monitor.limits.velocity.max_linear = max_linear
        self.safety_monitor.limits.velocity.max_angular = max_angular
        self.node.get_logger().info(f"Velocity limits updated: linear={max_linear:.3f} m/s, angular={max_angular:.3f} rad/s")
    
    def set_safety_limits(self, safety_limits: SafetyLimits):
        """
        Update all safety limits.
        
        Args:
            safety_limits: New safety limits configuration
        """
        self.safety_monitor.set_limits(safety_limits)
        self.node.get_logger().info("Safety limits updated")
    
    def get_safety_limits(self) -> SafetyLimits:
        """
        Get current safety limits.
        
        Returns:
            Current safety limits
        """
        return self.safety_monitor.limits
    
    # Utility Methods
    
    def print_current_pose(self):
        """Print current pose in human-readable format."""
        pose = self.get_current_pose()
        if pose:
            print_pose(pose, "Current Pose")
        else:
            print("Current pose not available")
    
    def print_joint_positions(self):
        """Print current joint positions."""
        joints = self.get_joint_positions()
        if joints:
            print("Joint Positions (radians):")
            for i, q in enumerate(joints, 1):
                print(f"  Joint {i}: {q:.4f}")
        else:
            print("Joint positions not available")
    
    def get_statistics(self) -> dict:
        """
        Get interface statistics.
        
        Returns:
            Dictionary with statistics
        """
        return {
            'is_connected': self.is_connected,
            'current_mode': str(self.current_mode),
            'is_moving': self.is_moving(),
            'safety_stats': self.safety_monitor.get_statistics()
        }
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
    
    def __del__(self):
        """Destructor - cleanup resources."""
        if self.is_connected:
            self.disconnect()


# Convenience function for quick testing

def create_robot_interface(safety_mode: str = 'normal') -> FrankaInterface:
    """
    Create a FrankaInterface with preset safety limits.
    
    Args:
        safety_mode: 'conservative', 'normal', or 'fast'
        
    Returns:
        Configured FrankaInterface instance
    """
    if safety_mode == 'conservative':
        limits = SafetyLimits.conservative()
    elif safety_mode == 'normal':
        limits = SafetyLimits()
    elif safety_mode == 'fast':
        limits = SafetyLimits(velocity=VelocityLimits.fast())
    else:
        raise ValueError(f"Unknown safety mode: {safety_mode}")
    
    return FrankaInterface(safety_limits=limits)
