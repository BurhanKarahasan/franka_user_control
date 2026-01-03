"""
Utility functions for pose conversions and transformations.
"""

import math
from typing import List, Tuple
from geometry_msgs.msg import Pose, Point, Quaternion # type: ignore


def pose_to_list(pose: Pose) -> List[float]:
    """
    Convert ROS Pose message to list [x, y, z, qx, qy, qz, qw].
    
    Args:
        pose: ROS Pose message
        
    Returns:
        List of 7 floats representing position and orientation
    """
    return [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ]


def list_to_pose(pose_list: List[float]) -> Pose:
    """
    Convert list [x, y, z, qx, qy, qz, qw] to ROS Pose message.
    
    Args:
        pose_list: List of 7 floats [x, y, z, qx, qy, qz, qw]
        
    Returns:
        ROS Pose message
    """
    if len(pose_list) != 7:
        raise ValueError("pose_list must have exactly 7 elements")
    
    pose = Pose()
    pose.position.x = pose_list[0]
    pose.position.y = pose_list[1]
    pose.position.z = pose_list[2]
    pose.orientation.x = pose_list[3]
    pose.orientation.y = pose_list[4]
    pose.orientation.z = pose_list[5]
    pose.orientation.w = pose_list[6]
    
    return pose


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion.
    
    Args:
        roll: Rotation around X-axis (radians)
        pitch: Rotation around Y-axis (radians)
        yaw: Rotation around Z-axis (radians)
        
    Returns:
        Tuple (qx, qy, qz, qw) representing quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return (qx, qy, qz, qw)


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        qx, qy, qz, qw: Quaternion components
        
    Returns:
        Tuple (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return (roll, pitch, yaw)


def clamp(value: float, min_value: float, max_value: float) -> float:
    """
    Clamp a value between min and max.
    
    Args:
        value: Value to clamp
        min_value: Minimum allowed value
        max_value: Maximum allowed value
        
    Returns:
        Clamped value
    """
    return max(min_value, min(max_value, value))


def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to [-pi, pi].
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def interpolate_poses(start_pose: List[float], 
                     end_pose: List[float], 
                     t: float) -> List[float]:
    """
    Linearly interpolate between two poses.
    
    Args:
        start_pose: Starting pose [x, y, z, qx, qy, qz, qw]
        end_pose: Ending pose [x, y, z, qx, qy, qz, qw]
        t: Interpolation parameter (0.0 to 1.0)
        
    Returns:
        Interpolated pose
    """
    if not (0.0 <= t <= 1.0):
        raise ValueError("Interpolation parameter t must be between 0 and 1")
    
    # Linear interpolation for position
    position = [
        start_pose[i] + t * (end_pose[i] - start_pose[i])
        for i in range(3)
    ]
    
    # Spherical linear interpolation (SLERP) for orientation
    # Simplified version - for production use, consider using transforms3d or scipy
    q_start = start_pose[3:7]
    q_end = end_pose[3:7]
    
    # Compute dot product
    dot = sum(q_start[i] * q_end[i] for i in range(4))
    
    # If quaternions are close, use linear interpolation
    if abs(dot) > 0.9995:
        orientation = [
            q_start[i] + t * (q_end[i] - q_start[i])
            for i in range(4)
        ]
        # Normalize
        norm = math.sqrt(sum(q * q for q in orientation))
        orientation = [q / norm for q in orientation]
    else:
        # SLERP
        if dot < 0:
            q_end = [-q for q in q_end]
            dot = -dot
        
        theta_0 = math.acos(dot)
        theta = theta_0 * t
        
        q_orthogonal = [q_end[i] - q_start[i] * dot for i in range(4)]
        norm = math.sqrt(sum(q * q for q in q_orthogonal))
        q_orthogonal = [q / norm for q in q_orthogonal]
        
        orientation = [
            q_start[i] * math.cos(theta) + q_orthogonal[i] * math.sin(theta)
            for i in range(4)
        ]
    
    return position + orientation


def print_pose(pose: Pose, name: str = "Pose"):
    """
    Print a pose in a human-readable format.
    
    Args:
        pose: ROS Pose message
        name: Name/label for the pose
    """
    roll, pitch, yaw = quaternion_to_euler(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )
    
    print(f"{name}:")
    print(f"  Position: [{pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}]")
    print(f"  Orientation (RPY): [{math.degrees(roll):.2f}°, {math.degrees(pitch):.2f}°, {math.degrees(yaw):.2f}°]")
    print(f"  Orientation (Quat): [{pose.orientation.x:.4f}, {pose.orientation.y:.4f}, {pose.orientation.z:.4f}, {pose.orientation.w:.4f}]")
