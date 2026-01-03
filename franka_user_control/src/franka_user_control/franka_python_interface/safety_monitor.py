"""
Python-side safety monitoring and validation.

Provides additional safety checks before sending commands to the robot.
"""

import math
from typing import List, Optional, Tuple
from .control_modes import SafetyLimits, VelocityLimits, WorkspaceLimits


class SafetyViolation:
    """Represents a safety constraint violation."""
    
    def __init__(self, 
                 violation_type: str,
                 description: str,
                 severity: float = 1.0):
        """
        Initialize a safety violation.
        
        Args:
            violation_type: Type of violation (velocity, force, workspace, etc.)
            description: Human-readable description
            severity: Severity level (0.0 = none, 1.0 = critical)
        """
        self.type = violation_type
        self.description = description
        self.severity = severity
    
    def __str__(self) -> str:
        return f"SafetyViolation({self.type}): {self.description} (severity: {self.severity:.2f})"
    
    def __bool__(self) -> bool:
        """A violation evaluates to True if it exists."""
        return self.type != "none"


class SafetyMonitor:
    """
    Monitors robot commands for safety violations.
    
    This is a Python-side safety check that runs before commands are sent
    to the C++ bridge. The C++ side also has its own safety monitoring.
    """
    
    def __init__(self, safety_limits: Optional[SafetyLimits] = None):
        """
        Initialize safety monitor.
        
        Args:
            safety_limits: Safety limits configuration. If None, uses defaults.
        """
        self.limits = safety_limits or SafetyLimits()
        self.warnings = []
        self.violation_count = 0
    
    def check_velocity_command(self, 
                              vx: float, vy: float, vz: float,
                              wx: float = 0.0, wy: float = 0.0, wz: float = 0.0
                              ) -> Optional[SafetyViolation]:
        """
        Check if a velocity command is safe.
        
        Args:
            vx, vy, vz: Linear velocities (m/s)
            wx, wy, wz: Angular velocities (rad/s)
            
        Returns:
            SafetyViolation if unsafe, None if safe
        """
        # Check linear velocity magnitude
        linear_magnitude = math.sqrt(vx**2 + vy**2 + vz**2)
        if linear_magnitude > self.limits.velocity.max_linear:
            self.violation_count += 1
            return SafetyViolation(
                "velocity_limit",
                f"Linear velocity {linear_magnitude:.3f} m/s exceeds limit "
                f"{self.limits.velocity.max_linear:.3f} m/s",
                severity=linear_magnitude / self.limits.velocity.max_linear - 1.0
            )
        
        # Check angular velocity magnitude
        angular_magnitude = math.sqrt(wx**2 + wy**2 + wz**2)
        if angular_magnitude > self.limits.velocity.max_angular:
            self.violation_count += 1
            return SafetyViolation(
                "velocity_limit",
                f"Angular velocity {angular_magnitude:.3f} rad/s exceeds limit "
                f"{self.limits.velocity.max_angular:.3f} rad/s",
                severity=angular_magnitude / self.limits.velocity.max_angular - 1.0
            )
        
        return None
    
    def check_position(self, 
                      x: float, y: float, z: float
                      ) -> Optional[SafetyViolation]:
        """
        Check if a position is within workspace limits.
        
        Args:
            x, y, z: Cartesian position (m)
            
        Returns:
            SafetyViolation if outside workspace, None if safe
        """
        ws = self.limits.workspace
        
        violations = []
        if x < ws.x_min:
            violations.append(f"x={x:.3f} < x_min={ws.x_min:.3f}")
        if x > ws.x_max:
            violations.append(f"x={x:.3f} > x_max={ws.x_max:.3f}")
        if y < ws.y_min:
            violations.append(f"y={y:.3f} < y_min={ws.y_min:.3f}")
        if y > ws.y_max:
            violations.append(f"y={y:.3f} > y_max={ws.y_max:.3f}")
        if z < ws.z_min:
            violations.append(f"z={z:.3f} < z_min={ws.z_min:.3f}")
        if z > ws.z_max:
            violations.append(f"z={z:.3f} > z_max={ws.z_max:.3f}")
        
        if violations:
            self.violation_count += 1
            return SafetyViolation(
                "workspace_limit",
                f"Position outside workspace: {', '.join(violations)}",
                severity=1.0
            )
        
        return None
    
    def check_relative_motion(self,
                            current_pos: Tuple[float, float, float],
                            dx: float, dy: float, dz: float
                            ) -> Optional[SafetyViolation]:
        """
        Check if a relative motion would exceed workspace limits.
        
        Args:
            current_pos: Current position (x, y, z)
            dx, dy, dz: Relative motion deltas (m)
            
        Returns:
            SafetyViolation if target outside workspace, None if safe
        """
        target_x = current_pos[0] + dx
        target_y = current_pos[1] + dy
        target_z = current_pos[2] + dz
        
        return self.check_position(target_x, target_y, target_z)
    
    def add_warning(self, message: str):
        """
        Add a warning message.
        
        Args:
            message: Warning message
        """
        self.warnings.append(message)
    
    def get_warnings(self) -> List[str]:
        """Get all active warnings."""
        return self.warnings.copy()
    
    def clear_warnings(self):
        """Clear all warnings."""
        self.warnings.clear()
    
    def get_statistics(self) -> dict:
        """
        Get safety monitoring statistics.
        
        Returns:
            Dictionary with statistics
        """
        return {
            'violation_count': self.violation_count,
            'active_warnings': len(self.warnings),
            'velocity_limits': self.limits.velocity.to_dict(),
            'workspace_limits': self.limits.workspace.to_dict()
        }
    
    def reset_statistics(self):
        """Reset violation counters and warnings."""
        self.violation_count = 0
        self.clear_warnings()
    
    def set_limits(self, safety_limits: SafetyLimits):
        """
        Update safety limits.
        
        Args:
            safety_limits: New safety limits
        """
        self.limits = safety_limits
    
    def clamp_velocity(self, 
                      vx: float, vy: float, vz: float,
                      wx: float = 0.0, wy: float = 0.0, wz: float = 0.0
                      ) -> Tuple[float, float, float, float, float, float]:
        """
        Clamp velocities to safety limits.
        
        Args:
            vx, vy, vz: Linear velocities (m/s)
            wx, wy, wz: Angular velocities (rad/s)
            
        Returns:
            Tuple of clamped velocities (vx, vy, vz, wx, wy, wz)
        """
        # Calculate magnitudes
        linear_magnitude = math.sqrt(vx**2 + vy**2 + vz**2)
        angular_magnitude = math.sqrt(wx**2 + wy**2 + wz**2)
        
        # Clamp linear velocity
        if linear_magnitude > self.limits.velocity.max_linear:
            scale = self.limits.velocity.max_linear / linear_magnitude
            vx *= scale
            vy *= scale
            vz *= scale
            self.add_warning(f"Linear velocity clamped to {self.limits.velocity.max_linear:.3f} m/s")
        
        # Clamp angular velocity
        if angular_magnitude > self.limits.velocity.max_angular:
            scale = self.limits.velocity.max_angular / angular_magnitude
            wx *= scale
            wy *= scale
            wz *= scale
            self.add_warning(f"Angular velocity clamped to {self.limits.velocity.max_angular:.3f} rad/s")
        
        return (vx, vy, vz, wx, wy, wz)
