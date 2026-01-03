"""
Control mode definitions and utilities.

Defines the available control modes and impedance settings.
"""

from enum import Enum
from typing import Dict, List


class ControlMode(Enum):
    """Available control modes for the robot."""
    
    IDLE = "idle"
    VELOCITY = "velocity"
    POSITION = "position"
    
    def __str__(self):
        return self.value


class ImpedanceMode(Enum):
    """Predefined impedance modes for different use cases."""
    
    STIFF = "stiff"        # High stiffness for precise positioning
    MEDIUM = "medium"      # Balanced stiffness for general use
    SOFT = "soft"          # Low stiffness for compliant interaction
    VERY_SOFT = "very_soft"  # Very low stiffness for contact tasks
    CUSTOM = "custom"      # User-defined impedance parameters
    
    def __str__(self):
        return self.value
    
    @classmethod
    def get_description(cls, mode: 'ImpedanceMode') -> str:
        """Get a human-readable description of the impedance mode."""
        descriptions = {
            cls.STIFF: "High stiffness for precise positioning tasks",
            cls.MEDIUM: "Balanced stiffness for general purpose tasks",
            cls.SOFT: "Low stiffness for compliant interaction and safety",
            cls.VERY_SOFT: "Very low stiffness for delicate contact tasks",
            cls.CUSTOM: "User-defined impedance parameters"
        }
        return descriptions.get(mode, "Unknown mode")


class VelocityLimits:
    """Velocity limits configuration."""
    
    def __init__(self, 
                 max_linear: float = 0.2,
                 max_angular: float = 1.0):
        """
        Initialize velocity limits.
        
        Args:
            max_linear: Maximum linear velocity (m/s)
            max_angular: Maximum angular velocity (rad/s)
        """
        self.max_linear = max_linear
        self.max_angular = max_angular
    
    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary representation."""
        return {
            'max_linear': self.max_linear,
            'max_angular': self.max_angular
        }
    
    @classmethod
    def conservative(cls) -> 'VelocityLimits':
        """Create conservative velocity limits for testing."""
        return cls(max_linear=0.05, max_angular=0.5)
    
    @classmethod
    def normal(cls) -> 'VelocityLimits':
        """Create normal velocity limits."""
        return cls(max_linear=0.2, max_angular=1.0)
    
    @classmethod
    def fast(cls) -> 'VelocityLimits':
        """Create faster velocity limits (use with caution)."""
        return cls(max_linear=0.5, max_angular=2.0)


class WorkspaceLimits:
    """Workspace boundary limits."""
    
    def __init__(self,
                 x_min: float = 0.1,
                 x_max: float = 0.85,
                 y_min: float = -0.5,
                 y_max: float = 0.5,
                 z_min: float = 0.0,
                 z_max: float = 0.8):
        """
        Initialize workspace limits.
        
        Args:
            x_min, x_max: X-axis boundaries (m)
            y_min, y_max: Y-axis boundaries (m)
            z_min, z_max: Z-axis boundaries (m)
        """
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max
    
    def is_inside(self, x: float, y: float, z: float) -> bool:
        """Check if a position is within workspace limits."""
        return (self.x_min <= x <= self.x_max and
                self.y_min <= y <= self.y_max and
                self.z_min <= z <= self.z_max)
    
    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary representation."""
        return {
            'x_min': self.x_min,
            'x_max': self.x_max,
            'y_min': self.y_min,
            'y_max': self.y_max,
            'z_min': self.z_min,
            'z_max': self.z_max
        }
    
    @classmethod
    def conservative(cls) -> 'WorkspaceLimits':
        """Create conservative workspace limits for testing."""
        return cls(
            x_min=0.2, x_max=0.7,
            y_min=-0.4, y_max=0.4,
            z_min=0.1, z_max=0.7
        )


class SafetyLimits:
    """Complete safety limits configuration."""
    
    def __init__(self,
                 velocity: VelocityLimits = None,
                 workspace: WorkspaceLimits = None,
                 force_thresholds: List[float] = None):
        """
        Initialize safety limits.
        
        Args:
            velocity: Velocity limits
            workspace: Workspace limits
            force_thresholds: External force/torque thresholds [Fx, Fy, Fz, Tx, Ty, Tz]
        """
        self.velocity = velocity or VelocityLimits.normal()
        self.workspace = workspace or WorkspaceLimits()
        self.force_thresholds = force_thresholds or [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
    
    def to_dict(self) -> Dict:
        """Convert to dictionary representation."""
        return {
            'velocity': self.velocity.to_dict(),
            'workspace': self.workspace.to_dict(),
            'force_thresholds': self.force_thresholds
        }
    
    @classmethod
    def conservative(cls) -> 'SafetyLimits':
        """Create conservative safety limits for testing."""
        return cls(
            velocity=VelocityLimits.conservative(),
            workspace=WorkspaceLimits.conservative(),
            force_thresholds=[20.0, 20.0, 20.0, 15.0, 15.0, 15.0]
        )
