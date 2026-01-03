#!/usr/bin/env python3
"""
Fake robot state publisher for simulation.

Publishes simulated joint states and robot status without real hardware.
"""

import math
import numpy as np
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from sensor_msgs.msg import JointState # type: ignore
from geometry_msgs.msg import Pose # type: ignore
from builtin_interfaces.msg import Time # type: ignore


class FakeRobotStatePublisher(Node):
    """
    Simulates robot state publishing for testing without hardware.
    """
    
    def __init__(self):
        super().__init__('fake_robot_state_publisher')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 100)  # Hz
        self.declare_parameter('simulate_motion', True)
        self.declare_parameter('add_noise', False)
        
        # Get parameters
        publish_rate = self.get_parameter('publish_rate').value
        self.simulate_motion = self.get_parameter('simulate_motion').value
        self.add_noise = self.get_parameter('add_noise').value
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 
            'joint_states', 
            10
        )
        
        # State variables
        self.joint_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # Home position
        self.joint_velocities = [0.0] * 7
        self.target_positions = self.joint_positions.copy()
        self.target_velocities = [0.0] * 7
        
        # Motion simulation parameters
        self.motion_speed = 0.1  # rad/s for position interpolation
        self.velocity_alpha = 0.05  # Smoothing factor for velocity changes
        
        # Timer
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.timer_callback
        )
        
        # Simulation time
        self.sim_time = 0.0
        self.dt = 1.0 / publish_rate
        
        self.get_logger().info('Fake robot state publisher started')
        self.get_logger().info(f'Publishing at {publish_rate} Hz')
        if self.simulate_motion:
            self.get_logger().info('Motion simulation enabled')
    
    def timer_callback(self):
        """Publish joint states at regular intervals."""
        self.sim_time += self.dt
        
        # Simulate motion if enabled
        if self.simulate_motion:
            self.simulate_joint_motion()
        
        # Create and publish joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fr3_link0'
        
        msg.name = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # Add noise if requested
        if self.add_noise:
            noise_level = 0.001  # Small noise in radians
            positions = [q + np.random.normal(0, noise_level) for q in self.joint_positions]
            velocities = [v + np.random.normal(0, noise_level * 10) for v in self.joint_velocities]
        else:
            positions = self.joint_positions
            velocities = self.joint_velocities
        
        msg.position = positions
        msg.velocity = velocities
        msg.effort = [0.0] * 7  # No effort simulation for now
        
        self.joint_state_pub.publish(msg)
    
    def simulate_joint_motion(self):
        """Simulate smooth joint motion towards target positions."""
        # Interpolate positions towards targets
        for i in range(7):
            # Position interpolation
            error = self.target_positions[i] - self.joint_positions[i]
            
            if abs(error) > 0.0001:  # Small threshold
                # Move towards target
                step = np.sign(error) * min(abs(error), self.motion_speed * self.dt)
                self.joint_positions[i] += step
                
                # Update velocity (derivative of position)
                self.joint_velocities[i] = step / self.dt
            else:
                # Close enough to target
                self.joint_positions[i] = self.target_positions[i]
                self.joint_velocities[i] *= 0.9  # Decay to zero
            
            # Smooth velocity changes
            vel_error = self.target_velocities[i] - self.joint_velocities[i]
            self.joint_velocities[i] += self.velocity_alpha * vel_error
    
    def set_target_positions(self, positions):
        """Set target joint positions for simulation."""
        if len(positions) == 7:
            self.target_positions = positions
    
    def set_target_velocities(self, velocities):
        """Set target joint velocities for simulation."""
        if len(velocities) == 7:
            self.target_velocities = velocities


def main(args=None):
    rclpy.init(args=args)
    
    node = FakeRobotStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()