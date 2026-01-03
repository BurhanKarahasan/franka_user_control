#!/usr/bin/env python3
"""
Simulation bridge node.

Provides the same services as the C++ bridge node but works in simulation
without requiring real hardware or libfranka.
"""

import time
import math
import numpy as np
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Pose, Twist # type: ignore
from sensor_msgs.msg import JointState # type: ignore


class SimulationBridgeNode(Node):
    """
    Simulation version of the Franka bridge node.
    
    Accepts the same service calls as the real bridge but simulates
    the robot behavior without hardware.
    """
    
    def __init__(self):
        super().__init__('simulation_bridge_node')
        
        # Declare parameters
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('conservative_limits', True)
        
        # State
        self.current_pose = Pose()
        self.current_pose.position.x = 0.5
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.4
        self.current_pose.orientation.w = 1.0
        
        self.current_velocity = [0.0] * 6  # [vx, vy, vz, wx, wy, wz]
        self.target_velocity = [0.0] * 6
        
        self.control_mode = 'idle'
        self.is_moving = False
        
        # Subscribers - listen for velocity commands
        self.velocity_cmd_sub = self.create_subscription(
            Twist,
            'velocity_command',
            self.velocity_command_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(Pose, 'current_pose', 10)
        self.status_pub = self.create_publisher(JointState, 'robot_status', 10)
        
        # Simulation timer (100 Hz)
        self.sim_timer = self.create_timer(0.01, self.simulation_update)
        
        # Status timer (10 Hz)
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        # Velocity ramping parameters
        self.velocity_ramp_rate = 0.05  # How fast to ramp velocity
        
        self.get_logger().info('Simulation bridge node started')
        self.get_logger().info('Ready to accept commands')
        
        # Print initial state
        self.get_logger().info(f'Initial position: [{self.current_pose.position.x:.3f}, '
                              f'{self.current_pose.position.y:.3f}, '
                              f'{self.current_pose.position.z:.3f}]')
    
    def velocity_command_callback(self, msg: Twist):
        """Handle velocity command."""
        self.target_velocity = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z
        ]
        
        self.control_mode = 'velocity'
        self.is_moving = any(abs(v) > 0.001 for v in self.target_velocity)
        
        if self.is_moving:
            self.get_logger().info(
                f'Velocity command: linear=[{msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f}]'
            )
    
    def simulation_update(self):
        """Update simulation state (called at 100 Hz)."""
        dt = 0.01  # 100 Hz
        
        # Ramp velocity towards target
        for i in range(6):
            vel_error = self.target_velocity[i] - self.current_velocity[i]
            self.current_velocity[i] += np.sign(vel_error) * min(
                abs(vel_error),
                self.velocity_ramp_rate * dt
            )
        
        # Update position based on velocity
        self.current_pose.position.x += self.current_velocity[0] * dt
        self.current_pose.position.y += self.current_velocity[1] * dt
        self.current_pose.position.z += self.current_velocity[2] * dt
        
        # Update orientation (simplified - just update based on angular velocity)
        # In a real simulation, this would use proper quaternion integration
        # For now, just track if rotating
        
        # Check if stopped
        if self.control_mode == 'velocity':
            self.is_moving = any(abs(v) > 0.001 for v in self.current_velocity)
    
    def publish_status(self):
        """Publish current robot status."""
        # Publish pose
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.header.frame_id = 'fr3_link0'
        self.pose_pub.publish(self.current_pose)
        
        # Publish status as joint state (reusing message type)
        status_msg = JointState()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = 'fr3_link0'
        status_msg.name = ['simulation_status']
        status_msg.position = [
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        ]
        status_msg.velocity = self.current_velocity
        self.status_pub.publish(status_msg)
    
    def move_relative(self, dx, dy, dz, max_velocity=0.1):
        """
        Simulate relative movement.
        
        In simulation, we'll just update the position directly after a delay.
        """
        self.control_mode = 'position'
        self.is_moving = True
        
        # Calculate duration based on distance and velocity
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        duration = distance / max_velocity if distance > 0 else 0
        
        self.get_logger().info(
            f'Simulating relative move: [{dx:.3f}, {dy:.3f}, {dz:.3f}] '
            f'over {duration:.2f}s'
        )
        
        # Start position
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        start_z = self.current_pose.position.z
        
        # Simulate motion over time
        start_time = time.time()
        while time.time() - start_time < duration:
            t = (time.time() - start_time) / duration
            
            # Linear interpolation
            self.current_pose.position.x = start_x + dx * t
            self.current_pose.position.y = start_y + dy * t
            self.current_pose.position.z = start_z + dz * t
            
            time.sleep(0.01)
        
        # Ensure we reach exact target
        self.current_pose.position.x = start_x + dx
        self.current_pose.position.y = start_y + dy
        self.current_pose.position.z = start_z + dz
        
        self.is_moving = False
        self.control_mode = 'idle'
        
        self.get_logger().info('Movement completed')


def main(args=None):
    rclpy.init(args=args)
    
    node = SimulationBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()