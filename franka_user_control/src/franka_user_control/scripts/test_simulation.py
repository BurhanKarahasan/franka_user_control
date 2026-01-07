#!/usr/bin/env python3
"""
Test script for simulation mode.

Tests the interface in simulation without requiring real hardware.
"""

import time
import rclpy # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from rclpy.node import Node # type: ignore


class SimulationTester(Node):
    """Simple tester for simulation mode."""
    
    def __init__(self):
        super().__init__('simulation_tester')
        
        # Publisher for velocity commands
        self.velocity_pub = self.create_publisher(Twist, 'velocity_command', 10)
        
        self.get_logger().info('Simulation tester initialized')
    
    def send_velocity(self, vx=0.0, vy=0.0, vz=0.0, duration=2.0):
        """Send a velocity command."""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        
        self.get_logger().info(f'Sending velocity: [{vx:.3f}, {vy:.3f}, {vz:.3f}] for {duration}s')
        
        # Send for duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.velocity_pub.publish(msg)
            time.sleep(0.01)
        
        # Stop
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.velocity_pub.publish(msg)
        
        self.get_logger().info('Velocity command completed')


def test_basic_movements(tester: SimulationTester):
    """Test basic movements in simulation."""
    print("\n" + "="*60)
    print("Testing Basic Movements in Simulation")
    print("="*60)
    
    # Test +X
    print("\n1. Moving +X...")
    tester.send_velocity(vx=0.05, duration=2.0)
    time.sleep(0.5)
    
    # Test -X
    print("2. Moving -X...")
    tester.send_velocity(vx=-0.05, duration=2.0)
    time.sleep(0.5)
    
    # Test +Y
    print("3. Moving +Y...")
    tester.send_velocity(vy=0.05, duration=2.0)
    time.sleep(0.5)
    
    # Test -Y
    print("4. Moving -Y...")
    tester.send_velocity(vy=-0.05, duration=2.0)
    time.sleep(0.5)
    
    # Test +Z
    print("5. Moving +Z...")
    tester.send_velocity(vz=0.05, duration=2.0)
    time.sleep(0.5)
    
    # Test -Z
    print("6. Moving -Z...")
    tester.send_velocity(vz=-0.05, duration=2.0)
    time.sleep(0.5)
    
    print("\nBasic movement tests completed!")


def test_diagonal_motion(tester: SimulationTester):
    """Test diagonal motion."""
    print("\n" + "="*60)
    print("Testing Diagonal Motion in Simulation")
    print("="*60)
    
    print("\n1. Diagonal XY...")
    tester.send_velocity(vx=0.05, vy=0.05, duration=2.0)
    time.sleep(0.5)
    
    print("2. Return diagonal...")
    tester.send_velocity(vx=-0.05, vy=-0.05, duration=2.0)
    time.sleep(0.5)
    
    print("\nDiagonal motion tests completed!")


def main():
    """Main test function."""
    print("="*60)
    print("  SIMULATION MODE TESTS")
    print("="*60)
    print()
    print("Testing robot control in simulation mode.")
    print("No real hardware required!")
    print()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create tester node
        tester = SimulationTester()
        
        print("Tester initialized. Starting tests...")
        input("Press Enter to start (or Ctrl+C to cancel)...")
        
        # Run tests
        test_basic_movements(tester)
        input("\nPress Enter to continue to next test...")
        
        test_diagonal_motion(tester)
        
        print("\n" + "="*60)
        print("All simulation tests completed!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\nTests interrupted by user")
    
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\nShutting down...")
        rclpy.shutdown()
        print("Done!")


if __name__ == '__main__':
    main()