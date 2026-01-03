#!/usr/bin/env python3
"""
Test script for velocity control.

Tests basic Cartesian velocity commands with the robot.
"""

import time
import rclpy # type: ignore
from franka_python_interface import FrankaInterface


def test_linear_velocities(robot: FrankaInterface):
    """Test linear velocities in each direction."""
    print("\n" + "="*60)
    print("Testing Linear Velocities")
    print("="*60)
    
    velocity = 0.03  # 3 cm/s - conservative for testing
    duration = 2.0   # seconds
    
    # Test +X
    print(f"\n1. Moving +X at {velocity} m/s for {duration}s...")
    robot.set_cartesian_velocity(vx=velocity, duration=duration)
    time.sleep(duration + 0.5)
    robot.print_current_pose()
    
    # Test -X (return)
    print(f"\n2. Moving -X at {velocity} m/s for {duration}s...")
    robot.set_cartesian_velocity(vx=-velocity, duration=duration)
    time.sleep(duration + 0.5)
    robot.print_current_pose()
    
    # Test +Y
    print(f"\n3. Moving +Y at {velocity} m/s for {duration}s...")
    robot.set_cartesian_velocity(vy=velocity, duration=duration)
    time.sleep(duration + 0.5)
    robot.print_current_pose()
    
    # Test -Y (return)
    print(f"\n4. Moving -Y at {velocity} m/s for {duration}s...")
    robot.set_cartesian_velocity(vy=-velocity, duration=duration)
    time.sleep(duration + 0.5)
    robot.print_current_pose()
    
    # Test +Z
    print(f"\n5. Moving +Z at {velocity} m/s for {duration}s...")
    robot.set_cartesian_velocity(vz=velocity, duration=duration)
    time.sleep(duration + 0.5)
    robot.print_current_pose()
    
    # Test -Z (return)
    print(f"\n6. Moving -Z at {velocity} m/s for {duration}s...")
    robot.set_cartesian_velocity(vz=-velocity, duration=duration)
    time.sleep(duration + 0.5)
    robot.print_current_pose()
    
    print("\nLinear velocity tests completed!")


def test_diagonal_motion(robot: FrankaInterface):
    """Test diagonal motion (combined velocities)."""
    print("\n" + "="*60)
    print("Testing Diagonal Motion")
    print("="*60)
    
    velocity = 0.03  # 3 cm/s
    duration = 3.0
    
    # Diagonal in XY plane
    print(f"\n1. Moving diagonally in XY plane...")
    robot.set_cartesian_velocity(vx=velocity, vy=velocity, duration=duration)
    time.sleep(duration + 0.5)
    robot.print_current_pose()
    
    # Return
    print(f"\n2. Returning...")
    robot.set_cartesian_velocity(vx=-velocity, vy=-velocity, duration=duration)
    time.sleep(duration + 0.5)
    robot.print_current_pose()
    
    print("\nDiagonal motion tests completed!")


def test_continuous_motion(robot: FrankaInterface):
    """Test continuous velocity control (user-stopped)."""
    print("\n" + "="*60)
    print("Testing Continuous Motion")
    print("="*60)
    
    velocity = 0.02  # 2 cm/s - very slow for safety
    
    print(f"\nStarting continuous motion at {velocity} m/s in +X direction...")
    print("Motion will continue until you press Enter to stop.")
    
    # Start motion
    robot.set_cartesian_velocity(vx=velocity)
    
    # Wait for user to stop
    input("\nPress Enter to stop the motion...")
    
    # Stop
    robot.stop()
    print("Motion stopped!")
    robot.print_current_pose()
    
    print("\nContinuous motion test completed!")


def test_velocity_ramping(robot: FrankaInterface):
    """Test velocity ramping (gradual acceleration/deceleration)."""
    print("\n" + "="*60)
    print("Testing Velocity Ramping")
    print("="*60)
    
    print("\nThis test demonstrates smooth velocity ramping.")
    print("The robot will gradually accelerate and decelerate.")
    
    # Slow acceleration
    print("\n1. Slow acceleration in +X...")
    robot.set_cartesian_velocity(vx=0.05, duration=3.0)
    time.sleep(3.5)
    
    # Stop (smooth deceleration)
    print("2. Smooth deceleration to stop...")
    robot.stop()
    time.sleep(1.0)
    
    robot.print_current_pose()
    print("\nVelocity ramping test completed!")


def test_safety_limits(robot: FrankaInterface):
    """Test safety limit enforcement."""
    print("\n" + "="*60)
    print("Testing Safety Limits")
    print("="*60)
    
    # Try to exceed velocity limit (should be rejected)
    print("\n1. Attempting to exceed velocity limit...")
    limits = robot.get_safety_limits()
    excessive_velocity = limits.velocity.max_linear * 2.0
    
    print(f"   Current limit: {limits.velocity.max_linear:.3f} m/s")
    print(f"   Attempting: {excessive_velocity:.3f} m/s")
    
    success = robot.set_cartesian_velocity(vx=excessive_velocity)
    
    if success:
        print("   WARNING: Excessive velocity was accepted (this shouldn't happen!)")
        robot.stop()
    else:
        print("   ✓ Excessive velocity correctly rejected by safety monitor")
    
    # Test velocity clamping
    print("\n2. Testing velocity clamping...")
    # The safety monitor should clamp to limits
    clamped = robot.safety_monitor.clamp_velocity(excessive_velocity, 0, 0)
    print(f"   Clamped velocity: {clamped[0]:.3f} m/s")
    print(f"   ✓ Velocity clamped to safe limit")
    
    print("\nSafety limit tests completed!")


def main():
    """Main test function."""
    print("="*60)
    print("  FRANKA VELOCITY CONTROL TESTS")
    print("="*60)
    print()
    print("This script will test various velocity control functions.")
    print("Make sure the robot has clear space to move!")
    print()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create robot interface with conservative limits
        print("Initializing robot interface with conservative safety limits...")
        robot = FrankaInterface()
        
        # Use conservative limits for testing
        from franka_python_interface.control_modes import SafetyLimits
        robot.set_safety_limits(SafetyLimits.conservative())
        
        # Connect
        print("Connecting to robot...")
        if not robot.connect():
            print("ERROR: Failed to connect to robot!")
            return
        
        print("✓ Connected successfully!")
        print()
        
        # Display initial state
        print("Initial state:")
        robot.print_current_pose()
        print()
        
        # Run tests
        input("Press Enter to start tests (or Ctrl+C to cancel)...")
        
        # Test 1: Linear velocities
        test_linear_velocities(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 2: Diagonal motion
        test_diagonal_motion(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 3: Continuous motion
        test_continuous_motion(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 4: Velocity ramping
        test_velocity_ramping(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 5: Safety limits
        test_safety_limits(robot)
        
        # Final state
        print("\n" + "="*60)
        print("All tests completed!")
        print("="*60)
        print("\nFinal state:")
        robot.print_current_pose()
        
        # Print statistics
        stats = robot.get_statistics()
        print("\nTest Statistics:")
        print(f"  Safety violations: {stats['safety_stats']['violation_count']}")
        
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
