#!/usr/bin/env python3
"""
Test script for combined control modes.

Tests switching between velocity and position control,
demonstrating real-world usage patterns.
"""

import time
import rclpy # type: ignore
from franka_python_interface import FrankaInterface, ControlMode, ImpedanceMode


def test_approach_and_position(robot: FrankaInterface):
    """
    Demonstrate approach using velocity, then precise positioning.
    
    This mimics a real task: approach an object slowly with velocity control,
    then switch to position control for precise placement.
    """
    print("\n" + "="*60)
    print("Test: Velocity Approach + Precise Positioning")
    print("="*60)
    
    print("\nScenario: Approaching a target, then precise placement")
    print("  Phase 1: Slow velocity approach")
    print("  Phase 2: Switch to position control for precision")
    
    # Phase 1: Velocity approach
    print("\n--- Phase 1: Velocity Approach ---")
    approach_velocity = 0.02  # 2 cm/s - very slow
    approach_distance = 0.1   # 10 cm total
    approach_time = approach_distance / approach_velocity
    
    print(f"Approaching at {approach_velocity} m/s for {approach_time:.1f}s...")
    robot.set_cartesian_velocity(vx=approach_velocity, duration=approach_time)
    time.sleep(approach_time + 0.5)
    
    print("Stopping velocity motion...")
    robot.stop()
    time.sleep(0.5)
    
    print("Position after approach:")
    robot.print_current_pose()
    
    # Phase 2: Precise positioning
    print("\n--- Phase 2: Precise Positioning ---")
    print("Fine-tuning position with position control...")
    robot.move_relative(dx=0.01, dz=-0.005, max_velocity=0.05,
                       impedance_mode=ImpedanceMode.STIFF, wait=True)
    time.sleep(0.5)
    
    print("Final position:")
    robot.print_current_pose()
    
    # Return to start
    print("\n--- Returning to start ---")
    robot.move_relative(dx=-0.11, dz=0.005, max_velocity=0.1, wait=True)
    
    print("\nTest completed!")


def test_sensor_feedback_pattern(robot: FrankaInterface):
    """
    Simulate a sensor-feedback pattern.
    
    Use velocity control to search, stop when "sensor detects something",
    then use position control to adjust.
    """
    print("\n" + "="*60)
    print("Test: Sensor-Feedback Pattern (Simulated)")
    print("="*60)
    
    print("\nScenario: Searching with velocity until sensor triggers")
    print("  (Simulated: will 'detect' after 1.5 seconds)")
    
    # Start velocity search
    search_velocity = 0.03  # 3 cm/s
    print(f"\nStarting search at {search_velocity} m/s in +Y direction...")
    robot.set_cartesian_velocity(vy=search_velocity)
    
    # Simulate sensor detection after delay
    detection_time = 1.5
    print(f"Searching...")
    time.sleep(detection_time)
    
    # "Sensor detected something!"
    print("\n>>> SENSOR DETECTION! <<<")
    print("Stopping velocity motion...")
    robot.stop()
    time.sleep(0.5)
    
    print("Position at detection:")
    robot.print_current_pose()
    
    # Adjust position based on "sensor feedback"
    print("\nAdjusting position based on sensor feedback...")
    robot.move_relative(dy=-0.01, dz=0.02, max_velocity=0.05,
                       impedance_mode=ImpedanceMode.MEDIUM, wait=True)
    
    print("Final adjusted position:")
    robot.print_current_pose()
    
    # Return
    print("\nReturning to start...")
    travel_distance = search_velocity * detection_time
    robot.move_relative(dy=-travel_distance + 0.01, dz=-0.02, 
                       max_velocity=0.1, wait=True)
    
    print("\nTest completed!")


def test_multi_waypoint_trajectory(robot: FrankaInterface):
    """
    Test moving through multiple waypoints with position control.
    """
    print("\n" + "="*60)
    print("Test: Multi-Waypoint Trajectory")
    print("="*60)
    
    print("\nMoving through 4 waypoints to create a pattern...")
    
    waypoints = [
        (0.1, 0.0, 0.0, "Waypoint 1"),
        (0.1, 0.1, 0.0, "Waypoint 2"),
        (0.0, 0.1, 0.05, "Waypoint 3"),
        (0.0, 0.0, 0.05, "Waypoint 4"),
    ]
    
    print("Initial position:")
    robot.print_current_pose()
    
    for i, (dx, dy, dz, name) in enumerate(waypoints, 1):
        print(f"\n{i}. Moving to {name}: ({dx:+.2f}, {dy:+.2f}, {dz:+.2f})...")
        robot.move_relative(dx=dx, dy=dy, dz=dz, max_velocity=0.1,
                           impedance_mode=ImpedanceMode.MEDIUM, wait=True)
        time.sleep(0.5)
        print(f"   Reached {name}")
    
    # Return to start
    print("\nReturning to starting position...")
    robot.move_relative(dx=0.0, dy=-0.1, dz=-0.05, max_velocity=0.1, wait=True)
    
    print("\nFinal position:")
    robot.print_current_pose()
    
    print("\nTest completed!")


def test_velocity_with_limits(robot: FrankaInterface):
    """
    Test velocity control with dynamically changing limits.
    """
    print("\n" + "="*60)
    print("Test: Dynamic Safety Limits")
    print("="*60)
    
    print("\nDemonstrating dynamic safety limit adjustment...")
    
    # Get initial limits
    initial_limits = robot.get_safety_limits()
    print(f"Initial max velocity: {initial_limits.velocity.max_linear:.3f} m/s")
    
    # Set conservative limits
    print("\n1. Setting conservative limits...")
    robot.set_velocity_limits(0.03, 0.5)
    print("   Max velocity: 0.03 m/s")
    
    print("   Moving at conservative speed...")
    robot.set_cartesian_velocity(vx=0.03, duration=2.0)
    time.sleep(2.5)
    robot.print_current_pose()
    
    # Increase limits
    print("\n2. Increasing limits...")
    robot.set_velocity_limits(0.1, 1.0)
    print("   Max velocity: 0.1 m/s")
    
    print("   Moving at higher speed...")
    robot.set_cartesian_velocity(vx=-0.08, duration=1.5)
    time.sleep(2.0)
    robot.print_current_pose()
    
    # Restore initial limits
    robot.set_velocity_limits(
        initial_limits.velocity.max_linear,
        initial_limits.velocity.max_angular
    )
    print(f"\n3. Restored initial limits: {initial_limits.velocity.max_linear:.3f} m/s")
    
    print("\nTest completed!")


def test_compliant_motion(robot: FrankaInterface):
    """
    Test compliant motion (soft impedance for contact).
    """
    print("\n" + "="*60)
    print("Test: Compliant Motion")
    print("="*60)
    
    print("\nDemonstrating compliant motion for contact tasks...")
    print("Using SOFT impedance mode for safe contact.")
    
    # Move down slowly with soft impedance
    print("\n1. Moving down with soft impedance...")
    robot.move_relative(dz=-0.05, max_velocity=0.02,
                       impedance_mode=ImpedanceMode.SOFT, wait=True)
    time.sleep(1.0)
    
    print("   At lower position (would contact surface)")
    robot.print_current_pose()
    
    # Move back up
    print("\n2. Moving back up...")
    robot.move_relative(dz=0.05, max_velocity=0.05,
                       impedance_mode=ImpedanceMode.MEDIUM, wait=True)
    
    print("   Returned to original height")
    robot.print_current_pose()
    
    print("\nTest completed!")
    print("Note: In real contact scenarios, you'd also monitor force/torque")


def main():
    """Main test function."""
    print("="*60)
    print("  FRANKA COMBINED CONTROL TESTS")
    print("="*60)
    print()
    print("This script demonstrates real-world usage patterns")
    print("combining velocity and position control.")
    print()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create robot interface
        print("Initializing robot interface...")
        robot = FrankaInterface()
        
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
        
        # Test 1: Approach + Position
        test_approach_and_position(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 2: Sensor feedback pattern
        test_sensor_feedback_pattern(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 3: Multi-waypoint
        test_multi_waypoint_trajectory(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 4: Dynamic limits
        test_velocity_with_limits(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 5: Compliant motion
        test_compliant_motion(robot)
        
        # Final state
        print("\n" + "="*60)
        print("All tests completed!")
        print("="*60)
        print("\nFinal state:")
        robot.print_current_pose()
        
        # Print statistics
        stats = robot.get_statistics()
        print("\nTest Statistics:")
        for key, value in stats.items():
            if key != 'safety_stats':
                print(f"  {key}: {value}")
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
