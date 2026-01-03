#!/usr/bin/env python3
"""
Test script for position control.

Tests absolute and relative position movements.
"""

import time
import rclpy # type: ignore
from franka_python_interface import FrankaInterface, ImpedanceMode


def test_relative_movements(robot: FrankaInterface):
    """Test relative position movements."""
    print("\n" + "="*60)
    print("Testing Relative Movements")
    print("="*60)
    
    step_size = 0.05  # 5 cm
    velocity = 0.1    # 10 cm/s
    
    print("\nInitial position:")
    robot.print_current_pose()
    
    # Move +X
    print(f"\n1. Moving +{step_size}m in X...")
    robot.move_relative(dx=step_size, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    robot.print_current_pose()
    
    # Move -X (return)
    print(f"\n2. Moving -{step_size}m in X (returning)...")
    robot.move_relative(dx=-step_size, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    robot.print_current_pose()
    
    # Move +Y
    print(f"\n3. Moving +{step_size}m in Y...")
    robot.move_relative(dy=step_size, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    robot.print_current_pose()
    
    # Move -Y (return)
    print(f"\n4. Moving -{step_size}m in Y (returning)...")
    robot.move_relative(dy=-step_size, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    robot.print_current_pose()
    
    # Move +Z
    print(f"\n5. Moving +{step_size}m in Z (up)...")
    robot.move_relative(dz=step_size, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    robot.print_current_pose()
    
    # Move -Z (return)
    print(f"\n6. Moving -{step_size}m in Z (down)...")
    robot.move_relative(dz=-step_size, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    robot.print_current_pose()
    
    print("\nRelative movement tests completed!")


def test_diagonal_movements(robot: FrankaInterface):
    """Test diagonal movements (combined axes)."""
    print("\n" + "="*60)
    print("Testing Diagonal Movements")
    print("="*60)
    
    step_size = 0.05  # 5 cm
    velocity = 0.1    # 10 cm/s
    
    print("\nInitial position:")
    robot.print_current_pose()
    
    # Diagonal XY
    print(f"\n1. Moving diagonally in XY plane...")
    robot.move_relative(dx=step_size, dy=step_size, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    robot.print_current_pose()
    
    # Return
    print(f"\n2. Returning...")
    robot.move_relative(dx=-step_size, dy=-step_size, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    robot.print_current_pose()
    
    # 3D diagonal
    print(f"\n3. Moving diagonally in 3D (XYZ)...")
    robot.move_relative(dx=step_size, dy=step_size, dz=step_size, 
                       max_velocity=velocity, wait=True)
    time.sleep(0.5)
    robot.print_current_pose()
    
    # Return
    print(f"\n4. Returning...")
    robot.move_relative(dx=-step_size, dy=-step_size, dz=-step_size,
                       max_velocity=velocity, wait=True)
    time.sleep(0.5)
    robot.print_current_pose()
    
    print("\nDiagonal movement tests completed!")


def test_square_pattern(robot: FrankaInterface):
    """Test moving in a square pattern."""
    print("\n" + "="*60)
    print("Testing Square Pattern")
    print("="*60)
    
    side_length = 0.1  # 10 cm
    velocity = 0.1     # 10 cm/s
    
    print(f"\nDrawing a {side_length}m x {side_length}m square in XY plane...")
    print("Starting position:")
    robot.print_current_pose()
    
    # Side 1: +X
    print("\n1. Moving along +X...")
    robot.move_relative(dx=side_length, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    
    # Side 2: +Y
    print("2. Moving along +Y...")
    robot.move_relative(dy=side_length, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    
    # Side 3: -X
    print("3. Moving along -X...")
    robot.move_relative(dx=-side_length, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    
    # Side 4: -Y (return to start)
    print("4. Moving along -Y (returning to start)...")
    robot.move_relative(dy=-side_length, max_velocity=velocity, wait=True)
    time.sleep(0.5)
    
    print("\nFinal position:")
    robot.print_current_pose()
    print("\nSquare pattern completed!")


def test_impedance_modes(robot: FrankaInterface):
    """Test different impedance modes."""
    print("\n" + "="*60)
    print("Testing Impedance Modes")
    print("="*60)
    
    distance = 0.05
    velocity = 0.05
    
    print("\nYou should notice different 'feel' for each impedance mode.")
    print("Stiff mode is more precise, soft mode is more compliant.")
    
    # Stiff mode
    print("\n1. Testing STIFF impedance mode...")
    robot.move_relative(dx=distance, max_velocity=velocity, 
                       impedance_mode=ImpedanceMode.STIFF, wait=True)
    time.sleep(1.0)
    robot.move_relative(dx=-distance, max_velocity=velocity,
                       impedance_mode=ImpedanceMode.STIFF, wait=True)
    time.sleep(1.0)
    print("   Stiff mode completed")
    
    # Medium mode
    print("\n2. Testing MEDIUM impedance mode...")
    robot.move_relative(dx=distance, max_velocity=velocity,
                       impedance_mode=ImpedanceMode.MEDIUM, wait=True)
    time.sleep(1.0)
    robot.move_relative(dx=-distance, max_velocity=velocity,
                       impedance_mode=ImpedanceMode.MEDIUM, wait=True)
    time.sleep(1.0)
    print("   Medium mode completed")
    
    # Soft mode
    print("\n3. Testing SOFT impedance mode...")
    robot.move_relative(dx=distance, max_velocity=velocity,
                       impedance_mode=ImpedanceMode.SOFT, wait=True)
    time.sleep(1.0)
    robot.move_relative(dx=-distance, max_velocity=velocity,
                       impedance_mode=ImpedanceMode.SOFT, wait=True)
    time.sleep(1.0)
    print("   Soft mode completed")
    
    print("\nImpedance mode tests completed!")
    print("Note: Differences may be subtle without external forces")


def test_velocity_scaling(robot: FrankaInterface):
    """Test different velocity scalings."""
    print("\n" + "="*60)
    print("Testing Velocity Scaling")
    print("="*60)
    
    distance = 0.1  # 10 cm
    
    print("\nMoving the same distance at different speeds...")
    
    # Slow
    print(f"\n1. Moving at 0.05 m/s (slow)...")
    start_time = time.time()
    robot.move_relative(dx=distance, max_velocity=0.05, wait=True)
    slow_time = time.time() - start_time
    print(f"   Completed in {slow_time:.2f} seconds")
    time.sleep(0.5)
    
    # Return
    robot.move_relative(dx=-distance, max_velocity=0.1, wait=True)
    time.sleep(0.5)
    
    # Medium
    print(f"\n2. Moving at 0.1 m/s (medium)...")
    start_time = time.time()
    robot.move_relative(dx=distance, max_velocity=0.1, wait=True)
    medium_time = time.time() - start_time
    print(f"   Completed in {medium_time:.2f} seconds")
    time.sleep(0.5)
    
    # Return
    robot.move_relative(dx=-distance, max_velocity=0.1, wait=True)
    time.sleep(0.5)
    
    # Fast
    print(f"\n3. Moving at 0.2 m/s (fast)...")
    start_time = time.time()
    robot.move_relative(dx=distance, max_velocity=0.2, wait=True)
    fast_time = time.time() - start_time
    print(f"   Completed in {fast_time:.2f} seconds")
    time.sleep(0.5)
    
    # Return
    robot.move_relative(dx=-distance, max_velocity=0.1, wait=True)
    
    print(f"\nTiming comparison:")
    print(f"  Slow (0.05 m/s): {slow_time:.2f}s")
    print(f"  Medium (0.1 m/s): {medium_time:.2f}s")
    print(f"  Fast (0.2 m/s): {fast_time:.2f}s")
    
    print("\nVelocity scaling tests completed!")


def main():
    """Main test function."""
    print("="*60)
    print("  FRANKA POSITION CONTROL TESTS")
    print("="*60)
    print()
    print("This script will test various position control functions.")
    print("Make sure the robot has clear space to move!")
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
        robot.print_joint_positions()
        print()
        
        # Run tests
        input("Press Enter to start tests (or Ctrl+C to cancel)...")
        
        # Test 1: Relative movements
        test_relative_movements(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 2: Diagonal movements
        test_diagonal_movements(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 3: Square pattern
        test_square_pattern(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 4: Impedance modes
        test_impedance_modes(robot)
        input("\nPress Enter to continue to next test...")
        
        # Test 5: Velocity scaling
        test_velocity_scaling(robot)
        
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
