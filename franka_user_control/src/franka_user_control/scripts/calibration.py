#!/usr/bin/env python3
"""
Calibration script for workspace limits and safety parameters.

Helps you safely determine appropriate workspace boundaries and
velocity limits for your specific setup.
"""

import time
import rclpy # type: ignore
from franka_python_interface import FrankaInterface


def calibrate_workspace_limits(robot: FrankaInterface):
    """
    Interactive calibration of workspace boundaries.
    
    Guides the user to move the robot to each boundary point.
    """
    print("\n" + "="*60)
    print("Workspace Calibration")
    print("="*60)
    print()
    print("This will help you determine safe workspace boundaries.")
    print("We'll move the robot to different positions and record them.")
    print()
    
    positions = {}
    
    # X limits
    print("--- X-Axis Limits ---")
    input("Move robot to MINIMUM X position (closest), then press Enter...")
    pose = robot.get_current_pose()
    if pose:
        positions['x_min'] = pose.position.x
        print(f"  Recorded X_min: {positions['x_min']:.3f} m")
    
    input("Move robot to MAXIMUM X position (farthest), then press Enter...")
    pose = robot.get_current_pose()
    if pose:
        positions['x_max'] = pose.position.x
        print(f"  Recorded X_max: {positions['x_max']:.3f} m")
    
    # Y limits
    print("\n--- Y-Axis Limits ---")
    input("Move robot to MINIMUM Y position (left), then press Enter...")
    pose = robot.get_current_pose()
    if pose:
        positions['y_min'] = pose.position.y
        print(f"  Recorded Y_min: {positions['y_min']:.3f} m")
    
    input("Move robot to MAXIMUM Y position (right), then press Enter...")
    pose = robot.get_current_pose()
    if pose:
        positions['y_max'] = pose.position.y
        print(f"  Recorded Y_max: {positions['y_max']:.3f} m")
    
    # Z limits
    print("\n--- Z-Axis Limits ---")
    input("Move robot to MINIMUM Z position (lowest), then press Enter...")
    pose = robot.get_current_pose()
    if pose:
        positions['z_min'] = pose.position.z
        print(f"  Recorded Z_min: {positions['z_min']:.3f} m")
    
    input("Move robot to MAXIMUM Z position (highest), then press Enter...")
    pose = robot.get_current_pose()
    if pose:
        positions['z_max'] = pose.position.z
        print(f"  Recorded Z_max: {positions['z_max']:.3f} m")
    
    # Summary
    print("\n" + "="*60)
    print("Calibration Results")
    print("="*60)
    print("\nAdd these values to config/robot_config.yaml:")
    print("\nworkspace:")
    print(f"  x_min: {positions['x_min']:.3f}")
    print(f"  x_max: {positions['x_max']:.3f}")
    print(f"  y_min: {positions['y_min']:.3f}")
    print(f"  y_max: {positions['y_max']:.3f}")
    print(f"  z_min: {positions['z_min']:.3f}")
    print(f"  z_max: {positions['z_max']:.3f}")
    print()
    
    return positions


def test_velocity_limits(robot: FrankaInterface):
    """
    Test different velocity limits to find comfortable settings.
    """
    print("\n" + "="*60)
    print("Velocity Limit Testing")
    print("="*60)
    print()
    print("We'll test different velocities to help you find comfortable limits.")
    print()
    
    test_velocities = [0.02, 0.05, 0.1, 0.15, 0.2]
    distance = 0.1  # 10 cm test move
    
    for vel in test_velocities:
        print(f"\n--- Testing velocity: {vel:.3f} m/s ---")
        print(f"The robot will move {distance}m at this speed.")
        
        response = input("Continue? (yes/no/done): ").strip().lower()
        if response == 'done':
            break
        if response != 'yes':
            continue
        
        # Move forward
        print("Moving forward...")
        robot.move_relative(dx=distance, max_velocity=vel, wait=True)
        
        # Ask for comfort rating
        rating = input("Rate this velocity (1=too slow, 5=perfect, 10=too fast): ").strip()
        
        # Return
        robot.move_relative(dx=-distance, max_velocity=vel, wait=True)
        
        if rating:
            print(f"  Rating: {rating}")
    
    print("\n" + "="*60)
    print("Velocity Testing Complete")
    print("="*60)
    print("\nRecommendations:")
    print("  - For precise work: 0.02 - 0.05 m/s")
    print("  - For general use: 0.05 - 0.1 m/s")
    print("  - For faster operation: 0.1 - 0.2 m/s")
    print("\nUpdate max_translation_velocity in config/robot_config.yaml")
    print()


def test_impedance_modes(robot: FrankaInterface):
    """
    Test different impedance modes interactively.
    """
    print("\n" + "="*60)
    print("Impedance Mode Testing")
    print("="*60)
    print()
    print("Test different compliance levels by gently pushing the robot")
    print("during motion.")
    print()
    
    from franka_python_interface import ImpedanceMode
    
    modes = [
        (ImpedanceMode.STIFF, "Stiff - High precision, resists push"),
        (ImpedanceMode.MEDIUM, "Medium - Balanced"),
        (ImpedanceMode.SOFT, "Soft - Compliant, gives way easily"),
    ]
    
    for mode, description in modes:
        print(f"\n--- Testing {mode.value.upper()} mode ---")
        print(f"Description: {description}")
        
        input("Press Enter to start movement...")
        
        # Simple back-and-forth motion
        print("Moving... (try gently pushing the robot)")
        robot.move_relative(dx=0.1, max_velocity=0.05, impedance_mode=mode, wait=True)
        time.sleep(1.0)
        robot.move_relative(dx=-0.1, max_velocity=0.05, impedance_mode=mode, wait=True)
        
        feedback = input("Did this feel appropriate? (yes/no): ").strip().lower()
        print(f"  Feedback: {feedback}")
    
    print("\n" + "="*60)
    print("Impedance Testing Complete")
    print("="*60)
    print("\nRecommendations:")
    print("  - Use STIFF for precise positioning tasks")
    print("  - Use MEDIUM for general manipulation")
    print("  - Use SOFT for compliant contact tasks")
    print()


def main():
    """Main calibration function."""
    print("="*60)
    print("  FRANKA ROBOT CALIBRATION")
    print("="*60)
    print()
    print("This script helps you calibrate workspace limits,")
    print("velocity limits, and impedance settings.")
    print()
    print("WARNING: Keep emergency stop button accessible!")
    print()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create robot interface with conservative limits
        print("Initializing robot interface...")
        from franka_python_interface.control_modes import SafetyLimits
        robot = FrankaInterface()
        robot.set_safety_limits(SafetyLimits.conservative())
        
        # Connect
        print("Connecting to robot...")
        if not robot.connect():
            print("ERROR: Failed to connect to robot!")
            print("Make sure the robot is powered on and bridge node is running.")
            return
        
        print("✓ Connected successfully!")
        print()
        
        # Show menu
        while True:
            print("\nCalibration Menu:")
            print("  1. Calibrate workspace limits")
            print("  2. Test velocity limits")
            print("  3. Test impedance modes")
            print("  4. View current settings")
            print("  0. Exit")
            print()
            
            choice = input("Enter choice: ").strip()
            
            if choice == '0':
                break
            elif choice == '1':
                calibrate_workspace_limits(robot)
            elif choice == '2':
                test_velocity_limits(robot)
            elif choice == '3':
                test_impedance_modes(robot)
            elif choice == '4':
                print("\nCurrent Settings:")
                limits = robot.get_safety_limits()
                print(f"  Max linear velocity: {limits.velocity.max_linear:.3f} m/s")
                print(f"  Max angular velocity: {limits.velocity.max_angular:.3f} rad/s")
                print(f"  Workspace X: [{limits.workspace.x_min:.3f}, {limits.workspace.x_max:.3f}]")
                print(f"  Workspace Y: [{limits.workspace.y_min:.3f}, {limits.workspace.y_max:.3f}]")
                print(f"  Workspace Z: [{limits.workspace.z_min:.3f}, {limits.workspace.z_max:.3f}]")
            else:
                print("Invalid choice!")
        
        print("\nCalibration complete!")
        
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user")
    
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