#!/usr/bin/env python3
"""
Terminal-based User Interface for Franka Robot Control

Provides a simple, menu-driven interface for controlling the robot.
"""

import sys
import time
from typing import Optional
import rclpy # type: ignore

from franka_python_interface import FrankaInterface, ControlMode, ImpedanceMode


class TerminalUI:
    """Terminal-based user interface for robot control."""
    
    def __init__(self):
        """Initialize the terminal UI."""
        self.robot: Optional[FrankaInterface] = None
        self.running = True
    
    def clear_screen(self):
        """Clear the terminal screen."""
        print("\033[2J\033[H", end="")
    
    def print_header(self):
        """Print the UI header."""
        print("=" * 60)
        print("  FRANKA RESEARCH 3 - CONTROL INTERFACE")
        print("=" * 60)
        print()
    
    def print_status(self):
        """Print current robot status."""
        if not self.robot or not self.robot.is_connected:
            print("Status: NOT CONNECTED")
            return
        
        print("Status:")
        print(f"  Connected: {'Yes' if self.robot.is_connected else 'No'}")
        print(f"  Mode: {self.robot.get_current_mode()}")
        print(f"  Moving: {self.robot.is_moving()}")
        
        # Print current pose
        pose = self.robot.get_current_pose()
        if pose:
            print(f"  Position: [{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]")
        
        # Print safety stats
        stats = self.robot.get_statistics()
        if 'safety_stats' in stats:
            safety = stats['safety_stats']
            print(f"  Safety Violations: {safety.get('violation_count', 0)}")
        
        print()
    
    def print_main_menu(self):
        """Print the main menu."""
        print("Main Menu:")
        print("  1. Velocity Control")
        print("  2. Position Control (Absolute)")
        print("  3. Position Control (Relative)")
        print("  4. View Current State")
        print("  5. Safety Settings")
        print("  6. Emergency Stop")
        print("  0. Exit")
        print()
    
    def velocity_control_menu(self):
        """Velocity control submenu."""
        while True:
            self.clear_screen()
            self.print_header()
            print("VELOCITY CONTROL MODE")
            print("-" * 60)
            print()
            
            print("Options:")
            print("  1. Move in +X direction")
            print("  2. Move in -X direction")
            print("  3. Move in +Y direction")
            print("  4. Move in -Y direction")
            print("  5. Move in +Z direction (up)")
            print("  6. Move in -Z direction (down)")
            print("  7. Custom velocity")
            print("  8. Stop")
            print("  0. Back to main menu")
            print()
            
            choice = input("Enter choice: ").strip()
            
            if choice == '0':
                self.robot.stop()
                break
            elif choice == '8':
                self.robot.stop()
                print("Robot stopped.")
                input("Press Enter to continue...")
            else:
                self.execute_velocity_command(choice)
    
    def execute_velocity_command(self, choice: str):
        """Execute a velocity command based on menu choice."""
        velocity = 0.05  # Default velocity (5 cm/s)
        
        # Ask for velocity magnitude
        vel_input = input(f"Enter velocity magnitude (default {velocity} m/s): ").strip()
        if vel_input:
            try:
                velocity = float(vel_input)
            except ValueError:
                print("Invalid input, using default velocity")
        
        # Ask for duration
        duration_input = input("Enter duration in seconds (0 for continuous): ").strip()
        duration = 0.0
        if duration_input:
            try:
                duration = float(duration_input)
            except ValueError:
                print("Invalid input, using continuous motion")
        
        # Execute command
        vx, vy, vz = 0.0, 0.0, 0.0
        
        if choice == '1':
            vx = velocity
            print(f"Moving in +X at {velocity} m/s")
        elif choice == '2':
            vx = -velocity
            print(f"Moving in -X at {velocity} m/s")
        elif choice == '3':
            vy = velocity
            print(f"Moving in +Y at {velocity} m/s")
        elif choice == '4':
            vy = -velocity
            print(f"Moving in -Y at {velocity} m/s")
        elif choice == '5':
            vz = velocity
            print(f"Moving in +Z at {velocity} m/s")
        elif choice == '6':
            vz = -velocity
            print(f"Moving in -Z at {velocity} m/s")
        elif choice == '7':
            vx = float(input("Enter vx (m/s): ") or "0")
            vy = float(input("Enter vy (m/s): ") or "0")
            vz = float(input("Enter vz (m/s): ") or "0")
        
        success = self.robot.set_cartesian_velocity(vx, vy, vz, duration=duration)
        
        if success:
            if duration > 0:
                print(f"Moving for {duration} seconds...")
                time.sleep(duration)
                print("Motion completed")
            else:
                print("Robot moving. Press Enter to stop...")
                input()
                self.robot.stop()
        else:
            print("Failed to set velocity!")
        
        input("Press Enter to continue...")
    
    def position_control_absolute_menu(self):
        """Absolute position control submenu."""
        self.clear_screen()
        self.print_header()
        print("ABSOLUTE POSITION CONTROL")
        print("-" * 60)
        print()
        
        # Get current pose
        current = self.robot.get_current_pose()
        if current:
            print(f"Current position: [{current.position.x:.3f}, {current.position.y:.3f}, {current.position.z:.3f}]")
        print()
        
        # Get target position
        try:
            x = float(input("Enter target X (m): "))
            y = float(input("Enter target Y (m): "))
            z = float(input("Enter target Z (m): "))
            
            # Get velocity
            velocity = float(input("Enter max velocity (m/s, default 0.1): ") or "0.1")
            
            # Get impedance mode
            print("\nImpedance modes:")
            print("  1. Stiff (precise)")
            print("  2. Medium (balanced)")
            print("  3. Soft (compliant)")
            mode_choice = input("Select mode (default 2): ") or "2"
            
            impedance_modes = {
                '1': ImpedanceMode.STIFF,
                '2': ImpedanceMode.MEDIUM,
                '3': ImpedanceMode.SOFT
            }
            impedance = impedance_modes.get(mode_choice, ImpedanceMode.MEDIUM)
            
            # Execute motion
            print(f"\nMoving to [{x:.3f}, {y:.3f}, {z:.3f}]...")
            target = [x, y, z, 0, 0, 0]  # Position only, keep current orientation
            success = self.robot.move_to_pose(target, max_velocity=velocity, impedance_mode=impedance)
            
            if success:
                print("Motion started successfully")
            else:
                print("Failed to start motion!")
        
        except ValueError:
            print("Invalid input!")
        except KeyboardInterrupt:
            print("\nMotion cancelled")
            self.robot.stop()
        
        input("\nPress Enter to continue...")
    
    def position_control_relative_menu(self):
        """Relative position control submenu."""
        self.clear_screen()
        self.print_header()
        print("RELATIVE POSITION CONTROL")
        print("-" * 60)
        print()
        
        # Get current pose
        current = self.robot.get_current_pose()
        if current:
            print(f"Current position: [{current.position.x:.3f}, {current.position.y:.3f}, {current.position.z:.3f}]")
        print()
        
        try:
            dx = float(input("Enter delta X (m): ") or "0")
            dy = float(input("Enter delta Y (m): ") or "0")
            dz = float(input("Enter delta Z (m): ") or "0")
            
            velocity = float(input("Enter max velocity (m/s, default 0.1): ") or "0.1")
            
            # Execute motion
            print(f"\nMoving by [{dx:.3f}, {dy:.3f}, {dz:.3f}]...")
            success = self.robot.move_relative(dx, dy, dz, max_velocity=velocity)
            
            if success:
                print("Motion started successfully")
            else:
                print("Failed to start motion!")
        
        except ValueError:
            print("Invalid input!")
        except KeyboardInterrupt:
            print("\nMotion cancelled")
            self.robot.stop()
        
        input("\nPress Enter to continue...")
    
    def view_state_menu(self):
        """View current robot state."""
        self.clear_screen()
        self.print_header()
        print("CURRENT ROBOT STATE")
        print("-" * 60)
        print()
        
        self.robot.print_current_pose()
        print()
        self.robot.print_joint_positions()
        print()
        
        stats = self.robot.get_statistics()
        print("Statistics:")
        for key, value in stats.items():
            print(f"  {key}: {value}")
        
        input("\nPress Enter to continue...")
    
    def safety_settings_menu(self):
        """Safety settings submenu."""
        self.clear_screen()
        self.print_header()
        print("SAFETY SETTINGS")
        print("-" * 60)
        print()
        
        limits = self.robot.get_safety_limits()
        print("Current Limits:")
        print(f"  Max linear velocity: {limits.velocity.max_linear:.3f} m/s")
        print(f"  Max angular velocity: {limits.velocity.max_angular:.3f} rad/s")
        print()
        
        print("Options:")
        print("  1. Set conservative limits (slow, safe)")
        print("  2. Set normal limits")
        print("  3. Custom velocity limits")
        print("  0. Back")
        print()
        
        choice = input("Enter choice: ").strip()
        
        if choice == '1':
            from franka_python_interface.control_modes import SafetyLimits
            self.robot.set_safety_limits(SafetyLimits.conservative())
            print("Conservative limits set")
        elif choice == '2':
            from franka_python_interface.control_modes import SafetyLimits
            self.robot.set_safety_limits(SafetyLimits())
            print("Normal limits set")
        elif choice == '3':
            try:
                max_linear = float(input("Enter max linear velocity (m/s): "))
                max_angular = float(input("Enter max angular velocity (rad/s): "))
                self.robot.set_velocity_limits(max_linear, max_angular)
                print("Custom limits set")
            except ValueError:
                print("Invalid input!")
        
        input("\nPress Enter to continue...")
    
    def emergency_stop_menu(self):
        """Emergency stop menu."""
        self.clear_screen()
        self.print_header()
        print("EMERGENCY STOP")
        print("-" * 60)
        print()
        
        confirm = input("Trigger emergency stop? (yes/no): ").strip().lower()
        if confirm == 'yes':
            self.robot.emergency_stop()
            print("\nEMERGENCY STOP ACTIVATED!")
            print("Robot has been stopped.")
            
            reset = input("\nReset emergency stop? (yes/no): ").strip().lower()
            if reset == 'yes':
                self.robot.reset_emergency_stop()
                print("Emergency stop reset")
        
        input("\nPress Enter to continue...")
    
    def run(self):
        """Run the main UI loop."""
        # Initialize ROS2
        rclpy.init()
        
        try:
            # Create robot interface
            print("Initializing robot interface...")
            self.robot = FrankaInterface()
            
            # Connect to robot
            print("Connecting to robot...")
            if not self.robot.connect():
                print("Failed to connect to robot!")
                input("Press Enter to exit...")
                return
            
            print("Connected successfully!")
            time.sleep(1)
            
            # Main loop
            while self.running:
                self.clear_screen()
                self.print_header()
                self.print_status()
                self.print_main_menu()
                
                choice = input("Enter choice: ").strip()
                
                if choice == '0':
                    confirm = input("Exit? (yes/no): ").strip().lower()
                    if confirm == 'yes':
                        self.running = False
                elif choice == '1':
                    self.velocity_control_menu()
                elif choice == '2':
                    self.position_control_absolute_menu()
                elif choice == '3':
                    self.position_control_relative_menu()
                elif choice == '4':
                    self.view_state_menu()
                elif choice == '5':
                    self.safety_settings_menu()
                elif choice == '6':
                    self.emergency_stop_menu()
                else:
                    print("Invalid choice!")
                    time.sleep(1)
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        
        finally:
            # Cleanup
            print("\nShutting down...")
            if self.robot:
                self.robot.disconnect()
            rclpy.shutdown()
            print("Goodbye!")


def main():
    """Main entry point."""
    ui = TerminalUI()
    ui.run()


if __name__ == '__main__':
    main()
