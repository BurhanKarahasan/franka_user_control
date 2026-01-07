// Copyright (c) 2024 Franka User Control
// Licensed under the Apache License, Version 2.0

#pragma once

#include <memory>
#include <string>
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <franka/robot.h>
#include <franka/exception.h>

#include "franka_user_control/motion_generator.hpp"
#include "franka_user_control/velocity_controller.hpp"
#include "franka_user_control/impedance_controller.hpp"
#include "franka_user_control/safety_monitor.hpp"

// Include generated service/message types
#include "franka_user_control/srv/set_cartesian_velocity.hpp"
#include "franka_user_control/srv/move_to_pose.hpp"
#include "franka_user_control/srv/move_relative.hpp"
#include "franka_user_control/srv/set_control_mode.hpp"
#include "franka_user_control/srv/emergency_stop.hpp"
#include "franka_user_control/msg/robot_status.hpp"
#include "franka_user_control/msg/safety_status.hpp"
#include "franka_user_control/msg/controller_state.hpp"

namespace franka_user_control {

/**
 * @brief Control mode enumeration
 */
enum class ControlMode {
  IDLE,       // No active control
  VELOCITY,   // Cartesian velocity control
  POSITION    // Position control with motion generation
};

/**
 * @brief Main ROS2 node bridging Python interface to libfranka
 * 
 * This node:
 * - Connects to Franka robot via libfranka
 * - Exposes ROS2 services for velocity and position control
 * - Publishes robot status and safety information
 * - Manages control mode switching
 * - Monitors safety constraints
 */
class FrankaBridgeNode : public rclcpp::Node {
public:
  /**
   * @brief Construct the bridge node
   */
  explicit FrankaBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Destructor - ensures safe shutdown
   */
  ~FrankaBridgeNode();

  /**
   * @brief Initialize connection to robot
   * 
   * @return true if connection successful
   */
  bool initialize();

  /**
   * @brief Start the control loop in a separate thread
   */
  void startControlLoop();

  /**
   * @brief Stop the control loop
   */
  void stopControlLoop();

private:
  // Service callbacks
  /**
   * @brief Handle velocity command service
   */
  void handleSetVelocity();

  /**
   * @brief Handle move to pose service
   */
  void handleMoveToPose();

  /**
   * @brief Handle relative move service
   */
  void handleMoveRelative();

  /**
   * @brief Handle control mode switch service
   */
  void handleSetControlMode();

  /**
   * @brief Handle emergency stop service
   */
  void handleEmergencyStop();

  // Control loop functions
  /**
   * @brief Main control loop running in separate thread
   */
  void controlLoop();

  /**
   * @brief Execute velocity control loop (called from controlLoop)
   */
  void executeVelocityControl();

  /**
   * @brief Execute position control loop (called from controlLoop)
   */
  void executePositionControl();

  /**
   * @brief Velocity control callback for libfranka
   */
  franka::CartesianVelocities velocityControlCallback(
    const franka::RobotState& robot_state,
    franka::Duration period);

  /**
   * @brief Position control callback for libfranka
   */
  franka::JointPositions positionControlCallback(
    const franka::RobotState& robot_state,
    franka::Duration period);

  /**
   * @brief Combined motion + impedance control callback
   */
  void combinedControlCallback(
    const franka::RobotState& robot_state,
    franka::Duration period);

  // Status publishing
  /**
   * @brief Publish robot status at regular intervals
   */
  void publishStatus();

  /**
   * @brief Timer callback for status publishing
   */
  void statusTimerCallback();

  // Helper functions
  /**
   * @brief Load parameters from ROS2 parameter server
   */
  void loadParameters();

  /**
   * @brief Set default robot behavior (collision thresholds, impedance)
   */
  void setDefaultBehavior();

  /**
   * @brief Convert Cartesian pose to joint angles (inverse kinematics)
   * 
   * @param pose Target Cartesian pose
   * @return std::array<double, 7> Joint angles, empty if IK fails
   */
  std::array<double, 7> poseToJoints(const geometry_msgs::msg::Pose& pose);

  /**
   * @brief Get current end-effector pose
   */
  geometry_msgs::msg::Pose getCurrentPose() const;

  // ROS2 communication
  // Services (typed)
  rclcpp::Service<franka_user_control::srv::SetCartesianVelocity>::SharedPtr srv_set_velocity_;
  rclcpp::Service<franka_user_control::srv::MoveToPose>::SharedPtr srv_move_to_pose_;
  rclcpp::Service<franka_user_control::srv::MoveRelative>::SharedPtr srv_move_relative_;
  rclcpp::Service<franka_user_control::srv::SetControlMode>::SharedPtr srv_set_control_mode_;
  rclcpp::Service<franka_user_control::srv::EmergencyStop>::SharedPtr srv_emergency_stop_;

  // Publishers (typed)
  rclcpp::Publisher<franka_user_control::msg::RobotStatus>::SharedPtr pub_robot_status_;
  rclcpp::Publisher<franka_user_control::msg::SafetyStatus>::SharedPtr pub_safety_status_;
  rclcpp::Publisher<franka_user_control::msg::ControllerState>::SharedPtr pub_controller_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

  // Timers
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Robot connection
  std::unique_ptr<franka::Robot> robot_;
  std::string robot_ip_;
  bool robot_connected_{false};

  // Controllers
  std::unique_ptr<VelocityController> velocity_controller_;
  std::unique_ptr<MotionGenerator> motion_generator_;
  std::unique_ptr<ImpedanceController> impedance_controller_;
  std::unique_ptr<SafetyMonitor> safety_monitor_;

  // State
  ControlMode current_mode_{ControlMode::IDLE};
  std::atomic<bool> emergency_stop_active_{false};
  std::atomic<bool> control_loop_running_{false};
  std::thread control_thread_;

  // Safety limits
  SafetyLimits safety_limits_;

  // Mutex for thread-safe access
  mutable std::mutex state_mutex_;
  franka::RobotState latest_robot_state_;
};

}  // namespace franka_user_control
