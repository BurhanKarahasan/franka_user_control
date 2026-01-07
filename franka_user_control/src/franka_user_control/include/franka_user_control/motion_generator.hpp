// Copyright (c) 2024 Franka User Control
// Licensed under the Apache License, Version 2.0

#pragma once

#include <array>
#include <Eigen/Dense>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <memory>

namespace franka_user_control {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector7i = Eigen::Matrix<int, 7, 1>;

/**
 * @brief Generates smooth joint position trajectories with synchronized motion
 * 
 * Based on libfranka's MotionGenerator from examples_common.cpp
 * Ensures all joints reach their target simultaneously with smooth acceleration profiles
 */
class MotionGenerator {
public:
  /**
   * @brief Construct a new Motion Generator
   * 
   * @param speed_factor Speed scaling factor (0.0 to 1.0)
   * @param q_goal Target joint positions (7 joints in radians)
   */
  MotionGenerator(double speed_factor, const std::array<double, 7>& q_goal);

  /**
   * @brief Generate next joint position command
   * 
   * @param robot_state Current robot state
   * @param period Time since last call
   * @return std::array<double, 7> Desired joint positions
   */
  std::array<double, 7> operator()(const franka::RobotState& robot_state,
                                   franka::Duration period);

  /**
   * @brief Check if motion is complete
   * 
   * @return true if all joints have reached their targets
   */
  bool isFinished() const { return motion_finished_; }

  /**
   * @brief Reset the motion generator with a new target
   * 
   * @param q_goal New target joint positions
   */
  void reset(const std::array<double, 7>& q_goal);

  /**
   * @brief Get estimated time to complete motion
   * 
   * @return double Estimated duration in seconds
   */
  double getEstimatedDuration() const;

private:
  /**
   * @brief Calculate desired joint positions at given time
   * 
   * @param time Current time in trajectory
   * @param delta_q_d Output: desired joint position deltas
   * @return true if motion is finished
   */
  bool calculateDesiredValues(double time, Vector7d* delta_q_d) const;

  /**
   * @brief Calculate synchronized motion parameters for all joints
   */
  void calculateSynchronizedValues();

  // Motion parameters
  static constexpr double kDeltaQMotionFinished = 1e-6;  // Convergence threshold
  Vector7d dq_max_{2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5};   // Max joint velocities [rad/s]
  Vector7d ddq_max_start_{5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};  // Max start acceleration [rad/s²]
  Vector7d ddq_max_goal_{5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};   // Max goal acceleration [rad/s²]

  // Target and trajectory state
  Vector7d q_goal_;
  Vector7d q_start_;
  Vector7d delta_q_;
  Vector7d dq_max_sync_;
  Vector7d t_1_sync_;
  Vector7d t_2_sync_;
  Vector7d t_f_sync_;
  Vector7d q_1_;

  double time_{0.0};
  bool motion_finished_{false};
};

/**
 * @brief Generates smooth Cartesian pose trajectories
 * 
 * Converts Cartesian targets to joint space and generates smooth motion
 */
class CartesianMotionGenerator {
public:
  /**
   * @brief Construct a new Cartesian Motion Generator
   * 
   * @param speed_factor Speed scaling factor (0.0 to 1.0)
   * @param target_pose Target end-effector pose [x, y, z, roll, pitch, yaw]
   */
  CartesianMotionGenerator(double speed_factor, 
                          const std::array<double, 6>& target_pose);

  /**
   * @brief Generate next command based on current state
   * 
   * @param robot_state Current robot state
   * @param period Time since last call
   * @return std::array<double, 7> Desired joint positions
   */
  std::array<double, 7> operator()(const franka::RobotState& robot_state,
                                   franka::Duration period);

  /**
   * @brief Check if motion is complete
   */
  bool isFinished() const;

private:
  std::array<double, 6> target_pose_;
  std::unique_ptr<MotionGenerator> joint_motion_generator_;
  bool initialized_{false};
};

}  // namespace franka_user_control
