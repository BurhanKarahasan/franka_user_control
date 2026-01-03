// Copyright (c) 2024 Franka User Control
// Licensed under the Apache License, Version 2.0

#pragma once

#include <array>
#include <Eigen/Dense>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/control_types.h>

namespace franka_user_control {

/**
 * @brief Controls robot with Cartesian velocity commands
 * 
 * Based on libfranka's cartesian_velocity_example.cpp
 * Provides smooth velocity ramping to prevent safety reflex triggering
 */
class VelocityController {
public:
  /**
   * @brief Construct a new Velocity Controller
   * 
   * @param max_linear_velocity Maximum linear velocity (m/s)
   * @param max_angular_velocity Maximum angular velocity (rad/s)
   * @param ramp_time Time to ramp up/down velocity (seconds)
   */
  VelocityController(double max_linear_velocity = 0.2,
                    double max_angular_velocity = 1.0,
                    double ramp_time = 0.5);

  /**
   * @brief Set target Cartesian velocity
   * 
   * @param velocity Target velocity [vx, vy, vz, wx, wy, wz]
   */
  void setTargetVelocity(const std::array<double, 6>& velocity);

  /**
   * @brief Stop the robot smoothly
   */
  void stop();

  /**
   * @brief Generate velocity command for current control cycle
   * 
   * @param robot_state Current robot state
   * @param period Time since last call
   * @return franka::CartesianVelocities Commanded velocities
   */
  franka::CartesianVelocities step(const franka::RobotState& robot_state,
                                  franka::Duration period);

  /**
   * @brief Check if velocity has reached zero (stopped)
   */
  bool isStopped() const;

  /**
   * @brief Get current commanded velocity
   */
  std::array<double, 6> getCurrentVelocity() const { return current_velocity_; }

  /**
   * @brief Get target velocity
   */
  std::array<double, 6> getTargetVelocity() const { return target_velocity_; }

  /**
   * @brief Set velocity limits
   */
  void setVelocityLimits(double max_linear, double max_angular);

  /**
   * @brief Set ramp time for smooth acceleration
   */
  void setRampTime(double ramp_time) { ramp_time_ = ramp_time; }

private:
  /**
   * @brief Apply velocity ramping (smooth acceleration/deceleration)
   * 
   * @param dt Time step
   */
  void applyVelocityRamping(double dt);

  /**
   * @brief Clamp velocity to maximum limits
   * 
   * @param velocity Velocity to clamp
   */
  void clampVelocity(std::array<double, 6>& velocity) const;

  /**
   * @brief Check if velocity is near zero
   */
  bool isNearZero(const std::array<double, 6>& velocity, double threshold = 1e-4) const;

  // Velocity limits
  double max_linear_velocity_;   // m/s
  double max_angular_velocity_;  // rad/s
  double ramp_time_;             // seconds

  // Current state
  std::array<double, 6> target_velocity_{0, 0, 0, 0, 0, 0};   // Desired velocity
  std::array<double, 6> current_velocity_{0, 0, 0, 0, 0, 0};  // Actual commanded velocity
  
  // Timing
  double elapsed_time_{0.0};
  bool stopping_{false};
};

/**
 * @brief Manages continuous velocity control with duration support
 * 
 * Allows commanding velocity for a specific duration, then automatically stopping
 */
class TimedVelocityController {
public:
  TimedVelocityController(double max_linear_velocity = 0.2,
                         double max_angular_velocity = 1.0);

  /**
   * @brief Set velocity with optional duration
   * 
   * @param velocity Target velocity
   * @param duration Duration to maintain velocity (0 = continuous)
   */
  void setVelocity(const std::array<double, 6>& velocity, double duration = 0.0);

  /**
   * @brief Update controller and generate command
   */
  franka::CartesianVelocities step(const franka::RobotState& robot_state,
                                  franka::Duration period);

  /**
   * @brief Stop the motion
   */
  void stop();

  /**
   * @brief Check if timed motion is complete
   */
  bool isFinished() const;

private:
  VelocityController velocity_controller_;
  double duration_{0.0};        // Commanded duration
  double elapsed_time_{0.0};    // Time since velocity command
  bool has_duration_{false};    // Whether a duration is set
};

}  // namespace franka_user_control
