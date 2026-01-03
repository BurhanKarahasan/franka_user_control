// Copyright (c) 2024 Franka User Control
// Licensed under the Apache License, Version 2.0

#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>
#include <franka/robot_state.h>
#include <franka/control_types.h>

namespace franka_user_control {

/**
 * @brief PD controller for joint-level impedance control
 * 
 * Based on libfranka's motion_with_control.cpp Controller class
 * Provides compliant motion with configurable stiffness and damping
 */
class ImpedanceController {
public:
  /**
   * @brief Construct impedance controller
   * 
   * @param dq_filter_size Size of velocity filter window
   * @param K_P Proportional gains for each joint
   * @param K_D Derivative gains for each joint
   */
  ImpedanceController(size_t dq_filter_size,
                     const std::array<double, 7>& K_P,
                     const std::array<double, 7>& K_D);

  /**
   * @brief Compute control torques based on current state
   * 
   * @param state Current robot state
   * @return franka::Torques Commanded joint torques
   */
  franka::Torques step(const franka::RobotState& state);

  /**
   * @brief Set proportional gains
   */
  void setProportionalGains(const std::array<double, 7>& K_P) { K_P_ = K_P; }

  /**
   * @brief Set derivative gains
   */
  void setDerivativeGains(const std::array<double, 7>& K_D) { K_D_ = K_D; }

  /**
   * @brief Get current gains
   */
  std::array<double, 7> getProportionalGains() const { return K_P_; }
  std::array<double, 7> getDerivativeGains() const { return K_D_; }

  /**
   * @brief Reset the velocity filter
   */
  void resetFilter();

private:
  /**
   * @brief Update the velocity filter with new measurement
   */
  void updateDQFilter(const franka::RobotState& state);

  /**
   * @brief Get filtered velocity for a specific joint
   */
  double getDQFiltered(size_t index) const;

  // Filter parameters
  size_t dq_current_filter_position_{0};
  size_t dq_filter_size_;
  std::vector<double> dq_buffer_;

  // Control gains
  std::array<double, 7> K_P_;  // Proportional gains (stiffness)
  std::array<double, 7> K_D_;  // Derivative gains (damping)

  // Desired velocities (typically zero for position control)
  std::array<double, 7> dq_d_{0, 0, 0, 0, 0, 0, 0};
};

/**
 * @brief Preset impedance modes for common use cases
 */
enum class ImpedanceMode {
  STIFF,    // High stiffness for precise positioning
  MEDIUM,   // Balanced stiffness for general use
  SOFT,     // Low stiffness for compliant interaction
  CUSTOM    // User-defined parameters
};

/**
 * @brief Factory for creating impedance controllers with preset modes
 */
class ImpedanceControllerFactory {
public:
  /**
   * @brief Create controller with preset impedance mode
   * 
   * @param mode Impedance preset
   * @param filter_size Velocity filter window size
   * @return ImpedanceController Configured controller
   */
  static ImpedanceController create(ImpedanceMode mode, size_t filter_size = 5);

  /**
   * @brief Create controller with custom gains
   * 
   * @param K_P Custom proportional gains
   * @param K_D Custom derivative gains
   * @param filter_size Velocity filter window size
   * @return ImpedanceController Configured controller
   */
  static ImpedanceController createCustom(const std::array<double, 7>& K_P,
                                         const std::array<double, 7>& K_D,
                                         size_t filter_size = 5);

  /**
   * @brief Get gains for a specific mode
   */
  static std::pair<std::array<double, 7>, std::array<double, 7>> 
    getGainsForMode(ImpedanceMode mode);
};

/**
 * @brief Cartesian impedance parameters
 */
struct CartesianImpedance {
  std::array<double, 3> translational_stiffness;  // N/m
  std::array<double, 3> rotational_stiffness;     // Nm/rad
  
  /**
   * @brief Convert to joint impedance gains (simplified mapping)
   */
  std::array<double, 7> toJointProportionalGains() const;
};

}  // namespace franka_user_control
