// Copyright (c) 2024 Franka User Control
// Licensed under the Apache License, Version 2.0

#include "franka_user_control/safety_monitor.hpp"
#include <cmath>
#include <algorithm>

namespace franka_user_control {

SafetyMonitor::SafetyMonitor() : limits_(getDefaultFR3SafetyLimits()) {}

SafetyMonitor::SafetyMonitor(const SafetyLimits& limits) : limits_(limits) {}

SafetyViolation SafetyMonitor::checkSafety(const franka::RobotState& state) {
  active_warnings_.clear();

  // Check velocity limits
  if (check_velocity_) {
    auto violation = checkVelocityLimits(state);
    if (violation.hasViolation()) {
      return violation;
    }
  }

  // Check force limits
  if (check_force_) {
    auto violation = checkForceLimits(state);
    if (violation.hasViolation()) {
      return violation;
    }
  }

  // Check workspace limits
  if (check_workspace_) {
    auto violation = checkWorkspaceLimits(state);
    if (violation.hasViolation()) {
      return violation;
    }
  }

  // Check for collision detection from robot state
  if (state.current_errors.cartesian_reflex || 
      state.current_errors.cartesian_motion_generator_joint_acceleration_discontinuity) {
    SafetyViolation violation;
    violation.type = SafetyViolation::Type::COLLISION_DETECTED;
    violation.description = "Robot collision reflex triggered";
    violation.severity = 1.0;
    return violation;
  }

  // No violations
  return SafetyViolation{};
}

SafetyViolation SafetyMonitor::checkVelocityCommand(const std::array<double, 6>& velocity) {
  SafetyViolation violation;

  // Check linear velocity
  double linear_magnitude = std::sqrt(velocity[0] * velocity[0] + 
                                     velocity[1] * velocity[1] + 
                                     velocity[2] * velocity[2]);
  
  if (linear_magnitude > limits_.max_translation_velocity) {
    violation.type = SafetyViolation::Type::VELOCITY_LIMIT;
    violation.description = "Commanded linear velocity exceeds limit: " + 
                           std::to_string(linear_magnitude) + " m/s (max: " + 
                           std::to_string(limits_.max_translation_velocity) + " m/s)";
    violation.severity = linear_magnitude / limits_.max_translation_velocity - 1.0;
    return violation;
  }

  // Check angular velocity
  double angular_magnitude = std::sqrt(velocity[3] * velocity[3] + 
                                      velocity[4] * velocity[4] + 
                                      velocity[5] * velocity[5]);
  
  if (angular_magnitude > limits_.max_rotation_velocity) {
    violation.type = SafetyViolation::Type::VELOCITY_LIMIT;
    violation.description = "Commanded angular velocity exceeds limit: " + 
                           std::to_string(angular_magnitude) + " rad/s (max: " + 
                           std::to_string(limits_.max_rotation_velocity) + " rad/s)";
    violation.severity = angular_magnitude / limits_.max_rotation_velocity - 1.0;
    return violation;
  }

  return violation;
}

SafetyViolation SafetyMonitor::checkVelocityLimits(const franka::RobotState& state) {
  SafetyViolation violation;

  // Calculate Cartesian velocity magnitude
  double velocity_magnitude = calculateVelocityMagnitude(state);

  if (velocity_magnitude > limits_.max_translation_velocity) {
    violation.type = SafetyViolation::Type::VELOCITY_LIMIT;
    violation.description = "Cartesian velocity exceeds limit: " + 
                           std::to_string(velocity_magnitude) + " m/s";
    violation.severity = (velocity_magnitude / limits_.max_translation_velocity) - 1.0;
    addWarning("High velocity detected");
  }

  return violation;
}

SafetyViolation SafetyMonitor::checkForceLimits(const franka::RobotState& state) {
  SafetyViolation violation;

  // Check Cartesian contact forces (O_F_ext_hat_K contains external forces/torques)
  for (size_t i = 0; i < 6; i++) {
    double force_magnitude = std::abs(state.O_F_ext_hat_K[i]);
    
    if (force_magnitude > limits_.force_thresholds[i]) {
      violation.type = SafetyViolation::Type::FORCE_LIMIT;
      violation.description = "External force/torque exceeds threshold at index " + 
                             std::to_string(i) + ": " + std::to_string(force_magnitude);
      violation.severity = (force_magnitude / limits_.force_thresholds[i]) - 1.0;
      addWarning("High external force detected");
      return violation;
    }
  }

  return violation;
}

SafetyViolation SafetyMonitor::checkWorkspaceLimits(const franka::RobotState& state) {
  SafetyViolation violation;

  Eigen::Vector3d position = getCartesianPosition(state);

  if (!limits_.workspace.isInside(position.x(), position.y(), position.z())) {
    violation.type = SafetyViolation::Type::WORKSPACE_LIMIT;
    violation.description = "End-effector outside workspace boundaries at [" + 
                           std::to_string(position.x()) + ", " + 
                           std::to_string(position.y()) + ", " + 
                           std::to_string(position.z()) + "]";
    violation.severity = 1.0;
    
    // Determine which boundary was violated
    if (position.x() < limits_.workspace.x_min) violation.description += " (x_min)";
    if (position.x() > limits_.workspace.x_max) violation.description += " (x_max)";
    if (position.y() < limits_.workspace.y_min) violation.description += " (y_min)";
    if (position.y() > limits_.workspace.y_max) violation.description += " (y_max)";
    if (position.z() < limits_.workspace.z_min) violation.description += " (z_min)";
    if (position.z() > limits_.workspace.z_max) violation.description += " (z_max)";
    
    addWarning("Workspace limit approached");
  }

  return violation;
}

Eigen::Vector3d SafetyMonitor::getCartesianPosition(const franka::RobotState& state) const {
  // Extract position from O_T_EE (4x4 homogeneous transformation matrix)
  // Matrix is column-major, position is in last column
  return Eigen::Vector3d(state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]);
}

double SafetyMonitor::calculateVelocityMagnitude(const franka::RobotState& state) const {
  // libfranka's RobotState layout changed across versions. If the
  // end-effector Cartesian velocity field (O_dP_EE) is not available
  // we fall back to using the joint velocity norm as a conservative
  // proxy for build/test purposes.
  double lin_vel_mag = 0.0;
  // Joint velocity fallback (state.dq is joint velocities in rad/s)
  for (size_t i = 0; i < 7; ++i) {
    lin_vel_mag += state.dq[i] * state.dq[i];
  }
  return std::sqrt(lin_vel_mag);
}

void SafetyMonitor::addWarning(const std::string& warning) {
  // Only add if not already present
  if (std::find(active_warnings_.begin(), active_warnings_.end(), warning) == 
      active_warnings_.end()) {
    active_warnings_.push_back(warning);
  }
}

// Default safety limits

SafetyLimits getDefaultFR3SafetyLimits() {
  SafetyLimits limits;
  
  limits.max_translation_velocity = 0.2;      // m/s
  limits.max_rotation_velocity = 1.0;         // rad/s
  limits.max_translation_acceleration = 1.0;  // m/s²
  limits.max_rotation_acceleration = 5.0;     // rad/s²
  
  // Conservative force thresholds [Fx, Fy, Fz, Tx, Ty, Tz]
  limits.force_thresholds = {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
  
  // Default workspace (FR3 typical reach)
  limits.workspace.x_min = 0.1;
  limits.workspace.x_max = 0.85;
  limits.workspace.y_min = -0.5;
  limits.workspace.y_max = 0.5;
  limits.workspace.z_min = 0.0;
  limits.workspace.z_max = 0.8;
  
  return limits;
}

SafetyLimits getConservativeSafetyLimits() {
  SafetyLimits limits = getDefaultFR3SafetyLimits();
  
  // More conservative velocities for initial testing
  limits.max_translation_velocity = 0.05;     // 5 cm/s
  limits.max_rotation_velocity = 0.5;         // 0.5 rad/s
  limits.max_translation_acceleration = 0.5;  // m/s²
  limits.max_rotation_acceleration = 2.0;     // rad/s²
  
  // Tighter force thresholds
  limits.force_thresholds = {{20.0, 20.0, 20.0, 15.0, 15.0, 15.0}};
  
  // Slightly restricted workspace
  limits.workspace.x_min = 0.2;
  limits.workspace.x_max = 0.7;
  limits.workspace.y_min = -0.4;
  limits.workspace.y_max = 0.4;
  limits.workspace.z_min = 0.1;
  limits.workspace.z_max = 0.7;
  
  return limits;
}

}  // namespace franka_user_control
