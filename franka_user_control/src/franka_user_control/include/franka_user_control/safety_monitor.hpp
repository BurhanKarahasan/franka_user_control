// Copyright (c) 2024 Franka User Control
// Licensed under the Apache License, Version 2.0

#pragma once

#include <array>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <franka/robot_state.h>

namespace franka_user_control {

/**
 * @brief Workspace boundary definition
 */
struct WorkspaceLimits {
  double x_min, x_max;
  double y_min, y_max;
  double z_min, z_max;

  bool isInside(double x, double y, double z) const {
    return (x >= x_min && x <= x_max &&
            y >= y_min && y <= y_max &&
            z >= z_min && z <= z_max);
  }
};

/**
 * @brief Safety limits configuration
 */
struct SafetyLimits {
  double max_translation_velocity;      // m/s
  double max_rotation_velocity;         // rad/s
  double max_translation_acceleration;  // m/s²
  double max_rotation_acceleration;     // rad/s²
  
  std::array<double, 6> force_thresholds;  // [Fx, Fy, Fz, Tx, Ty, Tz]
  WorkspaceLimits workspace;
};

/**
 * @brief Safety violation information
 */
struct SafetyViolation {
  enum class Type {
    NONE,
    VELOCITY_LIMIT,
    ACCELERATION_LIMIT,
    FORCE_LIMIT,
    WORKSPACE_LIMIT,
    COLLISION_DETECTED
  };

  Type type{Type::NONE};
  std::string description;
  double severity{0.0};  // 0.0 = no violation, 1.0 = severe violation

  bool hasViolation() const { return type != Type::NONE; }
};

/**
 * @brief Monitors robot safety constraints
 * 
 * Checks velocity limits, force limits, workspace boundaries, etc.
 */
class SafetyMonitor {
public:
  /**
   * @brief Construct safety monitor with default limits
   */
  SafetyMonitor();

  /**
   * @brief Construct with custom safety limits
   */
  explicit SafetyMonitor(const SafetyLimits& limits);

  /**
   * @brief Check if current state violates any safety constraints
   * 
   * @param state Current robot state
   * @return SafetyViolation Violation information (NONE if safe)
   */
  SafetyViolation checkSafety(const franka::RobotState& state);

  /**
   * @brief Check if commanded velocity is safe
   * 
   * @param velocity Commanded Cartesian velocity [vx, vy, vz, wx, wy, wz]
   * @return SafetyViolation Violation information
   */
  SafetyViolation checkVelocityCommand(const std::array<double, 6>& velocity);

  /**
   * @brief Update safety limits
   */
  void setSafetyLimits(const SafetyLimits& limits) { limits_ = limits; }

  /**
   * @brief Get current safety limits
   */
  SafetyLimits getSafetyLimits() const { return limits_; }

  /**
   * @brief Set workspace boundaries
   */
  void setWorkspaceLimits(const WorkspaceLimits& workspace) { 
    limits_.workspace = workspace; 
  }

  /**
   * @brief Get all active warnings
   */
  std::vector<std::string> getActiveWarnings() const { return active_warnings_; }

  /**
   * @brief Clear all warnings
   */
  void clearWarnings() { active_warnings_.clear(); }

  /**
   * @brief Enable/disable specific safety check
   */
  void enableVelocityCheck(bool enable) { check_velocity_ = enable; }
  void enableForceCheck(bool enable) { check_force_ = enable; }
  void enableWorkspaceCheck(bool enable) { check_workspace_ = enable; }

private:
  /**
   * @brief Check velocity limits
   */
  SafetyViolation checkVelocityLimits(const franka::RobotState& state);

  /**
   * @brief Check force/torque limits
   */
  SafetyViolation checkForceLimits(const franka::RobotState& state);

  /**
   * @brief Check workspace boundaries
   */
  SafetyViolation checkWorkspaceLimits(const franka::RobotState& state);

  /**
   * @brief Extract Cartesian position from robot state
   */
  Eigen::Vector3d getCartesianPosition(const franka::RobotState& state) const;

  /**
   * @brief Calculate Cartesian velocity magnitude
   */
  double calculateVelocityMagnitude(const franka::RobotState& state) const;

  /**
   * @brief Add warning to active warnings list
   */
  void addWarning(const std::string& warning);

  // Safety configuration
  SafetyLimits limits_;

  // Safety check flags
  bool check_velocity_{true};
  bool check_force_{true};
  bool check_workspace_{true};

  // State tracking
  std::vector<std::string> active_warnings_;
  
  // Previous state for acceleration checking
  std::array<double, 6> prev_velocity_{0, 0, 0, 0, 0, 0};
  double prev_timestamp_{0.0};
};

/**
 * @brief Default safety limits for Franka Research 3
 */
SafetyLimits getDefaultFR3SafetyLimits();

/**
 * @brief Conservative safety limits for initial testing
 */
SafetyLimits getConservativeSafetyLimits();

}  // namespace franka_user_control
