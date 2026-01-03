// Copyright (c) 2024 Franka User Control
// Licensed under the Apache License, Version 2.0

#include "franka_user_control/impedance_controller.hpp"
#include <algorithm>

namespace franka_user_control {

ImpedanceController::ImpedanceController(size_t dq_filter_size,
                                         const std::array<double, 7>& K_P,
                                         const std::array<double, 7>& K_D)
    : dq_filter_size_(dq_filter_size), K_P_(K_P), K_D_(K_D) {
  std::fill(dq_d_.begin(), dq_d_.end(), 0);
  dq_buffer_ = std::vector<double>(dq_filter_size_ * 7, 0.0);
}

franka::Torques ImpedanceController::step(const franka::RobotState& state) {
  updateDQFilter(state);

  std::array<double, 7> tau_J_d;
  for (size_t i = 0; i < 7; i++) {
    // PD control: tau = Kp * (q_desired - q_actual) + Kd * (dq_desired - dq_actual)
    tau_J_d[i] = K_P_[i] * (state.q_d[i] - state.q[i]) + 
                 K_D_[i] * (dq_d_[i] - getDQFiltered(i));
  }
  
  return tau_J_d;
}

void ImpedanceController::updateDQFilter(const franka::RobotState& state) {
  for (size_t i = 0; i < 7; i++) {
    dq_buffer_[dq_current_filter_position_ * 7 + i] = state.dq[i];
  }
  dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
}

double ImpedanceController::getDQFiltered(size_t index) const {
  double value = 0;
  for (size_t i = index; i < 7 * dq_filter_size_; i += 7) {
    value += dq_buffer_[i];
  }
  return value / static_cast<double>(dq_filter_size_);
}

void ImpedanceController::resetFilter() {
  std::fill(dq_buffer_.begin(), dq_buffer_.end(), 0.0);
  dq_current_filter_position_ = 0;
}

// ImpedanceControllerFactory implementation

ImpedanceController ImpedanceControllerFactory::create(ImpedanceMode mode, size_t filter_size) {
  auto [K_P, K_D] = getGainsForMode(mode);
  return ImpedanceController(filter_size, K_P, K_D);
}

ImpedanceController ImpedanceControllerFactory::createCustom(
    const std::array<double, 7>& K_P,
    const std::array<double, 7>& K_D,
    size_t filter_size) {
  return ImpedanceController(filter_size, K_P, K_D);
}

std::pair<std::array<double, 7>, std::array<double, 7>> 
ImpedanceControllerFactory::getGainsForMode(ImpedanceMode mode) {
  std::array<double, 7> K_P;
  std::array<double, 7> K_D;

  switch (mode) {
    case ImpedanceMode::STIFF:
      // High stiffness for precise positioning
      K_P = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
      K_D = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
      break;

    case ImpedanceMode::MEDIUM:
      // Balanced stiffness for general use
      K_P = {{400.0, 400.0, 400.0, 400.0, 150.0, 100.0, 30.0}};
      K_D = {{30.0, 30.0, 30.0, 30.0, 20.0, 15.0, 10.0}};
      break;

    case ImpedanceMode::SOFT:
      // Low stiffness for compliant interaction
      K_P = {{200.0, 200.0, 200.0, 200.0, 100.0, 50.0, 20.0}};
      K_D = {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 5.0}};
      break;

    case ImpedanceMode::CUSTOM:
      // Return default medium values for custom mode
      // User should override with custom values
      K_P = {{400.0, 400.0, 400.0, 400.0, 150.0, 100.0, 30.0}};
      K_D = {{30.0, 30.0, 30.0, 30.0, 20.0, 15.0, 10.0}};
      break;
  }

  return {K_P, K_D};
}

// CartesianImpedance implementation

std::array<double, 7> CartesianImpedance::toJointProportionalGains() const {
  // Simplified mapping from Cartesian to joint impedance
  // In a full implementation, this would use the Jacobian transpose
  // For now, use a heuristic mapping
  
  double avg_translational = (translational_stiffness[0] + 
                              translational_stiffness[1] + 
                              translational_stiffness[2]) / 3.0;
  
  double avg_rotational = (rotational_stiffness[0] + 
                           rotational_stiffness[1] + 
                           rotational_stiffness[2]) / 3.0;

  // Map to joint gains (simplified)
  // Joints 1-4 primarily affect translation
  // Joints 5-7 primarily affect rotation
  std::array<double, 7> joint_gains;
  joint_gains[0] = avg_translational * 0.2;  // Joint 1
  joint_gains[1] = avg_translational * 0.2;  // Joint 2
  joint_gains[2] = avg_translational * 0.2;  // Joint 3
  joint_gains[3] = avg_translational * 0.2;  // Joint 4
  joint_gains[4] = avg_rotational * 0.1;     // Joint 5
  joint_gains[5] = avg_rotational * 0.07;    // Joint 6
  joint_gains[6] = avg_rotational * 0.03;    // Joint 7

  return joint_gains;
}

}  // namespace franka_user_control
