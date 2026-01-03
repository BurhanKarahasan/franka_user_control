// Copyright (c) 2024 Franka User Control
// Licensed under the Apache License, Version 2.0

#include "franka_user_control/velocity_controller.hpp"
#include <cmath>
#include <algorithm>

namespace franka_user_control {

VelocityController::VelocityController(double max_linear_velocity,
                                       double max_angular_velocity,
                                       double ramp_time)
    : max_linear_velocity_(max_linear_velocity),
      max_angular_velocity_(max_angular_velocity),
      ramp_time_(ramp_time) {}

void VelocityController::setTargetVelocity(const std::array<double, 6>& velocity) {
  target_velocity_ = velocity;
  clampVelocity(target_velocity_);
  stopping_ = false;
  elapsed_time_ = 0.0;
}

void VelocityController::stop() {
  target_velocity_ = {0, 0, 0, 0, 0, 0};
  stopping_ = true;
}

franka::CartesianVelocities VelocityController::step(const franka::RobotState& robot_state,
                                                     franka::Duration period) {
  double dt = period.toSec();
  elapsed_time_ += dt;

  // Apply smooth velocity ramping
  applyVelocityRamping(dt);

  // Create velocity command
  franka::CartesianVelocities output = {{
    current_velocity_[0],  // vx
    current_velocity_[1],  // vy
    current_velocity_[2],  // vz
    current_velocity_[3],  // wx
    current_velocity_[4],  // wy
    current_velocity_[5]   // wz
  }};

  return output;
}

void VelocityController::applyVelocityRamping(double dt) {
  // Calculate ramp factor based on elapsed time
  double ramp_factor = std::min(1.0, elapsed_time_ / ramp_time_);
  
  // Smooth acceleration using a sine curve
  ramp_factor = std::sin(ramp_factor * M_PI / 2.0);

  // Interpolate from current to target velocity
  for (size_t i = 0; i < 6; i++) {
    double velocity_diff = target_velocity_[i] - current_velocity_[i];
    
    // Limit acceleration
    double max_velocity_change = (i < 3 ? max_linear_velocity_ : max_angular_velocity_) / ramp_time_ * dt;
    
    if (std::abs(velocity_diff) < max_velocity_change) {
      current_velocity_[i] = target_velocity_[i];
    } else {
      current_velocity_[i] += std::copysign(max_velocity_change, velocity_diff);
    }
  }
}

void VelocityController::clampVelocity(std::array<double, 6>& velocity) const {
  // Clamp linear velocities
  for (size_t i = 0; i < 3; i++) {
    velocity[i] = std::clamp(velocity[i], -max_linear_velocity_, max_linear_velocity_);
  }
  
  // Clamp angular velocities
  for (size_t i = 3; i < 6; i++) {
    velocity[i] = std::clamp(velocity[i], -max_angular_velocity_, max_angular_velocity_);
  }
}

bool VelocityController::isStopped() const {
  return isNearZero(current_velocity_);
}

bool VelocityController::isNearZero(const std::array<double, 6>& velocity, double threshold) const {
  for (double v : velocity) {
    if (std::abs(v) > threshold) {
      return false;
    }
  }
  return true;
}

void VelocityController::setVelocityLimits(double max_linear, double max_angular) {
  max_linear_velocity_ = max_linear;
  max_angular_velocity_ = max_angular;
}

// TimedVelocityController implementation

TimedVelocityController::TimedVelocityController(double max_linear_velocity,
                                                 double max_angular_velocity)
    : velocity_controller_(max_linear_velocity, max_angular_velocity) {}

void TimedVelocityController::setVelocity(const std::array<double, 6>& velocity, double duration) {
  velocity_controller_.setTargetVelocity(velocity);
  duration_ = duration;
  elapsed_time_ = 0.0;
  has_duration_ = (duration > 0.0);
}

franka::CartesianVelocities TimedVelocityController::step(const franka::RobotState& robot_state,
                                                          franka::Duration period) {
  elapsed_time_ += period.toSec();

  // Check if timed motion should stop
  if (has_duration_ && elapsed_time_ >= duration_) {
    velocity_controller_.stop();
    has_duration_ = false;
  }

  return velocity_controller_.step(robot_state, period);
}

void TimedVelocityController::stop() {
  velocity_controller_.stop();
  has_duration_ = false;
  elapsed_time_ = 0.0;
}

bool TimedVelocityController::isFinished() const {
  if (!has_duration_) {
    return velocity_controller_.isStopped();
  }
  return elapsed_time_ >= duration_ && velocity_controller_.isStopped();
}

}  // namespace franka_user_control
