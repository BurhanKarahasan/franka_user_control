// Copyright (c) 2024 Franka User Control
// Licensed under the Apache License, Version 2.0

#include "franka_user_control/bridge_node.hpp"
#include <chrono>

namespace franka_user_control {

FrankaBridgeNode::FrankaBridgeNode(const rclcpp::NodeOptions& options)
    : Node("franka_bridge_node", options) {
  
  RCLCPP_INFO(get_logger(), "Initializing Franka Bridge Node...");
  
  // Load parameters
  loadParameters();
  
  // Initialize safety monitor
  safety_monitor_ = std::make_unique<SafetyMonitor>(safety_limits_);
  
  // Create services (placeholder - will be properly typed after build)
  // Note: In actual implementation, these would use the generated service types
  
  RCLCPP_INFO(get_logger(), "Creating ROS2 services...");
  
  // Typed services
  srv_set_velocity_ = create_service<franka_user_control::srv::SetCartesianVelocity>(
      "set_cartesian_velocity",
      [this](const std::shared_ptr<franka_user_control::srv::SetCartesianVelocity::Request> req,
             std::shared_ptr<franka_user_control::srv::SetCartesianVelocity::Response> res) {
        RCLCPP_INFO(this->get_logger(), "SetCartesianVelocity called");
        std::array<double, 6> velocity = {req->vx, req->vy, req->vz, req->wx, req->wy, req->wz};
        auto violation = this->safety_monitor_->checkVelocityCommand(velocity);
        if (violation.hasViolation()) {
          res->success = false;
          res->message = "Velocity command violates safety limits: " + violation.description;
          return;
        }
        this->velocity_controller_->setTargetVelocity(velocity);
        this->current_mode_ = ControlMode::VELOCITY;
        res->success = true;
        res->message = "Velocity command accepted";
      });

  srv_move_to_pose_ = create_service<franka_user_control::srv::MoveToPose>(
      "move_to_pose",
      [this](const std::shared_ptr<franka_user_control::srv::MoveToPose::Request> req,
             std::shared_ptr<franka_user_control::srv::MoveToPose::Response> res) {
        RCLCPP_INFO(this->get_logger(), "MoveToPose called");
        auto joint_angles = this->poseToJoints(req->target_pose);
        double speed_factor = std::max(0.0, std::min(1.0, req->max_velocity / this->safety_limits_.max_translation_velocity));
        this->motion_generator_ = std::make_unique<MotionGenerator>(speed_factor, joint_angles);
        this->current_mode_ = ControlMode::POSITION;
        res->success = true;
        res->message = "Position motion started";
        res->estimated_duration = this->motion_generator_->getEstimatedDuration();
      });

  srv_move_relative_ = create_service<franka_user_control::srv::MoveRelative>(
      "move_relative",
      [this](const std::shared_ptr<franka_user_control::srv::MoveRelative::Request> req,
             std::shared_ptr<franka_user_control::srv::MoveRelative::Response> res) {
        RCLCPP_INFO(this->get_logger(), "MoveRelative called");
        auto current_pose = this->getCurrentPose();
        geometry_msgs::msg::Pose target = current_pose;
        target.position.x += req->dx;
        target.position.y += req->dy;
        target.position.z += req->dz;
        // Note: orientation deltas ignored for now
        auto joint_angles = this->poseToJoints(target);
        double speed_factor = std::max(0.0, std::min(1.0, req->max_velocity / this->safety_limits_.max_translation_velocity));
        this->motion_generator_ = std::make_unique<MotionGenerator>(speed_factor, joint_angles);
        this->current_mode_ = ControlMode::POSITION;
        res->success = true;
        res->message = "Relative position motion started";
        res->estimated_duration = this->motion_generator_->getEstimatedDuration();
      });

  srv_set_control_mode_ = create_service<franka_user_control::srv::SetControlMode>(
      "set_control_mode",
      [this](const std::shared_ptr<franka_user_control::srv::SetControlMode::Request> req,
             std::shared_ptr<franka_user_control::srv::SetControlMode::Response> res) {
        RCLCPP_INFO(this->get_logger(), "SetControlMode called: %s", req->mode.c_str());
        if (req->mode == "velocity") {
          this->current_mode_ = ControlMode::VELOCITY;
        } else if (req->mode == "position") {
          this->current_mode_ = ControlMode::POSITION;
        } else {
          this->current_mode_ = ControlMode::IDLE;
        }
        res->success = true;
        res->current_mode = (this->current_mode_ == ControlMode::VELOCITY) ? "velocity" : (this->current_mode_ == ControlMode::POSITION) ? "position" : "idle";
        res->message = "Mode set";
      });

  srv_emergency_stop_ = create_service<franka_user_control::srv::EmergencyStop>(
      "emergency_stop",
      [this](const std::shared_ptr<franka_user_control::srv::EmergencyStop::Request> req,
             std::shared_ptr<franka_user_control::srv::EmergencyStop::Response> res) {
        RCLCPP_WARN(this->get_logger(), "EmergencyStop called: %s", req->stop ? "STOP" : "RESET");
        if (req->stop) {
          this->emergency_stop_active_ = true;
          this->current_mode_ = ControlMode::IDLE;
          if (this->robot_) {
            try { this->robot_->stop(); } catch (const franka::Exception& e) { RCLCPP_ERROR(this->get_logger(), "Error stopping robot: %s", e.what()); }
          }
          res->success = true;
          res->message = "Emergency stop activated";
        } else {
          this->emergency_stop_active_ = false;
          res->success = true;
          res->message = "Emergency stop reset";
        }
      });

  // Publishers for status information
  pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);
  
  // Status publishing timer (10 Hz)
  status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FrankaBridgeNode::statusTimerCallback, this));
  
  RCLCPP_INFO(get_logger(), "Franka Bridge Node initialized");
}

FrankaBridgeNode::~FrankaBridgeNode() {
  RCLCPP_INFO(get_logger(), "Shutting down Franka Bridge Node...");
  stopControlLoop();
  
  if (robot_) {
    try {
      robot_->stop();
    } catch (const franka::Exception& e) {
      RCLCPP_ERROR(get_logger(), "Error stopping robot: %s", e.what());
    }
  }
}

bool FrankaBridgeNode::initialize() {
  RCLCPP_INFO(get_logger(), "Connecting to robot at %s...", robot_ip_.c_str());
  
  try {
    robot_ = std::make_unique<franka::Robot>(robot_ip_);
    robot_connected_ = true;
    
    RCLCPP_INFO(get_logger(), "Successfully connected to robot");
    
    // Set default behavior
    setDefaultBehavior();
    
    // Initialize controllers
    velocity_controller_ = std::make_unique<VelocityController>(
        safety_limits_.max_translation_velocity,
        safety_limits_.max_rotation_velocity);
    
    impedance_controller_ = std::make_unique<ImpedanceController>(
        5,  // filter size
        std::array<double, 7>{{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}},
        std::array<double, 7>{{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}}
    );
    
    return true;
    
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to robot: %s", e.what());
    robot_connected_ = false;
    return false;
  }
}

void FrankaBridgeNode::loadParameters() {
  // Declare and get parameters
  declare_parameter("robot_ip", "192.168.1.100");
  robot_ip_ = get_parameter("robot_ip").as_string();
  
  // Safety limits
  declare_parameter("max_translation_velocity", 0.2);
  declare_parameter("max_rotation_velocity", 1.0);
  declare_parameter("max_translation_acceleration", 1.0);
  declare_parameter("max_rotation_acceleration", 5.0);
  
  safety_limits_.max_translation_velocity = 
      get_parameter("max_translation_velocity").as_double();
  safety_limits_.max_rotation_velocity = 
      get_parameter("max_rotation_velocity").as_double();
  safety_limits_.max_translation_acceleration = 
      get_parameter("max_translation_acceleration").as_double();
  safety_limits_.max_rotation_acceleration = 
      get_parameter("max_rotation_acceleration").as_double();
  
  // Workspace limits
  declare_parameter("workspace.x_min", 0.1);
  declare_parameter("workspace.x_max", 0.85);
  declare_parameter("workspace.y_min", -0.5);
  declare_parameter("workspace.y_max", 0.5);
  declare_parameter("workspace.z_min", 0.0);
  declare_parameter("workspace.z_max", 0.8);
  
  safety_limits_.workspace.x_min = get_parameter("workspace.x_min").as_double();
  safety_limits_.workspace.x_max = get_parameter("workspace.x_max").as_double();
  safety_limits_.workspace.y_min = get_parameter("workspace.y_min").as_double();
  safety_limits_.workspace.y_max = get_parameter("workspace.y_max").as_double();
  safety_limits_.workspace.z_min = get_parameter("workspace.z_min").as_double();
  safety_limits_.workspace.z_max = get_parameter("workspace.z_max").as_double();
  
  // Force thresholds
  declare_parameter("force_thresholds", std::vector<double>{30.0, 30.0, 30.0, 25.0, 25.0, 25.0});
  auto force_vec = get_parameter("force_thresholds").as_double_array();
  std::copy(force_vec.begin(), force_vec.end(), safety_limits_.force_thresholds.begin());
  
  RCLCPP_INFO(get_logger(), "Loaded parameters:");
  RCLCPP_INFO(get_logger(), "  Robot IP: %s", robot_ip_.c_str());
  RCLCPP_INFO(get_logger(), "  Max translation velocity: %.3f m/s", 
              safety_limits_.max_translation_velocity);
  RCLCPP_INFO(get_logger(), "  Max rotation velocity: %.3f rad/s", 
              safety_limits_.max_rotation_velocity);
}

void FrankaBridgeNode::setDefaultBehavior() {
  if (!robot_) return;
  
  RCLCPP_INFO(get_logger(), "Setting default robot behavior...");
  
  try {
    // Set collision behavior (from examples_common.cpp)
    robot_->setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, 
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, 
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, 
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, 
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}
    );
    
    // Set joint impedance
    robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    
    // Set Cartesian impedance
    robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    
    RCLCPP_INFO(get_logger(), "Default behavior set successfully");
    
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to set default behavior: %s", e.what());
  }
}

void FrankaBridgeNode::statusTimerCallback() {
  if (!robot_connected_) return;
  
  publishStatus();
}

void FrankaBridgeNode::publishStatus() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Publish joint states
  auto joint_state_msg = sensor_msgs::msg::JointState();
  joint_state_msg.header.stamp = now();
  joint_state_msg.header.frame_id = "fr3_link0";
  
  joint_state_msg.name = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7"
  };
  
  joint_state_msg.position.assign(
      latest_robot_state_.q.begin(), 
      latest_robot_state_.q.end()
  );
  
  joint_state_msg.velocity.assign(
      latest_robot_state_.dq.begin(), 
      latest_robot_state_.dq.end()
  );
  
  pub_joint_states_->publish(joint_state_msg);
}

geometry_msgs::msg::Pose FrankaBridgeNode::getCurrentPose() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  geometry_msgs::msg::Pose pose;
  
  // Extract position from O_T_EE (4x4 transformation matrix, column-major)
  pose.position.x = latest_robot_state_.O_T_EE[12];
  pose.position.y = latest_robot_state_.O_T_EE[13];
  pose.position.z = latest_robot_state_.O_T_EE[14];
  
  // Extract rotation and convert to quaternion
  // For now, set identity quaternion (proper implementation would extract from matrix)
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  
  return pose;
}

void FrankaBridgeNode::startControlLoop() {
  if (control_loop_running_) {
    RCLCPP_WARN(get_logger(), "Control loop already running");
    return;
  }
  
  if (!robot_connected_) {
    RCLCPP_ERROR(get_logger(), "Cannot start control loop: robot not connected");
    return;
  }
  
  control_loop_running_ = true;
  control_thread_ = std::thread(&FrankaBridgeNode::controlLoop, this);
  
  RCLCPP_INFO(get_logger(), "Control loop started");
}

void FrankaBridgeNode::stopControlLoop() {
  if (!control_loop_running_) {
    return;
  }
  
  control_loop_running_ = false;
  
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
  
  RCLCPP_INFO(get_logger(), "Control loop stopped");
}

void FrankaBridgeNode::controlLoop() {
  RCLCPP_INFO(get_logger(), "Entering control loop...");
  
  try {
    while (control_loop_running_ && rclcpp::ok()) {
      // Check for emergency stop
      if (emergency_stop_active_) {
        RCLCPP_WARN(get_logger(), "Emergency stop active, waiting...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      
      // Execute control based on current mode
      switch (current_mode_) {
        case ControlMode::VELOCITY:
          executeVelocityControl();
          break;
          
        case ControlMode::POSITION:
          executePositionControl();
          break;
          
        case ControlMode::IDLE:
          // Just update state
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_robot_state_ = robot_->readOnce();
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          break;
      }
    }
    
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(get_logger(), "Control loop exception: %s", e.what());
    control_loop_running_ = false;
  }
  
  RCLCPP_INFO(get_logger(), "Control loop exited");
}

void FrankaBridgeNode::executeVelocityControl() {
  try {
    robot_->control(
        [this](const franka::RobotState& robot_state, franka::Duration period) 
            -> franka::CartesianVelocities {
          
          // Update latest state
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_robot_state_ = robot_state;
          }
          
          // Safety check
          auto violation = safety_monitor_->checkSafety(robot_state);
          if (violation.hasViolation()) {
            RCLCPP_ERROR(get_logger(), "Safety violation: %s", violation.description.c_str());
            emergency_stop_active_ = true;
            return franka::MotionFinished(franka::CartesianVelocities{{0, 0, 0, 0, 0, 0}});
          }
          
          // Generate velocity command
          auto velocities = velocity_controller_->step(robot_state, period);
          
          // Check if we should exit velocity mode
          if (!control_loop_running_ || current_mode_ != ControlMode::VELOCITY) {
            return franka::MotionFinished(velocities);
          }
          
          return velocities;
        });
        
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(get_logger(), "Velocity control error: %s", e.what());
    current_mode_ = ControlMode::IDLE;
  }
}

void FrankaBridgeNode::executePositionControl() {
  try {
    // Combined motion generation + impedance control
    robot_->control(
        [this](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {
          // Impedance control callback
          return impedance_controller_->step(robot_state);
        },
        [this](const franka::RobotState& robot_state, franka::Duration period) 
            -> franka::JointPositions {
          
          // Update latest state
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_robot_state_ = robot_state;
          }
          
          // Safety check
          auto violation = safety_monitor_->checkSafety(robot_state);
          if (violation.hasViolation()) {
            RCLCPP_ERROR(get_logger(), "Safety violation: %s", violation.description.c_str());
            emergency_stop_active_ = true;
            
            franka::JointPositions positions(robot_state.q_d);
            positions.motion_finished = true;
            return positions;
          }
          
          // Generate position command
          auto positions = motion_generator_->operator()(robot_state, period);
          
          franka::JointPositions output(positions);
          output.motion_finished = motion_generator_->isFinished();
          
          // Check if we should exit position mode
          if (!control_loop_running_ || current_mode_ != ControlMode::POSITION) {
            output.motion_finished = true;
          }
          
          if (output.motion_finished) {
            RCLCPP_INFO(get_logger(), "Position motion finished");
            current_mode_ = ControlMode::IDLE;
          }
          
          return output;
        });
        
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(get_logger(), "Position control error: %s", e.what());
    current_mode_ = ControlMode::IDLE;
  }
}

// Service handler implementations
// Note: These are simplified versions. After build, they would use proper service types

void FrankaBridgeNode::handleSetVelocity() {
  
  RCLCPP_INFO(get_logger(), "Set velocity service called");
  
  // In actual implementation, this would extract velocity from request
  // For now, this is a placeholder showing the pattern
  
  // Example usage pattern:
  // auto req = std::static_pointer_cast<franka_user_control::srv::SetCartesianVelocity::Request>(request);
  // std::array<double, 6> velocity = {req->vx, req->vy, req->vz, req->wx, req->wy, req->wz};
  
  // Check safety
  // auto violation = safety_monitor_->checkVelocityCommand(velocity);
  // if (violation.hasViolation()) {
  //   res->success = false;
  //   res->message = "Velocity command violates safety limits: " + violation.description;
  //   return;
  // }
  
  // Set velocity
  // velocity_controller_->setTargetVelocity(velocity);
  // current_mode_ = ControlMode::VELOCITY;
  
  // res->success = true;
  // res->message = "Velocity command accepted";
}

void FrankaBridgeNode::handleMoveToPose() {
  
  RCLCPP_INFO(get_logger(), "Move to pose service called");
  
  // In actual implementation:
  // auto req = std::static_pointer_cast<franka_user_control::srv::MoveToPose::Request>(request);
  
  // Convert Cartesian pose to joint angles (IK)
  // auto joint_angles = poseToJoints(req->target_pose);
  
  // Create motion generator with velocity limits
  // double speed_factor = req->max_velocity / safety_limits_.max_translation_velocity;
  // motion_generator_ = std::make_unique<MotionGenerator>(speed_factor, joint_angles);
  
  // Set impedance mode
  // if (req->impedance_mode == "stiff") {
  //   impedance_controller_ = std::make_unique<ImpedanceController>(
  //       ImpedanceControllerFactory::create(ImpedanceMode::STIFF));
  // }
  
  // current_mode_ = ControlMode::POSITION;
  
  // res->success = true;
  // res->estimated_duration = motion_generator_->getEstimatedDuration();
  // res->message = "Position motion started";
}

void FrankaBridgeNode::handleMoveRelative() {
  
  RCLCPP_INFO(get_logger(), "Move relative service called");
  
  // Get current pose
  // auto current_pose = getCurrentPose();
  
  // Calculate target pose
  // geometry_msgs::msg::Pose target_pose = current_pose;
  // target_pose.position.x += req->dx;
  // target_pose.position.y += req->dy;
  // target_pose.position.z += req->dz;
  
  // Then use same logic as MoveToPose
}

void FrankaBridgeNode::handleSetControlMode() {
  
  RCLCPP_INFO(get_logger(), "Set control mode service called");
  
  // auto req = std::static_pointer_cast<franka_user_control::srv::SetControlMode::Request>(request);
  
  // if (req->mode == "velocity") {
  //   current_mode_ = ControlMode::VELOCITY;
  // } else if (req->mode == "position") {
  //   current_mode_ = ControlMode::POSITION;
  // } else if (req->mode == "idle") {
  //   current_mode_ = ControlMode::IDLE;
  // }
  
  // res->success = true;
  // res->current_mode = req->mode;
}

void FrankaBridgeNode::handleEmergencyStop() {
  
  RCLCPP_WARN(get_logger(), "Emergency stop service called");
  
  // auto req = std::static_pointer_cast<franka_user_control::srv::EmergencyStop::Request>(request);
  
  // if (req->stop) {
  //   emergency_stop_active_ = true;
  //   current_mode_ = ControlMode::IDLE;
  //   
  //   if (robot_) {
  //     robot_->stop();
  //   }
  //   
  //   res->success = true;
  //   res->message = "Emergency stop activated";
  // } else {
  //   emergency_stop_active_ = false;
  //   res->success = true;
  //   res->message = "Emergency stop reset";
  // }
}

std::array<double, 7> FrankaBridgeNode::poseToJoints(const geometry_msgs::msg::Pose& pose) {
  // Placeholder for inverse kinematics
  // In a full implementation, this would use:
  // 1. Franka's model library for analytical IK
  // 2. Or a numerical IK solver like KDL or TracIK
  
  RCLCPP_WARN(get_logger(), "IK not yet implemented, returning current joint positions");
  
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::array<double, 7> joints;
  std::copy(latest_robot_state_.q.begin(), latest_robot_state_.q.end(), joints.begin());
  return joints;
}

}  // namespace franka_user_control
