// Copyright (c) 2024 Franka User Control
// Licensed under the Apache License, Version 2.0

#include <memory>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include "franka_user_control/franka_bridge_node.hpp"

// Global pointer for signal handler
std::shared_ptr<franka_user_control::FrankaBridgeNode> g_node;

void signalHandler(int signum) {
  (void)signum;  // Unused parameter
  
  if (g_node) {
    RCLCPP_WARN(g_node->get_logger(), "Interrupt signal received, shutting down...");
    g_node->stopControlLoop();
    rclcpp::shutdown();
  }
}

int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Set up signal handler for graceful shutdown
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  
  try {
    // Create node
    auto options = rclcpp::NodeOptions();
    g_node = std::make_shared<franka_user_control::FrankaBridgeNode>(options);
    
    // Initialize robot connection
    if (!g_node->initialize()) {
      RCLCPP_ERROR(g_node->get_logger(), "Failed to initialize robot connection");
      return 1;
    }
    
    RCLCPP_INFO(g_node->get_logger(), "Franka Bridge Node is ready");
    RCLCPP_INFO(g_node->get_logger(), "Waiting for service calls...");
    
    // Spin the node (handle callbacks)
    rclcpp::spin(g_node);
    
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
    return 1;
  }
  
  // Clean shutdown
  RCLCPP_INFO(rclcpp::get_logger("main"), "Franka Bridge Node shutting down");
  g_node.reset();
  rclcpp::shutdown();
  
  return 0;
}
