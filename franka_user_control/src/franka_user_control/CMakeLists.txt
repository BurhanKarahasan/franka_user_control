cmake_minimum_required(VERSION 3.8)
project(franka_user_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find external libraries
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetCartesianVelocity.srv"
  "srv/MoveToPose.srv"
  "srv/MoveRelative.srv"
  "srv/SetControlMode.srv"
  "srv/EmergencyStop.srv"
  "msg/RobotStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/ControllerState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# C++ Library: Core control logic
add_library(franka_control_lib
  src/motion_generator.cpp
  src/velocity_controller.cpp
  src/impedance_controller.cpp
  src/safety_monitor.cpp
)

target_link_libraries(franka_control_lib
  Franka::Franka
  Eigen3::Eigen
)

ament_target_dependencies(franka_control_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# C++ Executable: Main bridge node
add_executable(franka_bridge_node
  src/main.cpp
  src/franka_bridge_node.cpp
)

target_link_libraries(franka_bridge_node
  franka_control_lib
  Franka::Franka
  Eigen3::Eigen
)

ament_target_dependencies(franka_bridge_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Link against generated messages/services
rosidl_target_interfaces(franka_bridge_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install C++ executables
install(TARGETS
  franka_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ library
install(TARGETS
  franka_control_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install Python package
ament_python_install_package(franka_python_interface)

# Install Python scripts (make them executable)
install(PROGRAMS
  scripts/terminal_ui.py
  scripts/test_velocity.py
  scripts/test_position.py
  scripts/test_combined.py
  scripts/calibration.py
  scripts/fake_robot_state_publisher.py
  scripts/simulation_bridge_node.py
  scripts/test_simulation.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install documentation
install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  # Linting
  ament_lint_auto_find_test_dependencies()
  
  # C++ tests
  ament_add_gtest(test_motion_generator tests/test_motion_generator.cpp)
  target_link_libraries(test_motion_generator franka_control_lib)
  
  ament_add_gtest(test_safety tests/test_safety.cpp)
  target_link_libraries(test_safety franka_control_lib)
  
  # Python tests
  ament_add_pytest_test(test_interface tests/test_interface.py)
endif()

ament_package()