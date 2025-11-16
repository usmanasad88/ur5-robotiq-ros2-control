cmake_minimum_required(VERSION 3.8)
project(ur5_keyboard_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ur5_teleop_msgs REQUIRED)

# Add executable
add_executable(keyboard_teleop_node src/keyboard_teleop_node.cpp)

# Link dependencies
ament_target_dependencies(keyboard_teleop_node
  rclcpp
  ur5_teleop_msgs
)

# Install executable
install(TARGETS keyboard_teleop_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
