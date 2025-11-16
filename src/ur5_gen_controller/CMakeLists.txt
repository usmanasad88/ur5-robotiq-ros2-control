cmake_minimum_required(VERSION 3.8)
project(ur5_gen_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ur5_teleop_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(CURL REQUIRED)

# Add executable
add_executable(random_joint_goal src/random_joint_goal.cpp)

# Include directories
target_include_directories(random_joint_goal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CURL_INCLUDE_DIRS})

# Link dependencies
ament_target_dependencies(random_joint_goal
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  ur5_teleop_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(random_joint_goal
  ${CURL_LIBRARIES}
)

# Install executable
install(TARGETS random_joint_goal
  DESTINATION lib/${PROJECT_NAME})

ament_package()
