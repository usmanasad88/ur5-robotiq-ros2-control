cmake_minimum_required(VERSION 3.8)
project(ur5_robotiq_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY 
  urdf 
  meshes 
  launch
  config
  rviz
  srdf
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".git" EXCLUDE
)

# Install Python scripts
install(PROGRAMS
  scripts/gripper_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
