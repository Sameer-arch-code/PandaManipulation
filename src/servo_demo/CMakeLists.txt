cmake_minimum_required(VERSION 3.8)
project(servo_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable
add_executable(servo_cpp_interface_demo src/servo_cpp_interface_demo.cpp)

# Specify dependencies
ament_target_dependencies(servo_cpp_interface_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_servo
  moveit_ros_planning
  control_msgs
  geometry_msgs
  moveit_msgs
  shape_msgs
  tf2_ros
)

# Install executable
install(TARGETS
  servo_cpp_interface_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files if they exist
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()