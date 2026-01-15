cmake_minimum_required(VERSION 3.8)
project(planning_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Global planner executable
add_executable(global_planner src/global_planner.cpp)
ament_target_dependencies(global_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(global_planner Eigen3::Eigen)

# Local planner executable
add_executable(local_planner src/local_planner.cpp)
ament_target_dependencies(local_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(local_planner Eigen3::Eigen)

# Goal manager executable
add_executable(goal_manager src/goal_manager.cpp)
ament_target_dependencies(goal_manager
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

# Install executables
install(TARGETS
  global_planner
  local_planner
  goal_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
