cmake_minimum_required(VERSION 3.8)
project(humanoid_motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find pigpio library
find_library(PIGPIO_LIBRARY pigpio REQUIRED)


# Cytron MDDRC10 Motor Controller Node
add_executable(cytron_controller_node 
  src/cytron_controller_node.cpp
  src/cytron_controller.cpp
)
ament_target_dependencies(cytron_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_include_directories(cytron_controller_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(cytron_controller_node ${PIGPIO_LIBRARY})

# Encoder Reader Node
add_executable(encoder_node 
  src/encoder_node.cpp
)
ament_target_dependencies(encoder_node
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(encoder_node ${PIGPIO_LIBRARY})

# Differential Drive Controller
add_executable(diff_drive_controller 
  src/diff_drive_controller.cpp
)
ament_target_dependencies(diff_drive_controller
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  cytron_controller_node
  encoder_node
  diff_drive_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()