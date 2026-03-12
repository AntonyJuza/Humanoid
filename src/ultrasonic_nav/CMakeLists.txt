cmake_minimum_required(VERSION 3.8)
project(ultrasonic_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ultrasonic_sensor_node src/ultrasonic_sensor_node.cpp)
target_include_directories(ultrasonic_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(ultrasonic_sensor_node PUBLIC c_std_99 cxx_std_17)  
target_link_libraries(ultrasonic_sensor_node pigpiod_if2 rt)
ament_target_dependencies(ultrasonic_sensor_node "rclcpp" "std_msgs")

add_executable(navigation_node src/navigation_node.cpp)
target_include_directories(navigation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(navigation_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(navigation_node "rclcpp" "std_msgs" "geometry_msgs")

install(TARGETS 
  ultrasonic_sensor_node
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
