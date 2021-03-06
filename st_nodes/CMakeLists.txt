cmake_minimum_required(VERSION 3.5)
project(st_nodes)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(joy_distributer src/joy_distributer.cpp)
ament_target_dependencies(joy_distributer rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(move_navigator src/move_navigator.cpp)
ament_target_dependencies(move_navigator rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

add_executable(arm_navigator src/arm_navigator.cpp)
ament_target_dependencies(arm_navigator rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(elevator_navigator src/elevator_navigator.cpp)
ament_target_dependencies(elevator_navigator rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  joy_distributer move_navigator arm_navigator elevator_navigator
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
