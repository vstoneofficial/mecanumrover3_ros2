cmake_minimum_required(VERSION 3.8)
project(mecanumrover3_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# pub_odom
add_executable(pub_odom src/pub_odom.cpp)
ament_target_dependencies(
  pub_odom
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  tf2_ros
  tf2
)

# rover_gamepad
add_executable(rover_gamepad src/rover_gamepad.cpp)
ament_target_dependencies(
  rover_gamepad
  rclcpp
  geometry_msgs
  sensor_msgs
)

# rover_gamepad_tty
add_executable(rover_gamepad_tty src/rover_gamepad.cpp)
ament_target_dependencies(
  rover_gamepad_tty
  rclcpp
  geometry_msgs
  sensor_msgs
)

# gazebo_odom_bridge
add_executable(gazebo_odom_bridge src/gazebo_odom_bridge.cpp)
ament_target_dependencies(
  gazebo_odom_bridge
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  gazebo_msgs
)

install(TARGETS
  pub_odom
  rover_gamepad
  rover_gamepad_tty
  gazebo_odom_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

