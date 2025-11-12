cmake_minimum_required(VERSION 3.8)
project(follow_carrot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(turtle_tf2_broadcaster
  rclcpp geometry_msgs turtlesim tf2 tf2_ros tf2_geometry_msgs)

add_executable(carrot_broadcaster src/carrot_broadcaster.cpp)
ament_target_dependencies(carrot_broadcaster
  rclcpp geometry_msgs tf2 tf2_ros)

add_executable(turtle_follower src/turtle_follower.cpp)
ament_target_dependencies(turtle_follower
  rclcpp geometry_msgs tf2 tf2_ros)

add_executable(spawn_turtle src/spawn_turtle.cpp)
ament_target_dependencies(spawn_turtle
  rclcpp turtlesim)

install(TARGETS
  turtle_tf2_broadcaster
  carrot_broadcaster
  turtle_follower
  spawn_turtle
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
