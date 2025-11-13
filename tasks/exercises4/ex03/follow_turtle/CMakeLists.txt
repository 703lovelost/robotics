cmake_minimum_required(VERSION 3.8)
project(follow_turtle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(turtle_tf2_broadcaster
  rclcpp turtlesim tf2 tf2_ros tf2_geometry_msgs geometry_msgs)

add_executable(turtle_time_travel_listener src/turtle_time_travel_listener.cpp)
ament_target_dependencies(turtle_time_travel_listener
  rclcpp tf2 tf2_ros geometry_msgs)

install(TARGETS
  turtle_tf2_broadcaster
  turtle_time_travel_listener
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/spawn_turtle.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
