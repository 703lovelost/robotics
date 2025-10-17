cmake_minimum_required(VERSION 3.8)
project(move_to_goal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(move_to_goal src/move_to_goal.cpp)
ament_target_dependencies(move_to_goal rclcpp geometry_msgs turtlesim)
install(TARGETS
  move_to_goal
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
