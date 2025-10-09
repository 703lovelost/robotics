cmake_minimum_required(VERSION 3.8)
project(service_full_name)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SummFullName.msg"
  "srv/SummFullName.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(service_name src/full_name_server.cpp)
ament_target_dependencies(service_name rclcpp)
rosidl_target_interfaces(service_name ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(client_name src/full_name_client.cpp)
ament_target_dependencies(client_name rclcpp)
rosidl_target_interfaces(client_name ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  service_name
  client_name
  DESTINATION lib/${PROJECT_NAME})

ament_package()
