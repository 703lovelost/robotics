cmake_minimum_required(VERSION 3.16)
project(ros3)

set(ros3_MAJOR_VERSION 0)
set(ros3_MINOR_VERSION 0)
set(ros3_PATCH_VERSION 0)
set(ros3_VERSION
  ${ros3_MAJOR_VERSION}.${ros3_MINOR_VERSION}.${ros3_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
