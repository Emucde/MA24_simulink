cmake_minimum_required(VERSION 3.8)
project(command_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mpc_interfaces REQUIRED)

add_executable(command_publisher src/command_publisher.cpp)
ament_target_dependencies(
  command_publisher
  rclcpp
  std_msgs
  mpc_interfaces
)

install(TARGETS
  command_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
