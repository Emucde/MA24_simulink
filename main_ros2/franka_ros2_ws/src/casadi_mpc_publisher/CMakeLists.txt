cmake_minimum_required(VERSION 3.8)
project(casadi_mpc_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mpc_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(casadi_mpc_publisher src/casadi_mpc_publisher.cpp)

target_include_directories(
        casadi_mpc_publisher
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(
  casadi_mpc_publisher
  rclcpp
  std_msgs
  mpc_interfaces
)

install(TARGETS
  casadi_mpc_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
