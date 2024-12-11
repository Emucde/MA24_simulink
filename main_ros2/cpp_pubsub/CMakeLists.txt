cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)  # Add this line to find sensor_msgs

# Include directories if necessary (usually not needed for ROS 2)
# include_directories(include)

# Define the executable
add_executable(franka_joint_state_subscriber
  src/franka_joint_state_subscriber.cpp
  src/shared_memory.cpp
)

# Link dependencies
ament_target_dependencies(franka_joint_state_subscriber
  rclcpp
  sensor_msgs  # Add sensor_msgs here
)

install(
  TARGETS franka_joint_state_subscriber  # Install the executable
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch  # Keep only if you have launch files
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
set(CPACK_PACKAGE_VENDOR "Franka Robotics GmbH")
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/ros/humble")
set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_PACKAGE_VERSION 0.1.13)
set(CPACK_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_PROCESSOR})

# Debian versions require a dash
set(CPACK_DEBIAN_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION}-1)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Franka Robotics GmbH")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libfranka")
