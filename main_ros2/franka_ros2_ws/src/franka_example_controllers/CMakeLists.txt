cmake_minimum_required(VERSION 3.5)
project(franka_example_controllers)

set(OPT_FLAG "-Ofast;-march=x86-64;-mtune=generic;")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(generate_parameter_library)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(mpc_interfaces REQUIRED)
find_package(pinocchio REQUIRED)

set(MPC_SOURCE_DIR_REL ${CMAKE_CURRENT_SOURCE_DIR}/../../../../s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles)
cmake_path(SET MPC_SOURCE_DIR NORMALIZE "${MPC_SOURCE_DIR_REL}")

set(MPC_LIB_DIR_REL ${CMAKE_CURRENT_SOURCE_DIR}/../../../../s_functions/fr3_no_hand_6dof/mpc_libraries)
cmake_path(SET MPC_LIB_DIR NORMALIZE "${MPC_LIB_DIR_REL}")

set(MPC_UTILS_DIR_REL ${CMAKE_CURRENT_SOURCE_DIR}/../../../casadi_mpc/cpp_class_files)
cmake_path(SET MPC_UTILS_DIR NORMALIZE "${MPC_UTILS_DIR_REL}")

set(MPC_INCLUDE_DIR ${MPC_SOURCE_DIR})
set(MPC_LIBRARY_DIR ${MPC_LIB_DIR})
set(UTILS_SOURCE_DIR ${MPC_UTILS_DIR}/src)
set(UTILS_INCLUDE_DIR ${MPC_UTILS_DIR}/include)

message(STATUS "MPC source directory: ${MPC_SOURCE_DIR}")
message(STATUS "MPC include directory: ${MPC_INCLUDE_DIR}")
message(STATUS "MPC utils source directory: ${UTILS_SOURCE_DIR}")
message(STATUS "MPC utils include directory: ${UTILS_INCLUDE_DIR}")

# Find all MPC*.c files
file(GLOB MPC_SOURCES "${MPC_SOURCE_DIR}/MPC*.c" "${MPC_SOURCE_DIR}/param_robot.c" "${UTILS_SOURCE_DIR}/*.cpp" "${MPC_SOURCE_DIR}/fr3_ekf.c")
set(MPC_ALL_LIBRARIES "")
set(MPC_PARAM_LIBRARIES "")
set(MPC_LIBRARIES "")
set(MPC_UTILS_LIBRARIES "")

# Search shared libraries
foreach(SOURCE_FILE ${MPC_SOURCES})
        get_filename_component(FILE_NAME ${SOURCE_FILE} NAME_WE)  # Get the filename without extension

        # Collect the libraries for installation
        list(APPEND MPC_ALL_LIBRARIES ${FILE_NAME})

        if(SOURCE_FILE MATCHES ${MPC_SOURCE_DIR})
                if(FILE_NAME MATCHES "_param$")
                        list(APPEND MPC_PARAM_LIBRARIES ${FILE_NAME})
                elseif(FILE_NAME MATCHES "MPC")
                        list(APPEND MPC_LIBRARIES ${FILE_NAME})
                endif()
        else()
                list(APPEND MPC_UTILS_LIBRARIES ${FILE_NAME})
        endif()

endforeach()

# Check if the libraries were found
if(NOT DEFINED MPC_PARAM_LIBRARIES OR MPC_PARAM_LIBRARIES STREQUAL "")
        message(WARNING "MPC_PARAM_LIBRARIES on path \"${MPC_SOURCE_DIR}\" is empty or not defined")
else()
        message(STATUS "MPC param libraries: ${MPC_PARAM_LIBRARIES}")
endif()

if(NOT DEFINED MPC_LIBRARIES OR MPC_LIBRARIES STREQUAL "")
        message(WARNING "MPC_LIBRARIES on path \"${MPC_SOURCE_DIR}\" is empty or not defined")
else()
        message(STATUS "MPC libraries: ${MPC_LIBRARIES}")
endif()

if(NOT DEFINED MPC_UTILS_LIBRARIES OR MPC_UTILS_LIBRARIES STREQUAL "")
        message(WARNING "MPC_UTILS_LIBRARIES on path \"${UTILS_SOURCE_DIR}\" is empty or not defined")
else()
        message(STATUS "MPC utils libraries: ${MPC_UTILS_LIBRARIES}")
endif()

# list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
list(APPEND CMAKE_PREFIX_PATH "${MPC_SOURCE_DIR}")
link_directories("${MPC_SOURCE_DIR}")

add_library(
        ${PROJECT_NAME}
        SHARED
        src/gravity_compensation_example_controller.cpp
        src/conventional_workspace_controller.cpp
        src/mpc_crocoddyl_controller.cpp
        src/mpc_crocoddyl_controller_shm.cpp
        src/mpc_casadi_controller.cpp
        src/shared_memory.cpp
        src/move_to_start_example_controller.cpp
        src/motion_generator.cpp
)

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${MPC_INCLUDE_DIR}
        ${UTILS_INCLUDE_DIR}
)

add_definitions(-DMASTERDIR="$ENV{masterdir}")

ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        franka_semantic_components
        moveit_core
        moveit_msgs
        mpc_interfaces
)

generate_parameter_library(franka_example_controllers_parameters src/model_example_controller_parameters.yaml)

find_library(CASADI_CONTROLLER_LIBRARY
    NAMES CasadiController
    HINTS "${MPC_LIB_DIR_REL}"
)

find_library(CASADI_MPC_LIBRARY
    NAMES CasadiMPC
    HINTS "${MPC_LIB_DIR_REL}"
)

find_library(SHARED_MEMORY_LIBRARY
    NAMES SharedMemory
    HINTS "${MPC_LIB_DIR_REL}"
)

find_library(CASADI_EKF_LIBRARY
    NAMES CasadiEKF
    HINTS "${MPC_LIB_DIR_REL}"
)

find_library(SIGNAL_FILTER_LIBRARY
    NAMES SignalFilter
    HINTS "${MPC_LIB_DIR_REL}"
)

find_library(FULL_SYSTEM_TORQUE_MAPPER_LIB
    NAMES FullSystemTorqueMapper
    HINTS "${MPC_LIB_DIR_REL}"
)

find_library(CROCODDYLCONTROLLER_LIB
    NAMES CrocoddylController
    HINTS "${MPC_LIB_DIR_REL}"
)

find_library(WORKSPACECONTROLLER_LIB
    NAMES WorkspaceController
    HINTS "${MPC_LIB_DIR_REL}"
)

find_library(COMMONBASECONTROLLER_LIB
    NAMES CommonBaseController
    HINTS "${MPC_LIB_DIR_REL}"
)

find_library(COMMONBASEMPC_LIB
    NAMES CommonBaseMPC
    HINTS "${MPC_LIB_DIR_REL}"
)


target_link_libraries(
        ${PROJECT_NAME}
        franka_example_controllers_parameters
        ${MPC_ALL_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PRIVATE -fdiagnostics-color=always ${MPC_OPT_FLAG})

pluginlib_export_plugin_description_file(
        controller_interface franka_example_controllers.xml
)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${MPC_SOURCE_DIR}"
)

if(BUILD_TESTING)
        find_package(ament_cmake_clang_format REQUIRED)
        find_package(ament_cmake_copyright REQUIRED)
        find_package(ament_cmake_cppcheck REQUIRED)
        find_package(ament_cmake_flake8 REQUIRED)
        find_package(ament_cmake_lint_cmake REQUIRED)
        find_package(ament_cmake_pep257 REQUIRED)
        find_package(ament_cmake_xmllint REQUIRED)
        find_package(ament_cmake_gmock REQUIRED)
        find_package(controller_manager REQUIRED)
        find_package(ros2_control_test_assets REQUIRED)

        set(CPP_DIRECTORIES src include)
        ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
        ament_copyright(src ${CPP_DIRECTORIES} package.xml)
        ament_cppcheck(${CPP_DIRECTORIES})
        ament_lint_cmake(CMakeLists.txt)
        ament_flake8()
        ament_pep257()
        ament_xmllint()
        ament_add_gmock(${PROJECT_NAME}_test
                test/test_load_gravity_compensation_controller.cpp
        )
        target_include_directories(${PROJECT_NAME}_test PRIVATE include)
        ament_target_dependencies(${PROJECT_NAME}_test
                controller_manager
                ros2_control_test_assets
        )

        ament_add_gmock(${PROJECT_NAME}_test_load_move_to_start
                test/test_load_move_to_start_example_controller.cpp)
        target_include_directories(${PROJECT_NAME}_test_load_move_to_start PRIVATE include)
        ament_target_dependencies(${PROJECT_NAME}_test_load_move_to_start
                controller_manager
                ros2_control_test_assets
        )


        ament_add_gmock(${PROJECT_NAME}_gravity_test test/test_gravity_compensation_example.cpp)
        target_include_directories(${PROJECT_NAME}_gravity_test PRIVATE include)
        target_link_libraries(${PROJECT_NAME}_gravity_test ${PROJECT_NAME})

        ament_add_gmock(${PROJECT_NAME}_move_to_start_test test/test_move_to_start_example_controller.cpp)
        target_include_directories(${PROJECT_NAME}_move_to_start_test PRIVATE include)
        target_link_libraries(${PROJECT_NAME}_move_to_start_test ${PROJECT_NAME})

        if(CHECK_TIDY)
                find_package(ament_cmake_clang_tidy REQUIRED)
                set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
                ament_clang_tidy(${CMAKE_BINARY_DIR})
        endif()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        moveit_core
)
ament_package()