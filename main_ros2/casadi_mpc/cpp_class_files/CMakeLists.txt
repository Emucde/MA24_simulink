cmake_minimum_required(VERSION 3.10)
project(MPCLibraries)
# set(CMAKE_VERBOSE_MAKEFILE ON)

# Specify the build type; use 'Release' for optimization without debugging info
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)  # Default to Debug. Important: Then all optimization flags are forced to be -O0, other settings are ignored
endif()

# Add optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # -fdiagnostics-color=always: Always print diagnostics (warnings, errors) in color.
    # -O3: Optimize code for maximum speed and performance.
    # -ffast-math: Enable fast math optimizations that may violate strict IEEE or ANSI standards.
    # -march=native: Generate code optimized for the host machine's CPU.
    # -flto: Enable link-time optimization for better performance.
    # -Ofast: Optimize code for maximum speed and performance, disregarding strict standards compliance. (= -O3 -ffast-math + other optimizations)

    # set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE) # more modern method to enable LTO (-flto)
    # set(OPT_FLAG "-fdiagnostics-color=always;-O3;-ffast-math;-march=x86-64;-mtune=generic;-flto")
    # set(OPT_FLAG "-fdiagnostics-color=always;-O3;-ffast-math;-march=x86-64;-mtune=generic;")
    set(OPT_FLAG "-Ofast;-march=x86-64;-mtune=generic;-flto")
    set(MPC_OPT_FLAG ${OPT_FLAG})
elseif(CMAKE_BUILD_TYPE STREQUAL "DebugSpecial")
    set(OPT_FLAG "-O0")
    set(MPC_OPT_FLAG "-O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  # Ensure debug information is generated
endif()

# Enable IWYU only if the tool is available
# option(USE_IWYU "Run include-what-you-use" ON)

# if(USE_IWYU)
#     # Find the IWYU executable
#     find_program(IWYU_EXE NAMES include-what-you-use REQUIRED)

#     # Set IWYU for C++ targets
#     set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#     set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU_EXE})
# endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_WARN_DEPRECATED FALSE)

# Set output directory for libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always -pthread")
set(CMAKE_COLOR_DIAGNOSTICS ON)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)  # Change to 14/17/20 as necessary
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the path to the MPC source files
set(MPC_SOURCE_DIR_REL ${CMAKE_CURRENT_SOURCE_DIR}/../../../s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles)
cmake_path(SET MPC_SOURCE_DIR NORMALIZE "${MPC_SOURCE_DIR_REL}")
message(STATUS "MPC source directory: ${MPC_SOURCE_DIR}")

set(MPC_LIB_DIR_REL ${CMAKE_CURRENT_SOURCE_DIR}/../../../s_functions/fr3_no_hand_6dof/mpc_libraries)
cmake_path(SET MPC_LIB_DIR NORMALIZE "${MPC_LIB_DIR_REL}")

# Find all MPC*.c files
file(GLOB MPC_SOURCES "${MPC_SOURCE_DIR}/MPC*.c" "${MPC_SOURCE_DIR}/param_robot.c" "${MPC_SOURCE_DIR}/fr3_ekf.c")
set(MPC_ALL_LIBRARIES "")
set(MPC_PARAM_LIBRARIES "")
set(MPC_LIBRARIES "")

set(Boost_NO_BOOST_CMAKE ON)
set(CASADI_PATH $ENV{casadi_path})
set(CASADI_INCLUDE_DIRS "${CASADI_PATH}/include")
set(HIGHS_INCLUDE_DIRS "${CASADI_PATH}/include/highs")
set(CASADI_LIBRARY_DIRS "${CASADI_PATH}")

# file(GLOB JIT_LIBRARY_DIRS "${MPC_SOURCE_DIR}/*jit.so")

message(STATUS "HIGHS_INCLUDE_DIRS: ${HIGHS_INCLUDE_DIRS}")

find_package(PkgConfig REQUIRED)
# pkg_check_modules(HIGHS REQUIRED highs)

add_definitions(-DMASTERDIR="$ENV{masterdir}")

# Set up compile time measurement
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")


message(STATUS "CMAKE_C_COMPILER_ID: ${CMAKE_C_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")

if(CMAKE_C_COMPILER_ID MATCHES "GNU" AND CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    # Get the compiler version
    string(REPLACE "." ";" GCC_VERSION_LIST "${CMAKE_CXX_COMPILER_VERSION}")
    list(GET GCC_VERSION_LIST 0 GCC_VERSION_MAJOR)

    # Initialize base path
    set(BASE_PATH "/media/daten/Anwendungen/cpp_compiler")

    message(STATUS "GCC version: ${GCC_VERSION_MAJOR}")

    # Set specific version names for GCC-13 and GCC-14
    if(GCC_VERSION_MAJOR EQUAL 13)
        set(VERSION_NAME "xpack-gcc-13.3.0-1")
    elseif(GCC_VERSION_MAJOR EQUAL 14)
        set(VERSION_NAME "xpack-gcc-14.2.0-1")
    endif()

    # If VERSION_NAME is set, construct paths and check for existence
    if(VERSION_NAME)
        set(INCLUDE_PATH "${BASE_PATH}/${VERSION_NAME}/include/c++")
        set(LIBRARY_PATH "${BASE_PATH}/${VERSION_NAME}/lib64")

        # Check if the include directory exists
        if(EXISTS "${INCLUDE_PATH}")
            include_directories(${INCLUDE_PATH})
            message(STATUS "Using custom include path: ${INCLUDE_PATH}")
        else()
            message(WARNING "Specified path for ${VERSION_NAME} does not exist. Using default include paths.")
        endif()

        # Check if the library directory exists
        if(EXISTS "${LIBRARY_PATH}")
            link_directories(${LIBRARY_PATH})
            message(STATUS "Using custom library path: ${LIBRARY_PATH}")
        else()
            message(WARNING "Library path for ${VERSION_NAME} does not exist. Using default library paths.")
        endif()
    else()
        set(LIBRARY_PATH "")
        message(STATUS "Using default include paths for GCC-${GCC_VERSION_MAJOR}")
    endif()

endif()

message(STATUS "LIBRARY_PATH: ${LIBRARY_PATH}")



# Create a shared library for each MPC*.c file
foreach(SOURCE_FILE ${MPC_SOURCES})
    get_filename_component(FILE_NAME ${SOURCE_FILE} NAME_WE)  # Get the filename without extension

    # Create the shared library
    add_library(${FILE_NAME} SHARED ${SOURCE_FILE})

    # Set the output library name to 'lib<NAME>.so'
    set_target_properties(${FILE_NAME} PROPERTIES OUTPUT_NAME "${FILE_NAME}" INSTALL_RPATH "${LIBRARY_PATH}")

    # Include directories
    target_include_directories(${FILE_NAME} PRIVATE
        ${MPC_SOURCE_DIR}
        ${CASADI_INCLUDE_DIRS}
        ${HIGHS_INCLUDE_DIRS}
        )

    target_link_libraries(${FILE_NAME} PRIVATE 
        ${CASADI_LIBRARY_DIRS}/libipopt.so
        ${CASADI_LIBRARY_DIRS}/libfatrop.so
        ${CASADI_LIBRARY_DIRS}/libblasfeo.so
        ${CASADI_LIBRARY_DIRS}/libosqp.so
        ${CASADI_LIBRARY_DIRS}/libdaqp.so
        ${CASADI_LIBRARY_DIRS}/libhighs.so
        # ${JIT_LIBRARY_DIRS}
        )

    target_compile_options(${FILE_NAME} PRIVATE -fdiagnostics-color=always ${MPC_OPT_FLAG}) # it makes sense to optimize the MPC library also in debug mode

    # Collect the libraries for installation
    list(APPEND MPC_ALL_LIBRARIES ${FILE_NAME})

        # Sort into param and non-param lists
    if(FILE_NAME MATCHES "_param$")
        list(APPEND MPC_PARAM_LIBRARIES ${FILE_NAME})
    else()
        list(APPEND MPC_LIBRARIES ${FILE_NAME})
    endif()
endforeach()

if(NOT DEFINED MPC_ALL_LIBRARIES OR MPC_ALL_LIBRARIES STREQUAL "")
  message(WARNING "MPC_ALL_LIBRARIES on path \"${MPC_SOURCE_DIR}\" is empty or not defined")
else()
  message(STATUS "MPC libraries: ${MPC_ALL_LIBRARIES}")
endif()

list(APPEND CMAKE_PREFIX_PATH "$ENV{MAMBA_ROOT_PREFIX}/envs/ros_env/")

set(Python_EXECUTABLE "$ENV{_colcon_cd_root}/bin/python")
set(Python3_EXECUTABLE "$ENV{_colcon_cd_root}/bin/python3")

message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
list(APPEND CMAKE_PREFIX_PATH "${MPC_SOURCE_DIR}")
link_directories("${MPC_SOURCE_DIR}")

find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_library(RT_LIB rt)

# Add source files for main executable, including CasadiMPC.cpp
file(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")  # Recursively find all .cpp files in src folder
# Define utility libraries
set(UTILS_LIBRARIES
    CrocoddylController
    CrocoddylMPC
    WorkspaceController
    CasadiController
    FullSystemTorqueMapper
    RobotModel
    SharedMemory
    SignalFilter
    CasadiMPC
    TicToc
    TrajectoryGenerator
    CasadiEKF
)

foreach(SOURCE_FILE ${SRC_FILES})
    get_filename_component(FILE_NAME ${SOURCE_FILE} NAME_WE)  # Get the filename without extension

    # Create the shared library
    add_library(${FILE_NAME} SHARED ${SOURCE_FILE})

    # Set the output library name to 'lib<NAME>.so'
    set_target_properties(${FILE_NAME} PROPERTIES OUTPUT_NAME "${FILE_NAME}" INSTALL_RPATH "${MPC_SOURCE_DIR};${LIBRARY_PATH}")

    if(${FILE_NAME} STREQUAL "WorkspaceController")
        set(SHARED_LIBRARIES
            ${EIGEN3_LIBRARIES}
            ${pinocchio_LIBRARIES}
        )
    elseif(${FILE_NAME} STREQUAL "CasadiController" OR ${FILE_NAME} STREQUAL "CasadiMPC")
        set(SHARED_LIBRARIES
            ${EIGEN3_LIBRARIES}
            ${MPC_PARAM_LIBRARIES}
            ${MPC_LIBRARIES}
        )
    elseif(${FILE_NAME} STREQUAL "CrocoddylController" OR ${FILE_NAME} STREQUAL "CrocoddylMPC")
        set(SHARED_LIBRARIES
            ${EIGEN3_LIBRARIES}
            ${pinocchio_LIBRARIES}
            ${crocoddyl_LIBRARIES}
        )
    elseif(${FILE_NAME} STREQUAL "FullSystemTorqueMapper" OR ${FILE_NAME} STREQUAL "RobotModel")
        set(SHARED_LIBRARIES
            ${EIGEN3_LIBRARIES}
            ${pinocchio_LIBRARIES}
        )
    elseif(${FILE_NAME} STREQUAL "SharedMemory")
        set(SHARED_LIBRARIES
            ${rclcpp_LIBRARIES}
        )
    elseif(${FILE_NAME} STREQUAL "SignalFilter")
        set(SHARED_LIBRARIES
            ${EIGEN3_LIBRARIES}
        )
    elseif(${FILE_NAME} STREQUAL "TicToc")
        set(SHARED_LIBRARIES "")
    elseif(${FILE_NAME} STREQUAL "TrajectoryGenerator")
        set(SHARED_LIBRARIES
            ${EIGEN3_LIBRARIES}
        )
    elseif(${FILE_NAME} STREQUAL "CasadiEKF")
        set(SHARED_LIBRARIES
            ${EIGEN3_LIBRARIES}
            ${MPC_LIBRARIES}
        )
    endif()

    # Include directories
    target_include_directories(${FILE_NAME} PRIVATE
        ${pinocchio_INCLUDE_DIRS}
        ${crocoddyl_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${rclcpp_INCLUDE_DIRS}
        ${MPC_SOURCE_DIR} 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(${FILE_NAME} PRIVATE
        ${SHARED_LIBRARIES}
        ${RT_LIB}
        ${CMAKE_THREAD_LIBS_INIT}
        param_robot
    )

    target_compile_options(${FILE_NAME} PRIVATE -Wall -Wextra -fdiagnostics-color=always ${OPT_FLAG})

    # Collect the libraries for installation
endforeach()

# Install libraries
install(TARGETS ${UTILS_LIBRARIES} ${MPC_ALL_LIBRARIES}
    LIBRARY DESTINATION ${MPC_SOURCE_DIR}
)

# Define the path to the main source file
# Main executable
set(MAIN_SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

if(EXISTS ${MAIN_SOURCE_FILE})
    add_executable(main ${MAIN_SOURCE_FILE})
else()
    message(FATAL_ERROR "Main source file not found: ${MAIN_SOURCE_FILE}. Please check the path.")
endif()

set_target_properties(main PROPERTIES 
    BUILD_RPATH "${LIBRARY_PATH}"
    INSTALL_RPATH "${LIBRARY_PATH}"
)

# Specify include directories for the main executable
target_include_directories(main PRIVATE 
    ${pinocchio_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR} 
    ${rclcpp_INCLUDE_DIRS}
    ${MPC_SOURCE_DIR} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link libraries to the main executable
target_link_libraries(main PRIVATE 
    ${pinocchio_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${MPC_PARAM_LIBRARIES}
    ${MPC_LIBRARIES}
    ${UTILS_LIBRARIES}
    ${RT_LIB}
    ${CMAKE_THREAD_LIBS_INIT}
)

target_compile_options(main PRIVATE -Wall -Wextra -fdiagnostics-color=always ${OPT_FLAG})

# Final message to the user
add_custom_target(display_message ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Use 'make install' to copy the files to ${MPC_SOURCE_DIR}"
)
