cmake_minimum_required(VERSION 3.10)
project(MPCLibraries)
# set(CMAKE_VERBOSE_MAKEFILE ON)

# Specify the build type; use 'Release' for optimization without debugging info
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)  # Default to Debug. Important: Then all optimization flags are forced to be -O0, other settings are ignored
endif()

# Add optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(OPT_FLAG "-O3")
elseif(CMAKE_BUILD_TYPE STREQUAL "DebugSpecial")
    set(OPT_FLAG "-O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  # Ensure debug information is generated
endif()

# Set output directory for libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
set(CMAKE_COLOR_DIAGNOSTICS ON)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)  # Change to 14/17/20 as necessary
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the path to the MPC source files
set(MPC_SOURCE_DIR_REL ${CMAKE_CURRENT_SOURCE_DIR}/../../../s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles)
cmake_path(SET MPC_SOURCE_DIR NORMALIZE "${MPC_SOURCE_DIR_REL}")
message(STATUS "MPC source directory: ${MPC_SOURCE_DIR}")

# Find all MPC*.c files
file(GLOB MPC_SOURCES "${MPC_SOURCE_DIR}/MPC*.c" "${MPC_SOURCE_DIR}/param_robot.c")
set(MPC_ALL_LIBRARIES "")
set(MPC_PARAM_LIBRARIES "")
set(MPC_LIBRARIES "")

# Set up compile time measurement
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")

# Create a shared library for each MPC*.c file
foreach(SOURCE_FILE ${MPC_SOURCES})
    get_filename_component(FILE_NAME ${SOURCE_FILE} NAME_WE)  # Get the filename without extension

    # Create the shared library
    add_library(${FILE_NAME} SHARED ${SOURCE_FILE})

    # Set the output library name to 'lib<NAME>.so'
    set_target_properties(${FILE_NAME} PROPERTIES OUTPUT_NAME "${FILE_NAME}")

    # Include directories
    target_include_directories(${FILE_NAME} PRIVATE ${MPC_SOURCE_DIR})

    target_compile_options(${FILE_NAME} PRIVATE -fdiagnostics-color=always -O3) # it makes sense to optimize the MPC library also in debug mode

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

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
list(APPEND CMAKE_PREFIX_PATH "${MPC_SOURCE_DIR}")
link_directories("${MPC_SOURCE_DIR}")

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# Add source files for main executable, including CasadiMPC.cpp
file(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")  # Recursively find all .cpp files in src folder
set(UTILS_LIBRARIES "")

foreach(SOURCE_FILE ${SRC_FILES})
    get_filename_component(FILE_NAME ${SOURCE_FILE} NAME_WE)  # Get the filename without extension

    # Create the shared library
    add_library(${FILE_NAME} SHARED ${SOURCE_FILE})

    # Set the output library name to 'lib<NAME>.so'
    set_target_properties(${FILE_NAME} PROPERTIES OUTPUT_NAME "${FILE_NAME}" INSTALL_RPATH "${MPC_SOURCE_DIR}")

    # Include directories
    target_include_directories(${FILE_NAME} PRIVATE 
        ${pinocchio_INCLUDE_DIRS} 
        ${EIGEN3_INCLUDE_DIR}
        ${MPC_SOURCE_DIR} 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(${FILE_NAME} PRIVATE
        ${pinocchio_LIBRARIES} 
        ${EIGEN3_LIBRARIES} 
        ${MPC_PARAM_LIBRARIES}
        ${MPC_LIBRARIES}
    )

    target_compile_options(${FILE_NAME} PRIVATE -Wall -Wextra -fdiagnostics-color=always ${OPT_FLAG})

    # Collect the libraries for installation
    list(APPEND UTILS_LIBRARIES ${FILE_NAME})
endforeach()

if(NOT DEFINED UTILS_LIBRARIES OR UTILS_LIBRARIES STREQUAL "")
    message(WARNING "UTILS_LIBRARIES on path \"${CMAKE_CURRENT_SOURCE_DIR}\" is empty or not defined")
else()
    message(STATUS "UTILS libraries: ${UTILS_LIBRARIES}")
endif()

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

# Specify include directories for the main executable
target_include_directories(main PRIVATE 
    ${pinocchio_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR} 
    ${MPC_SOURCE_DIR} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link libraries to the main executable
target_link_libraries(main PRIVATE 
    ${pinocchio_LIBRARIES} 
    ${EIGEN3_LIBRARIES} 
    ${MPC_PARAM_LIBRARIES}
    ${MPC_LIBRARIES}
    ${UTILS_LIBRARIES}
)

target_compile_options(main PRIVATE -Wall -Wextra -fdiagnostics-color=always ${OPT_FLAG})

# Final message to the user
add_custom_target(display_message ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Use 'make install' to copy the files to ${MPC_SOURCE_DIR}"
)
