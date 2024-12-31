cmake_minimum_required(VERSION 3.10)
project(MPCLibraries)

# Specify the build type; use 'Release' for optimization without debugging info
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)  # Default to Release
endif()

# Set output directory for libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Define the path to the MPC source files
set(MPC_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles)

# Find all MPC*.c files
file(GLOB MPC_SOURCES "${MPC_SOURCE_DIR}/MPC*.c")

# Add optimization flags
add_compile_options(-O3)    # Adds optimization level 3
# Note: You generally don't need to specify -g or -G flags if you want no debugging information.

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

    # Collect the libraries for installation
    list(APPEND MPC_LIBRARIES ${FILE_NAME})
endforeach()

# Install libraries
install(TARGETS ${MPC_LIBRARIES}
    LIBRARY DESTINATION ${MPC_SOURCE_DIR}
)

# Final message to the user
add_custom_target(display_message ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Use 'make install' to copy the files to ${MPC_SOURCE_DIR}"
)