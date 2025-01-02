cmake_minimum_required(VERSION 3.10)
project(MPCLibraries)

# Specify the build type; use 'Release' for optimization without debugging info
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)  # Default to Release
endif()

# Set output directory for libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)  # Change to 14/17/20 as necessary
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the path to the MPC source files
set(MPC_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles)

# Find all MPC*.c files
file(GLOB MPC_SOURCES "${MPC_SOURCE_DIR}/MPC*.c")
file(GLOB MPC_PARAM_SOURCES "${MPC_SOURCE_DIR}/MPC*_param.c")
set(MPC_ALL_LIBRARIES "")
set(MPC_PARAM_LIBRARIES "")
set(MPC_LIBRARIES "")

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
    list(APPEND MPC_ALL_LIBRARIES ${FILE_NAME})

        # Sort into param and non-param lists
    if(FILE_NAME MATCHES "_param$")
        list(APPEND MPC_PARAM_LIBRARIES ${FILE_NAME})
    else()
        list(APPEND MPC_LIBRARIES ${FILE_NAME})
    endif()
endforeach()

# Install libraries
install(TARGETS ${MPC_ALL_LIBRARIES}
    LIBRARY DESTINATION ${MPC_SOURCE_DIR}
)

# Create the main executable
set(COLCON_INCLUDE "${colcon_cd_root}/include")  # Update with actual path
set(EIGEN_PATH "${eigen_path}")  # Update with actual path
set(CASADI_PATH "${casadi_path}")  # Update with actual path
set(MASTERDIR "${masterdir}")  # Update with actual master directory path
# set(MPC_ALL_LIBRARIES "MPC12_param" "MPC6_param" "MPC11_param" "MPC7_param" "MPC01_param" "MPC14_param" "MPC13_param" "MPC10_param" "MPC8_param" "MPC9_param" "MPC12" "MPC6" "MPC11" "MPC7" "MPC01" "MPC14" "MPC13" "MPC10" "MPC8" "MPC9")

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories for main executable
include_directories(include)

# Add source files for main executable, including CasadiMPC.cpp
file(GLOB_RECURSE SRC_FILES "src/*.cpp")  # Recursively find all .cpp files in src folder

list(REMOVE_ITEM SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/CasadiMPC.cpp")

# Define the shared library
add_library(CasadiMPC SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/CasadiMPC.cpp)
target_include_directories(CasadiMPC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${MPC_SOURCE_DIR})
target_link_libraries(CasadiMPC ${MPC_ALL_LIBRARIES} ${MPC_PARAM_LIBRARIES} ${MPC_LIBRARIES})
set(CASADI_MPC_LIBS CasadiMPC)

message(STATUS "CasadiMPC: ${CasadiMPC}")

# Define the path to the main source file
set(MAIN_SOURCE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

# Ensure the main source file is found
# Ensure the main source file is found
if(EXISTS ${MAIN_SOURCE_FILE})
    add_executable(main ${MAIN_SOURCE_FILE} ${SRC_FILES})  # Include all source files here
else()
    message(FATAL_ERROR "Main source file not found: ${MAIN_SOURCE_FILE}. Please check the path.")
endif()

# Specify include directories for your target
target_include_directories(main PRIVATE 
    ${pinocchio_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR} 
    ${MPC_SOURCE_DIR} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# Link the library to the main executable (if needed)
target_link_libraries(main 
    ${pinocchio_LIBRARIES} 
    ${EIGEN3_LIBRARIES} 
    ${MPC_ALL_LIBRARIES}  # Link against your generated libraries if needed
    ${MPC_PARAM_LIBRARIES}
    ${MPC_LIBRARIES}
    ${CASADI_MPC_LIBS}
)
# Final message to the user
add_custom_target(display_message ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Use 'make install' to copy the files to ${MPC_SOURCE_DIR}"
)
