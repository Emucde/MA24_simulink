cmake_minimum_required(VERSION 3.10)  # Specify the CMake version
project(BinaryFileReader)              # Specify the project name

# Create a shared library from MPC8.c
add_library(MPC8 SHARED MPC8.c)

# Add the source file for CasadiMPC
add_library(CasadiMPC STATIC CasadiMPC.cpp) # Add CasadiMPC library

set(CMAKE_CXX_STANDARD 11)             # Set the C++ standard

# Add the executable target
add_executable(${PROJECT_NAME} main.cpp)

# Link the shared library to the main executable
target_link_libraries(${PROJECT_NAME} MPC8 CasadiMPC)

# Include directory for the headers
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})