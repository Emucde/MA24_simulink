#!/bin/bash

# Default to debug build
export CLICOLOR_FORCE=1
export CMAKE_COLOR_DIAGNOSTICS=ON
BUILD_TYPE="debug"

BUILD_CURRENT_FILE=true
if [[ $# -eq 2 ]]; then
    if [ $2 == "$masterdir/main_ros2/read_bin_data" ]; then
        BUILD_CURRENT_FILE=false # use makefile to build
    else
        BUILD_CURRENT_FILE=true
    fi
fi
# echo "BUILD_CURRENT_FILE: $BUILD_CURRENT_FILE"

# Parse command line arguments
if [[ $# -gt 0 ]]; then
    key="$1"
    case $key in
        -r|--release)
        BUILD_TYPE="release"
        shift
        ;;
        -d|--debug)
        BUILD_TYPE="debug"
        shift
        ;;
        *)
        echo "Unknown option: $key"
        exit 1
        ;;
    esac
fi

if [[ $BUILD_CURRENT_FILE == true ]]; then
    echo "Building current file..."
    # Set optimization and debug flags based on build type
    if [ "$BUILD_TYPE" = "release" ]; then
        OPT_FLAGS="-O3"
        DEBUG_FLAGS=""
        EXECUTE_AFTER_BUILD=true   # Set a flag to indicate to execute
    else
        OPT_FLAGS="-O0"
        DEBUG_FLAGS="-g"
    fi

    # Construct the command (slow... use cmake or standalone Makefile instead)
    CMD="/usr/bin/g++-11 -fdiagnostics-color=always $OPT_FLAGS $DEBUG_FLAGS
        -Wl,-rpath,$masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
        $masterdir/main_ros2/read_bin_data/*.cpp $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/*_param.c $masterdir/main_ros2/read_bin_data/src/*.cpp
        -I $_colcon_cd_root/include
        -I $eigen_path
        -I $casadi_path/include
        -I $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
        -L $casadi_path
        -L $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
        $(pkg-config --cflags --libs pinocchio)
        -I $masterdir/main_ros2/read_bin_data/include
        -lMPC01 -lMPC6 -lMPC7 -lMPC8 -lMPC9 -lMPC10 -lMPC11 -lMPC12 -lMPC13 -lMPC14 -lcasadi
        -o $masterdir/main_ros2/read_bin_data/main"

    # Execute the command and measure time
    echo "Executing: $CMD"
    # Use the time command to measure duration and store exit status
    { time eval $CMD; }
    echo -e "\n"
    BUILD_STATUS=$?    # Capture the exit status of the build command
else
    # Use makefile to build
    echo "Building using makefile..."
    # make BUILD_TYPE=$BUILD_TYPE -j8
    cmake --build ./build -j8
    BUILD_STATUS=$?
fi

# Check if the build was successful
if [ $BUILD_STATUS -eq 0 ]; then
    echo -e "Build complete\n"
    # If the build was successful and it's a release build, run the executable
    if [ "$BUILD_TYPE" = "release" ]; then
        echo -e "Running the executable...\n----------------------------------\n"
        # $masterdir/main_ros2/read_bin_data/bin/$BUILD_TYPE/main
        $masterdir/main_ros2/read_bin_data/build/bin/main
        echo -e "\n----------------------------------\n"
    fi
else
    echo "Build failed with status code $BUILD_STATUS"
    exit $BUILD_STATUS   # Exit with the same status code as the build command
fi