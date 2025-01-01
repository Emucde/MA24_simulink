#!/bin/bash

# Default to debug build
BUILD_TYPE="debug"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
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
done

# Set optimization and debug flags based on build type
if [ "$BUILD_TYPE" = "release" ]; then
    OPT_FLAGS="-O3"
    DEBUG_FLAGS=""
    EXECUTE_AFTER_BUILD=true   # Set a flag to indicate to execute
else
    OPT_FLAGS="-O0"
    DEBUG_FLAGS="-g"
fi

# Set master directory
masterdir="/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink"

# Construct the command
CMD="/usr/bin/g++-11 -fdiagnostics-color=always $OPT_FLAGS $DEBUG_FLAGS
    -Wl,-rpath=$masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
    $masterdir/main_ros2/read_bin_data/*.cpp
    $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC*_param.c
    -I $_colcon_cd_root/include
    -I $eigen_path
    -I $casadi_path/include
    -I $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
    -L $casadi_path
    -L $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
    $(pkg-config --cflags --libs pinocchio)
    -lMPC01 -lMPC6 -lMPC7 -lMPC8 -lMPC9 -lMPC10 -lMPC11 -lMPC12 -lMPC13 -lMPC14 -lcasadi
    -o $masterdir/main_ros2/read_bin_data/main"

# Execute the command and measure time
echo "Executing: $CMD"
# Use the time command to measure duration and store exit status
{ time eval $CMD; }
echo -e "\n"
BUILD_STATUS=$?    # Capture the exit status of the build command

# Check if the build was successful
if [ $BUILD_STATUS -eq 0 ]; then
    echo -e "Build complete\n"
    # If the build was successful and it's a release build, run the executable
    if [ "$BUILD_TYPE" = "release" ]; then
        echo -e "Running the executable...\n----------------------------------\n"
        $masterdir/main_ros2/read_bin_data/main
        echo -e "\n----------------------------------\n"
    fi
else
    echo "Build failed with status code $BUILD_STATUS"
    exit $BUILD_STATUS   # Exit with the same status code as the build command
fi