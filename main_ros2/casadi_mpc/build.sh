#!/bin/bash

# Default to debug build
export CLICOLOR_FORCE=1
export CMAKE_COLOR_DIAGNOSTICS=ON
export TERM=dumb
BUILD_TYPE="debug"
STANDALONE_MAKEFILE=false

BUILD_CURRENT_FILE=true
if [[ $# -eq 2 ]]; then
    if [ $2 == "$masterdir/main_ros2/casadi_mpc/cpp_class_files" ]; then
        BUILD_CURRENT_FILE=false # use makefile to build
    else
        BUILD_CURRENT_FILE=true
    fi
fi

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

echo "BUILD_CURRENT_FILE: $BUILD_CURRENT_FILE"
echo "STANDALONE_MAKEFILE: $STANDALONE_MAKEFILE"
echo "BUILD_TYPE: $BUILD_TYPE"
echo ""

if [ "$BUILD_TYPE" = "release" ]; then
    OPT_FLAGS="-O3"
    DEBUG_FLAGS=""
    EXECUTE_AFTER_BUILD=true   # Set a flag to indicate to execute
    CMAKE_BUILD_PATH="./cpp_class_files/build_release"
else
    OPT_FLAGS="-O0"
    DEBUG_FLAGS="-g"
    CMAKE_BUILD_PATH="./cpp_class_files/build_debug"
fi

if [[ $BUILD_CURRENT_FILE == true ]]; then
    echo "Building current file..."

    # Construct the command (slow... use cmake or standalone Makefile instead)
    CMD="/usr/bin/g++-11 -fdiagnostics-color=always $OPT_FLAGS $DEBUG_FLAGS
        -Wl,-rpath,$masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
        $masterdir/main_ros2/casadi_mpc/cpp_class_files/*.cpp $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/*_param.c $masterdir/main_ros2/casadi_mpc/cpp_class_files/src/*.cpp
        -I $_colcon_cd_root/include
        -I $eigen_path
        -I $casadi_path/include
        -I $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
        -L $casadi_path
        -L $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
        $(pkg-config --cflags --libs pinocchio)
        -I $masterdir/main_ros2/casadi_mpc/cpp_class_files/include
        -lMPC01 -lMPC6 -lMPC7 -lMPC8 -lMPC9 -lMPC10 -lMPC11 -lMPC12 -lMPC13 -lMPC14 -lcasadi
        -o $masterdir/main_ros2/casadi_mpc/cpp_class_files/main"

    # Execute the command and measure time
    echo "Executing: $CMD"
    # Use the time command to measure duration and store exit status
    { time eval $CMD; }
    echo -e "\n"
    BUILD_STATUS=$?    # Capture the exit status of the build command
else
    # Use makefile to build
    echo "Building using makefile..."
    if [ $STANDALONE_MAKEFILE == true ]; then
        cd "cpp_class_files"
        make BUILD_TYPE=$BUILD_TYPE -j8
        cd ..
    else
        cmake --build $CMAKE_BUILD_PATH -j8
    fi
    
    BUILD_STATUS=$?
fi

# Check if the build was successful
if [ $BUILD_STATUS -eq 0 ]; then
    echo -e "Build complete\n"
    # If the build was successful and it's a release build, run the executable
    if [ "$BUILD_TYPE" = "release" ]; then
        echo -e "Running the executable...\n----------------------------------\n"
        if [ $STANDALONE_MAKEFILE == true ]; then
            $masterdir/main_ros2/casadi_mpc/cpp_class_files/bin/$BUILD_TYPE/main
        else
            cd $masterdir/main_ros2/casadi_mpc/cpp_class_files
            $masterdir/main_ros2/casadi_mpc/cpp_class_files/build_release/bin/main
        fi
        echo -e "\n----------------------------------\n"

        echo -e "run make install? (y/n)"
        read -r response
        if [[ $response =~ ^([yY][eE][sS]|[yY])$ ]]; then
            cd $masterdir/main_ros2/casadi_mpc/cpp_class_files/build_release
            make install
        fi
    fi
else
    echo "Build failed with status code $BUILD_STATUS"
    exit $BUILD_STATUS   # Exit with the same status code as the build command
fi
