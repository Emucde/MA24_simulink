#!/bin/bash
source ~/.bashrc

# Default to debug build
export CLICOLOR_FORCE=1
export CMAKE_COLOR_DIAGNOSTICS=ON
export TERM=dumb
BUILD_TYPE="debug"
STANDALONE_MAKEFILE=false

if [ ! -d "/opt/ros/humble" ]; then
    source $MAMBA_ROOT_PREFIX/etc/profile.d/mamba.sh
    mamba activate ros_env
    source $MAMBA_ROOT_PREFIX/envs/ros_env/setup.bash
else
    source /opt/ros/humble/setup.bash
fi

# export CXX=/usr/bin/g++
# export CC=/usr/bin/gcc

BUILD_CURRENT_FILE=false
RUN_MAKE_INSTALL=false
BUILD_ROS2_MPCS=false
RUN_FILE=true

if [[ $# -ge 2 ]]; then
    if [ $2 == "all" ]; then
        BUILD_CURRENT_FILE=false
        RUN_MAKE_INSTALL=true
        BUILD_ROS2_MPCS=true
        RUN_FILE=false
    fi
fi

# Parse command line arguments
JUST_CREATE_CMAKE=false
echo $#
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
        -c|--cmake)
        JUST_CREATE_CMAKE=true
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
echo "RUN_MAKE_INSTALL: $RUN_MAKE_INSTALL"
echo "BUILD_ROS2_MPCS: $BUILD_ROS2_MPCS"
echo "JUST_CREATE_CMAKE: $JUST_CREATE_CMAKE"
echo ""

if [ "$JUST_CREATE_CMAKE" = true ]; then
    cmake -B ./main_ros2/casadi_mpc/cpp_class_files/build_debug -S ./main_ros2/casadi_mpc/cpp_class_files/ -DCMAKE_BUILD_TYPE=Debug
    echo ""
    echo "----------------------------------"
    
    cmake -B ./main_ros2/casadi_mpc/cpp_class_files/build_release -S ./main_ros2/casadi_mpc/cpp_class_files/ -DCMAKE_BUILD_TYPE=Release
    echo ""
    echo "----------------------------------"
    exit 0
fi

if [ "$BUILD_TYPE" = "release" ]; then
    OPT_FLAGS="-O3"
    DEBUG_FLAGS=""
    EXECUTE_AFTER_BUILD=true   # Set a flag to indicate to execute
    CMAKE_BUILD_PATH="./main_ros2/casadi_mpc/cpp_class_files/build_release"
else
    OPT_FLAGS="-O0"
    DEBUG_FLAGS="-g"
    CMAKE_BUILD_PATH="./main_ros2/casadi_mpc/cpp_class_files/build_debug"
fi

# Create the build directory if it does not exist
if [ ! -d "$CMAKE_BUILD_PATH" ]; then
    CURRENT_DIR=$(pwd)
    mkdir -p "$CMAKE_BUILD_PATH"
    cd "$CMAKE_BUILD_PATH"
    cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
    cd "$CURRENT_DIR"
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
        UTILS_BUILD_PATH=./main_ros2/casadi_mpc/cpp_class_files/build_release
        if [ $RUN_FILE == true ]; then
            echo -e "Running the executable...\n----------------------------------\n"
            if [ $STANDALONE_MAKEFILE == true ]; then
                ./main_ros2/casadi_mpc/cpp_class_files/bin/$BUILD_TYPE/main
            else
                ./main_ros2/casadi_mpc/cpp_class_files/build_release/bin/main
            fi
        fi

        echo -e "\n----------------------------------\n"

        # if [ $RUN_MAKE_INSTALL == false ]; then
        #     echo -e "run make install? (y/n)"
        #     read -r response
        #     if [[ $response =~ ^([yY][eE][sS]|[yY])$ ]]; then
        #         cd $UTILS_BUILD_PATH
        #         make install
        #     fi
        # fi
    else
        UTILS_BUILD_PATH=./main_ros2/casadi_mpc/cpp_class_files/build_debug
    fi

    if [ $RUN_MAKE_INSTALL == true ]; then
        cd $UTILS_BUILD_PATH
        make install
    fi
else
    echo "Build failed with status code $BUILD_STATUS"
    exit $BUILD_STATUS   # Exit with the same status code as the build command
fi

# build ros2
if [ $BUILD_ROS2_MPCS == true ]; then
    if [ "$BUILD_TYPE" = "release" ]; then
        BUILD_PATH=./main_ros2/franka_ros2_ws/build_release
        BUILD_TYPE="Release"
    else
        BUILD_PATH=./main_ros2/franka_ros2_ws/build_debug
        BUILD_TYPE="Debug"
    fi

    CURRENT_PATH=$(pwd);
    LOG=$BUILD_PATH/log
    INSTALL=$BUILD_PATH/install
    BUILD=$BUILD_PATH/build
    
    mkdir -p $LOG
    mkdir -p $INSTALL
    mkdir -p $BUILD

    cd $masterdir/main_ros2/franka_ros2_ws
    COMMAND="colcon --log-base $LOG \
                build --cmake-args \
                -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
                -DCMAKE_VERBOSE_MAKEFILE=ON \
                --install-base $INSTALL \
                --build-base $BUILD"
    echo $COMMAND
    eval $COMMAND
    cd $CURRENT_PATH
fi