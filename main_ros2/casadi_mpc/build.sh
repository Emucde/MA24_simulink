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
    echo $PYTHONPATH
else
    source /opt/ros/humble/setup.bash
fi

# INSTALL_PATH=/media/daten/Anwendungen/cpp_compiler
# if [ -f "$INSTALL_PATH/xpack-gcc-14.2.0-1/bin/gcc" ] && [ -f "$INSTALL_PATH/xpack-gcc-14.2.0-1/bin/g++" ]; then
#     export CC=$HOME/xpack-gcc-14.2.0-1/bin/gcc
#     export CXX=$HOME/xpack-gcc-14.2.0-1/bin/g++
# fi
# export CXX=/usr/bin/g++
# export CC=/usr/bin/gcc

BUILD_CURRENT_FILE=false
RUN_MAKE_INSTALL=false
BUILD_ROS2_MPCS=false
CLEAR=false
CLEAR_ALL=false
CLEAR_MPC=false
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
        -cr|--cmake_release)
        JUST_CREATE_CMAKE=true
        BUILD_TYPE="release"
        shift
        ;;
        -cd|--cmake_debug)
        JUST_CREATE_CMAKE=true
        BUILD_TYPE="debug"
        shift
        ;;
        --clear_release)
        CLEAR=true
        CLEAR_ALL=false
        CLEAR_MPC=false
        BUILD_TYPE="release"
        shift
        ;;
        --clear_debug)
        CLEAR=true
        CLEAR_ALL=false
        CLEAR_MPC=false
        BUILD_TYPE="debug"
        shift
        ;;
        --clear_releasempc)
        CLEAR=true
        CLEAR_ALL=false
        CLEAR_MPC=true
        BUILD_TYPE="release"
        shift
        ;;
        --clear_debugmpc)
        CLEAR=true
        CLEAR_ALL=false
        CLEAR_MPC=true
        BUILD_TYPE="debug"
        shift
        ;;
        -iwyu|--include-what-you-use)
        INCLUDE_WHAT_YOU_USE=true
        RUN_FILE=false
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
echo "Check includes with includes-what-you-use (iwyu): $INCLUDE_WHAT_YOU_USE"
echo "CLEAR: $CLEAR"
echo "CLEAR_ALL: $CLEAR_ALL"
echo "CLEAR_MPC: $CLEAR_MPC"
echo "RUN_FILE: $RUN_FILE"
echo ""

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

if [ "$JUST_CREATE_CMAKE" = true ]; then
    if [ "$BUILD_TYPE" = "release" ]; then
        time cmake -Wno-deprecated -B ./main_ros2/casadi_mpc/cpp_class_files/build_release -S ./main_ros2/casadi_mpc/cpp_class_files/ -DCMAKE_BUILD_TYPE=Release
        echo ""
        echo "----------------------------------"
    else
        time cmake -Wno-deprecated -B ./main_ros2/casadi_mpc/cpp_class_files/build_debug -S ./main_ros2/casadi_mpc/cpp_class_files/ -DCMAKE_BUILD_TYPE=Debug
        echo ""
        echo "----------------------------------"
    fi
    exit 0
fi

if [ "$CLEAR" = true ]; then
    if [ "$CLEAR_MPC" = true ]; then
        rm -rf $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/*
        echo "Cleared $masterdir/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/"
    fi
    rm -rf $CMAKE_BUILD_PATH/*

    echo "Cleared $CMAKE_BUILD_PATH/"
    exit 0
fi

if [ "$INCLUDE_WHAT_YOU_USE" = true ]; then
    # Run include-what-you-use
    iwyu_tool -p ./main_ros2/casadi_mpc/cpp_class_files/build_debug/ --jobs=8 -- -I$(gcc -print-file-name=include) -I/usr/include > ./main_ros2/casadi_mpc/cpp_class_files/build_debug/iwyu_output.txt
    echo "Successfully created ./main_ros2/casadi_mpc/cpp_class_files/build_debug/iwyu_output.txt"
    cat ./main_ros2/casadi_mpc/cpp_class_files/build_debug/iwyu_output.txt
    exit 0
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
        time make BUILD_TYPE=$BUILD_TYPE -j8
        cd ..
    else
        time cmake --build $CMAKE_BUILD_PATH -j8
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
                time nice -n 0 ./main_ros2/casadi_mpc/cpp_class_files/bin/$BUILD_TYPE/main
            else
                time nice -n 0 ./main_ros2/casadi_mpc/cpp_class_files/build_release/bin/main
                # if error do the command, after that logout login
                # sudo sed -i '/# End of file/i @realtime soft nice -20\n@realtime hard nice -20' /etc/security/limits.d/realtime.conf
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
