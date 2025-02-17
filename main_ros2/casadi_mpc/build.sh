#!/bin/bash
source ~/.bashrc

# Default to debug build
export CLICOLOR_FORCE=1
export CMAKE_COLOR_DIAGNOSTICS=ON
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=1
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
RUN_ONLY=false

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
        --run_only)
        RUN_ONLY=true
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
echo "RUN_ONLY: $RUN_ONLY"
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

if [ "$RUN_ONLY" = true ]; then
    time nice -n -20 ./main_ros2/casadi_mpc/cpp_class_files/build_release/bin/main
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
       time make BUILD_TYPE=$BUILD_TYPE -j2
       BUILD_STATUS=$?
       cd ..
    else
        TIME_FILE="build_time.log"
        LAST_FILE="last_build_time.log"

        # Read previous build times or set defaults
        [ -f "$TIME_FILE" ] && MAX_BUILD_TIME=$(cat "$TIME_FILE") || MAX_BUILD_TIME=60
        [ -f "$LAST_FILE" ] && LAST_BUILD_TIME=$(cat "$LAST_FILE") || LAST_BUILD_TIME=60

        # if MAX_BUILD_TIME is no number set to 60
        if ! [[ "$MAX_BUILD_TIME" =~ ^[0-9]+$ ]]; then
            MAX_BUILD_TIME=60
        fi

        # if LAST_BUILD_TIME is no number set to 60
        if ! [[ "$LAST_BUILD_TIME" =~ ^[0-9]+$ ]]; then
            LAST_BUILD_TIME=60
        fi

        # Start a background process to show elapsed time and progress bars
        (
            start_time=$(date +%s)
            while true; do
                sleep 1
                current_time=$(date +%s)
                elapsed_time=$((current_time - start_time))

                # Calculate progress percentages
                [ "$LAST_BUILD_TIME" -gt 0 ] && last_percent=$((elapsed_time * 100 / LAST_BUILD_TIME)) || last_percent=0
                [ "$MAX_BUILD_TIME" -gt 0 ] && max_percent=$((elapsed_time * 100 / MAX_BUILD_TIME)) || max_percent=0

                # Cap progress at 100%
                [ "$last_percent" -gt 100 ] && last_percent=100
                [ "$max_percent" -gt 100 ] && max_percent=100

                # Create progress bars
                function progress_bar() {
                    local percent=$1
                    local bar_length=20
                    local filled=$((bar_length * percent / 100))
                    local empty=$((bar_length - filled))
                    printf "[%s%s] %3d%%" "$(printf '#%.0s' $(seq 1 $filled))" "$(printf ' %.0s' $(seq 1 $empty))" "$percent"
                }

                # Display elapsed time and progress bars
                printf "\r%02d:%02d:%02d  Last: %s  Max: %s" \
                    $((elapsed_time / 3600)) $(((elapsed_time % 3600) / 60)) $((elapsed_time % 60)) \
                    "$(progress_bar $last_percent)" "$(progress_bar $max_percent)"
            done 
        ) &
        timer_pid=$!

        # Run the cmake build command and capture its output
        start_build_time=$(date +%s)
        time cmake --build "$CMAKE_BUILD_PATH" -j8;
        BUILD_STATUS=$?
        end_build_time=$(date +%s)

        NEW_BUILD_TIME=$((end_build_time - start_build_time))

        # Save last build time
        echo "$NEW_BUILD_TIME" > "$LAST_FILE"

        # Calculate and save new build time
        if [ "$NEW_BUILD_TIME" -gt "$MAX_BUILD_TIME" ]; then
            echo "$NEW_BUILD_TIME" > "$TIME_FILE"
        fi

        # Once the build is complete, kill the timer
        kill $timer_pid

        # Optionally, wait for the background process to exit
        wait $timer_pid
    fi
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
                time nice -n -20 ./main_ros2/casadi_mpc/cpp_class_files/bin/$BUILD_TYPE/main
            else
                time nice -n -20 ./main_ros2/casadi_mpc/cpp_class_files/build_release/bin/main
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
        CURRENT_PATH=$(pwd);
        cd $UTILS_BUILD_PATH
        make install
        cd $CURRENT_PATH
    fi
else
    echo "Build failed with status code $BUILD_STATUS"
    exit $BUILD_STATUS   # Exit with the same status code as the build command
fi

# build ros2
if [ $BUILD_ROS2_MPCS == true ]; then
    if [ "$BUILD_TYPE" = "release" ]; then
        BUILD_PATH="$masterdir/main_ros2/franka_ros2_ws/build_release"
        BUILD_TYPE="Release"
    else
        BUILD_PATH="$masterdir/main_ros2/franka_ros2_ws/build_debug"
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
    COMMAND="RCUTILS_COLORIZED_OUTPUT=1 colcon --log-base $LOG \
                build --cmake-args \
                -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
                -DCMAKE_VERBOSE_MAKEFILE=ON \
                --install-base $INSTALL \
                --build-base $BUILD"
    echo $COMMAND
    eval $COMMAND
    cd $CURRENT_PATH
fi
