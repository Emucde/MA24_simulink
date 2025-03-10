#!/bin/bash

# Enable colorized output and logging to stdout
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=1

# Function to display help message
function show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -r           Run the release build"
    echo "  -d           Run the debug build"
    echo "  -h           Show this help message"
    echo ""
    echo "Environment:"
    echo "  Ensure ROS2 Humble or appropriate environment is sourced before running."
}

# Check if no arguments are provided
if [ $# -eq 0 ]; then
    echo "Error: No arguments provided."
    show_help
    exit 1
fi

# Parse command-line arguments
while getopts ":rdh" opt; do
    case $opt in
        r)
            BUILD_TYPE="release"
            ;;
        d)
            BUILD_TYPE="debug"
            ;;
        h)
            show_help
            exit 0
            ;;
        \?)
            echo "Error: Invalid option -$OPTARG"
            show_help
            exit 1
            ;;
    esac
done

# Check if ROS2 Humble is installed or source the environment
if [ ! -d "/opt/ros/humble" ]; then
    source $MAMBA_ROOT_PREFIX/etc/profile.d/mamba.sh
    mamba activate ros_env || { echo "Failed to activate ros_env"; exit 1; }
    source $MAMBA_ROOT_PREFIX/envs/ros_env/setup.bash || { echo "Failed to source ros_env setup.bash"; exit 1; }
else
    source /opt/ros/humble/setup.bash || { echo "Failed to source ROS2 Humble setup.bash"; exit 1; }
fi

# Set up the workspace based on the selected build type
if [ "$BUILD_TYPE" == "release" ]; then
    source $masterdir/main_ros2/franka_ros2_ws/build_release/install/setup.bash || { echo "Failed to source release setup.bash"; exit 1; }
elif [ "$BUILD_TYPE" == "debug" ]; then
    source $masterdir/main_ros2/franka_ros2_ws/build_debug/install/setup.bash || { echo "Failed to source debug setup.bash"; exit 1; }
else
    echo "Error: No valid build type selected."
    show_help
    exit 1
fi

# Run the ROS2 launch command (ensure robot_ip is set in your environment)
if [ -z "$robot_ip" ]; then
    echo "Error: robot_ip environment variable is not set."
    exit 1
fi

while true; do
    nice -n -20 ros2 launch franka_bringup mpc_crocoddyl_controller.launch.py arm_id:=fr3 robot_ip:=$robot_ip
    # nice -n -20 ros2 launch franka_bringup mpc_casadi_controller.launch.py arm_id:=fr3 robot_ip:=$robot_ip
    # ros2 launch franka_bringup mpc_crocoddyl_controller.launch.py arm_id:=fr3 robot_ip:=$robot_ip
    if [ $? -eq 0 ]; then
        echo "ROS2 node launched successfully."
        break
    else
        echo "ROS2 crashed. Restarting..."
        sleep 1
    fi
done

