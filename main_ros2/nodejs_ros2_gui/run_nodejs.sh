#!/bin/bash

# This script is used to run the Node.js application in a ROS 2 environment.
# It is a websocket server that communicates with the ROS 2 system and can
# be opened by using http://localhost:8080

# enable ros
if [ ! -d "/opt/ros/humble" ]; then
    source $MAMBA_ROOT_PREFIX/etc/profile.d/mamba.sh
    mamba activate ros_env
    source $MAMBA_ROOT_PREFIX/envs/ros_env/setup.bash
else
    source /opt/ros/humble/setup.bash
    source $CONDA_PREFIX/etc/profile.d/conda.sh
    conda activate mpc
fi

# enable franka ros2 workspace
source $masterdir/main_ros2/franka_ros2_ws/build_release/install/setup.bash

# enable conda environment
source ~/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$_colcon_cd_root/lib
PYTHONPATH=""

while true; do
    node $masterdir/main_ros2/nodejs_ros2_gui/src/app.js
    if [ $? -eq 0 ]; then
        echo "nodejs launched successfully."
        break
    else
        echo "nodejs crashed. Restarting..."
    fi
done