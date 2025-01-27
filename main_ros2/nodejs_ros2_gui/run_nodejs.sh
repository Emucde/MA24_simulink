#!/bin/bash

# enable ros
if [ ! -d "/opt/ros/humble" ]; then
    source $MAMBA_ROOT_PREFIX/etc/profile.d/mamba.sh
    mamba activate ros_env
    source $MAMBA_ROOT_PREFIX/envs/ros_env/setup.bash
else
    source /opt/ros/humble/setup.bash
fi

# enable franka ros2 workspace
source $masterdir/main_ros2/franka_ros2_ws/build_release/install/setup.bash

# enable conda environment
source ~/.bashrc
PYTHONPATH=""
source $CONDA_PREFIX/etc/profile.d/conda.sh
conda activate mpc

node $masterdir/main_ros2/nodejs_ros2_gui/src/app.js