#!/bin/bash
# This script is used to run the main Python script for the 7DOF manipulator simulation.
# It is run by using the task runner extension in VSCode.

if [ ! -d "/opt/ros/humble" ]; then
	eval "$(conda shell.bash hook)"
	conda activate mpc
else
	source ~/.bashrc
	PYTHONPATH=""
	source $CONDA_PREFIX/etc/profile.d/conda.sh
	conda activate mpc
fi

# current_dir=$(pwd)
# cd $masterdir
python $masterdir/main_python/main_crocoddyl_7DOF.py
# cd $current_dir
