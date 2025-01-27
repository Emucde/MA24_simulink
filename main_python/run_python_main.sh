#!/bin/bash
source ~/.bashrc
PYTHONPATH=""
source $CONDA_PREFIX/etc/profile.d/conda.sh
conda activate mpc

# current_dir=$(pwd)
# cd $masterdir
python $masterdir/main_python/main_crocoddyl_7DOF.py
# cd $current_dir