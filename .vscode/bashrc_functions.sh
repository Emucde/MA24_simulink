export casadi_path=/media/daten/Anwendungen/casadi-3.6.6-linux64-matlab2018b
export eigen_path=/usr/include/eigen3
export MATLAB_ROOT=/media/daten/Anwendungen/MATLAB/R2022b

export _colcon_cd_root=/opt/ros/humble/
export PKG_CONFIG_PATH=/opt/ros/humble/lib/x86_64-linux-gnu/pkgconfig/
export PIN_LIBS=$(pkg-config --cflags --libs pinocchio)


function ros2enable()
{
	if [ $CONDA_SHLVL -eq 1 ]; then
		export ROS2_ENABLED=true
		conda deactivate
		export $(cat /etc/environment)
		. ~/.bashrc
		cd $masterdir/main_ros2/franka_ros2_ws
		source /opt/ros/humble/setup.bash
	fi
}

function sourcempc()
{
	if [ ! $FRANKA_ROS2_SOURCED ]; then
		export OLD_ENV=$(env)
		export FRANKA_ROS2_SOURCED=true
	fi
	
	echo "source $masterdir/main_ros2/franka_ros2_ws/install/setup.bash"
	source $masterdir/main_ros2/franka_ros2_ws/install/setup.bash
}

function unsourcempc()
{
	if [ $FRANKA_ROS2_SOURCED ]; then
		export FRANKA_ROS2_SOURCED=false
		while IFS='=' read -r key value; do
			export "$key"="$value"
		done <<< "$OLD_ENV"
	fi
}

function buildmpc()
{
	unsourcempc
	ros2enable
	CURRENT_PATH=$(pwd)
	cd $masterdir/main_ros2/franka_ros2_ws
        echo "colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
	cd $CURRENT_PATH
}

function runmpc()
{
	ros2enable
	sourcempc
	echo "ros2 launch franka_bringup mpc_controller.launch.py arm_id:=fr3 robot_ip:=$robot_ip"
	ros2 launch franka_bringup mpc_controller.launch.py arm_id:=fr3 robot_ip:=$robot_ip
}

function runnodejs()
{
	ros2enable
	sourcempc
	conda activate mpc
	node $masterdir/main_ros2/nodejs_ros2_gui/src/app.js
}