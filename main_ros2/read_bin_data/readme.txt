There are many methods to get the shared library files:
# MPC01, MPC6, MPC7 takes extremely long to compile if using opti sfun (due to full rigid body model)

----------------------------------------------------------------
# Way 1: manually (optional: add optimization flag -O3):

gcc -shared -o libMPC8.so -fPIC MPC8.c

----------------------------------------------------------------
# Way 2: Using Vscode tasks.json

1) Press Ctrl+Shift+P
2) Input the text
>Tasks: Run Task
3) Select
Build MPC Shared Libraries
# Files are created at ./s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles

----------------------------------------------------------------
# Way3: Use cmake and make

cd ./main_ros2/read_bin_data/build
cmake ..
make
make install # files are copied to # Files are created at ./s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles

# optional: use multiple cores:

# Environment variable
set -x CMAKE_JOB_COUNT 4

# Using -j with cmakes
cmake -j4 ..

# optional: change optimization flag:
add_compile_options(-O3)    # Adds optimization level 3

----------------------------------------------------------------
# Way4: Use cmake and ninja

cd ./main_ros2/read_bin_data/build
cmake -G Ninja ..
ninja

optional: use multiple cores:
ninja -j 4

----------------------------------------------------------------
estimated compile times by using AMD Ryzen 7 4700U, and only 4 cores:

(mpc) [10:08:15 emu@ROS2 build]$ cmake ..
-- The C compiler identification is GNU 13.3.0
-- The CXX compiler identification is GNU 11.4.0
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /media/daten/Anwendungen/anaconda3/envs/mpc/bin/x86_64-conda-linux-gnu-cc - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done
-- Generating done
-- Build files have been written to: /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/main_ros2/read_bin_data/build
(mpc) [10:08:22 emu@ROS2 build]$ make -j4
[  5%] Building C object CMakeFiles/MPC01.dir/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC01.c.o
[ 15%] Building C object CMakeFiles/MPC11.dir/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC11.c.o
[ 15%] Building C object CMakeFiles/MPC12.dir/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC12.c.o
[ 20%] Building C object CMakeFiles/MPC10.dir/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC10.c.o
Elapsed time: 14 s. (time), 0.000702 s. (clock)
[ 25%] Linking C shared library lib/libMPC12.so
[ 25%] Built target MPC12
[ 30%] Building C object CMakeFiles/MPC13.dir/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC13.c.o
Elapsed time: 27 s. (time), 0.000705 s. (clock)
[ 35%] Linking C shared library lib/libMPC10.so
[ 35%] Built target MPC10
[ 40%] Building C object CMakeFiles/MPC14.dir/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC14.c.o
Elapsed time: 27 s. (time), 0.000877 s. (clock)
[ 45%] Linking C shared library lib/libMPC13.so
[ 45%] Built target MPC13
[ 50%] Building C object CMakeFiles/MPC6.dir/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC6.c.o
Elapsed time: 67 s. (time), 0.000887 s. (clock)
[ 55%] Linking C shared library lib/libMPC11.so
[ 55%] Built target MPC11
[ 60%] Building C object CMakeFiles/MPC7.dir/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC7.c.o
Elapsed time: 74 s. (time), 0.000843 s. (clock)
[ 65%] Linking C shared library lib/libMPC14.so
[ 65%] Built target MPC14
[ 70%] Building C object CMakeFiles/MPC8.dir/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC8.c.o
Elapsed time: 24 s. (time), 0.000761 s. (clock)
[ 75%] Linking C shared library lib/libMPC8.so
[ 75%] Built target MPC8
[ 80%] Building C object CMakeFiles/MPC9.dir/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC9.c.o
Elapsed time: 21 s. (time), 0.000749 s. (clock)
[ 85%] Linking C shared library lib/libMPC9.so
[ 85%] Built target MPC9
Use make\ install to copy the files to /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/main_ros2/read_bin_data/../../s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
[ 85%] Built target display_message


Elapsed time: 280 s. (time), 0.000709 s. (clock)
[ 90%] Linking C shared library lib/libMPC01.so
[ 90%] Built target MPC01
Elapsed time: 259 s. (time), 0.00082 s. (clock)
[ 95%] Linking C shared library lib/libMPC6.so
[ 95%] Built target MPC6
Elapsed time: 348 s. (time), 0.00077 s. (clock)
[100%] Linking C shared library lib/libMPC7.so
[100%] Built target MPC7
