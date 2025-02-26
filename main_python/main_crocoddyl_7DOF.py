import os
import sys
import numpy as np
import crocoddyl as cro
import pinocchio as pin
import time
import csv
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import scipy.io as sio
import meshcat as mc
from multiprocessing import shared_memory
import posix_ipc
import multiprocessing
sys.path.append(os.path.dirname(os.path.abspath('./utils_python')))
from utils_python.utils import *

# set nice priority to highest value
    # if error do the command, after that logout login
    # sudo sed -i '/# End of file/i @realtime soft nice -20\n@realtime hard nice -20' /etc/security/limits.d/realtime.conf
os.nice(0)
start_server() # start plotting update server
start_node_data_logger()

autostart_fr3 = False

if autostart_fr3:
    message = "start"
    asyncio.run(send_message(message))

################################################ REALTIME ###############################################

use_data_from_simulink = True
manual_traj_select = 1
use_feedforward = True
use_clipping = False
use_gravity = False
visualize_sol = True
plot_sol=True
debounce_delay = 0.1
explicit_mpc = False

# use_multisolving = False
# n_proc = 1 # number of processes for parallel solving
# TODO! Es stellt sich aber noch die Frage, ob es tatsächlich etwas bringt.
# Dabei muss ich den Code hier überarbeiten, denn es müssen n_proc unabhängige
# ddp objekte in n_proc unabhängigen Prozessen, die auf n_proc Kernen laufen
# erstellt werden. Auch die Referenzwerte werden in den parallelen Prozessen
# geschrieben. Dadurch kommt es zu keinen Reduktionen in der Laufzeit. Allerdings
# wäre es viel einfacher, wenn ich eine Klasse dazu schrieben würde. Aber dann kann
# ich es gleich in C++ machen. Dort kann ich noch um einiges mehr Performance herausholen
# da dort auch das Referenzwerte schreiben viel schneller geht.

use_custom_trajectory = False

param_traj_poly = {}
if not use_custom_trajectory:
    param_traj_poly['T_start'] = 0
    param_traj_poly['T_poly'] = 5
    param_traj_poly['T_end'] = 5
else:
    param_traj_poly['T_start'] = 0
    param_traj_poly['T_poly'] = 10
    param_traj_poly['T_end'] = 10
    param_traj_poly['T_max_horizon'] = 2

if use_data_from_simulink or explicit_mpc:
    # only available for franka research 3 robot (n_dof = 7)

    shm_objects, shm_data = initialize_shared_memory()
    data_from_python = shm_data['data_from_python']
    data_from_python_valid = shm_data['data_from_python_valid']
    data_from_simulink = shm_data['data_from_simulink']
    data_from_simulink_valid = shm_data['data_from_simulink_valid']
    data_from_simulink_start = shm_data['data_from_simulink_start']
    data_from_simulink_reset = shm_data['data_from_simulink_reset']
    data_from_simulink_stop = shm_data['data_from_simulink_stop']
    data_from_simulink_traj_switch = shm_data['data_from_simulink_traj_switch']
    readonly_mode = shm_data['readonly_mode']
    read_traj_length = shm_data['read_traj_length']
    read_traj_data = shm_data['read_traj_data']
    read_frequency = shm_data['read_frequency']
    read_state_data = shm_data['read_state_data']
    read_control_data = shm_data['read_control_data']
    read_traj_data_full = shm_data['read_traj_data_full']
    read_frequency_full = shm_data['read_frequency_full']
    read_state_data_full = shm_data['read_state_data_full']
    read_control_data_full = shm_data['read_control_data_full']
    shm_changed_semaphore = posix_ipc.Semaphore("/shm_changed_semaphore", posix_ipc.O_CREAT, initial_value=0)
    data_logger_semaphore = posix_ipc.Semaphore("/data_logger_semaphore", posix_ipc.O_CREAT, initial_value=0)
    ros2_semaphore = posix_ipc.Semaphore("/ros2_semaphore", posix_ipc.O_CREAT, initial_value=0)

    # create objects for debouncing the simulink buttons:
    start_button = DebouncedButton(debounce_delay)
    reset_button = DebouncedButton(debounce_delay)
    stop_button = DebouncedButton(debounce_delay)

# robot_name = 'ur5e_6dof'
robot_name = 'fr3_6dof_no_hand'
fr3_kin_model = False

if robot_name == 'ur5e_6dof':
    n_dof = 6 # (input data from simulink are 7dof states q, qp)
    mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files/Meshes_ur5e')
    urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', 'ur5e.urdf')
    urdf_tcp_frame_name = 'ur5e_tcp'

    trajectory_data_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/ur5e_6dof/trajectory_data/param_traj_data.mat')
    trajectory_param_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/ur5e_6dof/trajectory_data/param_traj.mat')
    n_indices = np.array([0, 1, 2, 3, 4, 5]) # use all joints
    n_x_indices = np.hstack([n_indices, n_indices+6])

    q_0_ref = np.array([0, 0, 0, 0, 0, 0])
   
   # Create a list of joints to lock
    jointsToLock = []
elif robot_name == 'fr3_6dof_no_hand':
    n_dof = 7 # (input data from simulink are 7dof states q, qp)
    mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files/Meshes_FR3')
    if fr3_kin_model:
        urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', 'fr3_no_hand_7dof_kinematic_model.urdf')
    else:
        urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', 'fr3_no_hand_7dof.urdf')
    urdf_tcp_frame_name = 'fr3_link8_tcp'
    trajectory_data_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/fr3_no_hand_6dof/trajectory_data/param_traj_data.mat')
    trajectory_param_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/fr3_no_hand_6dof/trajectory_data/param_traj.mat')

    use_only_q2q4 = False
    if use_only_q2q4:
        n_indices = -1 + np.array([2, 4]) # list of used joints
    else:
        n_indices = -1 + np.array([1, 2, 4, 5, 6, 7])

    n_x_indices = np.hstack([n_indices, n_indices+7])

    q_0_ref = np.array([0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, np.pi/4])

    # Create a list of joints to lock (starts with joint 1)
    jointsToLockIndex = np.setdiff1d(np.arange(0,7), n_indices) # at first use 7dof Model (will be later reduced)
    jointsToLock = jointsToLock = [f"fr3_joint{i+1}" for i in jointsToLockIndex] # list of joints to lock
else:
    raise Exception("Unknown robot name '{}'".format(robot_name), "Available robots: 'ur5e_6dof', 'fr3_6dof_no_hand'")

n_indices_all = np.arange(0, n_dof)
n_indices_fixed = n_indices_all[~np.isin(n_indices_all, n_indices)]

#########################################################################################################
############################################ MPC Settings ###############################################
#########################################################################################################

# see utils_python/mpc_weights_crocoddyl.json

#########################################################################################################
######################################### Build Robot Model #############################################
#########################################################################################################

robot_model_full, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

robot_data_full = robot_model_full.createData()
# robot = pin.robot_wrapper.RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir)

# https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b_examples_e_reduced_model.html
# create reduced model:
 
if len(n_indices) != robot_model_full.nq:
    # Get the ID of all existing joints
    jointsToLockIDs = []
    for jn in jointsToLock:
        if robot_model_full.existJointName(jn):
            jointsToLockIDs.append(robot_model_full.getJointId(jn))
        else:
            print("Warning: joint " + str(jn) + " does not belong to the model!")
    
    # Set initial position of both fixed and revoulte joints
    initialJointConfig = q_0_ref
    # initialJointConfig[n_indices] = q_0

    robot_model = pin.buildReducedModel(robot_model_full, jointsToLockIDs, initialJointConfig)
else:
    robot_model = robot_model_full

nq = robot_model.nq
nx = 2*nq
nu = nq # fully actuated

# Gravity should be in -z direction
if use_gravity:
    gravity = [0, 0, -9.81]
else:
    gravity = [0, 0, 0]

robot_model.gravity.linear[:] = gravity
robot_model_full.gravity.linear[:] = gravity

robot_data = robot_model.createData()

TCP_frame_id = robot_model.getFrameId(urdf_tcp_frame_name)

# The model loaded from urdf (via pinicchio)
print(robot_model)

#########################################################################################################
################################## Get Trajectory from mat file #########################################
#########################################################################################################

traj_data_all = sio.loadmat(trajectory_data_mat_file)['param_traj_data']
traj_param_all = sio.loadmat(trajectory_param_mat_file)['param_traj']

# in case of ur5e_6dof
'''
traj_select 0: Wrist Singularity 1, Polynomial, Workspace
traj_select 1: Allsingularity, Polynomial, Workspace
traj_select 2: Ellbow Singularity 1, Polynomial, Workspace
traj_select 3: ELLBOW Singularity 2, Polynomial, Workspace
traj_select 4: ELLBOW Singularity 2, Polynomial, joint space
traj_select 5: Shoulder Sing 1, Polynomial, Workspace
traj_select 6: Shoulder Sing 2, Polynomial, Workspace
'''

# in case of fr3_6dof_no_hand
'''
traj_select 1: Wrist Singularity 1, Equilibrium, Workspace
traj_select 2: Wrist Singularity 1, Diff filt, Workspace
traj_select 3: Wrist Singularity 1, Polynomial, Workspace
traj_select 4: Wrist Singularity 1, Sinus, Workspace
traj_select 5: 2DOF Test Trajectory, Polynomial, joint space
traj_select 6: Sing test stretch arm out, Polynomial, joint space
traj_select 7: Sing test stretch arm in, Polynomial, joint space
'''

if use_data_from_simulink:
    if data_from_simulink_traj_switch[0] == 0:
        current_traj_select = 1
        data_from_simulink_traj_switch[0] = 1
    else:
        current_traj_select = data_from_simulink_traj_switch[0]
else:
    current_traj_select = manual_traj_select

#################################TEMP
# current_traj_select = manual_traj_select
# data_from_simulink_traj_switch[0] = current_traj_select
##########################
traj_data, traj_init_config = process_trajectory_data(current_traj_select-1, traj_data_all, traj_param_all)

q_0_ref = traj_init_config['q_0']

x_min = np.hstack([robot_model_full.lowerPositionLimit, -robot_model_full.velocityLimit])
x_max = np.hstack([robot_model_full.upperPositionLimit, robot_model_full.velocityLimit])
param_robot = {
    'n_dof': n_dof,
    'n_red': len(n_indices),
    'n_indices': n_indices,
    'n_indices_fixed': n_indices_fixed,
    'n_x_indices': n_x_indices,
    'q_0_ref': q_0_ref,
    'x_min': x_min,
    'x_max': x_max,
    'x_mean': (x_max + x_min)/2,
    'use_gravity': use_gravity,
}
#########################################################################################################
############################################# INIT MPC ##################################################
#########################################################################################################
# T_horizon = 45e-3
# N_MPC = 3
# for run in range(40):
x_k_ndof = np.zeros(2*n_dof)
x_k_ndof[:n_dof] = q_0_ref
# x_k_ndof[n_x_indices] = np.array([-4.71541765e-05, -7.70960138e-01, -2.35629353e+00, 8.63920206e-05, \
#                                 1.57111388e+00, 7.85625356e-01, 1.18528803e-04, 1.41223310e-03, \
#                                 4.93944513e-04, -9.78304970e-04, -2.07886725e-04, -3.13358760e-03])
# x_k_ndof[n_dof::] = 0 # weil er stillsteht, jegliche Geschwindigkeitsmessung ist noise!
x_k = x_k_ndof[n_x_indices]

pin.forwardKinematics(robot_model_full, robot_data_full, q_0_ref)
pin.updateFramePlacements(robot_model_full, robot_data_full)

p_d = np.array([0.6, 0, 0.6])
R_d = robot_data_full.oMf[TCP_frame_id].rotation

param_target = {
    'p_d': p_d,
    'R_d': R_d,
}

ddp, xs, us, xs_init_guess, us_init_guess, TCP_frame_id, \
N_traj, Ts, hasConverged, warn_cnt, MPC_traj_indices, N_solver_steps, \
simulate_model, next_init_guess_fun, mpc_settings, param_mpc_weight, \
transient_traj, param_traj, title_text =   \
    init_crocoddyl( x_k_ndof, robot_model, robot_data, robot_model_full, robot_data_full, traj_data,     \
                    traj_init_config, param_robot, param_traj_poly, TCP_frame_id, \
                    use_custom_trajectory, param_target)

tcp_pose_list, tcp_rot_list, xprev_list, ctrl_prev_list = init_reference_lists(ddp, param_mpc_weight)

# N_MPC = N_MPC + 1

p_d = transient_traj['p_d']
p_d_p = transient_traj['p_d_p']
p_d_pp = transient_traj['p_d_pp']
R_d = transient_traj['R_d']

tau_full = calculate_ndof_torque_with_feedforward(us_init_guess[0], x_k_ndof, robot_model_full, robot_data_full, n_dof, n_indices, fr3_kin_model)

# jointspace controller settings for fixed joints
K_d = np.diag([100, 200, 500, 200, 50, 50, 10])
D_d = np.sqrt(2*K_d)

K_d_fixed = K_d[n_indices_fixed][:, n_indices_fixed]
D_d_fixed = D_d[n_indices_fixed][:, n_indices_fixed]

q_0_ref_fixed = q_0_ref[n_indices_fixed]

if use_data_from_simulink:
    run_flag = False
    start_solving = False

    data_from_python[:] = 0
    data_from_python_valid[:] = 0 # normally, only ROS2 set it to zero
else:
    start_solving = True
    run_flag = True

err_state = False
conv_max_limit = 10
update_interval = N_traj//100

freq_per_Ta_step = np.zeros(N_traj)

reload_page=True

u_prev = 0

measureSolver = TicToc()
measureSimu = TicToc()
measureTotal = TicToc()
measureTotal.tic()
i = 0
tau_i_prev = np.zeros(n_dof)
init_cnt = 0
init_cnt_max = 100

# add arrays for multisolving
# cost_array = multiprocessing.Array('d', n_proc)
# u_opt_array = multiprocessing.Array('d', n_proc*nq)
# has_converged_array = multiprocessing.Array('i', n_proc)

# folderpath1 = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240916_meeting/"
# folderpath2 = "/home/rslstudent/Students/Emanuel/crocoddyl_html_files/"
# paths = [folderpath1, folderpath2]
# folderpath = next((path for path in paths if os.path.exists(path)), None)
folderpath="./main_ros2/nodejs_ros2_gui/public"

plot_name = 'robot_plots.html'
plot_file_path = os.path.abspath(os.path.join(folderpath, plot_name))

visualize_name = 'robot_visualization.html'
visualize_file_path = os.path.abspath(os.path.join(folderpath, visualize_name))

if explicit_mpc:
    explicit_flag = True
    start_solving = True
else:
    explicit_flag = False

process_plot = None
process_vis = None
new_data_flag = False
run_loop = True
try:
    while run_loop and err_state == False:
        if use_data_from_simulink and not explicit_flag:
            # Read data
            shm_changed_semaphore.acquire()
            data_logger_semaphore.release()

            if readonly_mode[0] == 0:
                if(data_from_simulink_valid[0] == 1):
                    data_from_simulink_valid[:] = 0
                    x_k_ndof = data_from_simulink.copy()
                    x_k = x_k_ndof[n_x_indices]
                    new_data_flag = True

                # Daten von Simulink lesen
                start = start_button.debounce(data_from_simulink_start[:])
                reset = reset_button.debounce(data_from_simulink_reset[:])
                stop = stop_button.debounce(data_from_simulink_stop[:])
                
                if run_flag == False:
                    data_from_python[:] = np.zeros(n_dof)
                    if start == 1 and reset == 0 and stop == 0 and new_data_flag:
                        new_traj_select = data_from_simulink_traj_switch[0]
                        data_from_simulink_start[:] = 0

                        # set initial values
                        tau_i_prev = np.zeros(n_dof)

                        # reset time measurement
                        measureSolver.reset()
                        measureSimu.reset()
                        measureTotal.reset()
                        measureTotal.tic()

                        # Selbst wenn die Messung zu beginn eine Jointgeschwindigkeit ausgibt, wird diese auf 0 gesetzt
                        # weil der Roboter zu beginn stillsteht und es damit nur noise ist
                        x_k_ndof[n_dof::] = 0
                        init_cnt = 0

                        data_from_python[:] = np.zeros(n_dof)

                        if i == N_traj-1:
                            i = 0

                        if current_traj_select != new_traj_select:
                            current_traj_select = new_traj_select
                            traj_data, traj_init_config = process_trajectory_data(current_traj_select-1, traj_data_all, traj_param_all)
                            print()
                            print(f'New Trajectory selected: {current_traj_select}')

                        if i == 0:
                            measureTotal.reset()
                            measureTotal.tic()
                            # update mpc settings and problem
                            ddp, xs, us, xs_init_guess, us_init_guess, TCP_frame_id, \
                            N_traj, Ts, hasConverged, warn_cnt, MPC_traj_indices, N_solver_steps, \
                            simulate_model, next_init_guess_fun, mpc_settings, param_mpc_weight, \
                            transient_traj, param_traj, title_text =   \
                                init_crocoddyl( x_k_ndof, robot_model, robot_data, robot_model_full, robot_data_full, traj_data, \
                                                traj_init_config, param_robot, param_traj_poly, TCP_frame_id, \
                                                use_custom_trajectory, param_target)
                            
                            tcp_pose_list, tcp_rot_list, xprev_list, ctrl_prev_list = init_reference_lists(ddp, param_mpc_weight)
                            
                            # pin.forwardKinematics(robot_model_full, robot_data_full, x_k_ndof[:n_dof])
                            # pin.updateFramePlacements(robot_model_full, robot_data_full)
                            # xe0 = robot_data_full.oMf[TCP_frame_id].translation
                            
                            p_d = transient_traj['p_d']
                            p_d_p = transient_traj['p_d_p']
                            p_d_pp = transient_traj['p_d_pp']
                            R_d = transient_traj['R_d']


                        run_flag = True
                        print()
                        print("MPC started (control mode).")

                if run_flag is True:
                    if stop == 1:
                        print()
                        print("MPC stopped (control mode).")
                        data_from_python[:] = np.zeros(n_dof)
                        data_from_simulink_stop[:] = 0
                        run_flag = False
                        start_solving = False
                    elif new_data_flag:
                        start_solving = True
                        new_data_flag = False
                    if start == 1:
                        data_from_simulink_start[:] = 0 # already started, ignore start signal

                if reset == 1:
                    print()
                    print("MPC reset (control mode).")
                    data_from_python[:] = np.zeros(n_dof)
                    data_from_simulink_reset[:] = 0
                    run_flag = False
                    start_solving = False

                    if i > 0:
                        if plot_sol:
                            if process_plot is not None and process_plot.is_alive():
                                process_plot.terminate()
                            def plot_sol_act():
                                subplot_data = calc_7dof_data(us, xs, TCP_frame_id, robot_model_full, robot_data_full, transient_traj, freq_per_Ta_step, param_robot)
                                plot_solution_7dof(subplot_data, plot_fig = False, save_plot=True, file_name=plot_file_path, matlab_import=False, reload_page=reload_page, title_text=title_text)
                            process_plot = multiprocessing.Process(target=plot_sol_act)
                            process_plot.daemon = True # this will kill the process if the main process is killed
                            process_plot.start()

                        if visualize_sol:
                            if process_vis is not None and process_vis.is_alive():
                                process_vis.terminate()
                            def vis_sol_act():
                                visualize_robot(robot_model_full, robot_data_full, visual_model, TCP_frame_id,
                                                    xs[:, :n_dof], transient_traj, Ts,
                                                    frame_skip=1, create_html = True, html_name = visualize_file_path)
                            process_vis = multiprocessing.Process(target=vis_sol_act)
                            process_vis.daemon = False # this will kill the process if the main process is killed
                            process_vis.start()
                    i = 0
            elif not explicit_mpc:
                # in this mode, all data (states and torques) are read out of the shared memory. This mode is only
                # used by CasaDi from ROS 2 for logging the data.
                start = data_from_simulink_start[:]
                reset = data_from_simulink_reset[:]
                stop = data_from_simulink_stop[:]
                
                if start == 1:
                    data_from_simulink_start[:] = 0
                    run_flag = True
                    print()
                    print('Start MPC (data logging mode).')

                if reset == 1:
                    i=0
                    data_from_simulink_reset[:] = 0
                    run_flag = False
                    print()
                    print('Reset MPC (data logging mode).')

                    # read data from shared memory
                    xs = read_state_data_full.reshape(-1, 2*n_dof).copy()
                    us = read_control_data_full.reshape(-1, n_dof).copy()
                    freq_per_Ta_step = read_frequency_full.copy()
                    indices = np.arange(0, 19 * N_traj, 19)
                    q_d_temp = read_traj_data_full[indices[:, None] + np.arange(9, 13)].T.copy()
                    R_d_temp = quat2rotm_vec(q_d_temp).copy()
                    
                    transient_traj = {
                        't': np.linspace(0, (N_traj - 1)*Ts, N_traj),
                        'N_traj': N_traj,
                        'p_d': read_traj_data_full[indices[:, None] + np.arange(3)].T,
                        'p_d_p': read_traj_data_full[indices[:, None] + np.arange(3, 6)].T,
                        'p_d_pp': read_traj_data_full[indices[:, None] + np.arange(6, 9)].T,
                        'R_d': R_d_temp,
                        'q_d': q_d_temp,
                        'omega_d': read_traj_data_full[indices[:, None] + np.arange(13, 16)].T,
                        'omega_d_p': read_traj_data_full[indices[:, None] + np.arange(16, 19)].T
                    }

                    read_state_data_full[:] = 0
                    read_control_data_full[:] = 0
                    read_frequency_full[:] = 0
                    read_traj_data_full[:] = 0

                    if plot_sol:
                        if process_plot is not None and process_plot.is_alive():
                            process_plot.terminate()
                        def plot_sol_act():
                            subplot_data = calc_7dof_data(us, xs, TCP_frame_id, robot_model_full, robot_data_full, transient_traj, freq_per_Ta_step, param_robot)
                            plot_solution_7dof(subplot_data, plot_fig = False, save_plot=True, file_name=plot_file_path, matlab_import=False, reload_page=reload_page, title_text=title_text)
                        process_plot = multiprocessing.Process(target=plot_sol_act)
                        process_plot.daemon = True # this will kill the process if the main process is killed
                        process_plot.start()

                    if visualize_sol:
                        if process_vis is not None and process_vis.is_alive():
                            process_vis.terminate()
                        def vis_sol_act():
                            visualize_robot(robot_model_full, robot_data_full, visual_model, TCP_frame_id,
                                                xs[:, :n_dof], transient_traj, Ts,
                                                frame_skip=1, create_html = True, html_name = visualize_file_path)
                        process_vis = multiprocessing.Process(target=vis_sol_act)
                        process_vis.daemon = False # this will kill the process if the main process is killed
                        process_vis.start()

                    xs = np.zeros((N_traj, 2*n_dof))
                    us = np.zeros((N_traj, n_dof))

                if stop == 1:
                    data_from_simulink_stop[:] = 0
                    run_flag = False
                    print()
                    print('Stop MPC (data logging mode).')

                if i == 0:
                    measureTotal.reset()
                    measureTotal.tic()
                    # reset data
                    freq_per_Ta_step = np.zeros(N_traj)
                    N_traj = read_traj_length[0]
                    xs = np.zeros((N_traj, 2*n_dof))
                    us = np.zeros((N_traj, n_dof))
                    transient_traj = {
                        't': np.linspace(0, (N_traj - 1)*Ts, N_traj),
                        'N_traj': N_traj,
                        'p_d': np.zeros((3, N_traj)),
                        'p_d_p': np.zeros((3, N_traj)),
                        'p_d_pp': np.zeros((3, N_traj)),
                        'R_d': np.zeros((3, 3, N_traj)),
                        'q_d': np.zeros((4, N_traj)),
                        'omega_d': np.zeros((3, N_traj)),
                        'omega_d_p': np.zeros((3, N_traj))
                    }

                if run_flag is True:
                    if i < N_traj-1:
                        i += 1
                    else:
                        run_flag = False

                    if (i+1) % update_interval == 0:
                        print(f"{100 * (i+1)/N_traj:.2f} % | {measureTotal.get_time_str()} | {format_freq(read_frequency_full[i-1], 2)}     ", end='\r')


        if start_solving:
            if use_gravity:
                g_k = pin.computeGeneralizedGravity(robot_model, robot_data, x_k[:nq])
            else:
                g_k = np.zeros(nq)
            
            xs_init_guess_prev = xs_init_guess
            if i==0:
                us_init_guess_prev = g_k * np.ones((mpc_settings['N_MPC'], nu))
            else:
                us_init_guess_prev = us_init_guess

            for j, runningModel in enumerate(ddp.problem.runningModels):
                tcp_pose_list[j].reference = p_d[:, i+MPC_traj_indices[j]]
                tcp_rot_list[j].reference = R_d[:, :, i+MPC_traj_indices[j]]
                if(param_mpc_weight['q_xprev_common_weight'] > 0):
                    xprev_list[j].reference = xs_init_guess_prev[j]
                if(param_mpc_weight['q_uprev_cost'] > 0):
                    ctrl_prev_list[j].reference = us_init_guess_prev[j]
            tcp_pose_list[-1].reference = p_d[:, i+MPC_traj_indices[j+1]]
            tcp_rot_list[-1].reference = R_d[:, :, i+MPC_traj_indices[j+1]]
            if(param_mpc_weight['q_xprev_common_weight'] > 0):
                xprev_list[-1].reference = xs_init_guess_prev[j+1]
            if(param_mpc_weight['q_uprev_cost'] > 0):
                ctrl_prev_list[-1].reference = us_init_guess_prev[j+1]


            measureSimu.tic()

            measureSolver.tic()
            hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)
            u_k = ddp.us[0]
            measureSolver.toc()

            warn_cnt, err_state = check_solver_status(warn_cnt, hasConverged, ddp, i, Ts, conv_max_limit=5)
            
            xs_init_guess, us_init_guess = next_init_guess_fun(ddp, nq, nx, robot_model, robot_data, mpc_settings, param_traj)

            if use_clipping:
                du_max = 1 # Nm/ms
                du = u_k - u_prev
                mask_upper = du > du_max
                mask_lower = du < -du_max
                # Wende die Beschneidung an
                u_k = np.where(mask_upper, u_prev + du_max, u_k)
                u_k = np.where(mask_lower, u_prev - du_max, u_k)

                # Erzeuge die Flag
                if np.any(mask_upper | mask_lower):
                    print("u_k was clipped")

            # use feedforward for fixed joints
            if use_feedforward:
                tau_full = calculate_ndof_torque_with_feedforward(u_k, x_k_ndof, robot_model_full, robot_data_full, n_dof, n_indices, fr3_kin_model)
            else:
                tau_full = np.zeros(n_dof)
                tau_full[n_indices] = u_k

            # use pd control for fixed joints
            q_ndof = x_k_ndof[:n_dof]
            q_p_ndof = x_k_ndof[n_dof::]

            q_fixed = q_ndof[n_indices_fixed]
            q_p_fixed = q_p_ndof[n_indices_fixed]

            tau_fixed = -K_d_fixed @ (q_fixed - q_0_ref_fixed) - D_d_fixed @ q_p_fixed

            tau_full[n_indices_fixed] = tau_full[n_indices_fixed] + tau_fixed

            xs[i] = x_k_ndof
            us[i] = tau_full

            if use_data_from_simulink:
                if not err_state:
                    # check for jumps in torque
                    delta_u = tau_full - tau_i_prev
                    condition1 = np.logical_and(delta_u > 0, delta_u > 5)
                    condition2 = np.logical_and(delta_u < 0, delta_u < -5)

                    if np.any(np.logical_or(condition1, condition2)):
                        print()
                        print('tau_i_prev')
                        print(tau_i_prev)
                        print('tau_i_now')
                        print(tau_full)
                        print("Jump in torque (> 5 Nm/ms) detected, output zero torque")
                        data_from_python[:] = np.zeros(n_dof)
                        run_flag = False
                    else:
                        # Daten von Python an Simulink schreiben
                        data_from_python[:] = tau_full
                else:
                    data_from_python[:] = np.zeros(n_dof)
                    run_flag = False
                    err_state = False # damit python nicht crasht

                data_from_python_valid[:] = 1
                start_solving = False
                tau_i_prev = tau_full
            else:             
                x_kp1_ndof = simulate_model(x_k_ndof, tau_full, Ts, n_dof, 2*n_dof, robot_model_full, robot_data_full, param_robot, transient_traj)
                
                #### testing reduced model
                # q_red = q_ndof[n_indices]
                # q_p_red = q_p_ndof[n_indices]
                # x_k = np.hstack([q_red, q_p_red])
                
                # x_kp1_red = simulate_model(x_k, u_k, Ts, 6, 2*6, robot_model, robot_data, param_robot, transient_traj)
                # x_k_i_ndof = np.zeros(2*n_dof)
                # x_k_i_ndof[n_x_indices] = x_k
                
                # x_kp1_ndof = np.zeros(2*n_dof)
                # x_kp1_ndof[n_x_indices] = x_kp1_red
                # u_k_i_ndof = np.zeros(7)
                # u_k_i_ndof[n_indices] = u_k
                # xs[i] = x_k_i_ndof
                # us[i] = u_k_i_ndof    
                ####

                x_k_ndof = x_kp1_ndof
                x_k = x_k_ndof[n_x_indices]

            if i < N_traj-1 and run_flag == True:
                i += 1 # last value of i is N_traj-1
            
            if i == N_traj-1 and not use_data_from_simulink:
                if explicit_flag:
                    explicit_flag = False
                    i = 0
                    xs_ref = xs
                    us_ref = us
                    use_data_from_simulink = True
                else:
                    run_loop = False

            ##################################################################################
            ################ Set new data for the next optimization problem ##################
            ##################################################################################


            # if mpc_settings['version'] == 'MPC_v3_bounds_yN_ref':
            #     xs_init_guess_prev = np.array(xs_init_guess)
            #     xs_init_guess_model2 = xs_init_guess_prev[:, 6::]
            #     xs_init_guess_model2[:,nq:nx] = 0

            #     # v2: update reference values
            #     for j, runningModel in enumerate(ddp.problem.runningModels):
            #         ddp.problem.runningModels[j].model1.differential.y_d = p_d[:, i+MPC_traj_indices[j]]
            #         ddp.problem.runningModels[j].model1.differential.y_d_p = p_d_p[:, i+MPC_traj_indices[j]]
            #         ddp.problem.runningModels[j].model1.differential.y_d_pp = p_d_pp[:, i+MPC_traj_indices[j]]

            #         ddp.problem.runningModels[j].model2.differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j]]
            #         if(nq >= 6):
            #             ddp.problem.runningModels[j].model2.differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j]]
            #         ddp.problem.runningModels[j].model2.differential.costs.costs["stateReg"].cost.residual.reference = xs_init_guess_model2[j]
            #         ddp.problem.runningModels[j].model2.differential.costs.costs["stateRegBound"].cost.residual.reference = xs_init_guess_model2[j]
            #         ddp.problem.runningModels[j].model2.differential.costs.costs["ctrlReg"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
            #         ddp.problem.runningModels[j].model2.differential.costs.costs["ctrlRegBound"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation

            #     ddp.problem.terminalModel.model2.differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j+1]]
            #     if(nq >= 6):
            #         ddp.problem.terminalModel.model2.differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j+1]]
            #     ddp.problem.terminalModel.model2.differential.costs.costs["stateReg"].cost.residual.reference = xs_init_guess_model2[j+1]
            #     ddp.problem.terminalModel.model2.differential.costs.costs["stateRegBound"].cost.residual.reference = xs_init_guess_model2[j+1]
            #     ddp.problem.terminalModel.model2.differential.costs.costs["ctrlReg"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
            #     ddp.problem.terminalModel.model2.differential.costs.costs["ctrlRegBound"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
                
            #     x0_new = x_k
            #     x0_new[:6] = xs_init_guess_prev[1, :6]
            #     ddp.problem.x0 = x0_new
            # else:

            

            # # v2: update reference values
            # for j, runningModel in enumerate(ddp.problem.runningModels):
            #     if j > 0:
            #         ddp.problem.runningModels[j].differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j]]
            #         if(nq >= 6):
            #             ddp.problem.runningModels[j].differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j]]
                
            #     # if(param_mpc_weight['q_pp_common_weight'] > 0):
            #     #     ddp.problem.runningModels[j].differential.costs.costs["q_ppReg"].cost.residual.reference = np.zeros(nq) # because qpp is calculated approximated
            #     if(param_mpc_weight['q_xprev_common_weight'] > 0):
            #         ddp.problem.runningModels[j].differential.costs.costs["xprevReg"].cost.residual.reference = xs_init_guess_prev[j]
            #     # ddp.problem.runningModels[j].differential.costs.costs["ctrlReg"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
            #     if(param_mpc_weight['q_uprev_cost'] > 0):
            #         ddp.problem.runningModels[j].differential.costs.costs["ctrlPrev"].cost.residual.reference = us_init_guess_prev[j] #first us[0] is torque for gravity compensation
            #     # ddp.problem.runningModels[j].differential.costs.costs["ctrlRegBound"].cost.activation.bounds.lb = us_init_guess_prev[j] - 0.1
            #     # ddp.problem.runningModels[j].differential.costs.costs["ctrlRegBound"].cost.activation.bounds.ub = us_init_guess_prev[j] + 0.1

            # ddp.problem.terminalModel.differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j+1]]
            # if(nq >= 6):
            #     ddp.problem.terminalModel.differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j+1]]
            # # if(param_mpc_weight['q_pp_common_weight'] > 0):
            # #     ddp.problem.terminalModel.differential.costs.costs["q_ppReg"].cost.residual.reference = np.zeros(nq) # because qpp is calculated approximated
            # if(param_mpc_weight['q_xprev_common_weight'] > 0):
            #     ddp.problem.terminalModel.differential.costs.costs["xprevReg"].cost.residual.reference = xs_init_guess_prev[j+1]
            # # ddp.problem.terminalModel.differential.costs.costs["ctrlReg"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
            # if(param_mpc_weight['q_uprev_cost'] > 0):
            #     ddp.problem.terminalModel.differential.costs.costs["ctrlPrev"].cost.residual.reference = us_init_guess_prev[j] #first us[0] is torque for gravity compensation
            # # ddp.problem.terminalModel.differential.costs.costs["ctrlRegBound"].cost.activation.bounds.lb = us_init_guess_prev[j] - 0.1
            # # ddp.problem.terminalModel.differential.costs.costs["ctrlRegBound"].cost.activation.bounds.ub = us_init_guess_prev[j] + 0.1

            ddp.problem.x0 = x_k
            # v2 end

            # ddp = solver(problem)
            # ddp.setCallbacks([cro.CallbackVerbose()])
            
            elapsed_time = measureSimu.toc()

            if i<N_traj-1:
                freq_per_Ta_step[i] = 1/elapsed_time
            else:
                freq_per_Ta_step[N_traj-1] = 1/measureTotal.toc()

            if (i+1) % update_interval == 0:
                print(f"{100 * (i+1)/N_traj:.2f} % | {measureTotal.get_time_str()} | {format_freq(freq_per_Ta_step[i], 2)}     ", end='\r')
except KeyboardInterrupt:
    print("\nFinish Solving!")

try:
    if err_state:
        print("Error Occured, output zero torque:", hasConverged)
        if use_data_from_simulink:
            data_from_python[:] = np.zeros(n_dof)


    xs[N_traj-1] = x_k_ndof
    us[N_traj-1] = us[N_traj-2] # simply use previous value for display (does not exist)

    sol_time = measureSolver.print_time(additional_text='Total Solver time')
    tot_time = measureTotal.print_time(additional_text='Total MPC time')


    #########################################################################################################
    ############################################# Plot Results ##############################################
    #########################################################################################################

    if mpc_settings['version'] == 'MPC_v3_bounds_yN_ref':
        us = us[:, 3::]
        xs = xs[:, 6::]


    if plot_sol == True:# and err_state == False:
        subplot_data = calc_7dof_data(us, xs, TCP_frame_id, robot_model_full, robot_data_full, transient_traj, freq_per_Ta_step, param_robot)
        plot_solution_7dof(subplot_data, plot_fig = False, save_plot=True, file_name=plot_file_path, matlab_import=False, reload_page=reload_page, title_text=title_text)

    if visualize_sol is True:
        tic = time.time()
        q_sol = xs[:, :n_dof]
        visualize_robot(robot_model_full, robot_data_full, visual_model, TCP_frame_id,
                        q_sol, transient_traj, Ts,
                        frame_skip=1, create_html = True, html_name = visualize_file_path)
        print(f"Visualization time: {time.time()-tic:.2f} s")

    if use_data_from_simulink:
        user_input = input("Do you want to clear the shared memory? (y/n): ").lower()
        if user_input == 'y':
            for name, config in shm_objects.items():
                # Create shared memory object
                shm_objects[name].close()
                shm_objects[name].unlink()
            shm_changed_semaphore.unlink()
            shm_changed_semaphore.close()
            data_logger_semaphore.unlink()
            data_logger_semaphore.close()
            ros2_semaphore.unlink()
            ros2_semaphore.close()
            print("Shared memory closed and cleared.")
        else:
            print("Shared memory not cleared.")

    if autostart_fr3:
        user_input = input("Do you want to enable brakes? (y/n): ").lower()
        if user_input == 'y':
            message = "stop"
            asyncio.run(send_message(message))
except KeyboardInterrupt:
    print("Interrupted by user. Shared Memory not closed and cleared.")