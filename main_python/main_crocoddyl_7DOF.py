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
from multiprocessing import shared_memory
import posix_ipc
import multiprocessing
sys.path.append(os.path.dirname(os.path.abspath('./utils_python')))
from utils_python.utils import *

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

param_traj_poly = {}
param_traj_poly['T_start'] = 0
param_traj_poly['T_poly'] = 4
param_traj_poly['T_end'] = 5

if use_data_from_simulink:
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
    shm_changed_semaphore = posix_ipc.Semaphore("/shm_changed_semaphore", posix_ipc.O_CREAT, initial_value=0)

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

# TODO: in init_crocoddyl die initiale trajektorie erstellen!

ddp, xs, us, xs_init_guess, us_init_guess, TCP_frame_id, \
N_traj, Ts, hasConverged, warn_cnt, MPC_traj_indices, N_solver_steps, \
simulate_model, next_init_guess_fun, mpc_settings, param_mpc_weight, \
transient_traj, param_traj, title_text =   \
    init_crocoddyl( x_k_ndof, robot_model, robot_data, robot_model_full, robot_data_full, traj_data,     \
                    traj_init_config, param_robot, param_traj_poly, TCP_frame_id)

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

# folderpath1 = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240916_meeting/"
# folderpath2 = "/home/rslstudent/Students/Emanuel/crocoddyl_html_files/"
# paths = [folderpath1, folderpath2]
# folderpath = next((path for path in paths if os.path.exists(path)), None)
folderpath="./main_ros2/nodejs_ros2_gui/public"

plot_name = 'robot_plots.html'
plot_file_path = os.path.join(folderpath, plot_name)

visualize_name = 'robot_visualization.html'
visualize_file_path = os.path.join(folderpath, visualize_name)

new_data_flag = False
run_loop = True
try:
    while run_loop and err_state == False:
        if use_data_from_simulink:
            # Read data
            shm_changed_semaphore.acquire()
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
                        # update mpc settings and problem
                        ddp, xs, us, xs_init_guess, us_init_guess, TCP_frame_id, \
                        N_traj, Ts, hasConverged, warn_cnt, MPC_traj_indices, N_solver_steps, \
                        simulate_model, next_init_guess_fun, mpc_settings, param_mpc_weight, \
                        transient_traj, param_traj, title_text =   \
                            init_crocoddyl( x_k_ndof, robot_model, robot_data, robot_model_full, robot_data_full, traj_data,     \
                                            traj_init_config, param_robot, param_traj_poly, TCP_frame_id)
                        
                        # pin.forwardKinematics(robot_model_full, robot_data_full, x_k_ndof[:n_dof])
                        # pin.updateFramePlacements(robot_model_full, robot_data_full)
                        # xe0 = robot_data_full.oMf[TCP_frame_id].translation
                        
                        p_d = transient_traj['p_d']
                        p_d_p = transient_traj['p_d_p']
                        p_d_pp = transient_traj['p_d_pp']
                        R_d = transient_traj['R_d']


                    run_flag = True
                    print()
                    print("MPC started by Simulink")

            if run_flag == True:
                if stop == 1:
                    print()
                    print("MPC stopped by Simulink")
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
                print("MPC reset by Simulink")
                data_from_python[:] = np.zeros(n_dof)
                data_from_simulink_reset[:] = 0
                run_flag = False
                start_solving = False

                if plot_sol:
                    def plot_sol_act():
                        subplot_data = calc_7dof_data(us, xs, TCP_frame_id, robot_model_full, robot_data_full, transient_traj, freq_per_Ta_step, param_robot)
                        plot_solution_7dof(subplot_data, plot_fig = False, save_plot=True, file_name=plot_file_path, matlab_import=False, reload_page=reload_page, title_text=title_text)
                    process = multiprocessing.Process(target=plot_sol_act)
                    process.start()

                if visualize_sol:
                    def vis_sol_act():
                        q_sol = xs[:, :n_dof]
                        visualize_robot(robot_model_full, robot_data_full, visual_model, TCP_frame_id,
                                        q_sol, transient_traj, Ts,
                                        frame_skip=1, create_html = True, html_name = visualize_file_path)
                    process = multiprocessing.Process(target=vis_sol_act)
                    process.start()
                
                freq_per_Ta_step = np.zeros(N_traj)
                i = 0
        if start_solving:
            measureSimu.tic()

            measureSolver.tic()
            hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)
            measureSolver.toc()

            warn_cnt, err_state = check_solver_status(warn_cnt, hasConverged, ddp, i, Ts, conv_max_limit=5)
            
            xs_init_guess, us_init_guess = next_init_guess_fun(ddp, nq, nx, robot_model, robot_data, mpc_settings, param_traj)

            u_k = ddp.us[0]

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
                run_loop = False

            ##################################################################################
            ################ Set new data for the next optimization problem ##################
            ##################################################################################
            if use_gravity:
                g_k = pin.computeGeneralizedGravity(robot_model, robot_data, x_k[:nq])
            else:
                g_k = np.zeros(nq)

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
            
            xs_init_guess_prev = xs_init_guess
            if i==0:
                us_init_guess_prev = g_k * np.ones((mpc_settings['N_MPC']-1, nu))
            else:
                us_init_guess_prev = us_init_guess

            # v2: update reference values
            for j, runningModel in enumerate(ddp.problem.runningModels):
                if j > 0:
                    ddp.problem.runningModels[j].differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j]]
                    if(nq >= 6):
                        ddp.problem.runningModels[j].differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j]]
                
                if(param_mpc_weight['q_pp_common_weight'] > 0):
                    ddp.problem.runningModels[j].differential.costs.costs["q_ppReg"].cost.residual.reference = xs_init_guess_prev[j] # because qpp is calculated approximated
                if(param_mpc_weight['q_xprev_common_weight'] > 0):
                    ddp.problem.runningModels[j].differential.costs.costs["xprevReg"].cost.residual.reference = xs_init_guess_prev[j]
                # ddp.problem.runningModels[j].differential.costs.costs["ctrlReg"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
                if(param_mpc_weight['q_uprev_cost'] > 0):
                    ddp.problem.runningModels[j].differential.costs.costs["ctrlPrev"].cost.residual.reference = us_init_guess_prev[j] #first us[0] is torque for gravity compensation
                # ddp.problem.runningModels[j].differential.costs.costs["ctrlRegBound"].cost.activation.bounds.lb = us_init_guess_prev[j] - 0.1
                # ddp.problem.runningModels[j].differential.costs.costs["ctrlRegBound"].cost.activation.bounds.ub = us_init_guess_prev[j] + 0.1

            ddp.problem.terminalModel.differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j+1]]
            if(nq >= 6):
                ddp.problem.terminalModel.differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j+1]]
            if(param_mpc_weight['q_pp_common_weight'] > 0):
                ddp.problem.terminalModel.differential.costs.costs["q_ppReg"].cost.residual.reference = xs_init_guess_prev[j+1] # because qpp is calculated approximated
            if(param_mpc_weight['q_xprev_common_weight'] > 0):
                ddp.problem.terminalModel.differential.costs.costs["xprevReg"].cost.residual.reference = xs_init_guess_prev[j+1]
            # ddp.problem.terminalModel.differential.costs.costs["ctrlReg"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
            if(param_mpc_weight['q_uprev_cost'] > 0):
                ddp.problem.terminalModel.differential.costs.costs["ctrlPrev"].cost.residual.reference = us_init_guess_prev[j] #first us[0] is torque for gravity compensation
            # ddp.problem.terminalModel.differential.costs.costs["ctrlRegBound"].cost.activation.bounds.lb = us_init_guess_prev[j] - 0.1
            # ddp.problem.terminalModel.differential.costs.costs["ctrlRegBound"].cost.activation.bounds.ub = us_init_guess_prev[j] + 0.1

            ddp.problem.x0 = x_k
            # v2 end

            # ddp = solver(problem)
            # ddp.setCallbacks([cro.CallbackVerbose()])
            
            elapsed_time = measureSimu.toc()
            freq_per_Ta_step[i] = 1/elapsed_time

            if (i+1) % update_interval == 0:
                print(f"{100 * (i+1)/N_traj:.2f} % | {measureTotal.get_time_str()} | {format_freq(freq_per_Ta_step[i], 2)}     ", end='\r')
except KeyboardInterrupt:
    print("\nFinish Solving!")

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

if visualize_sol is True:
    def vis_sol_fin():
        q_sol = xs[:, :n_dof]
        visualize_robot(robot_model_full, robot_data_full, visual_model, TCP_frame_id,
                        q_sol, transient_traj, Ts,
                        frame_skip=1, create_html = True, html_name = visualize_file_path)
    process = multiprocessing.Process(target=vis_sol_fin)
    process.start()

if plot_sol == True:# and err_state == False:
    def plot_sol_fin():
        subplot_data = calc_7dof_data(us, xs, TCP_frame_id, robot_model_full, robot_data_full, transient_traj, freq_per_Ta_step, param_robot)
        plot_solution_7dof(subplot_data, plot_fig = False, save_plot=True, file_name=plot_file_path, matlab_import=False, reload_page=reload_page, title_text=title_text)
    process = multiprocessing.Process(target=plot_sol_fin)
    process.start()

# print('Max error:       y - y_d = {:.2e}'.format(np.max(np.abs(e))), 'm')
# print('Max error:   y_p - y_d_p = {:.2e}'.format(np.max(np.abs(e_p))), 'm/s')
# print('Max error: y_pp - y_d_pp = {:.2e}'.format(np.max(np.abs(e_pp))), 'm/s^2')
#print("\nTotal cost:", ddp.cost)

# visualize_sol=False
# if visualize_sol==True:
#     visualize_sol_robot(robot, q, transient_traj, Ts, 3, 1)

if use_data_from_simulink:
    user_input = input("Do you want to clear the shared memory? (y/n): ").lower()
    if user_input == 'y':
        for name, config in shm_objects.items():
            # Create shared memory object
            shm_objects[name].close()
            shm_objects[name].unlink()
        shm_changed_semaphore.unlink()
        shm_changed_semaphore.close()
        print("Shared Memory freigegeben.")
    else:
        print("Shared Memory nicht freigegeben.")

if autostart_fr3:
    user_input = input("Do you want to enable brakes? (y/n): ").lower()
    if user_input == 'y':
        message = "stop"
        asyncio.run(send_message(message))