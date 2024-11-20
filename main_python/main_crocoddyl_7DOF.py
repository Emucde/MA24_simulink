import os
import sys
import numpy as np
import crocoddyl as cro
import pinocchio as pin
import time
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import scipy.io as sio
from multiprocessing import shared_memory
import asyncio
import websockets

sys.path.append(os.path.dirname(os.path.abspath('./utils_python')))
from utils_python.utils import *

async def send_message(message):
    async with websockets.connect("ws://localhost:8765") as websocket:
        await websocket.send(message)
        print(f"Nachricht gesendet: {message}")

autostart_fr3 = False

if autostart_fr3:
    message = "start"
    asyncio.run(send_message(message))

################################################ REALTIME ###############################################

use_data_from_simulink = False
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


# robot_name = 'ur5e_6dof'
robot_name = 'fr3_6dof_no_hand'

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

param_robot = {
    'n_dof': n_dof,
    'n_red': len(n_indices),
    'n_indices': n_indices,
    'n_indices_fixed': n_indices_fixed,
    'n_x_indices': n_x_indices,
    'q_0_ref': q_0_ref
}

#########################################################################################################
############################################ MPC Settings ###############################################
#########################################################################################################

# see utils_python/mpc_weights_crocoddyl.json

#########################################################################################################
######################################### Build Robot Model #############################################
#########################################################################################################

robot_model_full, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

robot_data_full = robot_model_full.createData()

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
use_gravity = False
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

curent_traj_select = 3
traj_data, traj_data_true, traj_init_config = process_trajectory_data(curent_traj_select-1, traj_data_all, traj_param_all)

#########################################################################################################
############################################# INIT MPC ##################################################
#########################################################################################################

ddp, x_k, xs, us, xs_init_guess, us_init_guess, y_d_data, t, TCP_frame_id, \
N_traj, Ts, hasConverged, warn_cnt, MPC_traj_indices, N_solver_steps, \
simulate_model, next_init_guess_fun, mpc_settings, param_traj =   \
    init_crocoddyl( robot_model, robot_data, traj_data, traj_data_true,     \
                    traj_init_config, param_robot, TCP_frame_id)

p_d = y_d_data['p_d']
p_d_p = y_d_data['p_d_p']
p_d_pp = y_d_data['p_d_pp']
R_d = y_d_data['R_d']


q_0_ref = traj_init_config['q_0']
x_k_ndof = np.zeros(2*n_dof)
x_k_ndof[:n_dof] = q_0_ref

tau_full = calculate_ndof_torque_with_feedforward(us_init_guess[0], x_k_ndof, robot_model_full, robot_data_full, n_dof, n_indices, n_indices_fixed)

if use_data_from_simulink:
    run_flag = False
    start_solving = False

    data_from_python[:] = tau_full
    data_from_python_valid[:] = 1
else:
    start_solving = True
    run_flag = True


err_state = False
conv_max_limit = 10
update_interval = N_traj//100

freq_per_Ta_step = np.zeros(N_traj)

measureSolver = TicToc()
measureSimu = TicToc()
measureTotal = TicToc()
measureTotal.tic()
i = 0
folderpath = "/home/rslstudent/Students/Emanuel/crocoddyl_html_files/"
outputname = 'test.html'
output_file_path = os.path.join(folderpath, outputname)
run_loop = True
try:
    while run_loop and err_state == False:
        if use_data_from_simulink:
            if data_from_python_valid[0] == 1:
                data_from_python_valid[:] = 0
            # Daten von Simulink lesen
            start = data_from_simulink_start[:]
            reset = data_from_simulink_reset[:]
            stop = data_from_simulink_stop[:]
            new_traj_select = data_from_simulink_traj_switch[0]
            
            if run_flag == False:
                start_solving = False
                data_from_python[:] = np.zeros(n_dof)
                if start == 1 and reset == 0 and stop == 0:
                    data_from_python[:] = np.zeros(n_dof)

                    if i == N_traj-1:
                        i = 0

                    if curent_traj_select != new_traj_select:
                        curent_traj_select = new_traj_select
                        traj_data, traj_data_true, traj_init_config = process_trajectory_data(curent_traj_select-1, traj_data_all, traj_param_all)
                        print(f'New Trajectory selected: {curent_traj_select}')

                    # update mpc settings and problem
                    ddp, x_k, xs, us, xs_init_guess, us_init_guess, y_d_data, t, TCP_frame_id, \
                    N_traj, Ts, hasConverged, warn_cnt, MPC_traj_indices, N_solver_steps, \
                    simulate_model, next_init_guess_fun, mpc_settings, param_traj =   \
                        init_crocoddyl( robot_model, robot_data, traj_data, traj_data_true,     \
                                        traj_init_config, param_robot, TCP_frame_id)
                    
                    p_d = y_d_data['p_d']
                    p_d_p = y_d_data['p_d_p']
                    p_d_pp = y_d_data['p_d_pp']
                    R_d = y_d_data['R_d']

                    run_flag = True
                    print("MPC started by Simulink")
                elif reset == 1:# and i == N_traj-1:
                    i = 0
                    
                    subplot_data = calc_7dof_data(us, xs, t, TCP_frame_id, robot_model, robot_data, traj_data_true, freq_per_Ta_step)
                    plot_solution_7dof(subplot_data, plot_fig = False, save_plot=True, file_name=output_file_path, matlab_import=False)
                    data_from_simulink_reset[:] = 0
                    print("MPC reset by Simulink")
            elif run_flag == True and stop == 1:
                print("MPC stopped by Simulink")
                data_from_python[:] = np.zeros(n_dof)
                data_from_simulink_stop[:] = 0
                run_flag = False
                start_solving = False
            elif run_flag == True and reset == 1:
                print("MPC reset by Simulink")
                data_from_python[:] = np.zeros(n_dof)
                data_from_simulink_reset[:] = 0
                run_flag = False
                start_solving = False
                i = 0

            if(data_from_simulink_valid[:] == 1 and run_flag == True):
                data_from_simulink_valid[:] = 0
                data_from_python_valid[:] = 0
                x_k_ndof = data_from_simulink.copy()
                x_k = x_k_ndof[n_x_indices]
                start_solving = True
        if start_solving:
            measureSimu.tic()
            
            g_k = pin.computeGeneralizedGravity(robot_model, robot_data, x_k[:nq])

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
            xs_init_guess_prev = np.array(xs_init_guess)
            us_init_guess_prev = np.array(us_init_guess)
            xs_init_guess_prev[:,nq:nx] = 0
            # v2: update reference values
            for j, runningModel in enumerate(ddp.problem.runningModels):
                ddp.problem.runningModels[j].differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j]]
                if(nq >= 6):
                    ddp.problem.runningModels[j].differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j]]
                ddp.problem.runningModels[j].differential.costs.costs["stateReg"].cost.residual.reference = xs_init_guess_prev[j]
                ddp.problem.runningModels[j].differential.costs.costs["stateRegBound"].cost.residual.reference = xs_init_guess_prev[j]
                ddp.problem.runningModels[j].differential.costs.costs["ctrlReg"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
                ddp.problem.runningModels[j].differential.costs.costs["ctrlPrev"].cost.residual.reference = us_init_guess_prev[j] #first us[0] is torque for gravity compensation
                ddp.problem.runningModels[j].differential.costs.costs["ctrlRegBound"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation

            ddp.problem.terminalModel.differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j+1]]
            if(nq >= 6):
                ddp.problem.terminalModel.differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j+1]]
            ddp.problem.terminalModel.differential.costs.costs["stateReg"].cost.residual.reference = xs_init_guess_prev[j+1]
            ddp.problem.terminalModel.differential.costs.costs["stateRegBound"].cost.residual.reference = xs_init_guess_prev[j+1]
            ddp.problem.terminalModel.differential.costs.costs["ctrlReg"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
            ddp.problem.terminalModel.differential.costs.costs["ctrlPrev"].cost.residual.reference = us_init_guess_prev[j] #first us[0] is torque for gravity compensation
            ddp.problem.terminalModel.differential.costs.costs["ctrlRegBound"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation

            ddp.problem.x0 = x_k
            # v2 end

            # ddp = solver(problem)
            # ddp.setCallbacks([cro.CallbackVerbose()])

            measureSolver.tic()
            hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)
            measureSolver.toc()
            

            warn_cnt, err_state = check_solver_status(warn_cnt, hasConverged, ddp, i, Ts, conv_max_limit=5)

            if use_data_from_simulink:
                if not err_state:
                    xs_init_guess = ddp.xs
                    us_init_guess = ddp.us

                    u_k = ddp.us[0]

                    tau_full = calculate_ndof_torque_with_feedforward(u_k, x_k_ndof, robot_model_full, robot_data_full, n_dof, n_indices, n_indices_fixed)
                    
                    xs[i] = x_k_ndof
                    us[i] = tau_full
                    # tau_full = np.zeros(n_dof)
                    # tau_full[n_indices] = u_k

                    # Daten von Python an Simulink schreiben

                    if i == 0:
                        tau_i_prev = tau_full

                    delta_u = tau_full - tau_i_prev
                    condition1 = np.logical_and(delta_u > 0, delta_u > 2)
                    condition2 = np.logical_and(delta_u < 0, delta_u < -2)

                    if np.any(np.logical_or(condition1, condition2)): # aber kleiner als -0.8 ist ok
                        print(tau_full)
                        print("Jump in torque (> 5 Nm/ms) detected, output zero torque")
                        data_from_python[:] = np.zeros(n_dof)
                        if data_from_python_valid[0] == 1:
                            data_from_python_valid[:] = 0
                        run_flag = False
                    else:
                        data_from_python[:] = tau_full
                        if data_from_python_valid[0] == 0:
                            data_from_python_valid[:] = 1
                else:
                    data_from_python[:] = np.zeros(n_dof)
                    data_from_python_valid[:] = 0
                    run_flag = False
                    err_state = False # damit python nicht crasht

                start_solving = False

                tau_i_prev = tau_full
            else:
                u_k = ddp.us[0]
                tau_full = calculate_ndof_torque_with_feedforward(u_k, x_k_ndof, robot_model_full, robot_data_full, n_dof, n_indices, n_indices_fixed)
                
                K_d = np.diag([100, 200, 500, 200, 50, 50, 10])
                D_d = np.sqrt(2*K_d)
                
                K_d_fixed = K_d[n_indices_fixed][:, n_indices_fixed]
                D_d_fixed = D_d[n_indices_fixed][:, n_indices_fixed]
                
                q_ndof = x_k_ndof[:n_dof]
                q_p_ndof = x_k_ndof[n_dof::]

                q_fixed = q_ndof[n_indices_fixed]
                q_p_fixed = q_p_ndof[n_indices_fixed]

                tau_fixed = -K_d_fixed @ (q_fixed - q_0_ref[n_indices_fixed]) - D_d_fixed @ q_p_fixed

                tau_full[n_indices_fixed] = tau_full[n_indices_fixed] + tau_fixed

                # tau_full = np.zeros(n_dof)
                # tau_full[n_indices] = u_k
                
                x_kp1_ndof, x_k_i_ndof, u_k_i_ndof = simulate_model(x_k_ndof, tau_full, Ts, n_dof, 2*n_dof, robot_model_full, robot_data_full, traj_data)

                xs[i] = x_k_i_ndof
                us[i] = u_k_i_ndof

                x_k_ndof = x_kp1_ndof
                x_k = x_k_ndof[n_x_indices]

                xs_init_guess, us_init_guess = next_init_guess_fun(ddp, nq, nx, robot_model, robot_data, mpc_settings, param_traj)
                # alternative: Use only solver values (perfect tracking)
                # xs[i] = ddp.xs[0] # muss so sein, da x0 in ddp.xs[0] gespeichert ist
                # us[i] = ddp.us[0]

                # ddp.xs[0:-1] = ddp.xs[1::] # Problem: geht nur wenn alle werte in Ta abstand (TsMPC ist aber > Ta)
                # ddp.us[0:-1] = ddp.us[1::] # fehler da TsMPC > Ta ist.

                # xs_init_guess = ddp.xs
                # us_init_guess = ddp.us

                # x_k = ddp.xs[1]

            if (i+1) % update_interval == 0:
                print(f"{100 * (i+1)/N_traj:.2f} % | {measureTotal.get_time_str()}    ", end='\r')

            if i < N_traj-1 and run_flag == True:
                i += 1 # last value of i is N_traj-1
                # in case of use_data_from_simulink == True, the last trajectory value
                # is used for all further calculations (stay on the last position)
            # else:
            #     run_flag = False
            
            if i == N_traj-1 and use_data_from_simulink == False:
                run_loop = False
        
            elapsed_time = measureSimu.toc()
            freq_per_Ta_step[i] = 1/elapsed_time
except KeyboardInterrupt:
    print("\nFinish Solving!")

if err_state:
    print("Error Occured, output zero torque:", hasConverged)
    if use_data_from_simulink:
        data_from_python[:] = np.zeros(n_dof)
        data_from_python_valid[:] = 0
else:
    print("Minimum Found:", hasConverged)


xs[N_traj-1] = x_k_ndof
us[N_traj-1] = us[N_traj-2] # simply use previous value for display (does not exist)

measureSolver.print_time(additional_text='Total Solver time')
measureTotal.print_time(additional_text='Total MPC time')


#########################################################################################################
############################################# Plot Results ##############################################
#########################################################################################################

if mpc_settings['version'] == 'MPC_v3_bounds_yN_ref':
    us = us[:, 3::]
    xs = xs[:, 6::]

plot_full_model = True

if plot_full_model:
    # robot_data_full = robot_model_full.createData()

    # n_full = len(q_0_ref)
    # x_indiced = np.hstack([n_indices, n_indices+n_full])
    # us_full = np.zeros((N_traj,   n_full))
    # xs_full = np.zeros((N_traj, 2*n_full))
    # x_i = np.hstack([q_0_ref, np.zeros(n_full)])
    # for i in range(N_traj):
    #     q = x_i[:n_full]
    #     q_p = x_i[n_full::]

    #     x_i[x_indiced] = xs[i]

    #     q_red = xs[i][:nq]
    #     q_p_red = xs[i][nq::]
    #     u_red = us[i]

    #     q_pp_red = pin.aba(robot_model, robot_data, q_red, q_p_red, u_red)
    #     q_pp = np.zeros(n_full)
    #     q_pp[n_indices] = q_pp_red

    #     us_full[i] = pin.rnea(robot_model_full, robot_data_full, q, q_p, q_pp)

    #     # us_full[i, n_indices] = us[i]
    #     xs_full[i] = x_i
    
    # subplot_data = calc_7dof_data(us_full, xs_full, t, TCP_frame_id, robot_model_full, robot_data_full, traj_data_true, freq_per_Ta_step)
    subplot_data = calc_7dof_data(us, xs, t, TCP_frame_id, robot_model_full, robot_data_full, traj_data_true, freq_per_Ta_step, param_robot)
else:
    subplot_data = calc_7dof_data(us, xs, t, TCP_frame_id, robot_model, robot_data, traj_data_true, freq_per_Ta_step, param_robot)


# folderpath = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240916_meeting/"
folderpath = "/home/rslstudent/Students/Emanuel/crocoddyl_html_files/"
outputname = '240910_traj2_crocoddyl_T_horizon_25ms.html'
output_file_path = os.path.join(folderpath, outputname)

plot_sol=False
if plot_sol == True or use_data_from_simulink == False:# and err_state == False:
    plot_solution_7dof(subplot_data, plot_fig = False, save_plot=True, file_name=output_file_path, matlab_import=False)

# print('Max error:       y - y_d = {:.2e}'.format(np.max(np.abs(e))), 'm')
# print('Max error:   y_p - y_d_p = {:.2e}'.format(np.max(np.abs(e_p))), 'm/s')
# print('Max error: y_pp - y_d_pp = {:.2e}'.format(np.max(np.abs(e_pp))), 'm/s^2')
#print("\nTotal cost:", ddp.cost)

# visualize=False
# if visualize==True:
#     visualize_robot(robot, q, traj_data_true, Ts, 3, 1)

if use_data_from_simulink:
    user_input = input("Do you want to clear the shared memory? (y/n): ").lower()
    if user_input == 'y':
        for name, config in shm_objects.items():
            # Create shared memory object
            shm_objects[name].close()
            shm_objects[name].unlink()
            
        print("Shared Memory freigegeben.")
    else:
        print("Shared Memory nicht freigegeben.")

if autostart_fr3:
    user_input = input("Do you want to enable brakes? (y/n): ").lower()
    if user_input == 'y':
        message = "stop"
        asyncio.run(send_message(message))