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

sys.path.append(os.path.dirname(os.path.abspath('./utils_python')))
from utils_python.utils import *

# robot_name = 'ur5e_6dof'
robot_name = 'fr3_6dof_no_hand'

if robot_name == 'ur5e_6dof':
    mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files/Meshes_ur5e')
    urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', 'ur5e.urdf')
    urdf_tcp_frame_name = 'ur5e_tcp'

    trajectory_data_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/ur5e_6dof/trajectory_data/param_traj_data.mat')
    trajectory_param_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/ur5e_6dof/trajectory_data/param_traj.mat')
    n_indices = np.array([0, 1, 2, 3, 4, 5]) # use all joints

    q_0_ref = np.array([0, 0, 0, 0, 0, 0])
   
   # Create a list of joints to lock
    jointsToLock = []
elif robot_name == 'fr3_6dof_no_hand':
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
    q_0_ref = np.array([0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, np.pi/4])

    # Create a list of joints to lock (starts with joint 1)
    jointsToLockIndex = np.setdiff1d(np.arange(0,7), n_indices) # at first use 7dof Model (will be later reduced)
    jointsToLock = jointsToLock = [f"fr3_joint{i+1}" for i in jointsToLockIndex] # list of joints to lock

################################################ REALTIME ###############################################

use_data_from_simulink = True
if use_data_from_simulink:
    n_dof = 7 # (input data from simulink are 7dof states q, qp)
    def create_shared_memory(name, size):
        try:
            shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        except FileExistsError:
            shm = shared_memory.SharedMemory(name=name)
        return shm

    shm_data_from_python_name         = "data_from_python"
    shm_data_from_python_valid_name   = "data_from_python_valid"

    shm_data_from_simulink_name       = "data_from_simulink"
    shm_data_from_simulink_valid_name = "data_from_simulink_valid"

    shm_data_from_simulink_start_name = "data_from_simulink_start"
    shm_data_from_simulink_reset_name = "data_from_simulink_reset"
    shm_data_from_simulink_stop_name  = "data_from_simulink_stop"
    shm_data_from_simulink_traj_switch_name  = "data_from_simulink_traj_switch"

    python_buffer_bytes = n_dof * 8 # 1x7 torque inputs, 8 bytes per double
    python_flag_bytes = 1 * 8 # 8 byte
    simulink_buffer_bytes = 2 * n_dof * 8 # 2x7 states q, qp, 8 bytes per double
    simulink_flag_bytes = 1# 1 byte

    shm_data_from_python = create_shared_memory(shm_data_from_python_name, python_buffer_bytes)  # 8 bytes pro double
    shm_data_from_python_valid = create_shared_memory(shm_data_from_python_valid_name, python_flag_bytes)  # 1 bytes (bit possible?)
    shm_data_from_simulink = create_shared_memory(shm_data_from_simulink_name, simulink_buffer_bytes)  # 8 bytes pro double
    shm_data_from_simulink_valid = create_shared_memory(shm_data_from_simulink_valid_name, simulink_flag_bytes)  # 1 bytes pro double
    
    shm_data_from_simulink_start = create_shared_memory(shm_data_from_simulink_start_name, simulink_flag_bytes)  # 1 bytes pro double
    shm_data_from_simulink_reset = create_shared_memory(shm_data_from_simulink_reset_name, simulink_flag_bytes)  # 1 bytes pro double
    shm_data_from_simulink_stop = create_shared_memory(shm_data_from_simulink_stop_name, simulink_flag_bytes)  # 1 bytes pro double
    shm_data_from_simulink_traj_switch = create_shared_memory(shm_data_from_simulink_traj_switch_name, simulink_flag_bytes)  # 1 bytes pro double

    data_from_python = np.ndarray((python_buffer_bytes//8,), dtype=np.float64, buffer=shm_data_from_python.buf)
    data_from_python_valid = np.ndarray((python_flag_bytes//8,), dtype=np.float64, buffer=shm_data_from_python_valid.buf)

    data_from_simulink = np.ndarray((simulink_buffer_bytes//8,), dtype=np.float64, buffer=shm_data_from_simulink.buf)

    data_from_simulink_valid = np.ndarray((simulink_flag_bytes,), dtype=np.int8, buffer=shm_data_from_simulink_valid.buf)
    data_from_simulink_start = np.ndarray((simulink_flag_bytes,), dtype=np.int8, buffer=shm_data_from_simulink_start.buf)
    data_from_simulink_reset = np.ndarray((simulink_flag_bytes,), dtype=np.int8, buffer=shm_data_from_simulink_reset.buf)
    data_from_simulink_stop = np.ndarray((simulink_flag_bytes,),  dtype=np.int8, buffer=shm_data_from_simulink_stop.buf)
    data_from_simulink_traj_switch = np.ndarray((simulink_flag_bytes,),  dtype=np.int8, buffer=shm_data_from_simulink_traj_switch.buf)

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
traj_select 6: Sing test, Polynomial, joint space
'''

curent_traj_select = 3
traj_data, traj_data_true, traj_init_config = process_trajectory_data(curent_traj_select-1, traj_data_all, traj_param_all)

p_d = traj_data['p_d']
R_d = traj_data['R_d']

t = traj_data_true['t_true']

q_0 = traj_init_config['q_0'][n_indices]
q_0_p = traj_init_config['q_0_p'][n_indices]
q_0_pp = traj_init_config['q_0_pp'][n_indices]

N_traj = traj_init_config['N_traj_true']

# SIMULATION SETTINGS

T_start = 0
T_end = 10
Ts = 1e-3  # Time step of control

#########################################################################################################
############################################ MPC Settings ###############################################
#########################################################################################################

# see utils_python/mpc_weights_crocoddyl.json

######################################### Build Robot Model #############################################

robot_model_full, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

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
    initialJointConfig[n_indices] = q_0

    robot_model = pin.buildReducedModel(robot_model_full, jointsToLockIDs, initialJointConfig)
else:
    robot_model = robot_model_full

nq = robot_model.nq
nx = 2*nq
nu = nq # fully actuated

# Gravity should be in -y direction
# robot_model.gravity.linear[:] = [0, 0, -9.81]
robot_model.gravity.linear[:] = [0, 0, 0]

robot_data = robot_model.createData()

TCP_frame_id = robot_model.getFrameId(urdf_tcp_frame_name)

# The model loaded from urdf (via pinicchio)
print(robot_model)

pin_data = robot_model.createData()

#########################################################################################################
############################################# INIT MPC ##################################################
#########################################################################################################

mpc_settings, param_mpc_weight = load_mpc_config(robot_model)

opt_type = mpc_settings['version']
int_type = mpc_settings['int_method']
N_solver_steps = mpc_settings['solver_steps']
N_MPC = mpc_settings['N_MPC']
Ts_MPC = mpc_settings['Ts_MPC']

solver, init_guess_fun, create_ocp_problem, simulate_model  = get_mpc_funs(opt_type)

# param_casadi_fun_name.(MPC).variant = 'nlpsol';
# param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
# param_casadi_fun_name.(MPC).version  = 'v6_kin_int_path_following'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v6_kin_int_path_following )
# param_casadi_fun_name.(MPC).Ts      = 5e-3;
# param_casadi_fun_name.(MPC).rk_iter = 1;
# param_casadi_fun_name.(MPC).N_MPC   = 5;
# param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
# param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
# param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

N_step = int(Ts_MPC/Ts)
T_horizon = (N_MPC-1) * Ts_MPC

print(f'T_horizon: {T_horizon} s, Ts: {Ts} s, N_MPC: {N_MPC}\n')

if N_step <= 2:
    MPC_traj_indices = np.arange(0, N_MPC)
    MPC_int_time = Ts*np.ones(N_MPC)
else:
    MPC_traj_indices = np.hstack([[0, 1], N_step* np.arange(1, (N_MPC-1))])
    MPC_int_time = np.hstack([Ts, Ts_MPC-Ts, np.ones(N_MPC-2)*Ts_MPC])

# TODO: MPC solvt es nur wenn man equidistante Werte nimmt!!
# MPC_traj_indices = np.arange(1, (N_MPC) * N_step, N_step)
# MPC_int_time = np.ones(N_MPC)*Ts_MPC

param_traj = {
    'traj_indices': MPC_traj_indices,
    'int_time': MPC_int_time,
}

# use start pose at trajectory for first init guess
x_init_robot = np.hstack([q_0, q_0_p])
tau_init_robot = pin.rnea(robot_model, robot_data, q_0, q_0_p, q_0_pp)

x_k, xs, us, xs_init_guess, us_init_guess = init_guess_fun(tau_init_robot, x_init_robot, N_traj, N_MPC, nx, nu, traj_data)

#########################################################################################################
######################################### MPC Calcuations ###############################################
#########################################################################################################

if use_data_from_simulink:
    run_flag = False
    data_from_python_valid[:] = 0
    data_from_python[:] = np.zeros(n_dof)
else:
    run_flag = True

err_state = False
warn_cnt = 0
conv_max_limit = 10
update_interval = N_traj//100

freq_per_Ta_step = np.zeros(N_traj)

measureSolver = TicToc()
measureSimu = TicToc()
measureTotal = TicToc()
measureTotal.tic()

# create first problem:
y_d_ref = {
    'p_d': p_d[:,    0+MPC_traj_indices],
    'R_d': R_d[:, :, 0+MPC_traj_indices]
}

param_mpc_weight['xref'] = x_k
param_mpc_weight['uref'] = us_init_guess[0]

# Create a multibody state from the pinocchio model.
state = cro.StateMultibody(robot_model)
state.lb = param_mpc_weight['xmin']
state.ub = param_mpc_weight['xmax']

problem = create_ocp_problem(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings)

# Solve first optimization Problem for warm start
ddp = solver(problem)
hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)
warn_cnt, err_state = check_solver_status(warn_cnt, hasConverged, ddp, 0, Ts, conv_max_limit=5)

xs[0] = ddp.xs[0] # muss so sein, da x0 in ddp.xs[0] gespeichert ist
us[0] = ddp.us[0]

xs_init_guess = ddp.xs
us_init_guess = ddp.us

if use_data_from_simulink:
    start_solving = False
    data_from_python_valid[:] = 1
    us_temp = np.zeros(n_dof)
    us_temp[n_indices] = us[0]
    data_from_python[:] = us_temp
else:
    start_solving = True

i = 0
folderpath = "/home/rslstudent/Students/Emanuel/crocoddyl_html_files/"
outputname = 'test.html'
output_file_path = os.path.join(folderpath, outputname)
run_loop = True
try:
    while run_loop and err_state == False:
    # for i in range(N_traj):

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

                    if curent_traj_select != new_traj_select:
                        curent_traj_select = new_traj_select
                        traj_data, traj_data_true, traj_init_config = process_trajectory_data(curent_traj_select-1, traj_data_all, traj_param_all)
                        p_d = traj_data['p_d']
                        R_d = traj_data['R_d']
                        t = traj_data_true['t_true']
                        N_traj = traj_init_config['N_traj_true']
                        print(f'New Trajectory selected: {curent_traj_select}')

                        y_d_ref = {
                            'p_d': p_d[:,    0+MPC_traj_indices],
                            'R_d': R_d[:, :, 0+MPC_traj_indices]
                        }

                        q_0 = traj_init_config['q_0'][n_indices]
                        q_0_p = traj_init_config['q_0_p'][n_indices]
                        q_0_pp = traj_init_config['q_0_pp'][n_indices]

                        x_init_robot = np.hstack([q_0, q_0_p])
                        tau_init_robot = pin.rnea(robot_model, robot_data, q_0, q_0_p, q_0_pp)

                    # update mpc settings and problem
                    mpc_settings, param_mpc_weight = load_mpc_config(robot_model)

                    N_MPC = mpc_settings['N_MPC']
                    Ts_MPC = mpc_settings['Ts_MPC']
                    N_step = int(Ts_MPC/Ts)
                    T_horizon = (N_MPC-1) * Ts_MPC
                    if N_step <= 2:
                        MPC_traj_indices = np.arange(0, N_MPC)
                        MPC_int_time = Ts*np.ones(N_MPC)
                    else:
                        MPC_traj_indices = np.hstack([[0, 1], N_step* np.arange(1, (N_MPC-1))])
                        MPC_int_time = np.hstack([Ts, Ts_MPC-Ts, np.ones(N_MPC-2)*Ts_MPC])
                    param_traj = {
                        'traj_indices': MPC_traj_indices,
                        'int_time': MPC_int_time,
                    }

                    param_mpc_weight['xref'] = x_k
                    param_mpc_weight['uref'] = us_init_guess[0]
                    problem = create_ocp_problem(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings)
                    ddp = solver(problem)

                    if( i == N_traj-1): # autoreset but then no data is printed
                        i = 0

                    x_k, xs, us, xs_init_guess, us_init_guess = init_guess_fun(tau_init_robot, x_init_robot, N_traj, N_MPC, nx, nu, traj_data)

                    run_flag = True
                    
                    print("MPC started by Simulink")
                elif reset == 1 and i == N_traj-1:
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

            if(data_from_simulink_valid[:] == 1 and run_flag == True):
                data_from_simulink_valid[:] = 0
                data_from_python_valid[:] = 0
                x_k = data_from_simulink[np.hstack([n_indices, n_indices+7])]
                start_solving = True
        if start_solving:
            measureSimu.tic()
            
            g_k = pin.computeGeneralizedGravity(robot_model, pin_data, x_k[:nq])
            
            # v2: update reference values
            for j, runningModel in enumerate(problem.runningModels):
                problem.runningModels[j].differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j]]
                if(nq >= 6):
                    problem.runningModels[j].differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j]]
                # problem.runningModels[j].differential.costs.costs["stateReg"].cost.residual.reference = np.hstack([x_k[:nq], np.zeros(n_dof)])
                # problem.runningModels[j].differential.costs.costs["stateReg"].cost.residual.reference = x_k
                # problem.runningModels[j].differential.costs.costs["stateRegBound"].cost.residual.reference = x_k
                problem.runningModels[j].differential.costs.costs["stateReg"].cost.residual.reference = xs_init_guess[j]
                problem.runningModels[j].differential.costs.costs["stateRegBound"].cost.residual.reference = xs_init_guess[j]
                # problem.runningModels[j].differential.costs.costs["ctrlReg"].cost.residual.reference = us[i] #first us[0] is torque for gravity compensation
                # problem.runningModels[j].differential.costs.costs["ctrlRegBound"].cost.residual.reference = us[i] #first us[0] is torque for gravity compensation
                problem.runningModels[j].differential.costs.costs["ctrlReg"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
                problem.runningModels[j].differential.costs.costs["ctrlRegBound"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation

            problem.terminalModel.differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j+1]]
            if(nq >= 6):
                problem.terminalModel.differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j+1]]
            # problem.terminalModel.differential.costs.costs["stateReg"].cost.residual.reference = np.hstack([x_k[:nq], np.zeros(n_dof)])
            # problem.terminalModel.differential.costs.costs["stateReg"].cost.residual.reference = x_k
            # problem.terminalModel.differential.costs.costs["stateRegBound"].cost.residual.reference = x_k
            problem.terminalModel.differential.costs.costs["stateReg"].cost.residual.reference = xs_init_guess[j+1]
            problem.terminalModel.differential.costs.costs["stateRegBound"].cost.residual.reference = xs_init_guess[j+1]
            # problem.terminalModel.differential.costs.costs["ctrlReg"].cost.residual.reference = us[i] #first us[0] is torque for gravity compensation
            # problem.terminalModel.differential.costs.costs["ctrlRegBound"].cost.residual.reference = us[i] #first us[0] is torque for gravity compensation
            problem.terminalModel.differential.costs.costs["ctrlReg"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation
            problem.terminalModel.differential.costs.costs["ctrlRegBound"].cost.residual.reference = g_k #first us[0] is torque for gravity compensation

            problem.x0 = x_k
            # v2 end

            ddp = solver(problem)
            # ddp.setCallbacks([cro.CallbackVerbose()])

            measureSolver.tic()
            hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)
            measureSolver.toc()
            

            warn_cnt, err_state = check_solver_status(warn_cnt, hasConverged, ddp, i, Ts, conv_max_limit=5)

            if use_data_from_simulink:
                if not err_state:
                    xs[i] = ddp.xs[0] # muss so sein, da x0 in ddp.xs[0] gespeichert ist
                    us[i] = ddp.us[0]

                    xs_init_guess = ddp.xs
                    us_init_guess = ddp.us
                    # Daten von Python an Simulink schreiben

                    if i == 0:
                        u_i_prev = np.zeros(nq)

                    delta_u = us[i] - u_i_prev
                    condition1 = np.logical_and(delta_u > 0, delta_u > 5)
                    condition2 = np.logical_and(delta_u < 0, delta_u < -5)

                    if np.any(np.logical_or(condition1, condition2)): # aber kleiner als -0.8 ist ok
                        print(us[i])
                        print("Jump in torque (> 0.9 Nm/ms) detected, output zero torque")
                        data_from_python[:] = np.zeros(n_dof)
                        if data_from_python_valid[0] == 1:
                            data_from_python_valid[:] = 0
                        run_flag = False
                    else:
                        us_temp = np.zeros(n_dof)
                        us_temp[n_indices] = us[i]
                        data_from_python[:] = us_temp
                        if data_from_python_valid[0] == 0:
                            data_from_python_valid[:] = 1
                else:
                    data_from_python[:] = np.zeros(n_dof)
                    data_from_python_valid[:] = 0
                    run_flag = False
                    err_state = False # damit python nicht crasht

                start_solving = False

                u_i_prev = us[i]
            else:
                # x_k_old = x_k
                x_k, xs[i], us[i], xs_init_guess, us_init_guess = simulate_model(ddp, i, Ts, nq, nx, robot_model, robot_data, traj_data)
                # x_k = x_k_old
                
                # alternative: Use only solver values (perfect tracking)
                # xs[i] = ddp.xs[0] # muss so sein, da x0 in ddp.xs[0] gespeichert ist
                # us[i] = ddp.us[0]

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
    data_from_python[:] = np.zeros(n_dof)
    data_from_python_valid[:] = 0
else:
    print("Minimum Found:", hasConverged)


xs[N_traj-1] = x_k
us[N_traj-1] = us[N_traj-2] # simply use previous value for display (does not exist)

measureSolver.print_time(additional_text='Total Solver time')
measureTotal.print_time(additional_text='Total MPC time')


#########################################################################################################
############################################# Plot Results ##############################################
#########################################################################################################

plot_full_model = False

if plot_full_model:
    pin_data_full = robot_model_full.createData()

    n_full = len(q_0_ref)
    x_indiced = np.hstack([n_indices, n_indices+n_full])
    us_full = np.zeros((N_traj,   n_full))
    xs_full = np.zeros((N_traj, 2*n_full))
    x_i = np.hstack([q_0_ref, np.zeros(n_full)])
    for i in range(N_traj):
        q = x_i[:n_full]
        q_p = x_i[n_full::]

        x_i[x_indiced] = xs[i]

        q_red = xs[i][:nq]
        q_p_red = xs[i][nq::]
        u_red = us[i]

        q_pp_red = pin.aba(robot_model, pin_data, q_red, q_p_red, u_red)
        q_pp = np.zeros(n_full)
        q_pp[n_indices] = q_pp_red

        us_full[i] = pin.rnea(robot_model_full, pin_data_full, q, q_p, q_pp)

        # us_full[i, n_indices] = us[i]
        xs_full[i] = x_i
    
    subplot_data = calc_7dof_data(us_full, xs_full, t, TCP_frame_id, robot_model_full, pin_data_full, traj_data_true, freq_per_Ta_step)
else:
    subplot_data = calc_7dof_data(us, xs, t, TCP_frame_id, robot_model, robot_data, traj_data_true, freq_per_Ta_step)


# folderpath = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240916_meeting/"
folderpath = "/home/rslstudent/Students/Emanuel/crocoddyl_html_files/"
outputname = '240910_traj2_crocoddyl_T_horizon_25ms.html'
output_file_path = os.path.join(folderpath, outputname)

plot_sol=True
if plot_sol == True:# and err_state == False:
    plot_solution_7dof(subplot_data, plot_fig = False, save_plot=True, file_name=output_file_path, matlab_import=False)

# print('Max error:       y - y_d = {:.2e}'.format(np.max(np.abs(e))), 'm')
# print('Max error:   y_p - y_d_p = {:.2e}'.format(np.max(np.abs(e_p))), 'm/s')
# print('Max error: y_pp - y_d_pp = {:.2e}'.format(np.max(np.abs(e_pp))), 'm/s^2')
#print("\nTotal cost:", ddp.cost)

# visualize=False
# if visualize==True:
#     visualize_robot(robot, q, traj_data_true, Ts, 3, 1)

if use_data_from_simulink:
    shm_data_from_python.close()
    shm_data_from_python.unlink()
    shm_data_from_python_valid.close()
    shm_data_from_python_valid.unlink()
    shm_data_from_simulink.close()
    shm_data_from_simulink.unlink()
    shm_data_from_simulink_valid.close()
    shm_data_from_simulink_valid.unlink()
    shm_data_from_simulink_start.close()
    shm_data_from_simulink_start.unlink()
    shm_data_from_simulink_reset.close()
    shm_data_from_simulink_reset.unlink()
    shm_data_from_simulink_stop.close()
    shm_data_from_simulink_stop.unlink()
    shm_data_from_simulink_traj_switch.close()
    shm_data_from_simulink_traj_switch.unlink()
    print("Shared Memory freigegeben.")