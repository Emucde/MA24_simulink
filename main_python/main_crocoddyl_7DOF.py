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
elif robot_name == 'fr3_6dof_no_hand':
    mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files/Meshes_FR3')
    urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', 'fr3_no_hand_6dof.urdf')
    urdf_tcp_frame_name = 'fr3_link8_tcp'
    trajectory_data_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/fr3_no_hand_6dof/trajectory_data/param_traj_data.mat')
    trajectory_param_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/fr3_no_hand_6dof/trajectory_data/param_traj.mat')
    n_indices = np.array([0, 1, 3, 4, 5, 6]) # don't use joint 3

################################################ REALTIME ###############################################

use_data_from_simulink = False
if use_data_from_simulink:
    # n_dof = 7 (input data from simulink are 7dof states q, qp)
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

    python_buffer_bytes = 6 * 8 # 1x6 torque inputs, 8 bytes per double
    python_flag_bytes = 1 * 8 # 1 byte
    simulink_buffer_bytes = 2 * 7 * 8 # 2x7 states q, qp, 8 bytes per double
    simulink_flag_bytes = 1 * 8# 1 byte

    shm_data_from_python = create_shared_memory(shm_data_from_python_name, python_buffer_bytes)  # 8 bytes pro double
    shm_data_from_python_valid = create_shared_memory(shm_data_from_python_valid_name, python_flag_bytes)  # 1 bytes (bit possible?)
    shm_data_from_simulink = create_shared_memory(shm_data_from_simulink_name, simulink_buffer_bytes)  # 8 bytes pro double
    shm_data_from_simulink_valid = create_shared_memory(shm_data_from_simulink_valid_name, simulink_flag_bytes)  # 1 bytes pro double

    data_from_python = np.ndarray((python_buffer_bytes//8,), dtype=np.float64, buffer=shm_data_from_python.buf)
    data_from_python_valid = np.ndarray((python_flag_bytes//8,), dtype=np.float64, buffer=shm_data_from_python_valid.buf)

    data_from_simulink = np.ndarray((simulink_buffer_bytes//8,), dtype=np.float64, buffer=shm_data_from_simulink.buf)
    data_from_simulink_valid = np.ndarray((simulink_flag_bytes//8,), dtype=np.float64, buffer=shm_data_from_simulink_valid.buf)

######################################### Build Robot Model #############################################

robot_model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

nq = robot_model.nq
nx = 2*nq
nu = nq # fully actuated

# Gravity should be in -y direction
robot_model.gravity.linear[:] = [0, 0, -9.81]

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
traj_select 0: Wrist Singularity 1, Equilibrium, Workspace
traj_select 1: Wrist Singularity 1, Diff filt, Workspace
traj_select 2: Wrist Singularity 1, Polynomial, Workspace
traj_select 3: Wrist Singularity 1, Sinus, Workspace
traj_select 4: 2DOF Test Trajectory, Polynomial, joint space
'''

traj_select = 4

t = traj_data_all['t'][0,0][0, :]
p_d = traj_data_all['p_d'][0,0][:, :, traj_select]
p_d_p = traj_data_all['p_d_p'][0,0][:, :, traj_select]
p_d_pp = traj_data_all['p_d_pp'][0,0][:, :, traj_select]
R_d = traj_data_all['R_d'][0,0][:, :, :, traj_select]
q_d = traj_data_all['q_d'][0,0][:, :, traj_select]
omega_d = traj_data_all['omega_d'][0,0][:, :, traj_select]
omega_d_p = traj_data_all['omega_d_p'][0,0][:, :, traj_select]

traj_data = {'p_d': p_d, 'p_d_p': p_d_p, 'p_d_pp': p_d_pp, 'R_d': R_d, 'q_d': q_d, 'omega_d': omega_d, 'omega_d_p': omega_d_p}

q_0 = traj_param_all['q_0'][0,0][n_indices, traj_select]
q_0_p = traj_param_all['q_0_p'][0,0][n_indices, traj_select]
q_0_pp = traj_param_all['q_0_pp'][0,0][n_indices, traj_select]

# SIMULATION SETTINGS

T_start = 0
T_end = 10
Ts = 1e-3  # Time step of control

#########################################################################################################
############################################ MPC Settings ###############################################
#########################################################################################################


mpc_settings = {
    'version' : 'MPC_v1_bounds_terminate', # 'MPC_v1_soft_terminate' | 'MPC_v1_bounds_terminate' | 'MPC_v3_soft_yN_ref'| 'MPC_v3_bounds_yN_ref' ,
    'Ts_MPC' : 1e-3, # Interne Abtastzeit der MPC muss vielfaches von Ts sein
    'N_MPC': 5, # anzahl der Stützstellen innerhalb des Prädiktionshorizont
    'int_method': 'euler', # 'euler' | 'RK2' | 'RK3' | 'RK4'
    'solver_steps': 1000
    }

param_mpc_weight = {
    'q_tracking_cost': 1e5,            # penalizes deviations from the trajectory
    'q_terminate_tracking_cost': 1e8,  # penalizes deviations from the trajectory at the end
    'q_terminate_tracking_bound_cost': 1e5,  # penalizes deviations from the bounds of | y_N - y_N_ref | < eps
    'q_xreg_terminate_cost': 1e-5,  # penalizes deviations from the trajectory at the end
    'q_ureg_terminate_cost': 1e-5,  # penalizes deviations from the trajectory at the end
    'q_xreg_cost': 1e-10,              # penalizes changes from the current state
    'q_ureg_cost': 1e-10,              # penalizes changes from the current input
    'q_x_bound_cost': 1e5,              # penalizes ignoring the bounds
    'q_u_bound_cost': 1e5,              # penalizes ignoring the bounds
    'Kd': 100*np.eye(3),
    'Kp': 100*np.eye(3),
    'lb_y_ref_N': -1e-6*np.ones(3), # only used if MPC_v3_bounds_yN_ref
    'ub_y_ref_N': 1e-6*np.ones(3),
    'umin': -np.hstack([150, 150, 150, 28, 28, 28]),
    'umax': np.hstack([150, 150, 150, 28, 28, 28]),
    'xmin': -np.hstack([2*np.pi*np.ones(6), np.pi*np.ones(6)]),
    'xmax': np.hstack([2*np.pi*np.ones(6), np.pi*np.ones(6)]),
    'xref': np.zeros(nx),
    'uref': np.zeros(nq)
}

# good for mpcv1 soft terminate:
# N_MPC = 5
# N_step = 20
# param_mpc_weight = {
    # 'q_tracking_cost': 1e0,            # penalizes deviations from the trajectory
    # 'q_terminate_tracking_cost': 1e5,  # penalizes deviations from the trajectory at the end
    # 'q_xreg_terminate_cost': 1e1,  # penalizes deviations from the trajectory at the end
    # 'q_ureg_terminate_cost': 1e3,  # penalizes deviations from the trajectory at the end
    # 'q_xreg_cost': 1e1,              # penalizes changes from the current state
    # 'q_ureg_cost': 1e3,              # penalizes changes from the current input
    # 'Kd': 8*np.eye(3),
    # 'Kp': 8**2/4*np.eye(3),
    # 'lb_y_ref_N': -1e-6*np.ones(3), # only used if MPC_v3_bounds_yN_ref
    # 'ub_y_ref_N': 1e-6*np.ones(3),
    # 'umin': -1.5*np.ones(2),
    # 'umax': 1.5*np.ones(2),
    # 'xmin': -np.hstack([np.pi*np.ones(2), 5*np.ones(2)]),
    # 'xmax': np.hstack([np.pi*np.ones(2), 5*np.ones(2)])
# }


# Create a multibody state from the pinocchio model.
state = cro.StateMultibody(robot_model)
state.lb = param_mpc_weight['xmin']
state.ub = param_mpc_weight['xmax']


#########################################################################################################
############################################# INIT MPC ##################################################
#########################################################################################################

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
T = T_end+T_horizon-T_start # need more trajectory points for mpc
N_traj = int(T_end/Ts)
t = np.arange(T_start, T, Ts)
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


x_init_robot = None
err_state = False

if use_data_from_simulink:
    print("Warte auf Daten von Simulink...\n")
    try:
        while True:
            # Daten für Simulink schreiben
            # time.sleep(3e-3)
            if(data_from_python_valid[:] == 0):
                data_from_python_valid[:] = 1
                data_from_python[:] = np.zeros(6)
                print("Daten von Python:", data_from_python[:5])  # Zeige die ersten 5 Werte
            
            # Daten von Simulink lesen
            if(data_from_simulink_valid[:] == 1):
                print("Daten von Simulink:", data_from_simulink[:])  # Zeige die ersten 5 Werte
                data_from_simulink_valid[:] = 0
                data_from_python_valid[:] = 0
                x_init_robot = data_from_simulink[np.hstack([n_indices, n_indices+7])]
            # time.sleep(1e-9)
    except KeyboardInterrupt:
        if(x_init_robot is None):
            x_init_robot = np.hstack([q_0, q_0_p])
            err_state = True
            print("\033[31mError: Keine Daten von Simulink erhalten. Beende Programm...\033[0m")
        else:
            print("\nStart Solving\n")
else:
    x_init_robot = np.hstack([q_0, q_0_p])


tau_init_robot = pin.rnea(robot_model, robot_data, q_0, q_0_p, q_0_pp)

x_k, xs, us, xs_init_guess, us_init_guess = init_guess_fun(tau_init_robot, x_init_robot, N_traj, N_MPC, nx, nu, traj_data)

#########################################################################################################
######################################### MPC Calcuations ###############################################
#########################################################################################################

error = False
warn_cnt = 0
conv_max_limit = 10
update_interval = N_traj//100

freq_per_Ta_step = np.zeros(N_traj)

measureSolver = TicToc()
measureSimu = TicToc()
measureTotal = TicToc()
measureTotal.tic()

i = 0
# create first problem:
y_d_ref = {
    'p_d': p_d[:, 0+MPC_traj_indices],
    'R_d': R_d[:, :, 0+MPC_traj_indices]
}

param_mpc_weight['xref'] = x_k
param_mpc_weight['uref'] = us_init_guess[0]
problem = create_ocp_problem(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings)
# problem = create_ocp_problem(i, i+N_MPC, N_step, state, x_k, TCP_frame_id, traj_data, param_mpc_weight, Ts, int_type = int_type)

# Solve first optimization Problem for warm start
ddp = solver(problem)
hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)
warn_cnt = check_solver_status(warn_cnt, hasConverged, ddp, us, xs, i, t, Ts, N_MPC, N_step, TCP_frame_id, robot_model, traj_data, conv_max_limit=5, plot_sol=not False)

xs[0] = ddp.xs[0] # muss so sein, da x0 in ddp.xs[0] gespeichert ist
us[0] = ddp.us[0]

xs_init_guess = ddp.xs
us_init_guess = ddp.us

run_loop = True
try:
    while run_loop and err_state == False:
    # for i in range(N_traj):
        measureSimu.tic()

        if use_data_from_simulink:           
            # Daten von Simulink lesen
            if(data_from_simulink_valid[:] == 1):
                data_from_simulink_valid[:] = 0
                data_from_python_valid[:] = 0
                x_k = data_from_simulink[np.hstack([n_indices, n_indices+7])]

        # v1: inefficient: create new problem every time
        # param_mpc_weight['xref'] = x_k
        # param_mpc_weight['uref'] = us[i]

        # y_d_ref['p_d'] = p_d[:, i+MPC_traj_indices]
        # y_d_ref['pR_d_d'] = R_d[:, :, i+MPC_traj_indices]

        # problem = create_ocp_problem(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings)
        # v1 end
        
        # v2: update reference values
        for j, runningModel in enumerate(problem.runningModels):
            problem.runningModels[j].differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j]]
            problem.runningModels[j].differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j]]
            # problem.runningModels[j].differential.costs.costs["stateReg"].cost.residual.reference = np.hstack([x_k[:nq], np.zeros(6)])
            problem.runningModels[j].differential.costs.costs["stateReg"].cost.residual.reference = x_k
            problem.runningModels[j].differential.costs.costs["stateRegBound"].cost.residual.reference = x_k
            problem.runningModels[j].differential.costs.costs["ctrlReg"].cost.residual.reference = us[i] #first us[0] is torque for gravity compensation
            problem.runningModels[j].differential.costs.costs["ctrlRegBound"].cost.residual.reference = us[i] #first us[0] is torque for gravity compensation

        problem.terminalModel.differential.costs.costs["TCP_pose"].cost.residual.reference = p_d[:, i+MPC_traj_indices[j+1]]
        problem.terminalModel.differential.costs.costs["TCP_rot"].cost.residual.reference = R_d[:, :, i+MPC_traj_indices[j+1]]
        # problem.terminalModel.differential.costs.costs["stateReg"].cost.residual.reference = np.hstack([x_k[:nq], np.zeros(6)])
        problem.terminalModel.differential.costs.costs["stateReg"].cost.residual.reference = x_k
        problem.terminalModel.differential.costs.costs["stateRegBound"].cost.residual.reference = x_k
        problem.terminalModel.differential.costs.costs["ctrlReg"].cost.residual.reference = us[i] #first us[0] is torque for gravity compensation
        problem.terminalModel.differential.costs.costs["ctrlRegBound"].cost.residual.reference = us[i] #first us[0] is torque for gravity compensation

        problem.x0 = x_k
        # v2 end

        ddp = solver(problem)
        # ddp.setCallbacks([cro.CallbackVerbose()])

        measureSolver.tic()
        hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)
        measureSolver.toc()
        

        warn_cnt = check_solver_status(warn_cnt, hasConverged, ddp, us, xs, i, t, Ts, N_MPC, N_step, TCP_frame_id, robot_model, traj_data, conv_max_limit=5, plot_sol=not False)

        if use_data_from_simulink:
            xs[i] = ddp.xs[0] # muss so sein, da x0 in ddp.xs[0] gespeichert ist
            us[i] = ddp.us[0]

            xs_init_guess = ddp.xs
            us_init_guess = ddp.us
            # Daten von Python an Simulink schreiben
            if(data_from_python_valid[:] == 0):
                data_from_python_valid[:] = 1
                data_from_python[:] = us[i]
        else:
            x_k, xs[i], us[i], xs_init_guess, us_init_guess = simulate_model(ddp, i, Ts, nq, nx, robot_model, robot_data, traj_data)
            
            # alternative: Use only solver values (perfect tracking)
            # xs[i] = ddp.xs[0] # muss so sein, da x0 in ddp.xs[0] gespeichert ist
            # us[i] = ddp.us[0]

            # xs_init_guess = ddp.xs
            # us_init_guess = ddp.us

            # x_k = ddp.xs[1]

        if (i+1) % update_interval == 0:
            print(f"{100 * (i+1)/N_traj:.2f} % | {measureTotal.get_time_str()}    ", end='\r')

        if i < N_traj-1:
            i += 1 # last value of i is N_traj-1
            # in case of use_data_from_simulink == True, the last trajectory value
            # is used for all further calculations (stay on the last position)
        
        if i == N_traj-1 and use_data_from_simulink == False:
            run_loop = False
    
        elapsed_time = measureSimu.toc()
        freq_per_Ta_step[i] = 1/elapsed_time
except KeyboardInterrupt:
    print("\nFinish Solving!")

xs[N_traj-1] = x_k
us[N_traj-1] = us[N_traj-2] # simply use previous value for display (does not exist)

measureSolver.print_time(additional_text='Total Solver time')
measureTotal.print_time(additional_text='Total MPC time')

t = t[:N_traj]

traj_data['p_d']    = traj_data['p_d'][   :, 0:N_traj]
traj_data['p_d_p']  = traj_data['p_d_p'][ :, 0:N_traj]
traj_data['p_d_pp'] = traj_data['p_d_pp'][:, 0:N_traj]

traj_data['q_d']    = traj_data['q_d'][   :, 0:N_traj]
traj_data['omega_d']  = traj_data['omega_d'][ :, 0:N_traj]
traj_data['omega_d_p'] = traj_data['omega_d_p'][:, 0:N_traj]


#########################################################################################################
############################################# Plot Results ##############################################
#########################################################################################################

subplot_data = calc_7dof_data(us, xs, t, TCP_frame_id, robot_model, traj_data, freq_per_Ta_step)
# folderpath = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240916_meeting/"
folderpath = "/home/rslstudent/Students/Emanuel/crocoddyl_html_files/"
outputname = '240910_traj2_crocoddyl_T_horizon_25ms.html';
output_file_path = os.path.join(folderpath, outputname)

plot_sol=True
if plot_sol == True and err_state == False:
    plot_solution_7dof(subplot_data, plot_fig = False, save_plot=True, file_name=output_file_path, matlab_import=False)

# print('Max error:       y - y_d = {:.2e}'.format(np.max(np.abs(e))), 'm')
# print('Max error:   y_p - y_d_p = {:.2e}'.format(np.max(np.abs(e_p))), 'm/s')
# print('Max error: y_pp - y_d_pp = {:.2e}'.format(np.max(np.abs(e_pp))), 'm/s^2')
#print("\nTotal cost:", ddp.cost)
if err_state == False:
    print("Minimum Found:", hasConverged)

# visualize=False
# if visualize==True:
#     visualize_robot(robot, q, traj_data, Ts, 3, 1)

if use_data_from_simulink:
    shm_data_from_python.close()
    shm_data_from_python.unlink()
    shm_data_from_python_valid.close()
    shm_data_from_python_valid.unlink()
    shm_data_from_simulink.close()
    shm_data_from_simulink.unlink()
    shm_data_from_simulink_valid.close()
    shm_data_from_simulink_valid.unlink()
    print("Shared Memory freigegeben.")