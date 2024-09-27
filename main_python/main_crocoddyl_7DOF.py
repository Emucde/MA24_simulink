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

sys.path.append(os.path.dirname(os.path.abspath('./utils_python')))
from utils_python.utils import *

mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files/Meshes_ur5e')
urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', 'ur5e.urdf')

######################################### Build Robot Model #############################################

robot_model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

nq = robot_model.nq
nx = 2*nq
nu = nq # fully actuated

# Gravity should be in -y direction
robot_model.gravity.linear[:] = [0, 0, -9.81]

robot_data = robot_model.createData()

TCP_frame_id = robot_model.getFrameId('ur5e_tcp')

# The model loaded from urdf (via pinicchio)
print(robot_model)

#########################################################################################################
################################## Get Trajectory from mat file #########################################
#########################################################################################################

trajectory_data_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/ur5e_6dof/trajectory_data/param_traj_data.mat')
trajectory_param_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/ur5e_6dof/trajectory_data/param_traj.mat')

traj_data_all = sio.loadmat(trajectory_data_mat_file)['param_traj_data']
traj_param_all = sio.loadmat(trajectory_param_mat_file)['param_traj']

'''
traj_select 0: Wrist Singularity 1, Polynomial, Workspace
traj_select 1: Allsingularity, Polynomial, Workspace
traj_select 2: Ellbow Singularity 1, Polynomial, Workspace
traj_select 3: ELLBOW Singularity 2, Polynomial, Workspace
traj_select 4: ELLBOW Singularity 2, Polynomial, joint space
traj_select 5: Shoulder Sing 1, Polynomial, Workspace
traj_select 6: Shoulder Sing 2, Polynomial, Workspace
'''
traj_select = 0

t = traj_data_all['t'][0,0][0, :]
p_d = traj_data_all['p_d'][0,0][:, :, traj_select]
p_d_p = traj_data_all['p_d_p'][0,0][:, :, traj_select]
p_d_pp = traj_data_all['p_d_pp'][0,0][:, :, traj_select]
R_d = traj_data_all['R_d'][0,0][:, :, :, traj_select]
q_d = traj_data_all['q_d'][0,0][:, :, traj_select]
omega_d = traj_data_all['omega_d'][0,0][:, :, traj_select]
omega_d_p = traj_data_all['omega_d_p'][0,0][:, :, traj_select]

traj_data = {'p_d': p_d, 'p_d_p': p_d_p, 'p_d_pp': p_d_pp, 'R_d': R_d, 'q_d': q_d, 'omega_d': omega_d, 'omega_d_p': omega_d_p}

q_0 = traj_param_all['q_0'][0,0][:, traj_select]
q_0_p = traj_param_all['q_0_p'][0,0][:, traj_select]
q_0_pp = traj_param_all['q_0_pp'][0,0][:, traj_select]

T_start = 0
T_end = 10
dt = 1e-3  # Time step # y_offset nicht vergessen!!!

#########################################################################################################
############################################ MPC Settings ###############################################
#########################################################################################################

opt_type = 'MPC_v1_bounds_terminate' # 'MPC_v1_soft_terminate' | 'MPC_v1_bounds_terminate' | 'MPC_v3_soft_yN_ref'| 'MPC_v3_bounds_yN_ref' 
int_type = 'euler' # 'euler' | 'RK2' | 'RK3' | 'RK4'
N_solver_steps = 1000
N_MPC = 5 # anzahl der Stützstellen innerhalb des Prädiktionshorizont
Ts_MPC = 50e-3 # Interne Abtastzeit der MPC muss vielfaches von dt sein

param_mpc_weight = {
    'q_tracking_cost': 1e5,            # penalizes deviations from the trajectory
    'q_terminate_tracking_cost': 1e8,  # penalizes deviations from the trajectory at the end
    'q_terminate_tracking_bound_cost': 1e5,  # penalizes deviations from the bounds of | y_N - y_N_ref | < eps
    'q_xreg_terminate_cost': 1e-3,  # penalizes deviations from the trajectory at the end
    'q_ureg_terminate_cost': 1e-5,  # penalizes deviations from the trajectory at the end
    'q_xreg_cost': 1e-3,              # penalizes changes from the current state
    'q_ureg_cost': 1e-5,              # penalizes changes from the current input
    'q_x_bound_cost': 1e5,              # penalizes ignoring the bounds
    'q_u_bound_cost': 1e5,              # penalizes ignoring the bounds
    'Kd': 100*np.eye(3),
    'Kp': 100*np.eye(3),
    'lb_y_ref_N': -1e-6*np.ones(3), # only used if MPC_v3_bounds_yN_ref
    'ub_y_ref_N': 1e-6*np.ones(3),
    'umin': -np.hstack([150, 150, 150, 28, 28, 28]),
    'umax': np.hstack([150, 150, 150, 28, 28, 28]),
    'xmin': -np.hstack([2*np.pi*np.ones(6), np.pi*np.ones(6)]),
    'xmax': np.hstack([2*np.pi*np.ones(6), np.pi*np.ones(6)])
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

solver, init_guess_fun, create_ocp_problem, simulate_model  = get_mpc_funs(opt_type)

N_step = int(Ts_MPC/dt)
T_horizon = N_MPC*Ts_MPC
T = T_end+T_horizon-T_start # need more trajectory points for mpc
N_traj = int(T_end/dt)
t = np.arange(T_start, T, dt)
print(f'T_horizon: {T_horizon} s, dt: {dt} s, N_MPC: {N_MPC}\n')

x_init_robot = np.hstack([q_0, q_0_p])
tau_init_robot = pin.rnea(robot_model, robot_data, q_0, q_0_p, q_0_pp)

xk, xs, us, xs_init_guess, us_init_guess = init_guess_fun(tau_init_robot, x_init_robot, N_traj, N_MPC, nx, nu, traj_data)

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
for i in range(N_traj):
    measureSimu.tic()
    problem = create_ocp_problem(i, i+N_MPC, N_step, state, xk, TCP_frame_id, traj_data, param_mpc_weight, dt, int_type = int_type)
    ddp = solver(problem)
    # ddp.setCallbacks([cro.CallbackVerbose()])

    measureSolver.tic()
    hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)
    measureSolver.toc()
    

    warn_cnt = check_solver_status(warn_cnt, hasConverged, ddp, us, xs, i, t, dt, N_MPC, N_step, TCP_frame_id, robot_model, traj_data, conv_max_limit=5, plot_sol=not False)

    xk, xs[i], us[i], xs_init_guess, us_init_guess = simulate_model(ddp, i, dt, nq, nx, robot_model, robot_data, traj_data)

    if (i+1) % update_interval == 0:
        print(f"{100 * (i+1)/N_traj:.2f} % | {measureTotal.get_time_str()}    ", end='\r')
   
    elapsed_time = measureSimu.toc()
    freq_per_Ta_step[i] = 1/elapsed_time

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
plot_solution_7dof(subplot_data, plot_fig = False, save_plot=True, file_name=output_file_path, matlab_import=False)

plot_sol=False

# print('Max error:       y - y_d = {:.2e}'.format(np.max(np.abs(e))), 'm')
# print('Max error:   y_p - y_d_p = {:.2e}'.format(np.max(np.abs(e_p))), 'm/s')
# print('Max error: y_pp - y_d_pp = {:.2e}'.format(np.max(np.abs(e_pp))), 'm/s^2')
#print("\nTotal cost:", ddp.cost)
print("Minimum Found:", hasConverged)

# visualize=False
# if visualize==True:
#     visualize_robot(robot, q, traj_data, dt, 3, 1)