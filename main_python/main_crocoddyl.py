import os
import sys
import numpy as np
import crocoddyl as cro
import pinocchio as pin
import time
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize

sys.path.append(os.path.dirname(os.path.abspath('./utils_python')))
from utils_python.utils import *

mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files')
urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', '2dof_sys.urdf')

######################################### Build Robot Model #############################################

#model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

# Now load the model (using pinocchio)
robot = pin.robot_wrapper.RobotWrapper.BuildFromURDF(str(urdf_model_path))
robot_model = robot.model

nq = robot_model.nq
nx = 2*nq
nu = nq # fully actuated

# Gravity should be in -y direction
robot_model.gravity.linear[:] = [0, -9.81, 0]

# The model loaded from urdf (via pinicchio)
print(robot.model)

# Genau hier erkennt man das Problem, dass ein OCP statt einer MPC verwendet wird:
# Da nur sichergestellt wird, dass yref am Ende der Trajektorie mit einer sehr starken
# Gewichtung liegt. Allerdings ist ebendies ein großes Problem beim OCP Problem, da der
# Horizont eben extrem lang ist und wenn das Ende ohnehin im Arbeitsbereich liegt, kann man
# damit nicht sicherstellen, dass yref immer innerhlab der Trajektorie liegt.
# y_offset = 0.1 # Funktioniert bei OCP nicht ordentlich
y_offset = 0.1

# Calculate strechted position
qT = np.array([0,0])
robot_data = robot_model.createData()
pin.forwardKinematics(robot_model, robot_data, qT)
pin.updateFramePlacements(robot_model, robot_data)
TCP_frame_id = robot_model.getFrameId('TCP')
xeT = robot_data.oMf[TCP_frame_id].translation.T.copy()
xe0 = np.array([0.1, xeT[1], 0])

xeT[0] += y_offset
xe0[0] += y_offset

def f_cost_forward(q):
    pin.forwardKinematics(robot_model, robot_data, q, np.zeros(nq), np.zeros(nq))
    pin.updateFramePlacements(robot_model, robot_data)
    xe = robot_data.oMf[TCP_frame_id].translation.T
    return 1/2 * (xe - xe0) @ (xe - xe0)

bounds = ((-np.pi, np.pi), (-np.pi, np.pi))
qq0 = [1, 0] # Define the initial guess - avoid to start in singularity!! - Don't forget: There are always two q solutions for each pose!
res = minimize(f_cost_forward, qq0, method='SLSQP', bounds=bounds, options={'ftol': 1e-20, 'disp': False})
print(res.x)

# Create a multibody state from the pinocchio model.
state = cro.StateMultibody(robot_model)
state.lb[0:2] = np.array([-np.pi, -np.pi])
state.ub[0:2] = np.array([np.pi, np.pi])

state.lb[2:4] = -100 # wird ignored
state.ub[2:4] = 100

# Init trajectory rotation
R_init = np.eye(3)# H_0_init[0:3, 0:3]
R_target = np.eye(3)
RR = np.dot(R_init.T, R_target)
rot_quat = np.roll(Rotation.from_matrix(RR).as_quat(), 1) # as_quat has xyzw format: after roll: wxyz
rot_rho = rot_quat[0]
rot_alpha_scale = 2 * np.arccos(rot_rho)
if rot_alpha_scale == 0:
    rot_ax = np.array([0,0,1]) # random axis because rotation angle is 0
else:
    rot_ax = rot_quat[1:4] / np.sin(rot_alpha_scale / 2)

# Init state
q0   = res.x
q0_p = np.zeros(nq)

#########################################################################################################
################################## Trajectory settings ##################################################
#########################################################################################################

# Generate Trajectory
T_start = 0
T_end = 10
dt = 1e-3  # Time step # y_offset nicht vergessen!!!

# Parameters for the trajectory
param_traj_poly = {}
param_traj_poly['T'] = T_end/2-3

#########################################################################################################
############################################ MPC Settings ###############################################
#########################################################################################################

# Ich denke MPC_v3_bounds_yN_ref funktioniert nicht, weil sich diese boundaries nur auf die Inputs beziehen,
# Nicht aber auf die states. 

opt_type = 'MPC_v3_soft_yN_ref' # 'MPC_v1' | 'MPC_v3_soft_yN_ref'| 'MPC_v3_bounds_yN_ref' 
int_type = 'euler' # 'euler' | 'RK2' | 'RK3' | 'RK4'
N_solver_steps = 1000
N_horizon = 5
N_step = 1

param_mpc_weight = {
    'q_tracking_cost': 1e5,            # penalizes deviations from the trajectory
    'q_terminate_tracking_cost': 1e10,  # penalizes deviations from the trajectory at the end
    'q_xreg_cost': 0*1e-10,              # penalizes changes from the current state
    'q_ureg_cost': 0*1e-10,              # penalizes changes from the current input
    'Kd': 100*np.eye(3),
    'Kp': 2500*np.eye(3),
    'lb_y_ref_N': -1e-6*np.ones(3), # only used if MPC_v3_bounds_yN_ref
    'ub_y_ref_N': 1e-6*np.ones(3),
}

#########################################################################################################
############################################# INIT MPC ##################################################
#########################################################################################################

solver, init_guess_fun, create_ocp_problem, simulate_model  = get_mpc_funs(opt_type)

T_horizon = N_step*N_horizon*dt
T = T_end+T_horizon-T_start # need more trajectory points for mpc
N_traj = int(T_end/dt)
t = np.arange(T_start, T, dt)
param_trajectory = generate_trajectory(t, xe0, xeT, R_init, rot_ax, rot_alpha_scale, T_start, T, param_traj_poly, plot_traj=False)
print(f'T_horizon: {T_horizon} s, dt: {dt} s, N_horizon: {N_horizon}\n')

x_init_robot = np.hstack([q0, q0_p])
tau_init_robot = pin.rnea(robot_model, robot_data, q0, q0_p, np.zeros(nq))

xk, xs, us, xs_init_guess, us_init_guess = init_guess_fun(tau_init_robot, x_init_robot, N_traj, N_horizon, nx, nu, param_trajectory)

#########################################################################################################
######################################### MPC Calcuations ###############################################
#########################################################################################################

error = False
warn_cnt = 0
conv_max_limit = 10
update_interval = N_traj//100

measureSolver = TicToc()
measureTotal = TicToc()
measureTotal.tic()
for i in range(N_traj):
    problem = create_ocp_problem(i, i+N_horizon, N_step, state, xk, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type = int_type)
    ddp = solver(problem)
    # ddp.setCallbacks([cro.CallbackVerbose()])

    measureSolver.tic()
    hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)
    measureSolver.toc()

    warn_cnt = check_solver_status(warn_cnt, hasConverged, ddp, us, xs, i, t, dt, N_horizon, N_step, TCP_frame_id, robot_model, param_trajectory, conv_max_limit=5, plot_sol=not False)

    xk, xs[i], us[i], xs_init_guess, us_init_guess = simulate_model(ddp, i, dt, nq, nx, robot_model, robot_data, param_trajectory)

    if (i+1) % update_interval == 0:
        print(f"{100 * (i+1)/N_traj:.2f} % | {measureTotal.get_time_str()}    ", end='\r')

measureSolver.print_time(additional_text='Total Solver time')
measureTotal.print_time(additional_text='Total MPC time')

param_trajectory['p_d']    = param_trajectory['p_d'][   :, 0:N_traj]
param_trajectory['p_d_p']  = param_trajectory['p_d_p'][ :, 0:N_traj]
param_trajectory['p_d_pp'] = param_trajectory['p_d_pp'][:, 0:N_traj]


#########################################################################################################
############################################# Plot Results ##############################################
#########################################################################################################

plot_sol=True
if plot_sol:
    y_opt, y_opt_p, y_opt_pp, e, e_p, e_pp, w, q, q_p, q_pp, tau = plot_solution(us, xs, t, TCP_frame_id, robot_model, param_trajectory)
else:
    y_opt, y_opt_p, y_opt_pp, e, e_p, e_pp, w, q, q_p, q_pp, tau = plot_solution(us, xs, t, TCP_frame_id, robot_model, param_trajectory, plot_fig=False)

print('Max error:       y - y_d = {:.2e}'.format(np.max(np.abs(e))), 'm')
print('Max error:   y_p - y_d_p = {:.2e}'.format(np.max(np.abs(e_p))), 'm/s')
print('Max error: y_pp - y_d_pp = {:.2e}'.format(np.max(np.abs(e_pp))), 'm/s^2')
#print("\nTotal cost:", ddp.cost)
print("Minimum Found:", hasConverged)

visualize=False
if visualize==True:
    visualize_robot(robot, q, param_trajectory, dt, 3, 1)