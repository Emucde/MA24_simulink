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
y_offset = 0.1*0

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

# Generate Trajectory
T_start = 0
T_end = 10
dt = 1e-3  # Time step # y_offset nicht vergessen!!!

# Parameters for the trajectory
param_traj_poly = {}
param_traj_poly['T'] = T_end/2-3

# Choose the optimization type
opt_type = 'MPC' # 'OCP' or 'MPC'

if opt_type == 'OCP':
    param_mpc_weight = {
        'q_tracking_cost': 1e5,            # penalizes deviations from the trajectory
        'q_terminate_tracking_cost': 1e10, # penalizes deviations from the trajectory at the end
        'q_xreg_cost': 0,             # penalizes changes from the current state
        'q_ureg_cost': 0,                 # penalizes changes from the current input
        'Kd': 100*np.eye(3),
        'Kp': 2500*np.eye(3),
    }
    T = T_end-T_start
    N_traj = int(T/dt)

    start_index = 0
    end_index = N_traj

    N_solver_steps = 300

    t = np.arange(T_start, T, dt)

    param_trajectory = generate_trajectory(t, xe0, xeT, R_init, rot_ax, rot_alpha_scale, T_start, T, param_traj_poly, plot_traj=False)

    x0_robot = np.hstack([q0, q0_p])
    problem = ocp_problem_v3(start_index, end_index, 1, state, x0_robot, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type='euler', int_steps=1)

    # Creating the DDP solver for this OC problem, defining a logger
    ddp = cro.SolverDDP(problem)

    # IV. Callbacks
    ddp.setCallbacks([cro.CallbackVerbose()])

    tic()
    hasConverged = ddp.solve([], [], N_solver_steps, False, 1e-5)
    toc()

    # adapt trajectory for plotting
    param_trajectory['p_d']    = param_trajectory['p_d'][   :, start_index:end_index:1]
    param_trajectory['p_d_p']  = param_trajectory['p_d_p'][ :, start_index:end_index:1]
    param_trajectory['p_d_pp'] = param_trajectory['p_d_pp'][:, start_index:end_index:1]

    xs = np.array(ddp.xs)
    us = np.array(ddp.us)
elif opt_type == 'MPC':
    param_mpc_weight = {
        'q_tracking_cost': 1e5,            # penalizes deviations from the trajectory
        'q_terminate_tracking_cost': 1e10, # penalizes deviations from the trajectory at the end
        'q_xreg_cost': 1e-10,             # penalizes changes from the current state
        'q_ureg_cost': 1e-5,                 # penalizes changes from the current input
        'Kd': 100*np.eye(3),
        'Kp': 2500*np.eye(3),
    }

    N_solver_steps = 50
    N_horizon = 5
    N_step = 10
    T_horizon = N_step*N_horizon*dt

    print(f'T_horizon: {T_horizon} s, dt: {dt} s, N_horizon: {N_horizon}\n')

    T = T_end+T_horizon-T_start # need more trajectory points for mpc
    N_traj = int(T_end/dt)

    t = np.arange(T_start, T, dt)

    param_trajectory = generate_trajectory(t, xe0, xeT, R_init, rot_ax, rot_alpha_scale, T_start, T, param_traj_poly, plot_traj=False)
    xs = np.zeros((N_traj, 6+nx))
    us = np.zeros((N_traj, 3+nu))

    # Reihenfolge beachten:
    # Zuerst Model1: yref Model (yref = [x1,x2,x3], d/dt yref = [x4,x5,x6]), 
    # dann Model2: Robot model (q = [q1, q2] = [x7, x8], d/dt q = d/dt [q1, q2] = [x9, x10])

    # TODO: Warm start (simulate system)

    x0_init_robot = np.hstack([q0, q0_p])
    x0_init = np.hstack([param_trajectory['p_d'][:, 0], param_trajectory['p_d_p'][:, 0], x0_init_robot])
    xk = x0_init

    tau_init_robot = pin.rnea(robot_model, robot_data, q0, q0_p, np.zeros(nq))

    xs_init_guess = [x0_init] * (N_horizon)
    us_init_guess = [np.hstack([param_trajectory['p_d_pp'][:, 0], tau_init_robot])] * (N_horizon-1)

    tic()
    for i in range(N_traj):
        problem = ocp_problem_v3(i, i+N_horizon, N_step, state, xk, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type = 'euler', int_steps=1)
        ddp = cro.SolverDDP(problem)
        # ddp.setCallbacks([cro.CallbackVerbose()])
        hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)

        us[i] = ddp.us[0]
        xs[i] = xk

        xs_init_guess = ddp.xs
        us_init_guess = ddp.us

        # TODO: Simulation
        tau_k = us[i, 3:]
        q = xk[6:6+nq]
        q_p = xk[6+nq:6+nx]

        q_pp     = pin.aba(robot_model, robot_data, q, q_p, tau_k)
        q_p_next = q_p + dt * q_pp # Euler method
        q_next   = pin.integrate(robot_model, q, dt * q_p_next)

        xk[0:6]      = np.hstack([param_trajectory['p_d'][:, i+1], param_trajectory['p_d_p'][:, i+1]])
        xk[6:6+nq]   = q_next
        xk[6+nq:6+nx] = q_p_next
    toc()

    param_trajectory['p_d']    = param_trajectory['p_d'][   :, 0:N_traj]
    param_trajectory['p_d_p']  = param_trajectory['p_d_p'][ :, 0:N_traj]
    param_trajectory['p_d_pp'] = param_trajectory['p_d_pp'][:, 0:N_traj]
else:
    print("\033[91mUnknown opt_type\033[0m")
    print("\033[91mChosen option:\033[0m", opt_type)
    print("\033[91mAllowed options:\033[0m 'OCP' or 'MPC'")
    exit()

robot_data = robot_model.createData()

xs_T = xs[-1]
pin.forwardKinematics(robot_model, robot_data, xs_T[: state.nq])
pin.updateFramePlacements(robot_model, robot_data)
print(
    "\nFinally reached = ",
    robot_data.oMf[TCP_frame_id].translation.T,
)

#print("\nTotal cost:", ddp.cost)
print("Feasibility:", ddp.isFeasible)
print("Minimum Found:", hasConverged)

plot_sol=True
if plot_sol:
    plot_solution(us, xs, t, TCP_frame_id, robot_model, param_trajectory)

y_ref = xs[:, 0:3]
max_err = np.max(np.abs(y_ref - param_trajectory['p_d'].T), axis=0)
print(max_err)

visualize=False
if visualize==True:
    visualize_robot(robot, xs, param_trajectory, dt, 3, 1)