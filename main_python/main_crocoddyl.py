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
y_offset = 0

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
    pin.forwardKinematics(robot_model, robot_data, q)
    pin.updateFramePlacements(robot_model, robot_data)
    xe = robot_data.oMf[TCP_frame_id].translation.T
    return 1/2 * (xe - xe0) @ (xe - xe0)

bounds = ((-np.pi, np.pi), (-np.pi, np.pi))
qq0 = [1, 1] # Define the initial guess - avoid to start in singularity!!
res = minimize(f_cost_forward, qq0, method='SLSQP', bounds=bounds, options={'ftol': 1e-20, 'disp': False})

# Create a multibody state from the pinocchio model.
state = cro.StateMultibody(robot_model)
state.lb[0:2] = -np.pi
state.ub[0:2] = np.pi

state.lb[2:4] = -100
state.ub[2:4] = 100

dt = 10e-3  # Time step

# Generate Trajectory
T_start = 0
T_end = 10

N_traj = int((T_end-T_start)/dt)
N = N_traj  # Number of knots

# t = 0 : param_global.Ta : T_sim + T_horizon_max;
t = np.linspace(T_start, T_end, N_traj)

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

param_traj_poly = {}
param_traj_poly['T'] = T_end/2-3

param_trajectory = generate_trajectory(t, xe0, xeT, R_init, rot_ax, rot_alpha_scale, T_start, T_end, param_traj_poly)

plot_traj=False
if plot_traj:
    plot_trajectory(param_trajectory)

q0 = res.x

# OPT Prob

problem = ocp_problem_v3(state, q0, TCP_frame_id, param_trajectory, dt)

# Creating the DDP solver for this OC problem, defining a logger
ddp = cro.SolverDDP(problem)

# IV. Callbacks
ddp.setCallbacks([cro.CallbackVerbose()])

tic()
hasConverged = ddp.solve([], [], 300, False, 1e-5)
toc()

robot_data = robot_model.createData()
xT = ddp.xs[-1]
pin.forwardKinematics(robot_model, robot_data, xT[: state.nq])
pin.updateFramePlacements(robot_model, robot_data)
print(
    "\nFinally reached = ",
    robot_data.oMf[TCP_frame_id].translation.T,
)

print("\nTotal cost:", ddp.cost)
print("Feasibility:", ddp.isFeasible)
print("Minimum Found:", hasConverged)

plot_sol=True
if plot_sol:
    fig, axs = plt.subplots(3, 1, figsize=(10, 10))

    robot_data = robot_model.createData()
    xs = np.array(ddp.xs)
    y_opt = np.zeros((len(xs), 3))
    y_ref = np.zeros((len(xs), 3))
    t = np.linspace(T_start, T_end, N_traj)

    for i in range(len(xs)):
        q = xs[i, 6:6+robot_model.nq]
        #v = xs[i, robot_model.nq:robot_model.nq+robot_model.nv]
        pin.forwardKinematics(robot_model, robot_data, q)
        pin.updateFramePlacements(robot_model, robot_data)
        y_opt[i] = robot_data.oMf[TCP_frame_id].translation.T.copy()
        y_ref[i] = xs[i, 0:3]

    y_target = param_trajectory['p_d'].T

    labels = ['yd_x', 'yd_y', 'yd_z']
    labelsref = ['yref_x', 'yref_y', 'yref_z']

    # Plotten der x-Komponenten
    for i in range(3):
        axs[i].plot(t, y_opt[:,i], label=f'yopt[{i}]: {labels[i]} coordinate')
        axs[i].plot(t, y_ref[:, i], linestyle='-', label=f'yref[{i}]: {labelsref[i]} coordinate')
        axs[i].plot(t, y_target[:, i], linestyle='--', label=f'yd[{i}]: {labels[i]} coordinate')
        axs[i].set_xlabel('t (s)')
        axs[i].set_ylabel('TCP (Gripper) position (m)')
        axs[i].legend()

    plt.tight_layout()
    plt.show()

max_err = np.max(np.abs(y_ref - y_target), axis=0)
print(max_err)

visualize=True
if visualize==True:
    visualize_robot(robot, xs, param_trajectory, dt, 3, 1)