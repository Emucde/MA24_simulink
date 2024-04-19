import os
import sys
import numpy as np
import crocoddyl as cro
import pinocchio as pin
import time
import matplotlib.pyplot as plt
import meshcat.geometry as g
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

# Calculate strechted position
qT = np.array([0,0])
robot_data = robot_model.createData()
pin.forwardKinematics(robot_model, robot_data, qT)
pin.updateFramePlacements(robot_model, robot_data)
TCP_frame_id = robot_model.getFrameId('TCP')
xeT = robot_data.oMf[TCP_frame_id].translation.T.copy()
xe0 = np.array([0.1, xeT[1], 0])

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
#state = CustomState(robot_model)
#state = CustomState(urdf_model_path)
# TODO: 1. Abstract State Model so dass es 1:1 funktioniert
#       2. States erweitern.
#       3. MPC aufbauen
#state = ExtendedDynamicsModel(robot, 1)
#state.lb[0:2] = -np.pi
#state.ub[0:2] = np.pi

def my_dynamics(state, input):
    z1, z2 = state
    v = input
    return np.array([z2, v])

'''model = pin.Model()
model.dynamics = my_dynamics
geom_model = pin.GeometryModel()
joint_placement = pin.SE3.Identity()
joint_name = "joint_spherical"
parent_id = 0
joint_id = model.addJoint(parent_id, pin.JointModelRevoluteUnaligned(), joint_placement, "joint_revolute1")
parent_id = 1
joint_id = model.addJoint(parent_id, pin.JointModelRevoluteUnaligned(), joint_placement, "joint_revolute1")

data = model.createData()

q_p = np.array([0,0])
q = np.array([0,0])
q_T = np.array([0,0])
q_p_0 = np.array([0,0])
tau0 = np.array([0,0])
pin.forwardKinematics(model, data, q_T, q_p_0)
pin.aba(model, data, q, q_p, tau0)
'''

dt = 1e-3  # Time step

# Generate Trajectory
T_start = 0
T_end = 10

N_traj = int((T_end-T_start)/dt)
N = N_traj  # Number of knots

# t = 0 : param_global.Ta : T_sim + T_horizon_max;
t = np.linspace(T_start, T_end, N_traj)

# traj_data = np.concatenate((np.linspace(xe0, xeT, N_traj//2), np.linspace(xeT, xe0, N_traj//2)))

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

traj_data = param_trajectory['p_d'].T


# yy_DAM = DifferentialActionModelPinocchio()
# yy_ND = crocoddyl.DifferentialActionModelNumDiff(yy_DAM, True)
# yy_NDIAM = crocoddyl.IntegratedActionModelEuler(yy_ND, dt)

# data = yy_NDIAM.createData()
# yy_NDIAM.calc(data, np.array([0,0,0, 0,0,0]), np.array([0,0,0, 0,0,0, 0,0,0, 0,0,0]))


y_d_data    = param_trajectory['p_d'].T
y_d_p_data  = param_trajectory['p_d_p'].T
y_d_pp_data = param_trajectory['p_d_pp'].T

plot_traj=False
if plot_traj:
    plt.figure(figsize=(10, 6))  # Adjust figure size as needed

    plt.plot(y_d_data, label='y_d')
    plt.plot(y_d_p_data, label='y_d_p')
    plt.plot(y_d_pp_data, label='y_d_pp')

    # Add labels and title
    plt.xlabel('Time Step (Assuming data represents time series)')
    plt.ylabel('Data Value')
    plt.title('Plot of y_d, y_d_p, and y_d_pp Data')

    # Add legend
    plt.legend()

    # Show the plot
    plt.grid(True)
    plt.tight_layout()
    plt.show()

q0 = res.x
#x0 = np.concatenate([q0, pin.utils.zero(state.nv)])
# x0 = np.concatenate([q0, pin.utils.zero(state.nv), y_d_data[0], y_d_p_data[0]])
x0 = np.concatenate([y_d_data[0], y_d_p_data[0]])


# OPT Prob

running_cost_models = list()
terminate_cost_models = list()

actuationModel = cro.ActuationModelFull(state)

# Summenkosten
for i in range(N):
    y_d    = y_d_data[i]
    y_d_p  = y_d_p_data[i]
    y_d_pp = y_d_pp_data[i]

    yy_DAM = DifferentialActionModelPinocchio(y_d, y_d_p, y_d_pp)
    
    # data = yy_DAM.createData()
    # tic()
    # for j in range(0,10000):
    #     yy_DAM.calcDiff(data, x0, np.zeros(3))
    # toc()
    # quit()

    # yy_ND = crocoddyl.DifferentialActionModelNumDiff(yy_DAM, True) # langsam, nicht mehr notw.
    # yy_NDIAM = crocoddyl.IntegratedActionModelEuler(yy_ND, dt)
    yy_NDIAM = crocoddyl.IntegratedActionModelEuler(yy_DAM, dt)
    
    runningCostModel = cro.CostModelSum(state)

    goalTrackingCost = cro.CostModelResidual(
        state,
        cro.ResidualModelFrameTranslation(
            state, TCP_frame_id, traj_data[i]
        ),
    )
    xRegCost = cro.CostModelResidual(state, cro.ResidualModelState(state))
    uRegCost = cro.CostModelResidual(state, cro.ResidualModelControl(state))

    if i < N-1:
        runningCostModel.addCost("TCP_pose", goalTrackingCost, 1e5)
        runningCostModel.addCost("stateReg", xRegCost, 1e-5)
        runningCostModel.addCost("ctrlReg", uRegCost, 1e-2)
        #CombinedActionModel

        # running_cost_models.append(CombinedActionModel(yy_NDIAM, cro.IntegratedActionModelEuler(
        #     cro.DifferentialActionModelFreeFwdDynamics(
        #         state, actuationModel, runningCostModel
        #     ),
        #     dt,
        # )))

        running_cost_models.append(yy_NDIAM)

        # running_cost_models.append(cro.IntegratedActionModelEuler(
        #    cro.DifferentialActionModelFreeFwdDynamics(
        #        state, actuationModel, runningCostModel
        #    ),
        #    dt,
        # ))        
        
        # running_cost_models.append(cro.IntegratedActionModelEuler(
        #    DifferentialFwdDynamics(
        #        state, runningCostModel
        #    ),
        #    dt,
        # ))

        # running_cost_models.append(cro.IntegratedActionModelEuler(
        #    DifferentialFwdDynamics3(
        #        state, actuationModel, runningCostModel
        #    ),
        #    dt,
        # ))
    else: # i == N: # Endkostenterm
        terminalCostModel = cro.CostModelSum(state)
        terminalCostModel.addCost("TCP_pose", goalTrackingCost, 1e10)
        #terminalCostModel.addCost("stateReg", xRegCost, 1e0)
        #terminalCostModel.addCost("ctrlReg", uRegCost, 1e0)

        # terminate_cost_models.append(CombinedActionModel(yy_NDIAM, cro.IntegratedActionModelEuler(
        #     cro.DifferentialActionModelFreeFwdDynamics(
        #         state, actuationModel, terminalCostModel
        #     )
        # )))

        terminate_cost_models.append(yy_NDIAM)

        # terminate_cost_models.append(cro.IntegratedActionModelEuler(
        #     cro.DifferentialActionModelFreeFwdDynamics(
        #         state, actuationModel, terminalCostModel
        #     ),
        #     dt
        # ))

        # terminate_cost_models.append(cro.IntegratedActionModelEuler(
        #     DifferentialFwdDynamics(
        #        state, terminalCostModel
        #     ),
        #     dt
        # ))

        # terminate_cost_models.append(cro.IntegratedActionModelEuler(
        #     DifferentialFwdDynamics3(
        #         state, actuationModel, terminalCostModel
        #     ),
        #     dt
        # ))

# Create the shooting problem
seq = running_cost_models
problem = cro.ShootingProblem(x0, seq, terminate_cost_models[0])

# Creating the DDP solver for this OC problem, defining a logger
ddp = cro.SolverDDP(problem)

# IV. Callbacks
ddp.setCallbacks([cro.CallbackVerbose()])

tic()
hasConverged = ddp.solve([], [], 50, False, 1e-5)
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

    for i in range(len(xs)):
        #q = xs[i, :robot_model.nq]
        # q = xs[i, 6:6+robot_model.nq]
        # #v = xs[i, robot_model.nq:robot_model.nq+robot_model.nv]
        # pin.forwardKinematics(robot_model, robot_data, q)
        # pin.updateFramePlacements(robot_model, robot_data)
        # y_opt[i] = robot_data.oMf[TCP_frame_id].translation.T
        y_ref[i] = xs[i, 0:3]

    y_target = traj_data

    labels = ['yd_x', 'yd_y', 'yd_z']
    labelsref = ['yref_x', 'yref_y', 'yref_z']

    # Plotten der x-Komponenten
    for i in range(3):
        # axs[i].plot(t, y_opt[:,i], label=f'yopt[{i}]: {labels[i]} coordinate')
        axs[i].plot(t, y_ref[:, i], linestyle='-', label=f'yref[{i}]: {labelsref[i]} coordinate')
        axs[i].plot(t, y_target[:, i], linestyle='--', label=f'yd[{i}]: {labels[i]} coordinate')
        axs[i].set_xlabel('t (s)')
        axs[i].set_ylabel('TCP (Gripper) position (m)')
        axs[i].legend()

    plt.tight_layout()
    plt.show()

max_err = np.max(y_ref - y_target, axis=0)
print(max_err)

visualize=False
if visualize==True:
    # Meshcat Visualize
    robot_display = cro.MeshcatDisplay(robot, -1, 1, False)

    for i, target in enumerate(traj_data[::int(N_traj/100)]):
        robot_display.robot.viewer["target_" + str(i)].set_object(g.Sphere(1e-3))
        Href = np.array(
            [
                [1.0, 0.0, 0.0, target[0]],
                [0.0, 1.0, 0.0, target[1]],
                [0.0, 0.0, 1.0, target[2]],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        robot_display.robot.viewer["target_" + str(i)].set_transform(
            np.array(
                [
                    [1.0, 0.0, 0.0, target[0]],
                    [0.0, 1.0, 0.0, target[1]],
                    [0.0, 0.0, 1.0, target[2]],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
        )

    i = 0
    cnt=3# np.inf
    while i < cnt:
        time.sleep(1.0)
        robot_display.displayFromSolver(ddp)
        i = i +1
    
    create_video=False
    if create_video:
        with robot_display.robot.viz.create_video_ctx("test.mp4"):
            robot_display.robot.viz.play(xs[:,:robot_model.nq], dt)
'''
# Cost models
runningCostModel  = cro.CostModelSum(state, nu=2)
terminalCostModel = cro.CostModelSum(state, nu=2)

actuationModel = cro.ActuationModelFull(state)

# Add a cost for the configuration positions and velocities
xref = np.array([-np.pi/4, np.pi/2, 0, 0])  # Desired state
stateResidual = cro.ResidualModelState(state, xref=xref, nu=2)
stateCostModel = cro.CostModelResidual(state, stateResidual)
runningCostModel.addCost("state_cost", cost=stateCostModel, weight=1e-1 / dt)
terminalCostModel.addCost("state_cost", cost=stateCostModel, weight=1e3)

# Add a cost on control
controlResidual = cro.ResidualModelControl(state, nu=2)
bounds = cro.ActivationBounds(np.array([-1.0, -1.0]), np.array([1.0, 1.0]))
activation = cro.ActivationModelQuadraticBarrier(bounds)
controlCost = cro.CostModelResidual(
    state, activation=activation, residual=controlResidual
)
#runningCostModel.addCost("control_cost", cost=controlCost, weight=1e3 / dt)

# Create the action models for the state
actuationModel = cro.ActuationModelFull(state)
runningModel = cro.IntegratedActionModelEuler(
    cro.DifferentialActionModelFreeFwdDynamics(
        state, actuationModel, runningCostModel
    ),
    dt,
)
terminalModel = cro.IntegratedActionModelEuler(
    cro.DifferentialActionModelFreeFwdDynamics(
        state, actuationModel, terminalCostModel
    ),
    0.0,
)

# Define a shooting problem
q0 = np.zeros((state.nq,))  # Inital joint configurations
q0[0] = np.pi / 2  # Up
v0 = np.zeros((state.nv,))  # Initial joint velocities
x0 = np.concatenate((q0, v0))  # Inital robot state
problem = cro.ShootingProblem(x0, [runningModel] * T, terminalModel)


# Test the problem with a rollout
us = [0.01 * np.ones((2,))] * T
xs = problem.rollout(us)

# Handy to blat up the state and control trajectories
cro.plotOCSolution(xs, us, show=False, figIndex=99, figTitle="Test rollout")

# Put a grid on the plots
fig = plt.gcf()
axs = fig.axes
for ax in axs:
    ax.grid()

plt.tight_layout()  # Adjust subplots to prevent overlap
plt.show()


# Now stabilize the acrobot using FDDP
solver = cro.SolverFDDP(problem)

# Solve
callbacks = []
callbacks.append(cro.CallbackLogger())
callbacks.append(cro.CallbackVerbose())
solver.setCallbacks(callbacks)
solver.solve([], [], 300, False, 1e-5)

# Plotting the solution and the DDP convergence
log = solver.getCallbacks()[0]

cro.plotOCSolution(
    xs=log.xs, us=log.us, show=False, figIndex=1, figTitle="Solution"
)
fig = plt.gcf()
axs = fig.axes
for ax in axs:
    ax.grid(True)

plot_convergence=False
if plot_convergence:
    cro.plotConvergence(
        log.costs,
        log.pregs,
        log.dregs,
        log.grads,
        log.stops,
        log.steps,
        show=False,
        figIndex=2,
    )
    fig = plt.gcf()
    axs = fig.axes
    for ax in axs:
        ax.grid(True)

plt.show()

show_meshcat=False
if show_meshcat:
    # Display using meshcat
    robot_display = cro.MeshcatDisplay(robot, -1, 1, False)
    i = 0
    cnt=3# np.inf
    while i < cnt:
        time.sleep(1.0)
        robot_display.displayFromSolver(solver)
        i = i +1
'''