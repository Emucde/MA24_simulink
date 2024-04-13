import os
import numpy as np
import crocoddyl as cro
import pinocchio as pin
import time

mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files')
urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', '2dof_sys.urdf')

#model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

# Now load the model (using pinocchio)
robot = pin.robot_wrapper.RobotWrapper.BuildFromURDF(str(urdf_model_path))

# Gravity should be in -y direction
robot.model.gravity.linear[:] = [0, -9.81, 0]

# The model loaded from urdf (via pinicchio)
print(robot.model)

# Create a multibody state from the pinocchio model.
state = cro.StateMultibody(robot.model)

# OPT Prob
dt = 1e-3  # Time step
T = 1000  # Number of knots

# Cost models
runningCostModel = cro.CostModelSum(state, nu=2)
terminalCostModel = cro.CostModelSum(state, nu=2)

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
import matplotlib.pyplot as plt

fig = plt.gcf()
axs = fig.axes
for ax in axs:
    ax.grid()

#plt.tight_layout()  # Adjust subplots to prevent overlap
#plt.show()


# Now stabilize the acrobot using FDDP
solver = cro.SolverFDDP(problem)

# Solve
callbacks = []
callbacks.append(cro.CallbackLogger())
callbacks.append(cro.CallbackVerbose())
solver.setCallbacks(callbacks)
solver.solve([], [], 300, False, 1e-5)

# Display using meshcat
robot_display = cro.MeshcatDisplay(robot, -1, 1, False)
#robot_display.robot.viewer.jupyter_cell()
robot_display.displayFromSolver(solver)

# Display using gepetto-gui
if False:
    robot_display = cro.GepettoDisplay(robot, floor=False)
    robot_display.displayFromSolver(solver)


# Plotting the solution and the DDP convergence
log = solver.getCallbacks()[0]

import matplotlib.pyplot as plt

cro.plotOCSolution(
    xs=log.xs, us=log.us, show=False, figIndex=1, figTitle="Solution"
)
fig = plt.gcf()
axs = fig.axes
for ax in axs:
    ax.grid(True)

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

# vid endless loop
while True:
    time.sleep(1.0)
    robot_display.displayFromSolver(solver)