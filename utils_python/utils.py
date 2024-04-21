import numpy as np
from scipy.spatial.transform import Rotation
import time
import crocoddyl
import pinocchio
import matplotlib.pyplot as plt
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation
from pinocchio.visualize import MeshcatVisualizer
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.offline as py

def trajectory_poly(t, y0, yT, T):
    # The function plans a trajectory from y0 in R3 to yT in R3 and returns the desired position and velocity on this trajectory at time t.
    # The trajectory starts at t=0 with y_d(0) = y0 and dy_d(0) = 0 and reaches y_d(T)=yT and dy_d(T) = 0 at t=T.
    # The trajectory moves along the straight interconnection between y0 and yT in R3.

    # INPUTs:
    # t: current time
    # T: final time
    # y0: 3x1 vector of the initial position
    # yT: 3x1 vector of the final position

    # RETURN:
    # y_d: 3x1 vector of the desired position at time t
    # dy_d: 3x1 vector of the desired velocity at time t
    # ddy_d: 3x1 vector of the desired acceleration at time t

    y_d   = y0 + (6 / T ** 5 * t ** 5 - 15 / T ** 4 * t ** 4 + 10 * t ** 3 / T ** 3)  * (yT - y0)
    dy_d  =      (30 / T ** 5 * t ** 4 - 60 / T ** 4 * t ** 3 + 30 * t ** 2 / T ** 3) * (yT - y0)
    ddy_d =      (120 / T ** 5 * t ** 3 - 180 / T ** 4 * t ** 2 + 60 * t / T ** 3)    * (yT - y0)

    return y_d, dy_d, ddy_d

def create_poly_traj(x_target, x0_target, T_start, t, R_init, rot_ax, rot_alpha_scale, param_traj_poly):
    T = param_traj_poly['T']
    
    if t - T_start > T:
        p_d = np.concatenate((x_target[:3], [1]))
        p_d_p = np.concatenate((np.zeros(3), [1]))
        p_d_pp = np.concatenate((np.zeros(3), [1]))
    else:
        yT = np.concatenate((x_target[:3], [1]))  # poly contains [x, y, z, alpha]
        y0 = np.concatenate((x0_target[:3], [0]))
        p_d, p_d_p, p_d_pp = trajectory_poly(t - T_start, y0, yT, T)
    
    alpha = p_d[3]
    alpha_p = p_d_p[3]
    alpha_pp = p_d_pp[3]
    
    skew_ew = np.array([[0, -rot_ax[2], rot_ax[1]],
                        [rot_ax[2], 0, -rot_ax[0]],
                        [-rot_ax[1], rot_ax[0], 0]])
    
    R_act = R_init @ (np.eye(3) + np.sin(rot_alpha_scale * alpha) * skew_ew + (1 - np.cos(rot_alpha_scale * alpha)) * skew_ew @ skew_ew)
    
    x_d = {}
    x_d['p_d'] = p_d[:3]
    x_d['p_d_p'] = p_d_p[:3]
    x_d['p_d_pp'] = p_d_pp[:3]
    x_d['q_d'] = Rotation.from_matrix(R_act).as_quat()
    x_d['omega_d'] = alpha_p * rot_ax
    x_d['omega_d_p'] = alpha_pp * rot_ax
    
    return x_d

def generate_trajectory(t, xe0, xeT, R_init, rot_ax, rot_alpha_scale, T_start, T_end, param_traj_poly):
    N = len(t)
    p_d = np.zeros((3, N))
    p_d_p = np.zeros((3, N))
    p_d_pp = np.zeros((3, N))
    q_d = np.zeros((4, N))
    omega_d = np.zeros((3, N))
    omega_d_p = np.zeros((3, N))

    T_start_end = T_end/2
    T_start = 0
    flag = 0

    for i in range(N):
        x_d = create_poly_traj(xeT, xe0, T_start, t[i], R_init, rot_ax, rot_alpha_scale, param_traj_poly)
        p_d[:, i]       = x_d['p_d']
        p_d_p[:, i]     = x_d['p_d_p']
        p_d_pp[:, i]    = x_d['p_d_pp']
        q_d[:, i]       = x_d['q_d']
        omega_d[:, i]   = x_d['omega_d']
        omega_d_p[:, i] = x_d['omega_d_p']
        if t[i] >= T_start_end and flag == 0:
            temp = xe0
            xe0  = xeT
            xeT  = temp
            T_start = T_start_end
            flag = 1

    param_trajectory = {'p_d': p_d, 'p_d_p': p_d_p, 'p_d_pp': p_d_pp, 'q_d': q_d, 'omega_d': omega_d, 'omega_d_p': omega_d_p}

    return param_trajectory

def tic():
    global start_time
    start_time = time.time()

def toc():
    if 'start_time' in globals():
        elapsed_time = time.time() - start_time
        print(f"Elapsed time: {elapsed_time} seconds")
    else:
        print("Call tic() before calling toc()")

def block_diag_onlydiagmatr(a, b):
    return np.kron(a, b)

def block_diag(a, b):
    an, am = a.shape
    bn, bm = b.shape
    return np.block([[a, np.zeros((an, bm))], [np.zeros((bn, am)), b]])

###################### HERE ########################################################
class DifferentialActionModelPinocchio(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self, y_d, y_d_p, y_d_pp):
        self.m = 3
        self.n1 = 3
        self.n2 = 3
        self.n = 6

        crocoddyl.DifferentialActionModelAbstract.__init__(
            self, crocoddyl.StateVector(2*self.m), self.m, self.m
        )

        self.y_d    = y_d
        self.y_d_p  = y_d_p
        self.y_d_pp = y_d_pp

        self.Kd = 100*np.eye(self.m)
        self.Kp = 2500*np.eye(self.m)

        self.Kp_squarex2 = 2*self.Kp**2
        self.Kd_squarex2 = 2*self.Kd**2

        self.I = np.eye(self.m)
        self.Ix2 = 2*self.I # es ist wirklich schneller so

    def calc(self, data, x, u):
        n1 = self.n1
        n = self.n

        z1 = x[:n1]
        z2 = x[n1:n]

        alpha  = u

        data.xout = alpha # = z1ddot

        data.r = (alpha - self.y_d_pp) + self.Kd @ (z2 - self.y_d_p) + self.Kp @ (z1 - self.y_d)
        data.cost = np.linalg.norm(data.r)**2
        # data.cost = np.sum(data.r**2) # slower than linalg

    def calcDiff(self, data, x, u):
        n1 = self.n1
        n = self.n
        z1 = x[:n1]
        z2 = x[n1:n]

        Kp_squarex2 = self.Kp_squarex2
        Kd_squarex2 = self.Kd_squarex2

        # data.Fx = np.block([O, O]) # f = d^2/dt^2 z2 = u # ist eh standardmäßig 0
        data.Fu = self.I

        #data.Lx  = np.block([2*Kp_square @ (z1 - self.y_d), 2*Kd_square @ (z2 - self.y_d_p)])
        data.Lx[:n1]  = Kp_squarex2 @ (z1 - self.y_d)
        data.Lx[n1:n] = Kd_squarex2 @ (z2 - self.y_d_p)

        data.Lu = 2*(u - self.y_d_pp)
        
        #data.Lxx = np.block([[2*Kp_square, O], [O, 2*Kd_square]])
        data.Lxx[:n1, :n1]    = Kp_squarex2
        data.Lxx[n1:n:, n1:n] = Kd_squarex2
        data.Luu = self.Ix2
        #data.Lxu = np.block([[O],[O]]) # is eh standardmäßig 0

class CombinedActionModel(crocoddyl.ActionModelAbstract):
    def __init__(self, model1, model2):
        crocoddyl.ActionModelAbstract.__init__(self, crocoddyl.StateVector(model1.state.nx + model2.state.nx), model1.nu + model2.nu, model1.nr + model2.nr)
        self.model1 = model1
        self.model2 = model2 # model 2 should be robot model!!

        self.data1 = self.model1.createData()
        self.data2 = self.model2.createData()

        self.mu_model1 = self.model1.nu
        self.mu_model2 = self.model2.nu
        self.m = self.mu_model1 + self.mu_model2

        self.nx_model1 = self.model1.state.nx
        self.nx_model2 = self.model2.state.nx
        self.n = self.nx_model1 + self.nx_model2

    def calc(self, data, x, u):
        m1 = self.mu_model1
        m = self.m

        n1 = self.nx_model1
        n = self.n

        yref = x[:m1]
        self.model2.differential.costs.costs["TCP_pose"].cost.residual.reference = yref
        # self.model2.differential.costs.costs["TCP_pose"].cost.residual.reference = self.model1.differential.y_d

        self.model1.calc(self.data1, x[:n1],  u[:m1] )
        self.model2.calc(self.data2, x[n1:n], u[m1:m])

        data.xnext = np.hstack([self.data1.xnext, self.data2.xnext])
        data.cost  = self.data1.cost + self.data2.cost

    def calcDiff(self, data, x, u):
        m1 = self.mu_model1
        m = self.m

        n1 = self.nx_model1
        n = self.n

        yref = x[:m1]
        self.model2.differential.costs.costs["TCP_pose"].cost.residual.reference = yref
        # self.model2.differential.costs.costs["TCP_pose"].cost.residual.reference = self.model1.differential.y_d
        
        self.model1.calcDiff(self.data1, x[0:n1],  u[0:m1])
        self.model2.calcDiff(self.data2, x[n1:n],  u[m1:m])

        data.Fx[:n1, :n1]    = self.data1.Fx
        data.Fx[n1:n, n1:n]  = self.data2.Fx

        data.Fu[:n1, :m1]    = self.data1.Fu
        data.Fu[n1:n, m1:m]  = self.data2.Fu

        data.Lx[:n1]         = self.data1.Lx
        data.Lx[n1:n]        = self.data2.Lx
        
        data.Lu[:m1]         = self.data1.Lu
        data.Lu[m1:m]        = self.data2.Lu

        data.Lxx[:n1, :n1]   = self.data1.Lxx
        data.Lxx[n1:n, n1:n] = self.data2.Lxx

        data.Luu[:m1, :m1]   = self.data1.Luu
        data.Luu[m1:m, m1:m] = self.data2.Luu

        data.Lxu[:n1, :m1]   = self.data1.Lxu
        data.Lxu[n1:n, m1:m] = self.data2.Lxu

############### Beide Klassen sind equivalent zu crocoddyl.DifferentialActionModelFreeFwdDynamics: ################
# Ausführgeschwindigkeit: 140s (opt problem mit 10000 punkten)
class DifferentialFwdDynamics(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self, state, costModel):
        crocoddyl.DifferentialActionModelAbstract.__init__(
            self, state, state.nv, costModel.nr
        )
        self.costs = costModel

    def calc(self, data, x, u):
        q, v = x[: self.state.nq], x[-self.state.nv :]
        # Computing the dynamics using ABA or manually for armature case
        data.xout = pinocchio.aba(self.state.pinocchio, data.pinocchio, q, v, u)
        pinocchio.forwardKinematics(self.state.pinocchio, data.pinocchio, q, v)
        pinocchio.updateFramePlacements(self.state.pinocchio, data.pinocchio)
        self.costs.calc(data.costs, x, u)
        data.cost = data.costs.cost

    def calcDiff(self, data, x, u=None):
        q, v = x[: self.state.nq], x[-self.state.nv :]
        #self.calc(data, x, u) # bringt nix

        ## Computing the dynamics derivatives
        pinocchio.computeABADerivatives(
            self.state.pinocchio, data.pinocchio, q, v, u
        )
        data.Fx = np.hstack([data.pinocchio.ddq_dq, data.pinocchio.ddq_dv])
        data.Fu = data.pinocchio.Minv
        # Computing the cost derivatives
        self.costs.calcDiff(data.costs, x, u)

    def createData(self):
        data = crocoddyl.DifferentialActionModelAbstract.createData(self)
        data.pinocchio = pinocchio.Data(self.state.pinocchio)
        data.multibody = crocoddyl.DataCollectorMultibody(data.pinocchio)
        data.costs = self.costs.createData(data.multibody)
        data.costs.shareMemory(
            data
        )  # this allows us to share the memory of cost-terms of action model
        return data

# Ist 2-3 mal schneller als obige Klasse in der die Crocoddyl Funktionen direkt aufgerufen werden. (50s, (opt problem mit 10000 punkten))
# Diese Klasse ist identisch schnell wie crocoddyl.DifferentialActionModelFreeFwdDynamics
class DifferentialFwdDynamics2(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self, state, actuationModel, costModel):
        crocoddyl.DifferentialActionModelAbstract.__init__( # das wird ein problem sein, da ich so die dimension nicht verändern kann.
            self, state, state.nv, costModel.nr# besser: crocoddyl.ActionModelAbstract.__init__(self, crocoddyl.StateVector(model1.state.nx + model2.state.nx), model1.nu + model2.nu, model1.nr + model2.nr)
        )
        self.DAM_free = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            state, actuationModel, costModel
        )

    def calc(self, data, x, u):
        self.DAM_free.calc(data, x, u)

    def calcDiff(self, data, x, u=None):
        self.DAM_free.calcDiff(data, x, u)

    def createData(self):
        data = self.DAM_free.createData()
        return data
###################################################################################################################

def plot_trajectory(param_trajectory):
    plt.figure(figsize=(10, 6))  # Adjust figure size as needed

    plt.plot(param_trajectory['p_d'].T, label='y_d')
    plt.plot(param_trajectory['p_d_p'].T, label='y_d_p')
    plt.plot(param_trajectory['p_d_pp'].T, label='y_d_pp')

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

def plot_solution(us, xs, t, TCP_frame_id, robot_model, param_trajectory, save_plot=False, file_name='plot_saved'):
    robot_data = robot_model.createData()

    line_dict = dict(width=1)
    line_dict_dot = dict(width=1, dash='dot')

    n = robot_model.nq

    y_opt    = np.zeros((len(xs), 3))
    y_opt_p  = np.zeros((len(xs), 3))
    y_opt_pp = np.zeros((len(xs), 3))
    q        = np.zeros((len(xs), n))
    q_p      = np.zeros((len(xs), n))
    q_pp     = np.zeros((len(xs), n))
    w        = np.zeros(len(xs)) # manipulability

    y_ref    = xs[:, 0:3]
    y_p_ref  = xs[:, 3:6]
    y_pp_ref = us[:, 0:3]

    y_d    = param_trajectory['p_d'].T
    y_d_p  = param_trajectory['p_d_p'].T
    y_d_pp = param_trajectory['p_d_pp'].T

    tau = np.concatenate([us[:, 3:3+n], [us[-1, 3:3+n]]]) # es wird ja ein u weniger erzeugt, da es nicht notw. ist

    for i in range(len(xs)):
        q[i]   = xs[i, 6:6+n]
        q_p[i] = xs[i, 6+n:6+2*n]
        q_pp[i] = pinocchio.aba(robot_model, robot_data, q[i], q_p[i], tau[i])

        pinocchio.forwardKinematics(robot_model, robot_data, q[i])
        pinocchio.updateFramePlacements(robot_model, robot_data)
        y_opt[i] = robot_data.oMf[TCP_frame_id].translation.T.copy()

        J = pinocchio.computeJointJacobians(robot_model, robot_data)
        # J = pinocchio.getFrameJacobian(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.WORLD)# pinocchio.LOCAL
        J_p = pinocchio.computeJointJacobiansTimeVariation(robot_model, robot_data, q[i], q_p[i])
        # dJ = pinocchio.getFrameJacobianTimeVariation(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.WORLD)

        J_v   = J[  0:3, 0:2] # mir translatorischen anteil
        J_v_p = J_p[0:3, 0:2] # z ist unnötig

        y_opt_p[i]  = J_v @ q_p[i]
        y_opt_pp[i] = J_v @ q_pp[i] + J_v_p @ q_p[i]

        w[i] = np.sqrt(np.linalg.det(J_v[0:2, 0:2] @ J_v[0:2, 0:2].T))

    e    = y_d    - y_opt
    e_p  = y_d_p  - y_opt_p
    e_pp = y_d_pp - y_opt_pp

    yd_labels    = ["$\\Large{x^d}$",             "$\\Large{y^d}$"           ]
    yd_labels_t    = ["x_d", "y_d"]

    y_opt_labels = ["$\\Large{x^\\mathrm{opt}}$",  "$\\Large{y^\\mathrm{opt}}$"]
    y_opt_labels_t = ["x_opt",  "y_opt",]

    y_ref_labels = ["$\\Large{x^{\\mathrm{ref}}}$", "$\\Large{y^{\\mathrm{ref}}}$"]
    y_ref_labels_t = ["x_ref", "y_ref"]

    tau_labels = ["$\\Large{\\tau_1}$", "$\\Large{\\tau_2}$"]
    tau_labels_t = ["tau_1", "tau_2"]

    e_x_labels = ["$\\Large{e_x}$", "$\\Large{\\dot{e}_x}$", "$\\Large{\\ddot{e}_x}$"]
    e_x_labels_t = ["e_x", "d/dt e_x", "d^2/dt^2 e_x"]

    e_y_labels = ["$\\Large{e_y}$", "$\\Large{\\dot{e}_y}$", "$\\Large{\\ddot{e}_y}$"]
    e_y_labels_t = ["e_y", "d/dt e_y", "d^2/dt^2 e_y"]

    q1_labels = ["$\\Large{q_1}$", "$\\Large{\\dot{q}_1}$", "$\\Large{\\ddot{q}_1}$"]
    q1_labels_t = ["q_1", "d/dt q_1", "d^2/dt^2 q_1"]

    q2_labels = ["$\\Large{q_2}$", "$\\Large{\\dot{q}_2}$", "$\\Large{\\ddot{q}_2}$"]
    q2_labels_t = ["q_2", "d/dt q_2", "d^2/dt^2 q_2"]

    # Create a Plotly subplot
    fig = make_subplots(rows=3, cols=4, shared_xaxes=True, vertical_spacing=0.05, 
        subplot_titles=("$\\mathrm{TCP~position~(m)}$",                                                "$e_x = x^d - x\\mathrm{ (m)}$",                          "$e_y = y^d - y\\mathrm{~(m)}$",                          "$\\mathrm{Joint~coordinates~}q\\mathrm{~(rad)}$", 
                        "$\\mathrm{Manipulability~}w = \\sqrt{\\det(\\mathbf{J}\\mathbf{J}^\\mathrm{T})}$", "$\\dot{e}_x = \\dot{x}^d - \\dot{x}\\mathrm{~(m/s)}$",      "$\\dot{e}_y = \\dot{y}^d - \\dot{y}\\mathrm{~(m/s)}$",      "$\\mathrm{Joint~velocities~}\\dot{q}\\mathrm{~(rad/s)}$",
                        "$\\mathrm{Torque~}\\boldsymbol{\\tau}~(Nm)$",                                 "$\\ddot{e}_x = \\ddot{x}^d - \\ddot{x}\\mathrm{~(m/s^2)}$", "$\\ddot{e}_y = \\ddot{y}^d - \\ddot{y}\\mathrm{~(m/s^2)}$", "$\\mathrm{Joint~acceleration~}\\ddot{q}\\mathrm{~(rad/s^2)}$"))
    
    # Plot the x-components using Plotly
    for i in range(2): # nur bis 2 wegen x und y, z ist unwichtig
        fig.add_trace(go.Scatter(x=t, y=y_opt[:, i], name=f'{y_opt_labels[i]}', line = line_dict, hoverinfo = 'x+y+text', hovertext=y_opt_labels_t[i]), row=1, col=1)
    for i in range(2):
        # fig.add_trace(go.Scatter(x=t, y=y_ref[:, i], name=f'yref[{i}]: {y_ref_labels[i]}', line = line_dict), row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=y_d[  :, i], name=f'{yd_labels[i]}'     , line = line_dict_dot, hoverinfo = 'x+y+text', hovertext=yd_labels_t[i]), row=1, col=1)
    for i in range(2):
        fig.add_trace(go.Scatter(x=t, y=tau[:, i], line = line_dict, name = tau_labels[i], hoverinfo = 'x+y+text', hovertext=tau_labels_t[i]), row=3, col=1)

    fig.add_trace(go.Scatter(x=t, y=w, line = line_dict, name='$w$', hoverinfo = 'x+y+text', hovertext="Manip w"), row=2, col=1)

    e_x_arr = [e[:, 0], e_p[:, 0], e_pp[:, 0]]
    e_y_arr = [e[:, 1], e_p[:, 1], e_pp[:, 1]]

    q_arr = [q, q_p, q_pp]

    for i in range(3):
        fig.add_trace(go.Scatter(x=t, y=e_x_arr[i], line = line_dict, name = e_x_labels[i], hoverinfo = 'x+y+text', hovertext=e_x_labels_t[i]), row=i+1, col=2)
    for i in range(3):
        fig.add_trace(go.Scatter(x=t, y=e_y_arr[i], line = line_dict, name = e_y_labels[i], hoverinfo = 'x+y+text', hovertext=e_y_labels_t[i]), row=i+1, col=3)
    for i in range(3):
        fig.add_trace(go.Scatter(x=t, y=q_arr[i][:, 0], line = line_dict, name = q1_labels[i], hoverinfo = 'x+y+text', hovertext=q1_labels_t[i]), row=i+1, col=4)
        fig.add_trace(go.Scatter(x=t, y=q_arr[i][:, 1], line = line_dict, name = q2_labels[i], hoverinfo = 'x+y+text', hovertext=q2_labels_t[i]), row=i+1, col=4)

    fig.update_xaxes(title_text='t (s)', row=3, col=1)
    fig.update_xaxes(title_text='t (s)', row=3, col=2)
    fig.update_xaxes(title_text='t (s)', row=3, col=3)


    # fig.update_layout(plot_bgcolor='#1e1e1e', paper_bgcolor='#1e1e1e', font=dict(color='#ffffff'), legend=dict(orientation='h'))
    fig.update_layout(
    plot_bgcolor='#101010',  # Set plot background color
    paper_bgcolor='#1e1e1e',  # Set paper background color
    font=dict(color='#ffffff'),  # Set font color
    legend=dict(orientation='h'),  # Set legend orientation
    hovermode = 'closest',
    # Gridline customization for all subplots
    **{f'xaxis{i}': dict(gridwidth=1, gridcolor='#757575', linecolor='#757575', zerolinecolor='#757575', zerolinewidth=1) for i in range(1, 13)},
    **{f'yaxis{i}': dict(gridwidth=1, gridcolor='#757575', linecolor='#757575', zerolinecolor='#757575', zerolinewidth=1) for i in range(1, 13)}
    )

    fig.show()

    if(save_plot):
        py.plot(fig, filename=file_name, include_mathjax='cdn')
    

def visualize_robot(robot, x_sol, param_trajectory, dt, rep_cnt = np.inf, rep_delay_sec=1):
    # Meshcat Visualize
    robot_model = robot.model
    robot_display = MeshcatVisualizer(robot.model)
    robot_display.initViewer(open=False)

    y_d_data = param_trajectory['p_d'].T

    # Display points:
    # N_traj = len(y_d_data)
    # for i, y_d in enumerate(y_d_data[::int(N_traj/100)]):
    #     robot_display.viewer["y_d_" + str(i)].set_object(g.Sphere(1e-3))
    #     Href = np.array(
    #         [
    #             [1.0, 0.0, 0.0, y_d[0]],
    #             [0.0, 1.0, 0.0, y_d[1]],
    #             [0.0, 0.0, 1.0, y_d[2]],
    #             [0.0, 0.0, 0.0, 1.0],
    #         ]
    #     )
    #     robot_display.viewer["y_d_" + str(i)].set_transform(Href)

    # Display trajectory as line:
    vertices = np.hstack([y_d_data.T[:, 0][:, np.newaxis], np.repeat(y_d_data.T[:,1::], 2, 1)]).astype(np.float32)
    robot_display.viewer["line_segments"].set_object(
            g.LineSegments(g.PointsGeometry(
                vertices, np.repeat(np.array([[255.0, 165.0, 0.0]], dtype=np.float32).T, len(vertices.T), 1)
                ), g.LineBasicMaterial(vertexColors=True))
        )

    anim = Animation()
    vis = robot_display.viewer

    obj1 = robot.visual_model.geometryObjects[0]
    obj2 = robot.visual_model.geometryObjects[1]
    obj3 = robot.visual_model.geometryObjects[2]

    testobj1 = g.StlMeshGeometry.from_file(obj1.meshPath)
    testobj2 = g.StlMeshGeometry.from_file(obj2.meshPath)
    testobj3 = g.StlMeshGeometry.from_file(obj3.meshPath)

    vis["testobj1"].set_object(testobj1)
    vis["testobj2"].set_object(testobj2)
    vis["testobj3"].set_object(testobj3)

    vis["testobj1"].set_property('scale', obj1.meshScale.tolist()) # other settings: https://github.com/meshcat-dev/meshcat
    vis["testobj2"].set_property('scale', obj2.meshScale.tolist())
    vis["testobj3"].set_property('scale', obj3.meshScale.tolist())

    vis["testobj1"].set_property('color', obj1.meshColor.tolist())
    vis["testobj2"].set_property('color', obj2.meshColor.tolist())
    vis["testobj3"].set_property('color', obj3.meshColor.tolist())

    vis["testobj1"].set_property('visible', False)
    vis["testobj2"].set_property('visible', False)
    vis["testobj3"].set_property('visible', False)

    robot_display.setCameraPose(np.eye(4))
    robot_display.setCameraPosition([0,0,1])

    robot_data = robot.model.createData()

    # Iterate through trajectory data (assuming x_sol[:, 6:6+robot_model.nq] represents joint positions)
    for i in range(len(x_sol)):
        pinocchio.forwardKinematics(robot_model, robot_data, x_sol[i, 6:6+robot_model.nq])
        pinocchio.updateFramePlacements(robot_model, robot_data)

        H_s1_1 = obj2.placement.homogeneous # = H_0^s1, CoM s1 of link 1 in dependence of inertial KOS
        H_s2_2 = obj3.placement.homogeneous # = H_0^s2, CoM s1 of link 2 in dependence of inertial KOS

        H_0_s1 = robot_data.oMf[robot_model.getFrameId('link1')].homogeneous # = H_s1^1, joint 1 in dependence of CoM s1 of link 1
        H_0_s2 = robot_data.oMf[robot_model.getFrameId('link2')].homogeneous # = H_s2^2, joint 2 in dependence of CoM s2 of link 2

        with anim.at_frame(vis, i) as frame:
            frame["testobj1"].set_transform(tf.translation_matrix([0,0,0]))
            frame["testobj2"].set_transform(H_0_s1 @ H_s1_1)
            frame["testobj3"].set_transform(H_0_s2 @ H_s2_2)
            frame["testobj1"].get_clip().fps=1/dt
            frame["testobj2"].get_clip().fps=1/dt
            frame["testobj3"].get_clip().fps=1/dt

    # Set the animation to the Meshcat viewer
    vis.set_animation(anim)
    vis["testobj1"].set_property('visible', True)
    vis["testobj2"].set_property('visible', True)
    vis["testobj3"].set_property('visible', True)

    robot_display.viewer.open()

    # i = 0
    # cnt=rep_cnt
    # while i < cnt:
    #     time.sleep(rep_delay_sec)
    #     # robot_display.displayFromSolver(ddp)
    #     robot_display.robot.viz.play(x_sol[:, 6:6+robot_model.nq], dt)
    #     i = i +1
    # ffmpeg -r 60 -i "%07d.jpg" -vcodec "libx264" -preset "slow" -crf 18 -vf pad="width=ceil(iw/2)*2:height=ceil(ih/2)*2" output.mp4
    # ffmpeg -r 60 -i "%07d.png" -vcodec "libx264" -preset "slow" -crf 18 -vf pad="width=ceil(iw/2)*2:height=ceil(ih/2)*2" output.mp4

    time.sleep(1) # othervise visualization don't work
    
    create_video=False
    if create_video:
        robot_display = crocoddyl.MeshcatDisplay(robot, -1, 1, False, visibility=False)
        with robot_display.robot.viz.create_video_ctx("test.mp4"):
            robot_display.robot.viz.play(x_sol[:, 6:6+robot_model.nq], dt)

def ocp_problem_v3(state, q0, TCP_frame_id, param_trajectory, dt):
    y_d_data    = param_trajectory['p_d'].T
    y_d_p_data  = param_trajectory['p_d_p'].T
    y_d_pp_data = param_trajectory['p_d_pp'].T

    # Reihenfolge beachten:
    # Zuerst Model1: yref Model (yref = [x1,x2,x3], d/dt yref = [x4,x5,x6]), 
    # dann Model2: Robot model (q = [q1, q2] = [x7, x8], d/dt q = d/dt [q1, q2] = [x9, x10])
    x0 = np.concatenate([y_d_data[0], y_d_p_data[0], q0, pinocchio.utils.zero(state.nv)])

    N = len(y_d_data)

    # weights
    q_tracking_cost = 1e5
    q_terminate_tracking_cost = 1e10
    # q_xreg_cost = 1e1 # was tut das eigentlich?? addcost einkommentieren nichtvergessen
    q_ureg_cost = 0*1e-10

    running_cost_models = list()
    terminate_cost_models = list()

    actuationModel = crocoddyl.ActuationModelFull(state)

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

        yy_NDIAM = crocoddyl.IntegratedActionModelEuler(yy_DAM, dt)
        
        runningCostModel = crocoddyl.CostModelSum(state)

        goalTrackingCost = crocoddyl.CostModelResidual(
            state,
            crocoddyl.ResidualModelFrameTranslation(
                state, TCP_frame_id, np.zeros((3,1)) # np.zeros((3,1) wird in Klasse CombinedActionModel mit yref überschrieben.
            ),
        )
        xRegCost = crocoddyl.CostModelResidual(state, crocoddyl.ResidualModelState(state))
        uRegCost = crocoddyl.CostModelResidual(state, crocoddyl.ResidualModelControl(state))

        if i < N-1:
            runningCostModel.addCost("TCP_pose", goalTrackingCost, q_tracking_cost)
            # runningCostModel.addCost("stateReg", xRegCost, q_xreg_cost)
            runningCostModel.addCost("ctrlReg", uRegCost, q_ureg_cost)

            running_cost_models.append(CombinedActionModel(yy_NDIAM, crocoddyl.IntegratedActionModelEuler(
                crocoddyl.DifferentialActionModelFreeFwdDynamics(
                    state, actuationModel, runningCostModel
                ),
                dt,
            )))
        else: # i == N: # Endkostenterm
            terminalCostModel = crocoddyl.CostModelSum(state)
            terminalCostModel.addCost("TCP_pose", goalTrackingCost, q_terminate_tracking_cost)
            # terminalCostModel.addCost("stateReg", xRegCost, q_xreg_cost)
            terminalCostModel.addCost("ctrlReg", uRegCost, q_ureg_cost)

            terminate_cost_models.append(CombinedActionModel(yy_NDIAM, crocoddyl.IntegratedActionModelEuler(
                crocoddyl.DifferentialActionModelFreeFwdDynamics(
                    state, actuationModel, terminalCostModel
                )
            )))

    # Create the shooting problem
    seq = running_cost_models
    problem = crocoddyl.ShootingProblem(x0, seq, terminate_cost_models[-1])
    return problem