import numpy as np
from scipy.spatial.transform import Rotation
import scipy as sp
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

def generate_trajectory(t, xe0, xeT, R_init, rot_ax, rot_alpha_scale, T_start, T_end, param_traj_poly, plot_traj=False):
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

    if plot_traj:
        plot_trajectory(param_trajectory)

    return param_trajectory

def tic():
    global start_time
    start_time = time.time()

def toc():
    if 'start_time' in globals():
        elapsed_time = time.time() - start_time
        formatted_time = format_time(elapsed_time)
        print(f"\033[92mElapsed time: {formatted_time}\033[0m")
    else:
        print("Call tic() before calling toc()")

class TicToc:
    def __init__(self):
        self.start_time = None
        self.elapsed_total_time = 0

    def tic(self, reset=False):
        if reset:
            self.start_time = None
            self.elapsed_total_time = 0
        else:
            self.start_time = time.time()

    def toc(self):
        if self.start_time is None:
            print("Error: Tic not started.")
            return

        current_time = time.time()
        elapsed_time = current_time - self.start_time
        self.elapsed_total_time += elapsed_time
        self.start_time = current_time

    def get_time(self):
        self.toc()
        return self.elapsed_total_time
    
    def get_time_str(self, additional_text="time"):
        self.toc()
        formatted_time = format_time(self.elapsed_total_time)
        return f"\033[92mElapsed {additional_text}: {formatted_time}\033[0m"
    
    def print_time(self, additional_text="time"):
        time_str = self.get_time_str(additional_text)
        print(time_str, end='\n')

    def reset(self):
        self.start_time = None
        self.elapsed_total_time = 0

def format_time(elapsed_time):
    """Formats the elapsed time based on its magnitude."""
    if elapsed_time < 1e-3:
        return f"{elapsed_time * 1e6:.2f} μs"  # Microseconds
    elif elapsed_time < 1:
        return f"{elapsed_time * 1e3:.2f} ms"  # Milliseconds
    elif elapsed_time < 60:
        return f"{elapsed_time:.2f} s"  # Seconds
    else:
        minutes = int(elapsed_time // 60)
        seconds = elapsed_time % 60
        return f"{minutes}m {seconds:.2f}s"  # Minutes and seconds

def block_diag_onlydiagmatr(a, b):
    return np.kron(a, b)

def block_diag(a, b):
    an, am = a.shape
    bn, bm = b.shape
    return np.block([[a, np.zeros((an, bm))], [np.zeros((bn, am)), b]])

################################## MODEL TESTS ############################################


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


###################### MCP & OCP Models ########################################################

class DifferentialActionModelRunningYref(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self, y_d, y_d_p, y_d_pp, Kd, Kp):
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

        self.Kp = Kp
        self.Kd = Kd

        self.Kp_square = Kp**2
        self.Kd_square = Kd**2

        self.KdKp = Kd @ Kp

        self.I = np.eye(self.m)

    def calc(self, data, x, u):
        n1 = self.n1
        n = self.n

        z1 = x[:n1]
        z2 = x[n1:n]
        alpha  = u

        data.xout = alpha # = z1ddot

        r = (alpha - self.y_d_pp) + self.Kd @ (z2 - self.y_d_p) + self.Kp @ (z1 - self.y_d)
        data.r=r
        # data.cost = 1/2 * np.linalg.norm(r)**2
        # data.cost = 1/2 * r @ r
        # data.cost = 1/2 * np.sum(r**2)
        data.cost = 1/2*(r[0]**2 + r[1]**2 + r[2]**2) # fastest

    def calcDiff(self, data, x, u):
        n1 = self.n1 # TODO: Benennung verwirrend zu unteren Klassen
        n = self.n

        Kp = self.Kp
        Kd = self.Kd
        Kp_square = self.Kp_square
        Kd_square = self.Kd_square
        KdKp = self.KdKp

        I = self.I
        r = data.r

        # data.Fx = np.block([O, O]) # f = d^2/dt^2 z2 = u # ist eh standardmäßig 0
        data.Fu = I

        #data.Lx  = np.block([Kp @ r, Kd @ r])
        # data.Lx[:n1]  = Kp @ r
        # data.Lx[n1:n] = Kd @ r

        data.Lx[0]  = Kp[0,0]*r[0]
        data.Lx[1]  = Kp[1,1]*r[1]
        data.Lx[2]  = Kp[2,2]*r[2]
        
        data.Lx[3]  = Kd[0,0]*r[0]
        data.Lx[4]  = Kd[1,1]*r[1]
        data.Lx[5]  = Kd[2,2]*r[2]
        
        #data.Lxx = np.block([[Kp_square, KdKp], [KdKp, Kd_square]])
        data.Lxx[:n1, :n1]    = Kp_square
        data.Lxx[:n1, n1:n]   = KdKp
        data.Lxx[n1:n:, n1:n] = Kd_square
        data.Lxx[n1:n, :n1]   = KdKp
        
        data.Lu = r
        data.Luu = I

        data.Lxu[:n1, :n1] = Kp
        data.Lxu[n1:n, :n1] = Kd


# kann performance verbessern!
class CombinedDifferentialActionModel(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self, model1, model2):
        crocoddyl.DifferentialActionModelAbstract.__init__(self, crocoddyl.StateVector(model1.state.nx + model2.state.nx), model1.nu + model2.nu, model1.nr + model2.nr)
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
        self.model2.costs.costs["TCP_pose"].cost.residual.reference = yref
        # self.model2.differential.costs.costs["TCP_pose"].cost.residual.reference = self.model1.differential.y_d

        self.model1.calc(self.data1, x[:n1],  u[:m1] )
        self.model2.calc(self.data2, x[n1:n], u[m1:m])

        data.xout = np.hstack([self.data1.xout, self.data2.xout])
        data.cost  = self.data1.cost + self.data2.cost

    def calcDiff(self, data, x, u):
        m1 = self.mu_model1
        m = self.m

        n1 = self.nx_model1
        n = self.n

        yref = x[:m1]
        self.model2.costs.costs["TCP_pose"].cost.residual.reference = yref
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


class CombinedActionModel(crocoddyl.ActionModelAbstract):
    def __init__(self, model1, model2):
        crocoddyl.ActionModelAbstract.__init__(self, crocoddyl.StateVector(model1.state.nx + model2.state.nx), model1.nu + model2.nu, model1.nr + model2.nr)
        self.model1 = model1
        self.model2 = model2 # model 2 should be robot model!!

        self.TCP_frame_id = self.model2.state.pinocchio.getFrameId('TCP')
        self.q_tracking_cost = self.model2.differential.costs.costs["TCP_pose"].weight
        self.Q_tracking_cost = np.diag([self.q_tracking_cost]*3)

        self.data1 = self.model1.createData()
        self.data2 = self.model2.createData()

        self.mu_model1 = self.model1.nu
        self.mu_model2 = self.model2.nu
        self.m = self.mu_model1 + self.mu_model2

        self.nq_model1 = self.model1.state.nq
        self.nx_model1 = self.model1.state.nx
        self.nx_model2 = self.model2.state.nx
        self.n = self.nx_model1 + self.nx_model2

        self.robot_model = self.model2.state.pinocchio
        self.robot_data = self.robot_model.createData()

        self.dt_int = self.model1.dt

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

        nq1 = self.nq_model1
        n1 = self.nx_model1
        n = self.n

        dt_int = self.dt_int

        yref = x[:m1]
        y = self.data2.differential.pinocchio.oMf[self.TCP_frame_id].translation # GEHT NUR MIT EULER????
        
        # robot_data = self.robot_data
        # robot_model = self.robot_model
        # pinocchio.forwardKinematics(robot_model, robot_data, x[n1:n1+2])
        # pinocchio.updateFramePlacements(robot_model, robot_data)
        # y = robot_data.oMf[self.TCP_frame_id].translation.copy()

        self.model2.differential.costs.costs["TCP_pose"].cost.residual.reference = yref
        
        self.model1.calcDiff(self.data1, x[0:n1],  u[0:m1])
        self.model2.calcDiff(self.data2, x[n1:n],  u[m1:m])

        data.Fx[:n1, :n1]    = self.data1.Fx
        data.Fx[n1:n, n1:n]  = self.data2.Fx

        data.Fu[:n1, :m1]    = self.data1.Fu
        data.Fu[n1:n, m1:m]  = self.data2.Fu

        # crocoddyl.CostModelResidual verwendet activation a(.) = 1/2*||r(.)||^2 mit r = y-yref
        # womit cost = a(y-yref) = 1/2*||y-yref||^2 entsteht. Da y(x) mit der Optimierungsvariable x und
        # der Optimierungsvariable yref muss dieses Residual in der Ableitung nach x und z1 berücksichtigt
        # werden. Allerdings ist die Ableitung nach z1 in model1 nicht machbar, da model 1 nicht
        # y kennt. Daher berechne ich die Ableitungen hier manuell.
        data.Lx[:nq1]        = self.data1.Lx[:nq1] - dt_int*self.Q_tracking_cost @ (y - yref)
        data.Lx[nq1:n1]      = self.data1.Lx[nq1:n1]
        data.Lx[n1:n]        = self.data2.Lx # + self.Q_tracking_cost @ (y - yref) ist wegen goal residuum schon dabei

        data.Lxx[:nq1, :nq1]     = self.data1.Lxx[:nq1, :nq1] + dt_int*self.Q_tracking_cost # Kp_square + Q
        data.Lxx[:nq1, nq1:n1]   = self.data1.Lxx[:nq1, nq1:n1] # KdKp
        data.Lxx[nq1:n1, nq1:n1] = self.data1.Lxx[nq1:n1, nq1:n1] # Kd_square
        data.Lxx[nq1:n1, :nq1]   = self.data1.Lxx[nq1:n1, :nq1] # KdKp

        data.Lxx[n1:n, n1:n] = self.data2.Lxx
        
        data.Lu[:m1]         = self.data1.Lu
        data.Lu[m1:m]        = self.data2.Lu

        data.Luu[:m1, :m1]   = self.data1.Luu
        data.Luu[m1:m, m1:m] = self.data2.Luu

        data.Lxu[:n1, :m1]   = self.data1.Lxu
        data.Lxu[n1:n, m1:m] = self.data2.Lxu

class CombinedActionModelTerminal(CombinedActionModel):
    def __init__(self, model1, model2):
        # CombinedActionModel.__init__(self, model1, model2)
        self.model = CombinedActionModel(model1, model2)
        crocoddyl.ActionModelAbstract.__init__(self, crocoddyl.StateVector(self.model.state.nx), self.model.nu, self.model.nr)
        
        self.data1  = self.model.data1
        self.data2  = self.model.data2

        self.TCP_frame_id = self.model.TCP_frame_id
        self.Q_tracking_cost = self.model.Q_tracking_cost

        self.m1 = self.model.mu_model1
        self.m = self.model.m

        self.nq1 = self.model.nq_model1
        self.n1 = self.model.nx_model1
        self.n = self.model.n

        self.robot_model = self.model.robot_model
        self.robot_data = self.model.robot_data

        self.dt_int = self.model.dt_int

    def calc(self, data, x, u):
        m1 = self.m1
        m = self.m

        n1 = self.n1
        n = self.n

        yref = x[:m1]
        self.model.model2.differential.costs.costs["TCP_pose"].cost.residual.reference = yref
        self.model.model2.calc(self.data2, x[n1:n], u[m1:m])

        data.cost  = self.data2.cost

    def calcDiff(self, data, x, u):
        m1 = self.m1
        m = self.m

        nq1 = self.nq1
        n1 = self.n1
        n = self.n

        dt_int = self.dt_int

        yref = x[:m1] # das wird in model1.calc berechnet
        
        
        y = self.data2.differential.pinocchio.oMf[self.TCP_frame_id].translation # GEHT NUR MIT EULER????
        
        # robot_data = self.robot_data
        # robot_model = self.robot_model
        # pinocchio.forwardKinematics(robot_model, robot_data, x[n1:n1+2])
        # pinocchio.updateFramePlacements(robot_model, robot_data)
        # y = robot_data.oMf[self.TCP_frame_id].translation.copy()
        
        self.model.model2.differential.costs.costs["TCP_pose"].cost.residual.reference = yref
        # self.model.model1.calcDiff(self.data1, x[0:n1],  u[0:m1])
        self.model.model2.calcDiff(self.data2, x[n1:n], u[m1:m])

        # data.Fx[:n1, :n1]    = self.data1.Fx
        data.Fx[n1:n, n1:n]  = self.data2.Fx

        # data.Fu[:n1, :m1]    = self.data1.Fu
        data.Fu[n1:n, m1:m]  = self.data2.Fu

        # data.Lx[:n1]       = self.data1.Lx
        data.Lx[:nq1]         = -dt_int*self.Q_tracking_cost @ (y - yref)

        data.Lx[n1:n]        = self.data2.Lx
        
        # data.Lu[:m1]       = self.data1.Lu
        data.Lu[m1:m]        = self.data2.Lu

        # data.Lxx[:n1, :n1]   = self.data1.Lxx
        data.Lxx[:nq1, :nq1] = dt_int*self.Q_tracking_cost
        data.Lxx[n1:n, n1:n] = self.data2.Lxx

        # data.Luu[:m1, :m1]   = self.data1.Luu
        data.Luu[m1:m, m1:m] = self.data2.Luu

        # data.Lxu[:n1, :m1]   = self.data1.Lxu
        data.Lxu[n1:n, m1:m] = self.data2.Lxu

def IntegratedActionModelRK2(DAM, dt):
    return crocoddyl.IntegratedActionModelRK(DAM, crocoddyl.RKType(2), dt)

def IntegratedActionModelRK3(DAM, dt):
    return crocoddyl.IntegratedActionModelRK(DAM, crocoddyl.RKType(3), dt)

def IntegratedActionModelRK4(DAM, dt):
    return crocoddyl.IntegratedActionModelRK(DAM, crocoddyl.RKType(4), dt)

def get_int_type(int_type):
    if int_type == 'euler':
        return crocoddyl.IntegratedActionModelEuler
    elif int_type == 'RK2':
        return IntegratedActionModelRK2
    elif int_type == 'RK3':
        return IntegratedActionModelRK3
    elif int_type == 'RK4':
        return IntegratedActionModelRK4
    else:
        raise ValueError("int_type must be 'euler', 'RK2', 'RK3' or 'RK4'")

def ocp_problem_v1(start_index, end_index, N_step, state, x0, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type='euler'):
    IntegratedActionModel = get_int_type(int_type)

    y_d_data    = param_trajectory['p_d'].T
    y_d_p_data  = param_trajectory['p_d_p'].T
    y_d_pp_data = param_trajectory['p_d_pp'].T

    N = start_index + (end_index-start_index)*N_step

    # weights
    q_tracking_cost = param_mpc_weight['q_tracking_cost']
    q_terminate_tracking_cost = param_mpc_weight['q_terminate_tracking_cost']
    q_xreg_cost = param_mpc_weight['q_xreg_cost']
    q_ureg_cost = param_mpc_weight['q_ureg_cost']

    running_cost_models = list()
    terminate_cost_models = list()

    actuationModel = crocoddyl.ActuationModelFull(state)

    for i in range(start_index, N, N_step):
        y_d    = y_d_data[i]
        
        runningCostModel = crocoddyl.CostModelSum(state)

        if q_xreg_cost not in [0, None]:
            xRegCost = crocoddyl.CostModelResidual(state, residual=crocoddyl.ResidualModelState(state))
        if q_ureg_cost not in [0, None]:
            uRegCost = crocoddyl.CostModelResidual(state, residual=crocoddyl.ResidualModelControl(state))

        goalTrackingCost = crocoddyl.CostModelResidual(
            state,
            residual=crocoddyl.ResidualModelFrameTranslation(
                state, TCP_frame_id, y_d
            ),
        )

        if i < N-N_step:
            runningCostModel.addCost("TCP_pose", goalTrackingCost, q_tracking_cost)
            if q_xreg_cost not in [0, None]:
                runningCostModel.addCost("stateReg", xRegCost, q_xreg_cost)
            if q_ureg_cost not in [0, None]:
                runningCostModel.addCost("ctrlReg", uRegCost, q_ureg_cost)

            running_cost_models.append(IntegratedActionModel(
                crocoddyl.DifferentialActionModelFreeFwdDynamics(
                    state, actuationModel, runningCostModel
                ),
                dt*N_step
            ))
        else: # i == N: # Endkostenterm
            terminalCostModel = crocoddyl.CostModelSum(state)
            terminalCostModel.addCost("TCP_pose", goalTrackingCost, q_terminate_tracking_cost)
            if q_xreg_cost not in [0, None]:
                terminalCostModel.addCost("stateReg", xRegCost, q_xreg_cost)
            if q_ureg_cost not in [0, None]:
                terminalCostModel.addCost("ctrlReg", uRegCost, q_ureg_cost)

            terminate_cost_models.append(IntegratedActionModel(
                crocoddyl.DifferentialActionModelFreeFwdDynamics(
                    state, actuationModel, terminalCostModel
                ),
                dt*N_step
            ))

    # Create the shooting problem
    seq = running_cost_models
    problem = crocoddyl.ShootingProblem(x0, seq, terminate_cost_models[-1])
    return problem

def ocp_problem_v3(start_index, end_index, N_step, state, x0, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type='euler', use_bounds=False):

    ###################
    # Reihenfolge beachten:
    # Zuerst Model1: yref Model (yref = [x1,x2,x3], d/dt yref = [x4,x5,x6]), 
    # dann Model2: Robot model (q = [q1, q2] = [x7, x8], d/dt q = d/dt [q1, q2] = [x9, x10])

    IntegratedActionModel = get_int_type(int_type)

    y_d_data    = param_trajectory['p_d'].T
    y_d_p_data  = param_trajectory['p_d_p'].T
    y_d_pp_data = param_trajectory['p_d_pp'].T

    N = start_index + (end_index-start_index)*N_step

    # weights
    q_tracking_cost = param_mpc_weight['q_tracking_cost']
    q_terminate_tracking_cost = param_mpc_weight['q_terminate_tracking_cost']
    q_xreg_cost = param_mpc_weight['q_xreg_cost']
    q_ureg_cost = param_mpc_weight['q_ureg_cost']
    Kd = param_mpc_weight['Kd']
    Kp = param_mpc_weight['Kp']
    lb_y_ref_N = param_mpc_weight['lb_y_ref_N']
    ub_y_ref_N = param_mpc_weight['ub_y_ref_N']

    running_cost_models = list()
    terminate_cost_models = list()

    actuationModel = crocoddyl.ActuationModelFull(state)

    for i in range(start_index, N, N_step):
        y_d    = y_d_data[i]
        y_d_p  = y_d_p_data[i]
        y_d_pp = y_d_pp_data[i]

        yy_DAM = DifferentialActionModelRunningYref(y_d, y_d_p, y_d_pp, Kd, Kp)
        
        # data = yy_DAM.createData()
        # tic()
        # for j in range(0,100000):
        #     yy_DAM.calcDiff(data, x0, np.zeros(3))
        # toc()
        # quit()
        
        runningCostModel = crocoddyl.CostModelSum(state)

        if q_xreg_cost not in [0, None]:
            xRegCost = crocoddyl.CostModelResidual(state, residual=crocoddyl.ResidualModelState(state))
        if q_ureg_cost not in [0, None]:
            uRegCost = crocoddyl.CostModelResidual(state, residual=crocoddyl.ResidualModelControl(state))

        if i < N-N_step:
            yy_NDIAM = IntegratedActionModel(yy_DAM, dt*N_step)

            goalTrackingCost = crocoddyl.CostModelResidual(
                    state,
                    residual=crocoddyl.ResidualModelFrameTranslation(
                        state, TCP_frame_id, y_d # y_d wird in Klasse CombinedActionModel mit yref überschrieben.
                    ),
                )
            runningCostModel.addCost("TCP_pose", goalTrackingCost, q_tracking_cost)
            if q_xreg_cost not in [0, None]:
                runningCostModel.addCost("stateReg", xRegCost, q_xreg_cost)
            if q_ureg_cost not in [0, None]:
                runningCostModel.addCost("ctrlReg", uRegCost, q_ureg_cost)

            # running_cost_models.append(
            #     IntegratedActionModel(
            #         CombinedDifferentialActionModel(
            #             yy_DAM, 
            #             crocoddyl.DifferentialActionModelFreeFwdDynamics(
            #                 state, actuationModel, runningCostModel
            #             )
            #         ),
            #         dt*N_step
            #     )
            # )

            running_cost_models.append(CombinedActionModel(yy_NDIAM, IntegratedActionModel(
                crocoddyl.DifferentialActionModelFreeFwdDynamics(
                    state, actuationModel, runningCostModel
                ),
                dt*N_step
            )))
        else: # i == N: # Endkostenterm
            yy_NDIAM = IntegratedActionModel(yy_DAM, 0) # disable integration by dt=0
            if use_bounds:
                bounds = crocoddyl.ActivationBounds(lb_y_ref_N, ub_y_ref_N)
                activationModel = crocoddyl.ActivationModelQuadraticBarrier(bounds)
                terminalGoalTrackingCost = crocoddyl.CostModelResidual(
                    state,
                    activation=activationModel,
                    residual=crocoddyl.ResidualModelFrameTranslation(
                        state, TCP_frame_id, y_d # y_d wird in Klasse CombinedActionModel mit yref überschrieben.
                    ),
                )
            else:
                terminalGoalTrackingCost = crocoddyl.CostModelResidual(
                    state,
                    residual=crocoddyl.ResidualModelFrameTranslation(
                        state, TCP_frame_id, y_d # y_d wird in Klasse CombinedActionModel mit yref überschrieben.
                    ),
                )
            terminalCostModel = crocoddyl.CostModelSum(state)
            terminalCostModel.addCost("TCP_pose", terminalGoalTrackingCost, q_terminate_tracking_cost)
            if q_xreg_cost not in [0, None]:
                terminalCostModel.addCost("stateReg", xRegCost, q_xreg_cost)
            if q_ureg_cost not in [0, None]:
                terminalCostModel.addCost("ctrlReg", uRegCost, q_ureg_cost)

            if use_bounds:
                terminate_cost_models.append(CombinedActionModelTerminal(yy_NDIAM, IntegratedActionModel(
                    crocoddyl.DifferentialActionModelFreeFwdDynamics(
                        state, actuationModel, terminalCostModel
                    ),
                    0*dt*N_step # disable integration: https://github.com/loco-3d/crocoddyl/issues/962 (besser in calc ohne u beruecks.)
                )))
            else:
                # terminate_cost_models.append(
                #     IntegratedActionModel(
                #         CombinedDifferentialActionModel(
                #             yy_DAM, 
                #             crocoddyl.DifferentialActionModelFreeFwdDynamics(
                #                 state, actuationModel, terminalCostModel
                #             )
                #         ),
                #         dt*N_step
                #     )
                # )

                terminate_cost_models.append(CombinedActionModelTerminal(yy_NDIAM, IntegratedActionModel(
                    crocoddyl.DifferentialActionModelFreeFwdDynamics(
                        state, actuationModel, terminalCostModel
                    ),
                    0*dt*N_step # (unsicher obt dt=0 zulaessig: besser in calc ohne u beruecks.)
                )))

    # Create the shooting problem
    seq = running_cost_models
    problem = crocoddyl.ShootingProblem(x0, seq, terminate_cost_models[-1])
    return problem


#############

def get_mpc_funs(problem_name):
    if problem_name == 'MPC_v1':
        return crocoddyl.SolverDDP, first_init_guess_mpc_v1, create_ocp_problem_v1, simulate_model_mpc_v1
        # return crocoddyl.SolverBoxFDDP, first_init_guess_mpc_v1, create_ocp_problem_v1, simulate_model_mpc_v1
        # return crocoddyl.SolverFDDP, first_init_guess_mpc_v1, create_ocp_problem_v1, simulate_model_mpc_v1
    elif problem_name == 'MPC_v3_soft_yN_ref':
        return  crocoddyl.SolverDDP, first_init_guess_mpc_v3, create_ocp_problem_v3_soft_yN_ref, simulate_model_mpc_v3
    elif problem_name == 'MPC_v3_bounds_yN_ref':
        return crocoddyl.SolverBoxDDP, first_init_guess_mpc_v3, create_ocp_problem_v3_bounds_yN_ref, simulate_model_mpc_v3
        # return crocoddyl.SolverFDDP, first_init_guess_mpc_v3, create_ocp_problem_v3_bounds_yN_ref, simulate_model_mpc_v3
    else:
        raise ValueError("problem_name must be 'MPC_v1' | 'MPC_v3_soft_yN_ref' | 'MPC_v3_bounds_yN_ref'")

def create_ocp_problem_v1(start_index, end_index, N_step, state, xk, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type = 'euler'):
    return ocp_problem_v1(start_index, end_index, N_step, state, xk, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type)

def create_ocp_problem_v3_bounds_yN_ref(start_index, end_index, N_step, state, xk, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type = 'euler'):
    return ocp_problem_v3(start_index, end_index, N_step, state, xk, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type, use_bounds=True)

def create_ocp_problem_v3_soft_yN_ref(start_index, end_index, N_step, state, xk, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type = 'euler'):
    return ocp_problem_v3(start_index, end_index, N_step, state, xk, TCP_frame_id, param_trajectory, param_mpc_weight, dt, int_type, use_bounds=False)

##############

def simulate_model_mpc_v1(ddp, i, dt, nq, nx, robot_model, robot_data, param_trajectory):

    xk = ddp.xs[0] # muss so sein, da x0 in ddp.xs[0] gespeichert ist
    uk = ddp.us[0]

    # Modell Simulation
    tau_k = uk
    q     = xk[:nq]
    q_p   = xk[nq:nx]

    q_next, q_p_next = sim_model(robot_model, robot_data, q, q_p, tau_k, dt)

    xk[:nq]   = q_next
    xk[nq:nx] = q_p_next

    xs_init_guess = ddp.xs
    us_init_guess = ddp.us

    us_i = uk
    xs_i = xk

    return xk, xs_i, us_i, xs_init_guess, us_init_guess

def simulate_model_mpc_v3(ddp, i, dt, nq, nx, robot_model, robot_data, param_trajectory):

    xk = ddp.xs[0] # muss so sein, da x0 in ddp.xs[0] gespeichert ist
    uk = ddp.us[0]

    # Modell Simulation
    tau_k = uk[3:]
    q = xk[6:6+nq]
    q_p = xk[6+nq:6+nx]

    q_next, q_p_next = sim_model(robot_model, robot_data, q, q_p, tau_k, dt)

    xk[0:6]      = np.hstack([param_trajectory['p_d'][:, i+1], param_trajectory['p_d_p'][:, i+1]])
    xk[6:6+nq]   = q_next
    xk[6+nq:6+nx] = q_p_next

    xs_init_guess = ddp.xs
    us_init_guess = ddp.us

    us_i = uk
    xs_i = xk

    # us_i = ddp.us[-1]
    # xs_i = ddp.xs[-1]

    return xk, xs_i, us_i, xs_init_guess, us_init_guess

def sim_model(robot_model, robot_data, q, q_p, tau, dt):
    q_pp     = pinocchio.aba(robot_model, robot_data, q, q_p, tau)
    q_p_next = q_p + dt * q_pp # Euler method
    q_next   = pinocchio.integrate(robot_model, q, dt * q_p_next)
    return q_next, q_p_next

###############

def first_init_guess_mpc_v1(tau_init_robot, x_init_robot, N_traj, N_horizon, nx, nu, param_trajectory):
    xk = x_init_robot

    xs_init_guess = [x_init_robot]  * (N_horizon)
    us_init_guess = [tau_init_robot] * (N_horizon-1)

    xs = np.zeros((N_traj, nx))
    us = np.zeros((N_traj, nu))
    return xk, xs, us, xs_init_guess, us_init_guess

def first_init_guess_mpc_v3(tau_init_robot, x_init_robot, N_traj, N_horizon, nx, nu, param_trajectory):
    x0_init = np.hstack([param_trajectory['p_d'][:, 0], param_trajectory['p_d_p'][:, 0], x_init_robot])
    xk = x0_init

    xs_init_guess = [x0_init] * (N_horizon)
    us_init_guess = [np.hstack([param_trajectory['p_d_pp'][:, 0], tau_init_robot])] * (N_horizon-1)

    xs = np.zeros((N_traj, 6+nx))
    us = np.zeros((N_traj, 3+nu))
    return xk, xs, us, xs_init_guess, us_init_guess

###################################################################################################################


def check_solver_status(warn_cnt, hasConverged, ddp, us, xs, i, t, dt, N_horizon, N_step, TCP_frame_id, robot_model, param_trajectory, conv_max_limit=5, plot_sol = False):
    error = 0
    
    if not hasConverged:
        print("\033[43mWarning: Solver did not converge at time t =", f"{i*dt:.3f}", "\033[0m")
        warn_cnt += 1
        # plot_mpc_solution(ddp, i, dt, N_horizon, N_step, TCP_frame_id, robot_model, param_trajectory)
        if warn_cnt > conv_max_limit:
            print("\033[91mError: Solver failed to converge", f"{conv_max_limit}", "times in a row. Exiting...\033[0m")
            error = 1
    else:
        warn_cnt = 0

    if ddp.isFeasible == False:
        # print("\033[93mWarning: Solver is not feasible at time t = ", f"{i*dt:.3f}", "\033[0m")
        print("\033[91mError: Solver is not feasible at time t =", f"{i*dt:.3f}", "\033[0m")
        error = 1
    if np.isnan(ddp.xs).any() or np.isnan(ddp.us).any():
        # print("\033[93mWarning: NaN values detected in xs or us arrays at time t = ", f"{i*dt:.3f}", "\033[0m")
        print("\033[91mError: NaN values detected in xs or us arrays at time t =", f"{i*dt:.3f}", "\033[0m")
        error = 1
    if error:
        if plot_sol:
            plot_mpc_solution(ddp, i, dt, N_horizon, N_step, TCP_frame_id, robot_model, param_trajectory)
            plot_current_solution(us, xs, i, t, TCP_frame_id, robot_model, param_trajectory)
        exit()
    return warn_cnt

#######################################

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


def plot_solution(us, xs, t, TCP_frame_id, robot_model, param_trajectory, save_plot=False, file_name='plot_saved', plot_fig=True):
    robot_data = robot_model.createData()

    line_dict_y        = dict(width=1, color='#ffff11')
    line_dict_b        = dict(width=1, color='#14a1ff')
    line_dict_traj     = [dict(width=1, color='#ffff11'), dict(width=1, color='#14a1ff')]
    line_dict_traj_dot = [dict(width=1, color='green', dash='dash'), dict(width=1, color='#ff0000', dash='dash')]
    line_dict_yref_dot = [dict(width=1, color='gray', dash='dash'),  dict(width=1, color='darkred', dash='dash')]
    # #ffff11 = yellow, #ff0000 = red, #14a1ff = lightblue, #0000ff = darkblue

    N = len(xs)
    n = robot_model.nq

    y_opt    = np.zeros((N, 3))
    y_opt_p  = np.zeros((N, 3))
    y_opt_pp = np.zeros((N, 3))
    q        = np.zeros((N, n))
    q_p      = np.zeros((N, n))
    q_pp     = np.zeros((N, n))
    w        = np.zeros(N) # manipulability

    y_d    = param_trajectory['p_d'].T
    y_d_p  = param_trajectory['p_d_p'].T
    y_d_pp = param_trajectory['p_d_pp'].T

    if xs.shape[1] == 2*n+6:
        y_ref    = xs[:, 0:3]
        y_ref_p  = xs[:, 3:6]
        y_ref_pp = us[:, 0:3]

        tau = np.concatenate([us[:, 3:3+n], [us[-1, 3:3+n]]]) # es wird ja ein u weniger erzeugt, da es nicht notw. ist
        q   = xs[:, 6:6+n]
        q_p = xs[:, 6+n:6+2*n]
        plot_yref = True
    else:
        tau = np.concatenate([us, [us[-1]]]) # es wird ja ein u weniger erzeugt, da es nicht notw. ist
        q   = xs[:, 0:n  ]
        q_p = xs[:, n:2*n]
        plot_yref = False

    for i in range(N):
        q_pp[i] = pinocchio.aba(robot_model, robot_data, q[i], q_p[i], tau[i])

        pinocchio.forwardKinematics(robot_model, robot_data, q[i], q_p[i], q_pp[i])
        pinocchio.updateFramePlacements(robot_model, robot_data)
        y_opt[i] = robot_data.oMf[TCP_frame_id].translation.T.copy()

        pinocchio.computeJointJacobians(robot_model, robot_data, q[i])
        # J = pinocchio.computeFrameJacobian(robot_model, robot_data, q[i], TCP_frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J = pinocchio.getFrameJacobian(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        pinocchio.computeJointJacobiansTimeVariation(robot_model, robot_data, q[i], q_p[i])
        J_p = pinocchio.getFrameJacobianTimeVariation(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED) # ist falsch!!

        J_v   = J[  0:3, 0:2] # mir translatorischen anteil
        J_v_p = J_p[0:3, 0:2] # z ist unnötig

        y_opt_p[i]  = J_v @ q_p[i] # pinocchio.getVelocity(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear ist falsch
        # y_opt_pp[i] = J_v @ q_pp[i] + J_v_p @ q_p[i] # J_p und damit J_V_p stimmt einfach nicht, vgl maple????? vgl Issue auf Github Doku
        y_opt_pp[i] = pinocchio.getFrameClassicalAcceleration(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear # mit maple bestätigt.

        w[i] = np.sqrt(sp.linalg.det(J_v[0:2, 0:2] @ J_v[0:2, 0:2].T))

    e    = y_d    - y_opt
    e_p  = y_d_p  - y_opt_p
    e_pp = y_d_pp - y_opt_pp

    y_opt_labels      = [ "$\\Large{x^\\mathrm{opt}}$",        "$\\Large{y^\\mathrm{opt}}$"        ]
    y_opt_labels_t    = [ "x_opt",                             "y_opt",                            ]
    y_opt_p_labels    = [ "$\\Large{\\dot{x}^\\mathrm{opt}}$", "$\\Large{\\dot{y}^\\mathrm{opt}}$" ]
    y_opt_p_labels_t  = [ "d/dt x_opt",                        "d/dt y_opt",                       ]
    y_opt_pp_labels   = [ "$\\Large{\\ddot{x}^\\mathrm{opt}}$", "$\\Large{\\ddot{y}^\\mathrm{opt}}$" ]
    y_opt_pp_labels_t = [ "d^2/dt^2 x_opt",                    "d^2/dt^2 y_opt",                   ]

    y_ref_labels      = [ "$\\Large{x^{\\mathrm{ref}}}$",        "$\\Large{y^{\\mathrm{ref}}}$"          ]
    y_ref_labels_t    = [ "x_ref",                               "y_ref"                                 ]
    y_ref_p_labels    = [ "$\\Large{\\dot{x}^{\\mathrm{ref}}}$", "$\\Large{\\dot{y}^{\\mathrm{ref}}}$"   ]
    y_ref_p_labels_t  = [ "d/dt x_ref",                          "d/dt y_ref"                            ]
    y_ref_pp_labels   = [ "$\\Large{\\ddot{x}^{\\mathrm{ref}}}$", "$\\Large{\\ddot{y}^{\\mathrm{ref}}}$" ]
    y_ref_pp_labels_t = [ "d^2/dt^2 x_ref",                      "d^2/dt^2 y_ref"                        ]

    y_d_labels      = [ "$\\Large{x^d}$",         "$\\Large{y^d}$"        ]
    y_d_labels_t    = [ "x_d",                    "y_d"                   ]
    y_d_p_labels    = [ "$\\Large{\\dot{x}^d}$",  "$\\Large{\\dot{y}^d}$" ]
    y_d_p_labels_t  = [ "d/dt x_d",               "d/dt y_d"              ]
    y_d_pp_labels   = [ "$\\Large{\\ddot{x}^d}$", "$\\Large{\\ddot{y}^d}$" ]
    y_d_pp_labels_t = [ "d^2/dt^2 x_d",           "d^2/dt^2 y_d"          ]

    yy_opt_labels   = [ y_opt_labels   , y_opt_p_labels  , y_opt_pp_labels   ]
    yy_opt_labels_t = [ y_opt_labels_t , y_opt_p_labels_t, y_opt_pp_labels_t ]
    yy_ref_labels   = [ y_ref_labels   , y_ref_p_labels  , y_ref_pp_labels   ]
    yy_ref_labels_t = [ y_ref_labels_t , y_ref_p_labels_t, y_ref_pp_labels_t ]
    yy_d_labels     = [ y_d_labels     , y_d_p_labels    , y_d_pp_labels     ]
    yy_d_labels_t   = [ y_d_labels_t   , y_d_p_labels_t  , y_d_pp_labels_t   ]

    yy_d   = [y_d,   y_d_p,   y_d_pp]
    if plot_yref:
        yy_ref = [y_ref, y_ref_p, y_ref_pp]
    yy_opt = [y_opt, y_opt_p, y_opt_pp]

    tau_labels   = ["$\\Large{\\tau_1}$", "$\\Large{\\tau_2}$"]
    tau_labels_t = ["tau_1",              "tau_2"]

    e_x_labels = ["$\\Large{e_x}$", "$\\Large{\\dot{e}_x}$", "$\\Large{\\ddot{e}_x}$"]
    e_x_labels_t = ["e_x",          "d/dt e_x",              "d^2/dt^2 e_x"]

    e_y_labels = ["$\\Large{e_y}$", "$\\Large{\\dot{e}_y}$", "$\\Large{\\ddot{e}_y}$"]
    e_y_labels_t = ["e_y",          "d/dt e_y",              "d^2/dt^2 e_y"]

    q1_labels = ["$\\Large{q_1}$",  "$\\Large{\\dot{q}_1}$", "$\\Large{\\ddot{q}_1}$"]
    q1_labels_t = ["q_1",           "d/dt q_1",              "d^2/dt^2 q_1"]

    q2_labels = ["$\\Large{q_2}$",  "$\\Large{\\dot{q}_2}$", "$\\Large{\\ddot{q}_2}$"]
    q2_labels_t = ["q_2",           "d/dt q_2",              "d^2/dt^2 q_2"]

    # Create a Plotly subplot
    fig = make_subplots(rows=4, cols=4, shared_xaxes=False, vertical_spacing=0.05, horizontal_spacing=0.035, 
        subplot_titles=(
        "$\\Large{\\mathrm{TCP~position~(m)}}$",
        "$\\Large{e_x = x^d - x\\mathrm{~(m)}}$",
        "$\\Large{e_y = y^d - y\\mathrm{~(m)}}$",
        "$\\Large{\\mathrm{Joint~coordinates~}q\\mathrm{~(rad)}}$",
        "$\\Large{\\mathrm{TCP~velocity~(m)}}$",
        "$\\Large{\\dot{e}_x = \\dot{x}^d - \\dot{x}\\mathrm{~(m/s)}}$",
        "$\\Large{\\dot{e}_y = \\dot{y}^d - \\dot{y}\\mathrm{~(m/s)}}$",
        "$\\Large{\\mathrm{Joint~velocities~}\\dot{q}\\mathrm{~(rad/s)}}$",
        "$\\Large{\\mathrm{TCP~acceleration~(m)}}$",
        "$\\Large{\\ddot{e}_x = \\ddot{x}^d - \\ddot{x}\\mathrm{~(m/s^2)}}$",
        "$\\Large{\\ddot{e}_y = \\ddot{y}^d - \\ddot{y}\\mathrm{~(m/s^2)}}$",
        "$\\Large{\\mathrm{Joint~acceleration~}\\ddot{q}\\mathrm{~(rad/s^2)}}$",
        "$\\Large{\\mathrm{Manipulability~}w = \\sqrt{\\det(\\mathbf{J}\\mathbf{J}^\\mathrm{T})}}$",
        "$\\Large{\\mathrm{Torque~}\\boldsymbol{\\tau}~(Nm)}$",
    ))
    
    # Plot the x-components using Plotly
    for j in range(3):
        for i in range(2): # nur bis 2 wegen x und y, z ist unwichtig
            fig.add_trace(go.Scatter(x=t, y=yy_opt[j][:, i],   name=yy_opt_labels[j][i], line = line_dict_traj[i],     hoverinfo = 'x+y+text', hovertext=yy_opt_labels_t[j][i]), row=j+1, col=1)
        for i in range(2):
            fig.add_trace(go.Scatter(x=t, y=yy_d[j][  :, i],   name=yy_d_labels[j][i]  , line = line_dict_traj_dot[i], hoverinfo = 'x+y+text', hovertext=yy_d_labels_t[j][i]), row=j+1, col=1)
        if plot_yref:
            for i in range(2):
                fig.add_trace(go.Scatter(x=t, y=yy_ref[j][  :, i], name=yy_ref_labels[j][i], line = line_dict_yref_dot[i], hoverinfo = 'x+y+text', hovertext=yy_ref_labels_t[j][i]), row=j+1, col=1)

    e_x_arr = [e[:, 0], e_p[:, 0], e_pp[:, 0]]
    e_y_arr = [e[:, 1], e_p[:, 1], e_pp[:, 1]]

    q_arr = [q, q_p, q_pp]

    for i in range(3):
        fig.add_trace(go.Scatter(x=t, y=e_x_arr[i], line = line_dict_y, name = e_x_labels[i], hoverinfo = 'x+y', hovertext=e_x_labels_t[i]), row=i+1, col=2)
    for i in range(3):
        fig.add_trace(go.Scatter(x=t, y=e_y_arr[i], line = line_dict_y, name = e_y_labels[i], hoverinfo = 'x+y', hovertext=e_y_labels_t[i]), row=i+1, col=3)
    for i in range(3):
        fig.add_trace(go.Scatter(x=t, y=q_arr[i][:, 0], line = line_dict_traj[0], name = q1_labels[i], hoverinfo = 'x+y+text', hovertext=q1_labels_t[i]), row=i+1, col=4)
        fig.add_trace(go.Scatter(x=t, y=q_arr[i][:, 1], line = line_dict_traj[1], name = q2_labels[i], hoverinfo = 'x+y+text', hovertext=q2_labels_t[i]), row=i+1, col=4)

    fig.add_trace(go.Scatter(x=t, y=w, line = line_dict_y, name='$w$', hoverinfo = 'x+y', hovertext="Manip w"), row=4, col=1)
    for i in range(2):
        fig.add_trace(go.Scatter(x=t, y=tau[:, i], line = line_dict_traj[i], name = tau_labels[i], hoverinfo = 'x+y+text', hovertext=tau_labels_t[i]), row=4, col=2)

    fig.update_xaxes(title_text='$\\Large{t\\mathrm{~(s)}}$', row=4, col=1)
    fig.update_xaxes(title_text='$\\Large{t\\mathrm{~(s)}}$', row=4, col=2)
    fig.update_xaxes(title_text='$\\Large{t\\mathrm{~(s)}}$', row=3, col=3)
    fig.update_xaxes(title_text='$\\Large{t\\mathrm{~(s)}}$', row=3, col=4)


    # fig.update_layout(plot_bgcolor='#1e1e1e', paper_bgcolor='#1e1e1e', font=dict(color='#ffffff'), legend=dict(orientation='h'))
    fig.update_layout(
        plot_bgcolor='#101010',  # Set plot background color
        paper_bgcolor='#1e1e1e',  # Set paper background color
        font=dict(color='#ffffff'),  # Set font color
        legend=dict(orientation='h', yanchor='middle', y=10, yref='container'),  # Set legend orientation
        hovermode = 'closest',
        margin=dict(l=10, r=10, t=50, b=70),
        # legend_indentation = 0,
        # margin_pad=0,
        # Gridline customization for all subplots
        **{f'xaxis{i}': dict(gridwidth=1, gridcolor='#757575', linecolor='#757575', zerolinecolor='#757575', zerolinewidth=1) for i in range(1, 15)},
        **{f'yaxis{i}': dict(gridwidth=1, gridcolor='#757575', linecolor='#757575', zerolinecolor='#757575', zerolinewidth=1) for i in range(1, 15)}
    )

    fig.update_layout(
        **{f'xaxis{i}': dict(showticklabels=False) for i in range(1, 11)}
    )

    if(plot_fig):
        fig.show()

    if(save_plot):
        py.plot(fig, filename=file_name, include_mathjax='cdn')
    
    return y_opt, y_opt_p, y_opt_pp, e, e_p, e_pp, w, q, q_p, q_pp, tau

def plot_mpc_solution(ddp, i, dt, N_horizon, N_step, TCP_frame_id, robot_model, param_trajectory):
    xs = np.array(ddp.xs)
    us = np.array(ddp.us)
    t = np.arange(i*dt, (i+N_horizon)*dt, dt)
    param_trajectory_copy = {
        'p_d':    param_trajectory['p_d'][:,    i:i+N_step*N_horizon:N_step],
        'p_d_p':  param_trajectory['p_d_p'][:,  i:i+N_step*N_horizon:N_step],
        'p_d_pp': param_trajectory['p_d_pp'][:, i:i+N_step*N_horizon:N_step]
    }
    plot_solution(us, xs, t, TCP_frame_id, robot_model, param_trajectory_copy)
    
def plot_current_solution(us, xs, i, t, TCP_frame_id, robot_model, param_trajectory):
    param_trajectory_copy = {
        'p_d':    param_trajectory['p_d'][:,    0:i],
        'p_d_p':  param_trajectory['p_d_p'][:,  0:i],
        'p_d_pp': param_trajectory['p_d_pp'][:, 0:i]
    }
    plot_solution(us[0:i], xs[0:i], t[0:i], TCP_frame_id, robot_model, param_trajectory_copy)
####################################################### VIS ROBOT #################################################

def visualize_robot(robot, q_sol, param_trajectory, dt, rep_cnt = np.inf, rep_delay_sec=1):
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

    # Iterate through trajectory data (assuming q_sol[:, 6:6+robot_model.nq] represents joint positions)
    for i in range(len(q_sol)):
        pinocchio.forwardKinematics(robot_model, robot_data, q_sol[i])
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
    #     robot_display.robot.viz.play(q_sol[:, 6:6+robot_model.nq], dt)
    #     i = i +1
    # ffmpeg -r 60 -i "%07d.jpg" -vcodec "libx264" -preset "slow" -crf 18 -vf pad="width=ceil(iw/2)*2:height=ceil(ih/2)*2" output.mp4
    # ffmpeg -r 60 -i "%07d.png" -vcodec "libx264" -preset "slow" -crf 18 -vf pad="width=ceil(iw/2)*2:height=ceil(ih/2)*2" output.mp4

    time.sleep(1) # othervise visualization don't work
    
    create_video=False
    if create_video:
        robot_display = crocoddyl.MeshcatDisplay(robot, -1, 1, False, visibility=False)
        with robot_display.robot.viz.create_video_ctx("test.mp4"):
            robot_display.robot.viz.play(q_sol, dt)