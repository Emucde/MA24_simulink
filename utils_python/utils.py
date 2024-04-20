import numpy as np
from scipy.spatial.transform import Rotation
import time
import crocoddyl
import pinocchio

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