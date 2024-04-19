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

class ExtendedDynamicsModel(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self, robot, new_states_input):
        self.new_states_input = new_states_input
        self.robot = robot
        self.nq = robot.model.nq + 2
        self.nv = robot.model.nv + new_states_input

        # initialize the new states
        self.new_states = np.zeros(new_states_input)
        self.new_state_dot = np.zeros(new_states_input)
        self.u = np.zeros(robot.model.nv + new_states_input)

    def calc(self, data, x, u):
        # calculate the dynamics of the original model
        f = self.robot.forwardKinematics(x[:self.robot.model.nq], x[self.robot.model.nq:])
        fdot = self.robot.forwardDynamics(x[:self.robot.model.nq], x[self.robot.model.nq:], u[:self.robot.model.nv])

        # calculate the dynamics of the new states
        self.new_state_dot[:-1] = self.new_states[1:]
        self.new_state_dot[-1] = u[-1]

        # set the state dot
        data.state_dot.x[:self.robot.model.nq] = fdot
        data.state_dot.x[self.robot.model.nq:] = self.new_state_dot

    #def calcDiff(self, data, x, u):
    #    # calculate the derivatives of the dynamics of the original model
    #    # ...
    #
    #    # calculate the derivatives of the dynamics of the new states
    #    # ...

class CustomState(crocoddyl.StateAbstract):
    def __init__(self, urdf_path):
        # Load Pinocchio model from URDF file
        self.robot = pinocchio.robot_wrapper.RobotWrapper.BuildFromURDF(urdf_path)
        self.model = self.robot.model

        # Define state dimension
        self.nq_ = self.model.nq
        self.nv_ = self.model.nv
        self.nx_ = self.nq_ + self.nv_

        # Initialize state with correct dimensions
        crocoddyl.StateAbstract.__init__(self, self.nx_, self.nx_)

    def zero(self):
        q = pinocchio.neutral(self.model)
        v = pinocchio.zero(self.model.nv)
        return pinocchio.SE3(q, v)

    def integrate(self, x, dx):
        q, v = pinocchio.decompose(x)
        v += dx
        return pinocchio.SE3(q, v)

    def Jintegrate(self, x, dx):
        q, v = pinocchio.decompose(x)
        J = pinocchio.JointData(self.model)
        pinocchio.computeJointJacobians(self.model, q, J)
        Jv = J.matrix[self.model.nv:, :]
        return Jv

    def JintegrateTransport(self, x, dx):
        q, v = pinocchio.decompose(x)
        J = pinocchio.JointData(self.model)
        pinocchio.computeJointJacobians(self.model, q, J)
        Jv = J.matrix[self.model.nv:, :]
        return Jv

    def createData(self):
        return CustomData(self.model, self)

class CustomData(crocoddyl.ActionDataAbstract):
    def __init__(self, state):
        crocoddyl.StateDataAbstract.__init__(self, state)

    def copy(self):
        return pinocchio.SE3(self.q, self.v)

    def cost(self):
        return 0.0

    def costDiff(self):
        return pinocchio.SE3(self.q, self.v), pinocchio.SE3(self.q, self.v)

    def costQuad(self):
        return 0.0
    

class CustomJointSphericalWithVelocityInput(pinocchio.JointModelSpherical):
    """
    Erweitertes sphärisches Gelenkmodell mit zusätzlichem Eingang v für die Dynamik d/dt [z1, z2] = [z2, v].
    """

    def __init__(self, jidx, parent_jidx, name, joint_placement):
        super().__init__(jidx, parent_jidx, name, joint_placement)
        self.v = pinocchio.SE3.Identity()  # Eingabe für die Rotationsgeschwindigkeit

    def computeJointJacobian(self, model, data, jacobian):
        """
        Berechnet die Jacobian-Matrix des Gelenks.
        """
        super().computeJointJacobian(model, data, jacobian)

        # Hinzufügen einer zusätzlichen Spalte für den Eingang v
        jacobian[3, 0] = 0.0  # x-Komponente der Rotationsgeschwindigkeit
        jacobian[4, 0] = 0.0  # y-Komponente der Rotationsgeschwindigkeit
        jacobian[5, 0] = 1.0  # z-Komponente der Rotationsgeschwindigkeit

    def computeJointAcceleration(self, model, data, acceleration):
        """
        Berechnet die Gelenkbeschleunigung.
        """
        super().computeJointAcceleration(model, data, acceleration)

        # Implementieren Sie die Dynamik d/dt [z1, z2] = [z2, v]
        acceleration[0] = data.v[self.jointIndex + 3]  # z1_dot = z2
        acceleration[1] = self.v[2]  # z2_dot = v

###################### HERE ########################################################


class DifferentialActionModelPinocchio2222(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self):
        crocoddyl.DifferentialActionModelAbstract.__init__(
            self, crocoddyl.StateVector(2), 1, 2
        )  # nu = 1; nr = 2
        self.unone = np.zeros(self.nu)

        self.m1 = 1.0
        self.m2 = 0.1
        self.l = 0.5
        self.g = 9.81
        self.costWeights = [
            1.0,
            0.1,
            1.0,
        ]  # z1, z2, u

    def calc(self, data, x, u=None):
        if u is None:
            u = self.unone
        # Getting the state and control variables
        z1, z2 = x[0], x[1]
        u = u[0]

        # Defining the equation of motions
        z1dot = z2
        z2dot = u

        data.xout = np.matrix([z1dot, z2dot]).T

        # Computing the cost residual and value
        data.r = np.matrix(self.costWeights * np.array([z1, z2, u])).T
        data.cost = 0.5 * sum(np.asarray(data.r) ** 2)


class DifferentialActionModelPinocchio(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self, y_d, y_d_p, y_d_pp):
        self.m = 3

        crocoddyl.DifferentialActionModelAbstract.__init__(
            self, crocoddyl.StateVector(2*self.m), self.m, self.m
        )  # nu = 1; nr = 2

        self.y_d    = y_d
        self.y_d_p  = y_d_p
        self.y_d_pp = y_d_pp

        self.Kd = 100*np.eye(self.m)
        self.Kp = 2500*np.eye(self.m)

        self.Kp_squarex2 = 2*self.Kp**2
        self.Kd_squarex2 = 2*self.Kd**2

    def calc(self, data, x, u):
        # Getting the state and control variables
        m = self.m
        z1 = x[0:m]
        z2 = x[m:2*m]

        alpha  = u[0:m]

        # Defining the equation of motions
        z1dot = z2
        z2dot = alpha

        z1ddot = alpha

        #data.xout = np.matrix([z1dot, z2dot]).T#np.array([z1dot, z2dot]).T
        #data.xout = np.matrix([z1ddot]).T#np.array([z1dot, z2dot]).T
        data.xout = z1ddot
        # data.r = np.matrix(self.costWeights * np.array([z1, z2, u])).T
        # data.cost = 0.5 * sum(np.asarray(data.r) ** 2)
        data.r = (alpha - self.y_d_pp) + self.Kd @ (z2 - self.y_d_p) + self.Kp @ (z1 - self.y_d)
        data.cost = np.linalg.norm(data.r)**2

    def calcDiff(self, data, x, u):
        m=self.m
        z1 = x[0:m]
        z2 = x[m:2*m]
        #O = np.zeros((m,m))
        I = np.eye(m)
        Kp_squarex2 = self.Kp_squarex2
        Kd_squarex2 = self.Kd_squarex2

        # data.Fx = np.block([O, O]) # f = d^2/dt^2 z2 = u # ist eh standardmäßig 0
        data.Fu = I

        #data.Lx  = np.block([2*Kp_square @ (z1 - self.y_d), 2*Kd_square @ (z2 - self.y_d_p)])
        data.Lx[:m]  = Kp_squarex2 @ (z1 - self.y_d)
        data.Lx[-m:]  = Kd_squarex2 @ (z2 - self.y_d_p)

        data.Lu = 2*(u - self.y_d_pp)
        
        #data.Lxx = np.block([[2*Kp_square, O], [O, 2*Kd_square]])
        data.Lxx[:m, :m] = Kp_squarex2
        data.Lxx[-m:, -m:] = Kd_squarex2
        data.Luu = 2*I
        #data.Lxu = np.block([[O],[O]]) # is eh standardmäßig 0

def block_diag_onlydiagmatr(a, b):
    return np.kron(a, b)

def block_diag(a, b):
    an, am = a.shape
    bn, bm = b.shape
    return np.block([[a, np.zeros((an, bm))], [np.zeros((bn, am)), b]])

class CombinedActionModel(crocoddyl.ActionModelAbstract):
    def __init__(self, model1, model2):
        crocoddyl.ActionModelAbstract.__init__(self, crocoddyl.StateVector(model1.state.nx + model2.state.nx), model1.nu + model2.nu, model1.nr + model2.nr)
        self.model1 = model1
        self.model2 = model2
        self.data1 = self.model1.createData()
        self.data2 = self.model2.createData()
        self.m = 3

    def calc(self, data, x, u):
        m=self.m
        self.model1.calc(self.data1, x[0:2*m],  u[0:m] )
        self.model2.calc(self.data2, x[2*m:2*m+4], u[m:m+2])
        data.xnext = np.hstack([self.data1.xnext, self.data2.xnext])
        data.cost  = self.data1.cost + self.data2.cost

    def calcDiff(self, data, x, u):
        m=self.m
        self.model1.calcDiff(self.data1, x[0:2*m],  u[0:m] )
        self.model2.calcDiff(self.data2, x[2*m:2*m+4], u[m:m+2])

        data.Fx = block_diag(self.data1.Fx, self.data2.Fx)
        data.Fu = block_diag(self.data1.Fu, self.data2.Fu)

        data.Lx  = np.hstack([self.data1.Lx, self.data2.Lx])
        data.Lu  = np.hstack([self.data1.Lu, self.data2.Lu])
        data.Lxx = block_diag(self.data1.Lxx, self.data2.Lxx)
        data.Luu = block_diag(self.data1.Luu, self.data2.Luu)
        data.Lxu = block_diag(self.data1.Lxu, self.data2.Lxu)

class IntState(crocoddyl.StateAbstract):
# ist das gleiche wie "crocoddyl.StateVector(2)"
    def __init__(self):
        crocoddyl.StateAbstract.__init__(self, 2, 2)  # 2 states (z1, z2), 1 input (u)

    def zero(self):
        return self.zero()

    def integrate(self, x, dx):
        return np.array(x+dx)
    

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
    
# class DifferentialFwdDynamics2(crocoddyl.DifferentialActionModelFreeFwdDynamics):
#     def __init__(self, state, actuationModel, costModel):
#         crocoddyl.DifferentialActionModelFreeFwdDynamics.__init__(
#             self, state, actuationModel, costModel
#         )

#     def calc(self, data, x, u):
#         super(DifferentialFwdDynamics2, self).calc(data, x, u)
#         data.costs = 0

class DifferentialFwdDynamics3(crocoddyl.DifferentialActionModelAbstract):
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