import numpy as np
from scipy.spatial.transform import Rotation
import time
import crocoddyl

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