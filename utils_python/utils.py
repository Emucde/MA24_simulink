import numpy as np
from scipy.spatial.transform import Rotation
import scipy as sp
from scipy.signal import savgol_filter
import time
import crocoddyl
import pinocchio
import matplotlib.pyplot as plt
import meshcat.geometry as g
import meshcat.transformations as tf
import webbrowser
import os
import sys
import json
from meshcat.animation import Animation
from pinocchio.visualize import MeshcatVisualizer
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.offline as py
from typing import overload
from bs4 import BeautifulSoup
from multiprocessing import shared_memory, resource_tracker
import asyncio
import websockets
import psutil
import fcntl

class DebouncedButton:
    def __init__(self, delay):
        self.delay = delay
        self.last_press_time = 0

    def debounce(self, value):
        current_time = time.time()

        if value == 1 and (current_time - self.last_press_time > self.delay):
            self.last_press_time = current_time
            return 1
        else:
            return 0

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

def create_poly_traj(yT, y0, t, R_init, rot_ax, rot_alpha_scale, param_traj_poly):
    T_start = param_traj_poly['T_start']
    T_poly = param_traj_poly['T_poly']
    
    if t - T_start < 0:
        p_d = np.concatenate((y0[:3], [0]))
        p_d_p = np.concatenate((np.zeros(3), [0]))
        p_d_pp = np.concatenate((np.zeros(3), [0]))
    elif t - T_start > T_poly:
        p_d = np.concatenate((yT[:3], [rot_alpha_scale]))
        p_d_p = np.concatenate((np.zeros(3), [0]))
        p_d_pp = np.concatenate((np.zeros(3), [0]))
    else:
        y0 = np.concatenate((y0[:3], [0]))
        yT = np.concatenate((yT[:3], [rot_alpha_scale]))  # poly contains [x, y, z, alpha]
        p_d, p_d_p, p_d_pp = trajectory_poly(t - T_start, y0, yT, T_poly)
    
    alpha = p_d[3]
    alpha_p = p_d_p[3]
    alpha_pp = p_d_pp[3]
    
    skew_ew = np.array([[0, -rot_ax[2], rot_ax[1]],
                        [rot_ax[2], 0, -rot_ax[0]],
                        [-rot_ax[1], rot_ax[0], 0]])
    
    R_act = (np.eye(3) + np.sin(alpha) * skew_ew + (1 - np.cos(alpha)) * skew_ew @ skew_ew) @ R_init
    
    x_d = {}
    x_d['p_d'] = p_d[:3]
    x_d['p_d_p'] = p_d_p[:3]
    x_d['p_d_pp'] = p_d_pp[:3]
    x_d['R_d'] = R_act
    x_d['q_d'] = np.roll(Rotation.from_matrix(R_act).as_quat(), 1) # as_quat has xyzw format: after roll: wxyz
    x_d['omega_d'] = alpha_p * rot_ax
    x_d['omega_d_p'] = alpha_pp * rot_ax
    
    return x_d

def generate_trajectory(dt, xe0, xeT, R_init, R_target, param_traj_poly, plot_traj=False):
    T_start = param_traj_poly['T_start']
    T_poly = param_traj_poly['T_poly']
    T_end = param_traj_poly['T_end']

    if 'T_max_horizon' in param_traj_poly.keys():
        T_max_horizon = param_traj_poly['T_max_horizon']
        T_end += T_max_horizon
    else:
        T_max_horizon = 0

    t = np.arange(0, T_end, dt)

    # normalize time
    # param_traj_poly['T_start'] = 0
    # param_traj_poly['T_poly'] = T_poly - T_start
    # param_traj_poly['T_end'] = T_end - T_start

    # t = t_true - T_start

    N = len(t)
    p_d = np.zeros((3, N))
    p_d_p = np.zeros((3, N))
    p_d_pp = np.zeros((3, N))
    R_d = np.zeros((3, 3, N))
    q_d = np.zeros((4, N))
    omega_d = np.zeros((3, N))
    omega_d_p = np.zeros((3, N))

    # RR = np.dot(R_init.T, R_target)
    RR = R_target @ R_init.T
    rot_quat = np.roll(Rotation.from_matrix(RR).as_quat(), 1) # as_quat has xyzw format: after roll: wxyz
    rot_rho = rot_quat[0]
    rot_alpha_scale = 2 * np.arccos(rot_rho)
    if rot_alpha_scale == 0:
        rot_ax = np.array([0,0,0]) # random axis because rotation angle is 0
    else:
        rot_ax = rot_quat[1:4] / np.sin(rot_alpha_scale / 2)

    if(rot_alpha_scale > np.pi):
        rot_alpha_scale = 2*np.pi - rot_alpha_scale
        rot_ax = -rot_ax

    for i in range(N):
        x_d = create_poly_traj(xeT, xe0, t[i], R_init, rot_ax, rot_alpha_scale, param_traj_poly)
        p_d[:, i]       = x_d['p_d']
        p_d_p[:, i]     = x_d['p_d_p']
        p_d_pp[:, i]    = x_d['p_d_pp']
        R_d[:, :, i]    = x_d['R_d']
        q_d[:, i]       = x_d['q_d']
        omega_d[:, i]   = x_d['omega_d']
        omega_d_p[:, i] = x_d['omega_d_p']

    traj_data = {'t': t, 'N_traj': N, 'p_d': p_d, 'p_d_p': p_d_p, 'p_d_pp': p_d_pp, 'R_d': R_d, 'q_d': q_d, 'omega_d': omega_d, 'omega_d_p': omega_d_p}

    if plot_traj:
        plot_trajectory(traj_data)

    return traj_data

def create_custom_trajectory(q_k, param_target, TCP_frame_id, robot_model, robot_data, mpc_settings, param_traj_poly, plot_traj=False):
    # generate init trajectory
    xeT = param_target['p_d']
    R_target = param_target['R_d']

    pinocchio.forwardKinematics(robot_model, robot_data, q_k)
    pinocchio.updateFramePlacements(robot_model, robot_data)
    
    xe0 = robot_data.oMf[TCP_frame_id].translation
    R_init = robot_data.oMf[TCP_frame_id].rotation

    dt = mpc_settings['Ts']
    return generate_trajectory(dt, xe0, xeT, R_init, R_target, param_traj_poly, plot_traj=plot_traj)

def create_transient_trajectory(q_k, TCP_frame_id, robot_model, robot_data, traj_data, mpc_settings, param_traj_poly, plot_traj=False):

    # Extract data from y_d_data
    p_d = traj_data['p_d']
    p_d_p = traj_data['p_d_p']
    p_d_pp = traj_data['p_d_pp']
    R_d = traj_data['R_d']
    q_d = traj_data['q_d']
    omega_d = traj_data['omega_d']
    omega_d_p = traj_data['omega_d_p']
    N_traj_true = traj_data['N_traj_true'] # without the data at the end needed for mpc

    # generate init trajectory
    pinocchio.forwardKinematics(robot_model, robot_data, q_k)
    pinocchio.updateFramePlacements(robot_model, robot_data)

    R_target = R_d[:, :, 0]
    xeT = p_d[:, 0]
    
    xe0 = robot_data.oMf[TCP_frame_id].translation
    R_init = robot_data.oMf[TCP_frame_id].rotation

    dt = mpc_settings['Ts']
    init_traj_data = generate_trajectory(dt, xe0, xeT, R_init, R_target, param_traj_poly, plot_traj=plot_traj)
    
    N_init = init_traj_data['N_traj']
    # Extract data from init_traj_data
    p_d_init = init_traj_data['p_d']
    p_d_p_init = init_traj_data['p_d_p']
    p_d_pp_init = init_traj_data['p_d_pp']
    R_d_init = init_traj_data['R_d']
    q_d_init = init_traj_data['q_d']
    omega_d_init = init_traj_data['omega_d']
    omega_d_p_init = init_traj_data['omega_d_p']

    # Merge trajectories using numpy.hstack
    merged_p_d = np.hstack((p_d_init, p_d))
    merged_p_d_p = np.hstack((p_d_p_init, p_d_p))
    merged_p_d_pp = np.hstack((p_d_pp_init, p_d_pp))
    
    # For R_d, we need to handle the 3D array differently
    N_init = R_d_init.shape[2]
    N_traj = R_d.shape[2]
    merged_R_d = np.zeros((3, 3, N_init+N_traj))
    merged_R_d[:, :, :N_init] = R_d_init
    merged_R_d[:, :, N_init:] = R_d

    merged_q_d = np.hstack((q_d_init, q_d))
    merged_omega_d = np.hstack((omega_d_init, omega_d))
    merged_omega_d_p = np.hstack((omega_d_p_init, omega_d_p))

    # Create merged trajectory dictionary
    merged_trajectory = {
        'p_d': merged_p_d,
        'p_d_p': merged_p_d_p,
        'p_d_pp': merged_p_d_pp,
        'R_d': merged_R_d,
        'q_d': merged_q_d,
        'omega_d': merged_omega_d,
        'omega_d_p': merged_omega_d_p,
        'N_traj': N_traj_true,
        'N_init': N_init,
        'N_total': N_init + N_traj_true,
        't': np.arange(1,N_init + N_traj_true + 1)*dt
    }

    return merged_trajectory

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
        return elapsed_time

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
        return self.elapsed_total_time

    def reset(self):
        self.start_time = None
        self.elapsed_total_time = 0

def format_freq(frequency, precision):
    if frequency == 0:
        return "0 Hz"
    elif frequency < 1e3:
        return f"{frequency:.{precision}f} Hz"
    elif frequency < 1e6:
        return f"{frequency/1e3:.{precision}f} kHz"
    else:
        return f"{frequency/1e6:.{precision}f} MHz"

def format_time(elapsed_time, precision=2):
    """Formats the elapsed time based on its magnitude."""
    if elapsed_time == 0:
        return "0 s"
    elif elapsed_time < 1e-6:
        return f"{elapsed_time * 1e9:.{precision}f} ns"  # Nanoseconds
    elif elapsed_time < 1e-3:
        return f"{elapsed_time * 1e6:.{precision}f} µs"  # Microseconds
    elif elapsed_time < 1:
        return f"{elapsed_time * 1e3:.{precision}f} ms"  # Milliseconds
    elif elapsed_time < 60:
        return f"{elapsed_time:.{precision}f} s"         # Seconds
    else:
        minutes = int(elapsed_time // 60)
        seconds = elapsed_time % 60
        return f"{minutes} min {seconds:.{precision}f} s"

def format_long_list(time_list, max_entries=8):
    if len(time_list) <= max_entries:
        return str(time_list).replace("'", "")
    else:
        formatted_start = str(time_list[:max_entries]).replace("'", "")[:-1]  # Remove last bracket
        return f"{formatted_start}, ..., {time_list[-2]}, {time_list[-1]}]"

def block_diag_onlydiagmatr(a, b):
    return np.kron(a, b)

def block_diag(a, b):
    an, am = a.shape
    bn, bm = b.shape
    return np.block([[a, np.zeros((an, bm))], [np.zeros((bn, am)), b]])

def smooth_signal_savgol(signal, window_length, poly_order):
    """
    Smooths a signal using the Savitzky-Golay filter while maintaining the original length.
    
    Parameters:
    signal (array-like): The input signal to be smoothed.
    window_length (int): The length of the smoothing window. Must be odd.
    poly_order (int): The order of the polynomial for filtering.
    
    Returns:
    numpy.ndarray: The smoothed signal with the same length as the input signal.
    """
    if window_length % 2 == 0:
        raise ValueError("window_length must be odd.")
    
    # Convert the signal to a numpy array
    signal = np.array(signal)
    
    # Calculate the number of samples to pad at each end
    pad_size = window_length // 2
    
    # Pad the signal at both ends
    padded_signal = np.pad(signal, (pad_size, pad_size), mode='edge')
    
    # Apply the Savitzky-Golay filter to the padded signal
    smoothed_signal = savgol_filter(padded_signal, window_length, poly_order)
    
    # Remove the extra samples to restore the original length
    final_smoothed_signal = smoothed_signal[pad_size:-pad_size]
    
    return final_smoothed_signal

def process_trajectory_data(traj_select, traj_data_all, traj_param_all):
    # Extract N_traj_true and t
    N_traj_true = traj_data_all['N'][0, 0][0, 0]
    t = traj_data_all['t'][0, 0][0, :]

    # Extract trajectory data for the selected trajectory
    p_d = traj_data_all['p_d'][0, 0][:, :, traj_select]
    p_d_p = traj_data_all['p_d_p'][0, 0][:, :, traj_select]
    p_d_pp = traj_data_all['p_d_pp'][0, 0][:, :, traj_select]
    R_d = traj_data_all['R_d'][0, 0][:, :, :, traj_select]
    q_d = traj_data_all['q_d'][0, 0][:, :, traj_select]
    omega_d = traj_data_all['omega_d'][0, 0][:, :, traj_select]
    omega_d_p = traj_data_all['omega_d_p'][0, 0][:, :, traj_select]

    # Create traj_data dictionary
    traj_data = {
        'p_d': p_d,
        'p_d_p': p_d_p,
        'p_d_pp': p_d_pp,
        'R_d': R_d,
        'q_d': q_d,
        'omega_d': omega_d,
        'omega_d_p': omega_d_p,
        't': t,
        'N_traj_true': N_traj_true
    }

    # Extract initial configuration data
    n_indices = slice(None)  # Assuming n_indices is not defined, we use all indices
    q_0 = traj_param_all['q_0'][0, 0][:, traj_select]
    q_0_p = traj_param_all['q_0_p'][0, 0][:, traj_select]
    q_0_pp = traj_param_all['q_0_pp'][0, 0][:, traj_select]

    # Create traj_init_config dictionary
    traj_init_config = {
        'q_0': q_0,
        'q_0_p': q_0_p,
        'q_0_pp': q_0_pp,
        'N_traj_true': N_traj_true
    }

    return traj_data, traj_init_config

def get_trajectory_data(traj_data, traj_init_config, n_indices):
    p_d = traj_data['p_d']
    p_d_p = traj_data['p_d_p']
    p_d_pp = traj_data['p_d_pp']
    R_d = traj_data['R_d']
    # q_0 = traj_init_config['q_0'][n_indices]
    # q_0_p = traj_init_config['q_0_p'][n_indices]
    q_0_pp = traj_init_config['q_0_pp'][:]
    return p_d, p_d_p, p_d_pp, R_d, q_0_pp

def load_mpc_config(robot_model, file_path='utils_python/mpc_weights_crocoddyl.json'):
    with open(file_path, 'r') as f:
        config = json.load(f)
    
    mpc_settings = config['mpc_settings']
    param_mpc_weight = config['param_mpc_weight']
    
    # Convert Kd and Kp to numpy arrays
    param_mpc_weight['Kd'] = np.eye(3) * param_mpc_weight['Kd'][0]
    param_mpc_weight['Kp'] = np.eye(3) * param_mpc_weight['Kp'][0]
    
    # Convert lb_y_ref_N and ub_y_ref_N to numpy arrays
    param_mpc_weight['lb_y_ref_N'] = np.array(param_mpc_weight['lb_y_ref_N'])
    param_mpc_weight['ub_y_ref_N'] = np.array(param_mpc_weight['ub_y_ref_N'])

    param_mpc_weight['umin'] = -robot_model.effortLimit#*0.05
    param_mpc_weight['umax'] = robot_model.effortLimit#*0.05
    param_mpc_weight['xmin'] = np.hstack([robot_model.lowerPositionLimit, -robot_model.velocityLimit])
    param_mpc_weight['xmax'] = np.hstack([robot_model.upperPositionLimit, robot_model.velocityLimit])
    
    return mpc_settings, param_mpc_weight

def remove_shm_from_resource_tracker():
    def fix_register(name, rtype):
        if rtype == "shared_memory":
            return
        return resource_tracker._resource_tracker.register(name, rtype)
    
    def fix_unregister(name, rtype):
        if rtype == "shared_memory":
            return
        return resource_tracker._resource_tracker.unregister(name, rtype)
    
    resource_tracker.register = fix_register
    resource_tracker.unregister = fix_unregister
    
    if "shared_memory" in resource_tracker._CLEANUP_FUNCS:
        del resource_tracker._CLEANUP_FUNCS["shared_memory"]

# Call this function before creating shared memory
remove_shm_from_resource_tracker()

def initialize_shared_memory():
    remove_shm_from_resource_tracker()
    n_dof = 7  # (input data from simulink are 7dof states q, qp)

    def create_shared_memory(name, size):
        try:
            shm = shared_memory.SharedMemory(name=name)
        except FileNotFoundError:
            shm = shared_memory.SharedMemory(name=name, size=size, create=True)
        return shm

    # Shared memory configurations
    shm_configs = {
        "data_from_python":               {"size": n_dof * 8,     "dtype": np.float64},
        "data_from_python_valid":         {"size": 1,             "dtype": np.int8},
        "data_from_simulink":             {"size": 2 * n_dof * 8, "dtype": np.float64},
        "data_from_simulink_valid":       {"size": 1,             "dtype": np.int8},
        "data_from_simulink_start":       {"size": 1,             "dtype": np.int8},
        "data_from_simulink_reset":       {"size": 1,             "dtype": np.int8},
        "data_from_simulink_stop":        {"size": 1,             "dtype": np.int8},
        "data_from_simulink_traj_switch": {"size": 1,             "dtype": np.int8},
        "readonly_mode":                  {"size": 1,             "dtype": np.int8},
        "read_traj_length":               {"size": 4,             "dtype": np.uint32},
        "read_traj_data":                 {"size": 7 * 8,         "dtype": np.float64}, # pos and quaternion
        "read_frequency":                 {"size": 1 * 8,         "dtype": np.float64}, # scalar frequency (0 if skipped)
        "read_state_data":                {"size": 2 * n_dof * 8, "dtype": np.float64}, # q and qp
        "read_control_data":              {"size": n_dof * 8,     "dtype": np.float64}, # tau
    }

    shm_objects = {}
    shm_data = {}

    # for name, config in shm_objects.items():
    #     # Create shared memory object
    #     shm_objects[name].close()
    #     shm_objects[name].unlink()

    for name, config in shm_configs.items():
        # Create shared memory object
        shm_objects[name] = create_shared_memory(name, config["size"])
        
        # Create numpy array from shared memory buffer
        if config["dtype"] == np.float64:
            shape = (config["size"] // 8,)
        elif config["dtype"] == np.uint32:
            shape = (config["size"] // 4,)
        else:
            shape = (config["size"],)
        
        shm_data[name] = np.ndarray(shape, dtype=config["dtype"], buffer=shm_objects[name].buf)
        shm_data[name][:] = 0

    return shm_objects, shm_data

def calculate_ndof_torque_with_feedforward(u, x_k_ndof, robot_model_full, robot_data_full, n, n_indices, kin_model):
    """
    Calculate n-DOF torque with feedforward for fixed joints:
    Example: For a 7DOF robot with joint 3 fixed, q ∈ Rn, qp ∈ Rn
    1. Calculate qpp = M^-1(q, qp) * (tau|tau_fixed=0 - C(q, qp) - g(q))
    2. qpp(fixed_joints) = 0
    3. Calculate tau = M(q, qp) * qpp + C(q, qp) + g(q)
    Note: This way, tau[n_indices] == tau_red. Actually, only tau for fixed joints
    is calculated so that qpp(fixed_joints) = 0.
g
    Args:
    x (np.array): Reduced state vector
    u (np.array): Control input (reduced torque)
    x_k_ndof (np.array): Full n-DOF state measurement from Simulink
    robot_model_full (pinocchio.Model): Full robot model
    robot_data_full (pinocchio.Data): Pinocchio data
    n_red (int): Number of reduced DOF (non-fixed joints)
    n (int): Total number of DOF of the full robot
    n_indices (list): Indices of non-fixed joints
    kin_model (bool): True if kinematic model is used, False if dynamic model is used

    Returns:
    np.array: Full n-DOF torque
    """

    # for n_x_indices it have to be the same as x[0] as in the measurement
    # only for fixed values the measurement is used.
    q = x_k_ndof[:n]
    q_p = x_k_ndof[n:2*n]

    if kin_model:
        q_pp = np.zeros(n)
        q_pp[n_indices] = u
    else:
        tau_red = u

        M = pinocchio.crba(robot_model_full, robot_data_full, q)
        C_rnea = pinocchio.rnea(robot_model_full, robot_data_full, q, q_p, np.zeros(n)) # = Cq_p + g
        # C = pinocchio.computeCoriolisMatrix(robot_model_full, robot_data_full, q, q_p)
        M_red = M[n_indices][:, n_indices]
        C_rnea_tilde = C_rnea[n_indices]

        q_pp_red = np.linalg.solve(M_red, tau_red - C_rnea_tilde)

        q_pp = np.zeros(n)
        q_pp[n_indices] = q_pp_red

    # v1: 2x faster than v2
    tau_full = pinocchio.rnea(robot_model_full, robot_data_full, q, q_p, q_pp)
    
    # v2: 2x slower than v1 due to tau_full[n_indices] = tau_red
    # tau_full = np.zeros(n)
    # tau_full[n_indices] = tau_red
    # tau_full[n_indices_fixed] = M[n_indices_fixed] @ q_pp + C_rnea[n_indices_fixed]

    return tau_full

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

        self.TCP_frame_id = self.model2.state.pinocchio.getFrameId('fr3_link8_tcp')
        self.q_tracking_cost = self.model2.differential.costs.costs["TCP_pose"].weight
        self.Q_tracking_cost = np.diag([self.q_tracking_cost]*3)

        self.data1 = self.model1.createData()
        self.data2 = self.model2.createData()

        self.mu_model1 = self.model1.nu
        self.mu_model2 = self.model2.nu
        self.m = self.mu_model1 + self.mu_model2

        self.nq_model1 = self.model1.state.nq
        self.nx_model1 = self.model1.state.nx
        self.nq_model2 = self.model2.state.nq
        self.nx_model2 = self.model2.state.nx
        self.n = self.nx_model1 + self.nx_model2

        self.robot_model = self.model2.state.pinocchio
        self.robot_data = self.robot_model.createData()

        self.dt_int = self.model1.dt

        self.unone = np.zeros(self.m)

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

        nq2 = self.nq_model2

        nq1 = self.nq_model1
        n1 = self.nx_model1
        n = self.n

        dt_int = self.dt_int

        yref = x[:m1]
        # y = self.data2.differential.pinocchio.oMf[self.TCP_frame_id].translation # only works with euler??
        
        robot_data = self.robot_data
        robot_model = self.robot_model
        pinocchio.forwardKinematics(robot_model, robot_data, x[n1:n1+nq2])
        pinocchio.updateFramePlacements(robot_model, robot_data)
        y = robot_data.oMf[self.TCP_frame_id].translation.copy()

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
        CombinedActionModel.__init__(self, model1, model2)
        self.combined_action_model = CombinedActionModel(model1, model2)
        self.unone = self.combined_action_model.unone

    def calc(self, data, x):
        self.combined_action_model.calc(data, x, self.unone)

    def calcDiff(self, data, x):
        self.combined_action_model.calcDiff(data, x, self.unone)

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


def ocp_problem_v1(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings, use_bounds=False):
    int_type = mpc_settings['int_method']
    N_MPC = mpc_settings['N_MPC']
    Ts_MPC = mpc_settings['Ts_MPC']
    use_dt_scale = mpc_settings['use_dt_scale']
    int_time = param_traj['int_time']

    IntegratedActionModel = get_int_type(int_type)

    p_d_ref = y_d_ref['p_d'].T

    if state.nq >= 6:
        R_d_ref = y_d_ref['R_d']

    # weights
    q_yt_common_weight = param_mpc_weight['q_yt_common_weight']
    q_yr_common_weight = param_mpc_weight['q_yr_common_weight']
    Q_yt = np.array(param_mpc_weight['Q_yt'])
    Q_yr = np.array(param_mpc_weight['Q_yr'])
    q_yt_terminal_common_weight = param_mpc_weight['q_yt_terminal_common_weight']
    q_yr_terminal_common_weight = param_mpc_weight['q_yr_terminal_common_weight']
    Q_yt_terminal = np.array(param_mpc_weight['Q_yt_terminal'])
    Q_yr_terminal = np.array(param_mpc_weight['Q_yr_terminal'])

    q_p_common_weight = param_mpc_weight['q_p_common_weight']
    R_q_p = np.array(param_mpc_weight['R_q_p'])
    q_pp_common_weight = param_mpc_weight['q_pp_common_weight']
    R_q_pp = np.array(param_mpc_weight['R_q_pp'])
    q_xprev_common_weight = param_mpc_weight['q_xprev_common_weight']
    R_xprev = np.array(param_mpc_weight['R_xprev'])
    q_ureg_cost = param_mpc_weight['q_ureg_cost']
    q_uprev_cost = param_mpc_weight['q_uprev_cost']
    q_x_bound_cost = param_mpc_weight['q_x_bound_cost']
    q_u_bound_cost = param_mpc_weight['q_u_bound_cost']
    R_x_bounds = np.array(param_mpc_weight['R_x_bounds'])
    umin = param_mpc_weight['umin']
    umax = param_mpc_weight['umax']
    xmin = param_mpc_weight['xmin']
    xmax = param_mpc_weight['xmax']
    xref = param_mpc_weight['xref']
    xprev_ref = param_mpc_weight['xprev_ref']
    uref = param_mpc_weight['uref']
    qpp_ref = param_mpc_weight['qpp_ref']

    xmean = (xmin + xmax)/2

    running_cost_models = list()
    terminalDifferentialCostModel = crocoddyl.CostModelSum(state) # darf nur eines sein:
    actuationModel = crocoddyl.ActuationModelFull(state)

    for i in range(0, N_MPC):
        dt = int_time[i]

        if use_dt_scale:
            if i == 0:
                scale = Ts_MPC/int_time[0]
            else:
                scale = Ts_MPC/int_time[i-1]
        else:
            scale = 1

        p_d = p_d_ref[i]
        if state.nq >= 6:
            R_d  = R_d_ref[:,:,i] # rotation matrix!
        
        runningCostModel = crocoddyl.CostModelSum(state)

        # create bound cost models
        if use_bounds:
            if np.sum(q_x_bound_cost) not in [0, None]:
                bounds = crocoddyl.ActivationBounds(xmin, xmax)
                activationModel = crocoddyl.ActivationModelWeightedQuadraticBarrier(bounds, R_x_bounds)
                xRegBoundCost = crocoddyl.CostModelResidual(
                    state,
                    activation=activationModel,
                    residual=crocoddyl.ResidualModelState(state, xmean)
                )
                if i < N_MPC-1:
                    runningCostModel.addCost("stateRegBound", xRegBoundCost, q_x_bound_cost)
                else:
                    terminalDifferentialCostModel.addCost("stateRegBound", xRegBoundCost, q_x_bound_cost)
            if np.sum(q_u_bound_cost) not in [0, None]:
                bounds = crocoddyl.ActivationBounds(umin, umax)
                activationModel = crocoddyl.ActivationModelQuadraticBarrier(bounds)
                uRegBoundCost = crocoddyl.CostModelResidual(
                    state,
                    activation=activationModel,
                    residual=crocoddyl.ResidualModelControl(state, np.zeros(uref.shape))
                )
                if i < N_MPC-1:
                    runningCostModel.addCost("ctrlRegBound", uRegBoundCost, q_u_bound_cost)
                else:
                    terminalDifferentialCostModel.addCost("ctrlRegBound", uRegBoundCost, q_u_bound_cost)

        # create classic residual cost models
        R_q_p_weight_vec = np.hstack([np.zeros(state.nq), R_q_p])
        R_q_pp_weight_vec = R_q_pp
        q_pCost = crocoddyl.CostModelResidual(state, crocoddyl.ActivationModelWeightedQuad(R_q_p_weight_vec), crocoddyl.ResidualModelState(state, np.zeros(state.nx)))
        q_ppCost = crocoddyl.CostModelResidual(state, crocoddyl.ActivationModelWeightedQuad(R_q_pp_weight_vec), crocoddyl.ResidualModelJointAcceleration(state, qpp_ref))
        xprevRegCost = crocoddyl.CostModelResidual(state, crocoddyl.ActivationModelWeightedQuad(R_xprev), crocoddyl.ResidualModelState(state, xprev_ref))
        uRegCost = crocoddyl.CostModelResidual(state, residual=crocoddyl.ResidualModelControl(state, uref))
        uprevCost = crocoddyl.CostModelResidual(state, residual=crocoddyl.ResidualModelControl(state, uref))

        q_ytCost = crocoddyl.CostModelResidual(state, crocoddyl.ActivationModelWeightedQuad(Q_yt), crocoddyl.ResidualModelFrameTranslation(state, TCP_frame_id, p_d))
        q_yt_terminateCost = crocoddyl.CostModelResidual(state, crocoddyl.ActivationModelWeightedQuad(Q_yt_terminal), crocoddyl.ResidualModelFrameTranslation(state, TCP_frame_id, p_d))
        if state.nq >= 6:
            q_yrCost = crocoddyl.CostModelResidual(state, crocoddyl.ActivationModelWeightedQuad(Q_yr), crocoddyl.ResidualModelFrameRotation(state, TCP_frame_id, R_d))
            q_yr_terminateCost = crocoddyl.CostModelResidual(state, crocoddyl.ActivationModelWeightedQuad(Q_yr_terminal), crocoddyl.ResidualModelFrameRotation(state, TCP_frame_id, R_d))

        if i < N_MPC-1:
            if i > 0:
                runningCostModel.addCost("TCP_pose", q_ytCost,         weight = scale * q_yt_common_weight)
                if state.nq >= 6:
                    runningCostModel.addCost("TCP_rot", q_yrCost,      weight = scale * q_yr_common_weight)

            if q_p_common_weight not in [0, None]:
                runningCostModel.addCost("q_pReg", q_pCost,        weight = scale * q_p_common_weight) # Q q_p
            if q_pp_common_weight not in [0, None]:
                runningCostModel.addCost("q_ppReg", q_ppCost,      weight = scale * q_pp_common_weight) # approx Q qpp = Q (q_p - q_p_prev)/dt
            if q_xprev_common_weight not in [0, None]:
                runningCostModel.addCost("xprevReg", xprevRegCost, weight = scale * q_xprev_common_weight) # Q xprev = Q (x-x_prev)
            if q_ureg_cost not in [0, None]:
                runningCostModel.addCost("ctrlReg", uRegCost,      weight = scale * q_ureg_cost) # Q u
            if q_uprev_cost not in [0, None]:
                runningCostModel.addCost("ctrlPrev", uprevCost,    weight = scale * q_uprev_cost) # Q (u-u_prev)

            running_cost_models.append(IntegratedActionModel(
                crocoddyl.DifferentialActionModelFreeFwdDynamics(
                    state, actuationModel, runningCostModel
                ),
                dt
            ))
        else: # i == N: # Endkostenterm
            terminalDifferentialCostModel.addCost("TCP_pose", q_yt_terminateCost,    weight = scale * q_yt_terminal_common_weight)
            if state.nq >= 6:
                terminalDifferentialCostModel.addCost("TCP_rot", q_yr_terminateCost, weight = scale * q_yr_terminal_common_weight)
            if q_p_common_weight not in [0, None]:
                terminalDifferentialCostModel.addCost("q_pReg", q_pCost,             weight = scale * q_p_common_weight)
            if q_pp_common_weight not in [0, None]:
                terminalDifferentialCostModel.addCost("q_ppReg", q_ppCost,           weight = scale * q_pp_common_weight/dt)
            if q_xprev_common_weight not in [0, None]:
                terminalDifferentialCostModel.addCost("xprevReg", xprevRegCost,      weight = scale * q_xprev_common_weight)
            if q_ureg_cost not in [0, None]:
                terminalDifferentialCostModel.addCost("ctrlReg", uRegCost,           weight = scale * q_ureg_cost)
            if q_uprev_cost not in [0, None]:
                terminalDifferentialCostModel.addCost("ctrlPrev", uprevCost,         weight = scale * q_uprev_cost)

    dt = int_time[-1]
    # integrate terminal cost model (necessary?)
    terminalCostModel = IntegratedActionModel(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(
            state, actuationModel, terminalDifferentialCostModel
        ),
        dt
    )

    # Create the shooting problem
    seq = running_cost_models
    problem = crocoddyl.ShootingProblem(x_k, seq, terminalCostModel)
    return problem

# def ocp_problem_v3(start_index, end_index, N_step, state, x0, TCP_frame_id, traj_data, param_mpc_weight, dt, int_type='euler', use_bounds=False):
def ocp_problem_v3(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings, use_bounds=False):
    ###################
    # Reihenfolge beachten:
    # Zuerst Model1: yref Model (yref = [x1,x2,x3], d/dt yref = [x4,x5,x6]), 
    # dann Model2: Robot model (q = [q1, q2] = [x7, x8], d/dt q = d/dt [q1, q2] = [x9, x10])
    # Referenzsystem nur für translatorische position verwendet nicht für rotation

    int_type = mpc_settings['int_method']
    N_MPC = mpc_settings['N_MPC']
    Ts_MPC = mpc_settings['Ts_MPC']
    int_time = param_traj['int_time']

    IntegratedActionModel = get_int_type(int_type)

    p_d_ref = y_d_ref['p_d'].T
    p_d_p_ref = y_d_ref['p_d_p'].T
    p_d_pp_ref = y_d_ref['p_d_pp'].T

    if state.nq >= 6:
        R_d_ref = y_d_ref['R_d']

    # weights
    q_tracking_cost                 = param_mpc_weight['q_tracking_cost']
    q_terminate_tracking_cost       = param_mpc_weight['q_terminate_tracking_cost']
    q_terminate_tracking_bound_cost = param_mpc_weight['q_terminate_tracking_bound_cost']
    q_xreg_terminate_cost           = param_mpc_weight['q_xreg_terminate_cost']
    q_ureg_terminate_cost           = param_mpc_weight['q_ureg_terminate_cost']
    q_xreg_cost    = param_mpc_weight['q_xreg_cost']
    q_ureg_cost    = param_mpc_weight['q_ureg_cost']
    q_x_bound_cost = param_mpc_weight['q_x_bound_cost']
    q_u_bound_cost = param_mpc_weight['q_u_bound_cost']
    Kd = param_mpc_weight['Kd']
    Kp = param_mpc_weight['Kp']
    lb_y_ref_N = param_mpc_weight['lb_y_ref_N']
    ub_y_ref_N = param_mpc_weight['ub_y_ref_N']
    umin = param_mpc_weight['umin']
    umax = param_mpc_weight['umax']
    xmin = param_mpc_weight['xmin']
    xmax = param_mpc_weight['xmax']
    xref = param_mpc_weight['xref']
    uref = param_mpc_weight['uref']

    running_cost_models = list()
    terminalDifferentialCostModel = crocoddyl.CostModelSum(state) # darf nur eines sein:
    actuationModel = crocoddyl.ActuationModelFull(state)

    for i in range(0, N_MPC):
        dt = int_time[i]

        scale = 1

        p_d = p_d_ref[i]
        p_d_p = p_d_p_ref[i]
        p_d_pp = p_d_pp_ref[i]
        if state.nq >= 6:
            R_d  = R_d_ref[:,:,i] # rotation matrix!

        yy_DAM = DifferentialActionModelRunningYref(p_d, p_d_p, p_d_pp, Kd, Kp)
        
        # data = yy_DAM.createData()
        # tic()
        # for j in range(0,100000):
        #     yy_DAM.calcDiff(data, x0, np.zeros(3))
        # toc()
        # quit()
        
        runningCostModel = crocoddyl.CostModelSum(state)

        # create bound cost models
        if use_bounds:
            if np.sum(q_x_bound_cost) not in [0, None]:
                bounds = crocoddyl.ActivationBounds(xmin, xmax)
                activationModel = crocoddyl.ActivationModelQuadraticBarrier(bounds)
                xRegBoundCost = crocoddyl.CostModelResidual(
                    state,
                    activation=activationModel,
                    residual=crocoddyl.ResidualModelState(state, xref)
                )
                if i < N_MPC-1:
                    runningCostModel.addCost("stateRegBound", xRegBoundCost, q_x_bound_cost)
                else:
                    terminalDifferentialCostModel.addCost("stateRegBound", xRegBoundCost, q_x_bound_cost)
            if np.sum(q_u_bound_cost) not in [0, None]:
                bounds = crocoddyl.ActivationBounds(umin, umax)
                activationModel = crocoddyl.ActivationModelQuadraticBarrier(bounds)
                uRegBoundCost = crocoddyl.CostModelResidual(
                    state,
                    activation=activationModel,
                    residual=crocoddyl.ResidualModelControl(state, uref)
                )
                if i < N_MPC-1:
                    runningCostModel.addCost("ctrlRegBound", uRegBoundCost, q_u_bound_cost)
                else:
                    terminalDifferentialCostModel.addCost("ctrlRegBound", uRegBoundCost, q_u_bound_cost)

        # create classic residual cost models
        xRegCost = crocoddyl.CostModelResidual(state, residual=crocoddyl.ResidualModelState(state, xref))
        uRegCost = crocoddyl.CostModelResidual(state, residual=crocoddyl.ResidualModelControl(state))

        goalTrackingCost = crocoddyl.CostModelResidual(
            state,
            residual=crocoddyl.ResidualModelFrameTranslation(
                state, TCP_frame_id, p_d
            ),
        )

        yy_NDIAM = IntegratedActionModel(yy_DAM, dt)

        if state.nq >= 6:
            goalTrackingCost_r = crocoddyl.CostModelResidual(
                state,
                residual=crocoddyl.ResidualModelFrameRotation(
                    state, TCP_frame_id, R_d
                ),
            )

        if i < N_MPC-1:
            runningCostModel.addCost("TCP_pose", goalTrackingCost, q_tracking_cost)
            if state.nq >= 6:
                runningCostModel.addCost("TCP_rot", goalTrackingCost_r, q_tracking_cost)
            if np.sum(q_xreg_cost) not in [0, None]:
                runningCostModel.addCost("stateReg", xRegCost, scale * q_xreg_cost)
            if np.sum(q_ureg_cost) not in [0, None]:
                runningCostModel.addCost("ctrlReg", uRegCost, scale * q_ureg_cost)
            # running_cost_models.append(
            #     IntegratedActionModel(
            #         CombinedDifferentialActionModel(
            #             yy_DAM, 
            #             crocoddyl.DifferentialActionModelFreeFwdDynamics(
            #                 state, actuationModel, runningCostModel
            #             )
            #         ),
            #         dt
            #     )
            # )

            running_cost_models.append(CombinedActionModel(yy_NDIAM, IntegratedActionModel(
                crocoddyl.DifferentialActionModelFreeFwdDynamics(
                    state, actuationModel, runningCostModel
                ),
                dt
            )))
        else: # i == N: # Endkostenterm
            if use_bounds:
                # cost for bounds of lb_y_ref_N <= || yref_N - y_N || <= lb_y_ref_N
                bounds = crocoddyl.ActivationBounds(lb_y_ref_N, ub_y_ref_N)
                activationModel = crocoddyl.ActivationModelQuadraticBarrier(bounds)
                terminalTrackingBoundCost = crocoddyl.CostModelResidual(
                    state,
                    activation=activationModel,
                    residual=crocoddyl.ResidualModelFrameTranslation(
                        state, TCP_frame_id, p_d # p_d wird in Klasse CombinedActionModel mit yref überschrieben.
                    ),
                )
                terminalDifferentialCostModel.addCost("TCP_pose", terminalTrackingBoundCost, q_terminate_tracking_bound_cost)

            if state.nq >= 6:
                terminalDifferentialCostModel.addCost("TCP_rot", goalTrackingCost_r, scale * q_terminate_tracking_cost)
            if np.sum(q_xreg_terminate_cost) not in [0, None]:
                terminalDifferentialCostModel.addCost("stateReg", xRegCost, scale * q_xreg_terminate_cost)
            if np.sum(q_ureg_terminate_cost) not in [0, None]:
                terminalDifferentialCostModel.addCost("ctrlReg", uRegCost, scale * q_ureg_terminate_cost)

    dt = int_time[-1]

    # integrate terminal cost model (necessary?)
    terminalCostModel = CombinedActionModelTerminal(yy_NDIAM, IntegratedActionModel(
            crocoddyl.DifferentialActionModelFreeFwdDynamics(
                state, actuationModel, terminalDifferentialCostModel
            ),
            dt # (unsicher obt dt=0 zulaessig: besser in calc ohne u beruecks.)
        ))

    # Create the shooting problem
    seq = running_cost_models
    problem = crocoddyl.ShootingProblem(x_k, seq, terminalCostModel)
    return problem


#############

def get_mpc_funs(problem_name):
    if problem_name == 'MPC_v1_soft_terminate':
        return crocoddyl.SolverDDP, first_init_guess_mpc_v1, create_ocp_problem_v1_soft, simulate_model_mpc_v1, next_init_guess_mpc_v1
        # return crocoddyl.SolverBoxFDDP, first_init_guess_mpc_v1, create_ocp_problem_v1, simulate_model_mpc_v1
        # return crocoddyl.SolverFDDP, first_init_guess_mpc_v1, create_ocp_problem_v1, simulate_model_mpc_v1
    if problem_name == 'MPC_v1_bounds_terminate':
        return crocoddyl.SolverBoxDDP, first_init_guess_mpc_v1, create_ocp_problem_v1_bounds, simulate_model_mpc_v1, next_init_guess_mpc_v1
        # return crocoddyl.SolverBoxFDDP, first_init_guess_mpc_v1, create_ocp_problem_v1, simulate_model_mpc_v1
        # return crocoddyl.SolverFDDP, first_init_guess_mpc_v1, create_ocp_problem_v1, simulate_model_mpc_v1
    elif problem_name == 'MPC_v3_soft_yN_ref':
        return  crocoddyl.SolverDDP, first_init_guess_mpc_v3, create_ocp_problem_v3_soft_yN_ref, simulate_model_mpc_v3, next_init_guess_mpc_v1
    elif problem_name == 'MPC_v3_bounds_yN_ref':
        return crocoddyl.SolverBoxDDP, first_init_guess_mpc_v3, create_ocp_problem_v3_bounds_yN_ref, simulate_model_mpc_v3, next_init_guess_mpc_v1
        # return crocoddyl.SolverFDDP, first_init_guess_mpc_v3, create_ocp_problem_v3_bounds_yN_ref, simulate_model_mpc_v3
    else:
        raise ValueError("problem_name must be 'MPC_v1_soft_terminate' | 'MPC_v1_bounds_terminate' | 'MPC_v3_soft_yN_ref' | 'MPC_v3_bounds_yN_ref'")

def create_ocp_problem_v1_soft(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings):
    return ocp_problem_v1(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings, use_bounds=False)

def create_ocp_problem_v1_bounds(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings):
    return ocp_problem_v1(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings, use_bounds=True)

def create_ocp_problem_v3_soft_yN_ref(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings):
    return ocp_problem_v3(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings, use_bounds=False)

def create_ocp_problem_v3_bounds_yN_ref(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings):
    return ocp_problem_v3(x_k, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings, use_bounds=True)

##############

def next_init_guess_mpc_v1(ddp, nq, nx, robot_model, robot_data, mpc_settings, param_traj):
    # N_MPC = mpc_settings['N_MPC']
    # int_time = param_traj['int_time']
    # dt = mpc_settings['Ts']

    # # method 1: accurate next state simulation
    # # xx_next = np.zeros((N_MPC-1, nx))
    # # for j in range(0, N_MPC-1):
    # #     xk = ddp.xs[j]
    # #     uk = ddp.us[j]

    # #     # Modell Simulation
    # #     tau_k = uk
    # #     q = xk[:nq]
    # #     q_p = xk[nq:nx]

    # #     q_next, q_p_next = sim_model(robot_model, robot_data, q, q_p, tau_k, dt)
    # #     xx_next[j] = np.hstack([q_next, q_p_next])
    
    # # # method 2: approximate dynamics
    # # d_xx_next = np.diff(ddp.xs, axis=0) / int_time[:N_MPC-1, np.newaxis]
    # # xx_next = ddp.xs[:-1] + dt * d_xx_next

    # # # Vectorized calculation for uu_next
    # dtau = np.diff(ddp.us, axis=0) / int_time[:N_MPC-2, np.newaxis]
    # uu_next = ddp.us[:-1] + dt * dtau

    # ddp.us[0:-1] = uu_next # approximativ bestimmt

    # # method 3: calculate only first two steps because this are the not equidistant steps in distance of Ts and TsMPC-Ts
    # # all other values are in distance of Ts_MPC and therefore they can be shiftet
    # q_next1, q_p_next1 = sim_model(robot_model, robot_data, ddp.xs[0][:nq], ddp.xs[0][nq:nx], ddp.us[0], dt)
    # q_next2, q_p_next2 = sim_model(robot_model, robot_data, ddp.xs[1][:nq], ddp.xs[1][nq:nx], ddp.us[1], dt)
    # ddp.xs[0] = np.hstack([q_next1, q_p_next1])
    # ddp.xs[1] = np.hstack([q_next2, q_p_next2])
    # ddp.xs[2:-1] = ddp.xs[3::]

    # # Problem: geht nur wenn alle werte in Ta abstand (TsMPC ist aber > Ta)
    # # ddp.xs[0:-1] = ddp.xs[1::] # macht aber eig kaum einen unterschied ob man method 3 oder das verwendet...
    # # ddp.us[0:-1] = ddp.xs[1::] # das macht einen großen unterschied und der fehler wird fast doppelt so groß
   
    # Die obige Variante hat große Nachteile da sie insbesonders dann, wenn die Prädiktion sehr falsch ist
    # die MPC dazu verleitet viel zu stark von der momentanen Lösung abzuweichen.

    # simply use the last value as initial guess
    xs_init_guess = ddp.xs.copy()
    us_init_guess = ddp.us.copy()

    return xs_init_guess, us_init_guess

def simulate_model_mpc_v1(xk, uk, dt, nq, nx, robot_model, robot_data, param_robot, traj_data):

    # Modell Simulation
    tau_k = uk
    q     = xk[:nq]
    q_p   = xk[nq:nx]

    q_next, q_p_next = sim_model(robot_model, robot_data, q, q_p, tau_k, dt)
    
    xkp1 = np.hstack([q_next, q_p_next])
    # np.clip(xkp1, param_robot['x_min'], param_robot['x_max'], out=xkp1)

    return xkp1

def simulate_model_mpc_v3(ddp, i, dt, nq, nx, robot_model, robot_data, param_robot, traj_data):

    xk = ddp.xs[0] # muss so sein, da x0 in ddp.xs[0] gespeichert ist
    uk = ddp.us[0]

    # Modell Simulation
    tau_k = uk[3:]
    q = xk[6:6+nq]
    q_p = xk[6+nq:6+nx]

    q_next, q_p_next = sim_model(robot_model, robot_data, q, q_p, tau_k, dt)

    xk[0:6]      = np.hstack([traj_data['p_d'][:, i+1], traj_data['p_d_p'][:, i+1]])
    xk[6:6+nq]   = q_next
    xk[6+nq:6+nx] = q_p_next

    xs_init_guess = ddp.xs
    us_init_guess = ddp.us

    us_i = uk
    xs_i = xk

    # debug for ref trajectory: show only terminal state
    # us_i = ddp.us[-1]
    # xs_i = ddp.xs[-1]

    return xk, xs_i, us_i, xs_init_guess, us_init_guess

def sim_model(robot_model, robot_data, q, q_p, tau, dt):
    q_pp     = pinocchio.aba(robot_model, robot_data, q, q_p, tau)
    q_p_next = q_p + dt * q_pp # Euler method
    q_next   = pinocchio.integrate(robot_model, q, dt * q_p_next)
    return q_next, q_p_next

###############

def first_init_guess_mpc_v1(tau_init_robot, x_0_red, N_traj, N_horizon, param_robot, traj_data):
    nu = param_robot['n_dof']
    nx = 2*nu
    xk = x_0_red

    xs_init_guess = [x_0_red]  * (N_horizon)
    us_init_guess = [tau_init_robot] * (N_horizon-1)

    xs = np.zeros((N_traj, nx))
    us = np.zeros((N_traj, nu))
    return xk, xs, us, xs_init_guess, us_init_guess

def first_init_guess_mpc_v3(tau_init_robot, x_0_red, N_traj, N_horizon, param_robot, traj_data):
    nu = param_robot['n_dof']
    nx = 2*nu

    x0_init = np.hstack([traj_data['p_d'][:, 0], traj_data['p_d_p'][:, 0], x_init_robot])
    xk = x0_init

    xs_init_guess = [x0_init] * (N_horizon)
    us_init_guess = [np.hstack([traj_data['p_d_pp'][:, 0], tau_init_robot])] * (N_horizon-1)

    xs = np.zeros((N_traj, 6+nx))
    us = np.zeros((N_traj, 3+nu))
    return xk, xs, us, xs_init_guess, us_init_guess

###################################################################################################################
def add_noise(data, gain=1e-3, mean=0, std=0.01):
    arr = np.array(data)
    noisy_data = arr + gain*np.random.normal(mean, std, arr.shape)
    return [np.array(row) for row in noisy_data]

def check_solver_status(warn_cnt, hasConverged, ddp, i, dt, conv_max_limit=5):
    error = 0
    
    if not hasConverged:
        print("\033[43mWarning: Solver did not converge at time t =", f"{i*dt:.3f}", "\033[0m")
        warn_cnt += 1
        # plot_mpc_solution(ddp, i, dt, N_horizon, N_step, TCP_frame_id, robot_model, traj_data)
        if warn_cnt > conv_max_limit:
            print("\033[91mError: Solver failed to converge", f"{conv_max_limit}", "times in a row. Exiting...\033[0m")
            error = 1
    else:
        warn_cnt = 0

    if ddp.isFeasible == False:
        # print("\033[93mWarning: Solver is not feasible at time t = ", f"{i*dt:.3f}", "\033[0m")
        print("\033[91mError: Solver is not feasible at time t =", f"{i*dt:.3f}", "\033[0m")
        error = 1

    # ist recht langsam
    # if np.isnan(ddp.xs).any() or np.isnan(ddp.us).any():
    #     # print("\033[93mWarning: NaN values detected in xs or us arrays at time t = ", f"{i*dt:.3f}", "\033[0m")
    #     print("\033[91mError: NaN values detected in xs or us arrays at time t =", f"{i*dt:.3f}", "\033[0m")
    #     error = 1
    return warn_cnt, error

def init_crocoddyl(x_k, robot_model, robot_data, robot_model_full, robot_data_full, traj_data, traj_init_config, param_robot, param_traj_poly, TCP_frame_id, use_custom_trajectory=False, param_target=None):
    # because later I add the initial trajectory to the true trajectory
    n_dof = param_robot['n_dof']

    mpc_settings, param_mpc_weight = load_mpc_config(robot_model)
   
    if use_custom_trajectory:
        transient_traj = create_custom_trajectory(x_k[:n_dof], param_target, TCP_frame_id, robot_model_full, robot_data_full, mpc_settings, param_traj_poly, plot_traj=False)
        T_max_horizon = param_traj_poly['T_max_horizon']
        dt = mpc_settings['Ts']
        N_traj = transient_traj['N_traj'] - int(T_max_horizon/dt)
    else:
        transient_traj = create_transient_trajectory(x_k[:n_dof], TCP_frame_id, robot_model_full, robot_data_full, traj_data, mpc_settings, param_traj_poly, plot_traj=False)

        N_init_traj = transient_traj['N_init']
        N_traj = traj_init_config['N_traj_true'] + N_init_traj

    n_indices = param_robot['n_indices']
    n_x_indices = param_robot['n_x_indices']
    
    p_d, p_d_p, p_d_pp, R_d, q_0_pp = get_trajectory_data(transient_traj, traj_init_config, n_indices)

    q_0 = x_k[:n_dof]
    q_0_p = x_k[n_dof:2*n_dof]

    nq = robot_model.nq
    nx = 2*nq
    nu = nq # fully actuated

    opt_type = mpc_settings['version']
    N_solver_steps = mpc_settings['solver_steps']
    N_MPC = mpc_settings['N_MPC']

    Ts = mpc_settings['Ts']  # Time step of control
    N_Ts_samples = mpc_settings['N_extra_Ts_samples']  # Time step of control
    use_T_horizon = mpc_settings['use_T_horizon']

    if use_T_horizon:
        T_horizon = mpc_settings['T_horizon']
    else:
        Ts_MPC = mpc_settings['Ts_MPC']
        T_horizon = Ts_MPC*(N_MPC-N_Ts_samples)

    # find smalles multiple of T_horizon and N_MPC:
    if np.mod(T_horizon, Ts) != 0:
        print('T_horizon is not a multiple of Ts, T_horizon rounded to nearest Ts multiple!')
    max_time_index = np.round(T_horizon/Ts).astype(int)

    N_step = (max_time_index - N_Ts_samples)//(N_MPC-N_Ts_samples)

    if use_T_horizon:
        MPC_traj_indices = np.round(np.linspace(N_Ts_samples + N_step, max_time_index, N_MPC-N_Ts_samples)).astype(int)
    else:
        N_step = int(Ts_MPC/Ts)
        if N_Ts_samples >= N_step:
            print('N_Ts_samples > N_step, N_Ts_samples set to 0!')
            N_Ts_samples = 0
        MPC_traj_indices = N_step*np.arange(1, N_MPC+1-N_Ts_samples)

    if len(np.unique(MPC_traj_indices)) != len(MPC_traj_indices):
        print('MPC_traj_indices are not unique, N_MPC too large for T_horizon, N_MPC set to 1 and T_horizon ignored!')

    if use_T_horizon:
        Ts_MPC = N_step*Ts
    
    if N_step <= 1:
        MPC_traj_indices = np.arange(0, N_MPC+1)
    else:       
        extra_indices = np.arange(0, N_Ts_samples+1)
        MPC_traj_indices = np.hstack([extra_indices, MPC_traj_indices])

    MPC_int_time = (np.hstack((MPC_traj_indices[1::] - MPC_traj_indices[0:-1], MPC_traj_indices[-1] - MPC_traj_indices[-2])))*Ts

    T_horizon = MPC_traj_indices[-1]*Ts

    str_times = format_long_list([format_time(i, 0) for i in MPC_traj_indices*Ts])
    title_text = f'T_horizon: {format_time(T_horizon, 0)}, N_MPC = {N_MPC}, timesteps = {str_times}\n'
    print(title_text)

    param_traj = {
        'traj_indices': MPC_traj_indices,
        'int_time': MPC_int_time,
    }

    y_d_ref = {
        'p_d':    p_d[   :,    0+MPC_traj_indices],
        'p_d_p':  p_d_p[ :,    0+MPC_traj_indices],
        'p_d_pp': p_d_pp[:,    0+MPC_traj_indices],
        'R_d':    R_d[   :, :, 0+MPC_traj_indices]
    }

    y_d_data = {
        'p_d':    p_d,
        'p_d_p':  p_d_p,
        'p_d_pp': p_d_pp,
        'R_d':    R_d
    }

    solver, init_guess_fun, create_ocp_problem, simulate_model, next_init_guess_fun  = get_mpc_funs(opt_type)

    # use start pose at trajectory for first init guess
    x_init_robot = x_k
    tau_init_robot = pinocchio.rnea(robot_model, robot_data, q_0[n_indices], q_0_p[n_indices], q_0_pp[n_indices])

    x_k_red, xs, us, xs_init_guess, us_init_guess = init_guess_fun(tau_init_robot, x_k[n_x_indices], N_traj, N_MPC, param_robot, traj_data)

    # create first problem:

    param_mpc_weight['xref'] = np.hstack((param_robot['x_mean'][:nq], np.zeros(nq)))
    param_mpc_weight['xprev_ref'] = x_k_red[:nx]
    param_mpc_weight['qpp_ref'] = np.zeros(nq)

    if param_robot['use_gravity']:
        g_k = pinocchio.computeGeneralizedGravity(robot_model, robot_data, x_k[:nq])
    else:
        g_k = np.zeros(nq)
    param_mpc_weight['uref'] = g_k
    
    # Create a multibody state from the pinocchio model.
    state = crocoddyl.StateMultibody(robot_model)
    state.lb = np.hstack([robot_model.lowerPositionLimit, -robot_model.velocityLimit])
    state.ub = np.hstack([robot_model.upperPositionLimit, robot_model.velocityLimit])

    problem = create_ocp_problem(x_k_red, y_d_ref, state, TCP_frame_id, param_traj, param_mpc_weight, mpc_settings)

    # Solve first optimization Problem for warm start

    ddp = solver(problem)
    hasConverged = ddp.solve(xs_init_guess, us_init_guess, N_solver_steps, False, 1e-5)
    warn_cnt, err_state = check_solver_status(0, hasConverged, ddp, 0, Ts, conv_max_limit=5)

    xs_init_guess, us_init_guess = next_init_guess_fun(ddp, nq, nx, robot_model, robot_data, mpc_settings, param_traj)

    return ddp, xs, us, xs_init_guess, us_init_guess, TCP_frame_id, N_traj, Ts, hasConverged, warn_cnt, MPC_traj_indices, N_solver_steps, simulate_model, next_init_guess_fun, mpc_settings, param_mpc_weight, transient_traj, param_traj, title_text

###########################################################################
################################# PLOTTING ################################
###########################################################################

def plot_trajectory(traj_data):
    """
    Plots the x, y, z components of p_d, p_d_p, and p_d_pp in 9 subplots.
    Args:
    traj_data: A dictionary with keys 'p_d', 'p_d_p', 'p_d_pp', 
               each containing arrays with x, y, z components over time.
    """
    fig, axes = plt.subplots(3, 3, figsize=(15, 10))  # Create a 3x3 grid of subplots
    components = ['x', 'y', 'z']
    datasets = ['p_d', 'p_d_p', 'p_d_pp']
    
    for row, dataset in enumerate(datasets):
        for col, component in enumerate(components):
            ax = axes[row, col]
            ax.plot(traj_data[dataset][row, :], label=f"{dataset}_{component}")
            ax.set_title(f"{dataset} {component}")
            ax.set_xlabel("Time Step")
            ax.set_ylabel("Value")
            ax.legend()
            ax.grid(True)
    
    # Adjust layout for better spacing
    plt.tight_layout()
    plt.show()

def calc_7dof_data(us, xs, TCP_frame_id, robot_model, robot_data, traj_data, frep_per_Ta_step, param_robot):
    N = len(xs)
    n = robot_model.nq
    n_indices = param_robot['n_indices']

    p_e = np.zeros((N, 3))
    p_e_p = np.zeros((N, 3))
    p_e_pp = np.zeros((N, 3))

    quat_e = np.zeros((N, 4))
    omega_e = np.zeros((N, 3))
    omega_e_p = np.zeros((N, 3))

    t = traj_data['t'][:N]
    N_total = len(t)

    p_d = traj_data['p_d'][:N]
    p_d_p = traj_data['p_d_p'][:N]
    p_d_pp = traj_data['p_d_pp'][:N]

    quat_d = traj_data['q_d'][:N]
    omega_d = traj_data['omega_d'][:N]
    omega_d_p = traj_data['omega_d_p'][:N]

    w = np.zeros(N)  # manipulability

    q        = np.zeros((N, n))
    q_p      = np.zeros((N, n))
    q_pp     = np.zeros((N, n))

    p_err    = np.zeros((N, 3))
    p_err_p  = np.zeros((N, 3))
    p_err_pp = np.zeros((N, 3))

    e_x = np.zeros(N)
    e_x_p = np.zeros(N)
    e_x_pp = np.zeros(N)

    e_y = np.zeros(N)
    e_y_p = np.zeros(N)
    e_y_pp = np.zeros(N)

    e_z = np.zeros(N)
    e_z_p = np.zeros(N)
    e_z_pp = np.zeros(N)

    quat_err = np.zeros((N, 4))
    omega_err = np.zeros((N, 3))
    omega_err_p = np.zeros((N, 3))

    tau = np.zeros((N, n))

    window_length = 301
    poly_order = 2
    frep_per_Ta_step_mean = smooth_signal_savgol(frep_per_Ta_step, window_length, poly_order)

    for i in range(N):
        q[i] = xs[i, 0:n]
        q_p[i] = xs[i, n:2 * n]

        tau[i] = us[i]

        q_pp[i] = pinocchio.aba(robot_model, robot_data, q[i], q_p[i], tau[i])

        pinocchio.forwardKinematics(robot_model, robot_data, q[i], q_p[i], q_pp[i])
        pinocchio.updateFramePlacements(robot_model, robot_data)

        pinocchio.computeJointJacobians(robot_model, robot_data, q[i])
        J = pinocchio.getFrameJacobian(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        pinocchio.computeJointJacobiansTimeVariation(robot_model, robot_data, q[i], q_p[i])
        J_p = pinocchio.getFrameJacobianTimeVariation(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        p_e[i] = robot_data.oMf[TCP_frame_id].translation.T.copy()
        p_e_p[i] = J[0:3, :] @ q_p[i]
        # p_e_pp[i] = J[0:3, :] @ q_pp[i] + J_p[0:3, :] @ q_p[i] # update: Bug wurde gefixt, es ist nun 1:1 wie unterer Befehl
        p_e_pp[i] = pinocchio.getFrameClassicalAcceleration(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear

        quat_e_7val = pinocchio.SE3ToXYZQUAT(robot_data.oMf[TCP_frame_id])
        quat_e_xyzw = pinocchio.Quaternion(quat_e_7val[3::])
        quat_e[i] = np.hstack([quat_e_7val[6], quat_e_7val[3:6]]) #wxyz

        omega_e[i] = pinocchio.getFrameVelocity(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED).angular
        omega_e_p[i] = pinocchio.getFrameClassicalAcceleration(robot_model, robot_data, TCP_frame_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED).angular

        J_red = J[:, n_indices]
        w[i] = np.sqrt(np.abs(sp.linalg.det(J_red @ J_red.T)))

        p_err[i] = p_e[i] - p_d[:, i]
        p_err_p[i] = p_e_p[i] - p_d_p[:, i]
        p_err_pp[i] = p_e_pp[i] - p_d_pp[:, i]

        e_x[i] = p_err[i][0]
        e_y[i] = p_err[i][1]
        e_z[i] = p_err[i][2]

        e_x_p[i] = p_err_p[i][0]
        e_y_p[i] = p_err_p[i][1]
        e_z_p[i] = p_err_p[i][2]

        e_x_pp[i] = p_err_pp[i][0]
        e_y_pp[i] = p_err_pp[i][1]
        e_z_pp[i] = p_err_pp[i][2]

        quat_d_xyzw = pinocchio.Quaternion( np.hstack([quat_d[1:, i], quat_d[0, i]]) )
        q_d_xyzw_inv = pinocchio.Quaternion.inverse(quat_d_xyzw)

        quat_err_temp = quat_e_xyzw * q_d_xyzw_inv
        quat_err[i] = np.array([quat_err_temp.w, quat_err_temp.x, quat_err_temp.y, quat_err_temp.z])

        omega_err[i] = omega_e[i] - traj_data['omega_d'][:, i]

        omega_err_p[i] = omega_e_p[i] - traj_data['omega_d_p'][:, i]

    # make data shorter, use only each N_dec sample
    N_dec = 1
    t = t[::N_dec]

    p_e = p_e[::N_dec]
    p_e_p = p_e_p[::N_dec]
    p_e_pp = p_e_pp[::N_dec]

    quat_e = quat_e[::N_dec]
    omega_e = omega_e[::N_dec]
    omega_e_p = omega_e_p[::N_dec]

    p_d = p_d[:, ::N_dec]
    p_d_p = p_d_p[:, ::N_dec]
    p_d_pp = p_d_pp[:, ::N_dec]

    quat_d = quat_d[:, ::N_dec]
    omega_d = omega_d[:, ::N_dec]
    omega_d_p = omega_d_p[:, ::N_dec]

    p_err = p_err[::N_dec]
    p_err_p = p_err_p[::N_dec]
    p_err_pp = p_err_pp[::N_dec]

    e_x = e_x[::N_dec]
    e_y = e_y[::N_dec]
    e_z = e_z[::N_dec]

    e_x_p = e_x_p[::N_dec]
    e_y_p = e_y_p[::N_dec]
    e_z_p = e_z_p[::N_dec]

    e_x_pp = e_x_pp[::N_dec]
    e_y_pp = e_y_pp[::N_dec]
    e_z_pp = e_z_pp[::N_dec]
    
    quat_err = quat_err[::N_dec]
    omega_err = omega_err[::N_dec]
    omega_err_p = omega_err_p[::N_dec]
    
    w = w[::N_dec]
    
    q = q[::N_dec]
    q_p = q_p[::N_dec]
    q_pp = q_pp[::N_dec]
    tau = tau[::N_dec]

    # create subplot data array
    subplot1 = {'title': 'p_e (m) (Y: x, B: y, P: z)', 
                'sig_labels': ['p_e_x (m)', 'p_e_y (m)', 'p_e_z (m)', 'p_d_x (m)', 'p_d_y (m)', 'p_d_z (m)'],
                'sig_xdata': t,
                'sig_ydata': [p_e[:, 0], p_e[:, 1], p_e[:, 2], p_d[0], p_d[1], p_d[2]],
                'sig_linestyles': ['-', '-', '-', '--', '--', '--'],
                'sig_colors': ['rgb(255,255,17)', 'rgb(19,159,255)', 'rgb(255,153,200)', 'rgb(0,0,255)', 'rgb(255,0,0)', 'rgb(0,127,0)']}

    subplot2 = {'title': 'ṗ_e (m/s) (Y: x, B: y, P: z)',
                'sig_labels': ['ṗ_e_x (m/s)', 'ṗ_e_y (m/s)', 'ṗ_e_z (m/s)', 'ṗ_d_x (m/s)', 'ṗ_d_y (m/s)', 'ṗ_d_z (m/s)'],
                'sig_xdata': t,
                'sig_ydata': [p_e_p[:, 0], p_e_p[:, 1], p_e_p[:, 2], p_d_p[0], p_d_p[1], p_d_p[2],
                              ],
                'sig_linestyles': ['-', '-', '-', '--', '--', '--'],
                'sig_colors': ['rgb(255,255,17)', 'rgb(19,159,255)', 'rgb(255,153,200)', 'rgb(0,0,255)', 'rgb(255,0,0)', 'rgb(0,127,0)']}
    
    subplot3 = {'title': 'p̈_e (m/s²) (Y: x, B: y, P: z)',
                'sig_labels': ['p̈_e_x (m/s²)', 'p̈_e_y (m/s²)', 'p̈_e_z (m/s²)', 'p̈_d_x (m/s²)', 'p̈_d_y (m/s²)', 'p̈_d_z (m/s²)'],
                'sig_xdata': t,
                'sig_ydata': [p_e_pp[:, 0], p_e_pp[:, 1], p_e_pp[:, 2], p_d_pp[0], p_d_pp[1], p_d_pp[2]],
                'sig_linestyles': ['-', '-', '-', '--', '--', '--'],
                'sig_colors': ['rgb(255,255,17)', 'rgb(19,159,255)', 'rgb(255,153,200)', 'rgb(0,0,255)', 'rgb(255,0,0)', 'rgb(0,127,0)']}

    # manipulate subplot data array
    subplot4 = {'title': 'Manip. w = √( det( JJ⸆ ) )',
                'sig_labels': ['Manip. w = √( det( JJ⸆ ) )'],
                'sig_xdata': t,
                'sig_ydata': [w],
                'sig_linestyles': ['-'],
                'sig_colors': ['rgb(255,255,17)']}

    subplot5 = {'title': 'e_x (m)',
                'sig_labels': ['e_x (m)'],
                'sig_xdata': t,
                'sig_ydata': [e_x],
                'sig_linestyles': ['-'],
                'sig_colors': ['rgb(255,255,17)']}
    
    subplot6 = {'title': 'ė_x (m/s)',
                'sig_labels': ['ė_x (m/s)'],
                'sig_xdata': t,
                'sig_ydata': [e_x_p],
                'sig_linestyles': ['-'],
                'sig_colors': ['rgb(255,255,17)']}
    
    subplot7 = {'title': 'ё_x (m/s²)',
                'sig_labels': ['ё_x (m/s²)'],
                'sig_xdata': t,
                'sig_ydata': [e_x_pp],
                'sig_linestyles': ['-'],
                'sig_colors': ['rgb(255,255,17)']}
    
    sig_linestyles_7dof_arr = ['-', '-', '-', '-', '-', '-', '-']
    color_7dof_rgb_arr = ['rgb(255,255,17)', 'rgb(19,159,255)', 'rgb(255,105,41)', 'rgb(100,212,19)', 'rgb(183,70,255)', 'rgb(15,255,255)', 'rgb(255,19,166)']
    
    subplot8 = {'title': 'q (rad)',
                'sig_labels': [f"q{i+1}" for i in range(n)],
                'sig_xdata': t,
                'sig_ydata': [q[:, i] for i in range(n)],
                'sig_linestyles': sig_linestyles_7dof_arr[:n],
                'sig_colors': color_7dof_rgb_arr[:n]}

    subplot9 = {'title': 'e_y (m)',
                'sig_labels': ['e_y (m)'],
                'sig_xdata': t,
                'sig_ydata': [e_y],
                'sig_linestyles': ['-'],
                'sig_colors': ['rgb(19,159,255)']}
    
    subplot10 = {'title': 'ė_y (m/s)',
                'sig_labels': ['ė_y (m/s)'],
                'sig_xdata': t,
                'sig_ydata': [e_y_p],
                'sig_linestyles': ['-'],
                'sig_colors': ['rgb(19,159,255)']}
    
    subplot11 = {'title': 'ё_y (m/s²)',
                'sig_labels': ['ё_y (m/s²)'],
                'sig_xdata': t,
                'sig_ydata': [e_y_pp],
                'sig_linestyles': ['-'],
                'sig_colors': ['rgb(19,159,255)']}
    
    subplot12 = {'title': 'q̇ (rad/s)',
                'sig_labels': [f"q̇_{i+1}" for i in range(n)],
                'sig_xdata': t,
                'sig_ydata': [q_p[:, i] for i in range(n)],
                'sig_linestyles': sig_linestyles_7dof_arr[:n],
                'sig_colors': color_7dof_rgb_arr[:n]}
                 
    subplot13 = {'title': 'e_z (m)',
                'sig_labels': ['e_z (m)'],
                'sig_xdata': t,
                'sig_ydata': [e_z],
                'sig_linestyles': ['-'],
                'sig_colors': ['rgb(255,153,200)']}
    
    subplot14 = {'title': 'ė_z (m/s)',
                'sig_labels': ['ė_z (m/s)'],
                'sig_xdata': t,
                'sig_ydata': [e_z_p],
                'sig_linestyles': ['-'],
                'sig_colors': ['rgb(255,153,200)']}
    
    subplot15 = {'title': 'ё_z (m/s²)',
                'sig_labels': ['ё_z (m/s²)'],
                'sig_xdata': t,
                'sig_ydata': [e_z_pp],
                'sig_linestyles': ['-'],
                'sig_colors': ['rgb(255,153,200)']}
    
    subplot16 = {'title': 'q̈ (rad/s²)',
                'sig_labels': [f"q̈_{i+1}" for i in range(n)],
                'sig_xdata': t,
                'sig_ydata': [q_pp[:, i] for i in range(n)],
                'sig_linestyles': sig_linestyles_7dof_arr[:n],
                'sig_colors': color_7dof_rgb_arr[:n]}
    
    subplot17 = {'title': 'quat_e(2:4)',
                'sig_labels': ['quat_e_2', 'quat_e_3', 'quat_e_4', 'quat_d_2', 'quat_d_3', 'quat_d_4'],
                'sig_xdata': t,
                'sig_ydata': [quat_e[:, 1], quat_e[:, 2], quat_e[:, 3], quat_d[1], quat_d[2], quat_d[3]],
                'sig_linestyles': ['-', '-', '-', '--', '--', '--'],
                'sig_colors': ['rgb(255,255,17)', 'rgb(19,159,255)', 'rgb(255,105,41)', 'rgb(0,0,255)', 'rgb(255,0,0)', 'rgb(0,127,0)']}
    
    subplot18 = {'title': 'ω_e (rad/s)',
                'sig_labels': ['ω_e_x (rad/s)', 'ω_e_y (rad/s)', 'ω_e_z (rad/s)', 'ω_d_x (rad/s)', 'ω_d_y (rad/s)', 'ω_d_z (rad/s)'],
                'sig_xdata': t,
                'sig_ydata': [omega_e[:, 0], omega_e[:, 1], omega_e[:, 2], omega_d[0], omega_d[1], omega_d[2]],
                'sig_linestyles': ['-', '-', '-', '--', '--', '--'],
                'sig_colors': ['rgb(255,255,17)', 'rgb(19,159,255)', 'rgb(255,105,41)', 'rgb(0,0,255)', 'rgb(255,0,0)', 'rgb(0,127,0)']}
    
    subplot19 = {'title': 'ὠ_e (rad/s²)',
                'sig_labels': ['ὠ_e_x (rad/s²)', 'ὠ_e_y (rad/s²)', 'ὠ_e_z (rad/s²)', 'ὠ_d_x (rad/s²)', 'ὠ_d_y (rad/s²)', 'ὠ_d_z (rad/s²)'],
                'sig_xdata': t,
                'sig_ydata': [omega_e_p[:, 0], omega_e_p[:, 1], omega_e_p[:, 2], omega_d_p[0], omega_d_p[1], omega_d_p[2]],
                'sig_linestyles': ['-', '-', '-', '--', '--', '--'],
                'sig_colors': ['rgb(255,255,17)', 'rgb(19,159,255)', 'rgb(255,105,41)', 'rgb(0,0,255)', 'rgb(255,0,0)', 'rgb(0,127,0)']}
    
    subplot20 = {'title': 'freq per Ta step (Hz)',
                'sig_labels': ['frep_per_Ta_step', 'frep_per_Ta_step_mean'],
                'sig_xdata': t,
                'sig_ydata': [frep_per_Ta_step, frep_per_Ta_step_mean],
                'sig_linestyles': ['-', '-'],
                'sig_colors': ['rgb(255,255,17)', 'rgb(255,0,0)']}
    
    subplot21 = {'title': 'quat_err',
                'sig_labels': ['quat_err_2', 'quat_err_3', 'quat_err_4'],
                'sig_xdata': t,
                'sig_ydata': [quat_err[:, 1], quat_err[:, 2], quat_err[:, 3]],
                'sig_linestyles': ['-', '-', '-', '-'],
                'sig_colors': ['rgb(255,255,17)', 'rgb(19,159,255)', 'rgb(255,105,41)', 'rgb(100,212,19)']}
    
    subplot22 = {'title': 'e_ω (rad/s)',
                'sig_labels': ['e_ω_x (rad/s)', 'e_ω_y (rad/s)', 'e_ω_z (rad/s)'],
                'sig_xdata': t,
                'sig_ydata': [omega_err[:, 0], omega_err[:, 1], omega_err[:, 2]],
                'sig_linestyles': ['-', '-', '-'],
                'sig_colors': ['rgb(255,255,17)', 'rgb(19,159,255)', 'rgb(255,105,41)']}
    
    subplot23 = {'title': 'ė_ω (rad/s)',
                'sig_labels': ['ė_ω_x (rad/s)', 'ė_ω_y (rad/s)', 'ė_ω_z (rad/s)'],
                'sig_xdata': t,
                'sig_ydata': [omega_err_p[:, 0], omega_err_p[:, 1], omega_err_p[:, 2]],
                'sig_linestyles': ['-', '-', '-'],
                'sig_colors': ['rgb(255,255,17)', 'rgb(19,159,255)', 'rgb(255,105,41)']}
    
    subplot24 = {'title': 'tau (Nm)',
                'sig_labels': [f"tau_{i+1}" for i in range(n)],
                'sig_xdata': t,
                'sig_ydata': [tau[:, i] for i in range(n)],
                'sig_linestyles': sig_linestyles_7dof_arr[:n],
                'sig_colors': color_7dof_rgb_arr[:n]}

    subplot_data = [subplot1, subplot2, subplot3, subplot4, subplot5, subplot6, subplot7, subplot8, subplot9, subplot10, subplot11, subplot12, subplot13, subplot14, subplot15, subplot16, subplot17, subplot18, subplot19, subplot20, subplot21, subplot22, subplot23, subplot24]

    return subplot_data

###########################################################################
###########################################################################
###########################################################################
###########################################################################
###########################################################################
###########################################################################

def start_server(broadcast_message = 'reload_plotly'):
    LOCK_FILE = '/tmp/my_server.lock'
    SCRIPT_PATH = 'main_python/websocket_fr3.py'
    PID_FILE = '/tmp/my_server.pid'

    def is_server_running():
        if os.path.exists(LOCK_FILE):
            with open(PID_FILE, 'r') as pid_file:
                pid = int(pid_file.read())
                try:
                    # Check if the process is running
                    process = psutil.Process(pid)
                    return process.is_running()
                except psutil.NoSuchProcess:
                    # If the process does not exist, remove the lock file and return False
                    os.remove(LOCK_FILE)
                    if os.path.exists(PID_FILE):
                        os.remove(PID_FILE)
                    return False
        return False

    if is_server_running():
        print("Server is already running.")
        asyncio.run(send_message(broadcast_message))
        return False

    # Erstelle Lock-Datei
    with open(LOCK_FILE, 'w') as lock_file:
        try:
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            
            # Starte das Server-Skript im Hintergrund
            pid = os.spawnl(os.P_NOWAIT, sys.executable, sys.executable, SCRIPT_PATH)
            
            # Schreibe die PID in die Datei
            with open(PID_FILE, 'w') as pid_file:
                pid_file.write(str(pid))

            print("Server started successfully.")
            return True
        except IOError:
            print("Could not acquire lock. Server may be running.")
            return False

async def send_message(message, port=8765):
    try:
        async with websockets.connect(f"ws://localhost:{port}") as websocket:
            await websocket.send(message)
            print(f"Nachricht gesendet: {message}")
            return True
    except websockets.exceptions.ConnectionClosedError:
        print("Server not running.")
    except ConnectionRefusedError as e:
        print(f"Connection refused: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    
    return False

def get_autoscale_script():
    return '''
        rec_time = 100; //ms
        prev_xrange_flag = false;
        function autoscale_function(){
            //console.log('run');
            var graphDiv = document.querySelector('.plotly-graph-div');
            //console.log(graphDiv);
            if(graphDiv == null || graphDiv.on == undefined)
            {
                setTimeout(autoscale_function, rec_time);
            }
            else
            {
                var on_event=true;

                function test(eventdata){
                    //console.log(eventdata);
                    
                    graphDiv = document.querySelector('.plotly-graph-div');
                    
                    if(on_event==true)
                    {
                        on_event=false;
                        labels = graphDiv.layout.annotations;

                        // Get changed axes
                        yaxis_change = Object.keys(eventdata).some(key => key.includes('yaxis'));
                        xaxis_change = Object.keys(eventdata).some(key => key.includes('xaxis'));

                        changed_yaxis = Object.keys(eventdata).filter(key => key.includes('yaxis'));
                        yaxis_names = changed_yaxis.map(name => name.split('.')[0]);  // get only axis name
                        yaxis_names = [...new Set(yaxis_names)]; // get unique names
                        trace_numbers = yaxis_names.map(name => name.split('yaxis')[1]); // get trace numbers from names
                        trace_numbers = trace_numbers.map(name => name === '' ? 0 : Number(name)-1); // trace number '' should be 0

                        //create a array with length of the number of traces and false for each yaxis that was not in trace_numbers
                        // and true for each yaxis that is in trace numbers
                        yaxis_in_trace = Array.from({length: labels.length}, (_, i) => trace_numbers.includes(i));

                        if(prev_xrange_flag)
                        {
                            // get xrange from local storage and convert it to array of doubles
                            xrange = localStorage.getItem('xrange').split(',').map(Number);
                            prev_xrange_flag=false;
                            xaxis_change=true;
                            console.log(xrange);

                            update={};
                            labels.forEach(function(act_label, i){
                                if(i == 0)
                                {
                                    xlabel='xaxis.range';
                                }
                                else
                                {
                                    xlabel='xaxis'+(1+i)+'.range';
                                }
                                update[xlabel] = xrange;
                            });
                        }
                        else
                        {
                            xrange = graphDiv.layout.xaxis.range;
                            localStorage.setItem('xrange', xrange);
                            console.log(xrange);
                            update={};
                        }
                    
                        labels.forEach(function(act_label, i){

                            trace = graphDiv.data.filter(trace => trace.text.includes(labels[i].text));

                            yaxisName = i === 0 ? 'yaxis' : `yaxis${i + 1}`;
                            yrange = graphDiv.layout[yaxisName].range;
                            filteredIndices = trace[0].x.map((x, index) => x >= xrange[0] && x <= xrange[1] ? index : -1).filter(index => index !== -1);
                            //filteredX = filteredIndices.map(index => trace[0].x[index]);
                            
                            g_ymax=-Infinity;
                            g_ymin=Infinity;
                            trace.forEach(function(el,id){
                                filteredY = filteredIndices.map(index => el.y[index]);
                                if( (yaxis_change && !xaxis_change) || (yaxis_in_trace[i] && xaxis_change) )
                                {
                                    y_rangefilteredIndices = filteredY.map((y, index) => y >= yrange[0] && y <= yrange[1] ? index : -1).filter(index => index !== -1);
                                    filteredY = y_rangefilteredIndices.map(index => filteredY[index]);
                                }

                                act_min = Math.min.apply(null, filteredY);
                                act_max = Math.max.apply(null, filteredY);

                                g_ymax = Math.max(g_ymax, act_max);
                                g_ymin = Math.min(g_ymin, act_min);
                            });

                            offset = 1/9*(g_ymax - g_ymin)/2;
                            if(offset == 0)
                            {
                                offset=0.001;
                            }

                            if(i == 0)
                            {
                                ylabel='yaxis.range';
                            }
                            else
                            {
                                ylabel='yaxis'+(1+i)+'.range';
                            }

                            update[ylabel] = [g_ymin-offset,g_ymax+offset];
                        });
                        
                        Plotly.relayout(graphDiv, update);
                    }
                    else
                    {
                        on_event=true;
                    }
                    }

                graphDiv.on('plotly_relayout', test);

                prev_xrange = localStorage.getItem('xrange');
                if(prev_xrange != null)
                {
                    prev_xrange_flag=true;
                    var currentLayout = graphDiv.layout;

                    // Trigger the relayout event without changing the layout
                    Plotly.relayout(graphDiv, currentLayout);
                }
            }
        }
        setTimeout(autoscale_function, rec_time);
        '''

def get_reload_tab_script(reload_message = 'reload_plotly'):
    return '''
            function connectWebSocket() {
                const ws = new WebSocket('ws://localhost:8765');

                ws.onmessage = function(event) {
                    console.log(event.data);
                    if (event.data === "''' + reload_message + '''") {
                        location.reload();
                    }
                };

                ws.onclose = function() {
                    console.log('WebSocket disconnected, retrying in 5 seconds...');
                    setTimeout(connectWebSocket, 5000);
                };
            }

            connectWebSocket();
        '''

# returns the script for a custom download button in plotly
def get_custom_download_button_script():
    return '''
    function downloadData(data, filename, type) {
        let content;

        if (type === 'text/csv') {
            // Create a header row with the names of the traces
            const namesRow = ['t (s)'].concat(data.map(trace => trace.name)).join(',') + '\\n';

            // Create an array to hold the rows of data
            const dataRows = [];

            // Iterate over the x values (time values)
            const maxLength = Math.max(...data.map(trace => trace.x.length)); // Get the maximum length of x arrays
            
            for (let i = 0; i < maxLength; i++) {
                // Start with the x value for this row
                const row = [];

                // Get the x value (time)
                const xValue = data[0].x[i] !== undefined ? data[0].x[i] : ''; // Only take from the first trace

                row.push(xValue); // Add time value

                // Iterate over each trace to get y values (with gap if x is not from the same trace)
                data.forEach(trace => {
                    const yValue = trace.y[i] !== undefined ? trace.y[i] : ''; // Handle undefined y value
                    row.push(yValue);
                });

                // Join the row and push it to dataRows
                dataRows.push(row.join(','));
            }

            // Combine namesRow and all dataRows to create content
            content = namesRow + dataRows.join('\\n');
        } else {
            content = JSON.stringify(data);
        }

        var blob = new Blob([content], { type: type });
        var url = window.URL.createObjectURL(blob);
        var a = document.createElement('a');
        a.href = url;
        a.download = filename; // Ensure filename ends with .csv
        a.click();
        window.URL.revokeObjectURL(url);
    }

    const downloadBtn = document.createElement('div');
    downloadBtn.className = 'modebar-group';
    downloadBtn.innerHTML = `
        <a rel="tooltip" class="modebar-btn" data-title="Download plot as CSV" data-toggle="false" data-gravity="n">
            <svg viewBox="0 0 24 24" class="icon" height="1em" width="1em">
                <path d="M12 15l4-4h-3V3h-2v8H8l4 4zm-8 3h16v2H4z"/>
            </svg>
        </a>
    `;
    downloadBtn.onclick = () => {
        graphDiv = document.querySelector('.plotly-graph-div');
        var plotData = graphDiv.data;
        var exportData = plotData.map(trace => ({
            x: trace.x,
            y: trace.y,
            name: trace.name
        }));
        const currentDate = new Date();
        const year = currentDate.getFullYear();
        const month = (currentDate.getMonth() + 1).toString().padStart(2, '0');
        const day = currentDate.getDate().toString().padStart(2, '0');
        const hours = currentDate.getHours().toString().padStart(2, '0');
        const minutes = currentDate.getMinutes().toString().padStart(2, '0');
        const seconds = currentDate.getSeconds().toString().padStart(2, '0');

        const filename = 'robot_plots_' + year + month + day + '_' + hours + minutes + seconds + '.csv';
        downloadData(exportData, filename, 'text/csv');
    };

    var rec_time_btn = 100; //ms
    function add_custom_button() {
        var graphDiv = document.querySelector('.plotly-graph-div');
        if(graphDiv == null || graphDiv.on == undefined)
        {
            setTimeout(add_custom_button, rec_time_btn);
        }
        else
        {
            // Append the button to the modebar container
            const modebarContainer = document.querySelector('.modebar-container div');
            modebarContainer.insertAdjacentElement('afterbegin', downloadBtn);
        }
    }
    setTimeout(add_custom_button, rec_time_btn);
    '''

def plot_solution_7dof(subplot_data, save_plot=False, file_name='plot_saved', plot_fig=True, matlab_import=True, reload_page=False, title_text=''):
    subplot_number = len(subplot_data)
    sig_labels = np.empty(24, dtype=object)

    if matlab_import:
        for i in range(0, subplot_number):
            sig_label     = subplot_data[i][0][0][0][0]
            if len(subplot_data[i][0]) > 1:
                sig_labels[i] = sig_label[:-2]
            else:
                sig_labels[i] = sig_label
            #row=1+np.mod(i,4), col=1+int(np.floor(i/4)
    else:
        for i in range(0, subplot_number):
            sig_labels[i] = subplot_data[i]['title']

    sig_labels_orig = sig_labels
    sig_labels = sig_labels.reshape(6,4).T.flatten().tolist()
    
    # Create a Plotly subplot
    fig = make_subplots(rows=4, cols=int(subplot_number/4), shared_xaxes=False, vertical_spacing=0.05, horizontal_spacing=0.035, subplot_titles=sig_labels)
    
    # Plot tdata
    for i in range(0, subplot_number):
        sig_title = sig_labels_orig[i]
        signal_number = len(subplot_data[i]['sig_ydata'])
        for j in range(0, signal_number):
            if matlab_import:
                sig_label     = subplot_data[i][0][j][0][0]
                sig_xdata     = subplot_data[i][0][j][1][0]
                sig_ydata     = subplot_data[i][0][j][2][0]
                sig_linestyle = subplot_data[i][0][j][3][0]
                sig_color     = 255*subplot_data[i][0][j][4][0]
                sig_color     = f"rgb({','.join(map(str, sig_color))})"
            else:
                sig_label = subplot_data[i]['sig_labels'][j]
                sig_xdata = subplot_data[i]['sig_xdata']
                sig_ydata = subplot_data[i]['sig_ydata'][j]
                sig_linestyle = subplot_data[i]['sig_linestyles'][j]
                sig_color = subplot_data[i]['sig_colors'][j]

            if sig_linestyle == '-':
                line_style = dict(width=1, color=sig_color, dash='solid')
            elif sig_linestyle == '--':
                line_style = dict(width=1, color=sig_color, dash='dash')

            act_row = 1+np.mod(i,4)
            act_col = 1+int(np.floor(i/4))
            fig.add_trace(go.Scatter(x=sig_xdata, y=sig_ydata, name=sig_label, line = line_style, hoverinfo = 'x+y+text', hovertext=sig_label, text=sig_title), row=act_row, col=act_col)
            if act_row == 4:
                fig.update_xaxes(title_text='t (s)', row=act_row, col=act_col)

    # fig.update_layout(plot_bgcolor='#1e1e1e', paper_bgcolor='#1e1e1e', font=dict(color='#ffffff'), legend=dict(orientation='h'))
    fig.update_layout(
        plot_bgcolor='#101010',  # Set plot background color
        paper_bgcolor='#1e1e1e',  # Set paper background color
        font=dict(color='#ffffff'),  # Set font color
        legend=dict(orientation='h', yanchor='middle', y=10, yref='container'),  # Set legend orientation
        hovermode = 'closest',
        margin=dict(l=10, r=10, t=50, b=70),
        height=1080,
        title=title_text,
        title_x=0,
        title_y=1,
        # legend_indentation = 0,
        # margin_pad=0,
        # Gridline customization for all subplots
        **{f'xaxis{i}': dict(gridwidth=1, gridcolor='#757575', linecolor='#757575', zerolinecolor='#757575', zerolinewidth=1) for i in range(1, subplot_number+1)},
        **{f'yaxis{i}': dict(gridwidth=1, gridcolor='#757575', linecolor='#757575', zerolinecolor='#757575', zerolinewidth=1) for i in range(1, subplot_number+1)}
    )

    fig.update_layout(
        **{f'xaxis{i}': dict(showticklabels=False) for i in range(1, subplot_number+1-6)}
    )

    fig.update_xaxes(matches='x', autorange=True)

    if(plot_fig):
        fig.show()

    if(save_plot):
        autoscale_code = get_autoscale_script()

        custom_download_button_code = get_custom_download_button_script()

        reload_tab_code = get_reload_tab_script('reload_plotly')

        # py.plot(fig, filename=file_name, include_mathjax='cdn', auto_open=False, include_plotlyjs='cdn') # online use
        py.plot(fig, filename=file_name, include_mathjax='cdn', auto_open=False) # offline use
        with open(file_name, 'r', encoding='utf-8') as file:
            html_content = file.read()
            soup = BeautifulSoup(html_content, 'html.parser')
            first_script_tag = soup.find('script')
            if first_script_tag:
                new_script = soup.new_tag('script')
                new_script.string = "document.body.style.background='#1e1e1e';"+autoscale_code+custom_download_button_code+reload_tab_code
                first_script_tag.insert_after(new_script)
                with open(file_name, 'w', encoding='utf-8') as file:
                    file.write(str(soup))
        if(reload_page):
            started = start_server('reload_plotly')
            if started:
                webbrowser.open('file://' + file_name)
        else: # otherwise open in browser
            webbrowser.open('file://' + file_name)

def plot_solution(us, xs, t, TCP_frame_id, robot_model, traj_data, save_plot=False, file_name='plot_saved', plot_fig=True):
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

    y_d    = traj_data['p_d'].T
    y_d_p  = traj_data['p_d_p'].T
    y_d_pp = traj_data['p_d_pp'].T

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

        J_v   = J[  0:3, :] # mir translatorischen anteil
        J_v_p = J_p[0:3, :] # z ist unnötig

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

def plot_mpc_solution(ddp, i, dt, N_horizon, N_step, TCP_frame_id, robot_model, traj_data):
    xs = np.array(ddp.xs)
    us = np.array(ddp.us)
    t = np.arange(i*dt, (i+N_horizon)*dt, dt)
    param_trajectory_copy = {
        'p_d':    traj_data['p_d'][:,    i:i+N_step*N_horizon:N_step],
        'p_d_p':  traj_data['p_d_p'][:,  i:i+N_step*N_horizon:N_step],
        'p_d_pp': traj_data['p_d_pp'][:, i:i+N_step*N_horizon:N_step]
    }
    plot_solution(us, xs, t, TCP_frame_id, robot_model, param_trajectory_copy)
    
def plot_current_solution(us, xs, i, t, TCP_frame_id, robot_model, traj_data):
    param_trajectory_copy = {
        'p_d':    traj_data['p_d'][:,    0:i],
        'p_d_p':  traj_data['p_d_p'][:,  0:i],
        'p_d_pp': traj_data['p_d_pp'][:, 0:i]
    }
    plot_solution(us[0:i], xs[0:i], t[0:i], TCP_frame_id, robot_model, param_trajectory_copy)


# https://github.com/meshcat-dev/meshcat-python/blob/master/src/meshcat/geometry.py
def create_coordinate_system(vis, name, axis_length=0.1, line_width=1):
    kos = g.LineSegments(
        g.PointsGeometry(position=np.array([
            [0, 0, 0], [axis_length, 0, 0],
            [0, 0, 0], [0, axis_length, 0],
            [0, 0, 0], [0, 0, axis_length]]).astype(np.float32).T,
            color=np.array([
            [1, 0, 0], [1, 0.6, 0],
            [0, 1, 0], [0.6, 1, 0],
            [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
        ),
        g.LineBasicMaterial(vertexColors=True, linewidth=line_width))
    vis[name].set_object(kos)
    
def create_homogeneous_transform(translation, rotation):
    transform = np.eye(4)  # Create a 4x4 identity matrix
    transform[:3, :3] = rotation  # Set the top-left 3x3 submatrix to the rotation matrix
    transform[:3, 3] = translation  # Set the first three elements of the last column to the translation vector
    return transform
####################################################### VIS ROBOT #################################################

def visualize_robot(robot_model, robot_data, visual_model, TCP_frame_id, q_sol, traj_data, dt,
                    frame_skip = 1, create_html = False, html_name = 'robot_visualization.html',
                    style_settings = {'N_traj_KOS': 10, 'traj_KOS_len': 0.05, 'traj_KOS_linewidth': 1,
                                      'TCP_KOS_len': 0.1, 'TCP_KOS_linewidth': 2, 
                                      'traj_color': [255, 165, 0], 'traj_linewidth': 1, 'traj_linestyle': 'solid'}):
    # Meshcat Visualize
    robot_display = MeshcatVisualizer(robot_model)
    robot_display.initViewer(open=False)

    anim = Animation()
    vis = robot_display.viewer

    # Display trajectory as line:
    y_d_data = traj_data['p_d'].T
    R_d_data = traj_data['R_d']
    vertices = np.hstack([y_d_data.T[:, 0][:, np.newaxis], np.repeat(y_d_data.T[:,1::], 2, 1)]).astype(np.float32)
    
    traj_color = style_settings['traj_color']
    traj_linewidth = style_settings['traj_linewidth']
    traj_linestyle = style_settings['traj_linestyle']

    vis["line_segments"].set_object(
        g.LineSegments(
            g.PointsGeometry(vertices),
            g.LineBasicMaterial(color=traj_color, linewidth=traj_linewidth, dashed=traj_linestyle == 'dashed')
        )
    )
    
    # Add coordinate systems along the trajectory
    N_traj_KOS = style_settings['N_traj_KOS']
    traj_KOS_len = style_settings['traj_KOS_len']

    # indices = traj_data['N_init'] + np.linspace(0, traj_data['N_traj'], N_traj_KOS, dtype=int)
    indices = np.linspace(0, traj_data['N_traj']-1, N_traj_KOS, dtype=int)
    for i, index in enumerate(indices):
        create_coordinate_system(vis, f"y_coord_system_{i}", traj_KOS_len)
        H = create_homogeneous_transform(y_d_data[index], R_d_data[:,:,index])
        vis[f"y_coord_system_{i}"].set_transform(H)

    # Display KOS at TCP
    create_coordinate_system(vis, "TCP_KOS")

    mesh_directory = 'stl_files/Meshes_FR3/'
    link_to_meshes = {
        'fr3_link0': ['link0_white.stl', 'link0_black.stl', 'link0_off_white.stl'],
        'fr3_link1': ['link1_white.stl'],
        'fr3_link2': ['link2_white.stl'],
        'fr3_link3': ['link3_white.stl', 'link3_black.stl'],
        'fr3_link4': ['link4_white.stl', 'link4_black.stl'],
        'fr3_link5': ['link5_white.stl', 'link5_black.stl'],
        'fr3_link6': ['link6_white.stl', 'link6_black.stl', 'link6_green.stl', 'link6_light_blue.stl', 'link6_off_white.stl'],
        'fr3_link7': ['link7_white.stl', 'link7_black.stl'],
    }

    # Define RGBA colors for specific names
    color_map = {
        'white': [1.0, 1.0, 1.0, 1.0],
        'scaled_white': [1.0, 1.0, 1.0, 1.0],
        'off_white': [0.901961, 0.921569, 0.929412, 1.0],
        'black': [0.25, 0.25, 0.25, 1.0],
        'green': [0.0, 1.0, 0.0, 1.0],
        'light_blue': [0.039216, 0.541176, 0.780392, 1.0]
    }

    #    Store geometry objects for all links
    geometry_objects = []
    for obj in visual_model.geometryObjects:
        # Load mesh geometry
        geometry_objects.append({
            'name': obj.name[:-2],
            'frame_id': obj.parentFrame,
            'homogeneous': obj.placement.homogeneous,
            'visible': False,
        })

    for obj in geometry_objects:
        for stl_file in link_to_meshes[obj['name']]:
            
            # mesh = g.StlMeshGeometry.from_file('/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/stl_files/Meshes_planar_manipulator/planar_manipulator-bulk1_fin.stl')
            mesh_path = os.path.join(mesh_directory, stl_file)
            mesh = g.StlMeshGeometry.from_file(mesh_path)

            link_name = stl_file[:-4] # without .stl
            color_name = link_name.split('_', 1)[-1]
            vis[link_name].set_object(mesh)
            # vis[obj['name']].set_property('scale', obj['scale'])
            vis[link_name].set_property('color', color_map[color_name])
            vis[link_name].set_property('visible', False)

    transform_matrix = np.array([
        [0, 1, 0, 0],  # X-axis points down-right
        [0, 0, 1, 0],  # Y-axis points up
        [-1, 0, 0, 0], # Z-axis points forward (upward)
        [0, 0, 0, 1]   # Homogeneous coordinate
    ])

    camera_position = np.array([0.8, 0.8, 1.3])  # Example position
    camera_target = np.array([1.0, 1.0, 1.0])    # Looking at this point

    # Set the camera position and target
    vis.set_cam_pos(camera_position)
    vis.set_cam_target(camera_target)

    # Iterate through trajectory data
    for i in range(0, len(q_sol), frame_skip):
        pinocchio.forwardKinematics(robot_model, robot_data, q_sol[i])
        pinocchio.updateFramePlacements(robot_model, robot_data)

        with anim.at_frame(vis, i) as frame:
            H_0_E = robot_data.oMf[TCP_frame_id].homogeneous
            frame["TCP_KOS"].set_transform(H_0_E)
            frame["TCP_KOS"].get_clip().fps = 1 / dt
            for obj in geometry_objects:
                frame_id = obj["frame_id"]
                H_0_s = robot_data.oMf[frame_id].homogeneous
                H_s_obj = obj['homogeneous']
                for stl_file in link_to_meshes[obj['name']]:
                    link_name = stl_file[:-4] # without .stl
                    frame[link_name].set_transform(H_0_s @ H_s_obj)
                    frame[link_name].get_clip().fps = 1 / dt

    # Set the animation to the Meshcat viewer
    vis.set_animation(anim)
    for obj in geometry_objects:
        for stl_file in link_to_meshes[obj['name']]:
            link_name = stl_file[:-4] # without .stl
            vis[link_name].set_property('visible', True)

    vis.set_cam_pos(camera_position)
    vis.set_cam_target(camera_target)

    if create_html:
        html_out = vis.static_html()
        soup = BeautifulSoup(html_out, 'html.parser')
        script_tag = soup.new_tag('script', type='text/javascript')
        script_tag.string = get_reload_tab_script('reload_meshcat')

        # Add the script tag to the body of the HTML
        soup.body.append(script_tag)
        with open(html_name, 'w') as file:
            file.write(str(soup))
            url = os.path.realpath(file.name)
        # webbrowser.open(url)
        time.sleep(1) # othervise visualization don't work
    else:
        robot_display.viewer.open()

    create_video=False
    if create_video:
        robot_display = crocoddyl.MeshcatDisplay(robot, -1, 1, False, visibility=False)
        with robot_display.robot.viz.create_video_ctx("test.mp4"):
            robot_display.robot.viz.play(q_sol, dt)

    start_server('reload_meshcat') # start server to visualize robot












def visualize_robot_2DOF(robot, q_sol, traj_data, dt, rep_cnt = np.inf, rep_delay_sec=1):
    # Meshcat Visualize
    robot_model = robot.model
    robot_display = MeshcatVisualizer(robot.model)
    robot_display.initViewer(open=False)

    y_d_data = traj_data['p_d'].T

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

    download_html=False
    render_standalone=True
    if render_standalone or download_html:
        html_out = vis.static_html()
        with open('meshcat.html', 'w') as file:
            file.write(html_out)
            url = os.path.realpath(file.name)
        if render_standalone:
            webbrowser.open(url)
            time.sleep(1) # othervise visualization don't work
        #if not download_html:
            # remove html file
            #os.remove('meshcat.html')
    else:
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
    
    create_video=False
    if create_video:
        robot_display = crocoddyl.MeshcatDisplay(robot, -1, 1, False, visibility=False)
        with robot_display.robot.viz.create_video_ctx("test.mp4"):
            robot_display.robot.viz.play(q_sol, dt)