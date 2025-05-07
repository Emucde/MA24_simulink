import os
import sys
import numpy as np
import crocoddyl as cro
import pinocchio as pin
import time
import csv
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import scipy.io as sio
import meshcat as mc
from multiprocessing import shared_memory
import posix_ipc
import multiprocessing
sys.path.append(os.path.dirname(os.path.abspath('./utils_python')))
from utils_python.utils import *

# robot_name = 'ur5e_6dof'
robot_name = 'fr3_6dof_no_hand'
fr3_kin_model = False

if robot_name == 'ur5e_6dof':
    n_dof = 6 # (input data from simulink are 7dof states q, qp)
    mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files/Meshes_ur5e')
    urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', 'ur5e.urdf')
    urdf_tcp_frame_name = 'ur5e_tcp'

    trajectory_data_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/ur5e_6dof/trajectory_data/param_traj_data.mat')
    trajectory_param_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/ur5e_6dof/trajectory_data/param_traj.mat')
    n_indices = np.array([0, 1, 2, 3, 4, 5]) # use all joints
    n_x_indices = np.hstack([n_indices, n_indices+6])

    q_0_ref = np.array([0, 0, 0, 0, 0, 0])
   
   # Create a list of joints to lock
    jointsToLock = []
elif robot_name == 'fr3_6dof_no_hand':
    n_dof = 7 # (input data from simulink are 7dof states q, qp)
    mesh_dir = os.path.join(os.path.dirname(__file__), '..', 'stl_files/Meshes_FR3')
    if fr3_kin_model:
        urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', 'fr3_no_hand_7dof_kinematic_model.urdf')
    else:
        urdf_model_path = os.path.join(os.path.dirname(__file__), '..', 'urdf_creation', 'fr3_no_hand_7dof.urdf')
    urdf_tcp_frame_name = 'fr3_link8_tcp'
    trajectory_data_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/fr3_no_hand_6dof/trajectory_data/param_traj_data.mat')
    trajectory_param_mat_file = os.path.join(os.path.dirname(__file__), '..', './s_functions/fr3_no_hand_6dof/trajectory_data/param_traj.mat')

    use_only_q2q4 = False
    if use_only_q2q4:
        n_indices = -1 + np.array([2, 4]) # list of used joints
    else:
        n_indices = -1 + np.array([1, 2, 4, 5, 6, 7])

    n_x_indices = np.hstack([n_indices, n_indices+7])

    q_0_ref = np.array([0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, np.pi/4])

    # Create a list of joints to lock (starts with joint 1)
    jointsToLockIndex = np.setdiff1d(np.arange(0,7), n_indices) # at first use 7dof Model (will be later reduced)
    jointsToLock = jointsToLock = [f"fr3_joint{i+1}" for i in jointsToLockIndex] # list of joints to lock
else:
    raise Exception("Unknown robot name '{}'".format(robot_name), "Available robots: 'ur5e_6dof', 'fr3_6dof_no_hand'")

n_indices_all = np.arange(0, n_dof)
n_indices_fixed = n_indices_all[~np.isin(n_indices_all, n_indices)]

robot_model_full, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

robot_data_full = robot_model_full.createData()

TCP_frame_id = robot_model_full.getFrameId(urdf_tcp_frame_name)

folderpath="./main_ros2/nodejs_ros2_gui/public"
visualize_name = 'robot_visualization.html'
visualize_file_path = os.path.abspath(os.path.join(folderpath, visualize_name))

# read trajetory data from csv file
# TRAJ 1:
traj_path1 = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj1/Crocoddyl_N_step_1_N_MPC_20_yr1e5_v3_best/robot_plots_20250503_154144.csv'

# TRAJ 2:
traj_path2 = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj2/Crocoddyl_N_step_1_N_MPC_20_yr1e5_q_x_2e0_v4_best/robot_plots_20250503_154642.csv'

# TRAJ 4:
traj_path3 = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj4/Crocoddyl_N_step_1_N_MPC_20_yr1e5_v5_best/robot_plots_20250505_123517.csv'

# TRAJ 5:
traj_path4 = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj5/Crocoddyl_N_step_1_N_MPC_20_yr1e3_v7_best/robot_plots_20250505_161447.csv'

traj_path = traj_path4

with open(traj_path, 'r') as f:
    reader = csv.reader(f)
    headers = next(reader)
    traj_data = np.genfromtxt(traj_path, delimiter=',', skip_header=1)
    traj_dict = {header: traj_data[:, i] for i, header in enumerate(headers)}

# Convert quaternions to rotation matrices
if(traj_path == traj_path4):
    quat_d_1 = -np.sqrt(np.abs(1 - traj_dict['quat_d_2']**2 - traj_dict['quat_d_3']**2 - traj_dict['quat_d_4']**2))
else:
    quat_d_1 = np.sqrt(np.abs(1 - traj_dict['quat_d_2']**2 - traj_dict['quat_d_3']**2 - traj_dict['quat_d_4']**2))
quaternions = np.vstack((traj_dict['quat_d_2'], traj_dict['quat_d_3'], traj_dict['quat_d_4'], quat_d_1))
rotation_matrices = np.zeros((3, 3, quaternions.shape[1]))
for i in range(quaternions.shape[1]):
    rotation_matrices[:, :, i] = Rotation.from_quat(quaternions[:, i]).as_matrix()

quaternions = np.vstack((quat_d_1, traj_dict['quat_d_2'], traj_dict['quat_d_3'], traj_dict['quat_d_4']))

transient_traj = {
    'p_d': np.vstack((traj_dict['p_d_x (m)'], traj_dict['p_d_y (m)'], traj_dict['p_d_z (m)'])),
    'p_d_p': np.vstack((traj_dict['ṗ_d_x (m/s)'], traj_dict['ṗ_d_y (m/s)'], traj_dict['ṗ_d_z (m/s)'])),
    'p_d_pp': np.vstack((traj_dict['p̈_d_x (m/s²)'], traj_dict['p̈_d_y (m/s²)'], traj_dict['p̈_d_z (m/s²)'])),
    'R_d': rotation_matrices,
    'q_d': quaternions,  # Use quaternions directly for q_d
    'omega_d': np.vstack((traj_dict['ω_d_x (rad/s)'], traj_dict['ω_d_y (rad/s)'], traj_dict['ω_d_z (rad/s)'])),
    'omega_d_p': np.vstack((traj_dict['ὠ_d_x (rad/s²)'], traj_dict['ὠ_d_y (rad/s²)'], traj_dict['ὠ_d_z (rad/s²)'])),
    'N_traj': traj_dict['t (s)'].shape[0],
    'N_init': 0,
    'N_total': traj_dict['t (s)'].shape[0],
    't': traj_dict['t (s)']
}

q_sol = np.vstack((traj_dict['q1'], traj_dict['q2'], traj_dict['q3'], traj_dict['q4'], traj_dict['q5'], traj_dict['q6'], traj_dict['q7'])).T
Ts = traj_dict['t (s)'][1] - traj_dict['t (s)'][0]

visualize_robot(robot_model_full, robot_data_full, visual_model, TCP_frame_id,
                q_sol, transient_traj, Ts,
                frame_skip=1, create_html = True, html_name = visualize_file_path)