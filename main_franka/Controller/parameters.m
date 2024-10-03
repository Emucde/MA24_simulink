addpath(genpath('../'));
addpath(genpath('../../utils/matlab_init_general/'));
addpath(genpath('../../utils/matlab_utils/'));

n=6;
T_sim = 10;

bus_definitions;
robot_name = 'fr3_no_hand_6dof';
param_robot_init;
param_ct_pdplus_control_init.m;

load('../../s_functions/fr3_no_hand_6dof/trajectory_data/param_traj_data.mat');
param_traj_data.N = length(param_traj_data.t);

% TODO: robot model bus mit C

robot_ip = '172.16.10.2';
% restoredefaultpath
% addpath(genpath('./s_functions'));
% init_franka_matlab('0.13.0')

q_init = [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]';
param_global.Ta = 1e-3;

load('mpc_init_ref_test.mat');
% load('mpc_init_guess_test.mat');
load('param_MPC8_init_guess.mat');
init_guess_0 = param_MPC8_init_guess.init_guess(2,:);
load('mpc_init_weight_test.mat');