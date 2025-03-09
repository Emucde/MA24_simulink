%cd /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/
%cd ..

filepath = fileparts(mfilename('fullpath'));
if(~strcmp([pwd, '/main_matlab'], filepath))
    cd([filepath, '/..']);
end

%% INIT
if(exist('parameter_str', 'var') && strcmp(parameter_str, "parameters_2dof"))
    rmpath('./utils/matlab_init_2dof');
    rmpath('./maple/maple_generated/2_dof_system');
end

if(  ~( exist('mpc_casadi_main_state', 'var') && strcmp(mpc_casadi_main_state, "running") )  )
    %close all
    clc;
end


if(~exist('dont_clear', 'var'))
    clear;
end

if(~exist('reset_started_flag', 'var'))
    reset_started_flag = false; % only useful if dont_clear is set
end

if(  ~exist('overwrite_offline_traj_forced_extern', 'var') )
    overwrite_offline_traj_forced_extern = false;  % only useful if dont_clear is set
end

fprintf('Start Execution of ''parameters_7dof.m''\n\n');

%start_in_singularity = false;
%trajectory_out_of_workspace = false; % TODO: einfach offset 0 setzten
%x_traj_out_of_workspace_value = 0.1;

full_reset_flag = false; % Please set this flag true after cloning or if a full reset should be  done.
overwrite_offline_traj_forced = ~false; % if true then init guess is also created
warm_start = true;
overwrite_init_guess = false; % but automatically true when overwrite_offline_traj_forced is true

% set_param(gcs,'Profile','off'); % turn off profiler when not needed anymore

%% Global Parameters
T_sim = 10; % = param_vis.T (see init_visual.m)
param_global.T_sim = T_sim;
param_global.Ta = 1e-3;
param_global.traj_buffer_size = 1000;

parameter_str = "parameters_7dof";
simulink_main_model_name = 'sim_discrete_7dof';

% Add paths. The Idea is that for each robot there exists different paths
% that should be added or removed. So it is possible to use only one main
% simulink file for all different robots.
run('./utils/matlab_init_general/add_default_matlab_paths'); % because it is not on path per default

get_robot_name; % set robot_name to active robot from simulink (or default value when simulink is closed)
s_fun_path = ['./s_functions/', robot_name];

add_robot_matlab_paths;

% Init Casadi
init_casadi;
import casadi.*;

full_reset;

% open file defined in simulink_main_model_name
open_simulink_slx;

% %%%%%%%%%%%%%%%% SOLVE MPC BUG %%%%%%%%%%%%%%%%
% do not comment in, this would result in an endless loop!
% only copy this command into the command window if mpcs are not found!
% solve_mpc_notfound_bug(simulink_main_model_name, 'both');
% solve_mpc_notfound_bug(simulink_main_model_name, 'reload');
% solve_mpc_notfound_bug(simulink_main_model_name, 'comment');
% solve_mpc_notfound_bug(simulink_main_model_name, 'uncomment');

%% Init scripts

% This robots initializes the different robot parameters
param_robot_init;

% combo boxes are trajectory dependent changed for each robot!
% it has to be ensured that sim_discrete_7dof is open!
special_comment_mode = false; % all opti mpc are commented in
comment_in_out_mpc_blocks;

activate_simulink_logs;

bus_definitions;

init_MPC_weights; %% set MPC weights

create_trajectories;
 figure(2);
 plot(param_traj_data.p_d(:,:,4)')
%create_mpc_init_guess;

change_simulink_traj_combo_box; % saves system!

create_trajectory_frame_data;
create_traj_data_bus;

param_ct_pdplus_control_init;
param_EKF_init;

param_debug;

%set_param('sim_discrete_7dof/Visualization/Spline', 'MarkerSize', '1');
%%
% bspline_arr = acin_text_spline_test([0.3 0.7 0.5], 1/4, 1/2);
% N_dat = 1000;
% sigma = zeros(N_dat, 3);
% i=1;
% for theta=0:1/N_dat:1
    % CC = spline_fun(theta, 1, bspline_arr(4));
    % sigma(i, :) = CC(:,1)';
    % i=i+1;
% end
% 
% plot3(sigma(:,1), sigma(:,2), sigma(:,3), 'LineWidth', 1, 'Color', 'blue');

fprintf('\nExecution of ''parameters_7dof.m'' finished\n');
fprintf('--------------------------------------------------------------------\n');