import casadi.*;

%% GENERATE OFFLINE TRAJECTORY
param_MPC_traj_data_mat_file = [s_fun_path, '/trajectory_data/param_traj_data.mat'];
param_MPC_traj_data_bin_file = [s_fun_path, '/trajectory_data/param_traj_data.bin'];
param_MPC_x0_init_bin_file = [s_fun_path, '/trajectory_data/param_x0_init.bin'];
param_traj_file = [s_fun_path, '/trajectory_data/param_traj.mat'];
param_traj_cell_file = [s_fun_path, '/trajectory_data/param_traj_cell.mat'];
param_traj_settings = [s_fun_path, '/trajectory_data/param_traj_settings.mat'];
param_traj_settings_online_file = [s_fun_path, '/trajectory_data/param_traj_settings_online.mat'];
param_MPC_file_path = [s_fun_path, '/mpc_settings/'];
files = dir([param_MPC_file_path, '*.mat']);

% cellfun(@load, {files.name}); % if no files then the for loop doesn't run.
file_list = cellfun(@(file) [param_MPC_file_path, file], {files.name}, 'UniformOutput', false);
cellfun(@load, file_list);

traj_mode.equilibrium = 1;
traj_mode.differential_filter = 2;
traj_mode.differential_filter_jointspace = 3;
traj_mode.polynomial = 4;
traj_mode.polynomial_jointspace = 5;
traj_mode.sinus = 6;

overwrite_offline_traj = false; 

%%%%%%%%%%%%%%%%%%%% CREATE TRAJECTORY %%%%%%%%%%%%%%%%%%%%%
if(overwrite_offline_traj || overwrite_offline_traj_forced || overwrite_offline_traj_forced_extern)
    if(~reset_started_flag)
        overwrite_init_guess = true;
    else
        overwrite_init_guess = false;
    end
    create_param_traj; % generate param_traj struct
    % Idee: Es müssen nur die Trajektorien für die MPC des längsten Prädiktionshorizonts berechnet werden, da alle anderen
    % kürzeren Prädiktionshorizonte in den längeren enthalten sind.

    % 1. get maximum horizont length of all mpcs:
    T_horizon_struct = struct;
    T_horizon_max = 0;
    for name={files.name}
        name_mat_file    = name{1};
        param_MPC_name   = name_mat_file(1:end-4);
        param_MPC_struct = eval(param_MPC_name);
        MPC_name = param_MPC_name(7:end);

        T_horizon = param_MPC_struct.T_horizon;
        T_horizon_struct.(MPC_name) = T_horizon; % save for p

        if(T_horizon > T_horizon_max)
            T_horizon_max = T_horizon;
        end
    end

    T_horizon_max = 2;%1s default

    % create param_traj_settings for simulink (online)
    % TODO: ist doch komplizierter als gedacht!!!!
    param_traj_settings_online.traj_mode = traj_mode;

    % 2. calculate all trajectories for max horizon length
    t = 0 : param_global.Ta : T_sim + T_horizon_max;

    traj_settings.N_data = ceil( 1 + (T_sim + T_horizon_max) / param_global.Ta );
    traj_settings.N_data_real = ceil( T_sim / param_global.Ta );
    traj_settings.N_traj = param_traj.N_traj;
    traj_settings.t = t;
    traj_settings.x_d = init_bus_param.x_d;
    traj_settings.traj_mode = traj_mode;

    % then only create init guess wheN: mpc want it or it is forced by parameters_xdof.m
    param_traj_data = generate_trajectory(param_traj, init_bus_param, traj_settings);
    save(param_MPC_traj_data_mat_file, 'param_traj_data'); % save struct
    save(param_traj_file, 'param_traj'); % save struct
    save(param_traj_cell_file, 'param_traj_cell'); % save struct
    save(param_traj_settings, 'traj_settings'); % save struct
    save(param_traj_settings_online_file, 'param_traj_settings_online')
else
    load(param_MPC_traj_data_mat_file);
    load(param_traj_file);
    load(param_traj_cell_file);
    load(param_traj_settings);
    load(param_traj_settings_online_file);
end

save_trajectories_as_binary(param_traj_data, param_traj, param_MPC_traj_data_bin_file, param_MPC_x0_init_bin_file);