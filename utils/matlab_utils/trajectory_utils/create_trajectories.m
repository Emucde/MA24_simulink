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
cellfun(@load, {files.name}); % if no files then the for loop doesn't run.

traj_mode.equilibrium = 1;
traj_mode.differential_filter = 2;
traj_mode.differential_filter_jointspace = 3;
traj_mode.polynomial = 4;
traj_mode.polynomial_jointspace = 5;
traj_mode.sinus = 6;

overwrite_offline_traj = false; 

%%%%%%%%%%%%%%%%%%%% CREATE TRAJECTORY %%%%%%%%%%%%%%%%%%%%%
try
    if(overwrite_offline_traj || overwrite_offline_traj_forced)
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

        % create param_traj_settings for simulink (online)
        % TODO: ist doch komplizierter als gedacht!!!!
        if(bdIsLoaded(simulink_main_model_name))
            mpc_amount = length(mpc_list); % variable used from "comment_in_out_mpc_blocks.m"

            param_traj_settings_online = struct;
            param_traj_settings_online.control_select = [mpc_list.Value];
            
            param_traj_settings_online.T_horizon = zeros(1, mpc_amount);
            mpc_names = fieldnames(T_horizon_struct);
            for j = 1:mpc_amount
                for i = 1:length(mpc_names)
                    name = mpc_names{i};
                    if(contains(mpc_list(j).Label, name))
                        T_horizon_tmp = T_horizon_struct.(name);
                        break
                    else % then ct control was selected
                        T_horizon_tmp = param_global.Ta;
                    end
                end
                param_traj_settings_online.T_horizon(j) = T_horizon_tmp;
            end
        end
        param_traj_settings_online.traj_mode = traj_mode;

        % 2. calculate all trajectories for max horizon length
        t = 0 : param_global.Ta : T_sim + T_horizon_max;

        traj_settings.N_data = ceil( 1 + (T_sim + T_horizon_max) / param_global.Ta );
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

        % 3. calculate init guess for all mpcs
        create_mpc_init_guess;
    else
        files = dir([s_fun_path, '/initial_guess/*.mat']);
        cellfun(@load, {files.name});

        load(param_MPC_traj_data_mat_file);
        load(param_traj_file);
        load(param_traj_cell_file);
        load(param_traj_settings);
        load(param_traj_settings_online_file);
    end

catch ME
    disp('Cannot create init guess, please compile new')
    fprintf(2, 'Fehler: %s\n', getReport(ME));
end

save_trajectories_as_binary(param_traj_data, param_traj, param_MPC_traj_data_bin_file, param_MPC_x0_init_bin_file);

if(plot_trajectory)
        figure;
        title('polynomial 5th order')

        subplot(3,2,1);
        plot(param_MPC1_traj_data.t, param_MPC1_traj_data.p_d(1,:));
        ylabel('p^d_x(t) (m)');
        xlabel('t (s)');
        xlim([0 10])
        grid on;
        
        subplot(3,2,2);
        plot(param_MPC1_traj_data.t, param_MPC1_traj_data.p_d(2,:));
        ylabel('p^d_y(t) (m)');
        xlabel('t (s)');
        xlim([0 10])
        grid on;

        subplot(3,2,3);
        plot(param_MPC1_traj_data.t, param_MPC1_traj_data.p_d_p(1,:));
        ylabel('d/dt p^d_x(t) (m/s)');
        xlabel('t (s)');
        xlim([0 10])
        grid on;

        subplot(3,2,4);
        plot(param_MPC1_traj_data.t, param_MPC1_traj_data.p_d_p(2,:));
        ylabel('d/dt p^d_y(t) (m/s)');
        xlabel('t (s)');
        xlim([0 10])
        grid on;

        subplot(3,2,5);
        plot(param_MPC1_traj_data.t, param_MPC1_traj_data.p_d_pp(1,:));
        ylabel('d^2/dt^2 p^d_x(t) (m/s)');
        xlabel('t (s)');
        xlim([0 10])
        grid on;

        subplot(3,2,6);
        plot(param_MPC1_traj_data.t, param_MPC1_traj_data.p_d_pp(2,:));
        ylabel('d^2/dt^2 p^d_y(t) (m/s)');
        xlabel('t (s)');
        xlim([0 10])
        grid on;
        %%
        plot(param_MPC1_traj_data.t(1:end-2), (1/param_global.Ta^2)*diff(param_MPC1_traj_data.p_d(1,:)',2));
        ylabel('d/dt p(t) (m/s)');
        xlabel('t (s)');hold on;plot(param_MPC1_traj_data.t, param_MPC1_traj_data.p_d_pp(1,:));
        % [TODO]: plot all trajectories (p_d, p_d_p, p_d_pp, q_d, q_d_p, omega_d, omega_d_p)
end