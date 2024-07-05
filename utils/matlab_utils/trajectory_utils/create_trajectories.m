import casadi.*;

%%%%%%%%%%%%%%%%%%%% CREATE TRAJECTORY %%%%%%%%%%%%%%%%%%%%%
%% Trajectory selection
traj_select.equilibrium_traj = 1;
traj_select.differential_filter = 2;
traj_select.polynomial = 3;
traj_select.sinus = 4;
traj_select.traj_amount = param_global.traj_amount; % anzahl der trajektorien
%%%%%%%%%
traj_select_fin = 4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRAJECTORY 

param_traj = struct;
%% Calculate target positions

Q_pos = 1e0 * eye(m);  % Weight for the position error in the cost function.
Q_m = 0.5;             % Weight for the manipulability error in the cost function.
Q_q = 1e5 * eye(n);    % Weight for the deviaton of q_sol to q_d
Q_nl = 1e-1 * eye(n);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
q_d = param_robot.q_0_ref(n_indices);

xe0 = [0.3921; 0.3921; 0.5211; 0; 1/sqrt(2); 1/sqrt(2); 0];
xeT = [xe0(1:3,1) + [0; 0; -0.5]; rotm2quat_v4( Rz(pi/4)*quat2rotm_v2(xe0(4:7)) )]; % rotm2quat (from matlab) is very precise but slow

q_0 = fsolve(@(q) kin_fun(xe0, q), q_d); % test if the function works
%[q_0, ~] = inverse_kinematics(param_robot, xe0, q_d, Q_pos, Q_m, Q_q, Q_nl,  1e-2, 100, ct_ctrl_param);
H_0 = hom_transform_endeffector_py(q_0);
xe0 = [H_0(1:3,4); rotm2quat_v4(H_0(1:3,1:3))]; % better to exact start in point
q_0_p = zeros(n, 1);
q_0_pp = zeros(n, 1);

%tests;
R_init = quat2rotm_v2(xe0(4:7));
R_target = quat2rotm_v2(xeT(4:7));

traj_cell = cell(1, 4);
% Trajectory 1: Ruhelage
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.pose = [xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_select.equilibrium_traj];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global); % default settings not used [TODO: Mit extra index arbeiten]tructs ge
% man prüft beim combine ob diese structs existieren. dann führt man extra indices ein die auf ein struct verweisen das als array
% die parameter speichert. wenn es nicht exisitert, ist der index -1, exisitiert es wird der index abhängig von der anzahl der
% structs gesetzt, d. h. er kann sich von trajectory_select unterscheiden, lässt sich aber mit trajectory select ermitteln.
% aber zuerst muss ich mal prüfen ob in structs arrays von structs stehen dürfen, ich glaube eben nicht.
traj_struct = create_param_sin_poly(traj_struct, param_global); % default settings not used
traj_cell{1} = traj_struct;

% Trajectory 2: Differential filter 5th order
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.pose = [xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_select.differential_filter];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -10); % Param differential filter 5th order trajectory
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_cell{2} = traj_struct;

% Trajectory 3: Polynomial 5th order
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.pose = [xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_select.polynomial];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global); % Param differential filter 5th order trajectory
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_cell{3} = traj_struct;

% Trajectory 4:
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.pose = [xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_select.sinus];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global); % Param differential filter 5th order trajectory
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_cell{4} = traj_struct;

traj_struct_combined = combine_trajectories(traj_cell, param_global, param_robot);
param_traj.traj_init = traj_struct_combined;

% kürzeste rotationachse und winkel zwischen R_init und R_target berechnen.
[rot_ax, rot_alpha_scale] = find_rotation_axis(R_init, R_target);

%% GENERATE OFFLINE TRAJECTORY
param_MPC_traj_data_mat_file = [s_fun_path, '/trajectory_data/param_traj_data.mat'];
param_MPC_file_path = [s_fun_path, '/mpc_settings/'];
files = dir([param_MPC_file_path, '*.mat']);
cellfun(@load, {files.name}); % if no files then the for loop doesn't run.

overwrite_offline_traj = false;

try

    if(overwrite_offline_traj || overwrite_offline_traj_forced)
        % Idee: Es müssen nur die Trajektorien für die MPC des längsten Prädiktionshorizonts berechnet werden, da alle anderen
        % kürzeren Prädiktionshorizonte in den längeren enthalten sind.

        % 1. get maximum horizont length of all mpcs:
        T_horizon_max = 0;
        for name={files.name}
            name_mat_file    = name{1};
            param_MPC_name   = name_mat_file(1:end-4);
            param_MPC_struct = eval(param_MPC_name);

            T_horizon = param_MPC_struct.T_horizon;

            if(T_horizon > T_horizon_max)
                T_horizon_max = T_horizon;
            end
        end
        
        % 2. calculate all trajectories for max horizon length
        t = 0 : param_global.Ta : T_sim + T_horizon_max;

        traj_settings.N_data = ceil( 1 + (T_sim + T_horizon_max) / param_global.Ta );
        traj_settings.traj_amount = param_global.traj_amount;
        traj_settings.t = t;
        traj_settings.x_d = init_bus_param.x_d;

        param_traj_data = param_traj_data_fun(traj_settings, 'init');

        % then only create init guess wheN: mpc want it or it is forced by parameters_xdof.m
        if(overwrite_offline_traj || overwrite_offline_traj_forced)

            for i=1:traj_select.traj_amount
                tic;
                % TODO: Create function for that!!!!!
                new_traj_data = generate_trajectory(t, i, param_traj, init_bus_param);
                disp(['parameter.m: Execution Time for Trajectory Calculation: ', sprintf('%f', toc), 's']);
            
                param_traj_data = param_traj_data_fun(traj_settings, 'set', i, param_traj_data, new_traj_data);
            end

            save(param_MPC_traj_data_mat_file, 'param_traj_data'); % save struct
        else
            load(param_MPC_traj_data_mat_file);
        end
        % 3. calculate initial guess for alle trajectories and mpcs
        
        compile_sfun                    = false;
        weights_and_limits_as_parameter = true;
        plot_null_simu                  = false;
        print_init_guess_cost_functions = false;

        if(~overwrite_offline_traj_forced)
            if(~strcmp(overwrite_init_guess_name, ''))
                files = struct;
                files.name = overwrite_init_guess_name;
            end
        end

        tic
        for name={files.name}
            name_mat_file    = name{1};
            param_MPC_name   = name_mat_file(1:end-4);
            param_MPC_struct = eval(param_MPC_name);
        
            param_MPC_init_guess_name = [param_MPC_name, '_init_guess'];
            param_MPC_init_guess_mat_file = [s_fun_path, '/initial_guess/', param_MPC_init_guess_name,'.mat'];
        
            casadi_func_name = param_MPC_struct.name;
            MPC_variant      = param_MPC_struct.variant;
            MPC_solver       = param_MPC_struct.solver;
            Ts_MPC           = param_MPC_struct.Ts     ;
            rk_iter          = param_MPC_struct.rk_iter;
            N_MPC            = param_MPC_struct.N  ;
            T_horizon_MPC    = param_MPC_struct.T_horizon;                   
            N_step_MPC       = param_MPC_struct.N_step;
            MPC_version      = param_MPC_struct.version;
            int_method       = param_MPC_struct.int_method;
            fixed_parameter  = param_MPC_struct.fixed_parameter;

            weights_and_limits_as_parameter = ~fixed_parameter;
            DT = Ts_MPC;
            M = rk_iter;

            output_dir = [s_fun_path,'/'];
            input_dir = [s_fun_path,'/casadi_functions/'];

            f_opt = Function.load([s_fun_path, '/', casadi_func_name, '.casadi']);

            param_weight_init = param_weight.(casadi_func_name);
            param_weight_init_cell = merge_cell_arrays(struct2cell(param_weight_init), 'vector');

            init_guess_cell = cell(1, traj_select.traj_amount);            
            for ii=1:traj_select.traj_amount
                param_trajectory = param_traj_data_fun(traj_settings, 'get', ii, param_traj_data);

                % Achtung, hier drin werden for loops mit der Variable i verwendet, daher darf die äußere
                % Schleife nicht die Variable i verwenden!!!!
                nlpsol_generate_opt_problem;
                if(weights_and_limits_as_parameter)
                    % TODO - why +1e-15 necessary??
                    [~, xx_full_opt_sol, ~] = f_opt(mpc_init_reference_values, init_guess_0+1e-15, param_weight_init_cell);
                else
                    [~, xx_full_opt_sol] = f_opt(mpc_init_reference_values, init_guess_0);
                end
                init_guess = full(xx_full_opt_sol);

                init_guess_cell{ii} = init_guess;
            end

            init_guess_arr = vertcat(init_guess_cell{:});
        
            param_MPC = struct('init_guess', init_guess_arr);
            
            eval([param_MPC_init_guess_name, ' = param_MPC;']);
            save(param_MPC_init_guess_mat_file, param_MPC_init_guess_name);
        end
        disp(['parameter.m: Execution Time for Init guess Calculation: ', sprintf('%f', toc), 's']);

        init_MPC_weights; % why necessary?
    else
        files = dir([s_fun_path, '/initial_guess/*.mat']);
        cellfun(@load, {files.name});

        load(param_MPC_traj_data_mat_file);
    end

catch ME
    disp('Cannot create init guess, please compile new')
    fprintf(2, 'Fehler: %s\n', getReport(ME));
end

if(plot_trajectory)
        %{
        param_trajectory_test = param_MPC1_qrqp_opti_traj_data;
        subplot(2,1,1);
        plot(param_trajectory_test.t, param_trajectory_test.p_d);
        title('polynomial 5th order')
        ylabel('p(t) (m)');
        xlabel('t (s)');
        subplot(2,1,2);
        plot(param_trajectory_test.t(1:end-1), diff(param_trajectory_test.p_d'));
        ylabel('d/dt p(t) (m/s)');
        xlabel('t (s)');
        %}

        figure;
        title('polynomial 5th order')

        subplot(3,2,1);
        plot(param_MPC1_traj_data.t, param_MPC1_traj_data.p_d(1,:));
        ylabel('p^d_x(t) (m)');
        xlabel('t (s)');
        xlim([0 10])
        grid on;
        %plot(param_MPC1_traj_data.t(1:end-1), (1/param_global.Ta)*diff(param_MPC1_traj_data.p_d(1,:)'));
        %ylabel('p_y(t) (m)');
        %xlabel('t (s)')

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

%% DEBUG


function out = kin_fun(xe, q)
    H = hom_transform_endeffector_py(q);
    
    RR = H(1:3,1:3)*quat2rotm_v2(xe(4:7))';
    p_err = H(1:3,4) - xe(1:3);
    r_err = [RR(3,2) - RR(2,3); RR(1,3) - RR(3,1); RR(2,1) - RR(1,2)];
    out = r_err'*r_err + p_err'*p_err;

    % He = [quat2rotm_v2(xe(4:7)), xe(1:3); 0 0 0 1];
    % out = sum((H - He).^2, 'all');
end