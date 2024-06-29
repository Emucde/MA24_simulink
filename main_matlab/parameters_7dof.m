cd /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/

%% INIT
if(exist('parameter_str', 'var') && strcmp(parameter_str, "parameters_2dof"))
    rmpath('./utils/matlab_init_2dof');
    rmpath('./maple/maple_generated/2_dof_system');
end

%if strcmp(mfilename, 'parameters')
    clear
    %close all
    clc
%end

%start_in_singularity = false;
%trajectory_out_of_workspace = false; % TODO: einfach offset 0 setzten
%x_traj_out_of_workspace_value = 0.1;

plot_trajectory = ~true;
overwrite_offline_traj_forced = false;


% set_param(gcs,'Profile','off'); % turn off profiler when not needed anymore

% measure compile time: 
% disp('Compile Time:')
% tic;set_param('sim_discrete_planar', 'SimulationCommand', 'update');
% toc

parameter_str = "parameters_7dof";

% valid robot_names: fr3_7dof, fr3_6dof, ur5e
robot_name = 'fr3_7dof';
% robot_name = 'fr3_6dof';
% robot_name = 'ur5e_6dof';
s_fun_path = ['./s_functions/', robot_name];
run('./utils/matlab_init_general/add_matlab_paths'); % because it is not on path per default

init_casadi;
import casadi.*;

simulink_main_model_name = 'sim_discrete_7dof';
open_simulink_on_start = true;
%if(~bdIsLoaded(simulink_main_model_name) && open_simulink_on_start && sum(strfind([getenv().keys{:}], 'VSCODE')) == 0)
% && ~isSimulinkStarted funktioniert einfach nicht.
if( ~bdIsLoaded(simulink_main_model_name) && open_simulink_on_start && desktop('-inuse') == 1) % in vscode inuse delivers 0 (running in background)
    tic
    fprintf(['open simulink model \"' simulink_main_model_name, '\" ...'])
    open_system([simulink_main_model_name '.slx'])
    fprintf([' finished! (Loading time: ' num2str(toc) ' s)\n']);
end

%closeAllSimulinkModels('./MPC_shared_subsystems')
%closeAllSimulinkModels('.')
close_system('./main_simulink/controller_ref_subsys', 0);

activate_simulink_logs;

%% Other init scripts
T_sim = 10; % = param_vis.T (see init_visual.m)
param_global.T_sim = T_sim;
param_global.Ta = 1e-3;
param_global.traj_amount = 4; % number of trajectories

param_robot_init;

bus_definitions;
init_MPC_weights; %% set MPC weights

param_ct_control_init;


%%%%%%%%%%%%%%%%%%%% CREATE TRAJECTORY %%%%%%%%%%%%%%%%%%%%%
%% Trajectory selection
traj_select.equilibrium_traj = 1;
traj_select.differential_filter = 2;
traj_select.polynomial = 3;
traj_select.sinus = 4;
traj_select.traj_amount = param_global.traj_amount; % anzahl der trajektorien
%%%%%%%%%
traj_select_fin = 4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Param Trajektory differential filter 5th order
param_diff_filter_traj;

%% Param Trajectory Poly
param_traj_poly.T = T_sim/2-3; % in s

%% Param sinus poly trajectory
param_traj_sin_poly.T     = 1; % in s
T_period                  = 2; % in s
param_traj_sin_poly.omega = 2*pi*1/(T_period); % in rad
param_traj_sin_poly.phi   = 0; % in rad

%% Calculate target positions

n_indices = param_robot.n_indices;
n_indices_fixed = setdiff(1:n, n_indices);
disp(['Fixed joint: ', num2str(n_indices_fixed'), ', nDOF = ', num2str(n), ', see param_robot_init.m']);

q_0_init = [0; 0; pi/4; -pi/2; 0*pi/4; pi/2; 0];
% q_0 = [0; -pi/4; 0; -3*pi/4; pi/4; pi/2; pi/4];
% q_0 = [-1.081, -1.035, 1.231, -1.778, 0.967, 1.394, -0.652]';
q_0_p_init = zeros(7, 1);
q_0_pp_init = zeros(7, 1);

% 6 DOF Case
q_0 = q_0_init(n_indices);
q_0_p = q_0_p_init(n_indices);
q_0_pp = q_0_pp_init(n_indices);

% necessary only for visualization when less degrees of freedom are used than the robot has
% (only needed for fr3 at this moment, see visualization.slx)
param_robobt.q_0_init = q_0_init;
param_robobt.q_0_p_init = q_0_p_init;
param_robobt.q_0_pp_init = q_0_pp_init;

H_0_init = hom_transform_endeffector_py(q_0);

quat_init = rotation2quaternion(H_0_init(1:3, 1:3));
R_init = quat2rotm_v2(quat_init); % besser numerisch robust
xe0 = [H_0_init(1:3,4); quat_init]; % xe0 = [x,y,z,q1,q2,q3,q4]

rotq_fun = @(x1, x2) [x1(1:3) + x2(1:3); quat_mult(x1(4:7), x2(4:7))]; % multiplikation von quaternions = rotationen hintereinander ausführen
%xeT = xe0;
%xeT = rotq_fun(xe0, [0; 0; -0.5; 1; 0; 0; 0]); % correct addition of quaternions
%xeT = [xe0(1:3,1); rotation2quaternion(Rz(pi/4)*R_init)]; % correct addition of quaternions
R_target = Rz(pi/4)*R_init; % Vormultiplikation: Drehung erfolgt in Bezug auf Inertialsystem (muss so sein!)
%R_target = Rz(0)*R_init; % Vormultiplikation: Drehung erfolgt in Bezug auf Inertialsystem
xeT = [xe0(1:3,1) + [0; 0; -0.5]; rotation2quaternion(R_target)]; % correct addition of quaternions

%tests;

%% Inverse Kin (Zum Prüfen ob Endwert im Aufgabenraum ist.)
calc_inverse_kin = ~true;
if(calc_inverse_kin)
    % Define the cost function.
    Q1 = 1e10 * eye(m); % Weight for the position error in the cost function.
    Q2 = 0.5; % Weight for the manipulability error in the cost function.
    Q3 = 1*1e0 * eye(n); % Weight for the deviaton of q_sol to q_d
    Q4 = 1e-1 * eye(n);% Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
    tolerance = 0.01;
    random_q0_count = 100;

    q_d = param_robot.q_n;
    q_d = q_0;

    xeT_in = [xeT(1:3); rotm2eul(R_target, 'XYZ')'];
    
    [q_sol, ~] = inverse_kinematics(param_robot, xeT_in, q_d, Q1, Q2, Q3, Q4, tolerance, random_q0_count, ct_ctrl_param);
    %%
end

% kürzeste Rotationsachse und winkel zwischen R_init und R_target berechnen.
[rot_ax, rot_alpha_scale] = find_rotation_axis(R_init, R_target);

% Achtung: Hier wird angenommen, dass um eine feste Achse rotiert wird.
param_init_pose.T_start = T_sim/2;

param_init_pose.xe0 = xe0;
param_init_pose.xeT = xeT;

param_init_pose.q_0    = q_0;       % nDOF q_0
param_init_pose.q_0_p  = q_0_p;   % nDOF q_0_p
param_init_pose.q_0_pp = q_0_pp; % nDOF q_0_pp

param_init_pose.R_init          = R_init;
param_init_pose.R_target        = R_target;
param_init_pose.rot_alpha_scale = rot_alpha_scale;
param_init_pose.rot_ax          = rot_ax;
param_init_pose.Phi_init        = rotm2rpy(R_init);
param_init_pose.Phi_target      = rotm2rpy(R_target);
param_init_pose.delta_Phi       = param_init_pose.Phi_target - param_init_pose.Phi_init;
param_init_pose.alpha0          = 0;
param_init_pose.alphaT          = 1;

%% GENERATE OFFLINE TRAJECTORY
param_MPC_traj_data_mat_file = [s_fun_path, '/trajectory_data/param_traj_data.mat'];
param_MPC_file_path = [s_fun_path, '/mpc_settings/'];
files = dir([param_MPC_file_path, '*.mat']);
cellfun(@load, {files.name}); % if no files then the for loop doesn't run.

T_traj_poly         = param_traj_poly.T;
T_traj_sin_poly     = param_traj_sin_poly.T;
omega_traj_sin_poly = param_traj_sin_poly.omega;
phi_traj_sin_poly   = param_traj_sin_poly.phi;
T_switch            = param_traj_filter.T_switch;

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
                new_traj_data = generate_trajectory(t, i, param_init_pose, param_traj_filter, param_traj_poly, param_traj_sin_poly, init_bus_param);
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
                    [~, xx_full_opt_sol, ~] = f_opt(mpc_init_reference_values, init_guess_0, param_weight_init_cell);
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

ew_test_CT_CTRL = [diag(-ct_ctrl_param.Kd1/2 + sqrt(ct_ctrl_param.Kd1^2/4 - ct_ctrl_param.Kp1)) diag(-ct_ctrl_param.Kd1/2 - sqrt(ct_ctrl_param.Kd1^2/4 - ct_ctrl_param.Kp1))]';
plot_eigenvalues_controller_text([ew_test_CT_CTRL ew_test_CT_CTRL*0], 'Eigenvalues CT Ctrl', '');