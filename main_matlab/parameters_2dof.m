%% INIT
%if strcmp(mfilename, 'parameters')
    clear
    %close all
    clc
%end

% (turn off profiler when not needed anymore)
%set_param(gcs,'Profile','off');
% measure compile time: 
%disp('Compile Time:')

%tic;set_param('sim_discrete_planar', 'SimulationCommand', 'update');toc
%rmpath('/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions')

%closeAllSimulinkModels('./MPC_shared_subsystems')
%closeAllSimulinkModels('.')

%restoredefaultpath
cd /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/main_matlab
addpath(genpath('../main_matlab'));
addpath(genpath('../utils'));
addpath(genpath('../s_functions'));
addpath(genpath('../maple/maple_generated/2_dof_system'));
addpath(genpath('../urdf_creation'))
addpath(genpath('../main_simulink'))
cd /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink
set(groot, 'DefaultFigurePosition', [350, -650, 800, 600])

init_casadi;

simulink_main_model_name = 'sim_discrete_planar';
open_simulink_on_start = true;
if(~bdIsLoaded(simulink_main_model_name) && open_simulink_on_start && sum(strfind([getenv().keys{:}], 'VSCODE')) == 0)
    tic
    fprintf(['open simulink model \"' simulink_main_model_name, '\" ...'])
    open_system([simulink_main_model_name '.slx'])
    fprintf([' finished! (Loading time: ' num2str(toc) ' s)\n']);
end

%% debug parameter
start_in_singularity = false;
trajectory_out_of_workspace = true; % TODO: einfach offset 0 setzten
x_traj_out_of_workspace_value = 0.1;

plot_trajectory = ~true;
overwrite_offline_traj = false;

%% Other init scripts
T_sim = 10; % = param_vis.T (see init_visual.m)
param_global.Ta = 1e-3;

param_robot_init;
param_visual;
bus_definitions;
init_MPC_weights; %% set MPC weights

%% Trajectory selection
mdl = 'trajectory_generation_ref_subsys_model_ref';
traj_select.equilibrium_traj = 1;
traj_select.differential_filter = 2;
traj_select.polynomial = 3;
traj_select.sinus = 4;
traj_select.traj_amount = 4; % anzahl der trajektorien
%%%%%%%%%
traj_select_fin = 4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param_traj_allg.T_switch = 2;%T_sim/2; % ab dem zeitpunkt schält xe0 in xeT um und umgekehrt (only for differential filter)
%% Param CT Controller
ct_ctrl_param.Kd1 = diag([1e3 1e3]);
ct_ctrl_param.Kp1 = ct_ctrl_param.Kd1^2/4;

%ct_ctrl_param.Kd1 = diag(ones(1,2)*500);
%ct_ctrl_param.Kp1 = diag(ones(1,2)*1000);

%ct_ctrl_param.Kd1 = diag(ones(1,2)*80);
%ct_ctrl_param.Kp1 = diag(ones(1,2)*64);

ct_ctrl_param.mode = 2;
% 0: no sing robust
% 1: simpliy use J_pinv = (J'*J + k*E)^(-1)*J' = (J'*J + k*E)\J'
% 2: use Sugihara singular robust method: J_pinv = (J'*W_E*J + W_N)^(-1)*J' = (J'*W_E*J + W_N)\J'
% 3: set sing values sigma_i < eps to sign(sigma_i)/q_i_max

% 1:
ct_ctrl_param.k = 1e-2;

% 2:
ct_ctrl_param.w_bar_N = 1e-3*[param_robot.l__1^2; param_robot.l__2^2];
ct_ctrl_param.w_E = 1e0; %ct_ctrl_param.w_bar_N;

% 3:
ct_ctrl_param.eps  = 1e-1;

%% Param Trajektory differential filter 5th order
param_ct_traj_filter;

%% Param Trajectory Poly
param_traj_poly.T = T_sim/2-3; % in s

%% Param sinus poly trajectory
param_traj_sin_poly.T     = 1; % in s
T_period                  = 2; % in s
param_traj_sin_poly.omega = 2*pi*1/(T_period); % in rad
param_traj_sin_poly.phi   = 0; % in rad

%% Calculate target positions

H_streched = hom_transform_endeffector([0 0], param_robot); % singular pose

xeT = [H_streched(1:3,4);0;0;0];

xxe0 = [0.1 xeT(2)];

xe0 = [xxe0 0 0 0 0]';

if(start_in_singularity)
    % set xe0 to xeT and xeT to xe0;
    temp = xeT;
    xeT = xe0;
    xe0 = temp;
end

qq1 = inverse_kinematik_down([xe0(1) xe0(2)-1e-17], param_robot); % upper solution
qq2 = inverse_kinematik_up([xe0(1) xe0(2)-1e-17], param_robot); % lower solution
q_0 = qq1;
q_0_p = [0 0];

H_0_init = hom_transform_endeffector(q_0, param_robot);

if(trajectory_out_of_workspace)
    if(start_in_singularity)
        xe0(1) = xe0(1) + x_traj_out_of_workspace_value;
    else
        xeT(1) = xeT(1) + x_traj_out_of_workspace_value;
    end
    H_0_init(1, 4) = H_0_init(1, 4) + x_traj_out_of_workspace_value;
end

R_init = H_0_init(1:3, 1:3);

%R_target = eval_path_r(theta,1,path_param);
R_target = eye(3);
RR = R_init'*R_target;
rot_quat = rotation2quaternion(RR);
rot_rho = rot_quat(1);
rot_alpha_scale = 2*acos(rot_rho);
rot_ax = rot_quat(2:4)/sin(rot_alpha_scale/2); % bug: in case of non rotation rot_ax is NAN!

rot_alpha_scale_init = rot_alpha_scale;
rot_ax_init = rot_ax;

param_init_pose = struct;
param_init_pose.xe0 = xe0;
param_init_pose.xeT = xeT;
param_init_pose.q_0 = q_0;
param_init_pose.q_0_p = q_0_p;
param_init_pose.R_init = R_init;
param_init_pose.rot_alpha_scale_init = rot_alpha_scale;
param_init_pose.rot_ax_init = rot_ax;


%% GENERATE OFFLINE TRAJECTORY

files = dir('./s_functions/mpc_settings/*.mat');
cellfun(@load, {files.name}); % if no files then the for loop doesn't run.

% 1. get maximum horizont length of all mpcs:
T_horizon_max = 0;
N_sum = 0;
Ts_sum = 0;
for name={files.name}
    name_mat_file    = name{1};
    param_MPC_name   = name_mat_file(1:end-4);
    param_MPC_struct = eval(param_MPC_name);

    T_horizon = param_MPC_struct.T_horizon;
    N_sum = N_sum + param_MPC_struct.N;
    Ts_sum = Ts_sum + param_MPC_struct.Ts;

    if(T_horizon > T_horizon_max)
        T_horizon_max = T_horizon;
    end
end

param_MPC_traj_data_mat_file = "./s_functions/trajectory_data/param_traj_data.mat";
param_traj_data_old = './s_functions/trajectory_data/param_traj_data_old.mat';
load(param_traj_data_old);

T_traj_poly         = param_traj_poly.T;
T_traj_sin_poly     = param_traj_sin_poly.T;
omega_traj_sin_poly = param_traj_sin_poly.omega;
phi_traj_sin_poly   = param_traj_sin_poly.phi;
T_switch            = param_traj_allg.T_switch;

T_start = param_vis.T/2;
t = 0 : param_global.Ta : T_sim + T_horizon_max;

try

if(any(q_0 ~= q_0_old) || any(q_0_p ~= q_0_p_old) || ...
        any(xe0 ~= xe0_old) || any(xeT ~= xeT_old) || ...
        lamda_alpha ~= lamda_alpha_old || lamda_xyz ~= lamda_xyz_old || ...
        T_sim ~= T_sim_old || ...
        param_global.Ta ~= Ta_old || overwrite_offline_traj || ...
        T_horizon_max ~= T_horizon_max_old || ...
        T_traj_poly_old ~= T_traj_poly || ...
        T_traj_sin_poly_old ~= T_traj_sin_poly || ...
        omega_traj_sin_poly_old ~= omega_traj_sin_poly || ...
        phi_traj_sin_poly_old ~= phi_traj_sin_poly || ... 
        T_switch_old ~= T_switch || N_sum ~= N_sum_old || Ts_sum ~= Ts_sum_old )
    
    % 2. calculate all trajectories for max horizon length
    N_traj = ceil(1+(T_sim + T_horizon_max)/param_global.Ta);
    param_traj_data.t         = zeros(N_traj, 1);
    param_traj_data.p_d       = zeros(3, N_traj, traj_select.traj_amount);
    param_traj_data.p_d_p     = zeros(3, N_traj, traj_select.traj_amount);
    param_traj_data.p_d_pp    = zeros(3, N_traj, traj_select.traj_amount);
    param_traj_data.q_d       = zeros(4, N_traj, traj_select.traj_amount);
    param_traj_data.omega_d   = zeros(3, N_traj, traj_select.traj_amount);
    param_traj_data.omega_d_p = zeros(3, N_traj, traj_select.traj_amount);
    
    for i=1:traj_select.traj_amount
        tic;
        
        param_trajectory = generate_trajectory(t, i, xe0, xeT, R_init, rot_ax, rot_alpha_scale, T_start, param_traj_filter, param_traj_poly, param_traj_sin_poly, param_traj_allg);
        disp(['parameter.m: Execution Time for Trajectory Calculation: ', sprintf('%f', toc), 's']);
    
        param_traj_data.t(       :, :   ) = param_trajectory.t;
        param_traj_data.p_d(     :, :, i) = param_trajectory.p_d;
        param_traj_data.p_d_p(   :, :, i) = param_trajectory.p_d_p;
        param_traj_data.p_d_pp(  :, :, i) = param_trajectory.p_d_pp;
        param_traj_data.q_d(     :, :, i) = param_trajectory.q_d;
        param_traj_data.omega_d( :, :, i) = param_trajectory.omega_d;
        param_traj_data.omega_d_p(:, :, i) = param_trajectory.omega_d_p;
    end
    
    save(param_MPC_traj_data_mat_file, 'param_traj_data'); % save struct
    
    % 3. calculate initial guess for alle trajectories and mpcs
    
    compile_sfun                    = false;
    weights_and_limits_as_parameter = true;
    plot_null_simu                  = false;
    print_init_guess_cost_functions = false;
    
    tic
    for name={files.name}
        name_mat_file    = name{1};
        param_MPC_name   = name_mat_file(1:end-4);
        param_MPC_struct = eval(param_MPC_name);
    
        param_MPC_init_guess_name = param_MPC_name+"_init_guess";
        param_MPC_init_guess_mat_file = "./s_functions/initial_guess/" + param_MPC_init_guess_name + ".mat";
    
        casadi_func_name = param_MPC_struct.name;
        MPC_variant      = param_MPC_struct.variant;
        MPC_solver       = param_MPC_struct.solver;
        Ts_MPC           = param_MPC_struct.Ts     ;
        rk_iter          = param_MPC_struct.rk_iter;
        N_MPC            = param_MPC_struct.N  ;
        T_horizon_MPC    = param_MPC_struct.T_horizon;                   
        N_step_MPC       = param_MPC_struct.N_step;
        MPC_version      = param_MPC_struct.version;

        init_guess_cell = cell(1, traj_select.traj_amount);
            
        for ii=1:traj_select.traj_amount
            param_trajectory = struct;
            param_trajectory.p_d = param_traj_data.p_d(:,:,ii);
            param_trajectory.p_d_p = param_traj_data.p_d_p(:,:,ii);
            param_trajectory.p_d_pp = param_traj_data.p_d_pp(:,:,ii);
    
            opts = struct; % should be empty
            if(strcmp(MPC_variant, 'opti'))
                opti_opt_problem;
            elseif(strcmp(MPC_variant, 'nlpsol'))
                %nlpsol_opt_problem;
                %nlpsol_opt_problem_SX;
                nlpsol_opt_problem_SX_v2;
            else
                error(['Error: Variant = ', MPC_variant, ' is not valid. Should be (opti | nlpsol)']);
            end

            init_guess_cell{ii} = init_guess;
        end

        init_guess_arr = vertcat(init_guess_cell{:});
    
        param_MPC = struct('init_guess', init_guess_arr);
        
        eval(param_MPC_init_guess_name+ ' = param_MPC;');
        save(param_MPC_init_guess_mat_file, param_MPC_init_guess_name);
    end
    disp(['parameter.m: Execution Time for Init guess Calculation: ', sprintf('%f', toc), 's']);

    % save old data
    q_0_old = q_0;
    q_0_p_old = q_0_p;
    xe0_old = xe0;
    xeT_old = xeT;
    T_sim_old = T_sim;
    Ta_old = param_global.Ta;
    lamda_xyz_old = lamda_xyz;
    lamda_alpha_old = lamda_alpha;
    T_traj_poly_old         = T_traj_poly        ;
    T_traj_sin_poly_old     = T_traj_sin_poly    ;
    omega_traj_sin_poly_old = omega_traj_sin_poly;
    phi_traj_sin_poly_old   = phi_traj_sin_poly  ;
    T_switch_old = T_switch;
    T_horizon_max_old = T_horizon_max;
    N_sum_old = N_sum;
    Ts_sum_old = Ts_sum;

    save(param_traj_data_old, 'q_0_old', 'q_0_p_old', 'xe0_old', 'xeT_old', ...
         'lamda_alpha_old', 'lamda_xyz_old', 'T_sim_old', ...
         'Ta_old', 'T_traj_poly_old', ...
         'T_traj_sin_poly_old', 'omega_traj_sin_poly_old', 'phi_traj_sin_poly_old' , ...
         'T_switch_old', 'T_horizon_max_old', 'N_sum_old', 'Ts_sum_old');

    init_MPC_weights; % why necessary?
else
    files = dir('./s_functions/initial_guess/*.mat');
    cellfun(@load, {files.name});

    load(param_MPC_traj_data_mat_file);
end

catch
    disp('Cannot create init guess, please compile new')
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
end

%% DEBUG

ew_test_CT_CTRL = [diag(-ct_ctrl_param.Kd1/2 + sqrt(ct_ctrl_param.Kd1^2/4 - ct_ctrl_param.Kp1)) diag(-ct_ctrl_param.Kd1/2 - sqrt(ct_ctrl_param.Kd1^2/4 - ct_ctrl_param.Kp1))]';
plot_eigenvalues_controller_text([ew_test_CT_CTRL ew_test_CT_CTRL*0], 'Eigenvalues CT Ctrl', '');