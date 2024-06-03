%% INIT
if(exist('parameter_str', 'var') && strcmp(parameter_str, "parameters_2dof"))
    rmpath('../utils/matlab_init_2dof');
    rmpath('../maple/maple_generated/2_dof_system');
end

%if strcmp(mfilename, 'parameters')
    clear
    %close all
    clc
%end

start_in_singularity = false;
trajectory_out_of_workspace = false; % TODO: einfach offset 0 setzten
x_traj_out_of_workspace_value = 0.1;

plot_trajectory = ~true;
overwrite_offline_traj = false;

% (turn off profiler when not needed anymore)
% set_param(gcs,'Profile','off');
% measure compile time: 
%disp('Compile Time:')

%tic;set_param('sim_discrete_planar', 'SimulationCommand', 'update');toc
%rmpath('/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/s_functions')

%closeAllSimulinkModels('./MPC_shared_subsystems')
%closeAllSimulinkModels('.')

parameter_str = "parameters_7dof";
s_fun_path = 's_functions/s_functions_7dof';

%restoredefaultpath
cd /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/main_matlab % [TODO: besser machen]
addpath(genpath('../main_matlab'));
addpath(genpath('../utils/matlab_utils'));
addpath(genpath('../utils/matlab_init_7dof'));
addpath(genpath('../utils/utils_casadi'));
addpath(genpath(['../', s_fun_path]));
addpath(genpath('../maple/maple_generated/7_dof_system_fr3'));
addpath(genpath('../urdf_creation'))
addpath(genpath('../main_simulink'))
cd /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink
set(groot, 'DefaultFigurePosition', [350, -650, 800, 600])

init_casadi;
import casadi.*;

simulink_main_model_name = 'sim_discrete_7dof';
open_simulink_on_start = true;
%if(~bdIsLoaded(simulink_main_model_name) && open_simulink_on_start && sum(strfind([getenv().keys{:}], 'VSCODE')) == 0)
% && ~isSimulinkStarted funktioniert einfach nicht.
if( ~contains([license('inuse').feature], 'simulink') && open_simulink_on_start && desktop('-inuse') == 1) % in vscode inuse delivers 0 (running in background)
    tic
    fprintf(['open simulink model \"' simulink_main_model_name, '\" ...'])
    open_system([simulink_main_model_name '.slx'])
    fprintf([' finished! (Loading time: ' num2str(toc) ' s)\n']);
end

%% Other init scripts
T_sim = 10; % = param_vis.T (see init_visual.m)
param_global.Ta = 1e-3;

param_visual;
param_robot_init;
n = param_robot.n_DOF;
m = param_robot.m;

bus_definitions;
init_MPC_weights; %% set MPC weights

%% Trajectory selection
traj_select.equilibrium_traj = 1;
traj_select.differential_filter = 2;
traj_select.polynomial = 3;
traj_select.sinus = 4;
traj_select.traj_amount = 4; % anzahl der trajektorien
%%%%%%%%%
traj_select_fin = 4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param_traj_allg.T_switch = T_sim/2; % ab dem zeitpunkt schält xe0 in xeT um und umgekehrt (only for differential filter)
%% Param CT Controller

K_d_t = 1e3*diag([1 1 1]);
K_p_t = K_d_t^2/4;

K_d_r = 1e3*diag([1 1 1]);
K_p_r = K_d_r^2/4;

ct_ctrl_param.Kd1 = blkdiag(K_d_t, K_d_r);
ct_ctrl_param.Kp1 = blkdiag(K_p_t, K_p_r);

ct_ctrl_param.mode = 0;
% 0: no sing robust
% 1: simpliy use J_pinv = (J'*J + k*E)^(-1)*J' = (J'*J + k*E)\J'
% 2: use Sugihara singular robust method: J_pinv = (J'*W_E*J + W_N)^(-1)*J' = (J'*W_E*J + W_N)\J'
% 3: set sing values sigma_i < eps to sign(sigma_i)/q_i_max

% 1:
ct_ctrl_param.k = 1e-2;

% 2:
ct_ctrl_param.w_bar_N = 1e-3*[param_robot.l1^2; param_robot.l2^2; param_robot.l3^2; param_robot.l4^2; param_robot.l5^2; param_robot.l6^2];% ; param_robot.l7^2];
ct_ctrl_param.W_E = 1e0 * eye(6); %ct_ctrl_param.w_bar_N;

% 3:
ct_ctrl_param.eps  = 1e-1;

% nullspace for CT controller
%ct_ctrl_param.q_n = param_robot.q_n; % q_n = (q_max + q_min) / 2;
%ct_ctrl_param.K_n = 1e-2*eye(n);
%ct_ctrl_param.D_n = sqrt(4*ct_ctrl_param.K_n) + 1 * eye(n);

%ct_ctrl_param.K_n = 64*eye(n);
%ct_ctrl_param.D_n = 16*eye(n);

ct_ctrl_param.k_n_nl = 10*eye(n);
ct_ctrl_param.nl_spring_threshold = [0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2];

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

H_0_init = hom_transform_endeffector(q_0_init, param_robot);

R_init = H_0_init(1:3, 1:3);
quat_init = rotation2quaternion(R_init);
xe0 = [H_0_init(1:3,4); quat_init]; % xe0 = [x,y,z,q1,q2,q3,q4]

rotq_fun = @(x1, x2) [x1(1:3) + x2(1:3); quat_mult(x1(4:7), x2(4:7))]; % multiplikation von quaternions = rotationen hintereinander ausführen
%xeT = xe0;
%xeT = rotq_fun(xe0, [0; 0; -0.5; 1; 0; 0; 0]); % correct addition of quaternions
%xeT = [xe0(1:3,1); rotation2quaternion(Rz(pi/4)*R_init)]; % correct addition of quaternions
%R_target = R_init*Rz(pi/4); % Nachmultiplikation: Drehung erfolgt in Bezu auf körperfestes System d. h. hier R_init
R_target = Rz(pi/4)*R_init; % Vormultiplikation: Drehung erfolgt in Bezug auf Inertialsystem
xeT = [xe0(1:3,1) + [0; 0; -0.5]; rotation2quaternion(R_target)]; % correct addition of quaternions

if(start_in_singularity)
    % set xe0 to xeT and xeT to xe0;
    temp = xeT;
    xeT = xe0;
    xe0 = temp;
end

C_matlab = casadi.Function.load(['./', s_fun_path, '/C_fun.casadi']); % matlab
compute_tau_fun = Function.load(['./', s_fun_path, '/compute_tau_py.casadi']);
g      = casadi.Function.load(['./', s_fun_path, '/gravitational_forces_py.casadi']);
M      = casadi.Function.load(['./', s_fun_path, '/inertia_matrix_py.casadi']); %
M_inv  = casadi.Function.load(['./', s_fun_path, '/inverse_inertia_matrix_py.casadi']); %
C_rnea = casadi.Function.load(['./', s_fun_path, '/n_q_coriols_qp_plus_g_py.casadi']);

J      = casadi.Function.load(['./', s_fun_path, '/geo_jacobian_endeffector_py.casadi']); % perfekt
J_p    = casadi.Function.load(['./', s_fun_path, '/geo_jacobian_endeffector_p_py.casadi']); % perfekt
H      = casadi.Function.load(['./', s_fun_path, '/hom_transform_endeffector_py.casadi']); % perfect
quat   = casadi.Function.load(['./', s_fun_path, '/quat_endeffector_py.casadi']);
sys_fun_qpp = casadi.Function.load(['./', s_fun_path, '/sys_fun_qpp_py.casadi']);

qpp_fun = @(q, q_p, tau) M(q)\(tau - C_rnea(q, q_p));
qpp_fun_maple = @(q, q_p, tau, param) inertia_matrix(q, param)\(tau - coriolis_matrix(q, q_p, param)*q_p - gravitational_forces(q, param));
qpp_fun_maple_casadi_SX = @(q, q_p, tau, param) inertia_matrix_casadi_SX(q, param)\(tau - coriolis_matrix_casadi_SX(q, q_p, param)*q_p - gravitational_forces_casadi_SX(q, param));
% qpp_fun_maple_casadi_SX_fun = casadi.Function('qpp_fun_maple_casadi_SX_fun', {sys_fun_qpp.sx_in{1}, sys_fun_qpp.sx_in{2}, sys_fun_qpp.sx_in{3}}, {qpp_fun_maple_casadi_SX(sys_fun_qpp.sx_in{1}, sys_fun_qpp.sx_in{2}, sys_fun_qpp.sx_in{3}, param_robot)}, {'q', 'q_p', 'tau'}, {'qpp'});
% C_rnea_maple = casadi.Function('C_rnea', {sys_fun_qpp.sx_in{1}, sys_fun_qpp.sx_in{2}}, {coriolis_matrix_casadi_SX(sys_fun_qpp.sx_in{1}, sys_fun_qpp.sx_in{2}, param_robot)*sys_fun_qpp.sx_in{2} + gravitational_forces_casadi_SX(sys_fun_qpp.sx_in{1}, param_robot)}, {'q', 'q_p'}, {'C_rnea'});

robot_model_bus_fun = casadi.Function.load(['./', s_fun_path, '/robot_model_bus_fun_py.casadi']);

%tests;

%% Inverse Kin (Zum Prüfen ob Endwert im Aufgabenraum ist.)
calc_inverse_kin = false;
if(calc_inverse_kin)
    % Define the cost function.
    Q1 = 1e10 * eye(6); % Weight for the position error in the cost function.
    Q2 = 0.5; % Weight for the manipulability error in the cost function.
    Q3 = 1*1e0 * eye(7); % Weight for the deviaton of q_sol to q_d
    Q4 = 1e-1 * eye(7);% Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
    tolerance = 0.01;
    random_q0_count = 100;

    q_d = param_robot.q_n;
    q_d = q_0;
    
    [q_sol, ~] = inverse_kinematics(param_robot, xeT, q_d, Q1, Q2, Q3, Q4, tolerance, random_q0_count, ct_ctrl_param);
    %%
end

if(trajectory_out_of_workspace)
    if(start_in_singularity)
        xe0(1) = xe0(1) + x_traj_out_of_workspace_value;
    else
        xeT(1) = xeT(1) + x_traj_out_of_workspace_value;
    end
    H_0_init(1, 4) = H_0_init(1, 4) + x_traj_out_of_workspace_value;
end

% kürzeste Rotationsachse und winkel zwischen R_init und R_target
% berechnen.
[rot_ax, rot_alpha_scale] = find_rotation_axis(R_init, R_target);

% Achtung: Hier wird angenommen, dass um eine feste Achse rotiert wird.
param_init_pose = struct;
param_init_pose.T_start = T_sim/2;
param_init_pose.xe0 = xe0;
param_init_pose.xeT = xeT;
param_init_pose.q_0_init = q_0_init;       % 7DOF q_0
param_init_pose.q_0_p_init = q_0_p_init;   % 7DOF q_0
param_init_pose.q_0_pp_init = q_0_pp_init; % 7DOF q_0
param_init_pose.q_0 = q_0;       % nDOF q_0
param_init_pose.q_0_p = q_0_p;   % nDOF q_0_p
param_init_pose.q_0_pp = q_0_pp; % nDOF q_0_pp
param_init_pose.R_init = R_init;
param_init_pose.R_target = R_target;
param_init_pose.rot_alpha_scale = rot_alpha_scale;
param_init_pose.rot_ax = rot_ax;
param_init_pose.Phi_init = rotm2rpy(R_init);
param_init_pose.Phi_target = rotm2rpy(R_target); %rotm2rpy(quaternion2rotation(xeT(4:7)));
param_init_pose.delta_Phi = param_init_pose.Phi_target - param_init_pose.Phi_init;

%% GENERATE OFFLINE TRAJECTORY

files = dir(['./', s_fun_path, '/mpc_settings/*.mat']);
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

param_MPC_traj_data_mat_file = ['./', s_fun_path, '/trajectory_data/param_traj_data.mat'];
param_traj_data_old = ['./', s_fun_path, '/trajectory_data/param_traj_data_old.mat'];
load(param_traj_data_old);

T_traj_poly         = param_traj_poly.T;
T_traj_sin_poly     = param_traj_sin_poly.T;
omega_traj_sin_poly = param_traj_sin_poly.omega;
phi_traj_sin_poly   = param_traj_sin_poly.phi;
T_switch            = param_traj_allg.T_switch;

T_start = param_vis.T/2;
t = 0 : param_global.Ta : T_sim + T_horizon_max;

% TODO: Alles so inefffizient mit diesen dauernden hin und her speichern

try

if(any(q_0_init ~= q_0_old) || ...
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
    param_traj_data.p_d       = zeros(3, N_traj,         traj_select.traj_amount);
    param_traj_data.p_d_p     = zeros(3, N_traj,         traj_select.traj_amount);
    param_traj_data.p_d_pp    = zeros(3, N_traj,         traj_select.traj_amount);
    param_traj_data.Phi_d     = zeros(3, N_traj,         traj_select.traj_amount);
    param_traj_data.Phi_d_p   = zeros(3, N_traj,         traj_select.traj_amount);
    param_traj_data.Phi_d_pp  = zeros(3, N_traj,         traj_select.traj_amount);
    param_traj_data.R_d       = zeros(3, 3,      N_traj, traj_select.traj_amount);
    param_traj_data.q_d       = zeros(4, N_traj,         traj_select.traj_amount);
    param_traj_data.q_d_p     = zeros(4, N_traj,         traj_select.traj_amount);
    param_traj_data.q_d_pp    = zeros(4, N_traj,         traj_select.traj_amount);
    param_traj_data.omega_d   = zeros(3, N_traj,         traj_select.traj_amount);
    param_traj_data.omega_d_p = zeros(3, N_traj,         traj_select.traj_amount);
    
    for i=1:traj_select.traj_amount
        tic;
        
        param_trajectory = generate_trajectory(t, i, param_init_pose, param_traj_filter, param_traj_poly, param_traj_sin_poly, param_traj_allg);
        disp(['parameter.m: Execution Time for Trajectory Calculation: ', sprintf('%f', toc), 's']);
    
        param_traj_data.t(        :, :      ) = param_trajectory.t;
        param_traj_data.p_d(      :, :, i   ) = param_trajectory.p_d;
        param_traj_data.p_d_p(    :, :, i   ) = param_trajectory.p_d_p;
        param_traj_data.p_d_pp(   :, :, i   ) = param_trajectory.p_d_pp;
        param_traj_data.Phi_d(    :, :, i   ) = param_trajectory.Phi_d;
        param_traj_data.Phi_d_p(  :, :, i   ) = param_trajectory.Phi_d_p;
        param_traj_data.Phi_d_pp( :, :, i   ) = param_trajectory.Phi_d_pp;
        param_traj_data.R_d(      :, :, :, i) = param_trajectory.R_d;
        param_traj_data.q_d(      :, :, i   ) = param_trajectory.q_d;
        param_traj_data.q_d_p(    :, :, i   ) = param_trajectory.q_d_p;
        param_traj_data.q_d_pp(   :, :, i   ) = param_trajectory.q_d_pp;
        param_traj_data.omega_d(  :, :, i   ) = param_trajectory.omega_d;
        param_traj_data.omega_d_p(:, :, i   ) = param_trajectory.omega_d_p;
    end
    
    save(param_MPC_traj_data_mat_file, 'param_traj_data'); % save struct
    
    % 3. calculate initial guess for alle trajectories and mpcs
    
    compile_sfun                    = false;
    weights_and_limits_as_parameter = true;
    plot_null_simu                  = false;
    print_init_guess_cost_functions = false;
    
    % [TODO] : Es ist viel einfacher direkt MPC_v3 aufzurufen und sich die
    % notwendigen daten davon zu holen. Das muss dann eben für jede MPC
    % separat getan werden, d. h. eig das If von nlpsol_opt_problem_v2
    
    tic
    %f = Function.load([s_fun_path, '/', 'sys_fun_x_py.casadi']); % forward dynamics (FD), d/dt x = f(x, u), x = [q; dq]
    for name={files.name}
        name_mat_file    = name{1};
        param_MPC_name   = name_mat_file(1:end-4);
        param_MPC_struct = eval(param_MPC_name);
    
        param_MPC_init_guess_name = [param_MPC_name, '_init_guess'];
        param_MPC_init_guess_mat_file = ['./', s_fun_path, '/initial_guess/', param_MPC_init_guess_name,'.mat'];
    
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

        DT = Ts_MPC;
        M = rk_iter;

        output_dir = ['./', s_fun_path,'/'];

        f_opt = Function.load([s_fun_path, '/', casadi_func_name, '.casadi']);
%{
        F = integrate_casadi(f, DT, M, int_method);
        m=3; %%%%%% PUFSCH
        z     = SX.sym('z',     2*m);
        alpha = SX.sym('alpha',   m);

        h_ref = Function('h_ref', {z, alpha}, {[z(m+1:2*m); alpha]});
        H = integrate_casadi(h_ref, DT, M, int_method);
%}
        param_weight_init = param_weight.(casadi_func_name);
        param_weight_init_cell = merge_cell_arrays(struct2cell(param_weight_init), 'vector');


        init_guess_cell = cell(1, traj_select.traj_amount);
            
        for ii=1:traj_select.traj_amount
            param_trajectory           = struct;
            param_trajectory.p_d       = param_traj_data.p_d(      :, :, ii   );
            param_trajectory.p_d_p     = param_traj_data.p_d_p(    :, :, ii   );
            param_trajectory.p_d_pp    = param_traj_data.p_d_pp(   :, :, ii   );
            param_trajectory.Phi_d     = param_traj_data.Phi_d(    :, :, ii   );
            param_trajectory.Phi_d_p   = param_traj_data.Phi_d_p(  :, :, ii   );
            param_trajectory.Phi_d_pp  = param_traj_data.Phi_d_pp( :, :, ii   );
            param_trajectory.R_d       = param_traj_data.R_d(      :, :, :, ii);
            param_trajectory.q_d       = param_traj_data.q_d(      :, :, ii   );
            param_trajectory.q_d_p     = param_traj_data.q_d_p(    :, :, ii   );
            param_trajectory.q_d_pp    = param_traj_data.q_d_pp(   :, :, ii   );
            param_trajectory.omega_d   = param_traj_data.omega_d(  :, :, ii   );
            param_trajectory.omega_d_p = param_traj_data.omega_d_p(:, :, ii   );
%{
            p_d_0    = param_trajectory.p_d(    1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_0 ... y_N)
            p_d_p_0  = param_trajectory.p_d_p(  1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_p_0 ... y_p_N)
            p_d_pp_0 = param_trajectory.p_d_pp( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_pp_0 ... y_pp_N)

            % Phi_d_0 = param_trajectory.Phi_d(     1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (Phi_0 ... Phi_N)
            % Phi_d_p_0 = param_trajectory.Phi_d_p( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (Phi_p_0 ... Phi_p_N)
            % Phi_d_pp_0 = param_trajectory.Phi_d_pp(1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (Phi_pp_0 ... Phi_pp_N)
            % 
            % % PFUSCH [TODO]
            % p_d_0 = [p_d_0; Phi_d_0];
            % p_d_p_0 = [p_d_p_0; Phi_d_p_0];
            % p_d_pp_0 = [p_d_pp_0; Phi_d_pp_0];

            % PFUSCH: [TODO] Duplicat in MPC_V3
            u_k_0  = compute_tau_fun(q_0, q_0_p, q_0_pp); % much more faster than above command
            u_init_guess_0 = ones(n, N_MPC).*u_k_0; % fully actuated

            x_0_0  = [q_0; q_0_p];
            F_sim              = F.mapaccum(N_MPC);
            x_init_guess_kp1_0 = F_sim(x_0_0, u_init_guess_0);
            x_init_guess_0     = [x_0_0 full(x_init_guess_kp1_0)];
            u_init_guess_0 = ones(n, N_MPC).*u_k_0; % fully actuated

            z_0_0 = [p_d_0(:,1); p_d_p_0(:,1)]; % init for z_ref
            alpha_init_guess_0 = p_d_pp_0(:, 1:end-1);
            alpha_N_0 = p_d_pp_0(:,end);
            
            H_sim              = H.mapaccum(N_MPC);
            z_init_guess_kp1_0 = H_sim(z_0_0, alpha_init_guess_0);
            z_init_guess_0     = [z_0_0 full(z_init_guess_kp1_0)];

            lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(z_init_guess_0)+numel(alpha_init_guess_0) + numel(alpha_N_0), 1);
            lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(z_init_guess_0)+1, 1); % + 1 wegen eps
            
            init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); z_init_guess_0(:); alpha_init_guess_0(:); alpha_N_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

            
            mpc_init_reference_values = [x_0_0(:); z_0_0(:); p_d_0(:); p_d_p_0(:); p_d_pp_0(:)];
    %}
            nlpsol_generate_opt_problem;
            [~, xx_full_opt_sol, ~] = f_opt(mpc_init_reference_values, init_guess_0, param_weight_init_cell);
            init_guess = full(xx_full_opt_sol);

            init_guess_cell{ii} = init_guess;
        end

        init_guess_arr = vertcat(init_guess_cell{:});
    
        param_MPC = struct('init_guess', init_guess_arr);
        
        eval([param_MPC_init_guess_name, ' = param_MPC;']);
        save(param_MPC_init_guess_mat_file, param_MPC_init_guess_name);
    end
    disp(['parameter.m: Execution Time for Init guess Calculation: ', sprintf('%f', toc), 's']);
    
    % save old data
    q_0_old                 = q_0_init;
    q_0_p_old               = q_0_p_init;
    xe0_old                 = xe0;
    xeT_old                 = xeT;
    T_sim_old               = T_sim;
    Ta_old                  = param_global.Ta;
    lamda_xyz_old           = lamda_xyz;
    lamda_alpha_old         = lamda_alpha;
    T_traj_poly_old         = T_traj_poly;
    T_traj_sin_poly_old     = T_traj_sin_poly;
    omega_traj_sin_poly_old = omega_traj_sin_poly;
    phi_traj_sin_poly_old   = phi_traj_sin_poly;
    T_switch_old            = T_switch;
    T_horizon_max_old       = T_horizon_max;
    N_sum_old               = N_sum;
    Ts_sum_old              = Ts_sum;

    save(param_traj_data_old, 'q_0_old', 'q_0_p_old', 'xe0_old', 'xeT_old', ...
         'lamda_alpha_old', 'lamda_xyz_old', 'T_sim_old', ...
         'Ta_old', 'T_traj_poly_old', ...
         'T_traj_sin_poly_old', 'omega_traj_sin_poly_old', 'phi_traj_sin_poly_old' , ...
         'T_switch_old', 'T_horizon_max_old', 'N_sum_old', 'Ts_sum_old');

    init_MPC_weights; % why necessary?
else
    files = dir(['./', s_fun_path, '/initial_guess/*.mat']);
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