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
%cd /media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/matlab
addpath(genpath('./utils'));
addpath(genpath('./s_functions'));
addpath(genpath('./maple_generated/2_dof_system'));
addpath(genpath('./MPC_shared_subsystems'));
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
trajectory_out_of_workspace = ~true; % TODO: einfach offset 0 setzten
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
%%%%%%%%%
traj_select_fin = 3;
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

%% GENERATE OFFLINE TRAJECTORY
% verwendet scheinbar per default param_global.Ta
% show possible values: simConfig = simset('Solver', 'ode4', 'FixedStep', '0.001')
% https://github.com/mathworks/simulinkDroneReferenceApp/blob/master/data/slrt_plant_configset.m

files = dir('./s_functions/initial_guess/*.mat');
cellfun(@load, {files.name}); % if no files then the for loop doesn't run.

for name={files.name}
    name_mat_file                = name{1};
    param_MPC_name               = name_mat_file(1:end-4);
    param_MPC_old_data_file      = "./s_functions/trajectory_data/" + param_MPC_name+"_old_data.mat"; % TODO: dynamic path
    param_MPC_traj_data_name     = param_MPC_name+"_traj_data";
    param_MPC_traj_data_mat_file = "./s_functions/trajectory_data/" + param_MPC_traj_data_name + ".mat";

    load(param_MPC_old_data_file);
    load(param_MPC_name+"_traj_data");

    T_traj_poly         = param_traj_poly.T;
    T_traj_sin_poly     = param_traj_sin_poly.T;
    omega_traj_sin_poly = param_traj_sin_poly.omega;
    phi_traj_sin_poly   = param_traj_sin_poly.phi;
    T_switch            = param_traj_allg.T_switch;

    param_MPC_struct           = eval(param_MPC_name);
    param_MPC_traj_data_struct = eval(param_MPC_traj_data_name);

    casadi_func_name = param_MPC_struct.name;
    MPC_variant      = param_MPC_struct.variant;
    MPC_solver       = param_MPC_struct.solver;
    Ts_MPC           = param_MPC_struct.Ts     ;
    rk_iter          = param_MPC_struct.rk_iter;
    N_MPC            = param_MPC_struct.N  ;
    T_horizon_MPC    = Ts_MPC*N_MPC;                   
    N_step_MPC       = round(Ts_MPC/param_global.Ta);  

    N_traj_new = 1+(T_sim+T_horizon_MPC)/param_global.Ta;
    N_traj_old = length(param_MPC_traj_data_struct.t');

    if(any(q_0 ~= q_0_old) || any(q_0_p ~= q_0_p_old) || ...
        any(xe0 ~= xe0_old) || any(xeT ~= xeT_old) || ...
        (traj_select_fin == traj_select.differential_filter) && ...
        (lamda_alpha ~= lamda_alpha_old || lamda_xyz ~= lamda_xyz_old) || ...
        T_sim ~= T_sim_old || traj_select_fin ~= traj_select_fin_old || ...
        param_global.Ta ~= Ta_old || overwrite_offline_traj || ...
        N_MPC_old ~= N_MPC || Ts_MPC_old ~= Ts_MPC || ...
        N_traj_new ~= N_traj_old || T_traj_poly_old ~= T_traj_poly || ...
        T_traj_sin_poly_old ~= T_traj_sin_poly || ...
        omega_traj_sin_poly_old ~= omega_traj_sin_poly || ...
        phi_traj_sin_poly_old ~= phi_traj_sin_poly || T_switch_old ~= T_switch )
    
        % create trajectory
        param_trajectory = generateTrajectory_from_slx(mdl, q_0, H_0_init, traj_select_fin, T_sim, T_horizon_MPC, param_global);
        eval(param_MPC_traj_data_name+"=param_trajectory;"); % set new struct name
        
        compile_sfun                    = false;
        weights_and_limits_as_parameter = true;
        plot_null_simu                  = false;
        print_init_guess_cost_functions = false;
        
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
        save(param_MPC_traj_data_mat_file, param_MPC_traj_data_name); % save struct

        % save old data
        q_0_old = q_0;
        q_0_p_old = q_0_p;
        xe0_old = xe0;
        xeT_old = xeT;
        T_sim_old = T_sim;
        Ta_old = param_global.Ta;
        traj_select_fin_old = traj_select_fin;
        lamda_xyz_old = lamda_xyz;
        lamda_alpha_old = lamda_alpha;
        N_MPC_old = N_MPC;
        Ts_MPC_old = Ts_MPC;
        T_traj_poly_old         = T_traj_poly        ;
        T_traj_sin_poly_old     = T_traj_sin_poly    ;
        omega_traj_sin_poly_old = omega_traj_sin_poly;
        phi_traj_sin_poly_old   = phi_traj_sin_poly  ;
        T_switch_old = T_switch;
        
        save(param_MPC_old_data_file, 'q_0_old', 'q_0_p_old', 'xe0_old', 'xeT_old', ...
             'lamda_alpha_old', 'lamda_xyz_old', 'T_sim_old', 'traj_select_fin_old', ...
             'Ta_old', 'N_MPC_old', 'Ts_MPC_old', 'T_traj_poly_old', ...
             'T_traj_sin_poly_old', 'omega_traj_sin_poly_old', 'phi_traj_sin_poly_old' , ...
             'T_switch_old');
    %else
    %    load(param_MPC_name+"_traj_data");
    end
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

%parameter_force_create_new_initial_guess = false; % TODO: delete


%% DEBUG

ew_test_CT_CTRL = [diag(-ct_ctrl_param.Kd1/2 + sqrt(ct_ctrl_param.Kd1^2/4 - ct_ctrl_param.Kp1)) diag(-ct_ctrl_param.Kd1/2 - sqrt(ct_ctrl_param.Kd1^2/4 - ct_ctrl_param.Kp1))]';
plot_eigenvalues_controller_text([ew_test_CT_CTRL ew_test_CT_CTRL*0], 'Eigenvalues CT Ctrl', '');