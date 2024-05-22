% Init scripts
if(~exist('parameter_str', 'var')) % otherwise parameter_str is already defined from parameters_xdof
    %parameter_str = "parameters_2dof"; % default value
    parameter_str = "parameters_7dof"; % default value
end
run(parameter_str); init_casadi; import casadi.*;
%dbstop if error
%% GLOBAL SETTINGS FOR MPC

% show plot functions
print_init_guess_cost_functions = false;
plot_init_guess                 = false; % plot init guess
plot_null_simu                  = false; % plot system simulation for x0 = 0, u0 = ID(x0)
convert_maple_to_casadi         = false; % convert maple functions into casadi functions
fullsimu                        = false; % make full mpc simulation and plot results
traj_select_mpc                 = 3; % (1: equilibrium, 2: 5th order diff filt, 3: 5th order poly, 4: smooth sinus)
weights_and_limits_as_parameter = true; % otherwise minimal set of inputs and parameter is used. Leads to faster run time and compile time.
compile_sfun                    = true; % needed for simulink s-function, filename: "s_function_"+casadi_func_name
compile_matlab_sfunction        = ~true; % only needed for matlab MPC simu, filename: "casadi_func_name

% Compile Mode:
% compile_mode = 1 | nlpsol-sfun | fast compile time | very accurate,          | sometimes slower exec
% compile_mode = 2 | opti-sfun   | slow compile time | sometimes less accurate | sometimes faster exec

MPC='MPC1';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'ipopt'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version = 'v2'; % (v1: (J(u,y)) | v2: J(y,y_p,y_pp) | v3: ineq & traj feasible)
param_casadi_fun_name.(MPC).Ts      = 20e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).int_method = 'RK4'; % (RK4 | Euler)

MPC='MPC2';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v2'; % (v1: (J(u,y)) | v2: J(y,y_p,y_pp) | v3: ineq & traj feasible)
param_casadi_fun_name.(MPC).Ts      = 20e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).int_method = 'RK4'; % (RK4 | Euler)

MPC='MPC3';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v3'; % (v1: (J(u,y)) | v2: J(y,y_p,y_pp) | v3: ineq & traj feasible)
param_casadi_fun_name.(MPC).Ts      = 10e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).int_method = 'RK4'; % (RK4 | Euler)

MPC='MPC4';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v1'; % (v1: (J(u,y)) | v2: J(y,y_p,y_pp) | v3: ineq & traj feasible)
param_casadi_fun_name.(MPC).Ts      = 10e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).int_method = 'RK4'; % (RK4 | Euler)

MPC='MPC5';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v4'; % (v1: (J(u,y)) | v2: J(y,y_p,y_pp) | v3: ineq & traj feasible)
param_casadi_fun_name.(MPC).Ts      = 50e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).int_method = 'RK4'; % (RK4 | Euler)

MPC='MPC6';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v3_quat'; % (v1: (J(u,y)) | v2: J(y,y_p,y_pp) | v3: ineq & traj feasible)
param_casadi_fun_name.(MPC).Ts      = 1e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | Euler)

MPC='FeasibleBlock';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'traj_feasible'; % (v1: (J(u,y)) | v2: J(y,y_p,y_pp) | v3: ineq & traj feasible | traj_feasible)
param_casadi_fun_name.(MPC).Ts      = 1;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).int_method = 'RK4'; % (RK4 | Euler)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param_casadi_fun_struct = param_casadi_fun_name.MPC6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%param_casadi_fun_struct.name = 'MPC6_qrqp_nlpsol';

casadi_func_name = param_casadi_fun_struct.name;
Ts_MPC           = param_casadi_fun_struct.Ts     ; % MUSS TRAJEKTORIE OFFLINE BERECHNEN DAMIT DAS von param_global.Ta ABWEICHEN DARF
rk_iter          = param_casadi_fun_struct.rk_iter; % intermediate steps for runge kutta (rk_iter = 1 means no intermediate steps): DT = T_horizon_MPC/N_MPC/rk_iter;
N_MPC            = param_casadi_fun_struct.N_MPC  ; % Number of control intervals - so lang prädiziert man N=0: keine Prädiktion
T_horizon_MPC    = Ts_MPC*N_MPC;                    % Time horizon
N_step_MPC       = round(Ts_MPC/param_global.Ta);   % sampling steps for trajectory
MPC_variant      = param_casadi_fun_struct.variant;
MPC_solver       = param_casadi_fun_struct.solver;
MPC_version      = param_casadi_fun_struct.version;
compile_mode     = param_casadi_fun_struct.compile_mode;
int_method       = param_casadi_fun_struct.int_method;

% checks
if mod(Ts_MPC, param_global.Ta) ~= 0
    error('Error: Result is not an integer.');
end
%% Convert Maple Functions to casadi functions
if(convert_maple_to_casadi)
    maple_fun_arr = {"inertia_matrix.m", "coriolis_matrix.m", ...
        "gravitational_forces.m", "forward_kinematics_endeffector.m"...
        "hom_transform_endeffector.m", "geo_jacobian_endeffector.m", ...
        "geo_jacobian_endeffector_p.m"};
    maple_path = "maple/maple_generated/7_dof_system_fr3/"
    create_casadi_functions('SX', "maple/maple_generated/7_dof_system_fr3/", maple_fun_arr); %  script for matlab to casadi conversion
end

%% path for init guess
mpc_settings_path                = ['./', s_fun_path, '/mpc_settings/'];%todo:  UP
mpc_settings_struct_name         = "param_"+casadi_func_name;
param_MPC_traj_data_old_mat_file = ['./', s_fun_path, '/trajectory_data/param_traj_data_old.mat'];

%% Create trajectory for y_initial_guess
param_MPC_traj_data_mat_file = ['./', s_fun_path, '/trajectory_data/param_traj_data.mat'];

try
    load(param_MPC_traj_data_mat_file);
    load(param_MPC_traj_data_old_mat_file)
    traj_not_exist_flag = 0;
catch  
    traj_not_exist_flag = 1;
end

if(traj_not_exist_flag || T_horizon_MPC > T_horizon_max_old)
    if(traj_not_exist_flag)
        disp('mpc_casadi_main.m: Trajectory does not exist: create new trajectory');
    else
        disp('mpc_casadi_main.m: T_horizon_MPC > T_horizon_max_old: create new trajectory');
    end
    
    for i=1:traj_select.traj_amount % defined in parameters_xdof, x = 2, 7
        tic;
        
        param_trajectory = generate_trajectory(t, i, xe0, xeT, R_init, rot_ax, rot_alpha_scale, T_start, param_traj_filter, param_traj_poly, param_traj_sin_poly, param_traj_allg);
        disp(['parameter.m: Execution Time for Trajectory Calculation: ', sprintf('%f', toc), 's']);
    
        param_traj_data.t(       :, :   ) = param_trajectory.t;
        param_traj_data.p_d(     :, :, i) = param_trajectory.p_d;
        param_traj_data.p_d_p(   :, :, i) = param_trajectory.p_d_p;
        param_traj_data.p_d_pp(  :, :, i) = param_trajectory.p_d_pp;
        param_traj_data.R_d(     :, :, :, i) = param_trajectory.R_d;
        param_traj_data.q_d(     :, :, i) = param_trajectory.q_d;
        param_traj_data.q_d_p(     :, :, i) = param_trajectory.q_d_p;
        param_traj_data.omega_d( :, :, i) = param_trajectory.omega_d;
        param_traj_data.omega_d_p(:, :, i) = param_trajectory.omega_d_p;
    end
    
    save(param_MPC_traj_data_mat_file, 'param_traj_data'); % save struct
end

param_trajectory = struct;
param_trajectory.t         = param_traj_data.t;
param_trajectory.p_d       = param_traj_data.p_d(      :, :, traj_select_mpc);
param_trajectory.p_d_p     = param_traj_data.p_d_p(    :, :, traj_select_mpc);
param_trajectory.p_d_pp    = param_traj_data.p_d_pp(   :, :, traj_select_mpc);
param_trajectory.R_d       = param_traj_data.R_d(      :, :, :, traj_select_mpc);
param_trajectory.q_d       = param_traj_data.q_d(      :, :, traj_select_mpc);
param_trajectory.q_d_p     = param_traj_data.q_d_p(    :, :, traj_select_mpc);
param_trajectory.omega_d   = param_traj_data.omega_d(  :, :, traj_select_mpc);
param_trajectory.omega_d_p = param_traj_data.omega_d_p(:, :, traj_select_mpc);

%% OPT PROBLEM
%[TODO: oben init]
s_fun_path               = ['./', s_fun_path,'/']; % slash on end necessary! [TODO: more stable solution]
output_dir = s_fun_path; % needed?
casadi_fun_c_header_str  = [casadi_func_name, '.c'];
casadi_fun_h_header_str  = [casadi_func_name, '.h'];
s_func_name              = ['s_function_', casadi_fun_c_header_str]; % final name for Simulink s-function
s_fun_c_file_path        = [s_fun_path, s_func_name];
casadi_fun_h_header_path = [s_fun_path, casadi_func_name, '.h'];
casadi_fun_c_header_path = [s_fun_path, casadi_func_name, '.c'];
param_MPC_init_guess_name = ['param_', casadi_func_name, '_init_guess'];
param_MPC_init_guess_mat_file = ['./', s_fun_path, '/initial_guess/', param_MPC_init_guess_name, '.mat'];

substr = '_matlab';
MPC_matlab_name = [casadi_func_name, substr];

%% DEFINE OPTIMIZATION PROBLEM
if(strcmp(MPC_variant, 'opti'))
    error('Error: opti stack version currently not working!');
    opti_opt_problem;
elseif(strcmp(MPC_variant, 'nlpsol'))
    %nlpsol_opt_problem;
    %nlpsol_opt_problem_SX;
    nlpsol_opt_problem_SX_v2;
    %nlpsol_opt_problem_MX;
else
    error(['Error: Variant = ', MPC_variant, ' is not valid. Should be (opti | nlpsol)']);
end

%% Test MPC (fast)

if(plot_init_guess)
    HH_e_test = arrayfun(@(u) hom_transform_endeffector(x_init_guess(1:n,u), param_robot), 1:N_MPC+1, UniformOutput=false);
    HH_e_test_arr = cell2mat(HH_e_test);
    p_e_test_arr = HH_e_test_arr(1:2,4:4:(N_MPC+1)*4)';

    subplot(3,1,1)
    plot((0:N_MPC-1)*Ts_MPC, u_init_guess)
    xlabel('time (s)');
    ylabel('tau (Nm)')
    
    subplot(3,1,2);
    plot((0:N_MPC)*Ts_MPC, p_e_test_arr(:,1))
    xlabel('time (s)');
    ylabel('x pos (m)')
    
    subplot(3,1,3);
    plot((0:N_MPC)*Ts_MPC, p_e_test_arr(:,2))
    xlabel('time (s)');
    ylabel('y pos (m)')
end

%% Pre Simulation
% TODO: S-funktion slx file und inputs und outputs angeben. Wird aber nicht
% viel schneller als die eigentliche Simulation sein.
%disp(['Bevor "if(fullsimu)" Rechenzeit: ', sprintf('%f', toc), ' s']);
if(fullsimu)

    if(exist(MPC_matlab_name, 'file') == 3)
        matlab_sfun = str2func(MPC_matlab_name);
    else
        error(['Error: no matlab s-fun ', MPC_matlab_name,' exists!'])
    end
    
    %load("./s_functions/trajectory_data/"+"param_"+casadi_func_name+"_traj_data.mat")
    %param_trajectory = eval("param_"+casadi_func_name+"_traj_data");
    %load('traj_data.mat');

    N_traj          = length(param_trajectory.t);
    N_traj_original = N_traj - N_MPC*N_step_MPC;
    p_e_act_arr     = zeros(2, N_traj_original);
    u_k_new_arr     = zeros(n, N_traj_original);
    x_k_new_arr     = zeros(2*n, N_traj_original);
    x_k_new         = x_0_0;
    u_k_act         = u_k_0;

    % set additional needed N samples as last sample of trajectory
    % already done (see "parameter.m"):
    % traj_data = [param_trajectory.p_d; repmat(param_trajectory.p_d(end,:), N,1)];
    
    tic
    for i=1:1:N_traj_original
        p_d_traj    = param_trajectory.p_d(   1:3, i : N_step_MPC : i + (N_MPC  )*N_step_MPC);
        p_d_p_traj  = param_trajectory.p_d_p( 1:3, i : N_step_MPC : i + (N_MPC  )*N_step_MPC);
        p_d_pp_traj = param_trajectory.p_d_pp(1:3, i : N_step_MPC : i + (N_MPC-1)*N_step_MPC);

        y_ref    = p_d_traj(   1:2, 2:N_MPC+1); %       y1,   y2,   ... yN
        y_ref_p  = p_d_p_traj( 1:2, 2:N_MPC+1); %       y1p,  y2p,  ... yNp
        y_ref_pp = p_d_pp_traj(1:2, 1:N_MPC);   % y0pp, y1pp, y2pp, ... yN-1pp
        
        try
            [u_opt, x_init_guess, u_init_guess, lam_x_init_guess, lam_g_init_guess, cost_values_sol{1:length(cost_vars_SX)}] = f_opt(y_ref, y_ref_p, y_ref_pp, x_k_new, x_init_guess, u_init_guess, lam_x_init_guess, lam_g_init_guess, param_weight_init_cell{:});
        catch ME
            disp("error at " + (i-1)*param_global.Ta + " s ( i="+i+")")
            disp(ME.message);
        end
    
        HH_e_act = hom_transform_endeffector(x_k_new, param_robot);
        p_e_act  = HH_e_act(1:2,4);

        u_k_new     = full(u_opt);
        x_k_new_sim = full(sim(x_init_guess(:,1), u_k_new));
        x_k_new     = x_k_new_sim(:,1);
        %x_k_new = x_init_guess(:,2); % ist nicht gleich wie obere zeile???? Fehler 10^-6

        p_e_act_arr(:, i) = p_e_act;
        u_k_new_arr(:, i) = u_k_new;
        x_k_new_arr(:, i) = x_k_new;
    end
    disp(['full simu execution time: ', num2str(toc), ' s'])

    figure;
    subplot(3,1,1);
    plot(Ts_MPC*(1:N_traj_original), u_k_new_arr);
    title('u');
    subplot(3,1,2);
    plot(Ts_MPC*(1:N_traj_original), x_k_new_arr);
    title('x');
    subplot(3,1,3);
    plot(Ts_MPC*(1:N_traj_original), p_e_act_arr);
    title('p_e');
    
end
%% TODO: Save data (besser gleich in parameter speichern)

param_MPC = struct( ...
  'N',                    N_MPC, ...
  'N_step',               N_step_MPC, ...
  'Ts',                   Ts_MPC, ...
  'T_horizon',            T_horizon_MPC, ...
  'rk_iter',              rk_iter, ...
  'variant',              MPC_variant, ...
  'solver',               MPC_solver, ...
  'version',              MPC_version, ...
  'name',                 param_casadi_fun_struct.name, ...
  'int_method',           int_method ...
);

eval(mpc_settings_struct_name+"=param_MPC;"); % set new struct name
save(""+mpc_settings_path+mpc_settings_struct_name+'.mat', mpc_settings_struct_name);

% save old data %TODO: Struct speichern
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
%N_sum_old = -1;

save(param_traj_data_old, 'q_0_old', 'q_0_p_old', 'xe0_old', 'xeT_old', ...
     'lamda_alpha_old', 'lamda_xyz_old', 'T_sim_old', ...
     'Ta_old', 'T_traj_poly_old', ...
     'T_traj_sin_poly_old', 'omega_traj_sin_poly_old', 'phi_traj_sin_poly_old' , ...
     'T_switch_old', 'T_horizon_max_old', 'N_sum_old', 'Ts_sum_old');

%% COMPILE matlab s_function (can be used as normal function in matlab)
if(compile_matlab_sfunction)
    % re-define same casadi function with new name
    % Da der Name von f_opt der matlab name ist und im Objekt gespeichert ist verwendet 
    % die Funktion casadi_fun_to_mex ebenfalls diesen Namen und er muss daher nicht
    % übergeben werden.
    f_opt = Function(MPC_matlab_name, ...
        {input_vars_MX{:}},...
        {output_vars_MX{:}});
    casadi_fun_to_mex(f_opt, 's_functions', '-O2');
    disp(['Compile time for matlab s-function: ', num2str(toc), ' s']);
end

run(parameter_str);
cd ..
cd MA24_simulink