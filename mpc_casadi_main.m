%% Init scripts
parameters; init_casadi; import casadi.*;

%% GLOBAL SETTINGS FOR MPC

% show plot functions
print_init_guess_cost_functions = false;
plot_init_guess                 = false; % plot init guess
plot_null_simu                  = false; % plot system simulation for x0=0, u0 = ID(x0)
convert_maple_to_casadi         = false; % convert maple functions into casadi functions
fullsimu                        = false; % make full mpc simulation and plot results
weights_and_limits_as_parameter = true; % otherwise minimal set of inputs and parameter is used. Leads to faster run time and compile time.
compile_sfunction               = true; % needed for simulink s-function, filename: "s_function_"+casadi_func_name
compile_matlab_sfunction        = ~true; % only needed for matlab MPC simu, filename: "casadi_func_name

param_casadi_fun_name = struct();
param_casadi_fun_name.MPC1.variant = 'opti';
param_casadi_fun_name.MPC1.solver  = 'qrqp'; % (qrqp (sqp) | qpoases?)
param_casadi_fun_name.MPC1.Ts      = 5e-3;
param_casadi_fun_name.MPC1.rk_iter = 1;
param_casadi_fun_name.MPC1.N_MPC   = 5;
param_casadi_fun_name.MPC1.terminal_ineq_yref_N = false; % not implemented for opti
param_casadi_fun_name.MPC1.terminal_soft_yref_N = false; % not implemented for opti

param_casadi_fun_name.MPC2.variant = 'opti';
param_casadi_fun_name.MPC2.solver  = 'qrqp'; % (qrqp (sqp) | qpoases?)
param_casadi_fun_name.MPC2.Ts      = 20e-3;
param_casadi_fun_name.MPC2.rk_iter = 1;
param_casadi_fun_name.MPC2.N_MPC   = 5;
param_casadi_fun_name.MPC2.terminal_ineq_yref_N = false; % not implemented for opti
param_casadi_fun_name.MPC2.terminal_soft_yref_N = false; % not implemented for opti

param_casadi_fun_name.MPC3.variant = 'nlpsol';
param_casadi_fun_name.MPC3.solver  = 'ipopt'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.MPC3.Ts      = 5e-3;
param_casadi_fun_name.MPC3.rk_iter = 1;
param_casadi_fun_name.MPC3.N_MPC   = 5;
param_casadi_fun_name.MPC3.terminal_ineq_yref_N = false;
param_casadi_fun_name.MPC3.terminal_soft_yref_N = false;

param_casadi_fun_name.MPC4.variant = 'nlpsol';
param_casadi_fun_name.MPC4.solver  = 'ipopt'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.MPC4.Ts      = 20e-3;
param_casadi_fun_name.MPC4.rk_iter = 1;
param_casadi_fun_name.MPC4.N_MPC   = 5;
param_casadi_fun_name.MPC4.terminal_ineq_yref_N = false;
param_casadi_fun_name.MPC4.terminal_soft_yref_N = false;

param_casadi_fun_name.MPC5.variant = 'nlpsol';
param_casadi_fun_name.MPC5.solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.MPC5.Ts      = 20e-3;
param_casadi_fun_name.MPC5.rk_iter = 1;
param_casadi_fun_name.MPC5.N_MPC   = 5;
param_casadi_fun_name.MPC5.terminal_ineq_yref_N = false;
param_casadi_fun_name.MPC5.terminal_soft_yref_N = false;

param_casadi_fun_name.MPC6.variant = 'nlpsol';
param_casadi_fun_name.MPC6.solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.MPC6.Ts      = 5e-3;
param_casadi_fun_name.MPC6.rk_iter = 1;
param_casadi_fun_name.MPC6.N_MPC   = 5;
param_casadi_fun_name.MPC6.terminal_ineq_yref_N = true;
param_casadi_fun_name.MPC6.terminal_soft_yref_N = false;

param_casadi_fun_name.MPC7.variant = 'nlpsol';
param_casadi_fun_name.MPC7.solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.MPC7.Ts      = 20e-3;
param_casadi_fun_name.MPC7.rk_iter = 1;
param_casadi_fun_name.MPC7.N_MPC   = 5;    
param_casadi_fun_name.MPC7.terminal_ineq_yref_N = false;
param_casadi_fun_name.MPC7.terminal_soft_yref_N = true;

% set mpc names:
for name = fieldnames(param_casadi_fun_name)'
    MPC_name = [name{:}, '_', param_casadi_fun_name.(name{:}).solver, '_', param_casadi_fun_name.(name{:}).variant];
    param_casadi_fun_name = setfield(param_casadi_fun_name, name{:}, 'name', MPC_name);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param_casadi_fun_struct = param_casadi_fun_name.MPC7;
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

terminal_ineq_yref_N = param_casadi_fun_struct.terminal_ineq_yref_N;
terminal_soft_yref_N = param_casadi_fun_struct.terminal_soft_yref_N;

% checks
if mod(Ts_MPC, param_global.Ta) ~= 0
    error('Error: Result is not an integer.');
end
%% Convert Maple Functions to casadi functions
if(convert_maple_to_casadi)
    create_casadi_functions; %  script for matlab to casadi conversion
end

%% path for init guess
init_guess_path = './s_functions/initial_guess/';%todo: UP
init_guess_struct_name = "param_"+casadi_func_name;
param_MPC_old_data_file = "./s_functions/trajectory_data/param_" + casadi_func_name+"_old_data.mat";

%% Create trajectory for y_initial_guess
param_MPC_traj_data_name = "param_"+casadi_func_name+"_traj_data";
param_MPC_traj_data_mat_file = "./s_functions/trajectory_data/" + param_MPC_traj_data_name + ".mat";

try
    load(param_MPC_traj_data_mat_file);
    load(param_MPC_old_data_file)
    traj_not_exist_flag = 0;
    param_trajectory = eval("param_"+casadi_func_name+"_traj_data");
catch  
    traj_not_exist_flag = 1;
end

if(N_MPC_old ~= N_MPC || Ts_MPC_old ~= Ts_MPC || traj_not_exist_flag)
    if(traj_not_exist_flag)
        disp('mpc_casadi_main.m: Trajectory does not exist: create new trajectory');
    else
        disp('mpc_casadi_main.m: N_MPC or TS_MPC changed: create new trajectory');
    end
    param_trajectory = generateTrajectory_from_slx(mdl, q_0, H_0_init, traj_select_fin, T_sim, T_horizon_MPC, param_global);
    eval(param_MPC_traj_data_name+"=param_trajectory;");
    save(param_MPC_traj_data_mat_file, param_MPC_traj_data_name);
end
%% OPT PROBLEM
%[TODO: oben init]
s_fun_path = './s_functions/'; % backslash on end necessary! [TODO: up]
casadi_fun_c_header_str = [casadi_func_name, '.c'];
casadi_fun_h_header_str = [casadi_func_name, '.h'];
s_func_name = ['s_function_', casadi_fun_c_header_str]; % final name for Simulink s-function
s_fun_c_file_path = [s_fun_path, s_func_name];
casadi_fun_h_header_path = [s_fun_path, casadi_func_name, '.h'];
casadi_fun_c_header_path = [s_fun_path, casadi_func_name, '.c'];

%% DEFINE OPTIMIZATION PROBLEM
if(strcmp(MPC_variant, 'opti'))
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

HH_e_test = arrayfun(@(u) hom_transform_endeffector(x_init_guess(1:n,u), param_robot), 1:N_MPC+1, UniformOutput=false);
HH_e_test_arr = cell2mat(HH_e_test);
p_e_test_arr = HH_e_test_arr(1:2,4:4:(N_MPC+1)*4)';

if(plot_init_guess)
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

if(fullsimu)
    matlab_sfun = str2func(casadi_func_name);
    
    %load("./s_functions/trajectory_data/"+"param_"+casadi_func_name+"_traj_data.mat")
    %param_trajectory = eval("param_"+casadi_func_name+"_traj_data");
    %load('traj_data.mat');

    N_traj = length(param_trajectory.t);
    N_traj_original = N_traj - N_MPC*N_step_MPC;
    p_e_act_arr = zeros(2, N_traj_original);
    u_k_new_arr = zeros(n, N_traj_original);
    x_k_new_arr = zeros(2*n, N_traj_original);
    x_k_new = x_0_0;
    u_k_act = u_k_0;

    % set additional needed N samples as last sample of trajectory
    % already done (see "parameter.m"):
    % traj_data = [param_trajectory.p_d; repmat(param_trajectory.p_d(end,:), N,1)];
    
    x_init_guess_prev = x_init_guess;
    u_init_guess_prev = u_init_guess;
    tic
    for i=1:1:N_traj_original
        y_ref_init = param_trajectory.p_d(i+N_step_MPC:N_step_MPC:i+N_MPC*N_step_MPC,1:2)';
        
        % Slow version
        %{
        opti.set_value(y_ref,y_ref_init); % e.g stay at same point
        opti.set_value(u_km1, u_init_guess_test(:,1)); % nicht ganz korrekt
        opti.set_value(x_k,x_k_new); % p,x0
    
        sol = opti.solve();
        
        u_init_guess = full(sol.value(u));
        x_init_guess = full(sol.value(x));
        %}

        % did not work:
        %sys = s_function_qrqp_N_5(0, x_k_new, {y_ref_init, x_k_new, u_k_act, x_init_guess, u_init_guess, QQ, RR_u, RR_du, RR_dx, RR_dx_km1, xx_min, xx_max, uu_min, uu_max}, 0);
        try
            [u_opt, x_init_guess, u_init_guess, Jy_act, Ju_act, Jukm1_act, Jdx_act, Jdxkm1_act] = matlab_sfun(y_ref_init, x_k_new, u_k_act, x_init_guess, u_init_guess, QQ, RR_u, RR_du, RR_dx, RR_dx_km1, xx_min, xx_max, uu_min, uu_max);
        catch ME
            disp("error at " + (i-1)*Ts_MPC + " s ( i="+i+")")
        end

        u_k_new = u_init_guess(:,1);
        x_k_new_sim = full(sim(x_init_guess(:,1), u_k_new));
        x_k_new = x_k_new_sim(:,1);
        %x_k_new = x_init_guess(:,2); % ist nicht gleich wie obere zeile???? Fehler 10^-6
    
        HH_e_act = hom_transform_endeffector(x_init_guess(:,1), param_robot);
        p_e_act = HH_e_act(1:2,4);
        p_e_act_arr(:, i) = p_e_act;
        u_k_new_arr(:, i) = u_k_new;
        x_k_new_arr(:, i) = x_k_new;
    end
    disp(['full simu execution time: ', num2str(toc), ' s'])
    x_init_guess = x_init_guess_prev;
    u_init_guess = u_init_guess_prev;

    figure;
    subplot(3,1,1);
    plot(Ts_MPC*(1:1:N_traj_original), u_k_new_arr);
    title('u');
    subplot(3,1,2);
    plot(Ts_MPC*(1:1:N_traj_original), x_k_new_arr);
    title('x');
    subplot(3,1,3);
    plot(Ts_MPC*(1:1:N_traj_original), p_e_act_arr);
    title('p_e');
    
end
%% TODO: Save data (besser gleich in parameter speichern)

param_MPC = struct( ...
  'x_init_guess', x_init_guess, ...
  'u_init_guess', u_init_guess, ...
  'y_init_guess', y_ref_0, ...
  'N', N_MPC, ...
  'N_step', N_step_MPC, ...
  'Ts', Ts_MPC, ...
  'T_horizon', T_horizon_MPC, ...
  'rk_iter', rk_iter, ...
  'variant', param_casadi_fun_struct.variant, ...
  'solver', param_casadi_fun_struct.solver, ...
  'name', param_casadi_fun_struct.name, ...
  'terminal_ineq_yref_N', param_casadi_fun_struct.terminal_ineq_yref_N, ...
  'terminal_soft_yref_N', param_casadi_fun_struct.terminal_soft_yref_N ...
);

eval(init_guess_struct_name+"=param_MPC;"); % set new struct name
save(""+init_guess_path+init_guess_struct_name+'.mat', init_guess_struct_name);
%T_traj_poly_old = 
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
T_traj_poly_old         = param_traj_poly.T        ;
T_traj_sin_poly_old     = param_traj_sin_poly.T    ;
omega_traj_sin_poly_old = param_traj_sin_poly.omega;
phi_traj_sin_poly_old   = param_traj_sin_poly.phi  ;
T_switch_old = param_traj_allg.T_switch;

save(param_MPC_old_data_file, 'q_0_old', 'q_0_p_old', 'xe0_old', 'xeT_old', ...
     'lamda_alpha_old', 'lamda_xyz_old', 'T_sim_old', 'traj_select_fin_old', ...
     'Ta_old', 'N_MPC_old', 'Ts_MPC_old', 'T_traj_poly_old', ...
     'T_traj_sin_poly_old', 'omega_traj_sin_poly_old', 'phi_traj_sin_poly_old' , ...
     'T_switch_old');

%% COMPILE matlab s_function

if(compile_matlab_sfunction)
    tic;
    casadi_fun_to_mex(f_opt, 's_functions', '-O2');
    disp(['Compile time for matlab s-function: ', num2str(toc), ' s']);
    delete(casadi_fun_c_header_path);
end