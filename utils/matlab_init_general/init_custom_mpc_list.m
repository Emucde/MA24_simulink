function param_casadi_fun_name = init_custom_mpc_list()
% init_custom_mpc_list
%   This function initializes a structure containing the names and parameters
%   of various Model Predictive Control (MPC) configurations.
    
    param_casadi_fun_name = struct;
    
    MPC='MPC1_dyn_classic_N_MPC_3';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_y_u_MPC_v1'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 3;
    param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

    MPC='MPC1_dyn_classic_N_MPC_5';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_y_u_MPC_v1'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 5;
    param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)
    
    MPC='MPC2_dyn_parametric_N_MPC_3';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_y_u_MPC_v10_parametric'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 3;
    param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler

    MPC='MPC2_dyn_parametric_N_MPC_5';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_y_u_MPC_v10_parametric'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 5;
    param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler

    MPC='MPC2_dyn_parametric_N_MPC_10';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_y_u_MPC_v10_parametric'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 10;
    param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler
    
    MPC='MPC3_kin_int_thelenberg_N_MPC_5';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v4_kin_int'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 5;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

    MPC='MPC3_kin_int_thelenberg_N_MPC_10';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v4_kin_int'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 10;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

    MPC='MPC3_kin_int_thelenberg_N_MPC_15';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v4_kin_int'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 15;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

    MPC='MPC3_kin_int_thelenberg_N_MPC_20';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v4_kin_int'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 20;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

    MPC='MPC3_kin_int_thelenberg_N_MPC_30';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v4_kin_int'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 30;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

    MPC='MPC3_kin_int_parametric_N_MPC_5';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v9_parametric_kin_int'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 5;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

    MPC='MPC3_kin_int_parametric_N_MPC_10';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v9_parametric_kin_int'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 10;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

    MPC='MPC3_kin_int_parametric_N_MPC_15';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v9_parametric_kin_int'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 15;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

    MPC='MPC3_kin_int_parametric_N_MPC_20';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v9_parametric_kin_int'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 20;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

    MPC='MPC3_kin_int_parametric_N_MPC_30';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v9_parametric_kin_int'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 30;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)
    
    MPC='MPC4_kin_int_planner_N_MPC_5';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v7_kin_int_planner'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 5;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4  | SSPRK3 | Euler)
    
    MPC='MPC4_kin_int_planner_N_MPC_10';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v7_kin_int_planner'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 10;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4  | SSPRK3 | Euler)
    
    
    MPC='MPC4_kin_int_planner_N_MPC_15';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v7_kin_int_planner'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 15;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4  | SSPRK3 | Euler)
    
    
    MPC='MPC4_kin_int_planner_N_MPC_20';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v7_kin_int_planner'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 20;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4  | SSPRK3 | Euler)
    
    MPC='MPC4_kin_int_planner_N_MPC_30';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v7_kin_int_planner'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 30;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4  | SSPRK3 | Euler)
    

    MPC='MPC5_kin_dev_planner_N_MPC_10';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v8_kin_dev_planner'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 10;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4  | SSPRK3 | Euler)
    
    MPC='MPC5_kin_dev_planner_N_MPC_20';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v8_kin_dev_planner'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 20;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4  | SSPRK3 | Euler)
    
    MPC='MPC5_kin_dev_planner_N_MPC_30';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v8_kin_dev_planner'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 30;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4  | SSPRK3 | Euler)
end