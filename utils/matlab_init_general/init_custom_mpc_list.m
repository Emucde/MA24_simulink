function param_casadi_fun_name = init_custom_mpc_list()
    
    param_casadi_fun_name = struct;
    
    MPC='MPC01_dyn_classic';
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

    MPC='MPC2_dyn_parametric';
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
    
    MPC='MPC3_dyn_parametric_quat_refsys';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_ineq_feasible_traj_MPC_v3_quat'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 5;
    param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)
    
    MPC='MPC4_dyn_parametric_rpy_refsys';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_ineq_feasible_traj_MPC_v3_rpy'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 5;
    param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)
    
    MPC='MPC5_kin_int_thelenberg';
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
    
    MPC='MPC6_kin_int_ref_sys';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v4_kin_int_refsys'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 5;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)
    
    MPC='MPC7_kin_dev';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v5_kin_dev'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 10;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % is ignored here
    
    MPC='MPC8_kin_int_path_following';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v6_kin_int_path_following'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 10;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)
    
    MPC='MPC9_kin_int_planner';
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
    
    MPC='MPC10_kin_dev_planner';
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
    
    MPC='MPC11_kin_dev_path_following';
    param_casadi_fun_name.(MPC).name    = MPC;
    param_casadi_fun_name.(MPC).variant = 'nlpsol';
    param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
    param_casadi_fun_name.(MPC).version = 'opt_problem_MPC_v6_kin_dev_path_following'; % see nlpso_generate_opt_problem.m
    param_casadi_fun_name.(MPC).Ts      = 5e-3;
    param_casadi_fun_name.(MPC).rk_iter = 1;
    param_casadi_fun_name.(MPC).N_MPC   = 10;
    param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
    param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
    param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)
    
    MPC='MPC12_kin_parametric';
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
end