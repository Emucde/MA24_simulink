% nlpsol_opt_problem_SX_v2.m
% It is used by mpc_casadi_main.m to generate the optimization problem for the MPC solver.
% It also compiles the CasADi s-function for the use of simulink.

import casadi.*

fprintf('Start execution of ''nlpsol_opt_problem_SX_v2.m''\n');

% Create optimization Problem (depending on MPC_version)
nlpsol_generate_opt_problem;

%% Define MPC Function
input_parameter_SX = mpc_parameter_inputs;
input_opt_var_SX = mpc_opt_var_inputs;
input_initial_guess_SX = [input_opt_var_SX(:)', {lambda_x0}, {lambda_g0}];

input_vars_SX  = horzcat(input_parameter_SX, input_initial_guess_SX);
output_values_SX = {J, vertcat(w{:}), vertcat(g{:}), vertcat(p{:}), lambda_x0, lambda_g0};

get_str_names_from_sx_cell;

if(weights_and_limits_as_parameter)
    input_min_len = length(input_vars_SX);

    input_vars_SX = horzcat(input_vars_SX, struct2cell(pp)');
    get_costs_MX = Function('get_costs_MX', input_vars_SX, cost_vars_SX);
    get_limits_MX = Function('get_limits_MX', input_vars_SX, {lbw,   ubw,   lbg,   ubg});
end

get_outputs_MX = Function('get_outputs_MX', input_vars_SX, output_values_SX);


% get SX prop struct (SX is much faster than MX!)
[J, W, G, P, ~, ~] = get_outputs_MX(input_vars_SX{:});
prob = struct('f', J, 'x', W, 'g', G, 'p', P);

% convert SX variables to MX (nlpsolve can only use MX variables)
input_vars_MX       = cellfun(@(x) sx_to_mx(x, 'get_MX_sym_cell' ), input_vars_SX, 'UniformOutput', false);
%input_vars_MX_names = cellfun(@(x) sx_to_mx(x, 'get_MX_name_cell'), input_vars_SX, 'UniformOutput', false);

%% Create an NLP solver

nlp_solver_settings;

%|--------------------------------------------------------------------------------------|
%| Full name     | Short  | Description                                                 |
%|--------------------------------------------------------------------------------------|
%| NLPSOL_X0     | x0     | Decision variables, initial guess (nx x 1)                  |
%| NLPSOL_P      | p      | Value of fixed parameters (np x 1)                          |
%| NLPSOL_LBX    | lbx    | Decision variables lower bound (nx x 1), default -inf.      |
%| NLPSOL_UBX    | ubx    | Decision variables upper bound (nx x 1), default +inf.      |
%| NLPSOL_LBG    | lbg    | Constraints lower bound (ng x 1), default -inf.             |
%| NLPSOL_UBG    | ubg    | Constraints upper bound (ng x 1), default +inf.             |
%| NLPSOL_LAM_X0 | lam_x0 | Lagrange multipliers for bounds on X, initial guess (nx x 1)|
%| NLPSOL_LAM_G0 | lam_g0 | Lagrange multipliers for bounds on G, initial guess (ng x 1)|
%|--------------------------------------------------------------------------------------|

% [J_MX, w_MX, g_MX, p_MX,  J_y_MX, J_y_p_MX, J_y_pp_MX, D_0_MX, D_1_MX, D_N_MX, lbw_MX, ubw_MX, lbg_MX, ubg_MX] = get_outputs_MX(input_vars_MX{:}); % früher
[J_MX, w_MX, ~, p_MX, lambda_x0_MX, lambda_g0_MX] = get_outputs_MX(input_vars_MX{:});

if(weights_and_limits_as_parameter)
    [lbw_MX, ubw_MX, lbg_MX, ubg_MX] = get_limits_MX(input_vars_MX{:});

    sol_sym = solver('x0', w_MX, 'p', p_MX, 'lbx', lbw_MX, 'ubx', ubw_MX,...
        'lbg', lbg_MX, 'ubg', ubg_MX, 'lam_x0', lambda_x0_MX, 'lam_g0', lambda_g0_MX);
else
    sol_sym = solver('x0', w_MX, 'p', p_MX, 'lbx', lbw, 'ubx', ubw,...
        'lbg', lbg, 'ubg', ubg, 'lam_x0', lambda_x0_MX, 'lam_g0', lambda_g0_MX);
end

%--------------------------------------------------------------------------------------|
%| Full name    | Short | Description                                                  |
%--------------------------------------------------------------------------------------|
%| NLPSOL_X     | x     | Decision variables at the optimal solution (nx x 1)          |
%| NLPSOL_F     | f     | Cost function value at the optimal solution (1 x 1)          |
%| NLPSOL_G     | g     | Constraints function at the optimal solution (ng x 1)        |
%| NLPSOL_LAM_X | lam_x | Lagrange multipliers for bounds on X at the solution (nx x 1)|
%| NLPSOL_LAM_G | lam_g | Lagrange multipliers for bounds on G at the solution (ng x 1)|
%| NLPSOL_LAM_P | lam_p | Lagrange multipliers for bounds on P at the solution (np x 1)|
%--------------------------------------------------------------------------------------|

% generate solver solutions variables
u_opt = sol_sym.x(u_opt_indices);

output_vars_MX = [{u_opt}, merge_cell_arrays({sol_sym.x, sol_sym.lam_x, sol_sym.lam_g})];
if(weights_and_limits_as_parameter)
    cost_vars_len = length(cost_vars_SX);
    output_vars_MX_ext = cell(1, cost_vars_len);
    [output_vars_MX_ext{1:cost_vars_len}] = get_costs_MX(input_vars_MX{:});
    output_vars_MX = horzcat(output_vars_MX, output_vars_MX_ext);

    % So that all parameters can only be passed via a single input.
    input_vars_MX = horzcat({input_vars_MX{1:input_min_len}}, merge_cell_arrays({input_vars_MX{input_min_len+1:end}}));
end

% Merge Input Parameter:
input_parameter_len = length(input_parameter_SX);
input_initial_guess_len = length(input_initial_guess_SX);
input_vars_MX = horzcat(merge_cell_arrays({input_vars_MX{1:input_parameter_len}}), ... 
                merge_cell_arrays({input_vars_MX{input_parameter_len+(1:input_initial_guess_len)}}), ...
                {input_vars_MX{input_parameter_len+input_initial_guess_len+1 : end}});

%% Define f_opt and calculate final initial guess values
f_opt = Function(casadi_func_name, input_vars_MX, output_vars_MX, f_opt_input_str, f_opt_output_str);

%init_MPC_weights;
param_weight_init = param_weight.(casadi_func_name);

try
    if(create_test_solve)
        if(weights_and_limits_as_parameter)
            % Hint: If the init_guess is already the global minima, the solver
            % will get an Evaluation Failed because it get's NaN values due to
            % the perfect zero gradients. Therefore add eps to init_guess to
            % avoid this problem.
            init_guess_0 = init_guess_0;
            param_weight_init_cell = merge_cell_arrays(struct2cell(param_weight_init), 'vector');
            [u_opt_sol, xx_full_opt_sol, cost_values_sol{1:length(cost_vars_SX)}] = f_opt(mpc_init_reference_values, init_guess_0, param_weight_init_cell);
        else  % ohne extra parameter 5 % schneller (319s statt 335s)
            [u_opt_sol, xx_full_opt_sol] = f_opt(mpc_init_reference_values, init_guess_0);
        end

        % x_full = full(reshape(xx_full_opt_sol(1:numel(x)), size(x)));
        % u_full = full(reshape(xx_full_opt_sol(1+numel(x):numel(x)+numel(u)), size(u)));

        u_full = full(reshape(xx_full_opt_sol(1:numel(u)), size(u)));
        disp(u_full);

        if(x.is_symbolic())
            x_full = full(reshape(xx_full_opt_sol(1+numel(u):numel(u)+numel(x)), size(x)));        
            disp(x_full);
        end

        % show stats

        %z_full = full(reshape(xx_full_opt_sol(1+numel(u)+numel(x):numel(u)+numel(x)+numel(z)), size(z)));
        % z_t_full = full(reshape(xx_full_opt_sol(1+numel(u)+numel(x):numel(u)+numel(x)+numel(zt)), size(zt)));
        % z_r_full = full(reshape(xx_full_opt_sol(1+numel(u)+numel(x)+numel(zt):numel(u)+numel(x)+numel(zt)+numel(z_qw)), size(z_qw)));
        % alpha_t = full(reshape(xx_full_opt_sol(1+numel(u)+numel(x)+numel(zt)+numel(z_qw):numel(u)+numel(x)+numel(zt)+numel(z_qw)+numel(zt(1:3,:))), size(zt(1:3,:))));
        % alpha_r = full(reshape(xx_full_opt_sol(1+numel(u)+numel(x)+numel(zt)+numel(z_qw)+numel(zt(1:3,:)):numel(u)+numel(x)+numel(zt)+numel(z_qw)+numel(zt(1:3,:))+numel(z_qw(1:3,:))), size(z_qw(1:3,:))));

        %z_d_init_guess_0; % vs z_full?
        %disp(solver.stats())

        % set init guess
        init_guess = full(xx_full_opt_sol);

        if(any(isnan(full(xx_full_opt_sol))))
            error('init_guess_0 contains NaN values!');
        end

        if(print_init_guess_cost_functions && weights_and_limits_as_parameter)
            disp(['J = ', num2str(full( sum([ cost_values_sol{:} ]) )) ]);
            for i=1:length(cost_vars_names_cell)
                disp([cost_vars_names_cell{i}, '= ', num2str(full( cost_values_sol{i} ))])
            end
        end
    else
        init_guess = init_guess_0;
    end
catch ME
    try
        solver.stats(1)
    end
    disp(getReport(ME));
    init_guess = init_guess_0;
end

uu_indices = reshape(1:numel(u), size(u));
xx_indices = reshape(1+numel(u):numel(u)+numel(x), size(x));

qq_indices_arr  = xx_indices(1:size(x,1)/2, :);
qqp_indices_arr = xx_indices(size(x,1)/2+1:end, :);

tau_indices = uu_indices(:)';
qq_indices = qq_indices_arr(:)';
qqp_indices = qqp_indices_arr(:)';

tau_dim = size(uu_indices);
qq_dim = size(qq_indices_arr);
qqp_dim = size(qqp_indices_arr);

if(strcmp(MPC_version, 'opt_problem_ineq_feasible_traj_MPC_v3_quat') || strcmp(MPC_version, 'opt_problem_ineq_feasible_traj_MPC_v3_rpy'))
    
    z_indices = reshape(1+numel(u)+numel(x):numel(u)+numel(x)+numel(z), size(z));
    alpha_indices = reshape(1+numel(u)+numel(x)+numel(z):numel(u)+numel(x)+numel(z)+numel(alpha), size(alpha));

    y_ref_indices_arr = z_indices(1:n_y, :); 
    y_p_ref_indices_arr = z_indices(n_y+1:end, :);
    y_pp_ref_indices_arr = alpha_indices;

    y_ref_indices_dim = size(y_ref_indices_arr);
    y_p_ref_indices_dim = size(y_p_ref_indices_arr);
    y_pp_ref_indices_dim = size(y_pp_ref_indices_arr);

    sol_indices = struct( ...
        'tau', tau_indices, ...
        'tau_dim', tau_dim, ...
        'qq', qq_indices, ...
        'qq_dim', qq_dim, ...
        'qqp', qqp_indices, ...
        'qqp_dim', qqp_dim, ...
        'y_ref', y_ref_indices_arr, ...
        'y_ref_dim', y_ref_indices_dim, ...
        'y_p_ref', y_p_ref_indices_arr, ...
        'y_p_ref_dim', y_p_ref_indices_dim, ...
        'y_pp_ref', y_pp_ref_indices_arr, ...
        'y_pp_ref_dim', y_pp_ref_indices_dim ...
        );
else
    sol_indices = struct( ...
        'tau', tau_indices, ...
        'tau_dim', tau_dim, ...
        'qq', qq_indices, ...
        'qq_dim', qq_dim, ...
        'qqp', qqp_indices, ...
        'qqp_dim', qqp_dim ...
        );
end

%% COMPILE (nlpsol)

if(compile_sfun)
    if(compile_mode == 1)
        tic;
        compile_casadi_sfunction(f_opt, [s_fun_path, '/'], output_dir, MPC_solver, coptimflags, compile_mode, remove_sourcefiles, use_jit); % default nlpsol s-function
        disp(['Compile time for casadi s-function (nlpsol): ', num2str(toc), ' s']);
    elseif(compile_mode == 2)
        tic;
        compile_casadi_sfunction(f_opt, [s_fun_path, '/'], output_dir, MPC_solver, coptimflags, compile_mode, remove_sourcefiles, use_jit); % default opti s-function
        disp(['Compile time for casadi s-function (opti for nlpsol): ', num2str(toc), ' s']);
    end
end

%% s-function in Simulink: Name of s-function: MPC1
% 1) Insert S-Function Block in Matlab
% 2) add 
%        's_functions/MPC1.casadi', 'MPC1' 
%    to S-function parameters
% 2) add 's_function_MPC1' to S-function name
% 3) set S-function modules to ''
% or better see the output of 'compile_casadi_sfunction.m'

fprintf('\nExecution of ''nlpsol_opt_problem_SX_v2.m'' finished\n');
fprintf('--------------------------------------------------------------------\n\n');