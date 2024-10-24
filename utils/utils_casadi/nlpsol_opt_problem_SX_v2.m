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

%|-------------------|
%| List of plugins   |
%|-------------------|
%| AmplInterface     |
%| blocksqp          |
%| bonmin            |
%| ipopt             |
%| knitro            |
%| snopt             |
%| worhp             |
%| feasiblesqpmethod |
%| qrsqp             |
%| scpgen            |
%| sqpmethod         |
%|-------------------|

tic;
fprintf('\nStarting execution of solver = nlpsol(''solver'', ''sqpmethod'', prob, opts) (~105s)\n')
if(strcmp(MPC_solver, 'qrqp'))
    % DOKU: https://casadi.sourceforge.net/v2.0.0/api/html/d6/d07/classcasadi_1_1NlpSolver.html
    opts = struct; % Create a new structure
    opts.qpsol = 'qrqp'; % Set the QP solver to 'qrqp'
    opts.qpsol_options.print_iter = false; % Disable printing of QP solver iterations
    opts.qpsol_options.print_header = false; % Disable printing of QP solver header
    opts.qpsol_options.print_info = false; % Disable printing of QP solver info
    opts.qpsol_options.error_on_fail = false;
    
    opts.print_header = false; % Disable printing of solver header
    opts.print_iteration = false; % Disable printing of solver iterations
    % opts.print_iteration = true; % Disable printing of solver iterations
    opts.print_time = false; % Disable printing of solver time
    opts.print_status = false;
    opts.error_on_fail = false;
    opts.hessian_approximation = 'exact';
    % opts.iteration_callback_ignore_errors = true;
    % opts.elastic_mode = true;
    % opts.regularity_check = false;
    opts.show_eval_warnings = false;

    %opts.tol_du=1e-12;%1e-6;
    %opts.tol_pr=1e-12;%1e-6;
%
    %opts.max_iter = 10000;%1500;
    
    solver = nlpsol('solver', 'sqpmethod', prob, opts);

    % solver.print_options();
elseif(strcmp(MPC_solver, 'feasiblesqpmethod'))
    opts = struct; % Create a new structure
    opts.qpsol = 'qrqp'; % Set the QP solver to 'qrqp'
    opts.qpsol_options.print_iter = false; % Disable printing of QP solver iterations
    opts.qpsol_options.print_header = false; % Disable printing of QP solver header
    opts.qpsol_options.print_info = false; % Disable printing of QP solver info
    opts.qpsol_options.error_on_fail = false;
    
    opts.print_header = false; % Disable printing of solver header
    opts.print_iteration = false; % Disable printing of solver iterations
    opts.print_time = false; % Disable printing of solver time
    opts.print_status = false;
    opts.error_on_fail = false;
    opts.hessian_approximation = 'exact';
    % opts.iteration_callback_ignore_errors = true;
    % opts.elastic_mode = true;
    % opts.regularity_check = false;
    opts.show_eval_warnings = false;
    % opts.max_iter = 1000;
    opts.tol_du=1e-6;
    opts.tol_pr=1e-6;
    solver = nlpsol('solver', 'feasiblesqpmethod', prob, opts);
    % solver.print_options();
elseif(strcmp(MPC_solver, 'qpoases'))
    % DOKU: https://casadi.sourceforge.net/api/internal/d5/d43/classcasadi_1_1QpoasesInterface.html
    opts = struct; % Create a new structure
    opts.qpsol = 'qpoases'; % Set the QP solver to 'qrqp'
    opts.qpsol_options.error_on_fail = false;
    opts.qpsol_options.printLevel = 'none';
    opts.print_header = false; % Disable printing of solver header
    opts.print_iteration = false; % Disable printing of solver iterations
    opts.print_time = false; % Disable printing of solver time
    opts.print_status = false;
    opts.error_on_fail = false;
    opts.hessian_approximation = 'exact';
    % opts.iteration_callback_ignore_errors = true;
    % opts.elastic_mode = true;
    % opts.regularity_check = false;
    opts.show_eval_warnings = false;
    opts.verbose_init = false;
    % opts.max_iter = 10;
    % opts.tol_du=1e-3;
    % opts.tol_pr=1e-3;
    solver = nlpsol('solver', 'sqpmethod', prob, opts);
    % solver.print_options();
elseif(strcmp(MPC_solver, 'ipopt'))
    % DOKU: https://casadi.sourceforge.net/v2.0.0/api/html/d6/d07/classcasadi_1_1NlpSolver.html#plugin_NlpSolver_ipopt
    opts = struct;
    opts.show_eval_warnings = false;
    opts.error_on_fail = false;
    opts.print_time = 0;

    all_ipopt_options;
    opts.ipopt.print_level = 5;
    opts.ipopt.fast_step_computation = 'yes';
    opts.ipopt.print_info_string = 'yes';
    %opts.jit = true;
    %opts.compiler = 'shell';
    %opts.jit_options.flags = {'-O2'};
    %opts.jit_options.verbose = true;
    solver = nlpsol('solver', 'ipopt', prob, opts);
    %solver.print_options();
else
    error(['invalid MPC solver=', MPC_solver, ...
           ' is a valid solver for nlpsol (only "qrqp", "ipopt", "qpoases", ... supported for nlpsol)']);
end
disp(['Execution time of solver = nlpsol(''solver'', ''sqpmethod'', prob, opts): ', num2str(toc), ' s']);
fprintf('\n');

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
    if(weights_and_limits_as_parameter)
        % Hint: If the init_guess is already the global minima, the solver
        % will get an Evaluation Failed because it get's NaN values due to
        % the perfect zero gradients. Therefore add eps to init_guess to
        % avoid this problem.
        init_guess_0 = eps + init_guess_0;
        param_weight_init_cell = merge_cell_arrays(struct2cell(param_weight_init), 'vector');
        [u_opt_sol, xx_full_opt_sol, cost_values_sol{1:length(cost_vars_SX)}] = f_opt(mpc_init_reference_values, init_guess_0, param_weight_init_cell);
    else  % ohne extra parameter 5 % schneller (319s statt 335s)
        [u_opt_sol, xx_full_opt_sol] = f_opt(mpc_init_reference_values, init_guess_0);
    end
catch ME
    try
        solver.stats(1)
    end
    error(getReport(ME));
end

% x_full = full(reshape(xx_full_opt_sol(1:numel(x)), size(x)));
% u_full = full(reshape(xx_full_opt_sol(1+numel(x):numel(x)+numel(u)), size(u)));

u_full = full(reshape(xx_full_opt_sol(1:numel(u)), size(u)));
x_full = full(reshape(xx_full_opt_sol(1+numel(u):numel(u)+numel(x)), size(x)));

disp(u_full);
disp(x_full);

% show stats

%z_full = full(reshape(xx_full_opt_sol(1+numel(u)+numel(x):numel(u)+numel(x)+numel(z)), size(z)));
% z_t_full = full(reshape(xx_full_opt_sol(1+numel(u)+numel(x):numel(u)+numel(x)+numel(zt)), size(zt)));
% z_r_full = full(reshape(xx_full_opt_sol(1+numel(u)+numel(x)+numel(zt):numel(u)+numel(x)+numel(zt)+numel(z_qw)), size(z_qw)));
% alpha_t = full(reshape(xx_full_opt_sol(1+numel(u)+numel(x)+numel(zt)+numel(z_qw):numel(u)+numel(x)+numel(zt)+numel(z_qw)+numel(zt(1:3,:))), size(zt(1:3,:))));
% alpha_r = full(reshape(xx_full_opt_sol(1+numel(u)+numel(x)+numel(zt)+numel(z_qw)+numel(zt(1:3,:)):numel(u)+numel(x)+numel(zt)+numel(z_qw)+numel(zt(1:3,:))+numel(z_qw(1:3,:))), size(z_qw(1:3,:))));
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

if(strcmp(MPC_version, 'v3_quat') || strcmp(MPC_version, 'v3_rpy'))
    
    z_indices = reshape(1+numel(u)+numel(x):numel(u)+numel(x)+numel(z), size(z));
    alpha_indices = reshape(1+numel(u)+numel(x)+numel(z):numel(u)+numel(x)+numel(z)+numel(alpha), size(alpha));

    pp_indices_arr = [z_indices(1:2*n_yt_red, :); alpha_indices(1:n_yt_red, :)];
    rr_indices_arr = [z_indices(2*n_yt_red+1:end, :); alpha_indices(n_yt_red+1:end, :)];

    pp_indices = pp_indices_arr(:)';
    rr_indices = rr_indices_arr(:)';

    pp_dim = size(pp_indices_arr);
    rr_dim = size(rr_indices_arr);

    sol_indices = struct( ...
        'tau', tau_indices, ...
        'tau_dim', tau_dim, ...
        'qq', qq_indices, ...
        'qq_dim', qq_dim, ...
        'qqp', qqp_indices, ...
        'qqp_dim', qqp_dim, ...
        'pp', pp_indices, ...
        'pp_dim', pp_dim, ...
        'rr', rr_indices, ...
        'rr_dim', rr_dim ...
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


%z_d_init_guess_0; % vs z_full?
%disp(solver.stats())

% set init guess
init_guess = full(xx_full_opt_sol)+eps;


if(print_init_guess_cost_functions && weights_and_limits_as_parameter)
    disp(['J = ', num2str(full( sum([ cost_values_sol{:} ]) )) ]);
    for i=1:length(cost_vars_names_cell)
        disp([cost_vars_names_cell{i}, '= ', num2str(full( cost_values_sol{i} ))])
    end
end

%% COMPILE (nlpsol)
if(compile_sfun)
    if(compile_mode == 1)
        tic;
        compile_casadi_sfunction(f_opt, [s_fun_path, '/'], output_dir, MPC_solver, '-O3', compile_mode, remove_sourcefiles); % default nlpsol s-function
        disp(['Compile time for casadi s-function (nlpsol): ', num2str(toc), ' s']);
    elseif(compile_mode == 2)
        tic;
        compile_casadi_sfunction(f_opt, [s_fun_path, '/'], output_dir, MPC_solver, '-O2', compile_mode, remove_sourcefiles); % default opti s-function
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