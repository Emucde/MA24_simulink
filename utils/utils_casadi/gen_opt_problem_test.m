import casadi.*

fprintf('Start execution of ''nlpsol_opt_problem_SX_v2.m''\n');

%% Define MPC Function
input_parameter_SX = mpc_parameter_inputs;
input_opt_var_SX = mpc_opt_var_inputs;
input_initial_guess_SX = [input_opt_var_SX(:)', {lambda_x0}, {lambda_g0}];

input_vars_SX  = horzcat(input_parameter_SX, input_initial_guess_SX);
output_values_SX = {J, vertcat(w{:}), vertcat(g{:}), vertcat(p{:}), lambda_x0, lambda_g0};

get_str_names_from_sx_cell;

input_min_len = length(input_vars_SX);

get_costs_MX = Function('get_costs_MX', input_vars_SX, cost_vars_SX);
get_limits_MX = Function('get_limits_MX', input_vars_SX, {lbw,   ubw,   lbg,   ubg});

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

[lbw_MX, ubw_MX, lbg_MX, ubg_MX] = get_limits_MX(input_vars_MX{:});

sol_sym = solver('x0', w_MX, 'p', p_MX, 'lbx', lbw_MX, 'ubx', ubw_MX,...
    'lbg', lbg_MX, 'ubg', ubg_MX, 'lam_x0', lambda_x0_MX, 'lam_g0', lambda_g0_MX);

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

    % cost_vars_len = length(cost_vars_SX);
    % output_vars_MX_ext = cell(1, cost_vars_len);
    % [output_vars_MX_ext{1:cost_vars_len}] = get_costs_MX(input_vars_MX{:});
    % output_vars_MX = horzcat(output_vars_MX, output_vars_MX_ext);

    % % So that all parameters can only be passed via a single input.
    % input_vars_MX = horzcat({input_vars_MX{1:input_min_len}}, merge_cell_arrays({input_vars_MX{input_min_len+1:end}}));

% Merge Input Parameter:
input_parameter_len = length(input_parameter_SX);
input_initial_guess_len = length(input_initial_guess_SX);
input_vars_MX = horzcat(merge_cell_arrays({input_vars_MX{1:input_parameter_len}}), ... 
                merge_cell_arrays({input_vars_MX{input_parameter_len+(1:input_initial_guess_len)}}), ...
                {input_vars_MX{input_parameter_len+input_initial_guess_len+1 : end}});

%% Define f_opt and calculate final initial guess values
f_opt = Function(casadi_func_name, input_vars_MX, output_vars_MX);

try
    [u_opt_sol, xx_full] = f_opt(mpc_init_reference_values, init_guess_0);
catch ME
    try
        solver.stats(1)
    end
    error(getReport(ME));
end
% solver.stats(1)

if(size(u_opt_sol, 2) > 1)
    result = reshape(full(u_opt_sol), n_red, 2);
    qq_sol = zeros(n, 2);
    qq_sol(n_indices, :) = result;

    disp(hom_transform_endeffector_py(qq_sol(:,1)));
    disp(hom_transform_endeffector_py(qq_sol(:,2)));
    JJ1 = geo_jacobian_endeffector_py(qq_sol(:,1));
    JJ2 = geo_jacobian_endeffector_py(qq_sol(:,2));
    w1 = sqrt(det(JJ1 * JJ1'));
    w2 = sqrt(det(JJ2 * JJ2'));
    disp(['manip=', num2str(w1), ', |q1-q2|=', num2str(norm(qq_sol(:,1)-qq_sol(:,2), 2))]);
    disp(qq_sol);
else
    result = full(u_opt_sol);
    qq_sol = zeros(n, 1);
    qq_sol(n_indices) = result;

    JJ = full(J_red(result));
    w = sqrt(det(JJ * JJ'));
    disp(['manip=', num2str(w)]);
    disp(qq_sol);
end