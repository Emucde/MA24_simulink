% MPC v1: Optimization problem 
% States and references
% - x_k: Current state
% - y_n: Predicted TCP Position
% - y_n_ref: Reference values for TCP position

% Cost function (J_d,N)
% Minimizes the deviation between predicted outputs and references over a prediction horizon (N)
% Classic: J = (y - y_ref)_Q_y

% Control Law
% Optimal control input sequence (u_n) minimizes the cost function

% Constraints
% - System dynamics enforced through state equation (F)
% - Initial state set to current state (x_k)
% - Output (y_n) calculated from state (q_n) using output function (h)

% Note: Simplified notation used for clarity. Define matrices and functions explicitly in implementation.

import casadi.*

n = param_robot.n_DOF;

%% Calculate Initial Guess
q_init_guess_0 = ones(2,N_MPC+1).*q_0;
y_ref_0            = param_trajectory.p_d(    1:2, 1 : 1 : 1 + N_MPC  ); % (y_1 ... y_N)

lam_x_init_guess_0 = zeros(numel(q_init_guess_0) + numel(y_ref_0), 1);
lam_g_init_guess_0 = zeros(numel(y_ref_0), 1);

init_guess_0       = [q_init_guess_0(:); y_ref_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

% get weights from "init_MPC_weight.m"
param_weight_init = param_weight.(casadi_func_name);

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init);
else % hardcoded weights
    pp = param_weight_init;
end

%% Start with an empty NLP

% Optimization Variables:
q = SX.sym( 'q', n, N_MPC+1 );
y = SX.sym( 'y', 2, N_MPC+1 );

mpc_opt_var_inputs = {q, y};

u_opt_indices = numel(q) + (1:numel(y));

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.q_min, N_MPC + 1, 1); -Inf(size(y(:)));];
ubw = [repmat(pp.q_max, N_MPC + 1, 1);  Inf(size(y(:)));];

% input parameter
y_ref    = SX.sym( 'y_ref',    2,   N_MPC+1  ); % (y_ref_1 ... y_ref_N)
mpc_parameter_inputs = {y_ref};
mpc_init_reference_values = [y_ref_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g = cell(1, N_MPC+1);
lbg = zeros(numel(y), 1);
ubg = zeros(numel(y), 1)+1e-3;

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));


for i=0:N_MPC

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_casadi_SX(q(:, 1 + (i)), param_robot);
    g(1,    1 + (i+1)) = {y(:, 1 + (i)) - H_e(1:2, 4)}; % Set the state dynamics constraints
end

Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

J_y = Q_norm_square( y - y_ref, eye(2)  );

cost_vars_names = '{J_y}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);