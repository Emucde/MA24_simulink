% MPC v4: Optimization problem 

import casadi.*

casadi_func_name = 'find_singularities';

% if you have n_red < 6 it is much faster to neglect some positions - but it works also full poses.
yt_indices = param_robot.yt_indices;
yr_indices = param_robot.yr_indices;

n = param_robot.n_DOF; % Dimension of joint space
n_red = param_robot.n_red; % Dimension of reduced joint space
m = param_robot.m; % Dimension of Task Space

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);
J = Function.load([input_dir, 'geo_jacobian_endeffector_py.casadi']);

% N = 1; % OCP horizon
% ng = [1,0]; % Number of non-dynamic constraints, length N+1
% nu = [13,0]; % Number of controls, length N+1
% nx = [0,0]; % Number of states, length N+1

% build reduced model (fixed joint q3=0)
n_x_indices = param_robot.n_x_indices;
n_indices = param_robot.n_indices;
q_red = SX.sym( 'q',     n_red, 1 );
q_red_p  = SX.sym( 'q_p',   n_red, 1 );

q_0_ref = param_robot.q_0_ref;
q_subs    = SX(q_0_ref);
q_subs(n_indices) = q_red;

H_red = Function('H_red', {q_red}, {hom_transform_endeffector_py_fun(q_subs)});
quat_fun_red = Function('quat_fun_red', {q_red}, {quat_endeffector_py_fun(q_subs)});
J_red = Function('J_red', {q_red}, {J(q_subs)});
J_red_sx = J_red(q_red);
J_red = Function('J_red', {q_red}, {J_red_sx(:, n_indices)});

y_d_0 = [p_d_0; q_d_0];

q_1 = 2*pi*rand(n_red, 1);
q_1 = min(max(q_1, param_robot.q_limit_lower(n_indices)*q_scale_ref), param_robot.q_limit_upper(n_indices)*q_scale_ref);
x_init_guess_0 = [q_1];

lam_x_init_guess_0 = zeros(numel(x_init_guess_0), 1);
% lam_g_init_guess_0 = zeros(1, 1);
lam_g_init_guess_0 = [];

init_guess_0 = [x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

% Optimization Variables:
x = SX.sym( 'x',  n_red, 1 );

mpc_opt_var_inputs = {x};

u_opt_indices = [1:n_red];

% input parameter
q_manip = SX.sym( 'q1_manip', 1, 1 );
q_scale = SX.sym( 'q_scale', 1, 1 );
R_x = SX.sym( 'R_x', n_red, n_red );

q_manip_ref = 1e8;
q_scale_ref = 0.8;
R_x_ref = 0*eye(n_red);

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
q_max = param_robot.q_limit_upper(n_indices);
q_min = param_robot.q_limit_lower(n_indices);
q_n = param_robot.q_n(n_indices);
q_mean_up = (q_max - q_n);
q_mean_down = (q_min - q_n);
ubw = [repmat(q_n + q_mean_up*q_scale, size(x, 2), 1);];
lbw = [repmat(q_n + q_mean_down*q_scale, size(x, 2), 1);];

mpc_parameter_inputs = {R_x, q1_manip, q_scale};
mpc_init_reference_values = [R_x_ref(:); q1_manip_ref; q_scale_ref];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';

% lbg = epsilon;
% ubg = SX(inf);

lbg = [];
ubg = [];

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y       = SX(  7, 2); % TCP Pose:      (y_0 ... y_N)
R_e_arr = cell(1, 2);

q = x(:, 1 + (0));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Total number of equation conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% g = {norm_2(x(:, 1) - x(:, 2))};
g = {[]};

pp = struct;

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));


% J4 = -epsilon;
J1 = Q_norm_square(x(:, 1), R_x);
J2 = q1_manip*sqrt(det(J_red(x(:, 1))*J_red(x(:, 1))'));

cost_vars_names = '{J1, J2}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);