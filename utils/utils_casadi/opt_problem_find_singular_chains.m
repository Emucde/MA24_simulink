% opt_problem_find_singular_chains.m
% This function generates an optimization problem to find singular chains
% for a given robot configuration. It uses CasADi to define the problem and
% compiles the necessary functions for use in Simulink.

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

y_d_0 = [p_d_0; q_d_0];

N_points = 10;

epsilon_min = 0.1;
epsilon_max = norm( param_robot.q_limit_upper - param_robot.q_limit_lower, 2);

x_init_guess_0 = repmat(q_0_ref(n_indices), 1, N_points);
epsilon_init_guess_0 = epsilon_min*ones(N_points-1,1);

lam_x_init_guess_0 = zeros(numel(x_init_guess_0)+numel(epsilon_init_guess_0), 1);
% lam_g_init_guess_0 = zeros(1, 1);
lam_g_init_guess_0 = [];

init_guess_0 = [x_init_guess_0(:); epsilon_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

% Optimization Variables:
x      = SX.sym( 'x',  n_red, N_points );
epsilon = SX.sym( 'epsilon', 1, N_points-1 );

N_x = numel(x);
N_epsilon = numel(epsilon);

mpc_opt_var_inputs = {x, epsilon};

u_opt_indices = [1:2*n_red];

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
ubw = [repmat(param_robot.q_limit_upper(n_indices), size(x, 2), 1); repmat(epsilon_max, size(epsilon, 2), 1)];
lbw = [repmat(param_robot.q_limit_lower(n_indices), size(x, 2), 1); repmat(epsilon_min, size(epsilon, 2), 1)];

% input parameter
R_x = SX.sym( 'R_x',  n_red, n_red );
q_manip = SX.sym( 'q_manip', 1, 1 );
dq_eps = SX.sym( 'dq_eps', 1, 1 );
q_eps = SX.sym( 'q_eps', 1, 1 );

R_x_ref = 0*eye(n_red);
q_manip_ref = 1e4;
dq_eps_ref = 1e2;
q_eps_ref = 1e4;

mpc_parameter_inputs = {R_x, q_manip, dq_eps, q_eps};
mpc_init_reference_values = [R_x_ref(:); q_manip_ref; dq_eps_ref; q_eps_ref];

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Total number of equation conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% g = {norm_2(x(:, 1) - x(:, 2))};
g = {[]};

pp = struct;

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

J_x = Q_norm_square(x, R_x);

J_manip = 0;
for i=1:N_points
    J_manip = J_manip + q_manip*sqrt(det(J_red(x(:, i))*J_red(x(:, i))'));
end

J_eps = 0;
dJ_eps = 0;
for i=1:N_points-1
    
    J_eps = J_eps + q_eps*(epsilon_min - epsilon(i))^2;
    dJ_eps = dJ_eps + dq_eps*(norm_2(x(:, i) - x(:, i+1)) - epsilon(i))^2;
end

J_eps2 = 0;
dJ_eps2 = 0;
for i=1:N_points-1
    for j=2:N_points-1
        J_eps2 = J_eps2 + q_eps*(abs(epsilon(i) - epsilon(j)) - epsilon_min)^2;
    end
end

cost_vars_names = '{J_x, J_manip, dJ_eps, J_eps}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);