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

y_d_0 = [p_d_0; q_d_0];

x_init_guess_0 = [q_0(n_indices) q_1(n_indices)];

lam_x_init_guess_0 = zeros(numel(x_init_guess_0)+1, 1);
% lam_g_init_guess_0 = zeros(1, 1);
lam_g_init_guess_0 = [];

epsilon_min = 0.1;
epsilon_max = norm( param_robot.q_limit_upper - param_robot.q_limit_lower, 2);

init_guess_0 = [x_init_guess_0(:); epsilon_max; lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

% Optimization Variables:
x      = SX.sym( 'x',  n_red, 2 );
epsilon = SX.sym( 'epsilon', 1, 1 );

mpc_opt_var_inputs = {x, epsilon};

u_opt_indices = [1:2*n_red];

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
ubw = [repmat(param_robot.q_limit_upper(n_indices), size(x, 2), 1); epsilon_max];
lbw = [repmat(param_robot.q_limit_lower(n_indices), size(x, 2), 1); epsilon_min];

% input parameter
y_d = SX.sym( 'y_d',     7, 1 );
Qt_1 = SX.sym( 'Qt_1',   3, 3 );
Qr_1 = SX.sym( 'Qr_1',   3, 3 );
Qt_2 = SX.sym( 'Qt_2',   3, 3 );
Qr_2 = SX.sym( 'Qr_2',   3, 3 );
Qt_3 = SX.sym( 'Qt_3',   3, 3 );
Qr_3 = SX.sym( 'Qr_3',   3, 3 );
Q4 = SX.sym( 'Q4',  n_red, n_red );
Q5 = SX.sym( 'Q5',  n_red, n_red );
q1_manip = SX.sym( 'q1_manip', 1, 1 );
q2_manip = SX.sym( 'q2_manip', 1, 1 );
dq_eps = SX.sym( 'dq_eps', 1, 1 );
q_eps = SX.sym( 'q_eps', 1, 1 );

Qt_1_ref = 1e2*diag([1, 1, 1]);
Qr_1_ref = 1e0*diag([1, 1, 1]);
Qt_2_ref = 1e2*diag([1, 1, 1]);
Qr_2_ref = 1e0*diag([1, 1, 1]);
Qt_3_ref = 1e8*diag([1, 1, 1]);
Qr_3_ref = 1e8*diag([1, 1, 1]);
Q4_ref = 1e1*eye(n_red);
Q5_ref = 1e1*eye(n_red);
q1_manip_ref = 1e4;
q2_manip_ref = 1e4;
dq_eps_ref = 1e2;
q_eps_ref = 1e4;

mpc_parameter_inputs = {y_d, Qt_1, Qr_1, Qt_2, Qr_2, Qt_3, Qr_3, Q4, Q5, q1_manip, q2_manip, dq_eps, q_eps};
mpc_init_reference_values = [y_d_0(:); Qt_1_ref(:); Qr_1_ref(:); Qt_2_ref(:); Qr_2_ref(:); Qt_3_ref(:); Qr_3_ref(:); Q4_ref(:); Q5_ref(:); q1_manip_ref; q2_manip_ref; dq_eps_ref; q_eps_ref];

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

for i=0:1
    q = x(:, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(q);
    % y(4:7,   1 + (i)) = rotm2quat_v4_casadi(R_e);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);
end

q_d_0 = quat_fun_red(q_0(n_indices));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Total number of equation conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% g = {norm_2(x(:, 1) - x(:, 2))};
g = {[]};

pp = struct;

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

J1_t = Q_norm_square(y(1:3, 1 + (0)) - y_d(1:3), Qt_1);
J2_t = Q_norm_square(y(1:3, 1 + (1)) - y_d(1:3), Qt_2);
J3_t = Q_norm_square(y(1:3, 1 + (1)) - y(1:3, 1 + (0)), Qt_3);

q_y_yr_err1 = quat_mult(y(4:7, 1 + (0)), quat_inv(y_d(4:7)));
q_y_yr_err2 = quat_mult(y(4:7, 1 + (1)), quat_inv(y_d(4:7)));
q_y1_y2_err = quat_mult(y(4:7, 1 + (0)), quat_inv(y(4:7, 1 + (1))));
J1_r = Q_norm_square(q_y_yr_err1(2:4), Qr_1);
J2_r = Q_norm_square(q_y_yr_err2(2:4), Qr_2);
J3_r = Q_norm_square(q_y1_y2_err(2:4), Qr_3);

% dR1 = R_e_arr{1 + (0)}' * R_d_0 - R_d_0' * R_e_arr{1 + (0)};
% dR2 = R_e_arr{1 + (1)}' * R_d_0 - R_d_0' * R_e_arr{1 + (1)};
% dR3 = R_e_arr{1 + (0)}' * R_e_arr{1 + (1)} - R_e_arr{1 + (1)}' * R_e_arr{1 + (0)};
% J1_r = Q_norm_square([dR1(3,2); dR1(1,3); dR1(2,1)], Qr_1);
% J2_r = Q_norm_square([dR2(3,2); dR2(1,3); dR2(2,1)], Qr_2);
% J3_r = Q_norm_square([dR3(3,2); dR3(1,3); dR3(2,1)], Qr_3);

% J4 = -epsilon;
J4 = Q_norm_square(x(:, 1), Q4);
J5 = Q_norm_square(x(:, 2), Q5);

J6 = q1_manip*sqrt(det(J_red(x(:, 1))*J_red(x(:, 1))'));

% R_y_yr = R_e_arr{1 + (0)}' * R_e_arr{1 + (1)} - R_e_arr{1 + (1)}' * R_e_arr{1 + (0)};
% % q_y1_y2_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)];
% q_y1_y2_err = [1; R_y_yr(3,2); R_y_yr(1,3); R_y_yr(2,1)];
% J3_r = Q_norm_square(q_y1_y2_err(2:4), Qr_3);

dJ_eps = dq_eps*(norm_2(x(:, 1) - x(:, 2)) - epsilon)^2;
J_eps = q_eps*(epsilon_max - epsilon)^2;

cost_vars_names = '{J1_t, J1_r, J2_t, J2_r, J3_t, J3_r, J4, J5, J6, dJ_eps, J_eps}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);