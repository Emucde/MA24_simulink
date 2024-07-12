% MPC v4: Optimization problem 

import casadi.*

n = param_robot.n_DOF; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

[~, ~, Q] = quat_deriv(ones(4,1), ones(3,1), ones(3,1)); % get function handle

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.

% du = SX.sym('du', 2*n);
% dx = SX.sym('dx', 2*n); % x = [q1, ... qn, dq1, ... dqn]
% f = Function('f', {dx, du}, {dx});
% F = integrate_casadi(f, DT, M, int_method);

% Discrete yt_ref system
x = SX.sym('x', 2*n);
u = SX.sym('u', n);

% integrator for x
f = Function('f', {x, u}, {[x(n+1:2*n); u]});
F = integrate_casadi(f, DT, M, int_method);


%% Calculate Initial Guess

% Get trajectory data for initial guess
p_d_0    = param_trajectory.p_d(    1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_0 ... y_N)
p_d_p_0  = param_trajectory.p_d_p(  1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_p_0 ... y_p_N)
p_d_pp_0 = param_trajectory.p_d_pp( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_pp_0 ... y_pp_N)

q_d_0       = param_trajectory.q_d(       1:4, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_0 ... q_N)
omega_d_0   = param_trajectory.omega_d(   1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (omega_0 ... omega_N)
omega_d_p_0 = param_trajectory.omega_d_p( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (omega_p_0 ... omega_p_N)

% initial guess for reference trajectory
y_d_0    = [p_d_0;    q_d_0   ];
y_d_p_0  = [p_d_p_0;  omega_d_0 ];
y_d_pp_0 = [p_d_pp_0; omega_d_p_0];

% Robot System: Initial guess

% einige probleme beim simulieren [TODO]

x_0_0  = [q_0; q_0_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xd compute_tau_fun(q_0, dq_0, ddq_0); % gravity compensationof.m
q_0    = x_0_0(1   :   n); % useless line...
dq_0   = x_0_0(1+n : 2*n);
ddq_0  = q_0_pp;
xe_k_0 = p_d_0(1:3, 1); % x pos, y pos, defined in parameters_xdof.m
u_k_0  = ddq_0;

u_init_guess_0 = ones(n, N_MPC).*u_k_0;
x_init_guess_0 = [x_0_0 ones(2*n, N_MPC).*x_0_0];

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0), 1);

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

if(any(isnan(full(init_guess_0))))
    error('init_guess_0 contains NaN values!');
end

% get weights from "init_MPC_weight.m"
param_weight_init = param_weight.(casadi_func_name);

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init); % Matrizen sind keine Diagonlmatrizen [TODO]
else % hardcoded weights
    pp = param_weight_init;
end

% Optimization Variables:
u     = SX.sym( 'u',  n,   N_MPC   ); % u = q_0_pp
x     = SX.sym( 'x',  2*n, N_MPC+1 );

mpc_opt_var_inputs = {u, x};

% u = [u0; u1; ... uN-1], x = [x0; x1; ... xN] = [q_0;q_0_p;q_1;q_1_p;...;q_N;q_N_p]
% xx = [u; x] = [u0; u1; ... uN-1; x0; x1; ... xN]
%   u1 = q_1_pp = u( n+1:2+n)   |   q_1 = x(     1+2*n:    3*n),  |   q_1_p = x(     1+3*n:    4*n)
%   u1 = q_1_pp = xx(n+1:2+n)   |   q_1 = xx(N_u+1+2*n:N_u+3*n),  |   q_1_p = xx(N_u+1+3*n:N_u+4*n)
N_u = numel(u);
u_opt_indices = [1+2*n+N_u:N_u+3*n, 1+3*n+N_u:N_u+4*n, 1+n:2*n]; % [q_1, q_1_p, q_1_pp] needed for joint space CT control

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min, size(u, 2), 1); repmat(pp.x_min, N_MPC + 1, 1)];
ubw = [repmat(pp.u_max, size(u, 2), 1); repmat(pp.x_max, N_MPC + 1, 1)];

% input parameter
x_k  = SX.sym( 'x_k',  2*n, 1 ); % current x state = initial x state

y_d    = SX.sym( 'y_d',    m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d
y_d_p  = SX.sym( 'y_d_p',  m,   N_MPC+1 ); % (y_d_p_0 ... y_d_p_N)
y_d_pp = SX.sym( 'y_d_pp', m,   N_MPC+1 ); % (y_d_pp_0 ... y_d_pp_N)

mpc_parameter_inputs = {x_k, y_d, y_d_p, y_d_pp};
mpc_init_reference_values = [x_0_0(:); y_d_0(:); y_d_p_0(:); y_d_pp_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+1); % for F

if(weights_and_limits_as_parameter)
    lbg = SX(numel(x), 1);
    ubg = SX(numel(x), 1);
else
    lbg = zeros(numel(x), 1);
    ubg = zeros(numel(x), 1);
end

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y    = SX( 3, N_MPC+1 ); % transl. TCP position:      (y_0 ... y_N)

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_x(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk

for i=0:N_MPC
    % calculate q (q_0 ... q_N) and q_p values (q_p_0 ... q_p_N)
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_py_fun(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        g_x(1, 1 + (i+1))  = { F(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state dynamics constraints
    end
end

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

J_yt = Q_norm_square( y(1:3, 1 + (1:N_MPC-1) ) - y_d(1:3, 1 + (1:N_MPC-1)), pp.Q_y(1:3,1:3)  );
J_yt_N = Q_norm_square( y(1:3, 1 + (N_MPC) ) - y_d(1:3, 1 + (N_MPC)), pp.Q_yN(1:3,1:3)  );

J_yr = SX(1,1);
for i=1:N_MPC
    R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))';
    %q_y_y_err = rotation2quaternion_casadi( R_y_yr );
    q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)]; %ungenau aber schneller (flipping?)

    if(i < N_MPC)
        J_yr = J_yr + Q_norm_square( q_y_yr_err(2:4) , pp.Q_y(4:6,4:6)  );
    else
        J_yr_N = Q_norm_square( q_y_yr_err(2:4) , pp.Q_yN(4:6,4:6)  );
    end
end

g = g_x;

J_q_pp = Q_norm_square(u, pp.R_q_pp); %Q_norm_square(u, pp.R_u);

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);