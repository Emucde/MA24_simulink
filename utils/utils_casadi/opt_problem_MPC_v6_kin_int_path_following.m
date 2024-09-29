% MPC v4: Optimization problem 

import casadi.*

n = param_robot.n_DOF; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

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
theta = SX.sym('theta', 1);
v = SX.sym('v', 1);

% integrator for x
f = Function('f', {x, u}, {[x(n+1:2*n); u]});
lambda = 1;
h = Function('h', {theta, v}, {-lambda*theta + v});
F = integrate_casadi(f, DT, M, int_method); % runs with Ts_MPC
H = integrate_casadi(h, DT, M, int_method); % runs with Ts_MPC

DT_ctl = param_global.Ta/M;
F_kp1 = integrate_casadi(f, DT_ctl, M, int_method); % runs with Ta from sensors
H_kp1 = integrate_casadi(h, DT_ctl, M, int_method); % runs with Ta from sensors

if(N_step_MPC == 1)
    DT2 = DT_ctl; % special case if Ts_MPC = Ta
else
    DT2 = DT - DT_ctl;
end
F2 = integrate_casadi(f, DT2, M, int_method); % runs with Ts_MPC-2*Ta
H2 = integrate_casadi(h, DT2, M, int_method); % runs with Ts_MPC-2*Ta

%% set up path function (here hardcoded, TODO: make it more general)

param_traj_pose_idx = param_traj.start_index(2):param_traj.stop_index(2);
pose = param_traj.pose(1:3, param_traj_pose_idx);
quat = param_traj.pose(4:7, param_traj_pose_idx);

param_traj_time = param_traj.time(:, param_traj_pose_idx);
theta_i = param_traj_time/param_traj_time(end);

% define path function

% if_else_and = @(cond1, cond2, true_val, false_val) if_else(cond1, if_else(cond2, true_val, false_val), false_val);

if(strcmp(robot_name, 'ur5e_6dof'))
    sigma_def = if_else(theta < theta_i(1), pose(:,1), ...
                if_else(theta < theta_i(2), trajectory_poly(theta, pose(:,1), pose(:,2), theta_i(2)), ...
                if_else(theta < theta_i(3), trajectory_poly(theta-theta_i(2), pose(:,2), pose(:,3), theta_i(3)-theta_i(2)), ...
                if_else(theta < theta_i(4), trajectory_poly(theta-theta_i(3), pose(:,3), pose(:,4), theta_i(4)-theta_i(3)), pose(:,end)))));
else % only 3 points
    sigma_def = if_else(theta < theta_i(1), pose(:,1), ...
                if_else(theta < theta_i(2), trajectory_poly(theta, pose(:,1), pose(:,2), theta_i(2)), ...
                if_else(theta < theta_i(3), trajectory_poly(theta-theta_i(2), pose(:,2), pose(:,3), theta_i(3)-theta_i(2)), pose(:,end))));
end

if(N_step_MPC == 1)
    MPC_traj_indices = 1:N_MPC;
else
    MPC_traj_indices = [1, 2, N_step_MPC : N_step_MPC : 1 + (N_MPC-1) * N_step_MPC];
end

T = param_traj_time(end);
sigma_fun = Function('sigma_fun', {theta}, {sigma_def});
quat_d = quat(:, 1); %assumption: constant quaternion for all poses

%% Calculate Initial Guess

x_0_0  = [q_0; q_0_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xd compute_tau_fun(q_0, dq_0, ddq_0); % gravity compensationof.m
q_0    = x_0_0(1   :   n); % useless line...
dq_0   = x_0_0(1+n : 2*n);
ddq_0  = q_0_pp;
u_k_0  = ddq_0;

u_init_guess_0 = ones(n, N_MPC).*u_k_0;
x_init_guess_0 =   [x_0_0 ones(2*n, N_MPC).*x_0_0];

if(N_step_MPC == 1)
    t_init_guess = linspace(0, N_MPC*DT_ctl, N_MPC+1);
else
    t_init_guess = [0; DT_ctl; linspace(DT, (N_MPC-1)*DT, N_MPC-1)'];
end
theta_init_guess_0 = t_init_guess/T;
v_init_guess_0 = zeros(1, N_MPC);

theta_sys = false;

if(theta_sys)
    lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(theta_init_guess_0)+numel(v_init_guess_0), 1);
    lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(theta_init_guess_0)+2, 1);

    init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); theta_init_guess_0(:); v_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];
else
    lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(theta_init_guess_0), 1);
    lam_g_init_guess_0 = zeros(numel(x_init_guess_0), 1);

    init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); theta_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];
end

if(any(isnan(full(init_guess_0))))
    error('115: init_guess_0 contains NaN values!');
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
theta = SX.sym( 'theta', 1, N_MPC+1 );
v = SX.sym( 'v', 1, N_MPC );

if(theta_sys)
    mpc_opt_var_inputs = {u, x, theta, v};
else
    mpc_opt_var_inputs = {u, x, theta};
end

% u = [u0; u1; ... uN-1], x = [x0; x1; ... xN] = [q_0;q_0_p;q_1;q_1_p;...;q_N;q_N_p]
% xx = [u; x] = [u0; u1; ... uN-1; x0; x1; ... xN]
%   u0 = q_0_pp = u(   1 :   n) | q_0 = x(     1     : n      ) | q_0_p = x(     1+  n :     2*n)
%   u0 = q_0_pp = xx(  1 :   n) | q_0 = xx(N_u+1     : n+N_u  ) | q_0_p = xx(N_u+1+  n : N_u+2*n)
%   u1 = q_1_pp = u( n+1 : 2*n) | q_1 = x(     1+2*n :     3*n) | q_1_p = x(     1+3*n :     4*n)
%   u1 = q_1_pp = xx(n+1 : 2*n) | q_1 = xx(N_u+1+2*n : N_u+3*n) | q_1_p = xx(N_u+1+3*n : N_u+4*n)
N_u = numel(u);
q0_pp_idx = 1 : n;
u_opt_indices = q0_pp_idx;

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
if(theta_sys)
    lbw = [repmat(pp.u_min, size(u, 2), 1); repmat(pp.x_min, size(x, 2), 1); -inf(numel(theta), 1); -inf(numel(v), 1)];
    ubw = [repmat(pp.u_max, size(u, 2), 1); repmat(pp.x_max, size(x, 2), 1); inf(numel(theta), 1); inf(numel(v), 1)];
else
    lbw = [repmat(pp.u_min, size(u, 2), 1); repmat(pp.x_min, size(x, 2), 1); -inf(numel(theta), 1)];
    ubw = [repmat(pp.u_max, size(u, 2), 1); repmat(pp.x_max, size(x, 2), 1); inf(numel(theta), 1)];
end

% input parameter
x_k    = SX.sym( 'x_k',    2*n, 1 ); % current x state = initial x state
t_k = SX.sym( 't_k', 1, 1 ); % current time

mpc_parameter_inputs = {x_k, t_k};
mpc_init_reference_values = [x_0_0(:); 0];

theta_d = (t_k+t_init_guess)/T;

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+1); % for F
g_theta = cell(1, N_MPC+1); % for H

if(weights_and_limits_as_parameter)
    lbg = SX(numel(lam_g_init_guess_0), 1);
    ubg = SX(numel(lam_g_init_guess_0), 1);
else
    lbg = zeros(numel(lam_g_init_guess_0), 1);
    ubg = zeros(numel(lam_g_init_guess_0), 1);
end

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y       = SX(  7, N_MPC+1); % TCP Pose:      (y_0 ... y_N)

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_x(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk

for i=0:N_MPC
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_py_fun(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_endeffector_py_fun(q);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        if(i == 0)
            g_theta(1, 1 + (0)) = { y(1:3)' - sigma_fun(theta( 1 + (0) )) };
            g_theta(1, 1 + (i+1)) = { H_kp1(theta(1 + (i)), v(1 + (i))) - theta(1 + (i+1)) };
            g_x(1, 1 + (i+1))  = { F_kp1(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk = x(t0) = tilde x0 to xk+1 = x(t0+Ta)
        elseif(i == 1)
            g_theta(1, 1 + (i+1)) = { H2(theta(1 + (i)), v(1 + (i))) - theta(1 + (i+1)) };
            g_x(1, 1 + (i+1))  = { F2(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk+1 = x(t0+Ta) to x(t0+Ts_MPC) = tilde x1
        else
            g_theta(1, 1 + (i+1)) = { H(theta(1 + (i)), v(1 + (i))) - theta(1 + (i+1)) };
            g_x(1, 1 + (i+1))  = { F(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for x(t0+Ts_MPC*i) to x(t0+Ts_MPC*(i+1))
            % runs only to T_horizon-Ts_MPC, i. e. tilde x_{N-1} = x(t0+Ts_MPC*(N-1)) and x_N doesn't exist
            % Trajectory must be y(t0), y(t0+Ta), Y(t0+Ts_MPC), ..., y(t0+Ts_MPC*(N-1))
        end
    end
end

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));


J_yt = 0;
J_yr = 0;
J_theta = 0;
for i=0:N_MPC
    % R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))';
    % % q_y_y_err = rotation2quaternion_casadi( R_y_yr );
    % q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)]; %ungenau aber schneller (flipping?)
    q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(quat_d));

    if(i < N_MPC)
        J_yt = J_yt + Q_norm_square( y(1:3, 1 + (i)) - sigma_fun(theta(1 + (i))), pp.Q_y(1:3,1:3)  );
        J_yr = J_yr + Q_norm_square( q_y_yr_err(2:4) , pp.Q_y(4:6,4:6)  );
        J_theta = J_theta + Q_norm_square( theta(1 + (i)) - theta_d(1 + (i)), pp.Q_theta  );
    else
        J_yt_N = Q_norm_square( y(1:3, 1 + (i)) - sigma_fun(theta(1 + (i))), pp.Q_yN(1:3,1:3)  );
        J_yr_N = Q_norm_square( q_y_yr_err(2:4) , pp.Q_yN(4:6,4:6)  );
        J_thetaN = Q_norm_square( theta(1 + (i)) - theta_d(1 + (i)), pp.Q_thetaN  );
    end
end

if(theta_sys)
    g = [g_x, g_theta];
else
    g = g_x;
end

J_q_pp = Q_norm_square(u, pp.R_q_pp);

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp, J_theta, J_thetaN}';


cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);