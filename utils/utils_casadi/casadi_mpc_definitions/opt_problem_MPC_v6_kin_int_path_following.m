% MPC v4: Optimization problem 

import casadi.*

yt_indices = param_robot.yt_indices;
yr_indices = param_robot.yr_indices;

n = param_robot.n_DOF; % Dimension of joint space
n_red = param_robot.n_red; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

n_x_indices = [n_indices n_indices+n];

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

q_red = SX.sym( 'q',     n_red, 1 );
x_red = SX.sym( 'x',   2*n_red, 1 );
u_red = SX.sym( 'u',     n_red, 1 );
theta = SX.sym('theta', 1); % path parameter
lambda_theta_ew = SX.sym('lambda_theta_ew', 1);
v = SX.sym('v', 1);

q_subs            = SX(q_0);
q_subs(n_indices) = q_red;

H_red = Function('H_red', {q_red}, {hom_transform_endeffector_py_fun(q_subs)});
quat_fun_red = Function('quat_fun_red', {q_red}, {quat_endeffector_py_fun(q_subs)});

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
DT_ctl = param_global.Ta/M;
if(N_step_MPC <= 2)
    DT = DT_ctl; % special case if Ts_MPC = Ta
    DT2 = DT_ctl; % special case if Ts_MPC = Ta
else
    DT = N_step_MPC * DT_ctl; % = Ts_MPC
    DT2 = DT - DT_ctl; % = (N_step_MPC - 1) * DT_ctl = (N_MPC-1) * Ta/M = Ts_MPC - Ta
end

opt = struct;
opt.allow_free = true;

% integrator for x
f_red = Function('f_red', {x_red, u_red}, {[x_red(n_red+1:2*n_red); u_red]});
h = Function('h', {theta, v}, {-lambda_theta_ew*theta + v}, opt);

F = integrate_casadi(f_red, DT, M, int_method); % runs with Ts_MPC
H_int = integrate_casadi(h, DT, M, int_method); % runs with Ts_MPC
H = Function('H', {theta, v, lambda_theta_ew}, {H_int(theta, v)});

F_kp1 = integrate_casadi(f_red, DT_ctl, M, int_method); % runs with Ta from sensors
H_kp1_int = integrate_casadi(h, DT_ctl, M, int_method); % runs with Ta from sensors
H_kp1 = Function('H', {theta, v, lambda_theta_ew}, {H_kp1_int(theta, v)});

F2 = integrate_casadi(f_red, DT2, M, int_method); % runs with Ts_MPC-2*Ta
H2_int = integrate_casadi(h, DT2, M, int_method); % runs with Ts_MPC-2*Ta
H2 = Function('H', {theta, v, lambda_theta_ew}, {H2_int(theta, v)});

%% set up path function: TODO: input for path selection

traj_fun_cell = cell(1, param_traj.N_traj);
for traj_select_index = 1:param_traj.N_traj
    param_traj_pose_idx = param_traj.start_index(traj_select_index):param_traj.stop_index(traj_select_index);
    pose     = param_traj.pose(yt_indices, param_traj_pose_idx);
    quat     = param_traj.pose(4:7, param_traj_pose_idx);
    rotation = param_traj.rotation(:, :, param_traj_pose_idx);
    rot_ax   = param_traj.rot_ax(:, param_traj_pose_idx);
    alpha    = param_traj.alpha(:, param_traj_pose_idx);
    t_val    = param_traj.time(param_traj_pose_idx);

    param_traj_time = param_traj.time(:, param_traj_pose_idx);
    theta_i = param_traj_time/param_traj_time(end);

    N = size(pose,2);
    single_path_t = cell(1,N);
    single_path_r = cell(1,N);
    for i = 0:N
        if(i == 0)
            single_path_t{1} = pose(:,1);

            single_path_r{1} = rotation(:, :, 1);
        elseif(i < N)
            single_path_t{i+1} = trajectory_poly(theta-theta_i(i), pose(:,i), pose(:,i+1), theta_i(i+1)-theta_i(i));

            alpha_i = trajectory_poly(theta-theta_i(i), alpha(i), alpha(i+1), theta_i(i+1)-theta_i(i));
            skew_ew = skew(rot_ax(:, i+1));
            RR2 = (eye(3) + sin(alpha_i-alpha(i))*skew_ew + (1-cos(alpha_i-alpha(i)))*skew_ew^2) * rotation(:, :, i);
            single_path_r{i+1} = RR2;
        elseif(i == N)
            single_path_t{i+1} = pose(:,N);
            single_path_r{i+1} = rotation(:, :, N);
        end
    end

    sigma_t = single_path_t{end};
    sigma_r = single_path_r{end};
    for i = N:-1:1
        sigma_t = if_else(theta < theta_i(i), single_path_t{i}, sigma_t);
        sigma_r = if_else(theta < theta_i(i), single_path_r{i}, sigma_r);
    end

    traj_fun_cell{traj_select_index} = {sigma_t, sigma_r};
end

sigma_t_all = traj_fun_cell{end}{1};
sigma_r_all = traj_fun_cell{end}{2};
traj_select = SX.sym( 'traj_select', 1, 1);
for traj_select_index = param_traj.N_traj:-1:1
    sigma_t = traj_fun_cell{traj_select_index}{1};
    sigma_r = traj_fun_cell{traj_select_index}{2};

    sigma_t_all = if_else(traj_select == traj_select_index, sigma_t, sigma_t_all);
    sigma_r_all = if_else(traj_select == traj_select_index, sigma_r, sigma_r_all);
end

T = param_traj_time(end);
sigma_t_fun = Function('sigma_t_fun', {theta, traj_select}, {sigma_t_all});
sigma_r_fun = Function('sigma_r_fun', {theta, traj_select}, {sigma_r_all});
sigma_r_quat_fun = Function('sigma_r_quat_fun', {theta}, {rotm2quat_v4_casadi(sigma_r)});

%% Calculate Initial Guess
if(N_step_MPC <= 2)
    MPC_traj_indices = 1:(N_MPC+1);
else
    MPC_traj_indices = [0, 1, (1:1+(N_MPC-2))*N_step_MPC]+1;
end

% Robot System: Initial guess
q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m

u_k_0  = q_0_red_pp;

u_init_guess_0 = ones(n_red, N_MPC).*u_k_0;
x_init_guess_0 = [x_0_0 ones(2*n_red, N_MPC).*x_0_0];

t_init_guess = (MPC_traj_indices-1)*DT_ctl;
theta_init_guess_0 = t_init_guess/T;
theta_0 = theta_init_guess_0(1);
v_init_guess_0 = zeros(1, N_MPC);

theta_sys = false; % funktioniert zwar, aber macht keinen sinn da theta_k nicht korrekt ist!

if(theta_sys)
    lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(theta_init_guess_0)+numel(v_init_guess_0), 1);
    lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(theta_init_guess_0), 1); %-1 because theta_i(0) - theta_d(0) is not weighted
    % The core idea here:
    % Only if a dynamic for theta is used it is important to determine theta_0 so that y - sigma(theta_0) is as small as possible.
    % this and only this theta value is the true path parameter at the beginning of the MPC horizon.
    % when no theta_sys is used theta_0 is not important because q_0 is not a optimization variable, therefore theta_0 have
    % no influence on the optimization problem. You only choose the newer theta value for the prediction horizon so that the
    % y(theta_i) - sigma(theta_i) s.t. theta_i - t_i is as small as possible. In case of theta_i = t_i, sigma(theta_i) = y_d(t_i).
    % If theta_i != t_i, sigma(theta_i) != y_d(t_i) but maybe y(theta_i) - sigma(theta_i) is smaller than y(theta_i) - sigma(t_i)...

    init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); theta_init_guess_0(:); v_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];
else
    lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(theta_init_guess_0), 1);
    lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(theta_0), 1);

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
u     = SX.sym( 'u',  n_red,   N_MPC   ); % u = q_0_pp
x     = SX.sym( 'x',  2*n_red, N_MPC+1 );
theta = SX.sym( 'theta', 1, N_MPC+1 );
v = SX.sym( 'v', 1, N_MPC );

if(theta_sys)
    mpc_opt_var_inputs = {u, x, theta, v};
else
    mpc_opt_var_inputs = {u, x, theta};
end

% u = [u0; u1; ... uN-1], x = [x0; x1; ... xN] = [q_0;q_0_p;q_1;q_1_p;...;q_N;q_N_p]
% xx = [u; x] = [u0; u1; ... uN-1; x0; x1; ... xN]
%   u0 = q_0_pp = u(   1 :   n_red) | q_0 = x(     1     : n_red      ) | q_0_p = x(     1+  n_red :     2*n_red)
%   u0 = q_0_pp = xx(  1 :   n_red) | q_0 = xx(N_u+1     : n_red+N_u  ) | q_0_p = xx(N_u+1+  n_red : N_u+2*n_red)
%   u1 = q_1_pp = u( n_red+1 : 2*n_red) | q_1 = x(     1+2*n_red :     3*n_red) | q_1_p = x(     1+3*n_red :     4*n_red)
%   u1 = q_1_pp = xx(n_red+1 : 2*n_red) | q_1 = xx(N_u+1+2*n_red : N_u+3*n_red) | q_1_p = xx(N_u+1+3*n_red : N_u+4*n_red)
N_u = numel(u);
N_x = numel(x);

u_idx = [1 : numel(u)];
x_idx = N_u + [1 : numel(x)];

q0_pp_idx = u_idx(1:n_red);
x1_idx = x_idx(1+2*n_red : 4*n_red);
q1_pp_idx = u_idx(1+n_red : 2*n_red);
u_opt_indices = [q0_pp_idx, x1_idx, q1_pp_idx];

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';

% input parameter
x_k = SX.sym( 'x_k', 2*n_red, 1 ); % current x state = initial x state
t_k = SX.sym( 't_k', 1, 1 ); % current time
x_prev = SX.sym( 'x_prev',  2*n_red, N_MPC+1 );
theta_prev = SX.sym( 'theta_prev', 1, N_MPC+1 );

mpc_parameter_inputs = {x_k, t_k, x_prev, theta_prev, traj_select};
mpc_init_reference_values = [x_0_0(:); 0; x_init_guess_0(:); theta_init_guess_0(:); traj_select_mpc];

theta_d = (t_k+t_init_guess)/T;

if(theta_sys)
    lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); -inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1); -inf(numel(theta), 1); -inf(numel(v), 1)];
    ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1);  inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1);  inf(numel(theta), 1);  inf(numel(v), 1)];
else
    lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); -inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1); -inf(numel(theta), 1)];
    ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1);  inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1);  inf(numel(theta), 1)];
    % lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); repmat(pp.x_min([n_indices, n_indices+n]), size(x, 2), 1); theta_d'+pp.d_theta_min];
    % ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1); repmat(pp.x_max([n_indices, n_indices+n]), size(x, 2), 1); theta_d'+pp.d_theta_max];
end

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+1); % for F

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
g_theta = {theta(1) - theta_d(1)};

for i=0:N_MPC
    q = x(1:n_red, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(q);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        if(i == 0)
            g_x(1, 1 + (i+1))  = { F_kp1(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk = x(t0) = tilde x0 to xk+1 = x(t0+Ta)
        elseif(i == 1)
            g_x(1, 1 + (i+1))  = { F2(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk+1 = x(t0+Ta) to x(t0+Ts_MPC) = tilde x1
        else
            g_x(1, 1 + (i+1))  = { F(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for x(t0+Ts_MPC*i) to x(t0+Ts_MPC*(i+1))
        end
    end
end

g = [g_x, g_theta];

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

if isempty(yt_indices)
    J_yt = 0;
    J_yt_N = 0;
else
    J_yt = 0;
    for i=1:N_MPC
        if(i < N_MPC)
            J_yt = J_yt + Q_norm_square( y(yt_indices, 1 + (i)) - sigma_t_fun(theta(1 + (i)), traj_select), pp.Q_y(yt_indices,yt_indices)  );
        else
            J_yt_N = Q_norm_square( y(yt_indices, 1 + (i)) - sigma_t_fun(theta(1 + (i)), traj_select), pp.Q_yN(yt_indices,yt_indices)  );
        end
    end
end

if isempty(yr_indices)
    J_yr = 0;
    J_yr_N = 0;
else
    J_yr = 0;
    for i=1:N_MPC
        % R_y_yr = R_e_arr{1 + (i)} * sigma_r_fun(theta(1 + (i)))';
        % q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)];  % am genauesten
        
        R_y_yr = R_e_arr{1 + (i)}' * sigma_r_fun(theta(1 + (i)), traj_select) - sigma_r_fun(theta(1 + (i)), traj_select)' * R_e_arr{1 + (i)};
        q_y_yr_err = [1; R_y_yr(3,2); R_y_yr(1,3); R_y_yr(2,1)];

        % die beiden methoden weisen größere Quaternionenfehler auf:
        % q_y_yr_err = rotm2quat_v4_casadi(R_y_yr);
        % y_d = rotm2quat_v4_casadi(sigma_r_fun(theta(1 + (i)))');
        % q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(y_d));
    
        if(i < N_MPC)
            J_yr = J_yr + Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_y(3+yr_indices,3+yr_indices)  );
        else
            J_yr_N = Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_yN(3+yr_indices,3+yr_indices)  );
        end
    end
end

J_theta = Q_norm_square(theta(1 + (1:N_MPC-1)) - theta_d(1 + (1:N_MPC-1)), pp.Q_theta);
J_thetaN = Q_norm_square(theta(1 + (N_MPC)) - theta_d(1 + (N_MPC)), pp.Q_thetaN);

J_q_ref = Q_norm_square(x(1:n_red, :) - pp.q_ref(n_indices), pp.R_q_ref(n_indices, n_indices));
J_q_p = Q_norm_square(x(1+n_red:2*n_red, :), pp.R_q_p(n_indices, n_indices)); %Q_norm_square(u, pp.R_u);
J_u = Q_norm_square(u, pp.R_u(n_indices, n_indices)); %Q_norm_square(u, pp.R_u);

J_x_prev = Q_norm_square(x-x_prev, pp.R_x_prev(n_x_indices, n_x_indices));
J_theta_prev = Q_norm_square(theta-theta_prev, pp.R_theta_prev);

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_ref, J_q_p, J_u, J_theta, J_thetaN, J_x_prev, J_theta_prev}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);