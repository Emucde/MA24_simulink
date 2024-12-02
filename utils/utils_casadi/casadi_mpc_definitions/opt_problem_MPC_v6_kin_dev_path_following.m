% MPC v4: Optimization problem 

import casadi.*

implicit_xk = false; % false liefert robustere lösungen, true ist etwas schneller aber führt oft zu numerischen noise

diff_variant_mode = struct;
diff_variant_mode.numdiff = 1; % default forward, central, backward deviation, tens to peaks, does not work with implicit_xk=true
diff_variant_mode.savgol = 2; % savgol filtering and deviation
diff_variant_mode.savgol_v2 = 3; % savgol filtering without additional equations and deviation
diff_variant_mode.savgol_not_equidist = 4; % savgol filtering, non equidistant samples, for feedforward
diff_variant_mode.savgol_not_equidist_noise_supr = 5; % savgol filtering without additional equations and deviation
diff_variant_mode.savgol_not_equidist_combined = 6; % savgol filtering, non equidistant samples, for pd control, first derivatve approximated
diff_variant_mode.savgol_not_equidist_combined2 = 7; % savgol filtering, non equidistant samples, for pd control, first derivatve approximated

diff_variant = diff_variant_mode.savgol_not_equidist;

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

%% set up path function (hardcoded here, TODO: make it more general)
param_traj_pose_idx = param_traj.start_index(traj_select_mpc):param_traj.stop_index(traj_select_mpc);
pose     = param_traj.pose(yt_indices, param_traj_pose_idx);
quat     = param_traj.pose(4:7, param_traj_pose_idx);
rotation = param_traj.rotation(:, :, param_traj_pose_idx);
rot_ax   = param_traj.rot_ax(:, param_traj_pose_idx);
alpha    = param_traj.alpha(:, param_traj_pose_idx);
t_val    = param_traj.time(param_traj_pose_idx);

param_traj_time = param_traj.time(:, param_traj_pose_idx);
theta_i = param_traj_time/param_traj_time(end);

% define path function

% if_else_and = @(cond1, cond2, true_val, false_val) if_else(cond1, if_else(cond2, true_val, false_val), false_val);
% [TODO]

% if(strcmp(robot_name, 'ur5e_6dof'))
%     sigma_t = if_else(theta < theta_i(1), pose(:,1), ...
%               if_else(theta < theta_i(2), trajectory_poly(theta, pose(:,1), pose(:,2), theta_i(2)), ...
%               if_else(theta < theta_i(3), trajectory_poly(theta-theta_i(2), pose(:,2), pose(:,3), theta_i(3)-theta_i(2)), ...
%               if_else(theta < theta_i(4), trajectory_poly(theta-theta_i(3), pose(:,3), pose(:,4), theta_i(4)-theta_i(3)), pose(:,end)))));
% else % only 3 points
%     sigma_t = if_else(theta < theta_i(1), pose(:,1), ...
%               if_else(theta < theta_i(2), trajectory_poly(theta, pose(:,1), pose(:,2),            theta_i(2)), ...
%               if_else(theta < theta_i(3), trajectory_poly(theta-theta_i(2), pose(:,2), pose(:,3), theta_i(3)-theta_i(2)), pose(:,end))));

    % alpha_i = trajectory_poly(theta,            alpha(1), alpha(2), theta_i(2));
    % skew_ew = skew(rot_ax(:, 2));
    % RR1 = (eye(3) + sin(alpha_i-alpha(1))*skew_ew + (1-cos(alpha_i-alpha(1)))*skew_ew^2) * rotation(:, :, 1);

    % alpha_i = trajectory_poly(theta-theta_i(2), alpha(2), alpha(3), theta_i(3)-theta_i(2));
    % skew_ew = skew(rot_ax(:, 3));
    % RR2 = (eye(3) + sin(alpha_i-alpha(2))*skew_ew + (1-cos(alpha_i-alpha(2)))*skew_ew^2) * rotation(:, :, 2);

    % sigma_r = if_else(theta < theta_i(1), rotation(:, :, 1), ...
    %           if_else(theta < theta_i(2), RR1, ...
    %           if_else(theta < theta_i(3), RR2, rotation(:, :,end))));
% end

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

%% Calculate Initial Guess
if(N_step_MPC <= 2)
    MPC_traj_indices = 1:(N_MPC+1);
else
    MPC_traj_indices = [0, 1, (1:1+(N_MPC-2))*N_step_MPC]+1;
end

T = param_traj_time(end);
sigma_t_fun = Function('sigma_t_fun', {theta}, {sigma_t});
sigma_r_fun = Function('sigma_r_fun', {theta}, {sigma_r});
sigma_r_quat_fun = Function('sigma_r_quat_fun', {theta}, {rotm2quat_v4_casadi(sigma_r)});

% Robot System: Initial guess
q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m

u_k_0  = q_0_red_pp;

u_init_guess_0 = u_k_0;
q_init_guess_0 = ones(n_red, N_MPC+1).*x_0_0(1:n_red);

if(N_step_MPC <= 2)
    t_init_guess = linspace(0, N_MPC*DT_ctl, N_MPC+1);
else
    t_init_guess = [0; DT_ctl; linspace(DT, (N_MPC-1)*DT, N_MPC-1)'];
end
theta_init_guess_0 = t_init_guess/T;
theta_0_0 = theta_init_guess_0(1);

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(q_init_guess_0)+numel(theta_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(u_init_guess_0) + numel(x_0_0), 1);

init_guess_0 = [u_init_guess_0(:); q_init_guess_0(:); theta_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

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
u     = SX.sym( 'u',    n_red,       1 ); % %u0=q0pp (0 to Ta)
q     = SX.sym( 'q',    n_red, N_MPC+1 );
theta = SX.sym( 'theta', 1, N_MPC+1 );
x = q;

mpc_opt_var_inputs = {u, x, theta};

% u = [u0; u1; ... uN-1], x = [x0; x1; ... xN] = [q_0;q_0_p;q_1;q_1_p;...;q_N;q_N_p]
% xx = [u; x] = [u0; u1; ... uN-1; x0; x1; ... xN]
%   u0 = q_0_pp = u(   1 :   n_red) | q_0 = x(     1     : n_red      ) | q_0_p = x(     1+  n_red :     2*n_red)
%   u0 = q_0_pp = xx(  1 :   n_red) | q_0 = xx(N_u+1     : n_red+N_u  ) | q_0_p = xx(N_u+1+  n_red : N_u+2*n_red)
%   u1 = q_1_pp = u( n_red+1 : 2*n_red) | q_1 = x(     1+2*n_red :     3*n_red) | q_1_p = x(     1+3*n_red :     4*n_red)
%   u1 = q_1_pp = xx(n_red+1 : 2*n_red) | q_1 = xx(N_u+1+2*n_red : N_u+3*n_red) | q_1_p = xx(N_u+1+3*n_red : N_u+4*n_red)
% N_u = numel(u);
q0_pp_idx = 1 : n_red;
u_opt_indices = q0_pp_idx;

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';

lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); repmat(pp.x_min(n_indices), size(q, 2), 1); -inf(numel(theta), 1)];
ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1); repmat(pp.x_max(n_indices), size(q, 2), 1);  inf(numel(theta), 1)];

% input parameter
x_k = SX.sym( 'x_k', 2*n_red, 1 ); % current x state = initial x state
theta_k = SX.sym( 'theta_k', 1, 1 ); % next theta state
t_k = SX.sym( 't_k', 1, 1 ); % current time
q_prev = SX.sym( 'q_prev',  n_red, N_MPC+1 );
theta_prev = SX.sym( 'theta_prev', 1, N_MPC+1 );

mpc_parameter_inputs = {x_k, theta_k, t_k, q_prev, theta_prev};
mpc_init_reference_values = [x_0_0(:); theta_0_0; 0; q_init_guess_0(:); theta_init_guess_0(:)];

theta_d = (t_k+t_init_guess)/T;

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
if(implicit_xk)
    g_x  = cell(1, N_MPC+1);
else
    g_x  = cell(1, N_MPC+2);
end
g_u  = cell(1, size(u,2));


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

if(diff_variant == diff_variant_mode.numdiff)
    S_v = create_numdiff_matrix(DT_ctl, n_red, N_MPC+1, 'fwdbwdcentraltwotimes', DT);
    S_v_temp = S_v/DT_ctl;
    S_a_temp = S_v_temp^2;
    S_a = S_a_temp * DT_ctl^2;
    disp('Selected diff method: forward central backward, non-equidistant samples, good for pd but tends to peaks in u');
elseif(diff_variant == diff_variant_mode.savgol)
    % DD_DT  = create_numdiff_matrix(DT, n_red, N_MPC+1, 'savgol');
    DD_DT  = create_numdiff_matrix(DT_ctl, n_red, N_MPC+1, 'savgol', DT, MPC_traj_indices);

    S_v = DD_DT{2};
    S_a = DD_DT{3};
    disp('Selected diff method: savgol, equidistant samples');
elseif(diff_variant == diff_variant_mode.savgol_v2)
    DD_DT  = create_numdiff_matrix(DT, n_red, N_MPC+1, 'savgol');

    S_q = DD_DT{1};
    S_v = DD_DT{2};
    S_a = DD_DT{3};
    disp('Selected diff method: savgol_v2 (q = SV q)');
elseif(diff_variant == diff_variant_mode.savgol_not_equidist || diff_variant == diff_variant_mode.savgol_not_equidist_combined2)
    param_golay = struct('Nq', 2, 'd', 2);
    DD_DT  = create_numdiff_matrix(DT_ctl, n_red, N_MPC+1, 'savgol_notequidist', DT, MPC_traj_indices, param_golay);

    % S_q = DD_DT{1};
    S_v = DD_DT{2};
    S_a = DD_DT{3};
    disp('Selected diff method: savgol_notequidist, purely polynomial filtering');
elseif(diff_variant == diff_variant_mode.savgol_not_equidist_noise_supr)
    param_golay = struct('Nq', 2, 'd', 2);
    DD_DT  = create_numdiff_matrix(DT_ctl, n_red, N_MPC+1, 'savgol_notequidist', DT, MPC_traj_indices, param_golay);
    S_v = DD_DT{2};

    % Funktioniert auch, ist dann aber keine polynomableitung
    % Interessanterweise hat diese Variante eine enorm starke Rauschunterdrückung zur Folge.
    % Dafür ist es nicht so genau wie mit DT_DT{3}. Insbesondere wenn man die Gewichtungsmatrizen
    % für x-xprev zu klein hat, wird es instabil, wenn der TCP still stehen soll. Geht aber auch
    % nur für Nq=2.
    S_a = S_v^2;
    disp('Selected diff method: savgol_notequidist_noise_supr: S_a = S_v^2');
elseif(diff_variant == diff_variant_mode.savgol_not_equidist_combined)
    param_golay = struct('Nq', 2, 'd', 2);
    DD_DT  = create_numdiff_matrix(DT_ctl, n_red, N_MPC+1, 'savgol_notequidist', DT, MPC_traj_indices, param_golay);

    % S_q = DD_DT{1};
    S_v = DD_DT{2};
    S_a = DD_DT{3};
    disp('Selected diff method: savgol_notequidist, for d > 2 better with implicit_xk = false, good for pd control');

    % if the polynomordering d is 2 or lower, e.g. you have
    % in case of d = 2: q(t) = c0 + c1*t + c2*t^2
    % in case of d = 1: q(t) = c0 + c1*t

    % the problem is, that the second derivative is
    % in case of d = 2: q''(t) = 2*c2
    % in case of d = 1: q''(t) = 0

    % this means, the left and right side points are approximated by using q''(t) of the most left and right
    % window, the joint accelerations would be hold constant. This is not a big problem, if the feedforward is
    % used to calculate the torque out of q_pp. But if you try to use the PD controller it would lead to incorrect
    % q_pp at the approximated points. Therefore, in case of d <= 2 I use instead of q''(t) the numerical derivative
    % from create_numdiff_matrix(DT_ctl, n_red, N_MPC+1, 'fwdbwdcentraltwotimes', DT);

    % (a) for joint angle q: use forward backward derivatives at end and begin

    % first derivative should be as easy as possible because it should fit x_k(1+n_red:2*n_red) = q_p
    % Achtung, dann geht feedforward nicht mehr!
    if(param_golay.d <= 2)
        S_v_2 = create_numdiff_matrix(DT_ctl, n_red, N_MPC+1, 'fwdbwdcentraltwotimes', DT);
        S_a_2 = S_v_2^2;

        S_v(1:2*n_red, :) = S_v_2(1:2*n_red, :);
        S_v(end-2*n_red+1:end, :) = S_v_2(end-2*n_red+1:end, :);
        %S_a(1:2*n_red, :) = S_a_2(1:2*n_red, :);
        %S_a(end-2*n_red+1:end, :) = S_a_2(end-2*n_red+1:end, :);

        S_a_v3 = S_v^2;
        S_a(1:2*n_red, :) = S_a_v3(1:2*n_red, :);
        S_a(end-2*n_red+1:end, :) = S_a_v3(end-2*n_red+1:end, :);
    end
else
    error('invalid mode');
end

if(implicit_xk)
    qq = reshape(x(1:n_red, 2:end), n_red*(N_MPC+1-1), 1);
    qq = [x_k(1:n_red); qq];
else
    qq = reshape(x(1:n_red, :), n_red*(N_MPC+1), 1);
end

qq_p  = S_v * qq;
qq_pp = S_a * qq;

if(diff_variant == diff_variant_mode.savgol_v2)
    qq = S_q * qq;
end

q = reshape(qq, n_red, N_MPC+1);
q_p = reshape(qq_p, n_red, N_MPC+1);
q_pp = reshape(qq_pp, n_red, N_MPC+1);

if(diff_variant == diff_variant_mode.savgol_not_equidist_combined2)
    if(implicit_xk)
        q_p(:, 1) = x_k(1+n_red:2*n_red);
    else
        q_p(:, 1) = (q(:, 2) - q(:, 1))/(DT_ctl);
    end
    q_pp(:, 1) = (q_p(:, 2) - q_p(:, 1))/(DT_ctl);

    % q_p(:, 2) = (q(:, 3) - q(:, 1))/(DT); % optional, verschlechterts aber
    % q_pp(:, 2) = (q_p(:, 3) - q_p(:, 1))/(DT); % optional, verschlechterts aber
    q_p(:, end) = (q(:, end) - q(:, end-1))/(DT);
    q_pp(:, end) = (q_p(:, end) - q_p(:, end-1))/(DT);
elseif(implicit_xk)
    q_p(:, 1) = x_k(1+n_red:2*n_red);
end

g_x(1, 1 + (0)) = {x_k  - [q(:, 1 + (0)); q_p(:, 1 + (0))]}; % x0 = xk
g_x(1, 1 + (1)) = {u - q_pp(:, 1 + (0))}; % u0 = q0pp

for i=0:N_MPC
    q_i = q(:, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q_i);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(q_i);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);
end

g = g_x;

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

if isempty(yt_indices)
    J_yt = 0;
    J_yt_N = 0;
else
    J_yt = 0;
    for i=0:N_MPC
        if(i < N_MPC)
            J_yt = J_yt + Q_norm_square( y(yt_indices, 1 + (i)) - sigma_t_fun(theta(1 + (i))), pp.Q_y(yt_indices,yt_indices)  );
        else
            J_yt_N = Q_norm_square( y(yt_indices, 1 + (i)) - sigma_t_fun(theta(1 + (i))), pp.Q_yN(yt_indices,yt_indices)  );
        end
    end
end

if isempty(yr_indices)
    J_yr = 0;
    J_yr_N = 0;
else
    J_yr = 0;
    for i=0:N_MPC
        R_y_yr = R_e_arr{1 + (i)} * sigma_r_fun(theta(1 + (i)))';
        q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)];  % am genauesten
        
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

% Es macht mmn. keinen Sinn die Loop mit i=0 zu beginnen weil theta_0 nur für y - sigma(theta_0) verwendet werden sollte.
% Wird dafür ein Minimum gefunden ist garantiert, dass theta_0 = theta* d. h. der kürzeste Abstand zum gegenwärtigen Pfad ist.
% Problem: Es ist nicht garantiert, dass hier ein Minimum gefunden wird, da es eine Soft Constraint ist die relativ zu anderen
% Kosten gewichtet wird. Sauber ist es daher, theta* außerhalb der MPC zu berechnen und als Startwert zu verwenden.
% ... geht aber nicht... [TODO]
J_theta = 0;
for i=0:N_MPC
    if(i < N_MPC)
        J_theta = J_theta + Q_norm_square( theta(1 + (i)) - theta_d(1 + (i)), pp.Q_theta  );
    else
        J_thetaN = Q_norm_square( theta(1 + (i)) - theta_d(1 + (i)), pp.Q_thetaN  );
    end
end

J_q_p = Q_norm_square(q_p, pp.R_q_p(n_indices, n_indices)); %Q_norm_square(u, pp.R_u);
J_q_pp = Q_norm_square(u, pp.R_q_pp(n_indices, n_indices)); %Q_norm_square(u, pp.R_u);
J_theta_prev = Q_norm_square(theta-theta_prev, pp.R_theta_prev);

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_p, J_q_pp, J_theta, J_thetaN, J_theta_prev}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);