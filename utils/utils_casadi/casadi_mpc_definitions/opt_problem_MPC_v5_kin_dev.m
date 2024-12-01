% MPC v4: Optimization problem 

import casadi.*

diff_variant_mode = struct;
diff_variant_mode.numdiff = 1; % default forward, central, backward deviation
diff_variant_mode.savgol = 2; % savgol filtering and deviation
diff_variant_mode.savgol_v2 = 3; % savgol filtering without additional equations and deviation
diff_variant_mode.savgol_not_equidist = 4; % savgol filtering, non equidistant samples, for feedforward
diff_variant_mode.savgol_not_equidist_noise_supr = 5; % savgol filtering without additional equations and deviation
diff_variant_mode.savgol_not_equidist_combined = 6; % savgol filtering, non equidistant samples, for pd control, first derivatve approximated
diff_variant_mode.savgol_not_equidist_combined2 = 7; % savgol filtering, non equidistant samples, for pd control, first derivatve approximated

diff_variant = diff_variant_mode.savgol_not_equidist_combined2;

yt_indices = param_robot.yt_indices;
yr_indices = param_robot.yr_indices;

n = param_robot.n_DOF; % Dimension of joint space
n_red = param_robot.n_red; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

n_x_indices = [n_indices n_indices+n];

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

q_red = SX.sym( 'q',     n_red, 1 );

q_subs            = SX(q_0);
q_subs(n_indices) = q_red;

H_red = Function('H_red', {q_red}, {hom_transform_endeffector_py_fun(q_subs)});
quat_fun_red = Function('quat_fun_red', {q_red}, {quat_endeffector_py_fun(q_subs)});

M = 1; % RK4 steps per interval
DT_ctl = param_global.Ta/M;
if(N_step_MPC <= 2)
    DT = DT_ctl; % special case if Ts_MPC = Ta
    DT2 = DT_ctl; % special case if Ts_MPC = Ta
else
    DT = N_step_MPC * DT_ctl; % = Ts_MPC
    DT2 = DT - DT_ctl; % = (N_step_MPC - 1) * DT_ctl = (N_MPC-1) * Ta/M = Ts_MPC - Ta
end

%% Calculate Initial Guess
if(N_step_MPC <= 2)
    MPC_traj_indices = 1:(N_MPC+1);
else
    MPC_traj_indices = [0, 1, (1:1+(N_MPC-2))*N_step_MPC]+1;
end

p_d_0 = param_trajectory.p_d( 1:3, MPC_traj_indices ); % (y_0 ... y_N)
q_d_0 = param_trajectory.q_d( 1:4, MPC_traj_indices ); % (q_0 ... q_N)
y_d_0 = [p_d_0; q_d_0];

% Robot System: Initial guess
q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m

u_k_0  = q_0_red_pp;

u_init_guess_0 = ones(n_red, N_MPC).*u_k_0;
x_init_guess_0 = ones(2*n_red, N_MPC+1).*x_0_0;
q_init_guess_0 = x_init_guess_0(1:n_red, :);

lam_x_init_guess_0 = zeros(numel(u_init_guess_0) + numel(x_init_guess_0) + numel(q_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(u_init_guess_0) + numel(x_init_guess_0) + numel(x_0_0), 1);

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); q_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

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
u     = SX.sym( 'u',    n_red,   N_MPC   );
x     = SX.sym( 'x',    2*n_red, N_MPC+1 );
q     = SX.sym( 'x',    n_red, N_MPC+1 );

% confusing, but I need equation constraints to relate x to q and qp = Sv q.
% only then the joint and velocity limits are maintained in!
mpc_opt_var_inputs = {u, x, q};

%u_opt_indices = 1:n_red;

N_u = numel(u);
N_x = numel(x);

u_idx = [1 : numel(u)];
x_idx = N_u + [1 : numel(x)];
q_idx = N_u + N_x + [1 : numel(q)];

q0_pp_idx = u_idx(1:n_red);
x1_idx = x_idx(1+2*n_red : 4*n_red);
q1_pp_idx = u_idx(1+n_red : 2*n_red);
u_opt_indices = [q0_pp_idx, x1_idx, q1_pp_idx];

% TODO: So kann man q_p eigentlich nicht limitieren!!! vgl. mpc v8

% optimization variables cellarray w
% [TODO]: es wird nicht geprüft ob d/dt q auch die limits einhält!!!
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); repmat(pp.x_min(n_x_indices), size(x, 2), 1); repmat(pp.x_min(n_indices), size(q, 2), 1)];
ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1); repmat(pp.x_max(n_x_indices), size(x, 2), 1); repmat(pp.x_max(n_indices), size(q, 2), 1)];


% input parameter
x_k  = SX.sym( 'x_k',  2*n_red,       1 ); % current x state = initial x state
y_d  = SX.sym( 'y_d',  m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d
x_prev = SX.sym( 'x_prev', 2*n_red, N_MPC+1 );

mpc_parameter_inputs = {x_k, y_d, x_prev};
mpc_init_reference_values = [x_0_0(:); y_d_0(:); x_init_guess_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+2);
g_u  = cell(1, N_MPC);

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
y    = SX( 7, N_MPC+1 ); % TCP pose:      (y_0 ... y_N)
R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

if(diff_variant == diff_variant_mode.numdiff)
    S_v = create_numdiff_matrix(DT_ctl, n_red, N_MPC+1, 'fwdbwdcentraltwotimes', DT);
    S_a = S_v^2;
    disp('Selected diff method: forward central backward, non-equidistant samples');
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
    disp('Selected diff method: savgol_notequidist, purely polynomial filtering');

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
        S_a(1:n_red, :) = S_a_2(1:n_red, :);
        S_a(end-n_red+1:end, :) = S_a_2(end-n_red+1:end, :);
    end
else
    error('invalid mode');
end
%      x = [q(t0),   q(t1), ...  q(tN),  dq(t0),  dq(t1), ...  dq(tN)] = [qq;   qq_p ]
% d/dt x = [dq(t0), dq(t1), ... dq(tN), ddq(t0), ddq(t1), ... ddq(tN)] = [qq_p; qq_pp]

% p_d_p_0 = param_trajectory.p_d_p( 1:3, MPC_traj_indices ); % (y_0 ... y_N)
% pp = reshape(p_d_0, 3*(N_MPC+1), 1);
% S_v1 = create_numdiff_matrix(DT_ctl, 3, N_MPC+1, 'fwdbwdcentraltwotimes', DT);
% pp_p_v1 = S_v1 * pp;
% p_p_v1 = reshape(pp_p_v1, 3, N_MPC+1);
% plot(p_p_v1', 'linewidth',0.5); hold on; plot(p_d_p_0', '--', 'linewidth', 2)

% figure(2)
% DD_DT2  = create_numdiff_matrix(DT_ctl, 3, N_MPC+1, 'savgol_notequidist', DT, MPC_traj_indices);
% S_v2 = DD_DT2{2};
% pp_p_v2 = S_v2 * pp;
% p_p_v2 = reshape(pp_p_v2, 3, N_MPC+1);
% plot(p_p_v2', 'linewidth',0.5); hold on; plot(p_d_p_0', '--', 'linewidth', 2)

% N_traj = 100;
% MPC_traj_indices = [0, 1, (1:1+(N_traj-3))*N_step_MPC]+1;
% p_d_tst = param_trajectory.p_d( 1:3, MPC_traj_indices);
% p_d_p_tst = param_trajectory.p_d_p( 1:3, MPC_traj_indices);
% p_d_pp_tst = param_trajectory.p_d_pp( 1:3, MPC_traj_indices);
% pp = reshape(p_d_tst, 3*(N_traj), 1);
% DD_DT2  = create_numdiff_matrix(DT_ctl, 3, N_traj, 'savgol_notequidist', DT, MPC_traj_indices);
% S_v2 = DD_DT2{2};
% S_a2 = DD_DT2{3};

% % S_v2 = create_numdiff_matrix(DT_ctl, 3, N_traj, 'fwdbwdcentraltwotimes', DT);
% % S_a2 = S_v2^2;
% % pp_p_v2 = S_v2 * pp;
% % p_p_v2 = reshape(pp_p_v2, 3, N_traj);
% % plot(p_p_v2', 'linewidth',0.5); hold on; plot(p_d_p_tst', '--', 'linewidth', 2)
% pp_pp_v2 = S_a2 * pp;
% p_pp_v2 = reshape(pp_pp_v2, 3, N_traj);
% plot(p_pp_v2', 'linewidth',0.5); hold on; plot(p_d_pp_tst', '--', 'linewidth', 2)

qq = reshape(q, n_red*(N_MPC+1), 1);

qq_p  = S_v * qq;
qq_pp = S_a * qq;

if(diff_variant == diff_variant_mode.savgol_v2)
    qq = S_q * qq;
end

q = reshape(qq, n_red, N_MPC+1);
q_p = reshape(qq_p, n_red, N_MPC+1);
q_pp = reshape(qq_pp, n_red, N_MPC+1);

if(diff_variant == diff_variant_mode.savgol_not_equidist_combined2)
    q_p(:, 1) = (q(:, 2) - q(:, 1))/(DT_ctl);
    q_pp(:, 1) = (q_p(:, 2) - q_p(:, 1))/(DT_ctl);
    q_p(:, end) = (q(:, end) - q(:, end-1))/(DT);
    q_pp(:, end) = (q_p(:, end) - q_p(:, end-1))/(DT);
end

g_x(1, 1 + (0)) = {x_k           - x(   :, 1 + (0))}; % x0 = xk

for i=0:N_MPC
    q_i = q(:, 1 + (i));
    q_p_i = q_p(:, 1 + (i));
    q_pp_i = q_pp(:, 1 + (i));

    x_i = [q_i; q_p_i];

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q_i);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(q_i);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    % seems useless but ensures that q_p and q_pp are inside its limits!
    g_x(1, 1 + (i+1)) = {x(:, 1 + (i)) - x_i};
    if(i < N_MPC)
        g_u(1, 1 + (i))   = {u(:, 1 + (i)) - q_pp(:, 1 + (i))};
    end
end

g = [g_x, g_u];

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

if isempty(yt_indices)
    J_yt = 0;
    J_yt_N = 0;
else
    J_yt   = Q_norm_square( y(yt_indices, 1 + (1:N_MPC-1) ) - y_d(yt_indices, 1 + (1:N_MPC-1)), pp.Q_y( yt_indices,yt_indices) );
    J_yt_N = Q_norm_square( y(yt_indices, 1 + (  N_MPC  ) ) - y_d(yt_indices, 1 + (  N_MPC  )), pp.Q_yN(yt_indices,yt_indices) );
end

if isempty(yr_indices)
    J_yr = 0;
    J_yr_N = 0;
else
    J_yr = 0;
    for i=1:N_MPC
        % R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))';
        % q_y_yr_err = rotm2quat_v4_casadi(R_y_yr);
        %q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)]; %ungenau aber schneller (flipping?)
        
        q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(y_d(4:7, 1 + (i))));
        
        if(i < N_MPC)
            J_yr = J_yr + Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_y(3+yr_indices,3+yr_indices)  );
        else
            J_yr_N = Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_yN(3+yr_indices,3+yr_indices)  );
        end
    end
end

qq_prev = reshape(x_prev(1:n_red, :), n_red*(N_MPC+1), 1);
qq_prev_p  = S_v * qq_prev;
qq_prev_pp = S_a * qq_prev;

q_prev_p = reshape(qq_prev_p, n_red, N_MPC+1);
q_prev_pp = reshape(qq_prev_pp, n_red, N_MPC+1);

J_q_pp = Q_norm_square(u, pp.R_q_pp(n_indices, n_indices)); %Q_norm_square(u, pp.R_u);

x_err = [q - x_prev(1:n_red,:); q_p];

J_x0 = Q_norm_square(x_err(:, 1 + (0)),       pp.R_x0(n_x_indices, n_x_indices));
J_x  = Q_norm_square(x_err(:, 1 + (1:N_MPC)), pp.R_x(n_x_indices, n_x_indices));

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp, J_x, J_x0}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);