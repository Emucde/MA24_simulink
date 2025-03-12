% MPC v4: Optimization problem 

import casadi.*

% get weights from "init_MPC_weight.m"
param_weight_init = param_weight.(casadi_func_name);

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init); % Matrizen sind keine Diagonlmatrizen [TODO]
else % hardcoded weights
    pp = param_weight_init;
end

implicit_xk = false; % false liefert robustere lösungen, true ist etwas schneller aber führt oft zu numerischen noise

diff_variant_mode = struct;
diff_variant_mode.numdiff = 1; % default forward, central, backward deviation, tens to peaks, does not work with implicit_xk=true
diff_variant_mode.savgol = 2; % savgol filtering and deviation
diff_variant_mode.savgol_v2 = 3; % savgol filtering without additional equations and deviation
diff_variant_mode.savgol_not_equidist = 4; % savgol filtering, non equidistant samples, for feedforward
diff_variant_mode.savgol_not_equidist_noise_supr = 5; % savgol filtering without additional equations and deviation
diff_variant_mode.savgol_not_equidist_combined = 6; % savgol filtering, non equidistant samples, for pd control, first derivatve approximated
diff_variant_mode.savgol_not_equidist_combined2 = 7; % savgol filtering, non equidistant samples, for pd control, first derivatve approximated
diff_variant_mode.diff_filter = 8; % differential filter

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
N_step = pp.N_step;

DT_ctl = pp.dt/M;
DT = N_step * DT_ctl; % = Ts_MPC
DT2 = if_else(N_step > 1, DT - DT_ctl, DT_ctl); % = (N_step_MPC - 1) * DT_ctl = (N_MPC-1) * Ta/M = Ts_MPC - Ta

%% Calculate Initial Guess
%Get trajectory data for initial guess
time_points = SX(1, N_MPC+1);
time_points(1:2) = [0; DT_ctl];

for i = 0:N_MPC-2
    time_points(i+3) = DT_ctl + DT2 + i * DT; % Concatenate each term
end

MPC_traj_indices = time_points/DT_ctl + 1;
MPC_traj_indices_fun = Function('MPC_traj_indices_fun', {N_step, DT_ctl}, {MPC_traj_indices});
dt_int_arr = (MPC_traj_indices(2:end) - MPC_traj_indices(1:end-1))*DT_ctl;

MPC_traj_indices_val = round(full(MPC_traj_indices_fun(N_step_MPC, param_global.Ta)));
p_d_0 = param_trajectory.p_d( 1:3, MPC_traj_indices_val ); % (y_0 ... y_N)
q_d_0 = param_trajectory.q_d( 1:4, MPC_traj_indices_val ); % (q_0 ... q_N)
y_d_0 = [p_d_0; q_d_0];

% Robot System: Initial guess
q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m

u_k_0  = q_0_red_pp;

u_init_guess_0 = ones(n_red, N_MPC).*u_k_0;
x_init_guess_0 = ones(2*n_red, N_MPC+1).*x_0_0;

lam_x_init_guess_0 = zeros(numel(u_init_guess_0) + numel(x_init_guess_0), 1);
if(implicit_xk)
    lam_g_init_guess_0 = zeros(2*numel(u_init_guess_0) + numel(x_init_guess_0(1+n_red:2*n_red, 2:end)) + numel(x_0_0), 1);
else
    lam_g_init_guess_0 = zeros(2*numel(u_init_guess_0) + numel(x_init_guess_0(1+n_red:2*n_red, :)) + numel(x_0_0), 1);
end

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

if(any(isnan(full(init_guess_0))))
    error('init_guess_0 contains NaN values!');
end

% Optimization Variables:
u     = SX.sym( 'u',    n_red,   N_MPC   );
x     = SX.sym( 'x',    2*n_red, N_MPC+1 );

% confusing, but I need equation constraints to relate x to q and qp = Sv q.
% only then the joint and velocity limits are maintained in!
mpc_opt_var_inputs = {u, x};

%u_opt_indices = 1:n_red;

N_u = numel(u);
N_x = numel(x);

u_idx = [1 : numel(u)];
x_idx = N_u + [1 : numel(x)];

q0_pp_idx = u_idx(1:n_red);
x1_idx = x_idx(1+2*n_red : 4*n_red);
q1_pp_idx = u_idx(1+n_red : 2*n_red);
u_opt_indices = [q0_pp_idx, x1_idx, q1_pp_idx];

% TODO: So kann man q_p eigentlich nicht limitieren!!! vgl. mpc v8

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1);  inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1)];
lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); -inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1)];

% Interessant: Wenn u auch im Horizont beschränkt wird, erhält man eine bleibende Regelabweichung (1e-5).
% Hingegen wenn die u im Horizont nicht beschränkt werden, sondern nur das, dass ausgegeben wird, hat man das Problem nicht:
% ubw = [repmat(pp.u_max(n_indices), 1, 1);  inf(n_red*(N_MPC-1),1);  inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1)];
% lbw = [repmat(pp.u_min(n_indices), 1, 1); -inf(n_red*(N_MPC-1),1); -inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1)];

% input parameter
x_k  = SX.sym( 'x_k',  2*n_red,       1 ); % current x state = initial x state
y_d  = SX.sym( 'y_d',  m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d
x_prev = SX.sym( 'x_prev', 2*n_red, N_MPC+1 );
u_prev = SX.sym( 'u_prev', size(u) );

mpc_parameter_inputs = {x_k, y_d, x_prev, u_prev};
mpc_init_reference_values = [x_0_0(:); y_d_0(:); x_init_guess_0(:); u_init_guess_0(:)];

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
g_u  = cell(1, N_MPC);
g_u_prev = cell(1, N_MPC);

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
elseif(diff_variant == diff_variant_mode.diff_filter)
    warning('Das Ergebnis passt erst nach n_order steps, es sei denn man initialisiert die Filter mit dem korrekten Wert, den es vor n_order steps hat.');
    param_global_DT.Ta = DT; % Ts_MPC
    param_global_DT2.Ta = DT2; % DT - DT_ctl
    param_global_ctl.Ta = DT_ctl; % Ta
    lambda = -1000;
    traj_struct_DT_ctl = create_param_diff_filter(struct, param_global_ctl, 'lambda (1/s)', lambda, 'n_order', 3, 'n_input', 6, 'diff_filter_jointspace');
    traj_struct_DT2 = create_param_diff_filter(struct, param_global_DT2, 'lambda (1/s)', lambda, 'n_order', 3, 'n_input', 6, 'diff_filter_jointspace');
    traj_struct_DT = create_param_diff_filter(struct, param_global_DT, 'lambda (1/s)', lambda, 'n_order', 3, 'n_input', 6, 'diff_filter_jointspace');

    Phi_DT_ctl = traj_struct_DT_ctl.diff_filter_jointspace.Phi;
    Gamma_DT_ctl = traj_struct_DT_ctl.diff_filter_jointspace.Gamma;

    Phi_DT2 = traj_struct_DT2.diff_filter_jointspace.Phi;
    Gamma_DT2 = traj_struct_DT2.diff_filter_jointspace.Gamma;

    Phi_DT = traj_struct_DT.diff_filter_jointspace.Phi;
    Gamma_DT = traj_struct_DT.diff_filter_jointspace.Gamma;

    q_filt_index = double(traj_struct_DT_ctl.diff_filter_jointspace.p_d_index);
    q_p_filt_index = double(traj_struct_DT_ctl.diff_filter_jointspace.p_d_p_index);
    q_pp_filt_index = double(traj_struct_DT_ctl.diff_filter_jointspace.p_d_pp_index);
else
    error('invalid mode');
end

%      x = [q(t0),   q(t1), ...  q(tN),  dq(t0),  dq(t1), ...  dq(tN)] = [qq;   qq_p ]
% d/dt x = [dq(t0), dq(t1), ... dq(tN), ddq(t0), ddq(t1), ... ddq(tN)] = [qq_p; qq_pp]

% p_d_0 = param_trajectory.p_d( 1:3, MPC_traj_indices ); % (y_0 ... y_N)
% p_d_p_0 = param_trajectory.p_d_p( 1:3, MPC_traj_indices ); % (y_0 ... y_N)
% p_d_pp_0 = param_trajectory.p_d_pp( 1:3, MPC_traj_indices ); % (y_0 ... y_N)

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

% q = zeros( 3, N_MPC+1 );
% q_p = zeros( 3, N_MPC+1 );
% q_pp = zeros( 3, N_MPC+1 );
% x_0 = [ p_d_0(:,1)'; zeros(3-1, 3)];
% % x_0 = [ p_d_0(:,1)'; p_d_p_0(:,1)'; p_d_pp_0(:,1)'];
% % x_0 = [ param_trajectory.p_d( 1:3, MPC_traj_indices(1)-3 )'; param_trajectory.p_d_p( 1:3, MPC_traj_indices(1)-3 )'; param_trajectory.p_d_pp( 1:3, MPC_traj_indices(1)-3 )'];

% x_k_i = x_0(:);

% % calculate derivative values (q_p, q_pp)
% for i=0:N_MPC
%     if(i == 0)
%         x_k_i = Phi_DT_ctl*x_k_i + Gamma_DT_ctl*p_d_0(:, 1 + (i));
%     elseif(i == 1)
%          x_k_i = Phi_DT2*x_k_i + Gamma_DT2*p_d_0(:, 1 + (i));
%     else
%          x_k_i = Phi_DT*x_k_i + Gamma_DT*p_d_0(:, 1 + (i));
%     end
%         q(:, 1 + (i)) = x_k_i(q_filt_index);
%         q_p(:, 1 + (i)) = x_k_i(q_p_filt_index);
%         q_pp(:, 1 + (i)) = x_k_i(q_pp_filt_index);
% end
% % plot(q');hold on;plot(p_d_0')
% plot(q_p');hold on;plot(p_d_p_0')
% % plot(q_pp');hold on;plot(p_d_pp_0')

if(diff_variant == diff_variant_mode.diff_filter)
    q = SX( n_red, N_MPC+1 );
    q_p = SX( n_red, N_MPC+1 );
    q_pp = SX( n_red, N_MPC+1 );
    if(implicit_xk)
        x_0 = [ x_k(1:n_red)'; x_k(1+n_red:2*n_red)'; zeros(3-2, n_red)];
    else
        x_0 = [ x(1:n_red, 1)'; x(1+n_red:2*n_red, 1)'; zeros(3-2, n_red)];
    end

    x_k_i = x_0(:);

    % calculate derivative values (q_p, q_pp)
    for i=0:N_MPC
        if(i == 0)
            x_k_i = Phi_DT_ctl*x_k_i + Gamma_DT_ctl*x(1:n_red, 1 + (i));
        elseif(i == 1)
            x_k_i = Phi_DT2*x_k_i(:) + Gamma_DT2*x(1:n_red, 1 + (i));
        else
            x_k_i = Phi_DT*x_k_i(:) + Gamma_DT*x(1:n_red, 1 + (i));
        end

        q(:, 1 + (i)) = x_k_i(q_filt_index);
        q_p(:, 1 + (i)) = x_k_i(q_p_filt_index);
        q_pp(:, 1 + (i)) = x_k_i(q_pp_filt_index);
    end
else
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
end

g_x(1, 1 + (0)) = {x_k - x(   :, 1 + (0))}; % x0 = xk

for i=0:N_MPC
    q_i = q(:, 1 + (i));
    q_p_i = q_p(:, 1 + (i));
    q_pp_i = q_pp(:, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q_i);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(q_i);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    % seems useless but ensures that q_p and q_pp are inside its limits!
    if(implicit_xk)
        if(i > 0)
            g_x(1, 1 + (i)) = {x(1+n_red:2*n_red, 1 + (i)) - q_p_i};
        end
    else
        g_x(1, 1 + (i+1)) = {x(1+n_red:2*n_red, 1 + (i)) - q_p_i};
    end
    if(i < N_MPC)
        g_u(1, 1 + (i))   = {u(:, 1 + (i)) - q_pp(:, 1 + (i))};
        g_u_prev(1, 1 + (i)) = {u(:, 1 + (i)) - u_prev(:, 1 + (i))};
    end
end

g = [g_x, g_u, g_u_prev];

% jump in tau at max 100rad/s^2
max_du = pp.max_du;
max_du_arr = repmat(max_du*dt_int_arr, n_red, 1);

lbg(1+end-N_u:end, 1) = -max_du_arr(:);
ubg(1+end-N_u:end, 1) =  max_du_arr(:);

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(diag(Q), z));

if isempty(yr_indices)
    J_yr = 0;
    J_yr_N = 0;
else
    J_yr = 0;
    for i=1:N_MPC
        % R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))';
        % % q_y_yr_err = quat_R_endeffector_py_fun(R_y_yr); # immer nan?
        % q_y_yr_err = rotm2quat_v4_casadi(R_y_yr);

        %q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)]; %ungenau aber schneller (flipping?)
        
        q_err_tmp = quat_mult(quat_inv(y_d(4:7, 1 + (i))), y(4:7, 1 + (i)));
        
        eta = q_err_tmp(1);
        epsilon = q_err_tmp(2:4);

        R_d = quat2rotm_v2(y_d(4:7, 1 + (i)));

        q_err = 2 * R_d * (eta * eye(3) + skew(epsilon)) * epsilon;
        
        if(i < N_MPC)
            J_yr = J_yr + Q_norm_square( q_err(yr_indices) , pp.Q_y(3+yr_indices)  );
        else
            J_yr_N = Q_norm_square( q_err(yr_indices) , pp.Q_yN(3+yr_indices)  );
        end
    end
end

J_x_prev = Q_norm_square(x - x_prev, pp.R_x_prev(n_x_indices)); %Q_norm_square(u, pp.R_u);

J_q_ref = Q_norm_square(x(1:n_red, :) - pp.q_ref(n_indices), pp.R_q_ref(n_indices));

J_q_p = Q_norm_square(q_p, pp.R_q_p(n_indices)); %Q_norm_square(u, pp.R_u);
J_u = Q_norm_square(u, pp.R_u(n_indices)); %Q_norm_square(u, pp.R_u);

% it is really important to only weight the first control input!
J_u0_prev = Q_norm_square(u(:, 1) - u_prev(:, 1), pp.R_u0_prev(n_indices));

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_ref, J_q_p, J_u, J_x_prev, J_u0_prev}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);