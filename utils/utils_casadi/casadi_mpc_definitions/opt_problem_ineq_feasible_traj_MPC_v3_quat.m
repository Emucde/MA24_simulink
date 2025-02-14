% MPC v3: Optimization problem 

% States:
% - x_n: predicted states
% - z_n^ref: Reference trajectory (predicted future references)

% Parameters:
% - x_k: Current system state
% - [y_d, y_p,d]: Stacked vector containing:
%   - y_d: desired trajectory (used to calculate control law)
%   - y_p,d: desired velocity (derivative of y_d)

% Cost function (J_d,N):
% Minimizes the deviation between predicted outputs (y_n) and the desired trajectory (y_d) over a prediction horizon (N)
% J_d,N(k, x_k, (u_n)) = sum( ...                   % Loop over prediction steps (n=1 to N-1)
%   [ ||y_n       - y_ref_n ||_2^2 * Q_y; ...       % Deviation between predicted output and desired trajectory
%     ||alpha_n   - y_d_pp_n||_2^2 * I; ...         % Penalty on difference between acceleration reference (alpha_n) and desired acceleration (y_d_pp_n)
%     ||y_ref_p,n - y_p,d_n ||_2^2 * Q_y_d_p ] ...  % Deviation between reference velocity and desired velocity
%     ||y_ref_n   - y_d_n   ||_2^2 * Q_y_d ] ...    % Deviation between reference position and desired position
% );

% Control Law:
% Optimal control input sequence (u_n) minimizes the cost function (J_d,N), which is based on the desired trajectory (y_d).

% Optimization Problem formulation (MPC)
% min J_d,N(k, x_k, (u_n))
% s.t.  x_(n+1) = F(x_n, u_n)                   - System dynamics constraint
%       z_ref_(n+1) = H(z_ref_n, alpha_n)       - Reference trajectory dynamics constraint
%       x_0 = x_k                               - Initial state constraint
%       z_k = [y_d^0, y_p,d^0] OR [y_k, y_p,k]  - Initial desired state constraint
%       [q_n, q_p,n] = x_n                      - Relationship between state and predicted outputs
%       [y_ref_n, y_ref_p,n] = z_ref_n          - Reference trajectory linked to desired state
%       y_n = h(q_n)                            - Output calculation constraint
%       ||y_N - y_N_ref|| <= eps                - Terminal output constraint
%       x_n ∈ X                                 - State constraint
%       u_n ∈ U                                 - Control input constraint

% Note: Simplified notation used for clarity. Define functions and matrices explicitly.

import casadi.*

% get weights from "init_MPC_weight.m"
param_weight_init = param_weight.(casadi_func_name);

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init); % Matrizen sind keine Diagonlmatrizen [TODO]
else % hardcoded weights
    pp = param_weight_init;
end

yt_indices = param_robot.yt_indices;
yr_indices = param_robot.yr_indices;

n = param_robot.n_DOF; % Dimension of joint space
n_red = param_robot.n_red; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space [TODO: not correct, m = numel(yt_indices) + numel(yr_indices)]

n_x_indices = [n_indices n_indices+n];
n_y_indices = [yt_indices 3+yr_indices];
n_z_indices = [yt_indices 3+yt_indices 7:13]; % weil nur der quaternionenfehler mit yr_indices gewichtet wird, aber quaternionen immer 4 dim sind.

% Robot model Forward Dynamics: d/dt x = f(x, u)
gravity = false;
use_aba = true;
[f, compute_tau_fun, gravity_fun, hom_transform_endeffector_py_fun, quat_endeffector_py_fun] = ...
    load_robot_dynamics(input_dir, n, gravity, use_aba);

[~, ~, Q] = quat_deriv(ones(4,1), ones(3,1), ones(3,1)); % get function handle

q_red    = SX.sym( 'q',     n_red, 1 );
q_red_p  = SX.sym( 'q_p',   n_red, 1 );
q_red_pp = SX.sym( 'q_pp',  n_red, 1 );
x_red    = SX.sym( 'x',   2*n_red, 1 );
tau_red  = SX.sym( 'tau',   n_red, 1 );

q_subs    = SX(q_0); % assumption: all trajectories have same joint values for locked joints!
q_subs_p  = SX(q_0_p); % assumption: all trajectories have same joint values for locked joints!
q_subs_pp = SX(q_0_pp); % assumption: all trajectories have same joint values for locked joints!
x_subs    = SX([q_0; q_0_p]);

q_subs(n_indices)    = q_red;
q_subs_p(n_indices)  = q_red_p;
q_subs_pp(n_indices) = q_red_pp;
x_subs([n_indices n_indices+n]) = x_red;

H_red = Function('H_red', {q_red}, {hom_transform_endeffector_py_fun(q_subs)});
quat_fun_red = Function('quat_fun_red', {q_red}, {quat_endeffector_py_fun(q_subs)});

tau_subs = gravity_fun(x_subs(1:n)); % assumption: PD controller ensures gravity compensation on fixed joints
tau_subs(n_indices) = tau_red; % only actuated joints are controlled
d_dt_x = f(x_subs, tau_subs);
tau_full = compute_tau_fun(q_subs, q_subs_p, q_subs_pp);
g_vec_subs = gravity_fun(q_subs);
g_fun_red = Function('g_fun_red', {q_red}, {g_vec_subs(n_indices)});

tau_fun_red = Function('tau_fun_red', {q_red, q_red_p, q_red_pp}, {tau_full(n_indices)});
f_red = Function('f_red', {x_red, tau_red}, {d_dt_x([n_indices n_indices+n])});

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
N_step = pp.N_step;

DT_ctl = param_global.Ta/M;
DT = N_step * DT_ctl; % = Ts_MPC
DT2 = if_else(N_step > 1, DT - DT_ctl, DT_ctl); % = (N_step_MPC - 1) * DT_ctl = (N_MPC-1) * Ta/M = Ts_MPC - Ta

F_kp1 = integrate_casadi(f_red, DT_ctl, M, int_method); % runs with Ta from sensors
F2 = integrate_casadi(f_red, DT2, M, int_method); % runs with Ts_MPC-Ta
F = integrate_casadi(f_red, DT, M, int_method); % runs with Ts_MPC

%% Calculate Initial Guess
%Get trajectory data for initial guess
time_points = SX(1, N_MPC+1);
time_points(1:2) = [0; DT_ctl];

for i = 0:N_MPC-2
    time_points(i+3) = DT_ctl + DT2 + i * DT; % Concatenate each term
end

MPC_traj_indices = time_points/DT_ctl + 1;
MPC_traj_indices_fun = Function('MPC_traj_indices_fun', {N_step}, {MPC_traj_indices});
dt_int_arr = (MPC_traj_indices(2:end) - MPC_traj_indices(1:end-1))*DT_ctl;

MPC_traj_indices_val = round(full(MPC_traj_indices_fun(N_step_MPC)));
p_d_0    = param_trajectory.p_d(    1:3, MPC_traj_indices_val ); % (y_0 ... y_N)
p_d_p_0  = param_trajectory.p_d_p(  1:3, MPC_traj_indices_val ); % (y_p_0 ... y_p_N)
p_d_pp_0 = param_trajectory.p_d_pp( 1:3, MPC_traj_indices_val ); % (y_pp_0 ... y_pp_N)

q_d_0       = param_trajectory.q_d(       1:4, MPC_traj_indices_val ); % (q_0 ... q_N)
omega_d_0   = param_trajectory.omega_d(   1:3, MPC_traj_indices_val ); % (omega_0 ... omega_N)
omega_d_p_0 = param_trajectory.omega_d_p( 1:3, MPC_traj_indices_val ); % (omega_p_0 ... omega_p_N)

% initial guess for reference trajectory
y_d_0    = [p_d_0;    q_d_0   ];
y_d_p_0  = [p_d_p_0;  omega_d_0 ];
y_d_pp_0 = [p_d_pp_0; omega_d_p_0];

% Robot System: Initial guess

q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m
u_k_0  = full(tau_fun_red(q_0_red, q_0_red_p, q_0_red_pp)); % gravity compensation

u_init_guess_0 = ones(1, N_MPC).*u_k_0; % fully actuated
q_pp_init_guess_0 = zeros(n_red, 1); % initial guess for joint acceleration
x_init_guess_0 = ones(1, N_MPC+1).*x_0_0;

opt = struct;
opt.allow_free = true;

% Discrete yt_ref system
if(isempty(yt_indices))
    zt_init_guess_0 = [];
    alpha_t_init_guess_0 = [];
    alpha_t_N_0 = [];
    zt_0_0 = [];
    n_yt = 0;
    eps_t_cnt = 0;
    n_alpha_t = 0;
else
    eps_t_cnt = 1; % extra inequation constraint for terminal constraint

    n_yt = numel(yt_indices);
    n_alpha_t = n_yt;
    n_zt = 2*n_yt;

    zt = SX.sym('zt', n_zt);
    alpha_t_red = SX.sym('alpha_t_red', n_yt);

    ht_ref_red = Function('ht_ref_red', {zt, alpha_t_red}, {[zt(1+n_yt:2*n_yt); alpha_t_red]});

    % Translational part
    Ht = integrate_casadi(ht_ref_red, DT, M, int_method);
    Ht_kp1 = integrate_casadi(ht_ref_red, DT_ctl, M, int_method); % runs with Ta from sensors
    Ht2 = integrate_casadi(ht_ref_red, DT2, M, int_method); % runs with Ts_MPC-Ta

    % Ref System: Translational init guess
    zt_0_0               = [y_d_0(  yt_indices,  1); y_d_p_0(yt_indices,1)]; % init for zt_ref
    alpha_t_init_guess_0 =  y_d_pp_0(yt_indices, :);

    zt_init_guess_0 = [y_d_0(  yt_indices, :); y_d_p_0(yt_indices, :)];
end

if(isempty(yr_indices))
    zr_init_guess_0 = [];
    alpha_r_init_guess_0 = [];
    alpha_r_N_0 = [];
    zr_0_0 = [];
    eps_r_cnt = 0;
    n_zr = 0;
    n_alpha_r = 0;
else
    eps_r_cnt = 1; % extra inequation constraint for terminal constraint
    n_zr = 7;
    n_alpha_r = 3;

    % it is not possible to use a lower number of indices for the rotational part
    % even if numel(yr_indices) < 3, because the quaternion must have 4 elements

    zr = SX.sym('zr', n_zr);
    alpha_r = SX.sym('alpha_r', n_alpha_r);
    hr_ref = Function('h_qw_ref', {zr, alpha_r}, {[Q(zr(1:4)) * zr(5:7); alpha_r]});

    quat_r = SX.sym('quat_r', 4);
    omega_r = SX.sym('omega_r', 3);
    H_norm = Function('H_norm', {quat_r, omega_r}, {[if_else(norm_2(quat_r) == 0, SX([1; 0; 0; 0]), quat_r/norm_2(quat_r)); omega_r]});

    Hr_unorm = integrate_casadi(hr_ref, DT, M, int_method);
    Hr_val = Hr_unorm(zr, alpha_r); % ensure unit quaternion: normalize
    Hr = Function('Hr', {zr, alpha_r}, {[Hr_val(1:4)/norm_2(Hr_val(1:4)); Hr_val(5:7)]}, opt);
    % Hr = Function('H_qw', {zr, alpha_r}, {H_norm(Hr_val(1:4), Hr_val(5:7))});

    Hr_unorm = integrate_casadi(hr_ref, DT_ctl, M, int_method); % runs with Ta from sensors
    Hr_val = Hr_unorm(zr, alpha_r); % ensure unit quaternion: normalize
    Hr_kp1 = Function('Hr_kp1', {zr, alpha_r}, {[Hr_val(1:4)/norm_2(Hr_val(1:4)); Hr_val(5:7)]}, opt);
    % Hr_kp1 = Function('H_qw', {zr, alpha_r}, {H_norm(Hr_val(1:4), Hr_val(5:7))});

    Hr_unorm = integrate_casadi(hr_ref, DT2, M, int_method); % runs with Ts_MPC-Ta
    Hr_val = Hr_unorm(zr, alpha_r); % ensure unit quaternion: normalize
    Hr2 = Function('Hr2', {zr, alpha_r}, {[Hr_val(1:4)/norm_2(Hr_val(1:4)); Hr_val(5:7)]}, opt);
    % Hr2 = Function('H_qw', {zr, alpha_r}, {H_norm(Hr_val(1:4), Hr_val(5:7))});

    % Ref System: Rotational init guess
    zr_0_0               = [y_d_0(4:7,1); y_d_p_0(4:6,1)]; % init for zr_ref
    alpha_r_init_guess_0 = y_d_pp_0(4:6, :);

    zr_init_guess_0     = [y_d_0(4:7, :); y_d_p_0(4:6, :)];

    % Da bei den meisten Trajektorien die Orientierung konstant ist, kann der Solver hier oft
    % keine Lösung finden, da die Sensitivitäten exakt 0 sind. Komischerweise ist hier nur die
    % der letzte Wert betroffen. Daher wird hier ein minimaler Wert addiert.
    zr_init_guess_0(1:4, 6) = zr_init_guess_0(1:4, 6) + 1e-15;
end

z = SX.sym('z', n_zt + n_zr);
alpha = SX.sym('alpha', n_alpha_r + n_alpha_t);
H = Function('H', {z, alpha}, {[Ht(z(1:n_zt), alpha(1:n_alpha_t)); Hr(z(n_zt+1:end), alpha(n_alpha_t+1:end))]}, opt);
H_kp1 = Function('H_kp1', {z, alpha}, {[Ht_kp1(z(1:n_zt), alpha(1:n_alpha_t)); Hr_kp1(z(n_zt+1:end), alpha(n_alpha_t+1:end))]}, opt);
H2 = Function('H2', {z, alpha}, {[Ht2(z(1:n_zt), alpha(1:n_alpha_t)); Hr2(z(n_zt+1:end), alpha(n_alpha_t+1:end))]}, opt);

n_y = n_alpha_t + n_alpha_r;
n_alpha = n_y;
n_z = n_zt + n_zr;


% total init guess for ref system
z_init_guess_0     = [zt_init_guess_0;      zr_init_guess_0     ];
alpha_init_guess_0 = [alpha_t_init_guess_0; alpha_r_init_guess_0];
z_0_0              = [zt_0_0;               zr_0_0              ];

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(z_init_guess_0)+numel(alpha_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(z_init_guess_0)+numel(u_init_guess_0)+eps_t_cnt+eps_r_cnt, 1); % + 1 wegen eps

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); z_init_guess_0(:); alpha_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

if(any(isnan(full(init_guess_0))))
    error('init_guess_0 contains NaN values!');
end

% Optimization Variables:
u     = SX.sym( 'u',           n_red,     N_MPC   );
x     = SX.sym( 'x',         2*n_red,     N_MPC+1 );
z     = SX.sym( 'z',             n_z,     N_MPC+1 );
alpha = SX.sym( 'alpha',         n_alpha, N_MPC+1 );

mpc_opt_var_inputs = {u, x, z, alpha};

N_opt = numel(u) + numel(x) + numel(z) + numel(alpha);
N_u = numel(u);
N_x = numel(x);
N_z = numel(z);
u_opt_indices = 1:n_red; % tau_0

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); -Inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1); -Inf(size(z(:))); -Inf(size(alpha(:)))];
ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1);  Inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1);  Inf(size(z(:)));  Inf(size(alpha(:)))];

% input parameter
x_k  = SX.sym( 'x_k',  2*n_red,    1 ); % current x state = initial x state
z_k  = SX.sym( 'z_k',  n_z,        1 ); % current z state = initial z state z = [yt; yr]

y_d    = SX.sym( 'y_d',    m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d
y_d_p  = SX.sym( 'y_d_p',  m,   N_MPC+1 ); % (y_d_p_0 ... y_d_p_N)
y_d_pp = SX.sym( 'y_d_pp', m,   N_MPC+1 ); % (y_d_pp_0 ... y_d_pp_N)

x_prev    = SX.sym( 'x_prev', size(x));
z_prev    = SX.sym( 'z_prev', size(z));
alpha_prev = SX.sym( 'alpha_prev', size(alpha));
u_prev = SX.sym( 'u_prev', size(u) );

mpc_parameter_inputs = {x_k, z_k, y_d, y_d_p, y_d_pp, u_prev, x_prev, z_prev, alpha_prev};
mpc_init_reference_values = [x_0_0(:); z_0_0(:); y_d_0(:); y_d_p_0(:); y_d_pp_0(:); u_init_guess_0(:); x_init_guess_0(:); z_init_guess_0(:); alpha_init_guess_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+1); % for F
g_z = cell(1, N_MPC+1); % for H
g_eps = cell(1, eps_t_cnt+eps_r_cnt); % separate for transl and rotation
g_u_prev = cell(1, N_MPC); % for u_prev

if(weights_and_limits_as_parameter)
    lbg = SX(numel(lam_g_init_guess_0), 1);
    ubg = SX(numel(lam_g_init_guess_0), 1);
else
    lbg = zeros(numel(lam_g_init_guess_0), 1);
    ubg = zeros(numel(lam_g_init_guess_0), 1);
end

if(~isempty(yt_indices) && isempty(yr_indices))
    lbg(end) = 0;
    ubg(end) = pp.epsilon_t;
elseif(isempty(yt_indices) && ~isempty(yr_indices))
    lbg(end) = 0;
    ubg(end) = pp.epsilon_r;
elseif(~isempty(yt_indices) && ~isempty(yr_indices))
    lbg(end-1:end) = [0; 0];
    ubg(end-1:end) = [pp.epsilon_t; pp.epsilon_r];
end

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y    = SX( m+1, N_MPC+1 ); % transl. TCP position:      (y_0 ... y_N)
q_pp = SX( n_red, N_MPC ); % joint acceleration: (q_pp_0 ... q_pp_N-1) % last makes no sense: q_pp_N depends on u_N, wich is not known
q_p = SX( n_red, N_MPC+1 ); % joint velocity

% reference trajectory values
yt_ref    = SX( n_yt, N_MPC+1 ); % TCP position:      (yt_ref_0 ... yt_ref_N)
yt_p_ref  = SX( n_yt, N_MPC+1 ); % TCP velocity:      (yt_p_ref_0 ... yt_p_ref_N)
yt_pp_ref = SX( n_yt, N_MPC+1 ); % TCP acceleration:  (yt_pp_ref_0 ... yt_pp_ref_N)

yr_ref    = SX( 4, N_MPC+1 ); % TCP orientation:                (y_qw_ref_0 ... y_qw_ref_N)
yr_p_ref  = SX( 3, N_MPC+1 ); % TCP orientation velocity:      (y_qw_p_ref_0 ... y_qw_p_ref_N)
yr_pp_ref = SX( 3, N_MPC+1 ); % TCP orientation acceleration:  (y_qw_pp_ref_0 ... y_qw_pp_ref_N)

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_vec = SX(n_red, N_MPC);

g_x(1, 1 + (0))  = {x_k - x(:, 1 + (0))}; % x0 = xk
g_z(1, 1 + (0))  = {z_k - z(:, 1 + (0))}; % z0 = zk
for i=0:N_MPC
    % calculate q (q_0 ... q_N) and q_p values (q_p_0 ... q_p_N)
    q = x(1:n_red, 1 + (i));
    q_p(:, 1 + (i)) = x(n_red+1:end, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(q);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(~isempty(yt_indices))
        yt_ref(   :, 1 + (i)) = z(      1:n_yt,        1 + (i));
        yt_p_ref( :, 1 + (i)) = z(      1+n_yt:2*n_yt, 1 + (i));
        yt_pp_ref(:, 1 + (i)) = alpha(  1:n_yt,        1 + (i));
    end

    if(~isempty(yr_indices))
        yr_ref(   1:4, 1 + (i)) = z(     n_yt + [1:4], 1 + (i));
        yr_p_ref( 1:3, 1 + (i)) = z(     n_yt + [5:7], 1 + (i));
        yr_pp_ref(1:3, 1 + (i)) = alpha( n_yt + [1:3], 1 + (i));
    end

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        if(i == 0)
            g_x(1, 1 + (i+1))  = { F_kp1(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk = x(t0) = tilde x0 to xk+1 = x(t0+Ta)
            g_z(1, 1 + (i+1))  = { H_kp1(  z(:, 1 + (i)), alpha(:, 1 + (i)) ) - z( :, 1 + (i+1)) }; % Set the refsys constraints for zk = z(t0) = tilde z0 to zk+1 = z(t0+Ta)
        elseif(i == 1)
            g_x(1, 1 + (i+1))  = { F2(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk+1 = x(t0+Ta) to x(t0+Ts_MPC) = tilde x1
            g_z(1, 1 + (i+1))  = { H2(  z(:, 1 + (i)), alpha(:, 1 + (i)) ) - z( :, 1 + (i+1)) }; % Set the refsys constraints for zk+1 = z(t0+Ta) to z(t0+Ts_MPC) = tilde z1
        else
            g_x(1, 1 + (i+1))  = { F(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for x(t0+Ts_MPC*i) to x(t0+Ts_MPC*(i+1))
            g_z(1, 1 + (i+1))  = { H(  z(:, 1 + (i)), alpha(:, 1 + (i)) ) - z( :, 1 + (i+1)) }; % Set the refsys constraints for z(t0+Ts_MPC*i) to z(t0+Ts_MPC*(i+1))
        end

        dx   = f_red(x(:, 1 + (i)), u(:, 1 + (i))); % = [d/dt q, d^2/dt^2 q], Alternativ: Differenzenquotient
        q_pp(:, 1 + (i)) = dx(n_red+1:2*n_red, 1);
        g_vec(:, 1 + (i)) = g_fun_red(q);
        g_u_prev(1, 1 + (i)) = {u(:, 1 + (i)) - u_prev(:, 1 + (i))};
    end
end

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(diag(Q), z));

if isempty(yt_indices)
    J_yt = 0;
    gt_eps = [];
else
    J_yt = Q_norm_square( y(yt_indices, 1 + (1:N_MPC-1) ) - yt_ref(:, 1 + (1:N_MPC-1)), pp.Q_y(yt_indices)  );
    gt_eps = norm_2( (y(yt_indices,end) - yt_ref(:,end)) );

    et_pp = yt_pp_ref( :, 1 + (0:N_MPC) ) - y_d_pp( yt_indices, 1 + (0:N_MPC) );
    et_p  = yt_p_ref(  :, 1 + (0:N_MPC) ) - y_d_p(  yt_indices, 1 + (0:N_MPC) );
    et    = yt_ref(    :, 1 + (0:N_MPC) ) - y_d(    yt_indices, 1 + (0:N_MPC) );
end

if isempty(yr_indices)
    J_yr = 0;
    gr_eps = [];
else
    J_yr = 0;
    for i=1:N_MPC
        q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(yr_ref(:, 1 + (i))));

        if(i < N_MPC)
            J_yr = J_yr + Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_y(3+yr_indices)  );
        else
            gr_eps = norm_2( q_y_yr_err(1+yr_indices) );
        end
    end

    er_pp = yr_pp_ref( yr_indices, 1 + (0:N_MPC) ) - y_d_pp( 3+yr_indices, 1 + (0:N_MPC) );
    er_p  = yr_p_ref(  yr_indices, 1 + (0:N_MPC) ) - y_d_p(  3+yr_indices, 1 + (0:N_MPC) );
    er    = SX(numel(yr_indices), N_MPC+1);
    for i=0:N_MPC
        % R_yr_yd = quat2rotm_v2(yr_ref(:, 1 + (i))) * quat2rotm_v2(y_d(4:7, 1 + (i)))';
        %q_err = rotation2quaternion_casadi( R_yr_yd );
        % q_yr_yd_err = [1; R_yr_yd(3,2) - R_yr_yd(2,3); R_yr_yd(1,3) - R_yr_yd(3,1); R_yr_yd(2,1) - R_yr_yd(1,2)];
        q_yr_yd_err = quat_mult(yr_ref(:, 1 + (i)), quat_inv(y_d(4:7, 1 + (i))));
        er(:, 1 + (i)) = q_yr_yd_err(1+yr_indices);
    end
end

gcnt = 1;
if(~isempty(yt_indices))
    g_eps(1, gcnt) = {gt_eps};
    gcnt = 2; % = 2 for yr if yr_indices is not empty
end

if(~isempty(yr_indices))
    g_eps(1, gcnt) = {gr_eps};
end

% jumps in tau at max 1000Nm/s
max_du = pp.max_du;
max_du_arr = repmat(max_du*dt_int_arr, n_red, 1);

lbg(1+N_x+N_z:N_x+N_z+N_u, 1) = -max_du_arr(:);
ubg(1+N_x+N_z:N_x+N_z+N_u, 1) =  max_du_arr(:);

% Todo g_zt, g_zr wenn yt_indices od yr_indices leer sind geht nicht
g = [g_x, g_z, g_u_prev, g_eps]; % merge_cell_arrays([g_x, g_z, g_eps], 'vector')';

u_err = u-g_vec; % es ist stabiler es nicht gegenüber der vorherigen Lösung zu gewichten!

J_u  = Q_norm_square(u_err, pp.R_u(n_indices));
J_q_ref = Q_norm_square(x(1:n_red, :) - pp.q_ref(n_indices), pp.R_q_ref(n_indices));
J_q_p  = Q_norm_square(q_p, pp.R_q_p(n_indices));

J_x_prev     = Q_norm_square(x     - x_prev,     pp.R_x_prev(n_x_indices));
J_z_prev     = Q_norm_square(z     - z_prev,     pp.R_z_prev(n_z_indices));
J_alpha_prev = Q_norm_square(alpha - alpha_prev, pp.R_alpha_prev(n_y_indices));

% it is really important to only weight the first control input!
J_u0_prev = Q_norm_square(u(:, 1) - u_prev(:, 1), pp.R_u0_prev(n_indices));

Jt_yy_ref = Q_norm_square( alpha(yt_indices, :)        - y_d_pp(yt_indices,   :), pp.Q_y(yt_indices) );
Jr_yy_ref = Q_norm_square( alpha(n_yt + yr_indices, :) - y_d_pp(3+yr_indices, :), pp.Q_y(3+yr_indices) );
J_alpha = Jt_yy_ref + Jr_yy_ref;

cost_vars_names = '{J_yt, J_yr, J_u, J_q_ref, J_q_p, J_alpha, J_u0_prev, J_x_prev, J_z_prev, J_alpha_prev}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);