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
%       z_0 = [y_d^0, y_p,d^0] OR [y_k, y_p,k]  - Initial desired state constraint
%       [q_n, q_p,n] = x_n                      - Relationship between state and predicted outputs
%       [y_ref_n, y_ref_p,n] = z_ref_n          - Reference trajectory linked to desired state
%       y_n = h(q_n)                            - Output calculation constraint
%       ||y_N - y_N_ref|| <= eps                - Terminal output constraint
%       x_n ∈ X                                 - State constraint
%       u_n ∈ U                                 - Control input constraint

% Note: Simplified notation used for clarity. Define functions and matrices explicitly.

import casadi.*

yt_indices = param_robot.yt_indices;
yr_indices = param_robot.yr_indices;

n = param_robot.n_DOF; % Dimension of joint space
n_red = param_robot.n_red; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space [TODO: not correct, m = numel(yt_indices) + numel(yr_indices)]

% Robot model Forward Dynamics: d/dt x = f(x, u)
use_aba = false; % aba ist langsamer! (357s vs 335s)
if(use_aba)
    f = Function.load([input_dir, 'sys_fun_x_aba_py.casadi']); % forward dynamics (FD), d/dt x = f(x, u), x = [q; dq]
else
    f = Function.load([input_dir, 'sys_fun_x_sol_py.casadi']); % equivalent as above
end

compute_tau_fun = Function.load([input_dir, 'compute_tau_py.casadi']); % Inverse Dynamics (ID)
hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

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

tau_subs = compute_tau_fun(q_subs, q_subs_p, q_subs_pp);

q_subs(n_indices)    = q_red;
q_subs_p(n_indices)  = q_red_p;
q_subs_pp(n_indices) = q_red_pp;
tau_subs(n_indices)  = tau_red;

x_subs([n_indices n_indices+n]) = x_red;

H_red = Function('H_red', {q_red}, {hom_transform_endeffector_py_fun(q_subs)});
quat_fun_red = Function('quat_fun_red', {q_red}, {quat_endeffector_py_fun(q_subs)});

d_dt_x = f(x_subs, tau_subs);
tau_full = compute_tau_fun(q_subs, q_subs_p, q_subs_pp);

tau_fun_red = Function('tau_fun_red', {q_red, q_red_p, q_red_pp}, {tau_full(n_indices)});
f_red = Function('f_red', {x_red, tau_red}, {d_dt_x([n_indices n_indices+n])});

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.

F = integrate_casadi(f_red, DT, M, int_method);

DT_ctl = param_global.Ta/M;
F_kp1 = integrate_casadi(f_red, DT_ctl, M, int_method); % runs with Ta from sensors

if(N_step_MPC == 1)
    DT2 = DT_ctl; % special case if Ts_MPC = Ta
else
    DT2 = DT - DT_ctl;
end
F2 = integrate_casadi(f_red, DT2, M, int_method); % runs with Ts_MPC-Ta

%Get trajectory data for initial guess
if(N_step_MPC <= 2)
    MPC_traj_indices = 1:(N_MPC+1);
else
    MPC_traj_indices = [1, 2, N_step_MPC : N_step_MPC : 1 + (N_MPC-1) * N_step_MPC];
end

%% Calculate Initial Guess

% Get trajectory data for initial guess
p_d_0    = param_trajectory.p_d(    1:3, MPC_traj_indices ); % (y_0 ... y_N)
p_d_p_0  = param_trajectory.p_d_p(  1:3, MPC_traj_indices ); % (y_p_0 ... y_p_N)
p_d_pp_0 = param_trajectory.p_d_pp( 1:3, MPC_traj_indices ); % (y_pp_0 ... y_pp_N)

q_d_0       = param_trajectory.q_d(       1:4, MPC_traj_indices ); % (q_0 ... q_N)
omega_d_0   = param_trajectory.omega_d(   1:3, MPC_traj_indices ); % (omega_0 ... omega_N)
omega_d_p_0 = param_trajectory.omega_d_p( 1:3, MPC_traj_indices ); % (omega_p_0 ... omega_p_N)

% initial guess for reference trajectory
y_d_0    = [p_d_0;    q_d_0   ];
y_d_p_0  = [p_d_p_0;  omega_d_0 ];
y_d_pp_0 = [p_d_pp_0; omega_d_p_0];

% Robot System: Initial guess

q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m

u_k_0  = tau_fun_red(q_0_red, q_0_red_p, q_0_red_pp); % gravity compensation

u_init_guess_0 = ones(n_red, N_MPC).*u_k_0; % fully actuated

F_kp1_sim          = F_kp1.mapaccum(1);
F2_sim             = F2.mapaccum(1);
F_sim              = F.mapaccum(N_MPC-2);

x_DT_ctl_init_0 = F_kp1_sim(x_0_0,           u_init_guess_0(:, 1)    );
x_DT2_init_0    = F2_sim(   x_DT_ctl_init_0, u_init_guess_0(:, 2)    );
x_DT_init_0     = F_sim(    x_DT2_init_0,    u_init_guess_0(:, 3:end));

x_init_guess_0     = [x_0_0 full(x_DT_ctl_init_0) full(x_DT2_init_0) full(x_DT_init_0)];

% Discrete yt_ref system
if(isempty(yt_indices))
    zt_init_guess_0 = [];
    alpha_t_init_guess_0 = [];
    alpha_t_N_0 = [];
    zt_0_0 = [];
    n_yt_red = 0;
    eps_t_cnt = 0;
else
    eps_t_cnt = 1; % extra inequation constraint for terminal constraint

    n_yt_red = numel(yt_indices);
    n_zt_red = 2*n_yt_red;

    zt_red = SX.sym('zt_red', n_zt_red);
    alpha_t_red = SX.sym('alpha_t_red', n_yt_red);

    ht_ref_red = Function('ht_ref_red', {zt_red, alpha_t_red}, {[zt_red(1+n_yt_red:2*n_yt_red); alpha_t_red]});

    % Translational part
    Ht = integrate_casadi(ht_ref_red, DT, M, int_method);
    Ht_kp1 = integrate_casadi(ht_ref_red, DT_ctl, M, int_method); % runs with Ta from sensors
    Ht2 = integrate_casadi(ht_ref_red, DT2, M, int_method); % runs with Ts_MPC-Ta

    % Ref System: Translational init guess
    zt_0_0               = [y_d_0(  yt_indices, 1       ); y_d_p_0(yt_indices,1)]; % init for zt_ref
    alpha_t_init_guess_0 =  y_d_pp_0(yt_indices, 1:end-1);
    alpha_t_N_0          =  y_d_pp_0(yt_indices, end    );

    Ht_kp1_sim          = Ht_kp1.mapaccum(1);
    Ht2_sim             = Ht2.mapaccum(1);
    Ht_sim              = Ht.mapaccum(N_MPC-2);

    zt_DT_ctl_init_0 = Ht_kp1_sim(zt_0_0,           alpha_t_init_guess_0(:, 1)    );
    zt_DT2_init_0    = Ht2_sim(   zt_DT_ctl_init_0, alpha_t_init_guess_0(:, 2)    );
    zt_DT_init_0     = Ht_sim(    zt_DT2_init_0,    alpha_t_init_guess_0(:, 3:end));

    zt_init_guess_0     = [zt_0_0 full(zt_DT_ctl_init_0) full(zt_DT2_init_0) full(zt_DT_init_0)];
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

    Hr_unorm = integrate_casadi(hr_ref, DT, M, int_method);
    Hr_val = Hr_unorm(zr, alpha_r); % ensure unit quaternion: normalize
    Hr = Function('H_qw', {zr, alpha_r}, {[Hr_val(1:4)/norm_2(Hr_val(1:4)); Hr_val(5:7)]});

    Hr_unorm = integrate_casadi(hr_ref, DT_ctl, M, int_method); % runs with Ta from sensors
    Hr_val = Hr_unorm(zr, alpha_r); % ensure unit quaternion: normalize
    Hr_kp1 = Function('H_qw', {zr, alpha_r}, {[Hr_val(1:4)/norm_2(Hr_val(1:4)); Hr_val(5:7)]});

    Hr_unorm = integrate_casadi(hr_ref, DT2, M, int_method); % runs with Ts_MPC-Ta
    Hr_val = Hr_unorm(zr, alpha_r); % ensure unit quaternion: normalize
    Hr2 = Function('H_qw', {zr, alpha_r}, {[Hr_val(1:4)/norm_2(Hr_val(1:4)); Hr_val(5:7)]});

    % Ref System: Rotational init guess
    zr_0_0               = [y_d_0(4:7,1); y_d_p_0(4:6,1)]; % init for zr_ref
    alpha_r_init_guess_0 = y_d_pp_0(4:6, 1:end-1);
    alpha_r_N_0          = y_d_pp_0(4:6, end);

    Hr_kp1_sim          = Hr_kp1.mapaccum(1);
    Hr2_sim             = Hr2.mapaccum(1);
    Hr_sim              = Hr.mapaccum(N_MPC-2);

    zr_DT_ctl_init_0 = Hr_kp1_sim(zr_0_0,           alpha_r_init_guess_0(:, 1)    );
    zr_DT2_init_0    = Hr2_sim(   zr_DT_ctl_init_0, alpha_r_init_guess_0(:, 2)    );
    zr_DT_init_0     = Hr_sim(    zr_DT2_init_0,    alpha_r_init_guess_0(:, 3:end));

    zr_init_guess_0     = [zr_0_0 full(zr_DT_ctl_init_0) full(zr_DT2_init_0) full(zr_DT_init_0)];
end


% total init guess for ref system
z_init_guess_0     = [zt_init_guess_0;      zr_init_guess_0     ];
alpha_init_guess_0 = [alpha_t_init_guess_0; alpha_r_init_guess_0];
alpha_N_0          = [alpha_t_N_0;          alpha_r_N_0         ];
z_0_0              = [zt_0_0;               zr_0_0              ];

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(z_init_guess_0)+numel(alpha_init_guess_0) + numel(alpha_N_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(z_init_guess_0)+eps_t_cnt+eps_r_cnt, 1); % + 1 wegen eps

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); z_init_guess_0(:); alpha_init_guess_0(:); alpha_N_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

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
u     = SX.sym( 'u',           n_red,    N_MPC   );
x     = SX.sym( 'x',         2*n_red,    N_MPC+1 );
zt    = SX.sym( 'zt',        2*n_yt_red, N_MPC+1 );
zr    = SX.sym( 'zr',        n_zr,       N_MPC+1 );
alpha_t = SX.sym( 'alpha_t', n_yt_red,   N_MPC+1 );
alpha_r = SX.sym( 'alpha_r', n_alpha_r,  N_MPC+1 );

z = [ zt; zr ];
alpha = [ alpha_t; alpha_r ];

mpc_opt_var_inputs = {u, x, z, alpha};

u_opt_indices = 1:n_red;

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min(n_indices), N_MPC, 1); repmat(pp.x_min([n_indices, n_indices+n]), N_MPC + 1, 1); -Inf(size(z(:))); -Inf(size(alpha(:)))];
ubw = [repmat(pp.u_max(n_indices), N_MPC, 1); repmat(pp.x_max([n_indices, n_indices+n]), N_MPC + 1, 1);  Inf(size(z(:)));  Inf(size(alpha(:)))];

% input parameter
x_k  = SX.sym( 'x_k',  2*n_red,    1 ); % current x state = initial x state
zt_0 = SX.sym( 'zt_0', 2*n_yt_red, 1 ); % initial zt state
zr_0 = SX.sym( 'zr_0', n_zr,       1 ); % initial zr state

y_d    = SX.sym( 'y_d',    m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d
y_d_p  = SX.sym( 'y_d_p',  m,   N_MPC+1 ); % (y_d_p_0 ... y_d_p_N)
y_d_pp = SX.sym( 'y_d_pp', m,   N_MPC+1 ); % (y_d_pp_0 ... y_d_pp_N)
z_0 = [zt_0; zr_0];

mpc_parameter_inputs = {x_k, z_0, y_d, y_d_p, y_d_pp};
mpc_init_reference_values = [x_0_0(:); z_0_0(:); y_d_0(:); y_d_p_0(:); y_d_pp_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+1); % for F
g_zt = cell(1, N_MPC+1); % for H
g_zr = cell(1, N_MPC+1); % for H_qw
g_eps = cell(1, eps_t_cnt+eps_r_cnt); % separate for transl and rotation

if(weights_and_limits_as_parameter)
    lbg = SX(numel(x)+numel(z)+eps_t_cnt+eps_r_cnt, 1);
    ubg = SX(numel(x)+numel(z)+eps_t_cnt+eps_r_cnt, 1);
else
    lbg = zeros(numel(x)+numel(z)+eps_t_cnt+eps_r_cnt, 1);
    ubg = zeros(numel(x)+numel(z)+eps_t_cnt+eps_r_cnt, 1);
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
q_pp = SX( n_red, N_MPC   ); % joint acceleration: (q_pp_0 ... q_pp_N-1) % last makes no sense: q_pp_N depends on u_N, wich is not known

% reference trajectory values
yt_ref    = SX( n_yt_red, N_MPC+1 ); % TCP position:      (yt_ref_0 ... yt_ref_N)
yt_p_ref  = SX( n_yt_red, N_MPC+1 ); % TCP velocity:      (yt_p_ref_0 ... yt_p_ref_N)
yt_pp_ref = SX( n_yt_red, N_MPC+1 ); % TCP acceleration:  (yt_pp_ref_0 ... yt_pp_ref_N)

yr_ref    = SX( 4, N_MPC+1 ); % TCP orientation:                (y_qw_ref_0 ... y_qw_ref_N)
yr_p_ref  = SX( 3, N_MPC+1 ); % TCP orientation velocity:      (y_qw_p_ref_0 ... y_qw_p_ref_N)
yr_pp_ref = SX( 3, N_MPC+1 ); % TCP orientation acceleration:  (y_qw_pp_ref_0 ... y_qw_pp_ref_N)

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_x(1, 1 + (0))  = {x_k                    - x(:,              1 + (0))}; % x0 = xk
g_zt(1, 1 + (0)) = {z_0(1:n_zt_red, 1)     - z(1:n_zt_red,     1 + (0))}; % zt0
g_zr(1, 1 + (0)) = {z_0(n_zt_red+1:end, 1) - z(n_zt_red+1:end, 1 + (0))}; % zr0

for i=0:N_MPC
    % calculate q (q_0 ... q_N) and q_p values (q_p_0 ... q_p_N)
    q = x(1:n_red, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(q);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(~isempty(yt_indices))
        yt_ref(   :, 1 + (i)) = zt(      1:n_yt_red,            1 + (i));
        yt_p_ref( :, 1 + (i)) = zt(      1+n_yt_red:2*n_yt_red, 1 + (i));
        yt_pp_ref(:, 1 + (i)) = alpha_t( 1:n_yt_red,            1 + (i));

        if(i < N_MPC)
            g_zt(1, 1 + (i+1)) = { Ht(zt(:, 1 + (i)), alpha_t(:, 1 + (i)) ) - zt(:, 1 + (i+1)) }; % Set the yref_t dynamics constraints
        end
    end

    if(~isempty(yr_indices))
        yr_ref(   1:4, 1 + (i)) = zr( 1:4, 1 + (i));
        yr_p_ref( 1:3, 1 + (i)) = zr( 5:7, 1 + (i));
        yr_pp_ref(1:3, 1 + (i)) = alpha_r( 1:3, 1 + (i));

        if(i < N_MPC)
            g_zr(1, 1 + (i+1)) = { Hr(zr(:, 1 + (i)), alpha_r(:, 1 + (i)) ) - zr(:, 1 + (i+1)) }; % Set the yref_r dynamics constraints
        end
    end

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        g_x(1, 1 + (i+1))  = { F(  x(:, 1 + (i)), u(      :, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state dynamics constraints

        dx   = f_red(x(:, 1 + (i)), u(:, 1 + (i))); % = [d/dt q, d^2/dt^2 q], Alternativ: Differenzenquotient
        q_pp(:, 1 + (i)) = dx(n_red+1:2*n_red, 1);
    end
end

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

if isempty(yt_indices)
    J_yt = 0;
    g_zt = [];
    gt_eps = [];
    Jt_yy_ref = 0;
else
    J_yt = Q_norm_square( y(yt_indices, 1 + (1:N_MPC-1) ) - yt_ref(:, 1 + (1:N_MPC-1)), pp.Q_y(yt_indices,yt_indices)  );
    gt_eps = norm_2( (y(yt_indices,end) - yt_ref(:,end)) );

    et_pp = yt_pp_ref( :, 1 + (0:N_MPC) ) - y_d_pp( yt_indices, 1 + (0:N_MPC) );
    et_p  = yt_p_ref(  :, 1 + (0:N_MPC) ) - y_d_p(  yt_indices, 1 + (0:N_MPC) );
    et    = yt_ref(    :, 1 + (0:N_MPC) ) - y_d(    yt_indices, 1 + (0:N_MPC) );

    Jt_yy_ref = Q_norm_square( et_pp + ...
                            mtimes(pp.Q_y_p_ref(yt_indices, yt_indices), et_p) + ...
                            mtimes(pp.Q_y_ref(  yt_indices, yt_indices), et), ...
                            eye(numel(yt_indices))...
                            );
end

if isempty(yr_indices)
    J_yr = 0;
    g_zr = [];
    gr_eps = [];
    Jr_yy_ref = 0;
else
    J_yr = 0;
    er   = SX(numel(yr_indices), N_MPC+1);
    for i=1:N_MPC
        % R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(yr_ref(:, 1 + (i))))';
        % % q_y_y_err = rotation2quaternion_casadi( R_y_yr );
        % q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)]; %ungenau aber schneller (flipping?)
        q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(yr_ref(:, 1 + (i))));
        er(:, 1 + (i)) = q_y_yr_err(1+yr_indices);

        if(i < N_MPC)
            J_yr = J_yr + Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_y(3+yr_indices,3+yr_indices)  );
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

    Jr_yy_ref = Q_norm_square( er_pp + ...
                               mtimes(pp.Q_y_p_ref(3+yt_indices, 3+yt_indices), er_p) + ...
                               mtimes(pp.Q_y_ref(  3+yt_indices, 3+yt_indices), er), ...
                               eye(numel(yr_indices))...
                             );
end

gcnt = 1;
if(~isempty(yt_indices))
    g_eps(1, gcnt) = {gt_eps};
    gcnt = 2; % = 2 for yr if yr_indices is not empty
end

if(~isempty(yr_indices))
    g_eps(1, gcnt) = {gr_eps};
end

g_z = [g_zt, g_zr];
g = [g_x, g_z, g_eps]; % merge_cell_arrays([g_x, g_z, g_eps], 'vector')';

J_q_pp = Q_norm_square(q_pp, pp.R_q_pp(n_indices, n_indices)); %Q_norm_square(u, pp.R_u);

cost_vars_names = '{J_yt, Jt_yy_ref, J_yr, Jr_yy_ref, J_q_pp}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);