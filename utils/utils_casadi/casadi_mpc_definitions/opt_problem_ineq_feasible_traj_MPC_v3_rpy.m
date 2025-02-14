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
n_z_indices = [yt_indices 3+yt_indices 6+yr_indices 9+yr_indices]; % weil nur der quaternionenfehler mit yr_indices gewichtet wird, aber quaternionen immer 4 dim sind.

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

% Get trajectory data for initial guess
MPC_traj_indices_val = round(full(MPC_traj_indices_fun(N_step_MPC)));

p_d_0    = param_trajectory.p_d(    1:3, MPC_traj_indices_val ); % (y_0 ... y_N)
p_d_p_0  = param_trajectory.p_d_p(  1:3, MPC_traj_indices_val ); % (y_p_0 ... y_p_N)
p_d_pp_0 = param_trajectory.p_d_pp( 1:3, MPC_traj_indices_val ); % (y_pp_0 ... y_pp_N)

Phi_d_0 = param_trajectory.Phi_d(      1:3, MPC_traj_indices_val ); % (Phi_0 ... Phi_N)
Phi_d_p_0 = param_trajectory.Phi_d_p(  1:3, MPC_traj_indices_val ); % (Phi_p_0 ... Phi_p_N)
Phi_d_pp_0 = param_trajectory.Phi_d_pp(1:3, MPC_traj_indices_val ); % (Phi_pp_0 ... Phi_pp_N)

% q_d_0       = param_trajectory.q_d(       1:4, MPC_traj_indices_val ); % (q_0 ... q_N)
% omega_d_0   = param_trajectory.omega_d(   1:3, MPC_traj_indices_val ); % (omega_0 ... omega_N)
% omega_d_p_0 = param_trajectory.omega_d_p( 1:3, MPC_traj_indices_val ); % (omega_p_0 ... omega_p_N)

% Phi_d_0 = zeros(3, N_MPC+1);
% Phi_d_p_0 = zeros(3, N_MPC+1);
% Phi_d_pp_0 = zeros(3, N_MPC+1);

% for i=1:N_MPC+1
%     R_d = quat2rotm_v2(q_d_0(:, i));
%     Phi_d_0(:, i) = rotm2rpy(R_d);
%     Phi_d_0(Phi_d_0==pi) = -pi; % avoid flipping effects
%     Phi_d_p_0(:, i) = T_rpy(Phi_d_0(:, i))*omega_d_0(:, i);
%     Phi_d_pp_0(:, i) = T_rpy_p(Phi_d_0(:, i), Phi_d_p_0(:, i))*omega_d_0(:, i) + T_rpy(Phi_d_0(:, i))*omega_d_p_0(:, i);
% end
% warning('This methods suffers on flipping effects near to +-pi.');

% initial guess for reference trajectory
y_d_0    = [p_d_0;    Phi_d_0    ];
y_d_p_0  = [p_d_p_0;  Phi_d_p_0  ];
y_d_pp_0 = [p_d_pp_0; Phi_d_pp_0 ];

% Robot System: Initial guess

q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m
u_k_0  = full(tau_fun_red(q_0_red, q_0_red_p, q_0_red_pp)); % gravity compensation

u_init_guess_0 = ones(1, N_MPC).*u_k_0; % fully actuated
x_init_guess_0 = ones(1, N_MPC+1).*x_0_0;

% Discrete y_ref system
if(isempty(yt_indices))
    n_yt_red = 0;
    eps_t_cnt = 0;
else
    eps_t_cnt = 1; % extra inequation constraint for terminal constraint
    
    n_yt = 3;
    n_z = 6;
end

if(isempty(yr_indices))
    eps_r_cnt = 0;
else
    eps_r_cnt = 1; % extra inequation constraint for terminal constraint
    n_yr = 3;
    n_zr = 6;
end

n_z = n_zr + n_z;
n_y = n_yr + n_yt;

% e = SX.sym('e', n_z);
% v = SX.sym('v', n_y);
% K_p = SX.sym('K_p', n_y, n_y);
% K_d = SX.sym('K_d', n_y, n_y);

% ht_ref_red = Function('ht_ref_red', {et_red, alpha_t_red}, {[et_red(1+n_yt_red:2*n_yt_red); alpha_t_red]});
opt = struct;
opt.allow_free = true;
% h_ref_free = Function('h_ref_free', {e, v}, {[zeros(n_y), eye(n_y); -K_p, -K_d]*e + [zeros(n_y); eye(n_y)]*v}, opt);
% h_ref_free = Function('h_ref_free', {e, v}, {[e(1+n_y:n_z); v - K_p*e(1:n_y) - K_d*e(1+n_y:n_z)]}, opt);
% zb h_ref_red(e, pp.Q_y_ref, pp.Q_y_p_ref)

% Translational part
% H_free = integrate_casadi(h_ref_free, DT, M, int_method);
% H_free_kp1 = integrate_casadi(h_ref_free, DT_ctl, M, int_method); % runs with Ta from sensors
% H_free2 = integrate_casadi(h_ref_free, DT2, M, int_method); % runs with Ts_MPC-Ta

% H = Function('H_ref', {e, v, K_p, K_d}, {H_free(e, v)});
% H_kp1 = Function('H_kp1_ref', {e, v, K_p, K_d}, {H_free_kp1(e, v)});
% H2 = Function('H2_ref', {e, v, K_p, K_d}, {H_free2(e, v)});

v_ref = SX.sym('v_ref', n_y, 1);
z_ref = SX.sym('z_ref', n_z, 1);
y_p_ref = z_ref(1+n_y:n_z);
h_ref = Function('h_ref', {z_ref, v_ref}, {[y_p_ref; v_ref]});

H = integrate_casadi(h_ref, DT, M, int_method);
H_kp1 = integrate_casadi(h_ref, DT_ctl, M, int_method); % runs with Ta from sensors
H2 = integrate_casadi(h_ref, DT2, M, int_method); % runs with Ts_MPC-Ta

% Ref System: Translational init guess
z_0_0              = [y_d_0(:, 1); y_d_p_0(:, 1)]; % init for z_ref
alpha_init_guess_0 =  y_d_pp_0;
z_init_guess_0     = [y_d_0; y_d_p_0];


lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(z_init_guess_0)+numel(alpha_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(z_init_guess_0)+numel(u_init_guess_0)+eps_t_cnt+eps_r_cnt, 1); % + 1 wegen eps

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); z_init_guess_0(:); alpha_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

if(any(isnan(full(init_guess_0))))
    error('init_guess_0 contains NaN values!');
end

%% Start with an empty NLP

% Optimization Variables:
u       = SX.sym( 'u',     n_red,   N_MPC   );
x       = SX.sym( 'x',     2*n_red, N_MPC+1 );
z       = SX.sym( 'z',     n_z,     N_MPC+1 );
alpha   = SX.sym( 'alpha', n_y,     N_MPC+1 );

mpc_opt_var_inputs = {u, x, z, alpha};

N_opt = numel(u) + numel(x) + numel(z) + numel(alpha);
N_u = numel(u);
N_x = numel(x);
N_z = numel(z);
N_alpha = numel(alpha);
u_opt_indices = 1:n_red;

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); -Inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1); -Inf(size(z(:))); -Inf(size(alpha(:)))];
ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1);  Inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1);  Inf(size(z(:)));  Inf(size(alpha(:)))];

% lbw = -SX(Inf(size(w)));
% ubw = SX(Inf(size(w)));

% lbw(1:N_u) = repmat(pp.u_min(n_indices), size(u, 2), 1);
% ubw(1:N_u) = repmat(pp.u_max(n_indices), size(u, 2), 1);
% lbw(1+N_u+2*n_red:N_u+N_x) = repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1);
% ubw(1+N_u+2*n_red:N_u+N_x) = repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1);

% input parameter
x_k = SX.sym( 'x_k', 2*n_red, 1 ); % current x state = initial x state
z_k = SX.sym( 'z_k', n_z, 1 ); % initial z state

y_d    = SX.sym( 'y_d',    m, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d
y_d_p  = SX.sym( 'y_d_p',  m, N_MPC+1 ); % (y_d_p_0 ... y_d_p_N)
y_d_pp = SX.sym( 'y_d_pp', m, N_MPC+1 ); % (y_d_pp_0 ... y_d_pp_N)

x_prev     = SX.sym( 'x_prev', size(x));
z_prev     = SX.sym( 'z_prev', size(z));
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
g_u_prev = cell(1, N_MPC); % for u_prev
g_eps = cell(1, eps_t_cnt+eps_r_cnt); % separate for transl and rotation

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
y    = SX( 6, N_MPC+1 ); % transl. TCP position:      (y_0 ... y_N)
q_pp = SX( n_red, N_MPC ); % joint acceleration: (q_pp_0 ... q_pp_N-1) % last makes no sense: q_pp_N depends on u_N, wich is not known
q_p = SX( n_red, N_MPC+1 ); % joint velocity

% reference trajectory values
y_ref    = SX( n_y, N_MPC+1 );
y_p_ref  = SX( n_y, N_MPC+1 );
y_pp_ref = SX( n_y, N_MPC+1 );

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_vec = SX(n_red, N_MPC);

g_x(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk
g_z(1, 1 + (0)) = {z_k - z(:, 1 + (0))}; % z0

for i=0:N_MPC
    % calculate q (q_0 ... q_N) and q_p values (q_p_0 ... q_p_N)
    q = x(1:n_red, 1 + (i));
    q_p(:,  1 + (i)) = x(n_red+1:2*n_red, 1 + (i));
    
    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:6,   1 + (i)) = rotm2rpy_casadi(R_e);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);
    
    y_ref(   :, 1 + (i)) = z(     1:n_y,     1 + (i));
    y_p_ref( :, 1 + (i)) = z(     1+n_y:n_z, 1 + (i));
    y_pp_ref(:, 1 + (i)) = alpha( 1:n_y,     1 + (i));

    if(i < N_MPC)
        if(i == 0)
            g_z(1, 1 + (i+1)) = { H_kp1(z(:, 1 + (i)), alpha(:, 1 + (i))) - z(:, 1 + (i+1)) }; % Set the translational refsys dynamics constraints for zk = z(t0) = tilde z0 to zk+1 = z(t0+Ta)
        elseif(i == 1)
            g_z(1, 1 + (i+1)) = { H2(z(:, 1 + (i)), alpha(:, 1 + (i))) - z(:, 1 + (i+1)) }; % Set the translational refsys dynamics constraints for zk+1 = z(t0+Ta) to z(t0+Ts_MPC) = tilde z1
        else
            g_z(1, 1 + (i+1)) = { H(z(:, 1 + (i)), alpha(:, 1 + (i))) - z(:, 1 + (i+1)) }; % Set the translational refsys dynamics constraints for z(t0+Ts_MPC*i) to z(t0+Ts_MPC*(i+1))
        end
    end
    
    if(i < N_MPC)
        g_vec(:, 1 + (i)) = g_fun_red(q);
        if(i == 0)
            g_x(1, 1 + (i+1))  = { F_kp1(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk = x(t0) = tilde x0 to xk+1 = x(t0+Ta)
        elseif(i == 1)
            g_x(1, 1 + (i+1))  = { F2(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk+1 = x(t0+Ta) to x(t0+Ts_MPC) = tilde x1
        else
            g_x(1, 1 + (i+1))  = { F(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for x(t0+Ts_MPC*i) to x(t0+Ts_MPC*(i+1))
        end
        
        dx   = f_red(x(:, 1 + (i)), u(:, 1 + (i))); % = [d/dt q, d^2/dt^2 q], Alternativ: Differenzenquotient
        q_pp(:, 1 + (i)) = dx(n_red+1:2*n_red, 1);
        g_u_prev(1, 1 + (i)) = {u(:, 1 + (i)) - u_prev(:, 1 + (i))};
    end
end

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(diag(Q), z));

if isempty(yt_indices)
    J_yt = 0;
    Jt_yy_ref = 0;
    gt_eps = [];
else
    J_yt = Q_norm_square( y(yt_indices, 1 + (1:N_MPC-1) ) - y_ref(yt_indices, 1 + (1:N_MPC-1)), pp.Q_y(yt_indices)  );
    gt_eps = norm_2( (y(yt_indices, end) - y_ref(yt_indices,end)) );
end


if isempty(yr_indices)
    J_yr = 0;
    gr_eps = [];
    Jr_yy_ref = 0;
else
    J_yr = 0;
    for i=1:N_MPC
        % Hier werden nicht Euler Winkel sondern Fehler um Achsen gewichtet!!
        yr_ref_temp = y_ref(4:6, 1 + (i));
        R_y_yr = R_e_arr{1 + (i)}' * rpy2rotm_casadi(yr_ref_temp);
        % % %q_y_y_err = rotation2quaternion_casadi( R_y_yr );
        rpy_err = [R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)];

        % R_y_yr = R_e_arr{1 + (i)}' * rpy2rotm_casadi(yr_ref_temp) - rpy2rotm_casadi(yr_ref_temp)' * R_e_arr{1 + (i)};
        % rpy_err = [R_y_yr(3,2); R_y_yr(1,3); R_y_yr(2,1)];
        
        % interessanterweise wird es nur instabil wenn yr_indices=[1 2 3] ist, wenn es nur eine oder zwei Winkel sind, dann ist es stabil
        % rpy_err = y(3+yr_indices, 1 + (i)) - yr_ref(:, 1 + (i)); % das sollte eig gehen??? wird aber immer instabil [TODO]
        
        if(i < N_MPC)
            J_yr = J_yr + Q_norm_square( rpy_err(yr_indices) , pp.Q_y(3+yr_indices)  ); % Die Gewichtung sind die Fehler um eine Achse nicht die der RPY Winkel!!
        else
            gr_eps = norm_2( rpy_err(yr_indices) );
        end
    end
    % J_yr = Q_norm_square( y(3+yr_indices, 1 + (1:N_MPC-1) ) - y_ref(3+yr_indices, 1 + (1:N_MPC-1)), pp.Q_y(3+yr_indices,3+yr_indices)  );
    % gr_eps = norm_2( (y(3+yr_indices,end) - y_ref(3+yr_indices,end)) );
end

gcnt = 1;
if(~isempty(yt_indices))
    g_eps(1, gcnt) = {gt_eps};
    gcnt = 2; % = 2 for yr if yr_indices is not empty
end

if(~isempty(yr_indices))
    g_eps(1, gcnt) = {gr_eps};
end

g = [g_x, g_z, g_u_prev, g_eps]; % merge_cell_arrays([g_x, g_z, g_eps], 'vector')';

% jump in tau at max 1000Nm/s
max_du = pp.max_du;
max_du_arr = repmat(max_du*dt_int_arr, n_red, 1);

lbg(1+N_x+N_z:N_x+N_z+N_u) = -max_du_arr(:);
ubg(1+N_x+N_z:N_x+N_z+N_u) =  max_du_arr(:);

u_err = u-g_vec; % es ist stabiler es nicht gegenüber der vorherigen Lösung zu gewichten!

J_u  = Q_norm_square(u_err, pp.R_u(n_indices));
J_q_ref = Q_norm_square(x(1:n_red, :) - pp.q_ref(n_indices), pp.R_q_ref(n_indices));
J_q_p  = Q_norm_square(x(1+n_red:2*n_red, :), pp.R_q_p(n_indices));

J_x_prev     = Q_norm_square(x     - x_prev,     pp.R_x_prev(n_x_indices));
J_z_prev     = Q_norm_square(z     - z_prev,     pp.R_z_prev(n_z_indices));
J_alpha_prev = Q_norm_square(alpha - alpha_prev, pp.R_alpha_prev(n_y_indices));

J_u0_prev = Q_norm_square(u(:, 1) - u_prev(:, 1), pp.R_u0_prev(n_indices));

J_alpha = Q_norm_square(alpha - y_d_pp, pp.R_alpha);

cost_vars_names = '{J_yt, J_yr, J_u, J_q_ref, J_q_p, J_alpha, J_u0_prev, J_x_prev, J_z_prev, J_alpha_prev}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);