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
%     ||alpha_n   - y_pp_d_n||_2^2 * I; ...         % Penalty on difference between acceleration reference (alpha_n) and desired acceleration (y_pp_d_n)
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

n = param_robot.n_DOF; % Dimension of joint space
m = 6; % Dimension of Task Space
% m = param_robot.m; % Dimension of Task Space
m_t = 3;%param_robot.m_t; % Translational part of Task Space
m_r = 3;%param_robot.m_r; % Rotational part of Task Space 

% Model equations
% Forward Dynamics: d/dt x = f(x, u)
use_aba = false;
if(use_aba)
    f = Function.load([output_dir, 'sys_fun_x_py.casadi']); % forward dynamics (FD), d/dt x = f(x, u), x = [q; dq]
else
    M      = casadi.Function.load(['./', s_fun_path, '/inertia_matrix_py.casadi']); % ok, rundungsfehler +- 0.003
    C_rnea = casadi.Function.load(['./', s_fun_path, '/n_q_coriols_qp_plus_g_py.casadi']);
    % hilfsvariablen, werden unten überschrieben
    x = SX.sym('x', 2*n, 1); % x = [q; dq]
    u = SX.sym('u', n, 1); % u = [tau]
    f = Function('f', {x, u}, {vertcat(x(n+1:2*n), solve( M(x(1:n)), u - C_rnea(x(1:n), x(n+1:2*n)) ))}, {'x', 'u'}, {'dx'});
end

compute_tau_fun = Function.load([output_dir, 'compute_tau_py.casadi']); % Inverse Dynamics (ID)
hom_transform_endeffector_py_fun = Function.load([output_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([output_dir, 'quat_endeffector_py.casadi']);

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.

F = integrate_casadi(f, DT, M, int_method);

% Discrete yt_ref system
zt     = SX.sym('zt',     6);
zr     = SX.sym('zr',     8);
alpha_t = SX.sym('alpha_t', 3 );
alpha_r = SX.sym('alpha_r', 4 );

% Translational part
ht_ref = Function('h_ref', {zt, alpha_t}, {[zt(4:6); alpha_t]});
Ht = integrate_casadi(ht_ref, DT, M, int_method);

% Rotational part
hr_ref = Function('h_ref', {zr, alpha_r}, {[zr(5:8); alpha_r]});
Hr = integrate_casadi(hr_ref, DT, M, int_method);

%% Calculate Initial Guess
p_d_0    = param_trajectory.p_d(    1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_0 ... y_N)
p_d_p_0  = param_trajectory.p_d_p(  1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_p_0 ... y_p_N)
p_d_pp_0 = param_trajectory.p_d_pp( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_pp_0 ... y_pp_N)

%Phi_d_0 = param_trajectory.Phi_d(      1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (Phi_0 ... Phi_N)
%Phi_d_p_0 = param_trajectory.Phi_d_p(  1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (Phi_p_0 ... Phi_p_N)
%Phi_d_pp_0 = param_trajectory.Phi_d_pp(1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (Phi_pp_0 ... Phi_pp_N)

% R_d_0       = param_trajectory.R_d(       1:m_r, 1:m_r, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (R_0 ... R_N)
q_d_0    = param_trajectory.q_d(    1:4, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_0 ... q_N)
q_d_p_0  = param_trajectory.q_d_p(  1:4, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_p_0 ... q_p_N)
q_d_pp_0 = param_trajectory.q_d_pp( 1:4, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_p_0 ... q_p_N)
% omega_d_0   = param_trajectory.omega_d(   1:m_r,        1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (omega_0 ... omega_N)
% omega_d_p_0 = param_trajectory.omega_d_p( 1:m_r,        1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (omega_p_0 ... omega_p_N)

y_d_0    = [p_d_0;    q_d_0   ];
y_d_p_0  = [p_d_p_0;  q_d_p_0 ];
y_d_pp_0 = [p_d_pp_0; q_d_pp_0];

x_0_0  = [q_0; q_0_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m
q_0    = x_0_0(1   :   n); % useless line...
dq_0   = x_0_0(1+n : 2*n);
ddq_0  = q_0_pp;
xe_k_0 = xe0(1:m_t); % x pos, y pos, defined in parameters_xdof.m
%u_k_0  = compute_tau(q_0, dq_0, ddq_0, param_robot); % tau1, tau2
u_k_0  = compute_tau_fun(q_0, dq_0, ddq_0); % much more faster than above command

u_init_guess_0 = ones(n, N_MPC).*u_k_0; % fully actuated

% für die S-funktion ist der Initial Guess wesentlich!
F_sim              = F.mapaccum(N_MPC);
x_init_guess_kp1_0 = F_sim(x_0_0, u_init_guess_0);
x_init_guess_0     = [x_0_0 full(x_init_guess_kp1_0)];

zt_0_0               = [y_d_0(1:3,1); y_d_p_0(1:3,1)]; % init for zt_ref
alpha_t_init_guess_0 = y_d_pp_0(1:3, 1:end-1);
alpha_t_N_0          = y_d_pp_0(1:3,end);

zr_0_0               = [y_d_0(4:7,1); y_d_p_0(4:7,1)]; % init for zr_ref
alpha_r_init_guess_0 = y_d_pp_0(4:7, 1:end-1);
alpha_r_N_0          = y_d_pp_0(4:7,end);

Ht_sim              = Ht.mapaccum(N_MPC);
zt_init_guess_kp1_0 = Ht_sim(zt_0_0, alpha_t_init_guess_0);
zt_init_guess_0     = [zt_0_0 full(zt_init_guess_kp1_0)];
zt_d_init_guess_0   = [y_d_0(1:3, :); y_d_p_0(1:3, :)]; % init for zt_d

Hr_sim              = Hr.mapaccum(N_MPC);
zr_init_guess_kp1_0 = Hr_sim(zr_0_0, alpha_r_init_guess_0);
zr_init_guess_0     = [zr_0_0 full(zr_init_guess_kp1_0)];
zr_d_init_guess_0   = [y_d_0(4:7, :); y_d_p_0(4:7, :)]; % init for zr_d

z_init_guess_0 = zt_init_guess_0;
alpha_init_guess_0 = alpha_t_init_guess_0;
alpha_N_0 = alpha_t_N_0;
z_0_0 = zt_0_0;

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(z_init_guess_0)+numel(alpha_init_guess_0) + numel(alpha_N_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(z_init_guess_0)+1, 1); % + 1 wegen eps

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); z_init_guess_0(:); alpha_init_guess_0(:); alpha_N_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

% get weights from "init_MPC_weight.m"
param_weight_init = param_weight.(casadi_func_name);

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init); % Matrizen sind keine Diagonlmatrizen [TODO]
else % hardcoded weights
    pp = param_weight_init;
end

%% Start with an empty NLP

% Optimization Variables:
u     = SX.sym( 'u',       n, N_MPC   );
x     = SX.sym( 'x',     2*n, N_MPC+1 );
zt     = SX.sym( 'zt',     m,   N_MPC+1 );
zr     = SX.sym( 'zt',     8,   N_MPC+1 );
alpha_t = SX.sym( 'alpha_t', 3,   N_MPC+1 );
alpha_r = SX.sym( 'alpha_r', 4,   N_MPC+1 );

z = zt;
alpha = alpha_t;

mpc_opt_var_inputs = {u, x, zt, alpha};

u_opt_indices = 1:n;

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min, N_MPC, 1); repmat(pp.x_min, N_MPC + 1, 1); -Inf(size(z(:))); -Inf(size(alpha(:)))];
ubw = [repmat(pp.u_max, N_MPC, 1); repmat(pp.x_max, N_MPC + 1, 1);  Inf(size(z(:)));  Inf(size(alpha(:)))];

% input parameter
x_k    = SX.sym( 'x_k',    2*n, 1       ); % current x state
zt_0    = SX.sym( 'zt_0',    m,   1       ); % initial z state
zr_0    = SX.sym( 'zt_0',    8,   1       ); % initial z state
y_d    = SX.sym( 'y_d',    m+1, N_MPC+1 ); % (y_d_0 ... y_d_N)
y_p_d  = SX.sym( 'y_p_d',  m+1, N_MPC+1 ); % (y_p_d_0 ... y_p_d_N)
y_pp_d = SX.sym( 'y_pp_d', m+1, N_MPC+1 ); % (y_pp_d_0 ... y_pp_d_N)

z_0 = zt_0;

mpc_parameter_inputs = {x_k, z_0, y_d, y_p_d, y_pp_d};
mpc_init_reference_values = [x_0_0(:); z_0_0(:); y_d_0(:); y_d_p_0(:); y_d_pp_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x = cell(1, N_MPC+1); % for F
g_zt = cell(1, N_MPC+1); % for H
g_zr = cell(1, N_MPC+1); % for H
g_eps = cell(1, 1);

lbg = SX(numel(x)+numel(z)+1, 1);
ubg = SX(numel(x)+numel(z)+1, 1);

lbg(end) = 0;
ubg(end) = pp.epsilon;

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y    = SX( 3, N_MPC+1 ); % transl. TCP position:      (y_0 ... y_N)
q_pp = SX( n,   N_MPC   ); % joint acceleration: (q_pp_0 ... q_pp_N-1) % last makes no sense: q_pp_N depends on u_N, wich is not known

% reference trajectory values
yt_ref    = SX( 3, N_MPC+1 ); % TCP position:      (yt_ref_0 ... yt_ref_N)
yt_p_ref  = SX( 3, N_MPC+1 ); % TCP velocity:      (yt_p_ref_0 ... yt_p_ref_N)
yt_pp_ref = SX( 3, N_MPC+1 ); % TCP acceleration:  (yt_pp_ref_0 ... yt_pp_ref_N)

yr_ref    = SX( 4, N_MPC+1 ); % TCP position:      (yr_ref_0 ... yr_ref_N)
yr_p_ref  = SX( 4, N_MPC+1 ); % TCP velocity:      (yr_p_ref_0 ... yr_p_ref_N)
yr_pp_ref = SX( 4, N_MPC+1 ); % TCP acceleration:  (yr_pp_ref_0 ... yr_pp_ref_N)

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_x(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk
g_zt(1, 1 + (0)) = {zt_0 - zt(:, 1 + (0))}; % z0 = zk
g_zr(1, 1 + (0)) = {zr_0 - zr(:, 1 + (0))}; % z0 = zk
for i=0:N_MPC
    % calculate q (q_0 ... q_N) and q_p values (q_p_0 ... q_p_N)
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_py_fun(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);     %y_t_0 wird nicht verwendet
    %y(4:m+1, 1 + (i)) = rotation2quaternion_casadi(R_e); %y_r_0 wird nicht verwendet
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    yt_ref(   1:3, 1 + (i)) = zt(     1:3, 1 + (i));
    yt_p_ref( 1:3, 1 + (i)) = zt(     4:6, 1 + (i));
    yt_pp_ref(1:3, 1 + (i)) = alpha_t( 1:3, 1 + (i));

    yr_ref(   1:4, 1 + (i)) = zr(     1:4, 1 + (i));
    yr_p_ref( 1:4, 1 + (i)) = zr(     5:8, 1 + (i));
    yr_pp_ref(1:4, 1 + (i)) = alpha_r( 1:4, 1 + (i));

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        g_x(1, 1 + (i+1)) = {F(x(:, 1 + (i)), u(    :, 1 + (i))) - x(:, 1 + (i+1))}; % Set the state dynamics constraints
        
        g_zt(1, 1 + (i+1)) = {Ht(zt(:, 1 + (i)), alpha_t(:, 1 + (i))) - zt(:, 1 + (i+1))}; % Set the state dynamics constraints

        g_zr(1, 1 + (i+1)) = {Hr(zr(:, 1 + (i)), alpha_r(:, 1 + (i))) - zr(:, 1 + (i+1))}; % Set the state dynamics constraints

        dx   = f(x(:, 1 + (i)), u(:, 1 + (i))); % = [d/dt q, d^2/dt^2 q], Alternativ: Differenzenquotient
        q_pp(:, 1 + (i)) = dx(n+1:2*n, 1);
    end
end

% g_eps(1, 1) = {norm_2(  sub_fun_y( y(1:m+1, end),  z(1:m+1, end) )  )}; % for pp.epsilon
%g_eps(1, 1) = {norm_2( blkdiag(0*eye(3), 0*eye(4)) * (y(:,end)-yt_ref(:,end)) )}; % for pp.epsilon %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PROBLEM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gt_eps = norm_2( (y(1:3,end) - yt_ref(1:3,end)) );
%gr_eps = norm_2( quat_mult_vec(rotation2quaternion_casadi(R_e_arr{end}), quat_inv(yr_ref(1:4,end))) );
quat_err = rotation2quaternion_casadi(R_e_arr{end} * quat2rotm_v2(yr_ref(1:4,end))');
gr_eps = norm_2( quat_err(2:4) );

g_eps(1, 1) = {gt_eps};
g_z = g_zt;
g = [g_x, g_z, g_eps]; % merge_cell_arrays([g_x, g_z, g_eps], 'vector')';

Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

et_pp = yt_pp_ref( :, 1 + (0:N_MPC) ) - y_pp_d( 1:3, 1 + (0:N_MPC) );
et_p  = yt_p_ref(  :, 1 + (0:N_MPC) ) - y_p_d(  1:3, 1 + (0:N_MPC) );
et    = yt_ref(    :, 1 + (0:N_MPC) ) - y_d(    1:3, 1 + (0:N_MPC) );

%er_pp = yr_pp_ref( :, 1 + (0:N_MPC) ) - y_pp_d( 4:7, 1 + (0:N_MPC) );
%er_p  = yr_p_ref(  :, 1 + (0:N_MPC) ) - y_p_d(  4:7, 1 + (0:N_MPC) );
%er    = yr_ref(    :, 1 + (0:N_MPC) ) - y_d(    4:7, 1 + (0:N_MPC) );
er_pp = SX(3, N_MPC+1);
er_p  = SX(3, N_MPC+1);
er    = SX(3, N_MPC+1);
for i=0:N_MPC
    quat_err_pp = rotation2quaternion_casadi( quat2rotm_v2( yr_pp_ref( :, 1 + (0:N_MPC) ) ) * quat2rotm_v2( y_pp_d( 4:7, 1 + (i) ) )' );
    quat_err_p  = rotation2quaternion_casadi( quat2rotm_v2( yr_p_ref(  :, 1 + (0:N_MPC) ) ) * quat2rotm_v2( y_p_d(  4:7, 1 + (i) ) )' );
    quat_err    = rotation2quaternion_casadi( quat2rotm_v2( yr_ref(    :, 1 + (0:N_MPC) ) ) * quat2rotm_v2( y_d(    4:7, 1 + (i) ) )' );
    % alternative with quat_mult
    % quat_err_pp = quat_mult(  yr_pp_ref( :, 1 + (i) ), quat_inv( y_pp_d(4:7, 1 + (i)) )  );
    % quat_err_p  = quat_mult(  yr_p_ref(  :, 1 + (i) ), quat_inv( y_p_d(  4:7, 1 + (i)) )  );
    % quat_err    = quat_mult(  yr_ref(    :, 1 + (i) ), quat_inv( y_d(    4:7, 1 + (i)) )  );
    er_pp(:, 1 + (i)) = quat_err_pp(2:4);
    er_p( :, 1 + (i)) = quat_err_p( 2:4);
    er(   :, 1 + (i)) = quat_err(   2:4);
end

%e = SX(m, N_MPC+1);
Q_ori = SX(1,1);
%e(1:m_t, :) = yt_ref(   1:m_t, 1 + (0:N_MPC) ) - y_d(  1:m_t, 1 + (0:N_MPC));
for i=0:N_MPC
    %e(m_t+1:m, 1 + (i)) = rotm2rpy_casadi( rpy2rotm_casadi(yt_ref(m_t+1:m, 1 + (i))) * R_e_arr{1 + (i)}');
    %e(m_t+1:m, 1 + (i)) = rotm2rpy_casadi( R_e_arr{1 + (i)} * rpy2rotm_casadi(yt_ref(m_t+1:m, 1 + (i)))');
    q_err = rotation2quaternion_casadi( R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))');
    %q_err = rotm2quat_v3_casadi( R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))');
    Q_ori = Q_ori + Q_norm_square( q_err(2:4) , pp.Q_y(4:6,4:6)  );
end
% Q_ori = Q_norm_square( yt_ref(   4:6, 1 + (0:N_MPC) ) - y_d(  4:6, 1 + (0:N_MPC)), pp.Q_y(4:6,4:6)  );

%J_yt = Q_norm_square( y(1:3, 1 + (1:N_MPC-1) ) - yt_ref(1:3, 1 + (1:N_MPC-1)), pp.Q_y(1:3,1:3)  );
J_yt = Q_norm_square( y(1:3, 1 + (1:N_MPC-1) ) - y_d(1:3, 1 + (1:N_MPC-1)), pp.Q_y(1:3,1:3)  );
J_yr = Q_ori;
%J_yy_ref = 0*Q_norm_square(e_pp + mtimes(pp.Q_y_p_ref, e_p) + mtimes(pp.Q_yt_ref, e), blkdiag(eye(3), 1*eye(3))); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PROBLEM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Jt_yy_ref = Q_norm_square(et_pp + mtimes(pp.Q_y_p_ref(1:3, 1:3), et_p) + mtimes(pp.Q_y_ref(1:3, 1:3), et), eye(3));
%Jr_yy_ref = Q_norm_square(er_pp + mtimes(pp.Q_y_p_ref(4:6, 4:6), er_p) + mtimes(pp.Q_y_ref(4:6, 4:6), er), eye(3));
J_q_pp = Q_norm_square(q_pp, pp.R_q_pp); %Q_norm_square(u, pp.R_u);

%cost_vars_names = '{J_yt, J_yr, Jt_yy_ref, J_q_pp}';
cost_vars_names = '{J_yr, J_yt, Jt_yy_ref, J_q_pp}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);