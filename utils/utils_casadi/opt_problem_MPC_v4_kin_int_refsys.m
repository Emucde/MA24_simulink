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

% Discrete state space model
x = SX.sym('x', 2*n);
u = SX.sym('u', n);

% integrator for x
f = Function('f', {x, u}, {[x(n+1:2*n); u]});
F = integrate_casadi(f, DT, M, int_method);

% Discrete et_ref system
et = SX.sym('et', 6);
dummy_t = SX.sym('dummy_t', 3);
K_d_t = SX.sym('K_d_t', 3, 3);
K_p_t = SX.sym('K_p_t', 3, 3);

% Translational part
opt = struct;
opt.allow_free = true;
et_ref = Function('e_ref', {et, dummy_t}, {[et(4:6); -K_d_t*et(4:6) - K_p_t*et(1:3)]}, opt);
Et_int = integrate_casadi(et_ref, DT, M, int_method);
Et = Function('E_t', {et, K_d_t, K_p_t}, {Et_int(et, dummy_t)});

% Discrete er_ref system
er = SX.sym('er', 7); % er = [quat_err, omega_err]

K_d_r = SX.sym('K_d_r', 3, 3);
K_p_r = SX.sym('K_p_r', 3, 3);

quat_d = SX.sym('q_d', 4);
eps_d = quat_d(2:4);

quat_r = quat_mult(er(1:4), quat_d);
eps_r = quat_r(2:4);

omega_d = SX.sym('omega_d', 3);
omega_r = er(5:7) + omega_d;

dummy_r = SX.sym('dummy_r', 4); % not needed, but integrate_casadi needs f(x,u). dummy variable

Q_r = Q(quat_r);
Q_r_eps = Q_r(2:4, :);
Q_d = Q(quat_d);
Q_d_eps = Q_d(2:4, :);

% Rotational part
opt = struct;
opt.allow_free = true;
er_ref = Function('er_ref', {er, dummy_r}, {[Q_r*omega_r - Q_d*omega_d; K_d_r*er(5:7) - 4*Q_r_eps'*K_p_r*eps_r + 4*Q_d_eps'*K_p_r*eps_d]}, opt);
Er_unorm = integrate_casadi(er_ref, DT, M, int_method);
Er_val = Er_unorm(er, dummy_r);
% ensure unit quaternion: normalize
Er = Function('E_qw', {er, quat_d, omega_d, K_d_r, K_p_r}, {[Er_val(1:4)/norm_2(Er_val(1:4)); Er_val(5:7)]});

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

% Ref System: Translational init guess
et_0_0              = zeros(6, 1); % init for et_ref
et_init_guess_0     = zeros(6, N_MPC+1); 

% Ref System: Rotational init guess
er_0_0          = zeros(7, 1); % init for er_ref
er_init_guess_0 = zeros(7, N_MPC+1);

% total init guess for ref system
e_init_guess_0 = [et_init_guess_0; er_init_guess_0];
e_0_0 = [et_0_0; er_0_0];

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(e_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(e_init_guess_0)+2, 1); % + 1 wegen eps

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); e_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

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
u     = SX.sym( 'u',    n, N_MPC   );
x     = SX.sym( 'x',  2*n, N_MPC+1 );
et    = SX.sym( 'et', m,   N_MPC+1 );
er    = SX.sym( 'er', m+1, N_MPC+1 );

e = [ et; er ];

mpc_opt_var_inputs = {u, x, e};

% u = [u0; u1; ... uN-1], x = [x0; x1; ... xN] = [q_0;q_0_p;q_1;q_1_p;...;q_N;q_N_p]
% xx = [u; x] = [u0; u1; ... uN-1; x0; x1; ... xN]
%   u1 = q_1_pp = u( n+1:2+n)   |   q_1 = x(     1+2*n:    3*n),  |   q_1_p = x(     1+3*n:    4*n)
%   u1 = q_1_pp = xx(n+1:2+n)   |   q_1 = xx(N_u+1+2*n:N_u+3*n),  |   q_1_p = xx(N_u+1+3*n:N_u+4*n)
N_u = numel(u);
u_opt_indices = [1+2*n+N_u:N_u+3*n, 1+3*n+N_u:N_u+4*n, 1+n:2*n]; % [q_1, q_1_p, q_1_pp] needed for joint space CT control

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min, N_MPC, 1); repmat(pp.x_min, N_MPC + 1, 1); -Inf(size(e(:)));];
ubw = [repmat(pp.u_max, N_MPC, 1); repmat(pp.x_max, N_MPC + 1, 1);  Inf(size(e(:)));];

% input parameter
x_k  = SX.sym( 'x_k',  2*n, 1 ); % current x state = initial x state
et_0 = SX.sym( 'et_0', m,   1 ); % initial et state
er_0 = SX.sym( 'er_0', m+1, 1 ); % initial er state

y_d    = SX.sym( 'y_d',    m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d
y_d_p  = SX.sym( 'y_d_p',  m,   N_MPC+1 ); % (y_d_p_0 ... y_d_p_N)
y_d_pp = SX.sym( 'y_d_pp', m,   N_MPC+1 ); % (y_d_pp_0 ... y_d_pp_N)
e_0 = [et_0; er_0];

mpc_parameter_inputs = {x_k, e_0, y_d, y_d_p, y_d_pp};
mpc_init_reference_values = [x_0_0(:); e_0_0(:); y_d_0(:); y_d_p_0(:); y_d_pp_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+1); % for F
g_et = cell(1, N_MPC+1); % for H
g_er = cell(1, N_MPC+1); % for H_qw
g_eps = cell(1, 2); % separate for transl and rotation

if(weights_and_limits_as_parameter)
    lbg = SX(numel(x)+numel(e)+2, 1);
    ubg = SX(numel(x)+numel(e)+2, 1);
else
    lbg = zeros(numel(x)+numel(e)+2, 1);
    ubg = zeros(numel(x)+numel(e)+2, 1);
end

lbg(end-1:end) = [0; 0];
ubg(end-1:end) = [pp.epsilon_t; pp.epsilon_r];

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y    = SX( 3, N_MPC+1 ); % transl. TCP position:      (y_0 ... y_N)

% reference trajectory values
yt_ref    = SX( 3, N_MPC+1 ); % TCP position:      (yt_ref_0 ... yt_ref_N)
yr_ref    = SX( 4, N_MPC+1 ); % TCP orientation:                (y_qw_ref_0 ... y_qw_ref_N)

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_x(1, 1 + (0))   = {x_k             - x(:,       1 + (0))}; % x0 = xk
g_et(1, 1 + (0))  = {e_0(1:m, 1)     - e(1:m,     1 + (0))}; % et0
g_er(1, 1 + (0))  = {e_0(m+1:end, 1) - e(m+1:end, 1 + (0))}; % er0

Q_err = @(z1, z2) [quat_mult(z1(1:4), quat_inv(z2(1:4))); z1(5:7) - z2(5:7)];

for i=0:N_MPC
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_py_fun(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    R_e_arr{1 + (i)}  = H_e(1:3, 1:3);

    yt_ref(   1:3, 1 + (i)) = et(      1:3, 1 + (i)) + y_d(    1:3, 1 + (i));
    yr_ref(   1:4, 1 + (i)) = quat_mult(er( 1:4, 1 + (i)),  y_d(    4:7, 1 + (i)));

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        g_x(1, 1 + (i+1))  = { F(  x(:, 1 + (i)), u(      :, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state dynamics constraints

        K_d_t = pp.Q_y_p_ref(1:3, 1:3);
        K_p_t = pp.Q_y_ref(1:3, 1:3);

        g_et(1, 1 + (i+1)) = { Et(et(:, 1 + (i)), K_d_t, K_p_t) - et(:, 1 + (i+1)) }; % Set the yref_t dynamics constraints
        
        quat_d = y_d(    4:7, 1 + (i));
        omega_d = y_d_p(  4:6, 1 + (i));
        K_d_r = pp.Q_y_p_ref(4:6, 4:6);
        K_p_r = pp.Q_y_ref(4:6, 4:6);

        % Er = Function('E_qw', {er, quat_d, omega_d, K_d_r, K_p_r},
        z1 = Er(er(:, 1 + (i)), quat_d, omega_d, K_d_r, K_p_r);
        z2 = er(:, 1 + (i+1));
        
        g_er(1, 1 + (i+1)) = { z1 - z2 }; % Set the yref_t dynamics constraints
        % g_er(1, 1 + (i+1)) = { Q_err(z1, z2) }; % Set the yref_r dynamics constraints
    end
end

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));


J_yt = Q_norm_square( y(1:3, 1 + (1:N_MPC-1) ) - yt_ref(1:3, 1 + (1:N_MPC-1)), pp.Q_y(1:3,1:3)  );
gt_eps = norm_2( (y(1:3,end) - yt_ref(1:3,end)) );

J_yr = SX(1,1);
for i=1:N_MPC
    R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(yr_ref(1:4, 1 + (i)))';
    %q_y_y_err = rotation2quaternion_casadi( R_y_yr );
    q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)]; %ungenau aber schneller (flipping?)

    if(i < N_MPC)
        J_yr = J_yr + Q_norm_square( q_y_yr_err(2:4) , pp.Q_y(4:6,4:6)  );
    else
        gr_eps = norm_2( q_y_yr_err(2:4) );
    end
end

g_eps(1, 1) = {gt_eps};
g_eps(1, 2) = {gr_eps};
g_e = [g_et, g_er];
g = [g_x, g_e, g_eps]; % merge_cell_arrays([g_x, g_e, g_eps], 'vector')';

J_q_pp = Q_norm_square(u, pp.R_q_pp); %Q_norm_square(u, pp.R_u);

cost_vars_names = '{J_yt, J_yr, J_q_pp}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);