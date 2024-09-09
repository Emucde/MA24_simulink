% MPC v4: Optimization problem 

import casadi.*

diff_variant_mode = struct;
diff_variant_mode.numdiff = 1; % default forward, central, backward deviation
diff_variant_mode.savgol = 2; % savgol filtering and deviation
diff_variant_mode.savgol_v2 = 3; % savgol filtering without additional equations and deviation

diff_variant = diff_variant_mode.numdiff;

n = param_robot.n_DOF; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.
DT_ctl = param_global.Ta/M;

% Get trajectory data for initial guess
p_d_0       = param_trajectory.p_d( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_0 ... y_N)
q_d_0       = param_trajectory.q_d( 1:4, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_0 ... q_N)

% initial guess for reference trajectory parameter
if(N_step_MPC == 1 || N_step_MPC == 2)
    y_d_0    = [p_d_0; q_d_0];
else
    p_d_0_kp1 = param_trajectory.p_d(  1:3, 2:3 );
    q_d_0_kp1 = param_trajectory.q_d(  1:4, 2:3 );
    y_d_0    = [[p_d_0(:,1), p_d_0_kp1, p_d_0(:,2:end-2)]; [q_d_0(:,1), q_d_0_kp1, q_d_0(:,2:end-2)]];
end

%% Calculate Initial Guess

x_0_0  = [q_0; q_0_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xd compute_tau_fun(q_0, dq_0, ddq_0); % gravity compensationof.m
q_0    = x_0_0(1   :   n); % useless line...
dq_0   = x_0_0(1+n : 2*n);
ddq_0  = q_0_pp;
xe_k_0 = p_d_0(1:3, 1); % x pos, y pos, defined in parameters_xdof.m
u_k_0  = ddq_0;

u_init_guess_0 = u_k_0;
q_init_guess_0 = ones(n, N_MPC+1).*x_0_0(1:n);
q_p_init_guess_0 = ones(n, 2).*x_0_0(n+1:2*n);

lam_x_init_guess_0 = zeros(numel(u_init_guess_0) + numel(q_init_guess_0) + numel(q_p_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(u_init_guess_0) + 4*n, 1);
% eig brauch ich q0, q0p, q0pp, q1, q1p = 5n Gleichungsbedingungen.

init_guess_0 = [u_init_guess_0(:); q_init_guess_0(:); q_p_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

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
u     = SX.sym( 'u',    n, 1 ); % %u0=q0pp (0 to Ta)
q     = SX.sym( 'q',    n, N_MPC+1 );
q_p_out   = SX.sym( 'q_p',  n, 2 );
x     = [q, q_p_out];

mpc_opt_var_inputs = {u, q, q_p_out};

N_u = numel(u);
N_q = numel(q);
q0_pp_idx = [N_u+1   : N_u+  n, N_u+N_q+1   : N_u+N_q+n,  1 : n]; % [q_0, q_p_0, q_pp_0] % current desired state for CT Control
q1_pp_idx = [N_u+1+n : N_u+2*n, N_u+N_q+1+n : N_u+N_q+2*n      ]; % [q_1, q_p_1] % next init state for MPC
u_opt_indices = [q0_pp_idx, q1_pp_idx];

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min, size(u, 2), 1); repmat(pp.x_min(1:n), size(q, 2), 1); repmat(pp.x_min(n+1:2*n), size(q_p_out, 2), 1)];
ubw = [repmat(pp.u_max, size(u, 2), 1); repmat(pp.x_max(1:n), size(q, 2), 1); repmat(pp.x_max(n+1:2*n), size(q_p_out, 2), 1)];

% input parameter
x_k  = SX.sym( 'x_k',  2*n, 1 ); % current x state = initial x state
y_d  = SX.sym( 'y_d',  m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d

mpc_parameter_inputs = {x_k, y_d};
mpc_init_reference_values = [x_0_0(:); y_d_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, 4);

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
    S_v = create_numdiff_matrix(DT_ctl, n, N_MPC+1, 'fwdbwdcentralthreetimes', DT);
    S_a = S_v^2;
elseif(diff_variant == diff_variant_mode.savgol)
    DD_DT  = create_numdiff_matrix(DT, n, N_MPC+1, 'savgol');

    S_v = DD_DT{2};
    S_a = DD_DT{3};
elseif(diff_variant == diff_variant_mode.savgol_v2)
    DD_DT  = create_numdiff_matrix(DT, n, N_MPC+1, 'savgol');

    S_q = DD_DT{1};
    S_v = DD_DT{2};
    S_a = DD_DT{3};
else
    error('invalid mode');
end
%      x = [q(t0),   q(t1), ...  q(tN),  dq(t0),  dq(t1), ...  dq(tN)] = [qq;   qq_p ]
% d/dt x = [dq(t0), dq(t1), ... dq(tN), ddq(t0), ddq(t1), ... ddq(tN)] = [qq_p; qq_pp]
qq = reshape(q, n*(N_MPC+1), 1);
qq_p  = S_v * qq;
qq_pp = S_a * qq;

if(diff_variant == diff_variant_mode.savgol_v2)
    qq = S_q * qq;
end

q_p = reshape(qq_p, n, N_MPC+1);
q_pp = reshape(qq_pp, n, N_MPC+1);

g_x(1, 1 + (0)) = {x_k - [q(:, 1 + (0)); q_p(:, 1 + (0))]}; % x0 = xk
% g_x(1, 1 + (0)) = {x_k(1:n) - q(:, 1 + (0))}; % x0 = xk
g_x(1, 1 + (1)) = {q_p_out(:, 1 + (0)) - q_p(:, 1 + (0))};
g_x(1, 1 + (2)) = {q_p_out(:, 1 + (1)) - q_p(:, 1 + (1))};
g_x(1, 1 + (3)) = {u - q_pp(:, 1 + (0))}; % u0 = q0pp

for i=0:N_MPC
    q_i = q(:, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_py_fun(q_i);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_endeffector_py_fun(q_i);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);
end

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

J_yt   = Q_norm_square( y(1:3, 1 + (1:N_MPC-1) ) - y_d(1:3, 1 + (1:N_MPC-1)), pp.Q_y( 1:3,1:3)  );
J_yt_N = Q_norm_square( y(1:3, 1 + (  N_MPC  ) ) - y_d(1:3, 1 + (  N_MPC  )), pp.Q_yN(1:3,1:3)  );

J_yr = 0;
for i=1:N_MPC
    % R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))';
    % % q_y_y_err = rotation2quaternion_casadi( R_y_yr );
    % q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)]; %ungenau aber schneller (flipping?)
    q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(y_d(4:7, 1 + (i))));

    if(i < N_MPC)
        J_yr = J_yr + Q_norm_square( q_y_yr_err(2:4) , pp.Q_y(4:6,4:6)  );
    else
        J_yr_N = Q_norm_square( q_y_yr_err(2:4) , pp.Q_yN(4:6,4:6)  );
    end
end

J_q_pp = Q_norm_square(q_pp, pp.R_q_pp); %Q_norm_square(u, pp.R_u);

g = g_x;

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);