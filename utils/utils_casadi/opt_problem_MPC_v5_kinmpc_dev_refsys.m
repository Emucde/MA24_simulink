% MPC v4: Optimization problem 

import casadi.*

diff_variant_mode = struct;
diff_variant_mode.numdiff = 1; % default forward, central, backward deviation
diff_variant_mode.savgol = 2; % savgol filtering and deviation

diff_variant = diff_variant_mode.numdiff;
%diff_variant = diff_variant_mode.savgol;

n = param_robot.n_DOF; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

[~, ~, Q] = quat_deriv(ones(4,1), ones(3,1), ones(3,1)); % get function handle

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.

% Discrete yt_ref system
zt = SX.sym('zt', 6);
zr = SX.sym('zr', 7);
alpha_t = SX.sym('alpha_t', 3);
alpha_r = SX.sym('alpha_r', 3);

% Translational part
ht_ref = Function('h_ref', {zt, alpha_t}, {[zt(4:6); alpha_t]});
Ht = integrate_casadi(ht_ref, DT, M, int_method);

% Rotational part
hr_ref = Function('h_qw_ref', {zr, alpha_r}, {[Q(zr(1:4)) * zr(5:7); alpha_r]});
Hr_unorm = integrate_casadi(hr_ref, DT, M, int_method);
Hr_val = Hr_unorm(zr, alpha_r);
% ensure unit quaternion: normalize
Hr = Function('H_qw', {zr, alpha_r}, {[Hr_val(1:4)/norm_2(Hr_val(1:4)); Hr_val(5:7)]});


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

x_0_0  = [q_0; q_0_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xd compute_tau_fun(q_0, dq_0, ddq_0); % gravity compensationof.m
q_0    = x_0_0(1   :   n); % useless line...
dq_0   = x_0_0(1+n : 2*n);
ddq_0  = q_0_pp;
xe_k_0 = p_d_0(1:3, 1); % x pos, y pos, defined in parameters_xdof.m
u_k_0  = ddq_0;

u_init_guess_0 = u_k_0;
x_init_guess_0 = [x_0_0 ones(2*n, N_MPC).*x_0_0];

% Ref System: Translational init guess
zt_0_0               = [y_d_0(1:3,1); y_d_p_0(1:3,1)]; % init for zt_ref
alpha_t_init_guess_0 = y_d_pp_0(1:3, 1:end-1);
alpha_t_N_0          = y_d_pp_0(1:3,end);

Ht_sim              = Ht.mapaccum(N_MPC);
zt_init_guess_kp1_0 = Ht_sim(zt_0_0, alpha_t_init_guess_0);
zt_init_guess_0     = [zt_0_0 full(zt_init_guess_kp1_0)];

% Ref System: Rotational init guess
zr_0_0               = [y_d_0(4:7,1); y_d_p_0(4:6,1)]; % init for zr_ref
alpha_r_init_guess_0 = y_d_pp_0(4:6, 1:end-1);
alpha_r_N_0          = y_d_pp_0(4:6,end);

Hr_sim              = Hr.mapaccum(N_MPC);
zr_init_guess_kp1_0 = Hr_sim(zr_0_0, alpha_r_init_guess_0);
zr_init_guess_0     = [zr_0_0 full(zr_init_guess_kp1_0)];

% total init guess for ref system
z_init_guess_0 = [zt_init_guess_0; zr_init_guess_0];
alpha_init_guess_0 = [alpha_t_init_guess_0; alpha_r_init_guess_0];
alpha_N_0 = [alpha_t_N_0; alpha_r_N_0];
z_0_0 = [zt_0_0; zr_0_0];

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(z_init_guess_0)+numel(alpha_init_guess_0) + numel(alpha_N_0), 1);
if(diff_variant == diff_variant_mode.numdiff)
    % only equation constrained for q_p = S_v q
    lam_g_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0(:,1))+numel(x_init_guess_0(n+1:end,:))+numel(z_init_guess_0)+2, 1); % + 1 wegen eps
elseif(diff_variant == diff_variant_mode.savgol)
    % equation constrained for q_savgol = Q q and q_p_savgol = D q;
    lam_g_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0(:,1))+numel(x_init_guess_0)+numel(z_init_guess_0)+2, 1); % + 1 wegen eps
else
    error('invalid mode');
end

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
u     = SX.sym( 'u',    n, 1   ); % u = q_0_pp
x     = SX.sym( 'x',  2*n, N_MPC+1 );
zt    = SX.sym( 'zt', m,   N_MPC+1 );
zr    = SX.sym( 'zr', m+1, N_MPC+1 );
alpha_t = SX.sym( 'alpha_t', 3,   N_MPC+1 );
alpha_r = SX.sym( 'alpha_r', 3,   N_MPC+1 );

z = [ zt; zr ];
alpha = [ alpha_t; alpha_r ];

mpc_opt_var_inputs = {u, x, z, alpha};

u_opt_indices = [3*n+1:4*n, 4*n+1:5*n, 1:n]; % [q_1, dq_1, ddq_1] needed for joint space CT control

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
% TODO: get qpp_max, qpp_min via FD: qpp_max = FD(q_max?, q_pmax?, tau_max?)
lbw = [-Inf(size(u)); repmat(pp.x_min, N_MPC + 1, 1); -Inf(size(z(:))); -Inf(size(alpha(:)))];
ubw = [ Inf(size(u)); repmat(pp.x_max, N_MPC + 1, 1);  Inf(size(z(:)));  Inf(size(alpha(:)))];

% input parameter
x_k  = SX.sym( 'x_k',  2*n, 1 ); % current x state = initial x state
zt_0 = SX.sym( 'zt_0', m,   1 ); % initial zt state
zr_0 = SX.sym( 'zr_0', m+1, 1 ); % initial zr state

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
g_eps = cell(1, 2); % separate for transl and rotation

if(weights_and_limits_as_parameter)
    if(diff_variant == diff_variant_mode.numdiff)
        lbg = SX(numel(u)+numel(x(:, 1))+numel(x(n+1:end,:))+numel(z)+2, 1);
        ubg = SX(numel(u)+numel(x(:, 1))+numel(x(n+1:end,:))+numel(z)+2, 1);
    elseif(diff_variant == diff_variant_mode.savgol)
        lbg = SX(numel(u)+numel(x(:, 1))+numel(x)+numel(z)+2, 1);
        ubg = SX(numel(u)+numel(x(:, 1))+numel(x)+numel(z)+2, 1);
    else
        error('invalid mode');
    end
else
    if(diff_variant == diff_variant_mode.numdiff)
        lbg = zeros(numel(u)+numel(x(:, 1))+numel(x(n+1:end,:))+numel(z)+2, 1);
        ubg = zeros(numel(u)+numel(x(:, 1))+numel(x(n+1:end,:))+numel(z)+2, 1);
    elseif(diff_variant == diff_variant_mode.savgol)
        lbg = zeros(numel(u)+numel(x(:, 1))+numel(x)+numel(z)+2, 1);
        ubg = zeros(numel(u)+numel(x(:, 1))+numel(x)+numel(z)+2, 1);
    else
        error('invalid mode');
    end
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
yt_p_ref  = SX( 3, N_MPC+1 ); % TCP velocity:      (yt_p_ref_0 ... yt_p_ref_N)
yt_pp_ref = SX( 3, N_MPC+1 ); % TCP acceleration:  (yt_pp_ref_0 ... yt_pp_ref_N)

yr_ref    = SX( 4, N_MPC+1 ); % TCP orientation:                (y_qw_ref_0 ... y_qw_ref_N)
yr_p_ref  = SX( 3, N_MPC+1 ); % TCP orietnation velocity:      (y_qw_p_ref_0 ... y_qw_p_ref_N)
yr_pp_ref = SX( 3, N_MPC+1 ); % TCP orientation acceleration:  (y_qw_pp_ref_0 ... y_qw_pp_ref_N)

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

if(diff_variant == diff_variant_mode.numdiff)
    S_v = create_numdiff_matrix(DT, n, N_MPC+1, 'fwdbwdcentral');
    % S_v = create_numdiff_matrix(DT, n, N_MPC+1, 'bwd');
    S_a = S_v^2;

    %DD = create_numdiff_matrix(DT, n, N_MPC+1, 'savgol');
    %S_v = DD{2};
    %S_a = DD{3};
elseif(diff_variant == diff_variant_mode.savgol)
    DD = create_numdiff_matrix(DT, n, N_MPC+1, 'savgol'); % create the velocity deviation matrix S_v
    %S_q = eye(size(DD{1}));
    S_q = DD{1};
    S_v = DD{2};
    S_a = DD{3};
end
%      x = [q(t0),   q(t1), ...  q(tN),  dq(t0),  dq(t1), ...  dq(tN)] = [qq;   qq_p ]
% d/dt x = [dq(t0), dq(t1), ... dq(tN), ddq(t0), ddq(t1), ... ddq(tN)] = [qq_p; qq_pp]
qq = reshape(x(1:n, :), n*(N_MPC+1), 1);
qq_p  = S_v * qq;
qq_pp = S_a * qq;

q_p = reshape(qq_p, n, N_MPC+1);
q_pp = reshape(qq_pp, n, N_MPC+1);

x_p = [q_p; q_pp];

g_x(1, 1 + (0))     = { [ x_k            - x(   :,    1 + (0)); ...  % x_k = tilde x_0 = [tilde q_0; tilde q_0_p] = [q_k; q_k_p]
                        u               - q_pp(:,    1 + (1))]};     % tilde u = tilde q_pp_1

if(diff_variant == diff_variant_mode.numdiff)
    g_x(1, 1 + (0)) = {[g_x{1, 1 + (0)}; ... 
                        q_p(:, 1 + (0)) - x(n+1:end, 1 + (0))]}; % tilde q_p_0 = Sv tilde q_0
elseif(diff_variant == diff_variant_mode.savgol)
    qq_savgol = S_q * qq;
    q_savgol = reshape(qq_savgol, n, N_MPC+1);
    g_x(1, 1 + (0)) = {[g_x{1, 1 + (0)}; ...
                        x(:, 1 + (0)) - [q_savgol(:, 1 + (0)); q_p(:, 1 + (0))]]}; % [tilde q_0; tilde q_p_0] = [S_q tilde q_0; S_v tilde q_0]
else
    error('invalid mode');
end

g_zt(1, 1 + (0))    = {z_0(1:m, 1)     - z(1:m,     1 + (0))}; % zt0
g_zr(1, 1 + (0))    = {z_0(m+1:end, 1) - z(m+1:end, 1 + (0))}; % zr0

for i=0:N_MPC
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_py_fun(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    yt_ref(   1:3, 1 + (i)) = zt(     1:3, 1 + (i));
    yt_p_ref( 1:3, 1 + (i)) = zt(     4:6, 1 + (i));
    yt_pp_ref(1:3, 1 + (i)) = alpha_t( 1:3, 1 + (i));

    yr_ref(   1:4, 1 + (i)) = zr( 1:4, 1 + (i));
    yr_p_ref( 1:3, 1 + (i)) = zr( 5:7, 1 + (i));
    yr_pp_ref(1:3, 1 + (i)) = alpha_r( 1:3, 1 + (i));

    if(i < N_MPC)
        if(diff_variant == diff_variant_mode.numdiff)
            g_x(1, 1 + (i+1)) = { x(n+1:end, 1 + (i+1)) - q_p(:, 1 + (i+1))                          }; % q_p_1 = q_p_1
        elseif(diff_variant == diff_variant_mode.savgol)
            g_x(1, 1 + (i+1)) = { x(:,       1 + (i+1)) - [q_savgol(:, 1 + (i+1)); q_p(:, 1 + (i+1))]}; % q_savgol = S_q q_1, q_p_1 = q_p_1
        else
            error('invalid mode');
        end
        g_zt(1, 1 + (i+1)) = { Ht(zt(:, 1 + (i)), alpha_t(:, 1 + (i)) ) - zt(:, 1 + (i+1)) }; % Set the yref_t dynamics constraints
        g_zr(1, 1 + (i+1)) = { Hr(zr(:, 1 + (i)), alpha_r(:, 1 + (i)) ) - zr(:, 1 + (i+1)) }; % Set the yref_r dynamics constraints
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
g_z = [g_zt, g_zr];
g = [g_x, g_z, g_eps]; % merge_cell_arrays([g_x, g_z, g_eps], 'vector')';

et_pp = yt_pp_ref( :, 1 + (0:N_MPC) ) - y_d_pp( 1:3, 1 + (0:N_MPC) );
et_p  = yt_p_ref(  :, 1 + (0:N_MPC) ) - y_d_p(  1:3, 1 + (0:N_MPC) );
et    = yt_ref(    :, 1 + (0:N_MPC) ) - y_d(    1:3, 1 + (0:N_MPC) );

er_pp = yr_pp_ref( :, 1 + (0:N_MPC) ) - y_d_pp( 4:6, 1 + (0:N_MPC) );
er_p  = yr_p_ref(  :, 1 + (0:N_MPC) ) - y_d_p(  4:6, 1 + (0:N_MPC) );
er    = SX(3, N_MPC+1);
for i=0:N_MPC
    R_yr_yd = quat2rotm_v2(yr_ref(:, 1 + (i))) * quat2rotm_v2(y_d(4:7, 1 + (i)))';
    %q_err = rotation2quaternion_casadi( R_yr_yd );
    q_yr_yd_err = [1; R_yr_yd(3,2) - R_yr_yd(2,3); R_yr_yd(1,3) - R_yr_yd(3,1); R_yr_yd(2,1) - R_yr_yd(1,2)];
    er(:, 1 + (i)) = q_yr_yd_err(2:4);
end

Jt_yy_ref = Q_norm_square( et_pp + mtimes(pp.Q_y_p_ref(1:3, 1:3), et_p) + mtimes(pp.Q_y_ref(1:3, 1:3), et), eye(3));
Jr_yy_ref = Q_norm_square( er_pp + mtimes(pp.Q_y_p_ref(4:6, 4:6), er_p) + mtimes(pp.Q_y_ref(4:6, 4:6), er), eye(3));

J_q_pp = Q_norm_square(q_pp, pp.R_q_pp); %Q_norm_square(u, pp.R_u);

cost_vars_names = '{J_yt, Jt_yy_ref, J_yr, Jr_yy_ref, J_q_pp}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);