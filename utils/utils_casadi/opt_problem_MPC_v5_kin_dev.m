% MPC v4: Optimization problem 

import casadi.*

diff_variant_mode = struct;
diff_variant_mode.numdiff = 1; % default forward, central, backward deviation
diff_variant_mode.savgol = 2; % savgol filtering and deviation
diff_variant_mode.savgol_v2 = 3; % savgol filtering without additional equations and deviation
diff_variant_mode.numdiff_twotimes = 4; % forward, central, backward deviation with two different time steps

diff_variant = diff_variant_mode.numdiff;

is_int_for_xkp1 = (diff_variant == diff_variant_mode.numdiff || diff_variant == diff_variant_mode.savgol_v2 || diff_variant == diff_variant_mode.savgol);

n = param_robot.n_DOF; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

[~, ~, Q] = quat_deriv(ones(4,1), ones(3,1), ones(3,1)); % get function handle

% Discrete system dynamics
M = 1; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.
DT_ctl = param_global.Ta/M;

x = SX.sym('x', 2*n);
u = SX.sym('u', n);

% integrator for x
f = Function('f', {x, u}, {[x(n+1:2*n); u]});
F_kp1 = integrate_casadi(f, DT_ctl, M, int_method); % runs with Ta from sensors

f = Function('f', {x, u}, {[x(n+1:2*n); u]});
F_2 = integrate_casadi(f, DT-DT_ctl, M, int_method); % runs with Ta from sensors


%% Calculate Initial Guess

% Get trajectory data for initial guess
p_d_0    = param_trajectory.p_d(    1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_0 ... y_N)
%p_d_p_0  = param_trajectory.p_d_p(  1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_p_0 ... y_p_N)
%p_d_pp_0 = param_trajectory.p_d_pp( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_pp_0 ... y_pp_N)

q_d_0       = param_trajectory.q_d(       1:4, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_0 ... q_N)
%omega_d_0   = param_trajectory.omega_d(   1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (omega_0 ... omega_N)
%omega_d_p_0 = param_trajectory.omega_d_p( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (omega_p_0 ... omega_p_N)

p_d_0_kp1 = param_trajectory.p_d(1:3, 2);
q_d_0_kp1 = param_trajectory.q_d(1:4, 2);

p_d_0_kp2 = param_trajectory.p_d(1:3, 3);
q_d_0_kp2 = param_trajectory.q_d(1:4, 3);

% initial guess for reference trajectory

if(is_int_for_xkp1)
    y_d_0    = [p_d_0;    q_d_0   ];
    y_d_kp1_0  = [p_d_0_kp1;  q_d_0_kp1 ];
    y_d_kp2_0  = [p_d_0_kp2;  q_d_0_kp2 ];
else % diff_variant == diff_variant_mode.numdiff_twotimes
    y_d_0    = [[p_d_0(:,1), p_d_0_kp1, p_d_0(:,2:end-1)]; [q_d_0(:,1), q_d_0_kp1, q_d_0(:,2:end-1)]];
end

% Robot System: Initial guess

x_0_0  = [q_0; q_0_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xd compute_tau_fun(q_0, dq_0, ddq_0); % gravity compensationof.m
q_0    = x_0_0(1   :   n); % useless line...
dq_0   = x_0_0(1+n : 2*n);
ddq_0  = q_0_pp;
xe_k_0 = p_d_0(1:3, 1); % x pos, y pos, defined in parameters_xdof.m
u_k_0  = ddq_0;

u_init_guess_0 = [u_k_0; u_k_0];
x_init_guess_0 = [x_0_0 ones(2*n, N_MPC).*x_0_0];

if(is_int_for_xkp1)
    xkp1_init_guess_0 = x_0_0;
    xkp2_init_guess_0 = x_0_0;

    lam_x_init_guess_0 = zeros(numel(xkp1_init_guess_0)+numel(xkp2_init_guess_0)+numel(u_init_guess_0)+numel(x_init_guess_0), 1);

    if(diff_variant == diff_variant_mode.numdiff || diff_variant == diff_variant_mode.savgol_v2)
        % only equation constrained for q_p = S_v q
        lam_g_init_guess_0 = zeros(numel(xkp1_init_guess_0)+numel(xkp2_init_guess_0)+numel(x_init_guess_0(:,1))+numel(x_init_guess_0(n+1:end,:)), 1); % + 1 wegen eps
    elseif(diff_variant == diff_variant_mode.savgol)
        % equation constrained for q_savgol = Q q and q_p_savgol = D q;
        lam_g_init_guess_0 = zeros(numel(xkp1_init_guess_0)+numel(xkp2_init_guess_0)+numel(x_init_guess_0(:,1))+numel(x_init_guess_0), 1); % + 1 wegen eps
    else
        error('invalid mode');
    end

    init_guess_0 = [xkp1_init_guess_0(:); xkp2_init_guess_0(:); u_init_guess_0(:); x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];
else % diff_variant == diff_variant_mode.numdiff_twotimes
    lam_x_init_guess_0 = zeros(numel(u_init_guess_0) + numel(x_init_guess_0), 1);
    lam_g_init_guess_0 = zeros(numel(x_init_guess_0(:,1))+numel(x_init_guess_0(n+1:end,:)), 1);

    init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];
end


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
u     = SX.sym( 'u',    n,       2 ); % %u0=q0pp (0 to Ta), u1=q1pp (Ta to TsMPC)
x     = SX.sym( 'x',  2*n, N_MPC+1 );
%mpc_opt_var_inputs = {u, x};
%u_opt_indices = [3*n+1:4*n, 4*n+1:5*n, 1:n]; % [q_1, dq_1, ddq_1] needed for joint space CT control

if(is_int_for_xkp1)
    xkp1 = SX.sym('xkp1', 2*n);
    xkp2 = SX.sym('xkp2', 2*n);

    mpc_opt_var_inputs = {xkp1, xkp2, u, x}; %[ xkp1(1:2n), xkp2(2n+1:4n), uk(4n+1:5n), ukp1(5n+1:6n)]

    u_opt_indices = [1:2*n,  5*n+1:6*n]; %

    % optimization variables cellarray w
    w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
    lbw = [pp.x_min; pp.x_min; pp.u_min; pp.u_min; repmat(pp.x_min, N_MPC + 1, 1);];
    ubw = [pp.x_max; pp.x_max; pp.u_max; pp.u_max; repmat(pp.x_max, N_MPC + 1, 1);];
else % diff_variant == diff_variant_mode.numdiff_twotimes
    mpc_opt_var_inputs = {u, x};

    u_opt_indices = [3*n+1:4*n, 4*n+1:5*n, 1:n]; % [q_1, dq_1, ddq_1] needed for joint space CT control

    % optimization variables cellarray w
    w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
    lbw = [pp.u_min; pp.u_min; repmat(pp.x_min, N_MPC + 1, 1);];
    ubw = [pp.u_max; pp.u_max; repmat(pp.x_max, N_MPC + 1, 1);];
end



% input parameter
x_k  = SX.sym( 'x_k',  2*n,       1 ); % current x state = initial x state
y_d  = SX.sym( 'y_d',  m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d

if(is_int_for_xkp1)
    y_d_kp1 = SX.sym('y_d_kp1', m+1, 1); % y_d_N+1
    y_d_kp2 = SX.sym('y_d_kp2', m+1, 1); % y_d_N+1

    mpc_parameter_inputs = {x_k, y_d, y_d_kp1, y_d_kp2};
    mpc_init_reference_values = [x_0_0(:); y_d_0(:); y_d_kp1_0(:); y_d_kp2_0(:)];
else % diff_variant == diff_variant_mode.numdiff_twotimes
    mpc_parameter_inputs = {x_k, y_d};
    mpc_init_reference_values = [x_0_0(:); y_d_0(:)];
end

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+1); % for F
g_xkp1 = cell(1, 1); % for Fkp1

if(weights_and_limits_as_parameter)
    if(is_int_for_xkp1)
        if(diff_variant == diff_variant_mode.numdiff || diff_variant == diff_variant_mode.savgol_v2)
            lbg = SX(numel(x(:, 1))+numel(x(n+1:end,:))+2*numel(xkp1), 1);
            ubg = SX(numel(x(:, 1))+numel(x(n+1:end,:))+2*numel(xkp1), 1);
        elseif(diff_variant == diff_variant_mode.savgol)
            lbg = SX(numel(x(:, 1))+numel(x)+2*numel(xkp1), 1);
            ubg = SX(numel(x(:, 1))+numel(x)+2*numel(xkp1), 1);
        else
            error('invalid mode');
        end
    else % diff_variant == diff_variant_mode.numdiff_twotimes
        lbg = SX(numel(x(:, 1))+numel(x(n+1:end,:)), 1);
        ubg = SX(numel(x(:, 1))+numel(x(n+1:end,:)), 1);
        % TODO: SAVGOL
    end
else
    if(is_int_for_xkp1)
        if(diff_variant == diff_variant_mode.numdiff || diff_variant == diff_variant_mode.savgol_v2)
            lbg = zeros(numel(x(:, 1))+numel(x(n+1:end,:))+2*numel(xkp1), 1);
            ubg = zeros(numel(x(:, 1))+numel(x(n+1:end,:))+2*numel(xkp1), 1);
        elseif(diff_variant == diff_variant_mode.savgol)
            lbg = zeros(numel(x(:, 1))+numel(x)+2*numel(xkp1), 1);
            ubg = zeros(numel(x(:, 1))+numel(x)+2*numel(xkp1), 1);
        else
            error('invalid mode');
        end
    else % diff_variant == diff_variant_mode.numdiff_twotimes
        lbg = zeros(numel(x(:, 1))+numel(x(n+1:end,:)), 1);
        ubg = zeros(numel(x(:, 1))+numel(x(n+1:end,:)), 1);
        % TODO: SAVGOL
    end
end

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y    = SX( 7, N_MPC+1 ); % TCP pose:      (y_0 ... y_N)
R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

if(diff_variant == diff_variant_mode.numdiff)
    S_v = create_numdiff_matrix(DT, n, N_MPC+1, 'fwdbwdcentral');
    % S_v = create_numdiff_matrix(DT, n, N_MPC+1, 'bwd');
    S_a = S_v^2;
elseif(diff_variant == diff_variant_mode.savgol_v2)
    DD = create_numdiff_matrix(DT, n, N_MPC+1, 'savgol');
    S_v = DD{2};
    S_a = DD{3};
elseif(diff_variant == diff_variant_mode.savgol)
    DD = create_numdiff_matrix(DT, n, N_MPC+1, 'savgol'); % create the velocity deviation matrix S_v
    %S_q = eye(size(DD{1}));
    S_q = DD{1};
    S_v = DD{2};
    S_a = DD{3};
elseif(diff_variant == diff_variant_mode.numdiff_twotimes)
    % TODO SAVGOL MIT 3 X create_numdiff for Ta, Ta_MPC, Ta_MPC-Ta und entspr matrizen ersetzen
    S_v = create_numdiff_matrix(DT_ctl, n, N_MPC+1, 'fwdbwdcentraltwotimes', DT);
    S_a = S_v^2;
else
    error('invalid mode');
end
%      x = [q(t0),   q(t1), ...  q(tN),  dq(t0),  dq(t1), ...  dq(tN)] = [qq;   qq_p ]
% d/dt x = [dq(t0), dq(t1), ... dq(tN), ddq(t0), ddq(t1), ... ddq(tN)] = [qq_p; qq_pp]
qq = reshape(x(1:n, :), n*(N_MPC+1), 1);
qq_p  = S_v * qq;
qq_pp = S_a * qq;

if(diff_variant == diff_variant_mode.savgol_v2)
    qq = DD{1} * qq;
end

q_p = reshape(qq_p, n, N_MPC+1);
q_pp = reshape(qq_pp, n, N_MPC+1);

x_p = [q_p; q_pp];

g_x(1, 1 + (0))     = {x_k  - x(   :, 1 + (0))};

if(diff_variant == diff_variant_mode.numdiff || diff_variant == diff_variant_mode.savgol_v2 || diff_variant == diff_variant_mode.numdiff_twotimes)
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

for i=0:N_MPC
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_py_fun(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_endeffector_py_fun(q);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(i < N_MPC)
        if(diff_variant == diff_variant_mode.numdiff || diff_variant == diff_variant_mode.savgol_v2 || diff_variant == diff_variant_mode.numdiff_twotimes)
            g_x(1, 1 + (i+1)) = { x(n+1:end, 1 + (i+1)) - q_p(:, 1 + (i+1))                          }; % q_p_1 = q_p_1
        elseif(diff_variant == diff_variant_mode.savgol)
            g_x(1, 1 + (i+1)) = { x(:,       1 + (i+1)) - [q_savgol(:, 1 + (i+1)); q_p(:, 1 + (i+1))]}; % q_savgol = S_q q_1, q_p_1 = q_p_1
        else
            error('invalid mode');
        end
    end
end

if(is_int_for_xkp1)
    g_xkp1(1,1) = {[xkp1 - F_kp1(x(:,1), u(:,1)); xkp2 - F_kp1(xkp1, u(:,2))]};
    qkp1 = xkp1(1:n);
    Hkp1 = hom_transform_endeffector_py_fun(qkp1);
    ykp1 = [Hkp1(1:3, 4); quat_endeffector_py_fun(qkp1)];

    qkp2 = xkp2(1:n);
    Hkp2 = hom_transform_endeffector_py_fun(qkp2);
    ykp2 = [Hkp2(1:3, 4); quat_endeffector_py_fun(qkp2)];
end

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

J_yt   = Q_norm_square( y(1:3, 1 + (1:N_MPC-1) ) - y_d(1:3, 1 + (1:N_MPC-1)), pp.Q_y( 1:3,1:3)  );
J_yt_N = Q_norm_square( y(1:3, 1 + (  N_MPC  ) ) - y_d(1:3, 1 + (  N_MPC  )), pp.Q_yN(1:3,1:3)  );

J_yr = SX(1,1);
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

if(is_int_for_xkp1)
    q_ykp1_yr_err = quat_mult(ykp1(4:7), quat_inv(y_d_kp1(4:7)));
    q_ykp2_yr_err = quat_mult(ykp2(4:7), quat_inv(y_d_kp2(4:7)));

    J_yt_kp1 = Q_norm_square( ykp1(1:3) - y_d_kp1(1:3), pp.Q_ykp1(1:3,1:3) ) + Q_norm_square( ykp2(1:3) - y_d_kp2(1:3), pp.Q_ykp1(1:3,1:3) );
    J_yr_kp1 = Q_norm_square( q_ykp1_yr_err(2:4) , pp.Q_ykp1(4:6,4:6) ) + Q_norm_square( q_ykp2_yr_err(2:4) , pp.Q_ykp1(4:6,4:6) );
    g = [g_x, g_xkp1];
    cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp, J_yt_kp1, J_yr_kp1}';
else % diff_variant == diff_variant_mode.numdiff_twotimes
    g = g_x;

    cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp}';
end
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);