% MPC v4: Optimization problem 

import casadi.*

using_exact_kin_system = false;

kin_int_modes = param_mpc_mode.kin_int_modes;
mpc_mode = param_mpc_mode.mode; % defined in init_mpc_weights(.*).m

n = param_robot.n_DOF; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

[~, ~, Q] = quat_deriv(ones(4,1), ones(3,1), ones(3,1)); % get function handle

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.
DT_ctl = param_global.Ta/M;
if(N_step_MPC == 1)
    DT2 = DT_ctl; % special case if Ts_MPC = Ta
else
    DT2 = DT - DT_ctl;
end

opt = struct;
opt.allow_free = true;

% Discrete yt_ref system
x = SX.sym('x', 4*n);

v = SX.sym('v', n);
K_d = SX.sym('K_d', n, n);
K_p = SX.sym('K_p', n, n);

% integrator for x = [q; q_p; q_pp; q_ppp]

x1 = x(1:n);
x2 = x(n+1:2*n);
x3 = x(2*n+1:3*n);
x4 = x(3*n+1:4*n);

if(using_exact_kin_system)
    Phi_Ts_MPC = Phi_kin_aperiod_casadi_SX(DT, K_d);
    Phi_Ta = Phi_kin_aperiod_casadi_SX(DT_ctl, K_d);
    Phi_Ts_MPC_m_Ta = Phi_kin_aperiod_casadi_SX(DT2, K_d);

    Gamma_Ts_MPC = Gamma_kin_aperiod_casadi_SX(DT, K_d);
    Gamma_Ta = Gamma_kin_aperiod_casadi_SX(DT_ctl, K_d);
    Gamma_Ts_MPC_m_Ta = Gamma_kin_aperiod_casadi_SX(DT2, K_d);

    F = Function('F', {x, v, K_d, K_p}, {Phi_Ts_MPC*x + Gamma_Ts_MPC*v});
    F_kp1 = Function('F_kp1', {x, v, K_d, K_p}, {Phi_Ta*x + Gamma_Ta*v});
    F2 = Function('F2', {x, v, K_d, K_p}, {Phi_Ts_MPC_m_Ta*x + Gamma_Ts_MPC_m_Ta*v});
else
    f = Function('f', {x, v}, {[x2; x3; x4; v - K_p*x3 - K_d*x4]}, opt);
    F_int = integrate_casadi(f, DT, M, int_method); % runs with Ts_MPC
    F = Function('F', {x, v, K_d, K_p}, {F_int(x, v)});

    F_kp1_int = integrate_casadi(f, DT_ctl, M, int_method); % runs with Ta from sensors
    F_kp1 = Function('F_kp1', {x, v, K_d, K_p}, {F_kp1_int(x, v)});

    F2_int = integrate_casadi(f, DT2, M, int_method); % runs with Ts_MPC-Ta
    F2 = Function('F2', {x, v, K_d, K_p}, {F2_int(x, v)});
end

%% Calculate Initial Guess

% Get trajectory data for initial guess
p_d_0       = param_trajectory.p_d( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_0 ... y_N)
q_d_0       = param_trajectory.q_d( 1:4, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_0 ... q_N)

% initial guess for reference trajectory parameter
if(N_step_MPC == 1)
    y_d_0    = [p_d_0; q_d_0];
else
    p_d_0_kp1 = param_trajectory.p_d(  1:3, 2 );
    q_d_0_kp1 = param_trajectory.q_d(  1:4, 2 );
    y_d_0    = [[p_d_0(:,1), p_d_0_kp1, p_d_0(:,2:end-1)]; [q_d_0(:,1), q_d_0_kp1, q_d_0(:,2:end-1)]];
end

% Robot System: Initial guess
x_0_0  = [q_0; q_0_p; q_0_pp; zeros(n, 1)];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xd compute_tau_fun(q_0, dq_0, ddq_0); % gravity compensationof.m
q_0    = x_0_0(1   :   n); % needed?
dq_0   = x_0_0(1+n : 2*n); % needed?
ddq_0  = q_0_pp; % needed?
xe_k_0 = p_d_0(1:3, 1); % x pos, y pos, defined in parameters_xdof.m
u_k_0  = q_0_pp;

v_init_guess_0 = ones(n, N_MPC).* (param_weight.(casadi_func_name).K_P_u * u_k_0); % v = Kp*u in static case
x_init_guess_0 = [x_0_0, ones(4*n, N_MPC).*x_0_0];

lam_x_init_guess_0 = zeros(numel(x_init_guess_0)+numel(v_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0)-2*n, 1);

init_guess_0 = [x_init_guess_0(:); v_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

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
v = SX.sym( 'v',  n,   N_MPC   ); % u_pp + K_d * u_p + K_p * u = q_0_pppp + K_d * q_0_ppp + K_p * q_0_pp
u = v; % only for nlpsol_opt_problem_v2 necessary!
x = SX.sym( 'x',  4*n, N_MPC+1 );

mpc_opt_var_inputs = {x, v};

% v = [v0; v1; ... vN-1]
%
%     [x_0]   [q_0; q_0_p; q_0_pp; q_0_ppp]
% x = [x_1] = [q_1; q_1_p; q_1_pp; q_1_ppp]
%     [...] = [ ...                       ]
%     [x_N]   [q_N; q_N_p; q_N_pp; q_N_ppp]
%
% u   = q_0_pp
% u_p = q_0_ppp
% u_pp = q_0_pppp
% v   = u_pp + K_d * u_p + K_p * u = q_0_pppp + K_d * q_0_ppp + K_p * q_0_pp

N_x = 4*n;
u_opt_indices = (N_x+1 : N_x+3*n); % [q_1, q_1_p, q_1_pp] needed for joint space CT control

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat([pp.x_min; pp.u_min; pp.du_min], N_MPC + 1, 1); repmat(pp.v_min, N_MPC, 1);];
ubw = [repmat([pp.x_max; pp.u_max; pp.du_max], N_MPC + 1, 1); repmat(pp.v_max, N_MPC, 1);];

% input parameter
x_k  = SX.sym( 'x_k',  2*n, 1 ); % current x state = initial x state for q_0, q_0_p
y_d  = SX.sym( 'y_d',  m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d

mpc_parameter_inputs = {x_k, y_d};
mpc_init_reference_values = [x_0_0(1:2*n); y_d_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+1); % for F

if(weights_and_limits_as_parameter)
    lbg = SX(numel(x)-2*n, 1);
    ubg = SX(numel(x)-2*n, 1);
else
    lbg = zeros(numel(x)-2*n, 1);
    ubg = zeros(numel(x)-2*n, 1);
end

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y       = SX(  7, N_MPC+1); % TCP Pose:      (y_0 ... y_N)

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_x(1, 1 + (0)) = {x_k(1:2*n) - x(1:2*n, 1 + (0))}; % x0 = xk

for i=0:N_MPC
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_py_fun(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_endeffector_py_fun(q);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        if(i == 0)
            g_x(1, 1 + (i+1))  = { F_kp1( x(:, 1 + (i)), v(:, 1 + (i)), pp.K_D_u, pp.K_P_u) - x(:, 1 + (i+1)) }; % Set the state constraints for xk = x(t0) = tilde x0 to xk+1 = x(t0+Ta)
        elseif(i == 1)
            g_x(1, 1 + (i+1))  = { F2(    x(:, 1 + (i)), v(:, 1 + (i)), pp.K_D_u, pp.K_P_u) - x(:, 1 + (i+1)) }; % Set the state constraints for xk+1 = x(t0+Ta) to x(t0+Ts_MPC) = tilde x1
        else
            g_x(1, 1 + (i+1))  = { F(     x(:, 1 + (i)), v(:, 1 + (i)), pp.K_D_u, pp.K_P_u) - x(:, 1 + (i+1)) }; % Set the state constraints for x(t0+Ts_MPC*i) to x(t0+Ts_MPC*(i+1))
            % runs only to T_horizon-Ts_MPC, i. e. tilde x_{N-1} = x(t0+Ts_MPC*(N-1)) and x_N doesn't exist
            % Trajectory must be y(t0), y(t0+Ta), Y(t0+Ts_MPC), ..., y(t0+Ts_MPC*(N-1))
        end
    end
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

g = g_x;

J_q_pp = Q_norm_square(x(2*n+1:3*n, :), pp.R_q_pp);% + Q_norm_square(x(1:n, :), pp.R_q_pp) + Q_norm_square(x(n+1:2*n, :), pp.R_q_pp);
J_q_ppp = Q_norm_square(x(3*n+1:4*n, :), pp.R_q_ppp);
% J_v = Q_norm_square(v, pp.R_v);
J_v = Q_norm_square(v - pp.K_P_u * x(2*n+1:3*n,1:N_MPC ), pp.R_v); % weight in relation to stationary value

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp, J_q_ppp, J_v}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);