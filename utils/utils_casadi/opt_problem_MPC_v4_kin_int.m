% MPC v4: Optimization problem 

import casadi.*

kin_int_mode.standard = 1;
kin_int_mode.du0_cost = 2;
kin_int_mode.du_cost_extended = 3;

mpc_mode = kin_int_mode.standard;

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

% Discrete yt_ref system
x = SX.sym('x', 2*n);
u = SX.sym('u', n);

% integrator for x
f = Function('f', {x, u}, {[x(n+1:2*n); u]});
F = integrate_casadi(f, DT, M, int_method); % runs with Ts_MPC

DT_ctl = param_global.Ta/M;
F_kp1 = integrate_casadi(f, DT_ctl, M, int_method); % runs with Ta from sensors

if(N_step_MPC == 1)
    DT2 = DT_ctl; % special case if Ts_MPC = Ta
else
    DT2 = DT - DT_ctl;
end
F2 = integrate_casadi(f, DT2, M, int_method); % runs with Ts_MPC-Ta

%% Calculate Initial Guess

% Get trajectory data for initial guess
p_d_0       = param_trajectory.p_d( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_0 ... y_N)
q_d_0       = param_trajectory.q_d( 1:4, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_0 ... q_N)

% initial guess for reference trajectory parameter
if(mpc_mode == kin_int_mode.standard || mpc_mode == kin_int_mode.du0_cost)
    if(N_step_MPC == 1)
        y_d_0    = [p_d_0; q_d_0];
    else
        p_d_0_kp1 = param_trajectory.p_d(  1:3, 2 );
        q_d_0_kp1 = param_trajectory.q_d(  1:4, 2 );
        y_d_0    = [[p_d_0(:,1), p_d_0_kp1, p_d_0(:,2:end-1)]; [q_d_0(:,1), q_d_0_kp1, q_d_0(:,2:end-1)]];
    end
elseif(mpc_mode == kin_int_mode.du_cost_extended)
    % N must be odd!
    if(mod(N_MPC, 2) == 0)
        error('N_MPC must be odd for du_cost_extended');
    else
        N_du = round((N_MPC-1)/2);
    end

    p_d_kp1 = param_trajectory.p_d( 1:3, 2 : N_step_MPC : 2 + (N_MPC) * N_step_MPC );
    q_d_kp1 = param_trajectory.q_d( 1:4, 2 : N_step_MPC : 2 + (N_MPC) * N_step_MPC );

    N2 = round((N_MPC+1)/2);

    y_d_k   = [p_d_0(  :, 1:1:N2);   q_d_0(:, 1:1:N2)];
    y_d_kp1 = [p_d_kp1(:, 1:1:N2); q_d_kp1(:, 1:1:N2)];

    y_d_0 = reshape([y_d_k; y_d_kp1], 7, N_MPC+1);
else
    error(['mpc_mode ', num2str(mpc_mode), ' not implemented']);
end

% initial guess for previous input
if(mpc_mode == kin_int_mode.du0_cost)
    u_prev_0 = zeros(n, 1);
elseif(mpc_mode == kin_int_mode.du_cost_extended)
    u_prev_0 = zeros(n, N_du);
elseif(mpc_mode ~= kin_int_mode.standard)
    error(['mpc_mode ', num2str(mpc_mode), ' not implemented']);
end

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

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0), 1);

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

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
u     = SX.sym( 'u',  n,   N_MPC   ); % u = q_0_pp
x     = SX.sym( 'x',  2*n, N_MPC+1 );

mpc_opt_var_inputs = {u, x};

% u = [u0; u1; ... uN-1], x = [x0; x1; ... xN] = [q_0;q_0_p;q_1;q_1_p;...;q_N;q_N_p]
% xx = [u; x] = [u0; u1; ... uN-1; x0; x1; ... xN]
%   u1 = q_1_pp = u( n+1:2*n)   |   q_1 = x(     1+2*n:    3*n),  |   q_1_p = x(     1+3*n:    4*n)
%   u1 = q_1_pp = xx(n+1:2*n)   |   q_1 = xx(N_u+1+2*n:N_u+3*n),  |   q_1_p = xx(N_u+1+3*n:N_u+4*n)
N_u = numel(u);
% Achtung, das ist nicht korrekt! q1 = q(t+Ts_MPC) != q(t+Ta), wenn Ts_MPC > Ta
u_opt_indices = [1+2*n+N_u:N_u+3*n, 1+3*n+N_u:N_u+4*n, 1+n:2*n]; % [q_1, q_1_p, q_1_pp] needed for joint space CT control

%   u0 = q_0_pp = u( 1:n)   |   q_0 = x(     1+n:    2*n),  |   q_0_p = x(     1+2*n:    3*n)
%   u0 = q_0_pp = xx(1:n)   |   q_0 = xx(N_u+1+n:N_u+2*n),  |   q_0_p = xx(N_u+1+2*n:N_u+3*n)
% u_opt_indices = [N_u+1+n:N_u+2*n, N_u+1+2*n:N_u+3*n, 1:n]; % [q_1, q_1_p, q_1_pp] needed for joint space CT control

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min, size(u, 2), 1); repmat(pp.x_min, N_MPC + 1, 1)];
ubw = [repmat(pp.u_max, size(u, 2), 1); repmat(pp.x_max, N_MPC + 1, 1)];

% input parameter
x_k  = SX.sym( 'x_k',  2*n, 1 ); % current x state = initial x state
y_d  = SX.sym( 'y_d',    m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d

if(mpc_mode == kin_int_mode.standard)
    mpc_parameter_inputs = {x_k, y_d};
    mpc_init_reference_values = [x_0_0(:); y_d_0(:)];
elseif(mpc_mode == kin_int_mode.du0_cost)
    u1_prev = SX.sym( 'u1_prev', size(u,1), 1 ); % previous q_pp = [qpp0, qpp1, qpp2, ... qppN-1]
    mpc_parameter_inputs = {x_k, y_d, u1_prev};
    mpc_init_reference_values = [x_0_0(:); y_d_0(:); u_prev_0(:)];
elseif(mpc_mode == kin_int_mode.du_cost_extended)
    u_prev = SX.sym( 'u_prev', size(u, 1), N_du); % previous q_pp = [qpp1, qpp3, qpp5, ... qppN-1]
    mpc_parameter_inputs = {x_k, y_d, u_prev};
    mpc_init_reference_values = [x_0_0(:); y_d_0(:); u_prev_0(:)];
else
    error(['mpc_mode ', num2str(mpc_mode), ' not implemented']);
end

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+1); % for F

if(weights_and_limits_as_parameter)
    lbg = SX(numel(x), 1);
    ubg = SX(numel(x), 1);
else
    lbg = zeros(numel(x), 1);
    ubg = zeros(numel(x), 1);
end

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y       = SX(  7, N_MPC+1); % TCP Pose:      (y_0 ... y_N)

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_x(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk

for i=0:N_MPC
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_py_fun(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_endeffector_py_fun(q);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(i < N_MPC)
        if(mpc_mode == kin_int_mode.standard || mpc_mode == kin_int_mode.du0_cost)
            % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
            if(i == 0)
                g_x(1, 1 + (i+1))  = { F_kp1(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk = x(t0) = tilde x0 to xk+1 = x(t0+Ta)
            elseif(i == 1)
                g_x(1, 1 + (i+1))  = { F2(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk+1 = x(t0+Ta) to x(t0+Ts_MPC) = tilde x1
            else
                g_x(1, 1 + (i+1))  = { F(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for x(t0+Ts_MPC*i) to x(t0+Ts_MPC*(i+1))
                % runs only to T_horizon-Ts_MPC, i. e. tilde x_{N-1} = x(t0+Ts_MPC*(N-1)) and x_N doesn't exist
                % Trajectory must be y(t0), y(t0+Ta), Y(t0+Ts_MPC), ..., y(t0+Ts_MPC*(N-1))
            end
        elseif(mpc_mode == kin_int_mode.du_cost_extended)
            % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
            if(mod(i,2) == 0)
                g_x(1, 1 + (i+1))  = { F_kp1(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % x(t0+iTa) = F(x(t0+(i-1)Ta), u(t0+(i-1)Ta)
            else
                g_x(1, 1 + (i+1))  = { F2(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % x(t0+i(Ts_MPC-Ta)) = F(x(t0+(i-1)(Ts_MPC-Ta)), u(t0+(i-1)(Ts_MPC-Ta))
            end
            % N must be odd!
            % so you get the states
            %   x = [x(t0), x(t0+Ta), x(t0+Ts_MPC), x(t0+Ts_MPC+Ta), ... x(t0+((N-1)/2-1)*Ts_MPC), x(t0+((N-1)/2-1)*Ts_MPC + Ta)]
            % 
            % and 
            % 
            % g_x = 
            % {
            % x0   - xk               = x(t0)                            - xk,
            % x1   - F (x0, u0)       = x(t0                      + Ta ) - F(  x(t0                             ), u(t0                             )),
            % x2   - F2(x1, u1)       = x(t0 + (1)       * Ts_MPC      ) - F2( x(t0                        + Ta ), u(t0                        + Ta )),
            % x3   - F (x2, u2)       = x(t0 + (1)       * Ts_MPC + Ta ) - F(  x(t0 + (1)         * Ts_MPC      ), u(t0 + (1)         * Ts_MPC      )),
            % ...
            % xN-1 - F2(x_N-2, u_N-2) = x(t0 + ((N-1)/2) * Ts_MPC      ) - F2( x(t0 + ((N-1)/2-2) * Ts_MPC + Ta ), u(t0 + ((N-1)/2-2) * Ts_MPC + Ta )),
            % xN   - F (x_N-1, u_N-1) = x(t0 + ((N-1)/2) * Ts_MPC + Ta ) - F(  x(t0 + ((N-1)/2-1) * Ts_MPC      ), u(t0 + ((N-1)/2-1) * Ts_MPC      )),
            % }
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

J_q_pp = Q_norm_square(u, pp.R_q_pp); %Q_norm_square(u, pp.R_u);

if(mpc_mode == kin_int_mode.standard)
    cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp}';
elseif(mpc_mode == kin_int_mode.du0_cost)
    J_du = Q_norm_square((u(:, 1) - u1_prev), pp.R0_du);
    cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp, J_du}';
elseif(mpc_mode == kin_int_mode.du_cost_extended)
    J_du = Q_norm_square((u(:, 1:2:end-1) - u_prev), pp.R_du) + Q_norm_square((u(:, 2) - u(:, 1)), pp.R0_du);
    cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp, J_du}';
else
    error(['mpc_mode ', num2str(mpc_mode), ' not implemented']);
end


cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);