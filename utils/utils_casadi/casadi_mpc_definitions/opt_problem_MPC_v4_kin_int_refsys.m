% MPC v4: Optimization problem 

import casadi.*

% if you have n_red < 6 it is much faster to neglect some positions - but it works also full poses.
yt_indices = param_robot.yt_indices;
yr_indices = param_robot.yr_indices;

n = param_robot.n_DOF; % Dimension of joint space
n_red = param_robot.n_red; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

q_red = SX.sym( 'q',     n_red, 1 );
x_red = SX.sym( 'x',   2*n_red, 1 );

q_subs            = SX(q_0);
q_subs(n_indices) = q_red;

H_red = Function('H_red', {q_red}, {hom_transform_endeffector_py_fun(q_subs)});
quat_fun_red = Function('quat_fun_red', {q_red}, {quat_endeffector_py_fun(q_subs)});

% Discrete system dynamics
N_x = 3*n_red;
x = SX.sym('x', N_x);
v = SX.sym('v', n_red);
lambda_u = SX.sym('lambda_u', n_red, 1);

% integrator for x = [q; q_p; q_pp; q_ppp]
x1 = x(1:n_red);           % x1= q
x2 = x(n_red+1:2*n_red);   % x2 = d/dt q
x3 = x(2*n_red+1:3*n_red); % x3 = d^2/dt^2 q = u
% d/dt x3 = d^3/dt^3 q = -lambda_u x3 + v

M = rk_iter; % RK4 steps per interval
DT_ctl = param_global.Ta/M;
if(N_step_MPC <= 2)
    DT = DT_ctl; % special case if Ts_MPC = Ta
    DT2 = DT_ctl; % special case if Ts_MPC = Ta
else
    DT = N_step_MPC * DT_ctl; % = Ts_MPC
    DT2 = DT - DT_ctl; % = (N_step_MPC - 1) * DT_ctl = (N_MPC-1) * Ta/M = Ts_MPC - Ta
end

opt = struct;
opt.allow_free = true;

%f_red = Function('f_red', {x, v}, {[x2; x3; x4; K_p*v - K_p*x3 - K_d*x4]}, opt);
%f_red = Function('f_red', {x, v}, {[x2; K_p*x3; x4; v - K_p*x3 - K_d*x4]}, opt);
f_red = Function('f_red', {x, v}, {[x2; x3; -diag(lambda_u) * (x3  - v)]}, opt);

F_int = integrate_casadi(f_red, DT, M, int_method); % runs with Ts_MPC
F = Function('F', {x, v, lambda_u}, {F_int(x, v)});

F_kp1_int = integrate_casadi(f_red, DT_ctl, M, int_method); % runs with Ta from sensors
F_kp1 = Function('F_kp1', {x, v, lambda_u}, {F_kp1_int(x, v)});

F2_int = integrate_casadi(f_red, DT2, M, int_method); % runs with Ts_MPC-Ta
F2 = Function('F2', {x, v, lambda_u}, {F2_int(x, v)});

%% Calculate Initial Guess

% initial guess for reference trajectory parameter
if(N_step_MPC <= 2)
    MPC_traj_indices = 1:(N_MPC+1);
else
    MPC_traj_indices = [0, 1, (1:1+(N_MPC-2))*N_step_MPC]+1;
end

p_d_0 = param_trajectory.p_d( 1:3, MPC_traj_indices ); % (y_0 ... y_N)
q_d_0 = param_trajectory.q_d( 1:4, MPC_traj_indices ); % (q_0 ... q_N)
y_d_0 = [p_d_0; q_d_0];

% Robot System: Initial guess
q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);

u_k_0  = q_0_red_pp;
v_0_0  = diag( param_weight.(casadi_func_name).lambda_u(n_indices) ) * u_k_0;
x_0_0  = [q_0_red; q_0_red_p; v_0_0];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xd compute_tau_fun(q_0, dq_0, ddq_0); % gravity compensationof.m

v_init_guess_0 = zeros(n_red, N_MPC); % macht mehr sinn da v = d/dt u ist
x_init_guess_0 = ones(N_x, N_MPC+1) .* x_0_0;

lam_x_init_guess_0 = zeros(numel(x_init_guess_0)+numel(v_init_guess_0), 1);
% lam_g_init_guess_0 = zeros(numel(x_init_guess_0)-n_red, 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0), 1);

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
v = SX.sym( 'v',  n_red,   N_MPC   ); % u_pp + K_d * u_p + K_p * u = q_0_pppp + K_d * q_0_ppp + K_p * q_0_pp
u = v; % only for nlpsol_opt_problem_v2 necessary!
x = SX.sym( 'x',  N_x, N_MPC+1 );

mpc_opt_var_inputs = {x, v};

% v = [v0; v1; ... vN-1]
%
%     [x_0]   [q_0; q_0_p; q_0_pp]
% x = [x_1] = [q_1; q_1_p; q_1_pp]
%     [...] = [ ...              ]
%     [x_N]   [q_N; q_N_p; q_N_pp]
%
% u      = q_0_pp
% d/dt u = -lambda_u * q_0_pp + v
%(1+N_x : 2*N_x)
q0_pp_idx = 1+2*n_red : 3*n_red; % [u0 = q_0_pp] needed for joint space CT control
x1_idx = [1+3*n_red: 5*n_red];
q1_pp_idx = [1+5*n_red : 6*n_red];
u_opt_indices = [q0_pp_idx, x1_idx, q1_pp_idx];

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat([pp.x_min([n_indices, n_indices+n]); pp.u_min(n_indices)], N_MPC + 1, 1); repmat(pp.v_min(n_indices), N_MPC, 1);];
ubw = [repmat([pp.x_max([n_indices, n_indices+n]); pp.u_max(n_indices)], N_MPC + 1, 1); repmat(pp.v_max(n_indices), N_MPC, 1);];

% input parameter
x_k  = SX.sym( 'x_k',  N_x, 1 ); % current x state = initial x state for q_0, q_0_p
y_d  = SX.sym( 'y_d',  m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d

mpc_parameter_inputs = {x_k, y_d};
mpc_init_reference_values = [x_0_0(:); y_d_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+1); % for F

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
y       = SX(  7, N_MPC+1); % TCP Pose:      (y_0 ... y_N)

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

% g_x(1, 1 + (0)) = {x_k(1:2*n_red) - x(1:2*n_red, 1 + (0))}; % x0 = xk
g_x(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk

for i=0:N_MPC
    q = x(1:n_red, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(q);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        if(i == 0)
            g_x(1, 1 + (i+1))  = { F_kp1( x(:, 1 + (i)), v(:, 1 + (i)), pp.lambda_u(n_indices)) - x(:, 1 + (i+1)) }; % Set the state constraints for xk = x(t0) = tilde x0 to xk+1 = x(t0+Ta)
        elseif(i == 1)
            g_x(1, 1 + (i+1))  = { F2(    x(:, 1 + (i)), v(:, 1 + (i)), pp.lambda_u(n_indices)) - x(:, 1 + (i+1)) }; % Set the state constraints for xk+1 = x(t0+Ta) to x(t0+Ts_MPC) = tilde x1
        else
            g_x(1, 1 + (i+1))  = { F(     x(:, 1 + (i)), v(:, 1 + (i)), pp.lambda_u(n_indices)) - x(:, 1 + (i+1)) }; % Set the state constraints for x(t0+Ts_MPC*i) to x(t0+Ts_MPC*(i+1))
            % runs only to T_horizon-Ts_MPC, i. e. tilde x_{N-1} = x(t0+Ts_MPC*(N-1)) and x_N doesn't exist
            % Trajectory must be y(t0), y(t0+Ta), Y(t0+Ts_MPC), ..., y(t0+Ts_MPC*(N-1))
        end
    end
end

g = g_x;

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

if isempty(yt_indices)
    J_yt = 0;
    J_yt_N = 0;
else
    J_yt   = Q_norm_square( y(yt_indices, 1 + (1:N_MPC-1) ) - y_d(yt_indices, 1 + (1:N_MPC-1)), pp.Q_y( yt_indices,yt_indices) );
    J_yt_N = Q_norm_square( y(yt_indices, 1 + (  N_MPC  ) ) - y_d(yt_indices, 1 + (  N_MPC  )), pp.Q_yN(yt_indices,yt_indices) );
end

if isempty(yr_indices)
    J_yr = 0;
    J_yr_N = 0;
else
    J_yr = 0;
    for i=1:N_MPC
        q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(y_d(4:7, 1 + (i))));
        
        if(i < N_MPC)
            J_yr = J_yr + Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_y(3+yr_indices,3+yr_indices)  );
        else
            J_yr_N = Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_yN(3+yr_indices,3+yr_indices)  );
        end
    end
end

J_q_pp = 0*Q_norm_square(x(2*n_red+1:3*n_red, :), pp.R_q_pp(n_indices, n_indices));% + Q_norm_square(x(1:n_red, :), pp.R_q_pp) + Q_norm_square(x(n_red+1:2*n_red, :), pp.R_q_pp);
J_v = Q_norm_square(v, pp.R_v(n_indices, n_indices));
%J_v = Q_norm_square(v - pp.K_P_u*x(2*n_red+1:3*n_red,1:N_MPC ), pp.R_v); % weight in relation to stationary value: v == K_p*u

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_pp, J_v}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);