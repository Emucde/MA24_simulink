% MPC v4: Optimization problem 
% q = [q1, q2, q3, q4, q5, q6, q7]
% ONLY JOINTS A2 (q2) and A4 (q4) are used!

import casadi.*

n = 2; % Dimension of joint space
m = 2; % Dimension of Task Space

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']); % here not needed

q = SX.sym( 'q',  n,   N_MPC   );
pos_indices = [1 3]; % xz plane
q_24_indices = [2 3]; % q2, q4 only (q3 ist deaktiviert daher ist q4 an index 3)
q_subs = vertcat([q_0(1) q(1) q(2) q_0(4) q_0(5) q_0(6)]);
H_2dof = Function('H_2dof', {q}, {hom_transform_endeffector_py_fun(q_subs)});

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
    DT2 = DT - 2*DT_ctl;
end
F2 = integrate_casadi(f, DT2, M, int_method); % runs with Ts_MPC-2*Ta

lambda = -1;
A = [zeros(n), eye(n); -eye(n)*lambda^2, 2*eye(n)*lambda]; % = d/dt x = Ax + Bu = (A + BC)x, u=Cx
Q = 1*eye(2*n); % d/dt V = -x'Qx
P = lyap(A, Q); % V = x'Px => Idea: Use V as end cost term

%% Calculate Initial Guess
if(N_step_MPC <= 3)
    MPC_traj_indices = 1:(N_MPC+1);
else
    MPC_traj_indices = [1, 2, 3, N_step_MPC : N_step_MPC : 1 + (N_MPC-2) * N_step_MPC];
end

p_d_0 = param_trajectory.p_d( pos_indices, MPC_traj_indices ); % only xz plane
y_d_0 = p_d_0;

% Robot System: Initial guess

% einige probleme beim simulieren [TODO]
q_0_red = q_0(q_24_indices);
q_0_p_red = q_0_p(q_24_indices);
q_0_pp_red = q_0_pp(q_24_indices);

x_0_0  = [q_0_red; q_0_p_red];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xd compute_tau_fun(q_0, dq_0, ddq_0); % gravity compensationof.m
u_k_0  = q_0_pp_red;

u_init_guess_0 = ones(n, N_MPC).*u_k_0;
x_init_guess_0 = [x_0_0 ones(2*n, N_MPC).*x_0_0];

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0), 1);

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

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
u     = SX.sym( 'u',  n,   N_MPC   ); % u = q_0_pp
x     = SX.sym( 'x',  2*n, N_MPC+1 );

mpc_opt_var_inputs = {u, x};

% u = [u0; u1; ... uN-1], x = [x0; x1; ... xN] = [q_0;q_0_p;q_1;q_1_p;...;q_N;q_N_p]
% xx = [u; x] = [u0; u1; ... uN-1; x0; x1; ... xN]
%   u0 = q_0_pp = u(   1 :   n) | q_0 = x(     1     : n      ) | q_0_p = x(     1+  n :     2*n)
%   u0 = q_0_pp = xx(  1 :   n) | q_0 = xx(N_u+1     : n+N_u  ) | q_0_p = xx(N_u+1+  n : N_u+2*n)
%   u1 = q_1_pp = u( n+1 : 2*n) | q_1 = x(     1+2*n :     3*n) | q_1_p = x(     1+3*n :     4*n)
%   u1 = q_1_pp = xx(n+1 : 2*n) | q_1 = xx(N_u+1+2*n : N_u+3*n) | q_1_p = xx(N_u+1+3*n : N_u+4*n)
N_u = numel(u);
q0_pp_idx = 1 : n;
u_opt_indices = q0_pp_idx;

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min, size(u, 2), 1); repmat(pp.x_min, size(x, 2), 1)];
ubw = [repmat(pp.u_max, size(u, 2), 1); repmat(pp.x_max, size(x, 2), 1)];

% input parameter
x_k    = SX.sym( 'x_k',     2*n, 1 ); % current x state = initial x state
y_d    = SX.sym( 'y_d',     m, N_MPC+1 );

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
y = SX(m, N_MPC+1); % TCP Pose:      (y_0 ... y_N)

g_x(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk

for i=0:N_MPC
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_2dof(q);
    y(:,   1 + (i)) = H_e(pos_indices, 4);

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        if(i == 0 || i == 1)
            g_x(1, 1 + (i+1))  = { F_kp1(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk = x(t0) = tilde x0 to xk+1 = x(t0+Ta)
        elseif(i == 2)
            g_x(1, 1 + (i+1))  = { F2(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk+1 = x(t0+Ta) to x(t0+Ts_MPC) = tilde x1
        else
            g_x(1, 1 + (i+1))  = { F(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for x(t0+Ts_MPC*i) to x(t0+Ts_MPC*(i+1))
            % runs only to T_horizon-Ts_MPC, i. e. tilde x_{N-1} = x(t0+Ts_MPC*(N-1)) and x_N doesn't exist
            % Trajectory must be y(t0), y(t0+Ta), Y(t0+Ts_MPC), ..., y(t0+Ts_MPC*(N-1))
        end
    end
end

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

J_yt   = Q_norm_square( y(:, 1 + (2:N_MPC-1) ) - y_d(:, 1 + (2:N_MPC-1)), pp.Q_y   );
J_yt   = J_yt + Q_norm_square( y(:, 1 + ( 1) ) - y_d(:, 1 + ( 1       )), pp.Q_ykp1);
J_yt_N = Q_norm_square( y(:, 1 + (  N_MPC  ) ) - y_d(:, 1 + (  N_MPC  )), pp.Q_yN  );

g = g_x;

J_q_pp = Q_norm_square(u, pp.R_q_pp); %Q_norm_square(u, pp.R_u);

cost_vars_names = '{J_yt, J_yt_N, J_q_pp}';


cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);

n=6;