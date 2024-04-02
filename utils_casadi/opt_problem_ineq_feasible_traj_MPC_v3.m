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

n = param_robot.n_DOF;

% Declare model variables
x = SX.sym('x', 2*n);
u = SX.sym('u', n);

% Model equations
xdot = sys_fun_SX(x, u, param_robot);
f = Function('f', {x, u}, {xdot});

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.

X0 = SX.sym('X0', 2*n);
U  = SX.sym('U', n);
X  = X0;
for j=1:M
    % Runge-Kutta 4th order method
    k1 = f(X, U);
    k2 = f(X + DT/2 * k1, U);
    k3 = f(X + DT/2 * k2, U);
    k4 = f(X + DT * k3, U);
    X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
end
F = Function('F', {X0, U}, {X});

% Discrete y_ref system
z     = SX.sym('z',     2*2);
alpha = SX.sym('alpha',   2);

h_ref = Function('h_ref', {z, alpha}, {[z(3:4); alpha]});

M = 1; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.
Z0    = SX.sym('Z0',    2*2);
ALPHA = SX.sym('ALPHA',   2);
Z     = Z0;
for j=1:M
    % Runge-Kutta 4th order method
    k1 = h_ref(Z,             ALPHA);
    k2 = h_ref(Z + DT/2 * k1, ALPHA);
    k3 = h_ref(Z + DT/2 * k2, ALPHA);
    k4 = h_ref(Z + DT   * k3, ALPHA);
    Z  = Z + DT/6 * (k1 +2*k2 +2*k3 +k4);
end
H = Function('F', {Z0, ALPHA}, {Z});

%% Calculate Initial Guess
y_d_0    = param_trajectory.p_d(    1:2, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_0 ... y_N)
y_p_d_0  = param_trajectory.p_d_p(  1:2, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_p_0 ... y_p_N)
y_pp_d_0 = param_trajectory.p_d_pp( 1:2, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_pp_0 ... y_pp_N)

x_0_0  = [q_0; 0;0];%q1, q2, d/dt q1, d/dt q2
q_0    = x_0_0(1:n); % useless line...
dq_0   = x_0_0(n+1:2*n);
ddq_0  = [0;0];
xe_k_0 = xe0(1:2); % x pos, y pos
u_k_0  = compute_tau(q_0, dq_0, ddq_0, param_robot); % tau1, tau2

u_init_guess_0 = ones(2,N_MPC).*u_k_0;

% für die S-funktion ist der Initial Guess wesentlich!
F_sim              = F.mapaccum(N_MPC);
x_init_guess_kp1_0 = F_sim(x_0_0, u_init_guess_0);
x_init_guess_0     = [x_0_0 full(x_init_guess_kp1_0)];

z_0_0 = [y_d_0(:,1); y_p_d_0(:,1)];
alpha_init_guess_0 = y_pp_d_0(:, 1:end-1);
alpha_N_0 = y_pp_d_0(:,end);

H_sim              = H.mapaccum(N_MPC);
z_init_guess_kp1_0 = H_sim(z_0_0, alpha_init_guess_0);
z_init_guess_0     = [z_0_0 full(z_init_guess_kp1_0)];

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(z_init_guess_0)+numel(alpha_init_guess_0) + numel(alpha_N_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(z_init_guess_0)+1, 1); % + 1 wegen eps

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); z_init_guess_0(:); alpha_init_guess_0(:); alpha_N_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

% get weights from "init_MPC_weight.m"
param_weight_init = param_weight.(casadi_func_name);

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init);
else % hardcoded weights
    pp = param_weight_init;
end

%% Start with an empty NLP

% Optimization Variables:
u     = SX.sym( 'u',       n, N_MPC   );
x     = SX.sym( 'x',     2*n, N_MPC+1 );
z     = SX.sym( 'z',     2*2, N_MPC+1 );
alpha = SX.sym( 'alpha',   2, N_MPC+1 );

mpc_opt_var_inputs = {u, x, z, alpha};

u_opt_indices = 1:n;

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min, N_MPC, 1); repmat(pp.x_min, N_MPC + 1, 1); -Inf(size(z(:))); -Inf(size(alpha(:)))];
ubw = [repmat(pp.u_max, N_MPC, 1); repmat(pp.x_max, N_MPC + 1, 1);  Inf(size(z(:)));  Inf(size(alpha(:)))];

% input parameter
x_k    = SX.sym( 'x_k',    2*n,     1 ); % current x state
z_0    = SX.sym( 'z_0',    2*2,     1 ); % initial z state
y_d    = SX.sym( 'y_d',    2, N_MPC+1 ); % (y_d_0 ... y_d_N)
y_p_d  = SX.sym( 'y_p_d',  2, N_MPC+1 ); % (y_p_d_0 ... y_p_d_N)
y_pp_d = SX.sym( 'y_pp_d', 2, N_MPC+1 ); % (y_pp_d_0 ... y_pp_d_N)

mpc_parameter_inputs = {x_k, z_0, y_d, y_p_d, y_pp_d};
mpc_init_reference_values = [x_0_0(:); z_0_0(:); y_d_0(:); y_p_d_0(:); y_pp_d_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x = cell(1, N_MPC+1); % for F
g_z = cell(1, N_MPC+1); % for H
g_eps = cell(1, 1);

lbg = SX(numel(x)+numel(z)+1, 1);
ubg = SX(numel(x)+numel(z)+1, 1);

lbg(end) = 0;
ubg(end) = pp.epsilon;

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y        = SX( 2, N_MPC+1 ); % TCP position:      (y_0 ... y_N)

% reference trajectory values
y_ref    = SX( 2, N_MPC+1 ); % TCP position:      (y_ref_0 ... y_ref_N)
y_p_ref  = SX( 2, N_MPC+1 ); % TCP velocity:      (y_p_ref_0 ... y_p_ref_N)
y_pp_ref = SX( 2, N_MPC+1 ); % TCP acceleration:  (y_pp_ref_0 ... y_pp_ref_N)

q_pp = SX( n, N_MPC   ); % joint acceleration: (q_pp_0 ... q_pp_N-1)

g_x(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk
g_z(1, 1 + (0)) = {z_0 - z(:, 1 + (0))}; % x0 = xk
for i=0:N_MPC
    % calculate q (q_0 ... q_N) and q_p values (q_p_0 ... q_p_N)
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_casadi_SX(q, param_robot);
    y(:, 1 + (i)) = H_e(1:2, 4); %y_0 wird nicht verwendet

    y_ref(   :, 1 + (i)) = z(    1:2, 1 + (i));
    y_p_ref( :, 1 + (i)) = z(    3:4, 1 + (i));
    y_pp_ref(:, 1 + (i)) = alpha( : , 1 + (i));

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        g_x(1, 1 + (i+1)) = {F(x(:, 1 + (i)), u(    :, 1 + (i))) - x(:, 1 + (i+1))}; % Set the state dynamics constraints
        g_z(1, 1 + (i+1)) = {H(z(:, 1 + (i)), alpha(:, 1 + (i))) - z(:, 1 + (i+1))}; % Set the state dynamics constraints

        dx   = f(x(:, 1 + (i)), u(:, 1 + (i))); % = [d/dt q, d^2/dt^2 q], Alternativ: Differenzenquotient
        q_pp(:, 1 + (i  )) = dx(n+1:2*n, 1);
    end
end
g_eps(1, 1) = {norm_2(y(:,end)-y_ref(:,end))}; % for pp.epsilon
g = [g_x, g_z, g_eps];

Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

J_y        = Q_norm_square( y(        :, 1 + (1:N_MPC-1) ) - y_ref(    :, 1 + (1:N_MPC-1)), pp.Q_y  );

%J_y_pp_ref = Q_norm_square( y_pp_ref( :, 1 + (0:N_MPC) ) - y_pp_d( :, 1 + (0:N_MPC)), pp.Q_y_pp_ref );
%J_y_p_ref  = Q_norm_square( y_p_ref(  :, 1 + (0:N_MPC) ) - y_p_d(  :, 1 + (0:N_MPC)), pp.Q_y_p_ref  );
%J_y_ref    = Q_norm_square( y_ref(    :, 1 + (0:N_MPC) ) - y_d(    :, 1 + (0:N_MPC)), pp.Q_y_ref    );

e_pp = y_pp_ref( :, 1 + (0:N_MPC) ) - y_pp_d( :, 1 + (0:N_MPC));
e_p  = y_p_ref(  :, 1 + (0:N_MPC) ) - y_p_d(  :, 1 + (0:N_MPC));
e    = y_ref(    :, 1 + (0:N_MPC) ) - y_d(    :, 1 + (0:N_MPC));

J_yy_ref = Q_norm_square(e_pp + mtimes(pp.Q_y_p_ref, e_p) + mtimes(pp.Q_y_ref, e), eye(2,2));
J_q_pp = Q_norm_square(q_pp, pp.R_q_pp); %Q_norm_square(u, pp.R_u);

cost_vars_names = '{J_y, J_yy_ref, J_q_pp}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);