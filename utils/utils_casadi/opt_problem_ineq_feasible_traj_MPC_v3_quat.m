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

% get weights from "init_MPC_weight.m"
param_weight_init = param_weight.(casadi_func_name);

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init); % Matrizen sind keine Diagonlmatrizen [TODO]
else % hardcoded weights
    pp = param_weight_init;
end

n = param_robot.n_DOF; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space
m_t = param_robot.m_t; % Translational part of Task Space
m_r = param_robot.m_r; % Rotational part of Task Space 

% Declare model variables
x = SX.sym('x', 2*n);
u = SX.sym('u', n);
q_SX = SX.sym('q_SX', n);
dq_SX = SX.sym('dq_SX', n);
ddq_SX = SX.sym('ddq_SX', n);
tau_SX = SX.sym('tau_SX', n);

% Model equations
overwrite_sysfun = false;
sys_fun_path = fullfile(output_dir, 'sys_fun.casadi');
if(exist(sys_fun_path, 'file') || overwrite_sysfun)
    f = Function.load(sys_fun_path);
else
    fprintf('\n')
    disp('Computing system function (can take a while, ~5min)');
    xdot = sys_fun_SX(x, u, param_robot); % ultra slow (5min)
    f = Function('f', {x, u}, {xdot});
    f.save([output_dir, 'sys_fun', '.casadi']);
end

compute_tau_fun_path = fullfile(output_dir, 'compute_tau_fun.casadi');
if(exist(compute_tau_fun_path, 'file') || overwrite_sysfun)
    compute_tau_fun = Function.load(compute_tau_fun_path);
else
    fprintf('\n')
    disp('Computing tau function (can take a while, ~5min)');
    tau_SX = compute_tau_SX(q_SX, dq_SX, ddq_SX, param_robot); % ultra slow (5min)
    compute_tau_fun = Function('compute_tau', {q_SX, dq_SX, ddq_SX}, {tau_SX});
    compute_tau_fun.save([output_dir, 'compute_tau_fun', '.casadi']);
end
% TODO: save M, C and g as casadi functions and use it in sys_fun_SX and compute_tau

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

% EULER (CA Doppelt so schnell wie RK4, aber problematisch bei 5th order poly mit out of workspace)
%X  = X0;
%X = X + f(X, U)*DT;
%F = Function('F', {X0, U}, {X});

% Discrete y_ref system

quat_v1=false;
if(quat_v1)
    z     = SX.sym('z',     2*m);
    alpha = SX.sym('alpha',   m);
    h_ref = Function('h_ref', {z, alpha}, {[z(m+1:2*m); alpha]});
    M = rk_iter; % RK4 steps per interval
    DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.
    Z0    = SX.sym('Z0',    2*m);
    ALPHA = SX.sym('ALPHA',   m);
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
else
    % Achtung: hier kommen Errors rein,
    % Achtung: hier kommen Errors rein, nicht yref!!!!
    e_ref  = SX.sym('e_ref', 2*m+1); % e_ref = [y;quat;y_p;omega]
    K_d = diag(SX.sym('K_d', m, 1));
    K_p = diag(SX.sym('K_p', m, 1));

    e1_ref = e_ref(1:m_t); % e_y_ref transl
    e2_ref = e_ref(m_t+1:m+1); % e_q_ref quaternion (not joint angles!)
    e3_ref = e_ref(m+2:m+4); % e_y_ref_p
    e4_ref = e_ref(m+5:m+7); % e_omega_ref

    K_d_t = K_d(    1:m_t,     1:m_t );
    K_d_r = K_d(m_t+1:m,   m_t+1:m   );
    K_p_t = K_p(    1:m_t,     1:m_t );
    K_p_r = K_p(m_t+1:m,   m_t+1:m   );

    [q_ref_p, Q_q] = quat_deriv(e2_ref, e4_ref); % e2_ref=q_ref, e4_ref=omega_ref, Q_p is 4x3
    Q_eps = Q_q(2:4, :); % 3x3
    q_vec = e2_ref(2:4); % epsilon = q_vec

    e1_ref_p = e3_ref; % e_y_p_ref = y_p_d
    e2_ref_p = q_ref_p; % e_q_p_ref = Q(q_ref) * e_omega_ref
    e3_ref_p = -K_d_t*(e3_ref) -K_p_t*(e1_ref); % = e_y_pp_ref
    e4_ref_p = - K_d_r*e4_ref -2 * ( e2_ref(1)*eye(m_t) + skew(q_vec) ) * K_p_r * q_vec; 
    %e4_ref_p = - K_d_r*e4_ref-4 * q_vec' * K_p_r * q_vec; % ~28% faster, = e4_ref_p = omega_p_ref

    h_ref = Function('h_ref', {e_ref, K_d, K_p}, {[e1_ref_p; e2_ref_p; e3_ref_p; e4_ref_p]});

    M = rk_iter; % RK4 steps per interval
    DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.
    E_REF_0    = SX.sym('E_REF_0',    2*m+1);
    E_REF     = E_REF_0;
    K_D = diag(SX.sym('K_D', m, 1)); % only diag elements
    K_P = diag(SX.sym('K_P', m, 1));
    for j=1:M
        % Runge-Kutta 4th order method
        k1 = h_ref(E_REF            , K_D, K_P);
        k2 = h_ref(E_REF + DT/2 * k1, K_D, K_P);
        k3 = h_ref(E_REF + DT/2 * k2, K_D, K_P);
        k4 = h_ref(E_REF + DT   * k3, K_D, K_P);
        E_REF  = E_REF + DT/6 * (k1 +2*k2 +2*k3 +k4);
    end
    H = Function('H', {E_REF_0, K_D, K_P}, {E_REF});
end
% TODO: Funktion speichern!

%% Calculate Initial Guess
p_d_0    = param_trajectory.p_d(    1:m_t, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_0 ... y_N)
p_d_p_0  = param_trajectory.p_d_p(  1:m_t, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_p_0 ... y_p_N)
p_pp_d_0 = param_trajectory.p_d_pp( 1:m_t, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_pp_0 ... y_pp_N)

R_d_0       = param_trajectory.R_d(       1:m_r, 1:m_r, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (R_0 ... R_N)
q_d_0       = param_trajectory.q_d(       1:4,          1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_0 ... q_N)
q_d_p_0     = param_trajectory.q_d_p(     1:4,          1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_p_0 ... q_p_N)
omega_d_0   = param_trajectory.omega_d(   1:m_r,        1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (omega_0 ... omega_N)
omega_d_p_0 = param_trajectory.omega_d_p( 1:m_r,        1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (omega_p_0 ... omega_p_N)

x_0_0  = [q_0; q_0_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m
q_0    = x_0_0(1:n); % useless line...
dq_0   = x_0_0(n+1:2*n);
ddq_0  = zeros(n,1);
xe_k_0 = xe0(1:m_t); % x pos, y pos, defined in parameters_xdof.m
%u_k_0  = compute_tau(q_0, dq_0, ddq_0, param_robot); % tau1, tau2
u_k_0  = compute_tau_fun(q_0, dq_0, ddq_0); % much more faster than above command

u_init_guess_0 = ones(n, N_MPC).*u_k_0; % fully actuated

% für die S-funktion ist der Initial Guess wesentlich!
F_sim              = F.mapaccum(N_MPC);
x_init_guess_kp1_0 = F_sim(x_0_0, u_init_guess_0);
x_init_guess_0     = [x_0_0 full(x_init_guess_kp1_0)];

z_0_0 = [p_d_0(:,1); q_d_0(:,1); p_d_p_0(:,1); omega_d_0(:,1)]; % init for z_ref
e_ref_0_0 = zeros(size(z_0_0)); % init for e_ref = z_0_0 - yy_d_0: all errors are zero: start on trajectory

z_d_init_guess_0 = [p_d_0; q_d_0; p_d_p_0; omega_d_0]; % init for z_d

% z_0_0 = [y_d_0(:,1); q_d_0(:,1); y_p_d_0(:,1); q_d_p_0(:,1)]; % 14 Dimensional bei m=6

K_d_ref_0 = (param_weight.(casadi_func_name).K_d_ref); % Zahlenwerte aus init_MPC_weights.m
K_p_ref_0 = (param_weight.(casadi_func_name).K_p_ref); % nur notwendig um init guess zu berechnen
%alpha_init_guess_0 = [y_pp_d_0(:, 1:end-1); omega_d_p_0(:, 1:end-1)]; % 6 Dimensional bei m=6
%alpha_N_0 = [y_pp_d_0(:,end); omega_d_p_0(:,end)];

H_sim              = H.mapaccum(N_MPC);
e_ref_init_guess_kp1_0 = H_sim(e_ref_0_0, K_d_ref_0, K_p_ref_0);
z_ref_init_guess_0     = [e_ref_0_0 full(e_ref_init_guess_kp1_0)] + z_d_init_guess_0; % e = z-z_d => z = e+z_d

lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0)+numel(z_ref_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(z_ref_init_guess_0)+1, 1); % + 1 wegen eps

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); z_ref_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

%% Start with an empty NLP

% Optimization Variables:
u = SX.sym( 'u',     n, N_MPC   );
x = SX.sym( 'x',     2*n, N_MPC+1 );
z = SX.sym( 'z_ref', 1+2*m, N_MPC+1 );

% y_ref     = z(1:m_t); % y_ref transl
% q_ref     = z(m_t+1:m+1); % q_ref quaternion (not joint angles!)
% y_p_ref   = z(m+2:m+4); % y_ref_p
% omega_ref = z(m+5:m+7); % omega_ref

mpc_opt_var_inputs = {u, x, z};

u_opt_indices = 1:n;

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
ubw = [repmat(pp.u_max, N_MPC, 1); repmat(pp.x_max, N_MPC + 1, 1);  Inf(size(z(:)));];
lbw = [repmat(pp.u_min, N_MPC, 1); repmat(pp.x_min, N_MPC + 1, 1); -Inf(size(z(:)));];


% input parameter
x_k    = SX.sym( 'x_k',      2*n,      1 ); % current x state
z_0    = SX.sym( 'z_0',    1+2*m,      1 ); % initial z_ref state
z_d    = SX.sym( 'z_d',    1+2*m,      N_MPC+1 ); % z_d states

K_d = pp.K_d_ref;
K_p = pp.K_p_ref;

K_d_t = K_d(1:m_t);
K_d_r = K_d(m_t+1:m);
K_p_t = K_p(1:m_t);
K_p_r = K_p(m_t+1:m);

mpc_parameter_inputs = {x_k, z_0, z_d(:)};

mpc_init_reference_values = [x_0_0(:); z_0_0(:); z_d_init_guess_0(:)]; % Zahlenwerte

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
y    = SX( m+1, N_MPC+1 ); % 
q_pp = SX( n, N_MPC     ); % joint acceleration: (q_pp_0 ... q_pp_N-1) % last makes no sense: q_pp_N depends on u_N, wich is not known

g_x(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk
g_z(1, 1 + (0)) = {z_0 - z(:, 1 + (0))}; % x0 = xk
for i=0:N_MPC
    % calculate q (q_0 ... q_N) and q_p values (q_p_0 ... q_p_N)
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_casadi_SX(q, param_robot);
    y(    1:m_t, 1 + (i)) = H_e(1:m_t, 4);     %y_t_0 wird nicht verwendet
    y(m_t+1:m+1, 1 + (i)) = rotation2quaternion_casadi(H_e(1:m_r, 1:m_r)); %y_r_0 wird nicht verwendet

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        g_x(1, 1 + (i+1)) = {F(x(:, 1 + (i)), u(:, 1 + (i))) - x(:, 1 + (i+1))}; % Set the state dynamics constraints

        e_i_ref   = z(:, 1 + (i))   - z_d(:, 1 + (i));
        e_ip1_ref = z(:, 1 + (i+1)) - z_d(:, 1 + (i+1));
        g_z(1, 1 + (i+1)) = {H(e_i_ref, K_d, K_p) - e_ip1_ref}; % ggf special sub necessary [TODO]

        dx   = f(x(:, 1 + (i)), u(:, 1 + (i))); % = [d/dt q, d^2/dt^2 q], Alternativ: Differenzenquotient
        q_pp(:, 1 + (i  )) = dx(n+1:2*n, 1);
    end
end

% g_eps(1, 1) = {norm_2(  sub_fun_y( y(1:m+1, end),  z(1:m+1, end) )  )}; % for pp.epsilon
g_eps(1, 1) = {norm_2(  y(1:m+1, end) - z(1:m+1, end)  )}; % for pp.epsilon
g = merge_cell_arrays([g_x, g_z, g_eps], 'vector')';

%quat_err_fun1 = @(y1, y2) quat_mult_vec(  y1, quat_inv( y2 )  ); % liefert 3x1 Vektor
% oben muss noch rotm2quatvec gmacht werden, denn abs(q) ist immer 1, selbst wenn der quaternoinenfehler 0 ist.
quat_err_fun1 = @(y1, y2) rotm2quatvec(  quat2rotm_v2( y1 ) * quat2rotm_v2( y2 )'  ); % liefert 3x1 Vektor

sub_fun_y = @(y1, y2) [ y1(1:m_t) - y2(1:m_t); ... 
                        quat_err_fun1(y1(m_t+1:m+1), y2(m_t+1:m+1))]; % liefert 6x1 Vektor
sub_fun_z = @(y1, y2) [ sub_fun_y(y1, y2); ...
                      y1(m+2     : m+1+m_t) - y2(m+2     : m+1+m_t); ...
                      y1(m+2+m_t : 2*m+1  ) - y2(m+2+m_t :   2*m+1)]; % liefert 12x1 Vektor
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

% e = SX( m, N_MPC );
% for i=0:N_MPC-1
%     e(:, 1 + (i)) = sub_fun_y( y(:, 1 + (i)),  z(1:m+1, 1 + (i)) );
% end

e = y(:, 1 + (1:N_MPC-1)) - z(1:m+1, 1 + (1:N_MPC-1));

J_y    = Q_norm_square( e ,    pp.Q_y     );
J_q_pp = Q_norm_square( q_pp,  pp.R_q_pp  ); %Q_norm_square(u, pp.R_u);

cost_vars_names = '{J_y, J_q_pp}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);