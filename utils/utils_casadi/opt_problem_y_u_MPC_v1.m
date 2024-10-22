% MPC v1: Optimization problem 
% States and references
% - x_k: Current state
% - y_n: Predicted TCP Position
% - y_n_ref: Reference values for TCP position

% Cost function (J_d,N)
% Minimizes the deviation between predicted outputs and references over a prediction horizon (N)
% Classic: J = (y - y_ref)_Q_y + u_Ru

% Control Law
% Optimal control input sequence (u_n) minimizes the cost function

% Constraints
% - System dynamics enforced through state equation (F)
% - Initial state set to current state (x_k)
% - Output (y_n) calculated from state (q_n) using output function (h)
% - Velocity (y_p,n) and acceleration (y_pp,n) calculated from state and control input
%   - System dynamics (M, C, g, J_v, J_vp) used for calculation
% - State and control input constrained to admissible sets (X, U)

% Note: Simplified notation used for clarity. Define matrices and functions explicitly in implementation.

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

yt_indices = param_robot.yt_indices;
yr_indices = param_robot.yr_indices;

n = param_robot.n_DOF; % Dimension of joint space
n_red = param_robot.n_red; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

n_x_indices = [n_indices n_indices+n];

% Model equations
% Forward Dynamics: d/dt x = f(x, u)
use_aba = true;
if(use_aba)
    f = Function.load([input_dir, 'sys_fun_x_aba_py.casadi']); % forward dynamics (FD), d/dt x = f(x, u), x = [q; dq]
else
    f = Function.load([input_dir, 'sys_fun_x_sol_py.casadi']); % equivalent as above
end

compute_tau_fun = Function.load([input_dir, 'compute_tau_py.casadi']); % Inverse Dynamics (ID)
gravity_fun = Function.load([input_dir, 'gravitational_forces_py.casadi']); % Inverse Dynamics (ID)
hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

q_red    = SX.sym( 'q',     n_red, 1 );
q_red_p  = SX.sym( 'q_p',   n_red, 1 );
q_red_pp = SX.sym( 'q_pp',  n_red, 1 );
x_red    = SX.sym( 'x',   2*n_red, 1 );
tau_red  = SX.sym( 'tau',   n_red, 1 );

q_subs    = SX(q_0); % assumption: all trajectories have same joint values for locked joints!
q_subs_p  = SX(q_0_p); % assumption: all trajectories have same joint values for locked joints!
q_subs_pp = SX(q_0_pp); % assumption: all trajectories have same joint values for locked joints!
x_subs    = SX([q_0; q_0_p]);

q_subs(n_indices)    = q_red;
q_subs_p(n_indices)  = q_red_p;
q_subs_pp(n_indices) = q_red_pp;
x_subs(n_x_indices) = x_red;

H_red = Function('H_red', {q_red}, {hom_transform_endeffector_py_fun(q_subs)});
quat_fun_red = Function('quat_fun_red', {q_red}, {quat_endeffector_py_fun(q_subs)});

tau_subs = gravity_fun(x_subs(1:n)); % assumption: PD controller ensures gravity compensation on fixed joints
tau_subs(n_indices) = tau_red; % only actuated joints are controlled
d_dt_x = f(x_subs, tau_subs);
tau_full = compute_tau_fun(q_subs, q_subs_p, q_subs_pp);
g_vec_subs = gravity_fun(q_subs);
g_fun_red = Function('g_fun_red', {q_red}, {g_vec_subs(n_indices)});

tau_fun_red = Function('tau_fun_red', {q_red, q_red_p, q_red_pp}, {tau_full(n_indices)});
f_red = Function('f_red', {x_red, tau_red}, {d_dt_x(n_x_indices)});

% Discrete system dynamics

if(N_step_MPC <= 2)
    M = 1; % RK4 steps per interval
    DT_ctl = param_global.Ta/M;
    DT = DT_ctl; % special case if Ts_MPC = Ta
    DT2 = DT_ctl; % special case if Ts_MPC = Ta
else
    M = rk_iter; % RK4 steps per interval
    DT_ctl = param_global.Ta/M;
    DT = N_step_MPC * DT_ctl; % = Ts_MPC
    DT2 = DT - DT_ctl; % = (N_step_MPC - 1) * DT_ctl = (N_MPC-1) * Ta/M = Ts_MPC - Ta
end

F_kp1 = integrate_casadi(f_red, DT_ctl, 1, int_method); % runs with Ta from sensors, no finer integration needed
F2 = integrate_casadi(f_red, DT2, M, int_method); % runs with Ts_MPC-Ta
F = integrate_casadi(f_red, DT, M, int_method); % runs with Ts_MPC

%Get trajectory data for initial guess
if(N_step_MPC <= 2)
    MPC_traj_indices = 1:(N_MPC+1);
else
    MPC_traj_indices = [0, 1, (1:1+(N_MPC-2))*N_step_MPC]+1;
end
dt_int_arr = (MPC_traj_indices(2:end) - MPC_traj_indices(1:end-1))*DT_ctl;
dt_int_arr(:) = 1; % for debugging

p_d_0 = param_trajectory.p_d( 1:3, MPC_traj_indices ); % (y_0 ... y_N)
q_d_0 = param_trajectory.q_d( 1:4, MPC_traj_indices ); % (q_0 ... q_N)
y_d_0 = [p_d_0; q_d_0];

% initial guess for reference trajectory parameter
q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m

u_k_0  = tau_fun_red(q_0_red, q_0_red_p, q_0_red_pp); % gravity compensation

u_init_guess_0 = ones(n_red, N_MPC).*u_k_0; % fully actuated

% für die S-funktion ist der Initial Guess wesentlich!
F_kp1_sim          = F_kp1.mapaccum(1);
F2_sim             = F2.mapaccum(1);
F_sim              = F.mapaccum(N_MPC-2);

x_DT_ctl_init_0 = F_kp1_sim(x_0_0,           u_init_guess_0(:, 1)    );
x_DT2_init_0    = F2_sim(   x_DT_ctl_init_0, u_init_guess_0(:, 2)    );
x_DT_init_0     = F_sim(    x_DT2_init_0,    u_init_guess_0(:, 3:end));

x_init_guess_0     = [x_0_0 full(x_DT_ctl_init_0) full(x_DT2_init_0) full(x_DT_init_0)];
% x_init_guess_0     = x_0_0 * ones(1, N_MPC+1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET INIT GUESS 1/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0), 1);

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

% get weights from "init_MPC_weight.m"
param_weight_init = param_weight.(casadi_func_name);

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init); % Matrizen sind keine Diagonlmatrizen [TODO]
else % hardcoded weights
    pp = param_weight_init;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET OPT Variables 2/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u   = SX.sym( 'u',    n_red, N_MPC   );
x   = SX.sym( 'x',  2*n_red, N_MPC+1 );

mpc_opt_var_inputs = {u, x};

w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')'; % optimization variables cellarray w

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET OPT Variables Limits 3/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); repmat(pp.x_min(n_x_indices), size(x, 2), 1)];
ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1); repmat(pp.x_max(n_x_indices), size(x, 2), 1)];

u_opt_indices = 1:n_red;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET INPUT Parameter 4/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_k    = SX.sym( 'x_k', 2*n_red, 1       ); % current x state
y_d    = SX.sym( 'y_d', m+1,     N_MPC+1 ); % (y_d_0 ... y_d_N)
u_prev    = SX.sym( 'u_prev',   n_red, N_MPC );
x_prev    = SX.sym( 'x_prev', 2*n_red, N_MPC+1 );

mpc_parameter_inputs = {x_k, y_d, u_prev, x_prev};
mpc_init_reference_values = [x_0_0(:); y_d_0(:); u_init_guess_0(:); x_init_guess_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x = cell(1, N_MPC+1); % for F

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET Equation Constraint size 5/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Define Equation Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y     = SX(  7, N_MPC+1); % TCP Pose:      (y_0 ... y_N)

q_pp = SX( n_red, N_MPC   ); % joint acceleration: (q_pp_0 ... q_pp_N-1) % last makes no sense: q_pp_N depends on u_N, wich is not known
q_p = SX( n_red, N_MPC   ); % joint velocity

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_vec = SX(n_red, N_MPC);

g_x(  1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk
for i=0:N_MPC
    % calculate q (q_0 ... q_N) and q_p values (q_p_0 ... q_p_N)
    q = x(1:n_red, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(q);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(i < N_MPC)
        if(i == 0)
            g_x(1, 1 + (i+1))  = { F_kp1(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk = x(t0) = tilde x0 to xk+1 = x(t0+Ta)
        elseif(i == 1)
            g_x(1, 1 + (i+1))  = { F2(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk+1 = x(t0+Ta) to x(t0+Ts_MPC) = tilde x1
        else
            g_x(1, 1 + (i+1))  = { F(  x(:, 1 + (i)), u(:, 1 + (i)) ) - x( :, 1 + (i+1)) }; % Set the state constraints for x(t0+Ts_MPC*i) to x(t0+Ts_MPC*(i+1))
        end

        dx   = f_red(x(:, 1 + (i)), u(:, 1 + (i))); % = [d/dt q, d^2/dt^2 q], Alternativ: Differenzenquotient
        q_p(:,  1 + (i)) = dx(      1:  n_red, 1);
        q_pp(:, 1 + (i)) = dx(n_red+1:2*n_red, 1);

        g_vec(:, 1 + (i)) = g_fun_red(q);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Total number of equation conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = g_x;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Define Cost Function  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

if isempty(yt_indices)
    J_yt = 0;
    J_yt_N = 0;
else
    J_yt = 0;
    for i=1:N_MPC
        e_pos_err = y(yt_indices, 1 + (i) ) - y_d(yt_indices, 1 + (i));
        
        if(i < N_MPC)
            J_yt = J_yt + 1/dt_int_arr(i) * Q_norm_square( e_pos_err, pp.Q_y(yt_indices,yt_indices) );
        else
            J_yt_N = 1/dt_int_arr(i) * Q_norm_square( e_pos_err, pp.Q_y(yt_indices,yt_indices) );
        end
    end
end

if isempty(yr_indices)
    J_yr = 0;
    J_yr_N = 0;
else
    J_yr = 0;
    for i=1:N_MPC
        q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(y_d(4:7, 1 + (i))));
        
        if(i < N_MPC)
            J_yr = J_yr + 1/dt_int_arr(i) * Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_y(3+yr_indices,3+yr_indices)  );
        else
            J_yr_N = 1/dt_int_arr(i) * Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_yN(3+yr_indices,3+yr_indices)  );
        end
    end
end

u_err = u-u_prev;
x_err = x-x_prev;

% u_err = u-u_prev(:, 1);
% x_err = x-x_k;

% u_err = u-g_vec;
%u_err = u;
%u_err = q_pp;

J_q_pp = 0;
for i=0:N_MPC-1
    J_q_pp = J_q_pp + 1/dt_int_arr(1 + (i)) * Q_norm_square(q_pp(:, 1 + (i)), pp.R_q_pp(n_indices, n_indices)); % ist dann eig ziemlich equivalent zu qpp + qp gewichtung (aber am Schnellsten)
end

J_x = 0;
for i=1:N_MPC
    if(i < N_MPC)
        J_x = J_x + 1/dt_int_arr(i) * Q_norm_square(x_err(:, 1 + (i)), pp.R_x(n_x_indices, n_x_indices));
    else
        J_x = J_x + 1/dt_int_arr(end) * Q_norm_square(x_err(:, 1 + (i)), pp.R_x(n_x_indices, n_x_indices));
    end
end

% J_q_pp = Q_norm_square(q_pp, pp.R_q_pp(n_indices, n_indices)) + Q_norm_square(q_p, pp.R_q_pp(n_indices, n_indices));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Define Additional Outputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cost_vars_names = '{J_yt, J_yr, J_yt_N, J_yr_N, J_q_pp, J_x}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = 100*sum([cost_vars_SX{:}]);
% asdf