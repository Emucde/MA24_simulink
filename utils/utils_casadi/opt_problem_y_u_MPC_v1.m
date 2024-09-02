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

n = param_robot.n_DOF; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

% Model equations
% Forward Dynamics: d/dt x = f(x, u)
use_aba = false;
if(use_aba)
    f = Function.load([input_dir, 'sys_fun_x_aba_py.casadi']); % forward dynamics (FD), d/dt x = f(x, u), x = [q; dq]
else
    f = Function.load([input_dir, 'sys_fun_x_sol_py.casadi']); % equivalent as above
end

compute_tau_fun = Function.load([input_dir, 'compute_tau_py.casadi']); % Inverse Dynamics (ID)
hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);
J = Function.load([input_dir, 'geo_jacobian_endeffector_py.casadi']);
J_p = Function.load([input_dir, 'geo_jacobian_endeffector_p_py.casadi']);
gravity_vector_py_fun = Function.load([input_dir, 'gravitational_forces_py.casadi']);

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.

F = integrate_casadi(f, DT, M, int_method);

DT_ctl = param_global.Ta/M;
F_kp1 = integrate_casadi(f, DT_ctl, M, int_method); % runs with Ta from sensors

if(N_step_MPC == 1)
    DT2 = DT_ctl; % special case if Ts_MPC = Ta
else
    DT2 = DT - DT_ctl;
end
F2 = integrate_casadi(f, DT2, M, int_method); % runs with Ts_MPC-2*Ta

% Get trajectory data for initial guess
p_d_0       = param_trajectory.p_d( 1:3, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (y_0 ... y_N)
q_d_0       = param_trajectory.q_d( 1:4, 1 : N_step_MPC : 1 + (N_MPC) * N_step_MPC ); % (q_0 ... q_N)

% initial guess for reference trajectory parameter
% [TODO: not all cases implemented]
if(N_step_MPC == 1)
    y_d_0    = [p_d_0; q_d_0];
else
    p_d_0_kp1 = param_trajectory.p_d(  1:3, 2 );
    q_d_0_kp1 = param_trajectory.q_d(  1:4, 2 );
    y_d_0    = [[p_d_0(:,1), p_d_0_kp1, p_d_0(:,2:end-1)]; [q_d_0(:,1), q_d_0_kp1, q_d_0(:,2:end-1)]];
end

x_0_0  = [q_0; q_0_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m
q_0    = x_0_0(1   :   n); % useless line...
dq_0   = x_0_0(1+n : 2*n);
ddq_0  = q_0_pp;
xe_k_0 = p_d_0(1:3, 1); % x pos, y pos, defined in parameters_xdof.m
u_k_0  = compute_tau_fun(q_0, dq_0, ddq_0); % much more faster than above command

u_init_guess_0 = ones(n, N_MPC).*u_k_0; % fully actuated

% für die S-funktion ist der Initial Guess wesentlich!
F_sim              = F.mapaccum(N_MPC);
x_init_guess_kp1_0 = F_sim(x_0_0, u_init_guess_0);
x_init_guess_0     = [x_0_0 full(x_init_guess_kp1_0)];

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
u   = SX.sym( 'u',    n, N_MPC   );
x   = SX.sym( 'x',  2*n, N_MPC+1 );

mpc_opt_var_inputs = {u, x};

w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')'; % optimization variables cellarray w

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET OPT Variables Limits 3/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lbw = [repmat(pp.u_min, size(u, 2), 1); repmat(pp.x_min, size(x, 2), 1)];
ubw = [repmat(pp.u_max, size(u, 2), 1); repmat(pp.x_max, size(x, 2), 1)];
% lbw = [repmat(pp.u_min, size(u, 2), 1); repmat(pp.x_min, size(x, 2), 1)];
% ubw = [repmat(pp.u_max, size(u, 2), 1); repmat(pp.x_max, size(x, 2), 1)];

u_opt_indices = 1:n;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET INPUT Parameter 4/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_k    = SX.sym( 'x_k', 2*n, 1       ); % current x state
y_d    = SX.sym( 'y_d', m+1, N_MPC+1 ); % (y_d_0 ... y_d_N)

mpc_parameter_inputs = {x_k, y_d};
mpc_init_reference_values = [x_0_0(:); y_d_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x = cell(1, N_MPC+1); % for F

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET Equation Constraint size 5/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(weights_and_limits_as_parameter)
    lbg = SX(numel(x), 1);
    ubg = SX(numel(x), 1);
    % lbg = SX(numel(x), 1);
    % ubg = SX(numel(x), 1);
else
    lbg = zeros(numel(x), 1);
    ubg = zeros(numel(x), 1);
end

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Define Equation Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y     = SX(  7, N_MPC+1); % TCP Pose:      (y_0 ... y_N)

q_pp = SX( n, N_MPC   ); % joint acceleration: (q_pp_0 ... q_pp_N-1) % last makes no sense: q_pp_N depends on u_N, wich is not known

R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

g_x(  1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk
for i=0:N_MPC
    % calculate q (q_0 ... q_N) and q_p values (q_p_0 ... q_p_N)
    q = x(1:n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_py_fun(q);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_endeffector_py_fun(q);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);

    if(i < N_MPC)
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

        dx   = f(x(:, 1 + (i)), u(:, 1 + (i))); % = [d/dt q, d^2/dt^2 q], Alternativ: Differenzenquotient
        q_pp(:, 1 + (i  )) = dx(n+1:2*n, 1);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Total number of equation conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = g_x;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Define Cost Function  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

Q_ori = 0;
for i=0:N_MPC
    % R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))';
    % % q_y_y_err = rotation2quaternion_casadi( R_y_yr );
    % q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)]; %ungenau aber schneller (flipping?)
    q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(y_d(4:7, 1 + (i))));

    if(i < N_MPC)
        Q_ori = Q_ori + Q_norm_square( q_y_yr_err(2:4) , pp.Q_y(4:6, 4:6)  );
    else
        Q_ori_N = Q_norm_square( q_y_yr_err(2:4) , pp.Q_yN(4:6, 4:6)  );
    end
end

J_yt = Q_norm_square( y(1:3, 1 + (0:N_MPC-1) ) - y_d(1:3, 1 + (0:N_MPC-1)), pp.Q_y(1:3,1:3)  );
J_yr = Q_ori;

J_yt_N    = Q_norm_square(  y( 1:3, 1 + (N_MPC) ) - y_d( 1:3, 1 + (N_MPC) ), pp.Q_yN(1:3, 1:3)  );
J_yr_N    = Q_ori_N;

% J_q_pp = Q_norm_square(u, pp.R_q_pp);

% J_q_d_pp = Q_norm_square(u_d, pp.R_q_d);
J_q_pp = Q_norm_square(q_pp, pp.R_q_pp);
% J_u = Q_norm_square(u, pp.R_u);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Define Additional Outputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cost_vars_names = '{J_yt, J_yr, J_yt_N, J_yr_N, J_q_pp}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);