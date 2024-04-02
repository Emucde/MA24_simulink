% MPC v2: Optimization problem 
% States and references
% - x_k: Current state
% - y_n, y_p, y_pp: Predicted output, velocity, and acceleration at step n
% - y_n_ref, y_p_ref, y_pp_ref: Reference values for output, velocity, and acceleration

% Cost function (J_d,N)
% Minimizes the deviation between predicted outputs and references over a prediction horizon (N)
% - Stage Cost (D_0 to D_N): Penalizes deviation at each step
%   - Weighting matrices (Q_y, Q_p, Q_pp) determine importance of tracking each term
% - Terminal Cost (D_N, grey term): Penalizes deviation at final step (with weight of zero, currently)
% - Summation: Accumulates stage costs from step 2 to N-1
%   J_d,N(k, x_k, (u_n)) =
%     (y_pp,0 - y_pp,0_ref)' * Q_y_pp + ... % Stage Cost (D_0)   
%     sum( ...  % Loop over prediction steps (n=1 to N-1)
%         [ (y_n - y_n_ref)' * Q_y; ...  % Deviation of predicted output
%           (y_p,n - y_p,n_ref)' * Q_p; ...  % Deviation of predicted velocity
%           (y_pp,n - y_pp,n_ref)' * Q_pp ] ...  % Deviation of predicted acceleration
%     ) + ...
%     (y_N - y_N_ref)' * Q_y;  % Deviation of predicted final output (weight is zero)

% Control Law
% Optimal control input sequence (u_n) minimizes the cost function

% Optimization Problem formulation (MPC)
% min  - minimizes the cost function J_d,N(k, x_k, (u_n))
% st   - subject to:
%       x_(n+1) = F(x_n, u_n)                                        - System dynamics constraint
%       x_0 = x_k                                                    - Initial state constraint
%       [q_n; q_p,n] = x_n                                           - Separate state into position (q_n) and velocity (q_p,n)
%       q_pp,n = M^-1(q_n) * (u_n - C(q_n, q_p,n) * q_p,n - g(q_n))  - Predicted acceleration dynamics
%                                                                      - M: Inertia matrix (assumed invertible)
%                                                                      - C: Coriolis and centripetal forces function
%                                                                      - g: Gravitational forces function
%       y_n = h(q_n)                                                 - Output calculation constraint
%       y_p,n = J_v(q_n) * q_p,n                                     - Predicted velocity calculation (using velocity Jacobian J_v)
%       y_pp,n = J_v(q_n) * q_pp,n + J_v,p(q_n, q_p,n) * q_p,n       - Predicted acceleration calculation (using J_v and J_v,p)
%       x_n ∈ X                                                      - State constraint
%       u_n ∈ U                                                      - Control input constraint

% Note: Simplified notation used for clarity. Define matrices and functions explicitly in implementation.

import casadi.*

n = param_robot.n_DOF;

% Declare model variables
x = SX.sym('x', 2*n);
u = SX.sym('u', n);

% Model equations
xdot = sys_fun_SX(x, u, param_robot);
f = Function('f', {x, u}, {xdot});

% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.

X0 = SX.sym('X0', 2*n);
U = SX.sym('U', n);
X = X0;
for j=1:M
    % Runge-Kutta 4th order method
    k1 = f(X, U);
    k2 = f(X + DT/2 * k1, U);
    k3 = f(X + DT/2 * k2, U);
    k4 = f(X + DT * k3, U);
    X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
end

F = Function('F', {X0, U}, {X});

%% Calculate Initial Guess
x_0_0  = [q_0; 0;0];%q1, q2, d/dt q1, d/dt q2
q_0    = x_0_0(1:n); % useless line...
dq_0   = x_0_0(n+1:2*n);
ddq_0  = [0;0];
xe_k_0 = xe0(1:2); % x pos, y pos
u_k_0  = compute_tau(q_0, dq_0, ddq_0, param_robot); % tau1, tau2

u_init_guess_0 = ones(2,N_MPC).*u_k_0;

% für die S-funktion ist der Initial Guess wesentlich!
sim                = F.mapaccum(N_MPC);
x_init_guess_kp1_0 = sim(x_0_0, u_init_guess_0);
x_init_guess_0     = [x_0_0 full(x_init_guess_kp1_0)];

lam_x_init_guess_0 = zeros(numel([x_init_guess_0(:); u_init_guess_0(:)]), 1);
lam_g_init_guess_0 = zeros(numel([x_init_guess_0(:)]), 1);

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

y_ref_0            = param_trajectory.p_d(    1:2, 1 + N_step_MPC : N_step_MPC : 1 + (N_MPC  ) * N_step_MPC ); % (y_1 ... y_N)
y_p_ref_0          = param_trajectory.p_d_p(  1:2, 1 + N_step_MPC : N_step_MPC : 1 + (N_MPC  ) * N_step_MPC ); % (y_p_1 ... y_p_N)
y_pp_ref_0         = param_trajectory.p_d_pp( 1:2, 1              : N_step_MPC : 1 + (N_MPC-1) * N_step_MPC ); % (y_pp_0 ... y_pp_N-1)

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
u = SX.sym( 'u',   n, N_MPC   );
x = SX.sym( 'x', 2*n, N_MPC+1 );

mpc_opt_var_inputs = {u, x};

u_opt_indices = 1:n;

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min, N_MPC, 1); repmat(pp.x_min, N_MPC + 1, 1)];
ubw = [repmat(pp.u_max, N_MPC, 1); repmat(pp.x_max, N_MPC + 1, 1)];

% input parameter
x_k      = SX.sym( 'x_k',      2*n, 1       ); % current state
y_ref    = SX.sym( 'y_ref',    2,   N_MPC   ); % (y_ref_1 ... y_ref_N)
y_p_ref  = SX.sym( 'y_p_ref',  2,   N_MPC   ); % (y_p_ref_1 ... y_p_ref_N)
y_pp_ref = SX.sym( 'y_pp_ref', 2,   N_MPC   ); % (y_pp_ref_0 ... y_pp_ref_N-1)

mpc_parameter_inputs = {x_k, y_ref, y_p_ref, y_pp_ref};
mpc_init_reference_values = [x_0_0(:); y_ref_0(:); y_p_ref_0(:); y_pp_ref_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g = cell(1, N_MPC+1);
lbg = zeros(numel(x), 1);
ubg = zeros(numel(x), 1);

% lambda_x0, lambda_g0 initial guess
lambda_x0 = SX.sym('lambda_x0', size(w));
lambda_g0 = SX.sym('lambda_g0', size(lbg));

% Actual TCP data: y_0 und y_p_0 werden nicht verwendet
y    = SX( 2, N_MPC+1 ); % TCP position:      (y_0 ... y_N)
y_p  = SX( 2, N_MPC+1 ); % TCP velocity:      (y_p_0 ... y_p_N)
y_pp = SX( 2, N_MPC   ); % TCP acceleration:  (y_pp_0 ... y_pp_N-1)

q    = SX( n, N_MPC+1 ); % joint velocity:     (q_0 ... q_N)
q_p  = SX( n, N_MPC+1 ); % joint velocity:     (q_p_0 ... q_p_N)
q_pp = SX( n, N_MPC   ); % joint acceleration: (q_pp_0 ... q_pp_N-1)

g(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk
for i=0:N_MPC
    % calculate q (q_0 ... q_N) and q_p values (q_p_0 ... q_p_N)
    q(:,   1+ (i)) = x(1:n,     1 + (i));
    q_p(:, 1+ (i)) = x(n+1:2*n, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = hom_transform_endeffector_casadi_SX(q(:, 1 + (i)), param_robot);
    y(:, 1 + (i)) = H_e(1:2, 4); %y_0 wird nicht verwendet

    % calculate trajectory values (y_p_0 ... y_p_N)
    J = geo_jacobian_endeffector_casadi_SX(   q(:, 1 + (i)), param_robot);
    y_p(:, 1 + (i)) = J(1:2, 1:2) *         q_p(:, 1 + (i)); %y_p_0 wird nicht verwendet
    % or y(:, 0) = {H_e(x_k) or (y(:, i+1) - y(:, i))/param_global.Ta} for i=1
    % and y_p(:, i) = (y(:, i-1) - y(:, i))/param_global.Ta for i>1 and

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        g(1,    1 + (i+1)) = {F(x(:, 1 + (i)), u(:, 1 + (i))) - x(:, 1 + (i+1))}; % Set the state dynamics constraints

        % calculate q_pp values (q_pp_0 ... q_pp_N-1)
        dx   = f(x(:, 1 + (i)), u(:, 1 + (i))); % = [d/dt q, d^2/dt^2 q], Alternativ: Differenzenquotient
        q_pp(:, 1 + (i  )) = dx(n+1:2*n, 1);

        % calculate trajectory values (y_pp_0 ... y_N-1)
        J_p = geo_jacobian_endeffector_p_casadi_SX(q(:, 1 + (i)), q_p(:, 1 + (i)), param_robot);
        y_pp(:, 1 + (i  )) = J(1:2, 1:2)*q_pp(:, 1 + (i)) + J_p(1:2, 1:2)*q_p(:, 1 + (i));
        % or y_pp(:, i) = (y(:, i+2) - 2*y(:, i+1) + y(:, i))/param_global.Ta^2 for i=1
        % and y_pp(:, i) = (y(:, i-2) - 2*y(:, i-1) + y(:, i))/param_global.Ta^2 for i>1
    end
end

Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

% Achtung: % index 0 von y und y_p is y_1 bzw. y1_p, Index 0 von y_pp ist y0_pp
D_0    = Q_norm_square( y_pp( :, 1 + (0)         ) - y_pp_ref( :, 1 + (0)        ), pp.Q_y0_pp );

D_1    = Q_norm_square( y(    :, 1 + (1)         ) - y_ref(    :, 0 + (1)        ), pp.Q_y1    ) + ...
         Q_norm_square( y_p(  :, 1 + (1)         ) - y_p_ref(  :, 0 + (1)        ), pp.Q_y1_p  ) + ...
         Q_norm_square( y_pp( :, 1 + (1)         ) - y_pp_ref( :, 1 + (1)        ), pp.Q_y1_pp );

D_N    = Q_norm_square( y(    :, 1 + (N_MPC)     ) - y_ref(    :, 0 + (N_MPC)    ), pp.Q_yN    ) + ...
         Q_norm_square( y_p(  :, 1 + (N_MPC)     ) - y_p_ref(  :, 0 + (N_MPC)    ), pp.Q_yN_p  );

J_y    = Q_norm_square( y(    :, 1 + (2:N_MPC-1) ) - y_ref(    :, 0 + (2:N_MPC-1)), pp.Q_y     );
J_y_p  = Q_norm_square( y_p(  :, 1 + (2:N_MPC-1) ) - y_p_ref(  :, 0 + (2:N_MPC-1)), pp.Q_y_p   );
J_y_pp = Q_norm_square( y_pp( :, 1 + (2:N_MPC-1) ) - y_pp_ref( :, 1 + (2:N_MPC-1)), pp.Q_y_pp  );

C_0    = Q_norm_square( q_pp( :, 1 + (0)                                         ), pp.Q_q0_pp );

C_N    = Q_norm_square( q_p(  :, 1 + (N_MPC)                                     ), pp.Q_qN_p  );

J_q_p  = Q_norm_square( q_p(  :, 1 + (1:N_MPC-1)                                 ), pp.Q_q_p   );
J_q_pp = Q_norm_square( q_pp( :, 1 + (1:N_MPC-1)                                 ), pp.Q_q_pp  );

cost_vars_names = '{J_y, J_y_p, J_y_pp, D_0, D_1, D_N, J_q_p, J_q_pp, C_0, C_N}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);