% % MPC v1: Optimization problem

% States:
% - x: predicted states for n degrees of freedom (DoF) [joint positions, velocities]
% - y: predicted task-space outputs representing the manipulator's pose [position and orientation]
% - q_pp: joint accelerations to be determined over the prediction horizon

% Parameters:
% - x_k: Current state of the system [joint positions, velocities] at the initial time step
% - y_d: Desired trajectory stacked vector, which includes:
%   - p_d: desired positions in task space
%   - q_d: desired orientations in quaternion representation
%   - y_p: desired velocity (derivative of y_d)
% - x_prev: previous state for comparison to decrease oscillations

% Cost function (J_d,N):
% Minimizes the deviation between predicted task-space outputs (y_n) and the desired trajectory (y_d) over a prediction horizon (N)
% J_d,N(k, x_k, (u_n)) = sum( ...                   % Loop over prediction steps (n=1 to N)
%   [ ||y_n       - y_ref_n ||_2^2 * Q_y; ...       % Deviation between predicted output and desired trajectory
%     ||u_n - g_n||_2^2 * R_u; ...                 % Control effort penalty
%     ||q_pp_n - q_pp_ref_n||_2^2 * R_q_pp; ...    % Acceleration deviation penalty
%     ||x_n - x_prev,n ||_2^2 * R_x] ...            % State deviation from previous state
%     ||y_N - y_N_ref||_2^2 * Q_y_terminal];        % Soft terminal output constraint for final state
% );

% Control Law:
% Optimal control input sequence (u_n) is derived from the solutions to minimize the cost function (J_d,N), 
% ensuring that the manipulator follows the desired trajectory (y_d) while maintaining stability through previous state feedback.

% Optimization Problem formulation (MPC)
% min J_d,N(k, x_k, (u_n))
% s.t.  x_(n+1) = F(x_n, u_n)                   - Non-linear system dynamics constraint
%       x_0 = x_k                               - Initial state constraint
%       [y_n] = h(x_n)                          - Output calculation constraint connecting state to output
%       y_n = [p_n; q_n]                        - Position and orientation outputs from state
%       y_n ∈ Y                                 - Output constraints
%       u_n ∈ U                                 - Control input constraints

import casadi.*

% get weights from "init_MPC_weight.m"
param_weight_init = param_weight.(casadi_func_name);

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init); % Matrizen sind keine Diagonlmatrizen [TODO]
else % hardcoded weights
    pp = param_weight_init;
end

yt_indices = param_robot.yt_indices;
yr_indices = param_robot.yr_indices;

n = param_robot.n_DOF; % Dimension of joint space
n_red = param_robot.n_red; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

n_x_indices = [n_indices n_indices+n];

% Model equations
% Forward Dynamics: d/dt x = f(x, u)

gravity = false;
use_aba = false;
[f, compute_tau_fun, gravity_fun, hom_transform_endeffector_py_fun, quat_endeffector_py_fun] = ...
    load_robot_dynamics(input_dir, n, gravity, use_aba);

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
M = rk_iter; % RK4 steps per interval
N_step = pp.N_step;

DT_ctl = param_global.Ta/M;
DT = N_step * DT_ctl; % = Ts_MPC
DT2 = if_else(N_step > 1, DT - DT_ctl, DT_ctl); % = (N_step_MPC - 1) * DT_ctl = (N_MPC-1) * Ta/M = Ts_MPC - Ta

F_kp1 = integrate_casadi(f_red, DT_ctl, M, int_method); % runs with Ta from sensors, no finer integration needed
F2 = integrate_casadi(f_red, DT2, M, int_method); % runs with Ts_MPC-Ta
F = integrate_casadi(f_red, DT, M, int_method); % runs with Ts_MPC

%% Calculate Initial Guess
%Get trajectory data for initial guess
time_points = SX(1, N_MPC+1);
time_points(1:2) = [0; DT_ctl];

for i = 0:N_MPC-2
    time_points(i+3) = DT_ctl + DT2 + i * DT; % Concatenate each term
end

MPC_traj_indices = time_points/DT_ctl + 1;
MPC_traj_indices_fun = Function('MPC_traj_indices_fun', {N_step}, {MPC_traj_indices});
dt_int_arr = (MPC_traj_indices(2:end) - MPC_traj_indices(1:end-1))*DT_ctl;

MPC_traj_indices_val = round(full(MPC_traj_indices_fun(N_step_MPC)));
p_d_0 = param_trajectory.p_d( 1:3, MPC_traj_indices_val ); % (y_0 ... y_N)
q_d_0 = param_trajectory.q_d( 1:4, MPC_traj_indices_val ); % (q_0 ... q_N)
y_d_0 = [p_d_0; q_d_0];

% initial guess for reference trajectory parameter
q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m
u_k_0  = full(tau_fun_red(q_0_red, q_0_red_p, q_0_red_pp)); % gravity compensation

u_init_guess_0 = ones(1, N_MPC).*u_k_0; % fully actuated
x_init_guess_0 = ones(1, N_MPC+1).*x_0_0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET INIT GUESS 1/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lam_x_init_guess_0 = zeros(numel(u_init_guess_0)+numel(x_init_guess_0), 1);
lam_g_init_guess_0 = zeros(numel(x_init_guess_0)+numel(u_init_guess_0), 1);

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

if(any(isnan(full(init_guess_0))))
    error('init_guess_0 contains NaN values!');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET OPT Variables 2/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u   = SX.sym( 'u',    n_red, N_MPC   );
x   = SX.sym( 'x',  2*n_red, N_MPC+1 );

mpc_opt_var_inputs = {u, x};

w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')'; % optimization variables cellarray w

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET OPT Variables Limits 3/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); -inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1)];
% ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1);  inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1)];

% Interessant: Wenn u auch im Horizont beschränkt wird, erhält man eine bleibende Regelabweichung (1e-5).
% Hingegen wenn die u im Horizont nicht beschränkt werden, sondern nur das, dass ausgegeben wird, hat man das Problem nicht:
ubw = [repmat(pp.u_max(n_indices), 1, 1);  inf(n_red*(N_MPC-1),1);  inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1)];
lbw = [repmat(pp.u_min(n_indices), 1, 1); -inf(n_red*(N_MPC-1),1); -inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1)];

N_u = numel(u);
N_x = numel(x);

u_idx = [1 : numel(u)];
x_idx = N_u + [1 : numel(x)];

q0_pp_idx = u_idx(1:n_red);
x1_idx = x_idx(1+2*n_red : 4*n_red);
q1_pp_idx = u_idx(1+n_red : 2*n_red);
u_opt_indices = [q0_pp_idx, x1_idx, q1_pp_idx];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET INPUT Parameter 4/5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_k    = SX.sym( 'x_k', 2*n_red, 1       ); % current x state
y_d    = SX.sym( 'y_d', m+1,     N_MPC+1 ); % (y_d_0 ... y_d_N)
x_prev = SX.sym( 'x_prev', 2*n_red, N_MPC+1 );
u_prev = SX.sym( 'u_prev', size(u) );

mpc_parameter_inputs = {x_k, y_d, x_prev, u_prev};
mpc_init_reference_values = [x_0_0(:); y_d_0(:); x_init_guess_0(:); u_init_guess_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x = cell(1, N_MPC); % for F
g_u_prev = cell(1, N_MPC); % for u_prev

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
        
        % q_prev = x_prev(1:n_red, 1 + (i));
        %g_vec(:, 1 + (i)) = g_fun_red(q);
        % g_vec(:, 1 + (i)) = g_fun_red(q_prev);
        g_vec(:, 1 + (i)) = g_fun_red(q);
        g_u_prev(1, 1 + (i)) = {u(:, 1 + (i)) - u_prev(:, 1 + (i))};
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Total number of equation conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% jump in tau at max 1000Nm/s
max_du = pp.max_du;
max_du_arr = repmat(max_du*dt_int_arr, n_red, 1);

lbg(1+end-N_u:end, 1) = -max_du_arr(:);
ubg(1+end-N_u:end, 1) =  max_du_arr(:);
g = [g_x, g_u_prev];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Define Cost Function  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Q_norm_square = @(z, Q) dot( z, mtimes(diag(Q), z));

J_yt = 0;
J_yt_N = 0;
if ~isempty(yt_indices)
    J_yt   = J_yt + Q_norm_square( y(yt_indices, 1 + (1:N_MPC-1) ) - y_d(yt_indices, 1 + (1:N_MPC-1)), pp.Q_y(   yt_indices) );
    J_yt_N =        Q_norm_square( y(yt_indices, 1 + (  N_MPC  ) ) - y_d(yt_indices, 1 + (  N_MPC  )), pp.Q_yN(  yt_indices) );
end

if isempty(yr_indices)
    J_yr = 0;
    J_yr_N = 0;
else
    J_yr = 0;
    for i=1:N_MPC
        % Fehlerhaft bei sehr großen fehlern:
        % R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))';
        % q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)];  % am genauesten
        
        % R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))' - quat2rotm_v2(y_d(4:7, 1 + (i))) * R_e_arr{1 + (i)}';
        % q_y_yr_err = [1; R_y_yr(3,2); R_y_yr(1,3); R_y_yr(2,1)];
        
        % q_y_yr_err = rotm2quat_v4_casadi(R_y_yr);
        
        q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(y_d(4:7, 1 + (i))));
        
        % quat_e = y(4:7, 1 + (i));
        % quat_d = y_d(4:7, 1 + (i));
        
        % vec_e = quat_e(1)*quat_d(2:4) - quat_d(1)*quat_e(2:4) - cross(quat_d(2:4), quat_e(2:4));
        % q_y_yr_err = [1; vec_e];
        
        % q_y_yr_err = 1/2*simplify(quat_mult(y(4:7, 1 + (i)), quat_inv(y_d(4:7, 1 + (i)))) - quat_mult(y_d(4:7, 1 + (i)), quat_inv(y(4:7, 1 + (i)))));
        
        if(i < N_MPC)
            J_yr = J_yr + Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_y(3+yr_indices)  );
        else
            J_yr_N = Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_yN(3+yr_indices)  );
        end
    end
end

u_err = u-g_vec; % es ist stabiler es nicht gegenüber der vorherigen Lösung zu gewichten!
%u_err = u;
%u_err = q_pp;

% q_err = x(1:n_red, :) - x_prev(1:n_red, :);
% q_err_p = x(n_red+1:2*n_red, :);% - x_prev(n_red+1:2*n_red, :);
% x_err = [q_err; q_err_p];

J_x_prev = Q_norm_square(x - x_prev, pp.R_x_prev(n_x_indices));
J_q_ref = Q_norm_square(q - pp.q_ref(n_indices), pp.R_q_ref(n_indices));
J_q_p  = Q_norm_square(q_p, pp.R_q_p(n_indices));
J_u  = Q_norm_square(u_err, pp.R_u(n_indices));
% J_u = Q_norm_square(q_pp, pp.R_q_pp(n_indices, n_indices)) + Q_norm_square(q_p, pp.R_q_pp(n_indices, n_indices));

% it is really important to only weight the first control input!
J_u0_prev = Q_norm_square(u(:, 1) - u_prev(:, 1), pp.R_u0_prev(n_indices));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Define Additional Outputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cost_vars_names = '{J_yt, J_yr, J_yt_N, J_yr_N, J_u, J_q_ref, J_q_p, J_x_prev, J_u0_prev}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);