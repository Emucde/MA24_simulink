% MPC v4: Optimization problem

import casadi.*

% get weights from "init_MPC_weight.m"
param_weight_init = param_weight.(casadi_func_name);

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init); % Matrizen sind keine Diagonlmatrizen [TODO]
else % hardcoded weights
    pp = param_weight_init;
end

parametric_mode = struct;
parametric_mode.polynomial = 1;

parametric_type = parametric_mode.polynomial;

yt_indices = param_robot.yt_indices;
yr_indices = param_robot.yr_indices;

n = param_robot.n_DOF; % Dimension of joint space
n_red = param_robot.n_red; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

n_x_indices = [n_indices n_indices+n];

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);
quat_R_endeffector_py_fun = Function.load([input_dir, 'quat_R_endeffector_py.casadi']);

if(parametric_type == parametric_mode.polynomial)
    t0 = SX.sym('t0');
    theta0 = SX.sym('theta', n_red, 3);
    x0 = SX.sym('x0', 2*n_red, 1);
    q0 = x0(1:n_red);
    q1 = x0(n_red+1:end);
    q = Function('q_scalar', {x0, t0, theta0}, {q0 + q1*t0 + 1/2*theta0(:, 1)*t0^2 + 1/6*theta0(:, 2)*t0^3 + 1/12*theta0(:, 3)*t0^4});
    q_p = Function('q_scalar_p', {x0, t0, theta0}, {q1 + theta0(:, 1)*t0 + 1/2*theta0(:, 2)*t0^2 + 1/3*theta0(:, 3)*t0^3});
    q_pp = Function('q_scalar_pp', {t0, theta0}, {theta0(:, 1) + theta0(:, 2)*t0 + theta0(:, 3)*t0^2});
else
    error('parametric_type not supported');
end

q_red = SX.sym( 'q',     n_red, 1 );
x_red = SX.sym( 'x',   2*n_red, 1 );
u_red = SX.sym( 'u',     n_red, 1 );

q_subs            = SX(q_0);
q_subs(n_indices) = q_red;

H_red = Function('H_red', {q_red}, {cse(hom_transform_endeffector_py_fun(q_subs))});
quat_fun_red = Function('quat_fun_red', {q_red}, {cse(quat_endeffector_py_fun(q_subs))});
f_red = Function('f_red', {x_red, u_red}, {[x_red(n_red+1:2*n_red); u_red]});

x = SX.sym( 'x',   2*n, 1 );
u = SX.sym( 'u',     n, 1 );
f = Function('f', {x, u}, {[x(n+1:2*n); u]});

% Discrete system dynamics
M = rk_iter; % RK4 steps per interval
N_step = pp.N_step;

DT_ctl = param_global.Ta/M;
DT = N_step * DT_ctl; % = Ts_MPC
DT2 = if_else(N_step > 1, DT - DT_ctl, DT_ctl); % = (N_step_MPC - 1) * DT_ctl = (N_MPC-1) * Ta/M = Ts_MPC - Ta

F_kp1 = integrate_casadi(f_red, DT_ctl, M, int_method); % runs with Ta from sensors
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

% Robot System: Initial guess
q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m

u_k_0  = q_0_red_pp;

u_init_guess_0 = ones(1, N_MPC).*u_k_0;
x_init_guess_0 = ones(1, N_MPC+1).*x_0_0;
theta_init_guess_0 = zeros(n_red, 3);

lam_x_init_guess_0 = zeros(numel(u_init_guess_0) + numel(x_init_guess_0) + numel(theta_init_guess_0), 1);
lam_g_init_guess_0 = zeros(2*numel(u_init_guess_0) + numel(x_init_guess_0), 1);

init_guess_0 = [u_init_guess_0(:); x_init_guess_0(:); theta_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];

if(any(isnan(full(init_guess_0))))
    error('init_guess_0 contains NaN values!');
end

% Optimization Variables:
u     = SX.sym( 'u',    n_red,   N_MPC   );
x     = SX.sym( 'x',    2*n_red, N_MPC+1 );
theta = SX.sym( 'theta', n_red, 3);

% need opt vars only for the joint and velocity limits are maintained
mpc_opt_var_inputs = {u, x, theta};

N_u = numel(u);
N_x = numel(x);
N_theta = numel(theta);

x_idx = N_u + [ 1 : N_x];
x1_idx = x_idx(1+2*n_red: 4*n_red);

theta_idx = N_u + N_x + [1 : N_theta];

q0_pp_idx = [1 : n_red];
q1_pp_idx = [1+n_red : 2*n_red];
u_opt_indices = [q0_pp_idx, x1_idx, q1_pp_idx];

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
% lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); -inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1)];
% ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1);  inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1)];

% Interessant: Wenn u auch im Horizont beschränkt wird, erhält man eine bleibende Regelabweichung (1e-5).
% Hingegen wenn die u im Horizont nicht beschränkt werden, sondern nur das, dass ausgegeben wird, hat man das Problem nicht:
ubw = [repmat(pp.u_max(n_indices), 1, 1);  inf(n_red*(N_MPC-1),1);  inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1); inf(numel(theta),1)];
lbw = [repmat(pp.u_min(n_indices), 1, 1); -inf(n_red*(N_MPC-1),1); -inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1); -inf(numel(theta),1)];

% input parameter
x_k  = SX.sym( 'x_k',  2*n_red,       1 ); % current x state = initial x state
y_d  = SX.sym( 'y_d',  m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d
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
g_x  = cell(1, N_MPC+1);
g_u  = cell(1, N_MPC);
g_u_prev = cell(1, N_MPC);

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
y    = SX( 7, N_MPC+1 ); % TCP pose:      (y_0 ... y_N)
R_e_arr = cell(1, N_MPC+1); % TCP orientation:   (R_0 ... R_N)

for i=0:N_MPC
    % calculate trajectory values (y_0 ... y_N)
    t_k = time_points(1 + (i));
    q_i = q(x_k, t_k, theta);
    q_p_i = q_p(x_k, t_k, theta);
    q_pp_i = q_pp(t_k, theta);
    
    H_e = H_red(x(1:n_red, 1 + (i)));
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(x(1:n_red, 1 + (i)));
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);
    
    x_k_i = [q_i; q_p_i];

    g_x(1, 1 + (i)) = {x(:, 1 + (i)) - x_k_i};

    if(i < N_MPC)
        % Caclulate state trajectory: Given: x_0: (x_1 ... xN)
        % if(i == 0)
        %     g_x(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk
        % end

        % if(i == 0 || i == 1)
        %     g_x(1, 1 + (i+1))  = { F_kp1(  x_k_i, q_pp_i ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk = x(t0) = tilde x0 to xk+1 = x(t0+Ta)
        % elseif(i == 2)
        %     g_x(1, 1 + (i+1))  = { F2(  x_k_i, q_pp_i ) - x( :, 1 + (i+1)) }; % Set the state constraints for xk+1 = x(t0+Ta) to x(t0+Ts_MPC) = tilde x1
        % else
        %     g_x(1, 1 + (i+1))  = { F(  x_k_i, q_pp_i ) - x( :, 1 + (i+1)) }; % Set the state constraints for x(t0+Ts_MPC*i) to x(t0+Ts_MPC*(i+1))
        %     % runs only to T_horizon-Ts_MPC, i. e. tilde x_{N-1} = x(t0+Ts_MPC*(N-1)) and x_N doesn't exist
        %     % Trajectory must be y(t0), y(t0+Ta), Y(t0+Ts_MPC), ..., y(t0+Ts_MPC*(N-1))
        % end
        g_u(1, 1 + (i))   = {u(:, 1 + (i)) - q_pp_i};
        g_u_prev(1, 1 + (i)) = {u(:, 1 + (i)) - u_prev(:, 1 + (i))};
    end
end

g = [g_x, g_u, g_u_prev];

% jump in tau at max 100rad/s^2
max_du = pp.max_du;
max_du_arr = repmat(max_du*dt_int_arr, n_red, 1);

lbg(1+end-N_u:end, 1) = -max_du_arr(:);
ubg(1+end-N_u:end, 1) =  max_du_arr(:);

% Calculate Cost Functions and set equation constraints
Q_norm_square = @(z, Q) dot( z, mtimes(diag(Q), z));

if isempty(yt_indices)
    J_yt = 0;
    J_yt_N = 0;
else
    J_yt   = Q_norm_square( y(yt_indices, 1 + (1:N_MPC-1) ) - y_d(yt_indices, 1 + (1:N_MPC-1)), pp.Q_y( yt_indices) );
    J_yt_N = Q_norm_square( y(yt_indices, 1 + (  N_MPC  ) ) - y_d(yt_indices, 1 + (  N_MPC  )), pp.Q_yN(yt_indices) );
end

if isempty(yr_indices)
    J_yr = 0;
    J_yr_N = 0;
else
    J_yr = 0;
    for i=1:N_MPC
        % R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))';
        % q_y_yr_err = rotm2quat_v4_casadi(R_y_yr);
        %q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)]; %ungenau aber schneller (flipping?)
        
        q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(y_d(4:7, 1 + (i))));
        
        if(i < N_MPC)
            J_yr = J_yr + Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_y(3+yr_indices)  );
        else
            J_yr_N = Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_yN(3+yr_indices)  );
        end
    end
end

J_x_prev = Q_norm_square(x - x_prev, pp.R_x_prev(n_x_indices)); %Q_norm_square(u, pp.R_u);

J_q_ref = Q_norm_square(x(1:n_red, :) - pp.q_ref(n_indices), pp.R_q_ref(n_indices));

J_theta = Q_norm_square(theta', pp.R_theta);

J_q_p = Q_norm_square(x(n_red+1:end, :), pp.R_q_p(n_indices)); %Q_norm_square(u, pp.R_u);
J_u = Q_norm_square(u, pp.R_u(n_indices)); %Q_norm_square(u, pp.R_u);

% it is really important to only weight the first control input!
J_u0_prev = Q_norm_square(u(:, 1) - u_prev(:, 1), pp.R_u0_prev(n_indices));

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_theta, J_q_ref, J_q_p, J_u, J_x_prev, J_u0_prev}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);