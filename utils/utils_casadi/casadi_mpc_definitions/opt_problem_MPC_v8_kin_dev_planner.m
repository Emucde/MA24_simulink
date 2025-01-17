% MPC v4: Optimization problem 

import casadi.*

diff_variant_mode = struct;
diff_variant_mode.numdiff = 1; % default forward, central, backward deviation

diff_variant = diff_variant_mode.numdiff;

yt_indices = param_robot.yt_indices;
yr_indices = param_robot.yr_indices;

n = param_robot.n_DOF; % Dimension of joint space
n_red = param_robot.n_red; % Dimension of joint space
m = param_robot.m; % Dimension of Task Space

n_x_indices = [n_indices n_indices+n];

hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);

q_red = SX.sym( 'q',     n_red, 1 );
x_red = SX.sym( 'x',   2*n_red, 1 );
u_red = SX.sym( 'u',     n_red, 1 );

q_subs            = SX(q_0);
q_subs(n_indices) = q_red;

H_red = Function('H_red', {q_red}, {hom_transform_endeffector_py_fun(q_subs)});
quat_fun_red = Function('quat_fun_red', {q_red}, {quat_endeffector_py_fun(q_subs)});

M = 1; % RK4 steps per interval
DT_ctl = param_global.Ta/M;
if(N_step_MPC <= 3)
    DT = DT_ctl; % special case if Ts_MPC = Ta
else
    DT = N_step_MPC * DT_ctl; % = Ts_MPC
end

% initial guess for reference trajectory parameter
if(N_step_MPC <= 3)
    MPC_traj_indices = 1:(N_MPC+1);
else
    MPC_traj_indices = [0, 1, 2, (1:1+(N_MPC-3))*N_step_MPC]+1;
end

dt_int_arr = (MPC_traj_indices(2:end) - MPC_traj_indices(1:end-1))*DT_ctl;

p_d_0 = param_trajectory.p_d( 1:3, MPC_traj_indices ); % (y_0 ... y_N)
q_d_0 = param_trajectory.q_d( 1:4, MPC_traj_indices ); % (q_0 ... q_N)
y_d_0 = [p_d_0; q_d_0];

%% Calculate Initial Guess

% Robot System: Initial guess
q_0_red    = q_0(n_indices);
q_0_red_p  = q_0_p(n_indices);
q_0_red_pp = q_0_pp(n_indices);
x_0_0  = [q_0_red; q_0_red_p];%q1, .. qn, d/dt q1 ... d/dt qn, defined in parameters_xdof.m

u_k_0  = q_0_red_pp;

u_init_guess_0 = ones(n_red, N_MPC).*u_k_0;
x_init_guess_0 = ones(2*n_red, N_MPC+1).*x_0_0;

lam_x_init_guess_0 = zeros(numel(u_init_guess_0) + numel(x_init_guess_0), 1);
lam_g_init_guess_0 = zeros(2*numel(u_init_guess_0) + numel(x_init_guess_0(1+n_red:2*n_red, :)) + numel(x_0_0), 1);
% eig brauch ich q0, q0p, q0pp, q1, q1p = 5n Gleichungsbedingungen.

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
u     = SX.sym( 'u',    n_red,   N_MPC   );
x     = SX.sym( 'x',    2*n_red, N_MPC+1 );

% confusing, but I need equation constraints to relate x to q and qp = Sv q.
% only then the joint and velocity limits are maintained in!
mpc_opt_var_inputs = {u, x};

% u = [u0; u1; ... uN-1], x = [x0; x1; ... xN] = [q_0;q_0_p;q_1;q_1_p;...;q_N;q_N_p]
% xx = [u; x] = [u0; u1; ... uN-1; x0; x1; ... xN]
%   u0 = q_0_pp = u(   1 :   n_red) | q_0 = x(     1     : n_red      ) | q_0_p = x(     1+  n_red :     2*n_red)
%   u0 = q_0_pp = xx(  1 :   n_red) | q_0 = xx(N_u+1     : n_red+N_u  ) | q_0_p = xx(N_u+1+  n_red : N_u+2*n_red)
%   u1 = q_1_pp = u( n_red+1 : 2*n_red) | q_1 = x(     1+2*n_red :     3*n_red) | q_1_p = x(     1+3*n_red :     4*n_red)
%   u1 = q_1_pp = xx(n_red+1 : 2*n_red) | q_1 = xx(N_u+1+2*n_red : N_u+3*n_red) | q_1_p = xx(N_u+1+3*n_red : N_u+4*n_red)
N_u = numel(u);
q0_pp_idx = [N_u+1         : N_u+  n_red, N_u+1+  n_red:N_u+2*n_red, 1       :   n_red]; % [q_0, q_p_0, q_pp_0] % current desired state for CT Control
q1_pp_idx = [N_u+1+2*n_red : N_u+3*n_red, N_u+1+3*n_red:N_u+4*n_red, 1+n_red : 2*n_red]; % [q_1, q_p_1] % next init state for MPC
u_opt_indices = [q0_pp_idx, q1_pp_idx];

% optimization variables cellarray w
w = merge_cell_arrays(mpc_opt_var_inputs, 'vector')';
lbw = [repmat(pp.u_min(n_indices), size(u, 2), 1); -inf(2*n_red,1); repmat(pp.x_min(n_x_indices), size(x(:,2:end), 2), 1)];
ubw = [repmat(pp.u_max(n_indices), size(u, 2), 1);  inf(2*n_red,1); repmat(pp.x_max(n_x_indices), size(x(:,2:end), 2), 1)];

% input parameter
x_k  = SX.sym( 'x_k',  2*n_red, 1 ); % current x state = initial x state
y_d  = SX.sym( 'y_d',  m+1, N_MPC+1 ); % (y_d_0 ... y_d_N), p_d, q_d
x_prev = SX.sym( 'x_prev',  2*n_red, N_MPC+1 );
u_prev = SX.sym( 'u_prev', size(u) );

mpc_parameter_inputs = {x_k, y_d, x_prev, u_prev};
mpc_init_reference_values = [x_0_0(:); y_d_0(:); x_init_guess_0(:); u_init_guess_0(:)];

%% set input parameter cellaray p
p = merge_cell_arrays(mpc_parameter_inputs, 'vector')';
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% constraints conditions cellarray g
g_x  = cell(1, N_MPC+2);
g_u  = cell(1, N_MPC);

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

if(diff_variant == diff_variant_mode.numdiff)
    S_v = create_numdiff_matrix(DT_ctl, n_red, N_MPC+1, 'fwdbwdcentralthreetimes', DT);
    S_a = S_v^2;
    disp('Selected diff method: forward central backward, non-equidistant samples');
else
    error('invalid mode');
end
%      x = [q(t0),   q(t1), ...  q(tN),  dq(t0),  dq(t1), ...  dq(tN)] = [qq;   qq_p ]
% d/dt x = [dq(t0), dq(t1), ... dq(tN), ddq(t0), ddq(t1), ... ddq(tN)] = [qq_p; qq_pp]
qq = reshape(x(1:n_red, :), n_red*(N_MPC+1), 1);
qq_p  = S_v * qq;
qq_pp = S_a * qq;

q = reshape(qq, n_red, N_MPC+1);
q_p = reshape(qq_p, n_red, N_MPC+1);
q_pp = reshape(qq_pp, n_red, N_MPC+1);

g_x(1, 1 + (0)) = {x_k - [q(:, 1 + (0)); q_p(:, 1 + (0))]}; % x0 = xk

for i=0:N_MPC
    q_i = q(:, 1 + (i));
    q_p_i = q_p(:, 1 + (i));

    % calculate trajectory values (y_0 ... y_N)
    H_e = H_red(q_i);
    R_e = H_e(1:3, 1:3);
    y(1:3,   1 + (i)) = H_e(1:3, 4);
    y(4:7,   1 + (i)) = quat_fun_red(q_i);
    R_e_arr{1 + (i)} = H_e(1:3, 1:3);
    g_x(1, 1 + (i+1)) = {x(1+n_red:2*n_red, 1 + (i)) - q_p_i};

    if(i < N_MPC)
        g_u(1, 1 + (i))   = {u(:, 1 + (i)) - q_pp(:, 1 + (i))};
        g_u_prev(1, 1 + (i)) = {u(:, 1 + (i)) - u_prev(:, 1 + (i))};
    end
end

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
        %R_y_yr = R_e_arr{1 + (i)} * quat2rotm_v2(y_d(4:7, 1 + (i)))';
        % q_y_yr_err = rotm2quat_v4_casadi(R_y_yr);
        %q_y_yr_err = [1; R_y_yr(3,2) - R_y_yr(2,3); R_y_yr(1,3) - R_y_yr(3,1); R_y_yr(2,1) - R_y_yr(1,2)]; %ungenau aber schneller (flipping?)
        
        q_y_yr_err = quat_mult(y(4:7, 1 + (i)), quat_inv(y_d(4:7, 1 + (i))));
        
        if(i < N_MPC)
            J_yr = J_yr + Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_y(3+yr_indices,3+yr_indices)  );
        else
            J_yr_N = Q_norm_square( q_y_yr_err(1+yr_indices) , pp.Q_yN(3+yr_indices,3+yr_indices)  );
        end
    end
end

% avoid jump in q_pp
max_du = pp.max_du;
max_du_arr = repmat(max_du*dt_int_arr, n_red, 1);

lbg(1+end-N_u:end, 1) = -max_du_arr(:);
ubg(1+end-N_u:end, 1) =  max_du_arr(:);

g = [g_x, g_u, g_u_prev];

J_q_ref = Q_norm_square(x(1:n_red, :) - pp.q_ref(n_indices), pp.R_q_ref(n_indices, n_indices));

J_q_p = Q_norm_square(q_p, pp.R_q_p(n_indices, n_indices)); %Q_norm_square(u, pp.R_u);
J_u = Q_norm_square(u, pp.R_u(n_indices, n_indices)); %Q_norm_square(u, pp.R_u);

J_x_prev = Q_norm_square(x - x_prev, pp.R_x_prev(n_x_indices, n_x_indices)); %Q_norm_square(u, pp.R_u);

% it is really important to only weight the first control input!
J_u0_prev = Q_norm_square(u(:, 1) - u_prev(:, 1), pp.R_u0_prev(n_indices, n_indices));

cost_vars_names = '{J_yt, J_yt_N, J_yr, J_yr_N, J_q_ref, J_q_p, J_u, J_x_prev, J_u0_prev}';

cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);