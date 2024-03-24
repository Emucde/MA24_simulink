import casadi.*

%%
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
y_ref_0            = param_trajectory.p_d(    1:2, 1 + N_step_MPC : N_step_MPC : 1 + (N_MPC  ) * N_step_MPC ); % (y_1 ... y_N)
y_p_ref_0          = param_trajectory.p_d_p(  1:2, 1 + N_step_MPC : N_step_MPC : 1 + (N_MPC  ) * N_step_MPC ); % (y_p_1 ... y_p_N)
y_pp_ref_0         = param_trajectory.p_d_pp( 1:2, 1              : N_step_MPC : 1 + (N_MPC-1) * N_step_MPC ); % (y_pp_0 ... y_N-1)

% get weights from "init_MPC_weight.m"
% TODO: DELETE

param_weight_init = param_weight.(casadi_func_name);

% unnedig?
QQ_y    = param_weight_init.Q_y;
QQ_y_p  = param_weight_init.Q_y_p;
QQ_y_pp = param_weight_init.Q_y_pp;
QQ_yN   = param_weight_init.Q_yN;
xx_min  = param_weight_init.x_min;
xx_max  = param_weight_init.x_max;
uu_min  = param_weight_init.u_min;
uu_max  = param_weight_init.u_max;

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    pp = convert_doublestruct_to_casadi(param_weight_init);
else % hardcoded weights
    pp = param_weight_init;
end

%% Start with an empty NLP

% Optimization Variables:
x = SX.sym( 'x', 2*n, N_MPC+1 );
u = SX.sym( 'u',   n, N_MPC   );

% input parameter
x_k      = SX.sym( 'x_k',      2*n, 1       );
y_ref    = SX.sym( 'y_ref',    2,   N_MPC   ); % with y_N
y_p_ref  = SX.sym( 'y_p_ref',  2,   N_MPC   );
y_pp_ref = SX.sym( 'y_pp_ref', 2,   N_MPC   );

%% set input parameter cellaray p
p = [x_k; y_ref(:); y_p_ref(:); y_pp_ref(:)];
if(weights_and_limits_as_parameter) % debug input parameter
    p = [p; struct_to_row_vector(pp)'];
end

% optimization variables cellarray w
w = [x(:); u(:)];
lbw = [repmat(pp.x_min, N_MPC + 1, 1); repmat(pp.u_min, N_MPC, 1)];
ubw = [repmat(pp.x_max, N_MPC + 1, 1); repmat(pp.u_max, N_MPC, 1)];

% constraints conditions cellarray g
g = cell(1, N_MPC+1);
lbg = zeros(numel(x), 1);
ubg = zeros(numel(x), 1);

y    = SX( 2, N_MPC ); % TCP position:     (y_1 ... y_N)
y_p  = SX( 2, N_MPC ); % TCP velocity:     (y_p_1 ... y_p_N)
y_pp = SX( 2, N_MPC ); % TCP acceleration: (y_pp_0 ... y_N-1)

% Caclulate state trajectory: Given: x_0: (x_1 ... xN)
g(1, 1 + (0)) = {x_k - x(:, 1 + (0))}; % x0 = xk
for i=0:N_MPC-1
    % Set the state dynamics constraints
    g(1, 1 + (i+1)) = {F(x(:, 1 + (i)), u(:, 1 + (i))) - x(:, 1 + (i+1))}; % system equation
end

% calculate trajectory values (y_1 ... y_N)
for i=1:N_MPC
     q    =  x(1:n, 1 + (i));
     H_e = hom_transform_endeffector_casadi_SX(q, param_robot);
     y(:, 1 + (i-1)) = H_e(1:2, 4); % index 0 is y_1
end

% calculate trajectory values (y_p_1 ... y_p_N)
for i=1:N_MPC
    q    =  x(1:n    , 1 + (i));
    q_p  =  x(n+1:2*n, 1 + (i));

    J = geo_jacobian_endeffector_casadi_SX(q, param_robot);
    y_p(:, 0 + (i)) = J(1:2, 1:2)*q_p;
    % or y(:, 0) = {H_e(x_k) or (y(:, i+1) - y(:, i))/param_global.Ta} for i=1
    % and y_p(:, i) = (y(:, i-1) - y(:, i))/param_global.Ta for i>1 and
end

% calculate trajectory values (y_pp_0 ... y_N-1)
for i=0:N_MPC-1
    % calculate joint angles and velocities
    q    =  x(1:n, 1 + (i));
    %q_p  =  x(n+1:2*n, 1 + (i));

    dx   = f(x(:, 1 + (i)), u(:, 1 + (i))); % = [d/dt q, d^2/dt^2 q], Alternativ: Differenzenquotient
    q_p  =  dx(1:n    , 1);
    q_pp =  dx(n+1:2*n, 1);

    J = geo_jacobian_endeffector_casadi_SX(q, param_robot);
    J_p = geo_jacobian_endeffector_p_casadi_SX(q, q_p, param_robot);
    y_pp(:, 1 + (i-1)) = J(1:2, 1:2)*q_pp + J_p(1:2, 1:2)*q_p;
    % or y_pp(:, i) = (y(:, i+2) - 2*y(:, i+1) + y(:, i))/param_global.Ta^2 for i=1
    % and y_pp(:, i) = (y(:, i-2) - 2*y(:, i-1) + y(:, i))/param_global.Ta^2 for i>1
end

Q_norm_square = @(z, Q) dot( z, mtimes(Q, z));

% Achtung: % index 0 von y und y_p is y_1 bzw. y1_p, Index 0 von y_pp ist y0_pp
D_0    = Q_norm_square( y_pp( :, 1 + (0)         ) - y_pp_ref( :, 1 + (0)        ), pp.Q_y0_pp );

D_1    = Q_norm_square( y(    :, 0 + (1)         ) - y_ref(    :, 0 + (1)        ), pp.Q_y1    ) + ...
         Q_norm_square( y_p(  :, 0 + (1)         ) - y_p_ref(  :, 0 + (1)        ), pp.Q_y1_p  ) + ...
         Q_norm_square( y_pp( :, 1 + (1)         ) - y_pp_ref( :, 1 + (1)        ), pp.Q_y1_pp );

D_N    = Q_norm_square( y(    :, 0 + (N_MPC)     ) - y_ref(    :, 0 + (N_MPC)    ), pp.Q_yN    ) + ...
         Q_norm_square( y_p(  :, 0 + (N_MPC)     ) - y_p_ref(  :, 0 + (N_MPC)    ), pp.Q_yN_p  );

J_y    = Q_norm_square( y(    :, 0 + (2:N_MPC-1) ) - y_ref(    :, 0 + (2:N_MPC-1)), pp.Q_y     );
J_y_p  = Q_norm_square( y_p(  :, 0 + (2:N_MPC-1) ) - y_p_ref(  :, 0 + (2:N_MPC-1)), pp.Q_y_p   );
J_y_pp = Q_norm_square( y_pp( :, 1 + (2:N_MPC-1) ) - y_pp_ref( :, 1 + (2:N_MPC-1)), pp.Q_y_pp  );

cost_vars_names = '{J_y, J_y_p, J_y_pp, D_0, D_1, D_N}';
cost_vars_SX = eval(cost_vars_names);
cost_vars_names_cell = regexp(cost_vars_names, '\w+', 'match');

% calculate cost function
J = sum([cost_vars_SX{:}]);

% Redundant: Z184, Z191, Z195, Z212

input_vars_SX  = {y_ref, y_p_ref, y_pp_ref, x_k, x, u};
output_values_SX = {J, vertcat(w{:}), vertcat(g{:}), vertcat(p{:})};

if(weights_and_limits_as_parameter)
    %input_vars_SX  = {input_vars_SX{:},  Q_y, Q_y_p, Q_y_pp, Q_yN, x_min, x_max, u_min, u_max};
    input_vars_SX = horzcat(input_vars_SX, struct2cell(pp)');
    get_costs_MX = Function('get_costs_MX', input_vars_SX, cost_vars_SX);
    get_limits_MX = Function('get_limits_MX', input_vars_SX, {lbw,   ubw,   lbg,   ubg});
end

get_outputs_MX = Function('get_outputs_MX', input_vars_SX, output_values_SX);


% get SX prop struct (SX is much faster than MX!)
[J, W, G, P] = get_outputs_MX(input_vars_SX{:});
prob = struct('f', J, 'x', W, 'g', G, 'p', P);

% convert SX variables to MX (nlpsolve can only use MX variables)
input_vars_MX       = cellfun(@(x) sx_to_mx(x, 'get_MX_sym_cell' ), input_vars_SX, 'UniformOutput', false);
%input_vars_MX_names = cellfun(@(x) sx_to_mx(x, 'get_MX_name_cell'), input_vars_SX, 'UniformOutput', false);

%% Create an NLP solver

%|-------------------|
%| List of plugins   |
%|-------------------|
%| AmplInterface     |
%| blocksqp          |
%| bonmin            |
%| ipopt             |
%| knitro            |
%| snopt             |
%| worhp             |
%| feasiblesqpmethod |
%| qrsqp             |
%| scpgen            |
%| sqpmethod         |
%|-------------------|

if(strcmp(MPC_solver, 'qrqp'))
    % DOKU: https://casadi.sourceforge.net/v2.0.0/api/html/d6/d07/classcasadi_1_1NlpSolver.html
    opts = struct; % Create a new structure
    opts.qpsol = 'qrqp'; % Set the QP solver to 'qrqp'
    opts.qpsol_options.print_iter = false; % Disable printing of QP solver iterations
    opts.qpsol_options.print_header = false; % Disable printing of QP solver header
    opts.qpsol_options.print_info = false; % Disable printing of QP solver info
    opts.qpsol_options.error_on_fail = false;
    
    opts.print_header = false; % Disable printing of solver header
    opts.print_iteration = false; % Disable printing of solver iterations
    opts.print_time = false; % Disable printing of solver time
    opts.print_status = false;
    opts.error_on_fail = false;
    opts.hessian_approximation = 'exact';
    % opts.iteration_callback_ignore_errors = true;
    % opts.elastic_mode = true;
    % opts.regularity_check = false;
    opts.show_eval_warnings = false;
    % opts.max_iter = 10;
    % opts.tol_du=1e-3;
    % opts.tol_pr=1e-3;
    solver = nlpsol('solver', 'sqpmethod', prob, opts);
    % solver.print_options();
    
elseif(strcmp(MPC_solver, 'qpoases'))
    % DOKU: https://casadi.sourceforge.net/api/internal/d5/d43/classcasadi_1_1QpoasesInterface.html
    opts = struct; % Create a new structure
    opts.qpsol = 'qpoases'; % Set the QP solver to 'qrqp'
    opts.qpsol_options.error_on_fail = false;
    opts.qpsol_options.printLevel = 'none';
    opts.print_header = false; % Disable printing of solver header
    opts.print_iteration = false; % Disable printing of solver iterations
    opts.print_time = false; % Disable printing of solver time
    opts.print_status = false;
    opts.error_on_fail = false;
    opts.hessian_approximation = 'exact';
    % opts.iteration_callback_ignore_errors = true;
    % opts.elastic_mode = true;
    % opts.regularity_check = false;
    opts.show_eval_warnings = false;
    opts.verbose_init = false;
    % opts.max_iter = 10;
    % opts.tol_du=1e-3;
    % opts.tol_pr=1e-3;
    solver = nlpsol('solver', 'sqpmethod', prob, opts);
    % solver.print_options();
elseif(strcmp(MPC_solver, 'ipopt'))
    % DOKU: https://casadi.sourceforge.net/v2.0.0/api/html/d6/d07/classcasadi_1_1NlpSolver.html#plugin_NlpSolver_ipopt
    opts.show_eval_warnings = false;
    opts.error_on_fail = false;
    opts.print_time = 0;
    all_ipopt_options;
    solver = nlpsol('solver', 'ipopt', prob, opts);
    %solver.print_options();
else
    error(['invalid MPC solver=', MPC_solver, ...
           ' is a valid solver for nlpsol (only "qrqp", "ipopt", "qpoases", ... supported for nlpsol)']);
end

%|--------------------------------------------------------------------------------------|
%| Full name     | Short  | Description                                                 |
%|--------------------------------------------------------------------------------------|
%| NLPSOL_X0     | x0     | Decision variables, initial guess (nx x 1)                  |
%| NLPSOL_P      | p      | Value of fixed parameters (np x 1)                          |
%| NLPSOL_LBX    | lbx    | Decision variables lower bound (nx x 1), default -inf.      |
%| NLPSOL_UBX    | ubx    | Decision variables upper bound (nx x 1), default +inf.      |
%| NLPSOL_LBG    | lbg    | Constraints lower bound (ng x 1), default -inf.             |
%| NLPSOL_UBG    | ubg    | Constraints upper bound (ng x 1), default +inf.             |
%| NLPSOL_LAM_X0 | lam_x0 | Lagrange multipliers for bounds on X, initial guess (nx x 1)|
%| NLPSOL_LAM_G0 | lam_g0 | Lagrange multipliers for bounds on G, initial guess (ng x 1)|
%|--------------------------------------------------------------------------------------|

% [J_MX, w_MX, g_MX, p_MX,  J_y_MX, J_y_p_MX, J_y_pp_MX, D_0_MX, D_1_MX, D_N_MX, lbw_MX, ubw_MX, lbg_MX, ubg_MX] = get_outputs_MX(input_vars_MX{:}); % früher
[J_MX, w_MX, ~, p_MX] = get_outputs_MX(input_vars_MX{:});

if(weights_and_limits_as_parameter)
    [lbw_MX, ubw_MX, lbg_MX, ubg_MX] = get_limits_MX(input_vars_MX{:});

    sol_sym = solver('x0', w_MX, 'lbx', lbw_MX, 'ubx', ubw_MX,...
        'lbg', lbg_MX, 'ubg', ubg_MX, 'p', p_MX);
else
    sol_sym = solver('x0', w_MX, 'lbx', lbw, 'ubx', ubw,...
        'lbg', lbg, 'ubg', ubg, 'p', p_MX);
end

%--------------------------------------------------------------------------------------|
%| Full name    | Short | Description                                                  |
%--------------------------------------------------------------------------------------|
%| NLPSOL_X     | x     | Decision variables at the optimal solution (nx x 1)          |
%| NLPSOL_F     | f     | Cost function value at the optimal solution (1 x 1)          |
%| NLPSOL_G     | g     | Constraints function at the optimal solution (ng x 1)        |
%| NLPSOL_LAM_X | lam_x | Lagrange multipliers for bounds on X at the solution (nx x 1)|
%| NLPSOL_LAM_G | lam_g | Lagrange multipliers for bounds on G at the solution (ng x 1)|
%| NLPSOL_LAM_P | lam_p | Lagrange multipliers for bounds on P at the solution (np x 1)|
%--------------------------------------------------------------------------------------|

% generate solver solutions variables
u_opt      =        ( sol_sym.x(   numel(x)+(1:n) )                );
x_full_opt = reshape( sol_sym.x( 1:numel(x)       ) , 2*n, N_MPC+1 );
u_full_opt = reshape( sol_sym.x(   numel(x)+1:end ) ,   n, N_MPC   );

output_vars_MX = {u_opt, x_full_opt, u_full_opt};
if(weights_and_limits_as_parameter)
    [output_vars_MX_ext{1:length(cost_vars_SX)}] = get_costs_MX(input_vars_MX{:});
    output_vars_MX = {output_vars_MX{:}, output_vars_MX_ext{:}};
end

if(weights_and_limits_as_parameter)
    f_opt = Function(casadi_func_name, ...
        {input_vars_MX{:}},...
        {output_vars_MX{:}});

    %[u_opt_sol, x_full_opt_sol, u_full_opt_sol, J_y_sol, J_y_p_sol, J_y_pp_sol, D_N_sol] = ...
    %    f_opt(y_ref_0, y_p_ref_0, y_pp_ref_0, x_0_0, x_init_guess_0, u_init_guess_0, QQ_y, QQ_y_p, QQ_y_pp, QQ_yN, xx_min, xx_max, uu_min, uu_max);
    param_weight_init_cell = struct2cell(param_weight_init)';
    [u_opt_sol, x_full_opt_sol, u_full_opt_sol, cost_values_sol{1:length(cost_vars_SX)}] = f_opt(y_ref_0, y_p_ref_0, y_pp_ref_0, x_0_0, x_init_guess_0, u_init_guess_0, param_weight_init_cell{:});
else
    % ohne extra parameter 30-60 % schneller!
    f_opt = Function(casadi_func_name, ...
        {input_vars_MX{:}},...
        {output_vars_MX{:}});

    [u_opt_sol, x_full_opt_sol, u_full_opt_sol] = f_opt(y_ref_0, x_0_0, x_init_guess_0, u_init_guess_0);
end

% set init guess
x_init_guess = full(x_full_opt_sol);
u_init_guess = full(u_full_opt_sol);

if(print_init_guess_cost_functions && weights_and_limits_as_parameter)
    disp(['J = '      num2str(full( sum([ cost_values_sol{:} ]) )) ]);
    for i=1:length(cost_vars_names_cell)
        disp([cost_vars_names_cell{i}, '= ', num2str(full( cost_values_sol{i} ))])
    end
end

%% COMPILE (nlpsol)
if(compile_sfun)
    if(compile_mode == 1)
        tic;
        s_fun_name = 's_function_nlpsol.c';
        compile_casadi_sfunction(f_opt, s_fun_name, output_dir, MPC_solver, '', compile_mode); % default nlpsol s-function
    disp(['Compile time for casadi s-function (nlpsol): ', num2str(toc), ' s']);
    elseif(compile_mode == 2)
        tic;
        s_fun_name = 's_function_opti.c';
        compile_casadi_sfunction(f_opt, s_fun_name, output_dir, MPC_solver, '-O2', compile_mode); % default nlpsol s-function
        disp(['Compile time for casadi s-function (opti for nlpsol): ', num2str(toc), ' s']);
    end
end

%% s-function in Simulink: Name of s-function: MPC1
% 1) Insert S-Function Block in Matlab
% 2) add 
%        's_functions/MPC1.casadi', 'MPC1' 
%    to S-function parameters
% 2) add 's_function_MPC1' to S-function name
% 3) set S-function modules to ''