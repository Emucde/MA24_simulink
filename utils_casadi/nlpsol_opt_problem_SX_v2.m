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
x_0_0 = [q_0; 0;0];%q1, q2, d/dt q1, d/dt q2
q = x_0_0(1:n);
dq = x_0_0(n+1:2*n);
ddq = [0;0];
xe_k_0 = xe0(1:2); % x pos, y pos
u_k_0 = compute_tau(q, dq, ddq, param_robot); % tau1, tau2

u_init_guess_0 = ones(2,N_MPC).*u_k_0;

% für die S-funktion ist der Initial Guess wesentlich!
sim                  = F.mapaccum(N_MPC);
x_init_guess_kp1_res = sim(x_0_0, u_init_guess_0);
x_init_guess_0       = [x_0_0 full(x_init_guess_kp1_res)];
y_ref_0              = param_trajectory.p_d(1:2, 1+N_step_MPC : N_step_MPC : 1+N_MPC*N_step_MPC);

% get weights from "init_MPC_weight.m"
QQ     = eval("param_weight_"+casadi_func_name+".QQ;");
QQ_yN  = eval("param_weight_"+casadi_func_name+".QQ_yN;");
RR_u   = eval("param_weight_"+casadi_func_name+".RR_u;");
xx_min = eval("param_weight_"+casadi_func_name+".xx_min;");
xx_max = eval("param_weight_"+casadi_func_name+".xx_max;");
uu_min = eval("param_weight_"+casadi_func_name+".uu_min;");
uu_max = eval("param_weight_"+casadi_func_name+".uu_max;");

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    Q     = SX.sym('Q', 2,2); % x y weight
    Q_yN  = SX.sym('Q_yN', 2,2);
    R_u   = SX.sym('R_u', 2,2); % u absolut
    x_min = SX.sym('x_min', 4,1);
    x_max = SX.sym('x_max', 4,1);
    u_min = SX.sym('u_min', 2,1);
    u_max = SX.sym('u_max', 2,1);
else % hardcoded weights
    Q           = QQ          ;
    R_u         = RR_u        ;
    x_min       = xx_min      ;
    x_max       = xx_max      ;
    u_min       = uu_min      ;
    u_max       = uu_max      ;
    Q_yN        = QQ_yN  ;
end

%% Start with an empty NLP

% Optimization Variables:
x = SX.sym('x', 2*n, N_MPC+1 );
u = SX.sym('u', n,   N_MPC   );

% parameter
x_k       = SX.sym('x_k',    2*n, 1      );
y_kp1_ref = SX.sym('ys_ref', 2,   N_MPC  );

% optimization variables cellarray w
w = [x(:); u(:)];
lbw = [repmat(x_min, N_MPC + 1, 1); repmat(u_min, N_MPC, 1)];
ubw = [repmat(x_max, N_MPC + 1, 1); repmat(u_max, N_MPC, 1)];

% constraints conditions cellarray g
g = cell(1, N_MPC+1);

lbg = zeros(numel(x), 1);
ubg = zeros(numel(x), 1);

y  = SX(2,N_MPC); % forward kinematics
%todo: y_p und y_pp über jacobi

g(1, 1) = {x_k-x(:, 1)}; % x0 = xk
for i=1:N_MPC
    % Set the state dynamics constraints
    g(1, i+1) = {F(x(:,i),u(:,i)) - x(:,i+1)}; % system equation
    
    % calculate output
    H_e = hom_transform_endeffector_casadi_SX(x(1:n,i+1), param_robot);
    y(:,i) = H_e(1:2,4);
end

% calculate cost function
J_y = dot(y-y_kp1_ref, mtimes(Q, y-y_kp1_ref));
J_yN = dot(y(:,end)-y_kp1_ref(:,end), mtimes(Q_yN, y(:,end)-y_kp1_ref(:,end)));

J_u = dot(u, mtimes(R_u, u));

J = J_y + J_u + J_yN;
%% set of parameter cellaray p

p = [x_k; y_kp1_ref(:)];
if(weights_and_limits_as_parameter)
    p = [p; Q(:); Q_yN(:); R_u(:); ...
            x_min(:); x_max(:); u_min(:); u_max(:)];
end

input_vars_SX  = {y_kp1_ref, x_k, x, u};
output_vars_SX = {J, vertcat(w{:}), vertcat(g{:}), vertcat(p{:})};
if(weights_and_limits_as_parameter)
    input_vars_SX  = {input_vars_SX{:}, Q, Q_yN, R_u, x_min, x_max, u_min, u_max};
    output_vars_SX = {output_vars_SX{:}, J_y, J_u, J_yN, lbw, ubw, lbg, ubg};
end
get_props = Function('prop_fun', input_vars_SX, output_vars_SX);

% get SX prop struct (SX is much faster than MX!)
if(weights_and_limits_as_parameter)
    [J, W, G, P, ~, ~, ~, ~, ~, ~, ~] = get_props(input_vars_SX{:}); % ist nicht anders als J, w, g, p oben
else
    [J, W, G, P] = get_props(input_vars_SX{:});
end
prob = struct('f', J, 'x', W, 'g', G, 'p', P);

% convert SX variables to MX (nlpsolve can only use MX variables)
input_vars_MX       = cellfun(@(x) sx_to_mx(x, 'get_MX_sym_cell' ), input_vars_SX, 'UniformOutput', false);
%input_vars_MX_names = cellfun(@(x) sx_to_mx(x, 'get_MX_name_cell'), input_vars_SX, 'UniformOutput', false);

if(weights_and_limits_as_parameter)
    [J_MX, w_MX, ~, p_MX, J_y_MX, J_yN_MX, J_u_MX, lbw_MX, ubw_MX, lbg_MX, ubg_MX] = get_props(input_vars_MX{:});
else
    [J_MX, w_MX, ~, p_MX] = get_props(input_vars_MX{:});
end
%% Create an NLP solver

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
    error(['invalid MPC solver=', MPC_solver, ' is a valid solver for nlpsol (only "qrqp", "ipopt", "qpoases", ... supported for nlpsol)']);
end

if(weights_and_limits_as_parameter)
    sol_sym = solver('x0', w_MX, 'lbx', lbw_MX, 'ubx', ubw_MX,...
        'lbg', lbg_MX, 'ubg', ubg_MX, 'p', p_MX);
else
    sol_sym = solver('x0', w_MX, 'lbx', lbw, 'ubx', ubw,...
        'lbg', lbg, 'ubg', ubg, 'p', p_MX);
end

% generate solver solutions variables
u_opt = sol_sym.x(numel(x)+(1:n));
x_full_opt = reshape(sol_sym.x(1:numel(x)), 2*n, N_MPC+1);
u_full_opt = reshape(sol_sym.x(numel(x)+1:end), n, N_MPC);

output_vars_MX = {u_opt, x_full_opt, u_full_opt};
if(weights_and_limits_as_parameter)
    output_vars_MX = {output_vars_MX{:}, J_y_MX, J_yN_MX, J_u_MX};
end
%output_vars_MX_names = cellfun(@(x) sx_to_mx(x, 'get_MX_name_cell'), output_vars_SX, 'UniformOutput', false);

if(weights_and_limits_as_parameter)
    f_opt = Function(casadi_func_name, ...
        {input_vars_MX{:}},...
        {output_vars_MX{:}});
    [u_opt_sol, x_full_opt_sol, u_full_opt_sol, J_y_sol, J_yN_sol, J_u_sol] = f_opt(y_ref_0, x_0_0, x_init_guess_0, u_init_guess_0, QQ, QQ_yN, RR_u, xx_min, xx_max, uu_min, uu_max);
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
    disp(['J = '      num2str(num2str(full(sum([J_y_sol, Ju_sol, ]))))]);
    disp(['Jy = '     num2str(full(J_y_sol     ))]);
    disp(['Ju = '     num2str(full(J_u_sol     ))]);
end

%% COMPILE (nlpsol)
if(compile_sfunction)
    
    copyfile(strcat(s_fun_path, 's_function_nlpsol.c'), s_fun_c_file_path, 'f');
    replace_strings_in_casadi_file(s_fun_c_file_path, casadi_func_name);
    f_opt.save([s_fun_path casadi_func_name, '.casadi']);
    
    lib_path = GlobalOptions.getCasadiPath();
    inc_path = GlobalOptions.getCasadiIncludePath();
    
    tic;
    %mex('-v',['-I' inc_path],['-L' lib_path],'-lcasadi', s_fun_c_file_path); % verbose mode (show compiler infos)
    disp("Compiling Simulink " + s_func_name + " (nlpsol, solver="+MPC_solver+") ");
    mex(['-I' inc_path],['-L' lib_path],'-lcasadi', s_fun_c_file_path, '-output', s_fun_c_file_path);
    disp(['Compile time for casadi s-function (nlpsol): ', num2str(toc), ' s']);
    %s_func_name = strcat('s_function_', casadi_func_name, '.c'); % final name for Simulink s-function
    
    file_name = 'f_opt.casadi';
end