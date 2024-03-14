import casadi.*

terminal_constrained_y = false;
n = param_robot.n_DOF;

x = MX.sym('x', 2*n);
u = MX.sym('u', n);
xdot = sys_fun(x, u, param_robot);
f = Function('f', {x, u}, {xdot});

% Declare model variables
x = SX.sym('x', 2*n);
u = SX.sym('u', n);

% Model equations

xdot = sys_fun_SX(x, u, param_robot);
f_SX = Function('f', {x, u}, {xdot});

% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.
% TODO: Zwischenstützstellen

X0 = SX.sym('X0', 2*n);
U = SX.sym('U', n);
X = X0;
for j=1:M
    % Runge-Kutta 4th order method
    k1 = f_SX(X, U);
    k2 = f_SX(X + DT/2 * k1, U);
    k3 = f_SX(X + DT/2 * k2, U);
    k4 = f_SX(X + DT * k3, U);
    X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
end
% Die Funktion F wird in CasADi mit der Function-Funktion erstellt.
% Die Funktion hat drei Eingabevariablen: X0, U und X, und eine
% Ausgabevariable X. Die Eingabevariablen X0 und U werden als Parameter
% definiert, während X als Variable definiert wird.
F_SX = Function('F', {X0, U}, {X});

%% Calculate Initial Guess
x_0_0 = [q_0;0;0];%q1, q2, d/dt q1, d/dt q2
q = x_0_0(1:n);
dq = x_0_0(n+1:2*n);
ddq = [0;0];
xe_k_0 = xe0(1:2); % x pos, y pos
u_k_0 = compute_tau(q, dq, ddq, param_robot); % tau1, tau2

u_init_guess_0 = ones(2,N_MPC).*u_k_0;

% für die S-funktion ist der Initial Guess wesentlich!
sim = F_SX.mapaccum(N_MPC);
x_init_guess_kp1_res = sim(x_0_0, u_init_guess_0);
x_init_guess_0 = [x_0_0 full(x_init_guess_kp1_res)];
% y_ref_0 = ones(2,N_MPC+1).*xe_k_0;
y_ref_0 = param_trajectory.p_d(1:2, 1+N_step_MPC : N_step_MPC : 1+N_MPC*N_step_MPC);

QQ           = eval("param_weight_"+casadi_func_name+".QQ;");
RR_u         = eval("param_weight_"+casadi_func_name+".RR_u;");
RR_du        = eval("param_weight_"+casadi_func_name+".RR_du;");
RR_dx        = eval("param_weight_"+casadi_func_name+".RR_dx;");
RR_dx_km1    = eval("param_weight_"+casadi_func_name+".RR_dx_km1;");
xx_min       = eval("param_weight_"+casadi_func_name+".xx_min;");
xx_max       = eval("param_weight_"+casadi_func_name+".xx_max;");
uu_min       = eval("param_weight_"+casadi_func_name+".uu_min;");
uu_max       = eval("param_weight_"+casadi_func_name+".uu_max;");

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    Q =        SX.sym('Q', 2,2); % x y weight
    R_u =      SX.sym('R_u', 2,2); % u absolut
    R_du =     SX.sym('R_du', 2,2); %un-un-1
    R_dx =     SX.sym('R_dx', 4,4); % x = d/dt q, d^2/dt^2 q
    R_dx_km1 = SX.sym('R_dx_km1', 4,4); % xk-xkm1
    x_min =    SX.sym('x_min', 4,1);    
    x_max =    SX.sym('x_max', 4,1);
    u_min =    SX.sym('u_min', 2,1);
    u_max =    SX.sym('u_max', 2,1);
else % hardcoded weights
    Q =        QQ;
    R_u =      RR_u;
    R_du =     RR_du;
    R_dx =     RR_dx;
    R_dx_km1 = RR_dx_km1;
    x_min =    xx_min;
    x_max =    xx_max;
    u_min =    uu_min;
    u_max =    uu_max;
end

%% Start with an empty NLP

% needed:
x       = SX.sym('x',      2*n, N_MPC+1 );
u       = SX.sym('u',      n,   N_MPC   );

% parameter
x_k       = SX.sym('x_k',    2*n, 1      );
u_km1     = SX.sym('us_km1', n,   1      );
y_kp1_ref = SX.sym('ys_ref', 2,   N_MPC  );


% optimization variables cellarray w
w = [x(:); u(:)];
lbw = [repmat(x_min, N_MPC + 1, 1); repmat(u_min, N_MPC, 1)];
ubw = [repmat(x_max, N_MPC + 1, 1); repmat(u_max, N_MPC, 1)];

% constraints conditions cellarray g

if(terminal_constrained_y)
    g = cell(1, N_MPC+2);
    lbg = zeros(numel(x)+1, 1); % equation conditions should be exact!
    ubg = zeros(numel(x)+1, 1);
else
    g = cell(1, N_MPC+1);
    lbg = zeros(numel(x), 1); % equation conditions should be exact!
    ubg = zeros(numel(x), 1);
end

%maybe dx erst ab 1 oder N_MPC+1 states?
dx = SX(2*n,N_MPC); % = [d/dt q, d^2/dt^2 q] = d/dt x 
y  = SX(2,N_MPC); % forward kinematics

g(1, 1) = {x_k-x(:, 1)}; % x0 = xk
for i=1:N_MPC
    % Set the state dynamics constraints
    g(1, i+1) = {F_SX(x(:,i),u(:,i)) - x(:,i+1)}; % =dx

    % calulate dx
    dx(:,i) = f_SX(x(:,i),u(:,i));

    % calculate output
    H_e = hom_transform_endeffector_casadi_SX(x(1:n,i+1), param_robot);
    y(:,i) = H_e(1:2,4);
end
if(terminal_constrained_y)
    g(1, end) = {dot(y(:,end)-y_kp1_ref(:,end), mtimes(1*eye(2), y(:,end)-y_kp1_ref(:,end)))};
    lbg(end-1:end) = 0;
    ubg(end-1:end) = 0.2;
end

% calculate cost function
Jy = dot(y-y_kp1_ref, mtimes(Q, y-y_kp1_ref));

Jdx = dot(dx, mtimes(R_dx, dx));
Jdxkm1 = dot(dx(:,2:end)-dx(:, 1:end-1), mtimes(R_dx_km1, dx(:, 2:end)-dx(:, 1:end-1))); % macht nur für d^2/dt^2 q Sinn

Ju = dot(u, mtimes(R_u, u));
Jukm1 = dot(u(:,1)-u_km1, mtimes(R_du, u(:,1)-u_km1)) + ...
    dot(u(:,2:end)-u(:, 1:end-1), mtimes(R_du, u(:, 2:end)-u(:, 1:end-1)));

J = Jy + Ju + Jukm1 + Jdx + Jdxkm1;

%% set of parameter cellaray p
p = [x_k; u_km1; y_kp1_ref(:); ... 
    Q(:); R_u(:); R_du(:); R_dx(:); R_dx_km1(:); ...
    x_min(:); x_max(:); u_min(:); u_max(:)];

prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}), 'p', vertcat(p{:}));
% options = struct('ipopt',struct('print_level',0),'print_time',false);
% solver = nlpsol('solver', 'ipopt', prob, options);

%% MX CONVERSION
xs_k       = MX.sym('xs_k',      size(x_k       ));
us_km1     = MX.sym('us_km1',    size(u_km1     ));
ys_kp1_ref = MX.sym('ys_ref',    size(y_kp1_ref ));
us         = MX.sym('us'  ,      size(u         ));
xs         = MX.sym('xs'  ,      size(x         ));
Qs         = MX.sym('Qs',        size(Q         ));
Rs_u       = MX.sym('Rs_u',      size(R_u       ));
Rs_du      = MX.sym('Rs_du',     size(R_du      ));
Rs_dx      = MX.sym('Rs_dx',     size(R_dx      ));
Rs_dx_km1  = MX.sym('Rs_dx_km1', size(R_dx_km1  ));
xs_min     = MX.sym('xs_min',    size(x_min     ));
xs_max     = MX.sym('xs_max',    size(x_max     ));
us_min     = MX.sym('us_min',    size(u_min     ));
us_max     = MX.sym('us_max',    size(u_max     ));

pp = [xs_k(:); us_km1(:); ys_kp1_ref(:)];

if(weights_and_limits_as_parameter)
    pp = [pp;   Qs(:); Rs_u(:); Rs_du(:); Rs_dx(:); Rs_dx_km1(:); ...
                xs_min(:); xs_max(:); us_min(:); us_max(:)];
end

lbw_MX = [repmat(xs_min, N_MPC + 1, 1); repmat(us_min, N_MPC, 1)];
ubw_MX = [repmat(xs_max, N_MPC + 1, 1); repmat(us_max, N_MPC, 1)];

ys_kp1 = MX(2,N_MPC);
dxs = MX(2*n,N_MPC);
for i=1:N_MPC
    H_e = hom_transform_endeffector_casadi(xs(1:2,i+1), param_robot);
    ys_kp1(:,i) = H_e(1:2,4); 
    %dx(:,i) = F(xs_k(:,i),us_k(:,i)); % difference quotient would be necessary
    dxs(:,i) = f(xs(:,i),us(:,i)); % makes more sense
end

Jsy = dot(ys_kp1-ys_kp1_ref, mtimes(Qs, ys_kp1-ys_kp1_ref));

Jsdx = dot(dxs, mtimes(Rs_dx, dxs));
Jsdxkm1 = dot(dxs(:,2:end)-dxs(:, 1:end-1), mtimes(Rs_dx_km1, dxs(:, 2:end)-dxs(:, 1:end-1))); % macht nur für d^2/dt^2 q Sinn

Jsu = dot(us, mtimes(Rs_u, us));

Jsukm1 = dot(us(:,1)-us_km1, mtimes(Rs_du, us(:,1)-us_km1)) + ...
    dot(us(:,2:end)-us(:, 1:end-1), mtimes(Rs_du, us(:, 2:end)-us(:, 1:end-1)));

Js = Jsy + Jsu + Jsukm1 + Jsdx + Jsdxkm1;

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

sol_sym = solver('x0', [xs(:); us(:)], 'lbx', lbw_MX, 'ubx', ubw_MX,...
            'lbg', lbg, 'ubg', ubg, 'p', pp);

u_opt = sol_sym.x(numel(xs)+(1:n));
x_full_opt = reshape(sol_sym.x(1:numel(xs)), 2*n, N_MPC+1);
u_full_opt = reshape(sol_sym.x(numel(xs)+1:end), n, N_MPC);

%{ys_ref, xs_0, us_km1, xs_k, us_k, Qs, Rs_u, Rs_du, Rs_dx, Rs_dx_km1, xs_min, xs_max, us_min, us_max},...
if(weights_and_limits_as_parameter)
    f_opt = Function(casadi_func_name, ...
        {ys_kp1_ref, xs_k, us_km1, xs, us, Qs, Rs_u, Rs_du, Rs_dx, Rs_dx_km1, xs_min, xs_max, us_min, us_max},...
        {u_full_opt(:,1), x_full_opt, u_full_opt, Jsy, Jsu, Jsukm1, Jsdx, Jsdxkm1},...
        {'yref', 'xk', 'ukm1', 'x_init_guess', 'u_init_guess', 'Q', 'R_u', 'R_du', 'R_dx', 'R_dx_km1', 'x_min', 'x_max', 'u_min', 'u_max'},...
        {'u_opt', 'x_full_opt', 'u_full_opt', 'Jy', 'Ju', 'Jukm1', 'Jdx', 'Jdxkm1'});
    
    [u_opt_sol, x_full_opt_sol, u_full_opt_sol, Jy_sol, Ju_sol, Jukm1_sol, Jdx_sol, Jdxkm1_sol] = f_opt(y_ref_0, x_0_0, u_init_guess_0(:,1), x_init_guess_0, u_init_guess_0, QQ, RR_u, RR_du, RR_dx, RR_dx_km1, xx_min, xx_max, uu_min, uu_max);
else
    % ohne extra parameter 30-60 % schneller!
    f_opt = Function(casadi_func_name, ...
        {ys_kp1_ref, xs_k, us_km1, xs, us},...
        {u_full_opt(:,1), x_full_opt, u_full_opt, Jy, Ju, Jukm1, Jdx, Jdxkm1},...
        {'yref', 'xk', 'ukm1', 'x_init_guess', 'u_init_guess'},...
        {'u_opt', 'x_full_opt', 'u_full_opt', 'Jy', 'Ju', 'Jukm1', 'Jdx', 'Jdxkm1'});

    [u_opt_sol, x_full_opt_sol, u_full_opt_sol, Jy_sol, Ju_sol, Jukm1_sol, Jdx_sol, Jdxkm1_sol] = f_opt(y_ref_0, x_0_0, u_init_guess_0(:,1), x_init_guess_0, u_init_guess_0);
end

% set init guess
x_init_guess = full(x_full_opt_sol);
u_init_guess = full(u_full_opt_sol);

disp(['J = '      num2str(num2str(full(sum([Jy_sol, Ju_sol, Jukm1_sol, Jdx_sol, Jdxkm1_sol]))))]);
disp(['Jy = '     num2str(full(Jy_sol     ))]);
disp(['Jdx = '    num2str(full(Jdx_sol    ))]);
disp(['Jdxkm1 = ' num2str(full(Jdxkm1_sol ))]);
disp(['Ju = '     num2str(full(Ju_sol     ))]);
disp(['Jukm1 = '  num2str(full(Jukm1_sol  ))]);

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