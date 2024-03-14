import casadi.*

%% Integrator (achtung in MX Datentyp!! - daher in nlpsol_opt_problem_SX anders.
n = param_robot.n_DOF;

% Declare model variables
x = MX.sym('x', 2*n);
u = MX.sym('u', n);

% Model equations
xdot = sys_fun(x, u, param_robot);
f = Function('f', {x, u}, {xdot});

%% time-integration methods

%{
% Integrator to discretize the system
intg_options = struct;
intg_options.tf = T_horizon_MPC/N_MPC; % Time step
% KEINE ZWISCHENSTÜTZSTELLEN
intg_options.simplify = true; % Simplify the expressions
intg_options.number_of_finite_elements = 1; % Number of finite elements (1 euler)

% DAE problem structure
dae = struct;
dae.x = x;         % What are states?
dae.p = u;         % What are parameters (=fixed during the integration horizon)?
dae.ode = f(x,u);  % Expression for the right-hand side

% Define the integrator
% The 'rk' option specifies the Runge-Kutta method to use for integration.
intg = integrator('intg','rk',dae,intg_options);

res = intg('x0',x,'p',u); % initial condition x, control input u

x_next = res.xf; % Extract the next state from the integration result

% Define a CasADi function for the system dynamics
% {x,u}: A list of input variables to the function. In this case, x and u are the input variables.
% {x_next}: A list of output variables from the function. In this case, x_next is the output variable.
F = Function('F', {x,u}, {x_next}, {'x', 'u'}, {'x_next'});
%}

%%{
% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator
M = rk_iter; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step
% TODO: Zwischenstützstellen

X0 = MX.sym('X0', 2*n);
U = MX.sym('U', n);
X = X0;
for j=1:M
    % Runge-Kutta 4th order method
    k1 = f(X, U);
    k2 = f(X + DT/2 * k1, U);
    k3 = f(X + DT/2 * k2, U);
    k4 = f(X + DT * k3, U);
    X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
end
% Die Funktion F wird in CasADi mit der Function-Funktion erstellt.
% Die Funktion hat drei Eingabevariablen: X0, U und X, und eine
% Ausgabevariable X. Die Eingabevariablen X0 und U werden als Parameter
% definiert, während X als Variable definiert wird.
F = Function('F', {X0, U}, {X}, {'x0','u'}, {'xf'});
%}
%% Test Discrete State Space Model - compare res with simulink!

sim = F.mapaccum(N_MPC);

% Define the initial state
u0 = [0 0]';
x0 = [0 0 0 0]';

% Simulate the system with zero control input
res_test = sim(x0, ones(2,N_MPC).*u0);

% Plot the results
if plot_null_simu==true
    figure;
    tgrid = linspace(0,T_horizon_MPC,N_MPC+2);
    plot(tgrid, full([x0 res_test]));
    legend('q1','q2', 'd/dt q1', 'd/dt q2');
    xlabel('t [s]');
end

%% Calculate Initial Guess
x_0_0 = [q_0;0;0];%q1, q2, d/dt q1, d/dt q2
q = x_0_0(1:n);
dq = x_0_0(n+1:2*n);
ddq = [0;0];
xe_k_0 = xe0(1:2); % x pos, y pos
u_k_0 = compute_tau(q, dq, ddq, param_robot); % tau1, tau2

u_init_guess_0 = ones(2,N_MPC).*u_k_0;

% für die S-funktion ist der Initial Guess wesentlich!
x_init_guess_kp1_res = sim(x_0_0, u_init_guess_0);
x_init_guess_0 = [x_0_0 full(x_init_guess_kp1_res)];
% y_ref_0 = ones(2,N_MPC+1).*xe_k_0;
y_ref_0 = param_trajectory.p_d(1:2, 1+N_step_MPC : N_step_MPC : 1+N_MPC*N_step_MPC);

%% Optimal control problem using multiple-shooting (WITH OPTI)
% Create an optimization problem

opti = casadi.Opti();

% Volldiskretisierung: Optimierungsvariablen sind x und u
x = opti.variable(2*n,N_MPC+1); % Define decision variables for state trajectory
u = opti.variable(n,N_MPC); % Define decision variables for control input

% forward kinematics
%H_e = hom_transform_endeffector_casadi(x(1:2,  end  ), param_robot); %zweiter x wert ist nächster!
%y = H_e(1:2,4); % x and y position of TCP

% ACHTUNG: Da x0=xk fix ist kann dieser Zustand nicht verändert werden. Da
% die homogene Transformation nur vom Zustand abhängt, macht es daher
% keinen Sinn, die Ausgangstrajektorie hier zu gewichten.

% Idee: Nur erster Wert soll zur Positionsregelung verwendet werden, da die
% anderen Werte im Horizont ruhig davon abweichen dürfen. (outdated)
y_kp1_ref = opti.parameter(2,N_MPC); % yref kann nur zukünftige werte beeinflussen! x0=xk ist fix, damit ist y0 = H(x0) fix.
u_km1 = opti.parameter(2,1); % u_{-1}
x_k = opti.parameter(2*n,1); % xk current value

%% SET LIMITS and default weights
QQ        = eval("param_weight_"+casadi_func_name+".QQ;");
RR_u      = eval("param_weight_"+casadi_func_name+".RR_u;");
RR_du     = eval("param_weight_"+casadi_func_name+".RR_du;");
RR_dx     = eval("param_weight_"+casadi_func_name+".RR_dx;");
RR_dx_km1 = eval("param_weight_"+casadi_func_name+".RR_dx_km1;");
%xx_min    = eval("param_weight_"+casadi_func_name+".xx_min;");
%xx_max    = eval("param_weight_"+casadi_func_name+".xx_max;");
%uu_min    = eval("param_weight_"+casadi_func_name+".uu_min;");
%uu_max    = eval("param_weight_"+casadi_func_name+".uu_max;");

% komischerweise dürfen sie bei opti nicht inf sein...
xx_min = [ -pi; -pi; -10; -10 ];
xx_max = [ pi; pi; 10; 10];
uu_min = 1*[ -10; -10 ];
uu_max = 1*[ 10; 10 ];

%% SET WEIGHTS (for initial guess)
% weights are already set in "parameters.m"
% see "init_MPC_weights.m"

% weights as parameter (~inputs)
if(weights_and_limits_as_parameter)
    Q = opti.parameter(2,2); % x y weight
    R_u = opti.parameter(2,2); % u absolut
    R_du = opti.parameter(2,2); %un-un-1
    R_dx = opti.parameter(4,4); % x = d/dt q, d^2/dt^2 q
    R_dx_km1 = opti.parameter(4,4); % xk-xkm1
    x_min = opti.parameter(4,1);
    x_max = opti.parameter(4,1);
    u_min = opti.parameter(2,1);
    u_max = opti.parameter(2,1);
else % hardcoded weights
    Q = QQ; % TODO
    R_u = RR_u;
    R_du = RR_du;
    R_dx = RR_dx;
    R_dx_km1 = RR_dx_km1;
    x_min = xx_min;
    x_max = xx_max;
    u_min = uu_min;
    u_max = uu_max;
end

%% Define equation and inequation constraint

for k=1:N_MPC
    % Set the state dynamics constraints
    opti.subject_to(x(:,k+1) == F(x(:,k),u(:,k))); % =dx
    %opti.subject_to(x(:,k+1) == dx(:,k)); % =dx

    % Set the control input constraints
    opti.subject_to(u_min<=u(:,k)<=u_max);

    % Set the state constraints
    opti.subject_to(x_min<=x(:,k)<=x_max);

    %opti.subject_to(ddqmin<=dx(n+1:2*n,k)<=ddqmax);
end

% Set the initial state constraint
opti.subject_to(x(:,1)==x_k); % init x0

% Set the final state constraint
opti.subject_to(x_min<=x(:,N_MPC+1)<=x_max);

dx = MX(2*n,N_MPC); % = [d/dt q, d^2/dt^2 q] = d/dt x, maybe erst ab 1 oder N_MPC+1 states?
y = MX(2,N_MPC); % forward kinematics

%y = MX(2,N_MPC+1);
%for i=2:N_MPC+1
%    H_e = hom_transform_endeffector_casadi(x(1:2,i), param_robot);
%    y(:,i) = H_e(1:2,4); 
%end
for i=1:N_MPC
    % calculate [d/dt q, d^2/dt^2 q] = d/dt x
    %dx(:,i) = F(x(:,i),u(:,i)); % difference quotient would be necessary
    dx(:,i) = f(x(:,i),u(:,i)); % makes more sense

    % calculate output
    H_e = hom_transform_endeffector_casadi(x(1:2,i+1), param_robot);
    y(:,i) = H_e(1:2,4); 
end

%% DEFINE COST FUNKTION
Jy = dot(y-y_kp1_ref, mtimes(Q, y-y_kp1_ref));

Jdx = dot(dx, mtimes(R_dx, dx));
Jdxkm1 = dot(dx(:,2:end)-dx(:, 1:end-1), mtimes(R_dx_km1, dx(:, 2:end)-dx(:, 1:end-1))); % macht nur für d^2/dt^2 q Sinn

Ju = dot(u, mtimes(R_u, u));
Jukm1 = dot(u(:,1)-u_km1, mtimes(R_du, u(:,1)-u_km1)) + ...
    dot(u(:,2:end)-u(:, 1:end-1), mtimes(R_du, u(:, 2:end)-u(:, 1:end-1)));

J = Jy + Ju + Jukm1 + Jdx + Jdxkm1;

% Set the objective function
opti.minimize(J);

%% Print optimization problem
%opti

%% Choose a concerete solver
if(strcmp(MPC_solver, 'qrqp'))
    opts = struct; % Create a new structure
    opts.qpsol = 'qrqp'; % Set the QP solver to 'qrqp'
    opts.print_header = false; % Disable printing of solver header
    opts.print_iteration = false; % Disable printing of solver iterations
    opts.print_time = false; % Disable printing of solver time
    opts.qpsol_options.print_iter = false; % Disable printing of QP solver iterations
    opts.qpsol_options.print_header = false; % Disable printing of QP solver header
    opts.qpsol_options.print_info = false; % Disable printing of QP solver info
    opts.print_status = false;
    opts.error_on_fail = false;
    %opts.hessian_approximation = 'bfgs'; % verbessert MPC, aber führt zu problemen in s-funktion
    opti.solver('sqpmethod',opts);
else
    error(['invalid MPC solver=', MPC_solver, ' is a valid solver for opti.solver (only "qrqp" supported for opti.solver)']);
end

% IPOPT NOT SUPPORTED FOR opti version
%opts.ipopt.print_level = 0;
%opts.ipopt.print_time = 0;
%opts.ipopt.print_level = 0;
%opts.ipopt.sb = 'yes';
%opti.solver('ipopt',opts);

%% Set Parameter and initial guess
% ich sollte im offline modus für y_ref gleich die trajektorie nehmen
% das ist somit alles für den online modus
opti.set_initial(x, x_init_guess_0);
opti.set_initial(u, u_init_guess_0);

% TEST SOLVER
opti.set_value(y_kp1_ref, y_ref_0); % e.g stay at same point
opti.set_value(u_km1, u_init_guess_0(:,1)); % nicht ganz korrekt
opti.set_value(x_k, x_0_0); % p,x0

if(weights_and_limits_as_parameter)
    opti.set_value(Q, QQ);
    opti.set_value(R_u, RR_u);
    opti.set_value(R_du,RR_du);
    opti.set_value(R_dx,RR_dx);
    opti.set_value(R_dx_km1,RR_dx_km1);
    opti.set_value(x_min, xx_min);
    opti.set_value(x_max, xx_max);
    opti.set_value(u_min, uu_min);
    opti.set_value(u_max, uu_max);
end

% calculate init guess
sol = opti.solve();
u_init_guess = full(sol.value(u));
x_init_guess = full(sol.value(x));

%{
% TODO: DELETE
QQ        = full(QQ);
RR_u      = full(RR_u);
RR_du     = full(RR_du);
RR_dx     = full(RR_dx);
RR_dx_km1 = full(RR_dx_km1);
xx_min    = full(xx_min);
xx_max    = full(xx_max);
uu_min    = full(uu_min); 
uu_max    = full(uu_max);
%}

if(print_init_guess_cost_functions)
    disp(['J = '      num2str(sol.value(J     ))]);
    disp(['Jy = '     num2str(sol.value(Jy    ))]);
    disp(['Jdx = '    num2str(sol.value(Jdx   ))]);
    disp(['Jdxkm1 = ' num2str(sol.value(Jdxkm1))]);
    disp(['Ju = '     num2str(sol.value(Ju    ))]);
    disp(['Jukm1 = '  num2str(sol.value(Jukm1 ))]);
end

%% COMPILE (OPTI)

if(weights_and_limits_as_parameter)
    f_opt = opti.to_function(casadi_func_name, ...
        {y_kp1_ref, x_k, u_km1, x, u, Q, R_u, R_du, R_dx, R_dx_km1, x_min, x_max, u_min, u_max},...
        {u(:,1), x, u, Jy, Ju, Jukm1, Jdx, Jdxkm1},...
        {'yref', 'xk', 'ukm1', 'x_guess', 'u_guess', 'Q', 'R_u', 'R_du', 'R_dx', 'R_dx_km1', 'x_min', 'x_max', 'u_min', 'u_max'},...
        {'u_opt', 'x_opt', 'u_full_opt', 'Jy', 'Ju', 'Jukm1', 'Jdx', 'Jdxkm1'});
else
    % ohne extra parameter 30-60 % schneller!
    f_opt = opti.to_function(casadi_func_name, {y_kp1_ref, x_k, u_km1, x, u},...
                      {u(:,1), x, u, Jy, Ju, Jukm1, Jdx, Jdxkm1},...
                      {'yref', 'xk', 'ukm1', 'x_guess', 'u_guess'},...
                      {'u_opt', 'x_opt', 'u_full_opt', 'Jy', 'Ju', 'Jukm1', 'Jdx', 'Jdxkm1'});
end

if(compile_sfunction)
    opts = struct('main', true, ...
        'mex', true);
    f_opt.generate(casadi_fun_c_header_str, opts);
    %f_opt.generate(casadi_fun_c_header_str);
    
    cg_options = struct;
    cg_options.casadi_real = 'real_T';
    cg_options.real_min = num2str(eps);
    cg_options.casadi_int = 'int_T';
    cg_options.with_header = true;
    cg = CodeGenerator(casadi_func_name,cg_options);
    cg.add_include('simstruc.h');
    cg.add(f_opt);
    cg.generate();

    movefile(casadi_fun_c_header_str, s_fun_path);
    movefile(casadi_fun_h_header_str, s_fun_path);

    disp("Compiling Simulink " + s_func_name + " (opti, solver="+MPC_solver+") ");
    
    % change s_function so that it is compatible with new name
    copyfile(strcat(s_fun_path, 's_function.c'), strcat(s_fun_path, s_func_name), 'f');
    replace_strings_in_casadi_file(strcat(s_fun_path,s_func_name), casadi_func_name);

    tic;
    
    % Optimization options for MATLAB compilation
    
    % Level  | Execution Time             | Code Size | Memory Usage | Compile Time
    %--------|----------------------------|-----------|--------------|--------------
    % -O0    | Default (no opt) (slow)    |           |              |      (+0%   )
    % -O1    | Balanced (size/time)       | ++        | ++           | ++   (+340% )
    % -O2    | More emphasis (size/time)  | ++        | +++          | +++  (+540% )
    % -O3    | Most emphasis (size/time)  | ---       | +++          | ++++ (+1860%)
    % -Os    | Optimize for code size     | ++        | --           | ++   (+400% )
    % -Ofast | Similar to -O3 (fast math) | +++       | +++          | ++++ (+2180%)

    opt_flag = '-O2'; % default in mex
    % Todo: add -v for verbose mode
    mex(s_fun_c_file_path, casadi_fun_c_header_path, '-outdir', s_fun_path); % ist anders als O2 aber compiliert fast gleich schnell?
    %mex(s_fun_c_file_path, casadi_fun_c_header_path, '-outdir', s_fun_path, ['COPTIMFLAGS="',opt_flag,'"']);
    %mex(s_fun_c_file_path, casadi_fun_c_header_path, '-outdir', s_fun_path, '-largeArrayDims', ['COPTIMFLAGS="',opt_flag,'"']);
    disp(['Compile time for casadi s-function (opti): ', num2str(toc), ' s']);

    delete(s_fun_c_file_path);
    delete(casadi_fun_h_header_path);
    delete(casadi_fun_c_header_path);
end