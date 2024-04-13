import casadi.*

n = param_robot.n_DOF;

% Declare model variables
x = SX.sym('x', 2*n);
u = SX.sym('u', n);

% Model equations
xdot = sys_fun_SX(x, u, param_robot);
f = Function('f', {x, u}, {xdot});

%%{
% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator
M = 1; % RK4 steps per interval
DT = T_horizon_MPC/N_MPC/M; % Time step - KEINE ZWISCHENST.
% TODO: Zwischenstützstellen

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
% Die Funktion F wird in CasADi mit der Function-Funktion erstellt.
% Die Funktion hat drei Eingabevariablen: X0, U und X, und eine
% Ausgabevariable X. Die Eingabevariablen X0 und U werden als Parameter
% definiert, während X als Variable definiert wird.
F = Function('F', {X0, U}, {X}, {'x0','u'}, {'xf'});
%}

import casadi.*

QQ = (diag([1e5 1e5])); % x y weight
RR_u = (diag([1e-4 1e-4])); % u absolut
RR_du = (diag([0 0])); % un-un-1
RR_dx = (diag([1e-2 1e-2 1e0 1e0]));
RR_dx_km1 = (diag([0 0 0 0]));

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

XX = [];
YY = [];
UU = [];

% Decision variables
w = {}; % List of decision variables
w_x = {};
w_u = {};
w0 = []; % Initial guess for decision variables
lbw = []; % Lower bounds for decision variables
ubw = []; % Upper bounds for decision variables

% Constraints
g = {}; % List of constraints
lbg = []; % Lower bounds for constraints
ubg = []; % Upper bounds for constraints

% Parameters
p = {}; % List of parameters
p_y = {}; % temporary list of y_ref parameter

% "Lift" initial conditions
X0 = SX.sym('X_0', 2*n);  % create first state, x0 is actual state
w_x = {w_x{:}, X0};  % add X0 to optimization variables
lbw = [lbw; x_min];  % add lower bounds for X0 optimization variables
ubw = [ubw; x_max];  % add upper bounds for X0 optimization variables
w0 = [w0; x_init_guess_0(:,1)];  % first guess value of X0 value
% The Guess x_init_guess_test was obtained by simulation of the system starting at
% u_init_guess = compute_tau(x_0), x_0 = [q_0;0];

% Create parameter symbol S0
S0 = SX.sym('S0', 2*n);
g = {g{:}, S0-X0}; % The Parameter S0 is x0 or xk i.e. the measured or
% estimated state that should be predicted. Therefore we add it to the
% equation constraints
p = {p{:}, S0}; % Add the Parameter to the list of parameters p
lbg = [lbg; zeros(2*n,1)]; % a value of 0 means that the equation constraints cannot be violated!
ubg = [ubg; zeros(2*n,1)]; % i.e. the equation constraint have to be exactly fulfilled

% Create Parameter Symbol for u_-1 = u_k-1 (previous input)
Ukm1 = SX.sym('Ukm1', n);
p = {p{:}, Ukm1}; % Add the Parameter to the list of parameters p

% Create first input value of actual prediction horizon:
Uk = SX.sym('U_0', n);

% Formulate the NLP
Xk = X0;

dXk = f(Xk,Uk);%maybe erst ab 1 oder N_MPC+1 states?

% First Cost function is spezial because we use here the parameter Ukm1
% Later the variable Ukm1 is a variable, don't be confused!
Jy     = 0;
Jdx    = dot( dXk,         mtimes(R_dx,     dXk      ));
Jdxkm1 = 0;
Ju     = dot(Uk,      mtimes( R_u,  Uk      ));
Jukm1  = dot(Uk-Ukm1, mtimes( R_du, Uk-Ukm1 ));

XX = [XX X0];
UU = [UU Uk];

for k=1:N_MPC
    % New NLP variable for the control

    w_u = {w_u{:}, Uk};
    lbw = [lbw; u_min];
    ubw = [ubw; u_max];
    w0 = [w0; u_init_guess_0(:,k)]; % init Uk as [0;0]

    % New reference output parameter
    Yref = SX.sym(['Yref_' num2str(k)], 2);
    p_y = {p_y{:}, Yref};

    % Integrate till the end of the interval
    Xk_end = F(Xk, Uk);
    

    % New NLP variable for state at end of interval
    Xkp1 = SX.sym(['X_' num2str(k)], 2*n);
    w_x = {w_x{:}, Xkp1};
    lbw = [lbw; x_min];
    ubw = [ubw; x_max];
    w0 = [w0; x_init_guess_0(:,k+1)];

    % New output value
    H_e = hom_transform_endeffector_casadi_SX(Xkp1, param_robot);
    Y = H_e(1:2,4);

    % Add equality constraint
    g = {g{:}, Xk_end-Xkp1};
    lbg = [lbg; zeros(2*n,1)]; %the equation constraint have to be exactly fulfilled
    ubg = [ubg; zeros(2*n,1)];

    XX = [XX Xkp1];
    YY = [YY Yref];

    % The 'if' statement is needed because the last input value has no influence on
    % the optimization problem because our end state with N = N_MPC
    % is xN and xN+1 = F(xN, uN) and will be never evaluated.
    if(k<N_MPC)
        Ukp1 = SX.sym(['U_' num2str(k)], n); % Next input value
        UU = [UU Ukp1];
        dXkp1 = f(Xkp1,Ukp1); % do not want to weight the first value
        Jdxkm1 = Jdxkm1 + dot( dXkp1-dXk,   mtimes(R_dx_km1, dXkp1-dXk )); % macht nur für d^2/dt^2 q Sinn
        
        dXk = dXkp1;
        Jdx    = Jdx    + dot( dXk,         mtimes(R_dx,     dXk      ));

        Ju     = Ju     + dot( Ukp1,        mtimes(R_u,     Ukp1     ));
        Jukm1  = Jukm1  + dot( Ukp1-Uk,     mtimes(R_du,    Ukp1-Uk  ));
    end

    Jy     = Jy     + dot( Y-Yref,      mtimes(Q,        Y-Yref      ));

    Xk = Xkp1;
    Uk = Ukp1;
end

p = {p{:}, p_y{:}}; % add y parameter
w = {w_x{:}, w_u{:}};

if(weights_and_limits_as_parameter)
    p = {p{:}, Q(:), R_u(:), R_du(:), R_dx(:), R_dx_km1(:), ...
               x_min(:), x_max(:), u_min(:), u_max(:)};
end

J = Jy + Ju + Jukm1 + Jdx + Jdxkm1;

%% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}), 'p', vertcat(p{:}));
% options = struct('ipopt',struct('print_level',0),'print_time',false);
% solver = nlpsol('solver', 'ipopt', prob, options);

opts = struct; % Create a new structure
opts.qpsol = 'qrqp'; % Set the QP solver to 'qrqp'
opts.print_header = false; % Disable printing of solver header
opts.print_iteration = false; % Disable printing of solver iterations
opts.print_time = false; % Disable printing of solver time
opts.qpsol_options.print_iter = false; % Disable printing of QP solver iterations
opts.qpsol_options.print_header = false; % Disable printing of QP solver header
opts.qpsol_options.print_info = false; % Disable printing of QP solver info
opts.qpsol_options.error_on_fail = false;
opts.print_iteration = false;
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

xs_0       = MX.sym('xs_0',      2*n            );
us_km1     = MX.sym('us_km1',      n            );
ys_kp1_ref = MX.sym('ys_ref',      2*(N_MPC)    );
us_k       = MX.sym('us_k',        n*(N_MPC)    );
xs_k       = MX.sym('xs_k',      2*n*(N_MPC+1)  );
Qs         = MX.sym('Qs',        numel(Q)       );
Rs_u       = MX.sym('Rs_u',      numel(R_u)     );
Rs_du      = MX.sym('Rs_du',     numel(R_du)    );
Rs_dx      = MX.sym('Rs_dx',     numel(R_dx)    );
Rs_dx_km1  = MX.sym('Rs_dx_km1', numel(R_dx_km1));
xs_min     = MX.sym('xs_min',    numel(x_min)   );
xs_max     = MX.sym('xs_max',    numel(x_max)   );
us_min     = MX.sym('us_min',    numel(u_min)   );
us_max     = MX.sym('us_max',    numel(u_max)   );

pp = [xs_0; us_km1; ys_kp1_ref];

if(weights_and_limits_as_parameter)
    pp = [pp;   Qs; Rs_u; Rs_du; Rs_dx; Rs_dx_km1; ...
                xs_min; xs_max; us_min; us_max];
end

lbw_MX = [repmat([xs_min; us_min], N_MPC, 1); xs_min];
ubw_MX = [repmat([xs_max; us_max], N_MPC, 1); xs_max];

sol_sym = solver('x0', [xs_k; us_k], 'lbx', lbw_MX, 'ubx', ubw_MX,...
            'lbg', lbg, 'ubg', ubg, 'p', pp);

u_opt = sol_sym.x(numel(xs_k)+(1:n));
x_full_opt = reshape(sol_sym.x(1:numel(xs_k)), 2*n, N_MPC+1);
u_full_opt = reshape(sol_sym.x(numel(xs_k)+1:end), n, N_MPC);

xs_0       = reshape(xs_0      , 2*n,     1   );
us_km1     = reshape(us_km1    , n,       1   );
ys_kp1_ref = reshape(ys_kp1_ref    , 2,   N_MPC   );
us_k       = reshape(us_k      , n,   N_MPC   );
xs_k       = reshape(xs_k      , 2*n, N_MPC+1 );
Qs         = reshape(Qs        , size(Q       ));
Rs_u       = reshape(Rs_u      , size(R_u     ));
Rs_du      = reshape(Rs_du     , size(R_du    ));
Rs_dx      = reshape(Rs_dx     , size(R_dx    ));
Rs_dx_km1  = reshape(Rs_dx_km1 , size(R_dx_km1));
xs_min     = reshape(xs_min    , size(x_min   ));
xs_max     = reshape(xs_max    , size(x_max   ));
us_min     = reshape(us_min    , size(u_min   ));
us_max     = reshape(us_max    , size(u_max   ));

ys_kp1 = MX(2,N_MPC);
dx = MX(2*n,N_MPC);
for i=1:N_MPC
    H_e = hom_transform_endeffector_casadi(xs_k(1:2,i+1), param_robot);
    ys_kp1(:,i) = H_e(1:2,4); 
    %dx(:,i) = F(xs_k(:,i),us_k(:,i)); % difference quotient would be necessary
    dx(:,i) = f(xs_k(:,i),us_k(:,i)); % makes more sense
end

Jy = dot(ys_kp1-ys_kp1_ref, mtimes(Qs, ys_kp1-ys_kp1_ref));

Jdx = dot(dx, mtimes(Rs_dx, dx));
Jdxkm1 = dot(dx(:,2:end)-dx(:, 1:end-1), mtimes(Rs_dx_km1, dx(:, 2:end)-dx(:, 1:end-1))); % macht nur für d^2/dt^2 q Sinn

Ju = dot(us_k, mtimes(Rs_u, us_k));

Jukm1 = dot(us_k(:,1)-us_km1, mtimes(Rs_du, us_k(:,1)-us_km1)) + ...
    dot(us_k(:,2:end)-us_k(:, 1:end-1), mtimes(Rs_du, us_k(:, 2:end)-us_k(:, 1:end-1)));

J = Jy + Ju + Jukm1 + Jdx + Jdxkm1;

%{ys_ref, xs_0, us_km1, xs_k, us_k, Qs, Rs_u, Rs_du, Rs_dx, Rs_dx_km1, xs_min, xs_max, us_min, us_max},...
if(weights_and_limits_as_parameter)
    f_opt = Function(casadi_func_name, ...
        {ys_kp1_ref, xs_0, us_km1, xs_k, us_k, Qs, Rs_u, Rs_du, Rs_dx, Rs_dx_km1, xs_min, xs_max, us_min, us_max},...
        {u_full_opt(:,1), x_full_opt, u_full_opt, Jy, Ju, Jukm1, Jdx, Jdxkm1},...
        {'yref', 'xk', 'ukm1', 'x_init_guess', 'u_init_guess', 'Q', 'R_u', 'R_du', 'R_dx', 'R_dx_km1', 'x_min', 'x_max', 'u_min', 'u_max'},...
        {'u_opt', 'x_full_opt', 'u_full_opt', 'Jy', 'Ju', 'Jukm1', 'Jdx', 'Jdxkm1'});
    
    [u_opt_sol, x_full_opt_sol, u_full_opt_sol, Jy_sol, Ju_sol, Jukm1_sol, Jdx_sol, Jdxkm1_sol] = f_opt(y_ref_0, x_0_0, u_init_guess_0(:,1), x_init_guess_0, u_init_guess_0, QQ, RR_u, RR_du, RR_dx, RR_dx_km1, xx_min, xx_max, uu_min, uu_max);
else
    % ohne extra parameter 30-60 % schneller!
    f_opt = Function(casadi_func_name, ...
        {ys_kp1_ref, xs_0, us_km1, xs_k, us_k},...
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
    disp("Compiling Simulink " + s_func_name + " (nlpsol)");
    mex(['-I' inc_path],['-L' lib_path],'-lcasadi', s_fun_c_file_path, '-output', s_fun_c_file_path);
    disp(['Compile time for casadi s-function (nlpsol): ', num2str(toc), ' s']);
    %s_func_name = strcat('s_function_', casadi_func_name, '.c'); % final name for Simulink s-function

    %file_name = 'f_opt.casadi';
end