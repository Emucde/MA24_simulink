%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Documentation: Tune ESP controller with M(q)
% Name: Pascal Ban
% Measurement date: 14.04.2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Optimization problem

% Cost functions

% Min
% fun = @(x) norm(diag(inertia_matrix(x, param_robot)));

% Max
fun = @(x) -norm(diag(inertia_matrix_py(x)));

% Startpoint
x0 = zeros(n,1);

% Bounds
lb = param_robot.q_limit_lower;
ub = param_robot.q_limit_upper;

% Nonlinear constrains
nonlcon = [];

% Options choice
flag_optimization_options = 1;

switch(flag_optimization_options)

    case 1 % SQP - Method
        opt = optimoptions('fmincon','Display','iter','Algorithm','sqp');
    
    case 2 % Active-Set - Method
        opt = optimoptions('fmincon','Display','iter','Algorithm','active-set');
    
    case 3 % Interior-point - Method
        opt = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
    
    otherwise
        disp('other option')

end

% Additional options
% opt = optimoptions(opt,'CheckGradients', true);
% opt = optimoptions(opt,'FiniteDifferenceType', 'central');
% opt = optimoptions(opt,'FiniteDifferenceStepSize', 1e-10);
opt = optimoptions(opt,'MaxFunctionEvaluations', 1e5);
opt = optimoptions(opt,'MaxIterations', 5e3);
opt = optimoptions(opt, 'EnableFeasibilityMode', true);
opt = optimoptions(opt, 'Display', 'off' );


%% Find single local minima - fmincon

% [x_sol,fval,exitflag,output] = fmincon(fun,x0,[],[],[],[],lb,ub,nonlcon,opt);


%% Find multiple local minima - MultiStart

rng default % For reproducibility

% Create optimization problem
problem = createOptimProblem('fmincon', 'objective', fun, 'x0', x0, 'Aineq', [],...
    'bineq', [], 'Aeq', [], 'beq', [],  'lb', lb, 'ub', ub, 'nonlcon', nonlcon,'options', opt);

% MultiStart options
ms = MultiStart('Display', 'iter', 'StartPointsToRun', 'bounds');
ms = MultiStart(ms, 'UseParallel', true);

% The positive integer k specifies the number of start points MultiStart uses.
% MultiStart generates random start points using the dimension of the problem
% and bounds from the problem structure. MultiStart generates k - 1 random
% start points, and also uses the x0 start point from the problem structure.
k = 1e4;

% Jointspace
resolution = 5;
x_range = zeros(n, resolution);

for i = 1:n
    x_range(i,:) = linspace(lb(i), ub(i), resolution);
end

% Startpoints
i = 1;
sps = zeros(resolution^n, n);

for i1 = 1:resolution
    for i2 = 1:resolution
        for i3 = 1:resolution
            for i4 = 1:resolution
                for i5 = 1:resolution
                    for i6 = 1:resolution
                        for i7 = 1:resolution
                            if(n==7)
                                sps(i,:) = [x_range(1,i1), x_range(2,i2), x_range(3,i3), x_range(4,i4), x_range(5,i5), x_range(6,i6), x_range(7,i7)];
                            else
                                sps(i,:) = [x_range(1,i1), x_range(2,i2), x_range(3,i3), x_range(4,i4), x_range(5,i5), x_range(6,i6)];
                            end
                            i = i + 1;
                        end
                    end
                end
            end
        end
    end
end

% Get custom startpoint set
startpointset = CustomStartPointSet(sps);

% Run MultiStart
% [x_sol, fmin, flag, outpt, allmins] = run(ms, problem, k);

[x_sol, fmin, flag, outpt, allmins] = run(ms, problem, startpointset);

% Solution
M_sol = diag(inertia_matrix_py(x_sol));

q_pp_max = M_sol.^(-1) .* param_robot.torque_limit_upper;
q_pp_min = M_sol.^(-1) .* param_robot.torque_limit_lower;

%q_pp_max = inertia_matrix_py(x_sol) \ param_robot.torque_limit_upper;
%q_pp_min = inertia_matrix_py(x_sol) \ param_robot.torque_limit_lower;

% Min and Max - default parameters
% M_sol_min = [0.6063, 1.4648, 0.5683, 0.7269, 0.0213, 0.0160, 0.0011];
% M_sol_max = [5.2091, 5.1907, 0.0785, 0.8825, 0.0129, 0.0160, 0.0011];

% Min and Max - identified parameters
% M_sol_min = [0.5507, 1.4885, 0.5474, 0.7733, 0.0128, 0.0254, 0.0015];
% M_sol_max = [5.3353, 5.3512, 0.0226, 0.9606, 0.0125, 0.0262, 0.0015];


