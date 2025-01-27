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

import casadi.*;

tic;
fprintf('\nStarting execution of solver = nlpsol(''solver'', ''%s'', prob, opts)\n', MPC_solver)
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
    opts.verbose = false;

     opts.hessian_approximation = 'exact';
%     % opts.hessian_approximation = 'limited-memory';
%     % opts.iteration_callback_ignore_errors = true;
%     % opts.elastic_mode = true;
%     % opts.regularity_check = true;
%     opts.show_eval_warnings = false;

%     %opts.tol_du=1e-12;%1e-6;
%     %opts.tol_pr=1e-12;%1e-6;
% %
%     %opts.max_iter = 10000;%1500;

    opts.max_iter = 100;  % Up to 100 iterations; tune as necessary
    opts.min_step_size = 1e-6;  % Setting a reasonable minimum step
    opts.gamma_0 = 10;  % Initial penalty; adjust according to observation
    opts.gamma_max = 100;  % Maximum penalty
    opts.jac_penalty = 5;  % Avoid full Jacobian computation when not necessary
    opts.enable_jacobian = false;  % Toggle based on need
    opts.enable_forward = false;  % Toggle based on need
    opts.enable_reverse = true;  % Keep this if reverse mode is used
    opts.max_num_dir = 5;  % Adjust based on profiling

    opts.show_eval_warnings = false;  % Disable evaluation warnings 
    
    solver = nlpsol('solver', 'sqpmethod', prob, opts);

    % solver.print_options();
elseif(strcmp(MPC_solver, 'fatrop'))
    opts = struct;
    
    % General options to control the solver behavior
    opts.print_time = true; % Enable printing solver execution time for debugging
    opts.error_on_fail = false; % Throw exceptions when function evaluation fails
    opts.verbose = false; % Verbose output to help with debugging
    opts.show_eval_warnings = false; % Show evaluation warnings
    opts.regularity_check = false; % Throw exceptions when NaN or Inf appears during evaluation
    opts.debug = false; % Enable debug mode for the solver

    % Fatrop-specific options
    opts.fatrop = struct;
    opts.fatrop.print_level = 0; % Set the level of printing (-1 for no printing, 0 for normal, 1 for verbose)
    opts.fatrop.max_iter = 100; % Specify maximum iterations, adjust based on problem scale
    opts.fatrop.tol = 1e-6; % Set a tolerance for the convergence criteria
    opts.fatrop.mu_init = 0.1;

    % opts.structure_detection = 'auto';
    % opts.equality = {1};
    if(exist('ng', 'var') && exist('nu', 'var') && exist('nx', 'var') && exist('N', 'var'))
        opts.N = N; % OCP horizon
        opts.ng = ng; % Number of non-dynamic constraints, length N+1
        opts.nu = nu; % Number of controls, length N+1
        opts.nx = nx; % Number of states, length N+1
        opts.structure_detection = 'manual'; % Specify structure detection method
    end

    % opts.convexify_margin=1e-7; % [OT_DOUBLE] when using a convexification strategy, make sure that the smallest eigenvalue is at least this (default: 1e-7).
    % opts.convexify_strategy='regularize'; % [OT_STRING] NONE|regularize|eigen-reflect|eigen-clip. Strategy to convexify the Lagrange Hessian before passing it to the solver.

    % Specify these options if needed for optimization
    % opts.expand = true; % Replace MX with SX expressions in problem formulation [false]
    opts.max_num_dir = 50; % Maximum number of directions for derivatives, adjust as necessary
    % opts.ad_weight = 0.5; % Balance between forward and reverse mode AD; may depend on problem structure

    opts.show_eval_warnings = false;  % Disable evaluation warnings 
    
    solver = nlpsol('solver', 'fatrop', prob, opts);

elseif(strcmp(MPC_solver, 'feasiblesqpmethod'))
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
    % opts.max_iter = 1000;
    opts.tol_du=1e-6;
    opts.tol_pr=1e-6;
    solver = nlpsol('solver', 'feasiblesqpmethod', prob, opts);
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
    opts = struct;
    opts.show_eval_warnings = false;
    opts.error_on_fail = false;
    opts.print_time = 0;

    all_ipopt_options;
    %opts.ipopt.print_level = 5;
    %opts.ipopt.fast_step_computation = 'yes';
    %opts.ipopt.print_info_string = 'yes';
    % opts.jit = true;
    % opts.compiler = 'shell';
    % opts.jit_options.flags = {'-O3'};
    % opts.jit_options.verbose = true;
    solver = nlpsol('solver', 'ipopt', prob, opts);
    %solver.print_options();
else
    error(['invalid MPC solver=', MPC_solver, ...
           ' is not a valid solver for nlpsol (only "qrqp", "ipopt", "qpoases", "fatrop" ... supported for nlpsol)']);
end
disp(['Execution time of solver = nlpsol(''solver'', ''', MPC_solver, ''', prob, opts): ', num2str(toc), ' s']);
fprintf('\n');

% solver.print_options();