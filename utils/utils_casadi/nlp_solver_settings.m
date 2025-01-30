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
%| qpoases           |
%| qrqp              |
%| osqp              |
%| ooqp              |
%| proxqp            |
%| daqp              |
%| fatrop            |
%| highs             |
%|-------------------|

% Interface to the JIT compiler SHELL
% See sourcecode: ./casadi/solvers/shell_compiler.cpp
%                 ./swig/doc_merged.i

% Extra doc: https://github.com/casadi/casadi/wiki/L_22w

% >List of available options

% +----------------------+-----------------+---------------------------------+
% |          Id          |      Type       |           Description           |
% +======================+=================+=================================+
% | cleanup              | OT_BOOL         | Cleanup temporary files when    |
% |                      |                 | unloading. Default: true        |
% +----------------------+-----------------+---------------------------------+
% | compiler             | OT_STRING       | Compiler command                |
% +----------------------+-----------------+---------------------------------+
% | compiler_flags       | OT_STRINGVECTOR | Alias for 'compiler_flags'      |
% +----------------------+-----------------+---------------------------------+
% | compiler_output_flag | OT_STRING       | Compiler flag to denote object  |
% |                      |                 | output. Default: '-o '          |
% +----------------------+-----------------+---------------------------------+
% | compiler_setup       | OT_STRING       | Compiler setup command.         |
% |                      |                 | Intended to be fixed. The       |
% |                      |                 | 'flag' option is the prefered   |
% |                      |                 | way to set custom flags.        |
% +----------------------+-----------------+---------------------------------+
% | directory            | OT_STRING       | Directory to put temporary      |
% |                      |                 | objects in. Must end with a     |
% |                      |                 | file separator.                 |
% +----------------------+-----------------+---------------------------------+
% | extra_suffixes       | OT_STRINGVECTOR | List of suffixes for extra      |
% |                      |                 | files that the compiler may     |
% |                      |                 | generate. Default: None         |
% +----------------------+-----------------+---------------------------------+
% | flags                | OT_STRINGVECTOR | Compile flags for the JIT       |
% |                      |                 | compiler. Default: None         |
% +----------------------+-----------------+---------------------------------+
% | linker               | OT_STRING       | Linker command                  |
% +----------------------+-----------------+---------------------------------+
% | linker_flags         | OT_STRINGVECTOR | Linker flags for the JIT        |
% |                      |                 | compiler. Default: None         |
% +----------------------+-----------------+---------------------------------+
% | linker_output_flag   | OT_STRING       | Linker flag to denote shared    |
% |                      |                 | library output. Default: '-o '  |
% +----------------------+-----------------+---------------------------------+
% | linker_setup         | OT_STRING       | Linker setup command. Intended  |
% |                      |                 | to be fixed. The 'flag' option  |
% |                      |                 | is the prefered way to set      |
% |                      |                 | custom flags.                   |
% +----------------------+-----------------+---------------------------------+
% | name                 | OT_STRING       | The file name used to write out |
% |                      |                 | compiled objects/libraries. The |
% |                      |                 | actual file names used depend   |
% |                      |                 | on 'temp_suffix' and include    |
% |                      |                 | extensions. Default:            |
% |                      |                 | 'tmp_casadi_compiler_shell'     |
% +----------------------+-----------------+---------------------------------+
% | temp_suffix          | OT_BOOL         | Use a temporary (seemingly      |
% |                      |                 | random) filename suffix for     |
% |                      |                 | file names. This is desired for |
% |                      |                 | thread-safety. This behaviour   |
% |                      |                 | may defeat caching compiler     |
% |                      |                 | wrappers. Default: true         |
% +----------------------+-----------------+---------------------------------+

import casadi.*;

opts=struct;

if(use_jit)
    opts.jit = true;
    opts.jit_cleanup = false;
    opts.jit_temp_suffix = false;
    opts.jit_name = 'test';
    opts.compiler = 'shell';
    opts.jit_options.flags = {'-O3'};
    opts.jit_options.temp_suffix = false;
    opts.jit_options.verbose = true;
    opts.jit_options.name = ['lib', casadi_func_name, '_jit'];
    opts.jit_options.directory = [s_fun_path, '/mpc_c_sourcefiles/'];
end

tic;
fprintf('\nStarting execution of solver = nlpsol(''solver'', ''%s'', prob, opts)\n', MPC_solver)
if(strcmp(MPC_solver, 'qrqp') || strcmp(MPC_solver, 'osqp') || strcmp(MPC_solver, 'ooqp') || strcmp(MPC_solver, 'proxqp') || strcmp(MPC_solver, 'daqp') || strcmp(MPC_solver, 'highs') || strcmp(MPC_solver, 'test'))
    % DOKU: https://web.casadi.org/api/html/d4/d89/group__nlpsol.html
    opts.qpsol_options = struct;
    opts.print_header = false; % Disable printing of solver header
    opts.print_iteration = false; % Disable printing of solver iterations
    opts.print_time = false; % Disable printing of solver time
    opts.print_status = false;
    opts.error_on_fail = false;
    opts.verbose = false;
    opts.hessian_approximation = 'exact';

    if(strcmp(MPC_solver, 'qrqp'))
        opts.qpsol = 'qrqp'; % Set the QP solver to 'qrqp'
        opts.qpsol_options.max_iter = 100;                % Maximum number of iterations [Default: 1000], [Allowed: > 0]
        opts.qpsol_options.constr_viol_tol = 1e-8;       % Constraint violation tolerance [Default: 1e-08], [Allowed: > 0]
        opts.qpsol_options.dual_inf_tol = 1e-8;          % Dual feasibility violation tolerance [Default: 1e-08], [Allowed: > 0]
        opts.qpsol_options.print_header = false;            % Print header [Default: true], [Allowed: true/false]
        opts.qpsol_options.print_iter = false;              % Print iterations [Default: true], [Allowed: true/false]
        opts.qpsol_options.print_info = false;              % Print info [Default: true], [Allowed: true/false]
        opts.qpsol_options.print_lincomb = false;          % Print dependent linear combinations of constraints [Default: false], [Allowed: true/false]
        opts.qpsol_options.min_lam = 0;      
        opts.qpsol_options.error_on_fail = false;
    elseif(strcmp(MPC_solver, 'osqp'))
        % DOKU: https://github.com/casadi/casadi/blob/main/casadi/interfaces/osqp/osqp_interface.cpp
        opts.qpsol = 'osqp'; % Set the QP solver to 'osqp'
        opts.qpsol_options.error_on_fail = false;
        opts.qpsol_options.print_problem = false;
        opts.qpsol_options.print_time = false;
        opts.qpsol_options.verbose = false;
        opts.qpsol_options.warm_start_primal = true; % Use x0 input to warmstart
        opts.qpsol_options.warm_start_dual = true; % Use lam_a0 and lam_x0 input to warmstart
        opts.qpsol_options.osqp.rho = 0.1; % ADMM rho step
        opts.qpsol_options.osqp.sigma = 1e-06; % ADMM sigma step
        opts.qpsol_options.osqp.scaling = 10; % Number of scaling iterations
        opts.qpsol_options.osqp.adaptive_rho = true; % Use adaptive rho
        opts.qpsol_options.osqp.adaptive_rho_interval = 0; % Adaptive rho interval
        opts.qpsol_options.osqp.adaptive_rho_tolerance = 5; % Tolerance for adapting rho
        opts.qpsol_options.osqp.max_iter = 100; % Maximum number of iterations
        opts.qpsol_options.osqp.eps_abs = 1e-03; % Absolute tolerance
        opts.qpsol_options.osqp.eps_rel = 1e-03; % Relative tolerance
        opts.qpsol_options.osqp.eps_prim_inf = 1e-04; % Primal infeasibility tolerance
        opts.qpsol_options.osqp.eps_dual_inf = 1e-04; % Dual infeasibility tolerance
        opts.qpsol_options.osqp.alpha = 1.6; % ADMM relaxation parameter
        opts.qpsol_options.osqp.delta = 1e-06; % Polishing regularization parameter
        opts.qpsol_options.osqp.polish = false; % Perform polishing
        opts.qpsol_options.osqp.polish_refine_iter = 3; % Refinement iterations in polishing
        opts.qpsol_options.osqp.verbose = false; % Print output
        opts.qpsol_options.osqp.scaled_termination = false; % Scaled termination conditions
        opts.qpsol_options.osqp.check_termination = 25; % Check termination interval
    elseif(strcmp(MPC_solver, 'proxqp'))
        % DOKU: https://github.com/casadi/casadi/blob/main/casadi/interfaces/proxqp/proxqp_interface.cpp
        warning('Code generation not supported for ProxqpInterface');
        opts.qpsol = 'proxqp'; % Set the QP solver to 'proxqp'
        opts.qpsol_options.error_on_fail = false;
        opts.qpsol_options.print_problem = false;
        opts.qpsol_options.print_time = false;
        opts.qpsol_options.verbose = false;
        opts.qpsol_options.warm_start_primal = true; % Use x0 input to warmstart
        opts.qpsol_options.warm_start_dual = true; % Use lam_a0 and lam_x0 input to warmstart
        opts.qpsol_options.proxqp.default_rho = 1.0; % Regularization parameter
        opts.qpsol_options.proxqp.default_mu_eq = 1.0; % Default value for the dual variable associated with equality constraints
        opts.qpsol_options.proxqp.default_mu_in = 1.0; % Default value for the dual variable associated with inequality constraints
        opts.qpsol_options.proxqp.eps_abs = 1e-06; % Absolute stopping criterion
        opts.qpsol_options.proxqp.eps_rel = 1e-06; % Relative stopping criterion
        opts.qpsol_options.proxqp.max_iter = 100; % Maximum number of iterations
        opts.qpsol_options.proxqp.verbose = false; % Print output
        opts.qpsol_options.proxqp.backend = 'sparse'; % The solver backend to use
    elseif(strcmp(MPC_solver, 'daqp'))
        % https://darnstrom.github.io/daqp/parameters/
        opts.qpsol = 'daqp'; % Set the QP solver to 'qrqp'
        opts.qpsol_options.daqp.primal_tol = 1e-6;     % Tolerance for primal infeasibility [Default: 1e-6], [Allowed: > 0]
        opts.qpsol_options.daqp.dual_tol = 1e-12;      % Tolerance for dual infeasibility [Default: 1e-12], [Allowed: > 0]
        opts.qpsol_options.daqp.zero_tol = 1e-11;      % Values below are regarded as zero [Default: 1e-11], [Allowed: > 0]
        opts.qpsol_options.daqp.pivot_tol = 1e-6;      % Value used for determining if rows in the LDL' factorization should be exchanged [Default: 1e-6], [Allowed: > 0]
        opts.qpsol_options.daqp.progress_tol = 1e-6;   % Minimum change in objective function to consider it progress [Default: 1e-6], [Allowed: > 0]
        opts.qpsol_options.daqp.cycle_tol = 10;        % Allowed number of iterations without progress before terminating [Default: 10], [Allowed: >= 0]
        opts.qpsol_options.daqp.iter_limit = 1000;     % Maximum number of iterations before terminating [Default: 1000], [Allowed: > 0]
        opts.qpsol_options.daqp.fval_bound = 1e30;     % Maximum allowed objective function value [Default: 1e30], [Allowed: > -inf]
        opts.qpsol_options.daqp.eps_prox = 0;          % Regularization parameter used for proximal-point iterations [Default: 0], [Allowed: >= 0]
        opts.qpsol_options.daqp.eta_prox = 1e-6;       % Tolerance that determines if a fix-point has been reached during proximal-point iterations [Default: 1e-6], [Allowed: > 0]
        opts.qpsol_options.daqp.rho_soft = 1e-3;       % Weight used for soft constraints [Default: 1e-3], [Allowed: > 0]
        opts.qpsol_options.daqp.rel_subopt = 0;        % Allowed relative suboptimality in branch and bound [Default: 0], [Allowed: >= 0]
        opts.qpsol_options.daqp.abs_subopt = 0;        % Allowed absolute suboptimality in branch and bound [Default: 0], [Allowed: >= 0]
        opts.qpsol_options.error_on_fail = false; % Throw exceptions when function evaluation fails
    elseif(strcmp(MPC_solver, 'highs'))
        % https://darnstrom.github.io/daqp/parameters/
        opts.qpsol = 'highs'; % Set the QP solver to 'qrqp'
           % Print iterations [Default: true], [Allowed: true/false]
        opts.qpsol_options.verbose = false; % Print output 
        opts.qpsol_options.error_on_fail = false; % Throw exceptions when function evaluation fails
        opts.error_on_fail=false;
        opts.qpsol_options.highs.presolve = 'choose'; % [Default: 'choose'], [Allowed: 'off', 'choose', 'on']
        opts.qpsol_options.highs.solver = 'choose'; % [Default: 'choose'], [Allowed: 'simplex', 'choose', 'ipm', 'pdlp']
        opts.qpsol_options.highs.parallel = 'choose'; % [Default: 'choose'], [Allowed: 'off', 'choose', 'on']
        opts.qpsol_options.highs.run_crossover = 'on'; % [Default: 'on'], [Allowed: 'off', 'choose', 'on']
        opts.qpsol_options.highs.time_limit = inf; % [Default: inf], [Allowed: [0, inf]]
        opts.qpsol_options.highs.ranging = 'off'; % [Default: 'off'], [Allowed: 'off', 'on']
        opts.qpsol_options.highs.infinite_cost = 1e+15; % [Default: 1e+20], [Allowed: [1e+15, inf]]
        opts.qpsol_options.highs.infinite_bound = 1e+15; % [Default: 1e+20], [Allowed: [1e+15, inf]]
        opts.qpsol_options.highs.small_matrix_value = 1e-09; % [Default: 1e-09], [Allowed: [1e-12, inf]]
        opts.qpsol_options.highs.large_matrix_value = 1e+15; % [Default: 1e+15], [Allowed: [1, inf]]
        opts.qpsol_options.highs.primal_feasibility_tolerance = 1e-07; % [Default: 1e-07], [Allowed: [1e-10, inf]]
        opts.qpsol_options.highs.dual_feasibility_tolerance = 1e-07; % [Default: 1e-07], [Allowed: [1e-10, inf]]
        opts.qpsol_options.highs.ipm_optimality_tolerance = 1e-08; % [Default: 1e-08], [Allowed: [1e-12, inf]]
        opts.qpsol_options.highs.objective_bound = inf; % [Default: inf], [Allowed: [-inf, inf]]
        opts.qpsol_options.highs.objective_target = -inf; % [Default: -inf], [Allowed: [-inf, inf]]
        opts.qpsol_options.highs.random_seed = 0; % [Default: 0], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.threads = 0; % [Default: 0], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.user_bound_scale = 0; % [Default: 0], [Allowed: {-2147483647, 2147483647}]
        opts.qpsol_options.highs.user_cost_scale = 0; % [Default: 0], [Allowed: {-2147483647, 2147483647}]
        opts.qpsol_options.highs.simplex_strategy = 1; % [Default: 1], [Allowed: {0, 4}]
        opts.qpsol_options.highs.simplex_scale_strategy = 1; % [Default: 1], [Allowed: {0, 5}]
        opts.qpsol_options.highs.simplex_dual_edge_weight_strategy = -1; % [Default: -1], [Allowed: {-1, 2}]
        opts.qpsol_options.highs.simplex_primal_edge_weight_strategy = -1; % [Default: -1], [Allowed: {-1, 2}]
        opts.qpsol_options.highs.simplex_iteration_limit = 2147483647; % [Default: 2147483647], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.simplex_update_limit = 5000; % [Default: 5000], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.simplex_max_concurrency = 8; % [Default: 8], [Allowed: {1, 8}]
        opts.qpsol_options.highs.output_flag = false; % [Default: 'true'], [Allowed: boolean]
        opts.qpsol_options.highs.log_to_console = false; % [Default: 'true'], [Allowed: boolean]
        opts.qpsol_options.highs.solution_file = ''; % [Default: ''], [Allowed: string]
        opts.qpsol_options.highs.log_file = ''; % [Default: ''], [Allowed: string]
        opts.qpsol_options.highs.write_solution_to_file = false; % [Default: 'false'], [Allowed: boolean]
        opts.qpsol_options.highs.write_solution_style = 0; % [Default: 0], [Allowed: {-1, 4}]
        opts.qpsol_options.highs.glpsol_cost_row_location = 0; % [Default: 0], [Allowed: {-2, 2147483647}]
        opts.qpsol_options.highs.write_model_file = ''; % [Default: ''], [Allowed: string]
        opts.qpsol_options.highs.write_model_to_file = false; % [Default: 'false'], [Allowed: boolean]
        opts.qpsol_options.highs.mip_detect_symmetry = true; % [Default: 'true'], [Allowed: boolean]
        opts.qpsol_options.highs.mip_allow_restart = true; % [Default: 'true'], [Allowed: boolean]
        opts.qpsol_options.highs.mip_max_nodes = 2147483647; % [Default: 2147483647], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.mip_max_stall_nodes = 2147483647; % [Default: 2147483647], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.mip_improving_solution_save = false; % [Default: 'false'], [Allowed: boolean]
        opts.qpsol_options.highs.mip_improving_solution_report_sparse = false; % [Default: 'false'], [Allowed: boolean]
        opts.qpsol_options.highs.mip_improving_solution_file = ''; % [Default: ''], [Allowed: string]
        opts.qpsol_options.highs.mip_max_leaves = 2147483647; % [Default: 2147483647], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.mip_max_improving_sols = 2147483647; % [Default: 2147483647], [Allowed: {1, 2147483647}]
        opts.qpsol_options.highs.mip_lp_age_limit = 10; % [Default: 10], [Allowed: {0, 32767}]
        opts.qpsol_options.highs.mip_pool_age_limit = 30; % [Default: 30], [Allowed: {0, 1000}]
        opts.qpsol_options.highs.mip_pool_soft_limit = 10000; % [Default: 10000], [Allowed: {1, 2147483647}]
        opts.qpsol_options.highs.mip_pscost_minreliable = 8; % [Default: 8], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.mip_min_cliquetable_entries_for_parallelism = 100000; % [Default: 100000], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.mip_feasibility_tolerance = 1e-06; % [Default: 1e-06], [Allowed: [1e-10, inf]]
        opts.qpsol_options.highs.mip_heuristic_effort = 0.05; % [Default: 0.05], [Allowed: [0, 1]]
        opts.qpsol_options.highs.mip_rel_gap = 0.0001; % [Default: 0.0001], [Allowed: [0, inf]]
        opts.qpsol_options.highs.mip_abs_gap = 1e-06; % [Default: 1e-06], [Allowed: [0, inf]]
        opts.qpsol_options.highs.mip_min_logging_interval = 5; % [Default: 5], [Allowed: [0, inf]]
        opts.qpsol_options.highs.ipm_iteration_limit = 2147483647; % [Default: 2147483647], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.pdlp_native_termination = false; % [Default: 'false'], [Allowed: boolean]
        opts.qpsol_options.highs.pdlp_scaling = true; % [Default: 'true'], [Allowed: boolean]
        opts.qpsol_options.highs.pdlp_iteration_limit = 2147483647; % [Default: 2147483647], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.pdlp_e_restart_method = 1; % [Default: 1], [Allowed: {0, 2}]
        opts.qpsol_options.highs.pdlp_d_gap_tol = 0.0001; % [Default: 0.0001], [Allowed: [1e-12, inf]]
        opts.qpsol_options.highs.qp_iteration_limit = 100; % [Default: 2147483647], [Allowed: {0, 2147483647}]
        opts.qpsol_options.highs.qp_nullspace_limit = 4000; % [Default: 4000], [Allowed: {0, 2147483647}]
    elseif(strcmp(MPC_solver, 'test'))
        % https://darnstrom.github.io/daqp/parameters/
        opts.qpsol = 'gurobi'; % Set the QP solver to 'qrqp'
           % Print iterations [Default: true], [Allowed: true/false]
        opts.qpsol_options.verbose = false; % Print output 
        opts.qpsol_options.error_on_fail = false; % Throw exceptions when function evaluation fails
        opts.error_on_fail=false;
        
    % elseif(strcmp(MPC_solver, 'ooqp'))
    %     .../casadi/core/function_internal.cpp:146: Error calling Sqpmethod::init for 'solver':
    %     .../casadi/core/plugin_interface.hpp:292: Plugin 'ooqp' is not found.
    %     % DOKU: https://github.com/casadi/casadi/blob/main/casadi/interfaces/ooqp/ooqp_interface.cpp
    %     opts.qpsol = 'ooqp'; % Set the QP solver to 'ooqp'
    %     opts.qpsol_options.print_iter = false; % Disable printing of QP solver iterations
    %     opts.qpsol_options.print_header = false; % Disable printing of QP solver header
    %     opts.qpsol_options.print_info = false; % Disable printing of QP solver info
    %     opts.qpsol_options.error_on_fail = false;
    %     opts.qpsol_options.print_level = 0; % Print level
    %     opts.qpsol_options.mutol = 1e-08; % Tolerance for setting MuTol
    %     opts.qpsol_options.artol = 1e-08; % Tolerance for setting ArTol
    end
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
    
    % General options to control the solver behavior
    opts.print_time = false; % Enable printing solver execution time for debugging
    opts.error_on_fail = false; % Throw exceptions when function evaluation fails
    opts.verbose = false; % Verbose output to help with debugging
    opts.show_eval_warnings = false; % Show evaluation warnings
    opts.regularity_check = true; % Throw exceptions when NaN or Inf appears during evaluation
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
    opts.qpsol_options = struct;
    opts.qpsol = 'qrqp'; % Set the QP solver to 'qrqp'
    opts.qpsol_options.max_iter = 100;                % Maximum number of iterations [Default: 1000], [Allowed: > 0]
    opts.qpsol_options.constr_viol_tol = 1e-8;       % Constraint violation tolerance [Default: 1e-08], [Allowed: > 0]
    opts.qpsol_options.dual_inf_tol = 1e-8;          % Dual feasibility violation tolerance [Default: 1e-08], [Allowed: > 0]
    opts.qpsol_options.print_header = false;            % Print header [Default: true], [Allowed: true/false]
    opts.qpsol_options.print_iter = false;              % Print iterations [Default: true], [Allowed: true/false]
    opts.qpsol_options.print_info = false;              % Print info [Default: true], [Allowed: true/false]
    opts.qpsol_options.print_lincomb = false;          % Print dependent linear combinations of constraints [Default: false], [Allowed: true/false]
    opts.qpsol_options.min_lam = 0;      
    opts.qpsol_options.error_on_fail = false;

    opts.print_header = false; % Disable printing of solver header
    opts.print_iteration = false; % Disable printing of solver iterations
    opts.print_time = false; % Disable printing of solver time
    opts.print_status = false;
    opts.error_on_fail = false;
    opts.verbose = false;

    opts.print_time = false; % Disable printing of solver time
    opts.verbose = false; % Disable verbose output
    opts.verbose_init = false; % Disable verbose output
    opts.print_in = false; % Set the level of printing (-1 for no printing, 0 for normal, 1 for verbose)
    opts.print_out = false; % % Set the level of printing (-1 for no printing, 0 for normal, 1 for verbose)
    opts.record_time = false; % Record the time spent in the solver
    opts.iteration_callback_ignore_errors = false; % Ignore errors in the iteration callback
    opts.warn_initial_bounds = false; % Warn if initial bounds are not satisfied
    
    opts.solve_type = 'SQP'; % The solver type: Either SQP or SLP. [Default: SQP], [Allowed: 'SQP', 'SLP']
    opts.hessian_approximation = 'exact'; % Hessian approximation type [Default: 'exact'], [Allowed: 'limited-memory', 'exact']
    opts.max_iter = 1000; % Maximum number of SQP iterations [Default: 1000], [Allowed: > 0]
    opts.min_iter = 0; % Minimum number of SQP iterations [Default: 0], [Allowed: >= 0]
    opts.tol_pr = 1e-10; % Stopping criterion for primal infeasibility [Default: 1e-6], [Allowed: > 0]
    opts.tol_du = 1e-10; % Stopping criterion for dual infeasibility [Default: 1e-6], [Allowed: > 0]
    opts.merit_memory = 10; % Size of memory for merit function values [Default: 10], [Allowed: >= 0]
    opts.lbfgs_memory = 10; % Size of L-BFGS memory [Default: 10], [Allowed: >= 0]
    opts.print_header = false; % Print the header with problem statistics [Default: true], [Allowed: true, false]
    opts.print_iteration = false; % Print the iterations [Default: true], [Allowed: true, false]
    opts.print_status = false; % Print a status message after solving [Default: true], [Allowed: true, false]
    opts.convexify_strategy = 'regularize'; % Strategy to convexify the Lagrange Hessian [Default: 'none'], [Allowed: 'none', 'regularize', 'eigen-reflect', 'eigen-clip']
    opts.convexify_margin = 1e-7; % Minimum eigenvalue when using a convexification strategy [Default: 1e-7], [Allowed: any non-negative number]
    opts.max_iter_eig = 50; % Maximum number of iterations to compute an eigenvalue decomposition [Default: 50], [Allowed: > 0]
    opts.init_feasible = true; % Initialize the QP subproblems with a feasible initial value [Default: false], [Allowed: true, false]
    opts.optim_tol = 1e-8; % Optimality tolerance [Default: 1e-8], [Allowed: > 0]
    opts.feas_tol = 1e-8; % Feasibility tolerance [Default: 1e-8], [Allowed: > 0]
    opts.tr_rad0 = 1.0; % Initial trust-region radius [Default: 1.0], [Allowed: > 0]
    opts.tr_eta1 = 0.5; % Lower eta in trust-region acceptance criterion [Default: 0.5], [Allowed: > 0]
    opts.tr_eta2 = 0.75; % Upper eta in trust-region acceptance criterion [Default: 0.75], [Allowed: > 0]
    opts.tr_alpha1 = 0.1; % Lower alpha in trust-region size criterion [Default: 0.1], [Allowed: > 0]
    opts.tr_alpha2 = 2.0; % Upper alpha in trust-region size criterion [Default: 2.0], [Allowed: > 0]
    opts.tr_tol = 1e-8; % Trust-region tolerance [Default: 1e-8], [Allowed: > 0]
    opts.tr_acceptance = 0.9; % Proportion to accept trust-region step [Default: 0.9], [Allowed: > 0]
    opts.tr_rad_min = 1e-10; % Minimum trust-region radius [Default: 1e-10], [Allowed: >= 0]
    opts.tr_rad_max = 10.0; % Maximum trust-region radius [Default: 10.0], [Allowed: > 0]
    
    trust_u = repmat(exp(linspace(0, -1, size(u, 2))), size(u, 1), 1);
    trust_x = repmat(exp(linspace(0, -1, size(x, 2))), size(x, 1), 1);
    
    % opts.tr_scale_vector = [trust_u(:); trust_x(:)]; % Vector to indicate where trust-region is applied [Default: 1], [Allowed: any numeric vector of size nx_]
    opts.contraction_acceptance_value = 0.5; % Empirical contraction rate acceptance for feasibility iterations [Default: 0.5], [Allowed: any value]
    opts.watchdog = 5; % Number of watchdog iterations in feasibility iterations [Default: 5], [Allowed: > 0]
    opts.max_inner_iter = 50; % Maximum number of inner iterations [Default: 50], [Allowed: > 0]
    opts.use_anderson = false; % Use Anderson Acceleration [Default: false], [Allowed: true, false]
    opts.anderson_memory = 1; % Anderson memory if used [Default: 1], [Allowed: >= 0]
    solver = nlpsol('solver', 'feasiblesqpmethod', prob, opts);
    % solver.print_options();
elseif(strcmp(MPC_solver, 'qpoases'))
    % DOKU: https://casadi.sourceforge.net/api/internal/d5/d43/classcasadi_1_1QpoasesInterface.html
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
    %opts.ipopt.print_level = 5;
    %opts.ipopt.fast_step_computation = 'yes';
    %opts.ipopt.print_info_string = 'yes';
    % opts.jit = true;
    % opts.compiler = 'shell';
    % opts.jit_options.flags = {'-O3'};
    % opts.jit_options.verbose = true;
    solver = nlpsol('solver', 'ipopt', prob, opts);
elseif(strcmp(MPC_solver, 'snopt'))
    % DOKU: https://casadi.sourceforge.net/v2.0.0/api/html/d6/d07/classcasadi_1_1NlpSolver.html#plugin_NlpSolver_ipopt
    opts.show_eval_warnings = false;
    opts.error_on_fail = false;
    opts.print_time = 0;

    %opts.ipopt.print_level = 5;
    %opts.ipopt.fast_step_computation = 'yes';
    %opts.ipopt.print_info_string = 'yes';
    % opts.jit = true;
    % opts.compiler = 'shell';
    % opts.jit_options.flags = {'-O3'};
    % opts.jit_options.verbose = true;
    solver = nlpsol('solver', 'snopt', prob, opts);
else
    error(['invalid MPC solver=', MPC_solver, ...
           ' is not a valid solver for nlpsol (only "qrqp", "ipopt", "qpoases", "fatrop" ... supported for nlpsol)']);
end
disp(['Execution time of solver = nlpsol(''solver'', ''', MPC_solver, ''', prob, opts): ', num2str(toc), ' s']);
fprintf('\n');

% solver.print_options();