% https://coin-or.github.io/Ipopt/OPTIONS.html
% https://casadi.sourceforge.net/v2.0.0/api/html/d6/d07/classcasadi_1_1NlpSolver.html#plugin_NlpSolver_ipopt

% Desired convergence tolerance (relative)
opts.ipopt.tol = 1e-8;  % Default: 1e-8

% Scaling threshold for the NLP error (advanced)
% opts.ipopt.s_max = 100;  % Default: 100

% Maximum number of iterations
opts.ipopt.max_iter = 3000;  % Default: 3000

% Maximum walltime clock seconds
opts.ipopt.max_wall_time = 10+20;  % Default: 10+20

% Maximum CPU seconds
opts.ipopt.max_cpu_time = 10+20;  % Default: 10+20

% Desired threshold for the dual infeasibility
opts.ipopt.dual_inf_tol = 1.0;  % Default: 1.0

% Desired threshold for constraint and variable bound violation
opts.ipopt.constr_viol_tol = 1e-4;  % Default: 1e-4

% Desired threshold for the complementarity conditions
opts.ipopt.compl_inf_tol = 1e-4;  % Default: 1e-4

% ----- Acceptable Tolerance Options -----

% "Acceptable" convergence tolerance (relative)
opts.ipopt.acceptable_tol = 1e-6;  % Default: 1e-6

% Number of "acceptable" iterates before triggering termination
opts.ipopt.acceptable_iter = 15;  % Default: 15

% "Acceptance" threshold for the dual infeasibility
opts.ipopt.acceptable_dual_inf_tol = 1e+10;  % Default: 1e+10

% "Acceptance" threshold for the constraint violation
opts.ipopt.acceptable_constr_viol_tol = 0.01;  % Default: 0.01

% "Acceptance" threshold for the complementarity conditions
opts.ipopt.acceptable_compl_inf_tol = 0.01;  % Default: 0.01

% "Acceptance" stopping criterion based on objective function change
%opts.ipopt.acceptable_obj_change_tol = 1e+20;  % Default: 1e+20

% Threshold for maximal value of primal iterates
%opts.ipopt.diverging_iterates_tol = 1e+20;  % Default: 1e+20

% Desired value of complementarity
opts.ipopt.mu_target = 0.0;  % Default: 0.0

% ----- Output Options -----

% Output verbosity level
opts.ipopt.print_level = 0;  % Default: 5

% File name of desired output file (leave unset for no file output)
opts.ipopt.output_file = '';  % Default: ''

% Verbosity level for output file
% opts.ipopt.file_print_level = 5;  % Default: 5

% Whether to append to output file
% opts.ipopt.file_append = 'no';  % Default: 'no'

% Print all options set by the user
opts.ipopt.print_user_options = 'no';  % Default: 'no'

% Switch to print all algorithmic options with documentation
opts.ipopt.print_options_documentation = 'no';  % Default: 'no'

% Switch to print timing statistics
opts.ipopt.print_timing_statistics = 'no';  % Default: 'no'

% format in which to print options documentation
% opts.ipopt.print_options_mode = 'text';  % Default: 'text'

% whether to print also advanced options (advanced)
% opts.ipopt.print_advanced_options = 'no';  % Default: 'no'

% Enables printing of additional info string at end of iteration output
% opts.ipopt.print_info_string = 'no';  % Default: 'no'

% Determines what value is printed in the "inf_pr" output column
opts.ipopt.inf_pr_output = 'original';  % Default: 'original'

% Determines at which iteration frequency to print output
opts.ipopt.print_frequency_iter = 1;  % Default: 1

% Determines at which time frequency to print output
opts.ipopt.print_frequency_time = 0;  %