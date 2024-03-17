# MA24_simulink

# MA24_simulink

<details>
 <summary>./casadi/casadi/solvers/linsol_ldl.cpp: const Options LinsolLdl</summary>
  <table><tr><th>= {{&ProtoFunction::options_},</td></th>
  <tr><td>   {{"incomplete",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>     "Incomplete factorization, without any fill-in"}},</td></tr>
  <tr><td>    {"preordering",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>     "Approximate minimal degree (AMD) preordering"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/linear_interpolant.cpp: const Options LinearInterpolant</summary>
  <table><tr><th>= {{&Interpolant::options_},</td></th>
  <tr><td>   {{"lookup_mode",</td></tr>
  <tr><td>     {OT_STRINGVECTOR,</td></tr>
  <tr><td>      "Sets, for each grid dimenion, the lookup algorithm used to find the correct index. "</td></tr>
  <tr><td>      "'linear' uses a for-loop + break; "</td></tr>
  <tr><td>      "'exact' uses floored division (only for uniform grids)."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/ipqp.cpp: const Options Ipqp</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"max_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of iterations [1000]."}},</td></tr>
  <tr><td>    {"constr_viol_tol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Constraint violation tolerance [1e-8]."}},</td></tr>
  <tr><td>    {"dual_inf_tol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Dual feasibility violation tolerance [1e-8]"}},</td></tr>
  <tr><td>    {"print_header",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print header [true]."}},</td></tr>
  <tr><td>    {"print_iter",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print iterations [true]."}},</td></tr>
  <tr><td>    {"print_info",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print info [true]."}},</td></tr>
  <tr><td>    {"linear_solver",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "A custom linear solver creator function [default: ldl]"}},</td></tr>
  <tr><td>    {"linear_solver_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to the linear solver"}},</td></tr>
  <tr><td>    {"min_lam",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Smallest multiplier treated as inactive for the initial active set [0]."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/qrsqp.cpp: const Options Qrsqp</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"qpsol",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "The QP solver to be used by the SQP method [qrqp]"}},</td></tr>
  <tr><td>    {"qpsol_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to the QP solver"}},</td></tr>
  <tr><td>    {"hessian_approximation",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "limited-memory|exact"}},</td></tr>
  <tr><td>    {"max_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of SQP iterations"}},</td></tr>
  <tr><td>    {"min_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Minimum number of SQP iterations"}},</td></tr>
  <tr><td>    {"max_iter_ls",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of linesearch iterations"}},</td></tr>
  <tr><td>    {"tol_pr",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion for primal infeasibility"}},</td></tr>
  <tr><td>    {"tol_du",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion for dual infeasability"}},</td></tr>
  <tr><td>    {"c1",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Armijo condition, coefficient of decrease in merit"}},</td></tr>
  <tr><td>    {"beta",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Line-search parameter, restoration factor of stepsize"}},</td></tr>
  <tr><td>    {"merit_memory",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Size of memory to store history of merit function values"}},</td></tr>
  <tr><td>    {"lbfgs_memory",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Size of L-BFGS memory."}},</td></tr>
  <tr><td>    {"regularize",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Automatic regularization of Lagrange Hessian."}},</td></tr>
  <tr><td>    {"print_header",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print the header with problem statistics"}},</td></tr>
  <tr><td>    {"print_iteration",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print the iterations"}},</td></tr>
  <tr><td>    {"min_step_size",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "The size (inf-norm) of the step size should not become smaller than this."}},</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/scpgen.cpp: const Options Scpgen</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"qpsol",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "The QP solver to be used by the SQP method"}},</td></tr>
  <tr><td>    {"qpsol_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to the QP solver"}},</td></tr>
  <tr><td>    {"hessian_approximation",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "gauss-newton|exact"}},</td></tr>
  <tr><td>    {"max_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of SQP iterations"}},</td></tr>
  <tr><td>    {"max_iter_ls",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of linesearch iterations"}},</td></tr>
  <tr><td>    {"tol_pr",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion for primal infeasibility"}},</td></tr>
  <tr><td>    {"tol_du",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion for dual infeasability"}},</td></tr>
  <tr><td>    {"tol_reg",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion for regularization"}},</td></tr>
  <tr><td>    {"tol_pr_step",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion for the step size"}},</td></tr>
  <tr><td>    {"c1",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Armijo condition, coefficient of decrease in merit"}},</td></tr>
  <tr><td>    {"beta",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Line-search parameter, restoration factor of stepsize"}},</td></tr>
  <tr><td>    {"merit_memsize",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Size of memory to store history of merit function values"}},</td></tr>
  <tr><td>    {"merit_start",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Lower bound for the merit function parameter"}},</td></tr>
  <tr><td>    {"lbfgs_memory",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Size of L-BFGS memory."}},</td></tr>
  <tr><td>    {"regularize",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Automatic regularization of Lagrange Hessian."}},</td></tr>
  <tr><td>    {"print_header",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print the header with problem statistics"}},</td></tr>
  <tr><td>    {"codegen",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "C-code generation"}},</td></tr>
  <tr><td>    {"reg_threshold",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Threshold for the regularization."}},</td></tr>
  <tr><td>    {"name_x",</td></tr>
  <tr><td>     {OT_STRINGVECTOR,</td></tr>
  <tr><td>      "Names of the variables."}},</td></tr>
  <tr><td>    {"print_x",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Which variables to print."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/linsol_qr.cpp: const Options LinsolQr</summary>
  <table><tr><th>= {{&LinsolInternal::options_},</td></th>
  <tr><td>   {{"eps",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Minimum R entry before singularity is declared [1e-12]"}},</td></tr>
  <tr><td>    {"cache",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Amount of factorisations to remember (thread-local) [0]"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/newton.cpp: const Options Newton</summary>
  <table><tr><th>= {{&Rootfinder::options_},</td></th>
  <tr><td>   {{"abstol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion tolerance on max(|F|)"}},</td></tr>
  <tr><td>    {"abstolStep",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion tolerance on step size"}},</td></tr>
  <tr><td>    {"max_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of Newton iterations to perform before returning."}},</td></tr>
  <tr><td>    {"print_iteration",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print information about each iteration"}},</td></tr>
  <tr><td>    {"line_search",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Enable line-search (default: true)"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/sqpmethod.cpp: const Options Sqpmethod</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"qpsol",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>    "The QP solver to be used by the SQP method [qpoases]"}},</td></tr>
  <tr><td>  {"qpsol_options",</td></tr>
  <tr><td>    {OT_DICT,</td></tr>
  <tr><td>    "Options to be passed to the QP solver"}},</td></tr>
  <tr><td>  {"hessian_approximation",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>    "limited-memory|exact"}},</td></tr>
  <tr><td>  {"max_iter",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Maximum number of SQP iterations"}},</td></tr>
  <tr><td>  {"min_iter",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Minimum number of SQP iterations"}},</td></tr>
  <tr><td>  {"max_iter_ls",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Maximum number of linesearch iterations"}},</td></tr>
  <tr><td>  {"tol_pr",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Stopping criterion for primal infeasibility"}},</td></tr>
  <tr><td>  {"tol_du",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Stopping criterion for dual infeasability"}},</td></tr>
  <tr><td>  {"c1",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Armijo condition, coefficient of decrease in merit"}},</td></tr>
  <tr><td>  {"beta",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Line-search parameter, restoration factor of stepsize"}},</td></tr>
  <tr><td>  {"merit_memory",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Size of memory to store history of merit function values"}},</td></tr>
  <tr><td>  {"lbfgs_memory",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Size of L-BFGS memory."}},</td></tr>
  <tr><td>  {"print_header",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Print the header with problem statistics"}},</td></tr>
  <tr><td>  {"print_iteration",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Print the iterations"}},</td></tr>
  <tr><td>  {"print_status",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Print a status message after solving"}},</td></tr>
  <tr><td>  {"min_step_size",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "The size (inf-norm) of the step size should not become smaller than this."}},</td></tr>
  <tr><td>  {"hess_lag",</td></tr>
  <tr><td>    {OT_FUNCTION,</td></tr>
  <tr><td>    "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</td></tr>
  <tr><td>  {"jac_fg",</td></tr>
  <tr><td>    {OT_FUNCTION,</td></tr>
  <tr><td>    "Function for calculating the gradient of the objective and Jacobian of the constraints "</td></tr>
  <tr><td>    "(autogenerated by default)"}},</td></tr>
  <tr><td>  {"convexify_strategy",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>    "NONE|regularize|eigen-reflect|eigen-clip. "</td></tr>
  <tr><td>    "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</td></tr>
  <tr><td>  {"convexify_margin",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "When using a convexification strategy, make sure that "</td></tr>
  <tr><td>    "the smallest eigenvalue is at least this (default: 1e-7)."}},</td></tr>
  <tr><td>  {"max_iter_eig",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</td></tr>
  <tr><td>  {"elastic_mode",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Enable the elastic mode which is used when the QP is infeasible (default: false)."}},</td></tr>
  <tr><td>  {"gamma_0",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Starting value for the penalty parameter of elastic mode (default: 1)."}},</td></tr>
  <tr><td>  {"gamma_max",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Maximum value for the penalty parameter of elastic mode (default: 1e20)."}},</td></tr>
  <tr><td>  {"gamma_1_min",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Minimum value for gamma_1 (default: 1e-5)."}},</td></tr>
  <tr><td>  {"second_order_corrections",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Enable second order corrections. "</td></tr>
  <tr><td>    "These are used when a step is considered bad by the merit function and constraint norm "</td></tr>
  <tr><td>    "(default: false)."}},</td></tr>
  <tr><td>  {"init_feasible",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Initialize the QP subproblems with a feasible initial value (default: false)."}}</td></tr>
  <tr><td>  }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/fast_newton.cpp: const Options FastNewton</summary>
  <table><tr><th>= {{&Rootfinder::options_},</td></th>
  <tr><td>   {{"abstol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion tolerance on ||g||__inf)"}},</td></tr>
  <tr><td>    {"abstolStep",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion tolerance on step size"}},</td></tr>
  <tr><td>    {"max_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of Newton iterations to perform before returning."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/shell_compiler.cpp: const Options ShellCompiler</summary>
  <table><tr><th>= {{&ImporterInternal::options_},</td></th>
  <tr><td>   {{"compiler",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Compiler command"}},</td></tr>
  <tr><td>    {"linker",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Linker command"}},</td></tr>
  <tr><td>    {"directory",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Directory to put temporary objects in. Must end with a file separator."}},</td></tr>
  <tr><td>    {"compiler_setup",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Compiler setup command. Intended to be fixed."</td></tr>
  <tr><td>      " The 'flag' option is the prefered way to set"</td></tr>
  <tr><td>      " custom flags."}},</td></tr>
  <tr><td>    {"linker_setup",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Linker setup command. Intended to be fixed."</td></tr>
  <tr><td>      " The 'flag' option is the prefered way to set"</td></tr>
  <tr><td>      " custom flags."}},</td></tr>
  <tr><td>    {"compiler_flags",</td></tr>
  <tr><td>     {OT_STRINGVECTOR,</td></tr>
  <tr><td>      "Alias for 'compiler_flags'"}},</td></tr>
  <tr><td>    {"flags",</td></tr>
  <tr><td>      {OT_STRINGVECTOR,</td></tr>
  <tr><td>      "Compile flags for the JIT compiler. Default: None"}},</td></tr>
  <tr><td>    {"linker_flags",</td></tr>
  <tr><td>     {OT_STRINGVECTOR,</td></tr>
  <tr><td>      "Linker flags for the JIT compiler. Default: None"}},</td></tr>
  <tr><td>    {"cleanup",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Cleanup temporary files when unloading. Default: true"}},</td></tr>
  <tr><td>    {"compiler_output_flag",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>     "Compiler flag to denote object output. Default: '-o '"}},</td></tr>
  <tr><td>    {"linker_output_flag",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>     "Linker flag to denote shared library output. Default: '-o '"}},</td></tr>
  <tr><td>    {"extra_suffixes",</td></tr>
  <tr><td>     {OT_STRINGVECTOR,</td></tr>
  <tr><td>     "List of suffixes for extra files that the compiler may generate. Default: None"}},</td></tr>
  <tr><td>    {"name",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "The file name used to write out compiled objects/libraries. "</td></tr>
  <tr><td>      "The actual file names used depend on 'temp_suffix' and include extensions. "</td></tr>
  <tr><td>      "Default: 'tmp_casadi_compiler_shell'"}},</td></tr>
  <tr><td>    {"temp_suffix",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use a temporary (seemingly random) filename suffix for file names. "</td></tr>
  <tr><td>      "This is desired for thread-safety. "</td></tr>
  <tr><td>      "This behaviour may defeat caching compiler wrappers. "</td></tr>
  <tr><td>      "Default: true"}},</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/implicit_to_nlp.cpp: const Options ImplicitToNlp</summary>
  <table><tr><th>= {{&Rootfinder::options_},</td></th>
  <tr><td>   {{"nlpsol",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Name of solver."}},</td></tr>
  <tr><td>    {"nlpsol_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to solver."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/qrqp.cpp: const Options Qrqp</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"max_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of iterations [1000]."}},</td></tr>
  <tr><td>    {"constr_viol_tol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Constraint violation tolerance [1e-8]."}},</td></tr>
  <tr><td>    {"dual_inf_tol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Dual feasibility violation tolerance [1e-8]"}},</td></tr>
  <tr><td>    {"print_header",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print header [true]."}},</td></tr>
  <tr><td>    {"print_iter",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print iterations [true]."}},</td></tr>
  <tr><td>    {"print_info",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print info [true]."}},</td></tr>
  <tr><td>    {"print_lincomb",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print dependant linear combinations of constraints [false]. "</td></tr>
  <tr><td>      "Printed numbers are 0-based indices into the vector of [simple bounds;linear bounds]"}},</td></tr>
  <tr><td>    {"min_lam",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Smallest multiplier treated as inactive for the initial active set [0]."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/feasiblesqpmethod.cpp: const Options Feasiblesqpmethod</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"solve_type",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "The solver type: Either SQP or SLP. Defaults to SQP"}},</td></tr>
  <tr><td>    {"qpsol",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "The QP solver to be used by the SQP method [qpoases]"}},</td></tr>
  <tr><td>    {"qpsol_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to the QP solver"}},</td></tr>
  <tr><td>    {"hessian_approximation",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "limited-memory|exact"}},</td></tr>
  <tr><td>    {"max_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of SQP iterations"}},</td></tr>
  <tr><td>    {"min_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Minimum number of SQP iterations"}},</td></tr>
  <tr><td>    {"tol_pr",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion for primal infeasibility"}},</td></tr>
  <tr><td>    {"tol_du",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion for dual infeasability"}},</td></tr>
  <tr><td>    {"merit_memory",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Size of memory to store history of merit function values"}},</td></tr>
  <tr><td>    {"lbfgs_memory",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Size of L-BFGS memory."}},</td></tr>
  <tr><td>    {"print_header",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print the header with problem statistics"}},</td></tr>
  <tr><td>    {"print_iteration",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print the iterations"}},</td></tr>
  <tr><td>    {"print_status",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print a status message after solving"}},</td></tr>
  <tr><td>    {"f",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the objective function (autogenerated by default)"}},</td></tr>
  <tr><td>    {"g",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the constraints (autogenerated by default)"}},</td></tr>
  <tr><td>    {"grad_f",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the gradient of the objective (autogenerated by default)"}},</td></tr>
  <tr><td>    {"jac_g",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the Jacobian of the constraints (autogenerated by default)"}},</td></tr>
  <tr><td>    {"hess_lag",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</td></tr>
  <tr><td>    {"convexify_strategy",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "NONE|regularize|eigen-reflect|eigen-clip. "</td></tr>
  <tr><td>      "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</td></tr>
  <tr><td>    {"convexify_margin",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "When using a convexification strategy, make sure that "</td></tr>
  <tr><td>      "the smallest eigenvalue4 is at least this (default: 1e-7)."}},</td></tr>
  <tr><td>    {"max_iter_eig",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</td></tr>
  <tr><td>    {"init_feasible",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Initialize the QP subproblems with a feasible initial value (default: false)."}},</td></tr>
  <tr><td>    {"optim_tol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Optimality tolerance. Below this value an iterate is considered to be optimal."}},</td></tr>
  <tr><td>    {"feas_tol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Feasibility tolerance. Below this tolerance an iterate is considered to be feasible."}},</td></tr>
  <tr><td>    {"tr_rad0",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Initial trust-region radius."}},</td></tr>
  <tr><td>    {"tr_eta1",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Lower eta in trust-region acceptance criterion."}},</td></tr>
  <tr><td>    {"tr_eta2",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Upper eta in trust-region acceptance criterion."}},</td></tr>
  <tr><td>    {"tr_alpha1",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Lower alpha in trust-region size criterion."}},</td></tr>
  <tr><td>    {"tr_alpha2",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Upper alpha in trust-region size criterion."}},</td></tr>
  <tr><td>    {"tr_tol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Trust-region tolerance. "</td></tr>
  <tr><td>      "Below this value another scalar is equal to the trust region radius."}},</td></tr>
  <tr><td>    {"tr_acceptance",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Is the trust-region ratio above this value, the step is accepted."}},</td></tr>
  <tr><td>    {"tr_rad_min",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Minimum trust-region radius."}},</td></tr>
  <tr><td>    {"tr_rad_max",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Maximum trust-region radius."}},</td></tr>
  <tr><td>    {"tr_scale_vector",</td></tr>
  <tr><td>     {OT_DOUBLEVECTOR,</td></tr>
  <tr><td>      "Vector that tells where trust-region is applied."}},</td></tr>
  <tr><td>    {"contraction_acceptance_value",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "If the empirical contraction rate in the feasibility iterations "</td></tr>
  <tr><td>      "is above this value in the heuristics the iterations are aborted."}},</td></tr>
  <tr><td>    {"watchdog",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Number of watchdog iterations in feasibility iterations. "</td></tr>
  <tr><td>      "After this amount of iterations, it is checked with the contraction acceptance value, "</td></tr>
  <tr><td>      "if iterations are converging."}},</td></tr>
  <tr><td>    {"max_inner_iter",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Maximum number of inner iterations."}},</td></tr>
  <tr><td>    {"use_anderson",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use Anderson Acceleration. (default false)"}},</td></tr>
  <tr><td>    {"anderson_memory",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Anderson memory. If Anderson is used default is 1, else default is 0."}},</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/collocation.cpp: const Options Collocation</summary>
  <table><tr><th>= {{&ImplicitFixedStepIntegrator::options_},</td></th>
  <tr><td>   {{"interpolation_order",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Order of the interpolating polynomials"}},</td></tr>
  <tr><td>    {"collocation_scheme",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Collocation scheme: radau|legendre"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/symbolic_qr.cpp: const Options SymbolicQr</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"fopts",</td></tr>
  <tr><td>    {OT_DICT,</td></tr>
  <tr><td>     "Options to be passed to generated function objects"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/bspline_interpolant.cpp: const Options BSplineInterpolant</summary>
  <table><tr><th>= {{&Interpolant::options_},</td></th>
  <tr><td>   {{"degree",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Sets, for each grid dimension, the degree of the spline."}},</td></tr>
  <tr><td>     {"linear_solver",</td></tr>
  <tr><td>      {OT_STRING,</td></tr>
  <tr><td>       "Solver used for constructing the coefficient tensor."}},</td></tr>
  <tr><td>     {"linear_solver_options",</td></tr>
  <tr><td>      {OT_DICT,</td></tr>
  <tr><td>       "Options to be passed to the linear solver."}},</td></tr>
  <tr><td>     {"algorithm",</td></tr>
  <tr><td>      {OT_STRING,</td></tr>
  <tr><td>       "Algorithm used for fitting the data: 'not_a_knot' (default, same as Matlab),"</td></tr>
  <tr><td>      " 'smooth_linear'."}},</td></tr>
  <tr><td>     {"smooth_linear_frac",</td></tr>
  <tr><td>      {OT_DOUBLE,</td></tr>
  <tr><td>       "When 'smooth_linear' algorithm is active, determines sharpness between"</td></tr>
  <tr><td>       " 0 (sharp, as linear interpolation) and 0.5 (smooth)."</td></tr>
  <tr><td>       "Default value is 0.1."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/qp_to_nlp.cpp: const Options QpToNlp</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"nlpsol",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Name of solver."}},</td></tr>
  <tr><td>    {"nlpsol_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to solver."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/hpipm/hpipm_interface.cpp: const Options HpipmInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"N",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "OCP horizon"}},</td></tr>
  <tr><td>    {"nx",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Number of states, length N+1"}},</td></tr>
  <tr><td>    {"nu",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Number of controls, length N"}},</td></tr>
  <tr><td>    {"ng",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Number of non-dynamic constraints, length N+1"}},</td></tr>
  <tr><td>    {"inf",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Replace infinities by this amount [default: 1e8]"}},</td></tr>
  <tr><td>    {"hpipm",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to hpipm"}}}</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/lapack/lapack_lu.cpp: const Options LapackLu</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"equilibration",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Equilibrate the matrix"}},</td></tr>
  <tr><td>    {"allow_equilibration_failure",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Non-fatal error when equilibration fails"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/lapack/lapack_qr.cpp: const Options LapackQr</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"max_nrhs",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of right-hand-sides that get processed in a single pass [default:10]."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/highs/highs_interface.cpp: const Options HighsInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"highs",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to HiGHS."</td></tr>
  <tr><td>      }},</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/alpaqa/alpaqa_interface.cpp: const Options AlpaqaInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"alpaqa",</td></tr>
  <tr><td>      {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to Alpaqa"}}</td></tr>
  <tr><td>    }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/mumps/mumps_interface.cpp: const Options MumpsInterface</summary>
  <table><tr><th>= {{&ProtoFunction::options_},</td></th>
  <tr><td>   {{"symmetric",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>     "Symmetric matrix"}},</td></tr>
  <tr><td>    {"posdef",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>     "Positive definite"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/daqp/daqp_interface.cpp: const Options DaqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"daqp",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to Daqp."</td></tr>
  <tr><td>      }},</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/worhp/worhp_interface.cpp: const Options WorhpInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"worhp",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to WORHP"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/cvodes_interface.cpp: const Options CvodesInterface</summary>
  <table><tr><th>= {{&SundialsInterface::options_},</td></th>
  <tr><td>   {{"linear_multistep_method",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>    "Integrator scheme: BDF|adams"}},</td></tr>
  <tr><td>  {"nonlinear_solver_iteration",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>    "Nonlinear solver type: NEWTON|functional"}},</td></tr>
  <tr><td>  {"min_step_size",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Min step size [default: 0/0.0]"}},</td></tr>
  <tr><td>  {"fsens_all_at_once",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Calculate all right hand sides of the sensitivity equations at once"}},</td></tr>
  <tr><td>  {"always_recalculate_jacobian",</td></tr>
  <tr><td>   {OT_BOOL,</td></tr>
  <tr><td>    "Recalculate Jacobian before factorizations, even if Jacobian is current [default: true]"}}</td></tr>
  <tr><td>  }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/kinsol_interface.cpp: const Options KinsolInterface</summary>
  <table><tr><th>= {{&Rootfinder::options_},</td></th>
  <tr><td>   {{"max_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of Newton iterations. Putting 0 sets the default value of KinSol."}},</td></tr>
  <tr><td>    {"print_level",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Verbosity level"}},</td></tr>
  <tr><td>    {"abstol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Stopping criterion tolerance"}},</td></tr>
  <tr><td>    {"linear_solver_type",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "dense|banded|iterative|user_defined"}},</td></tr>
  <tr><td>    {"upper_bandwidth",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Upper bandwidth for banded linear solvers"}},</td></tr>
  <tr><td>    {"lower_bandwidth",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Lower bandwidth for banded linear solvers"}},</td></tr>
  <tr><td>    {"max_krylov",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum Krylov space dimension"}},</td></tr>
  <tr><td>    {"exact_jacobian",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use exact Jacobian information"}},</td></tr>
  <tr><td>    {"iterative_solver",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "gmres|bcgstab|tfqmr"}},</td></tr>
  <tr><td>    {"f_scale",</td></tr>
  <tr><td>     {OT_DOUBLEVECTOR,</td></tr>
  <tr><td>      "Equation scaling factors"}},</td></tr>
  <tr><td>    {"u_scale",</td></tr>
  <tr><td>     {OT_DOUBLEVECTOR,</td></tr>
  <tr><td>      "Variable scaling factors"}},</td></tr>
  <tr><td>    {"pretype",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Type of preconditioner"}},</td></tr>
  <tr><td>    {"use_preconditioner",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Precondition an iterative solver"}},</td></tr>
  <tr><td>    {"strategy",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Globalization strategy"}},</td></tr>
  <tr><td>    {"disable_internal_warnings",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Disable KINSOL internal warning messages"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/idas_interface.cpp: const Options IdasInterface</summary>
  <table><tr><th>= {{&SundialsInterface::options_},</td></th>
  <tr><td>   {{"suppress_algebraic",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Suppress algebraic variables in the error testing"}},</td></tr>
  <tr><td>  {"calc_ic",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Use IDACalcIC to get consistent initial conditions."}},</td></tr>
  <tr><td>  {"constraints",</td></tr>
  <tr><td>    {OT_INTVECTOR,</td></tr>
  <tr><td>    "Constrain the solution y=[x,z]. 0 (default): no constraint on yi, "</td></tr>
  <tr><td>    "1: yi >= 0.0, -1: yi <= 0.0, 2: yi > 0.0, -2: yi < 0.0."}},</td></tr>
  <tr><td>  {"calc_icB",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Use IDACalcIC to get consistent initial conditions for "</td></tr>
  <tr><td>    "backwards system [default: equal to calc_ic]."}},</td></tr>
  <tr><td>  {"abstolv",</td></tr>
  <tr><td>    {OT_DOUBLEVECTOR,</td></tr>
  <tr><td>    "Absolute tolerarance for each component"}},</td></tr>
  <tr><td>  {"max_step_size",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Maximim step size"}},</td></tr>
  <tr><td>  {"first_time",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "First requested time as a fraction of the time interval"}},</td></tr>
  <tr><td>  {"cj_scaling",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "IDAS scaling on cj for the user-defined linear solver module"}},</td></tr>
  <tr><td>  {"init_xdot",</td></tr>
  <tr><td>    {OT_DOUBLEVECTOR,</td></tr>
  <tr><td>    "Initial values for the state derivatives"}}</td></tr>
  <tr><td>  }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/sundials_interface.cpp: const Options SundialsInterface</summary>
  <table><tr><th>= {{&Integrator::options_},</td></th>
  <tr><td>   {{"max_num_steps",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Maximum number of integrator steps"}},</td></tr>
  <tr><td>  {"reltol",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Relative tolerence for the IVP solution"}},</td></tr>
  <tr><td>  {"abstol",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Absolute tolerence for the IVP solution"}},</td></tr>
  <tr><td>  {"newton_scheme",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>    "Linear solver scheme in the Newton method: DIRECT|gmres|bcgstab|tfqmr"}},</td></tr>
  <tr><td>  {"max_krylov",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Maximum Krylov subspace size"}},</td></tr>
  <tr><td>  {"sensitivity_method",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>    "Sensitivity method: SIMULTANEOUS|staggered"}},</td></tr>
  <tr><td>  {"max_multistep_order",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Maximum order for the (variable-order) multistep method"}},</td></tr>
  <tr><td>  {"use_preconditioner",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Precondition the iterative solver [default: true]"}},</td></tr>
  <tr><td>  {"stop_at_end",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "[DEPRECATED] Stop the integrator at the end of the interval"}},</td></tr>
  <tr><td>  {"disable_internal_warnings",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Disable SUNDIALS internal warning messages"}},</td></tr>
  <tr><td>  {"quad_err_con",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Should the quadratures affect the step size control"}},</td></tr>
  <tr><td>  {"fsens_err_con",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "include the forward sensitivities in all error controls"}},</td></tr>
  <tr><td>  {"steps_per_checkpoint",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Number of steps between two consecutive checkpoints"}},</td></tr>
  <tr><td>  {"interpolation_type",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>    "Type of interpolation for the adjoint sensitivities"}},</td></tr>
  <tr><td>  {"linear_solver",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>    "A custom linear solver creator function [default: qr]"}},</td></tr>
  <tr><td>  {"linear_solver_options",</td></tr>
  <tr><td>    {OT_DICT,</td></tr>
  <tr><td>    "Options to be passed to the linear solver"}},</td></tr>
  <tr><td>  {"second_order_correction",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Second order correction in the augmented system Jacobian [true]"}},</td></tr>
  <tr><td>  {"step0",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "initial step size [default: 0/estimated]"}},</td></tr>
  <tr><td>  {"max_step_size",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Max step size [default: 0/inf]"}},</td></tr>
  <tr><td>  {"max_order",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Maximum order"}},</td></tr>
  <tr><td>  {"nonlin_conv_coeff",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Coefficient in the nonlinear convergence test"}},</td></tr>
  <tr><td>  {"scale_abstol",</td></tr>
  <tr><td>   {OT_BOOL,</td></tr>
  <tr><td>    "Scale absolute tolerance by nominal value"}}</td></tr>
  <tr><td>  }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/gurobi/gurobi_interface.cpp: const Options GurobiInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"vtype",</td></tr>
  <tr><td>     {OT_STRINGVECTOR,</td></tr>
  <tr><td>      "Type of variables: [CONTINUOUS|binary|integer|semicont|semiint]"}},</td></tr>
  <tr><td>    {"gurobi",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to gurobi."}},</td></tr>
  <tr><td>    {"sos_groups",</td></tr>
  <tr><td>     {OT_INTVECTORVECTOR,</td></tr>
  <tr><td>      "Definition of SOS groups by indices."}},</td></tr>
  <tr><td>    {"sos_weights",</td></tr>
  <tr><td>     {OT_DOUBLEVECTORVECTOR,</td></tr>
  <tr><td>      "Weights corresponding to SOS entries."}},</td></tr>
  <tr><td>    {"sos_types",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Specify 1 or 2 for each SOS group."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/cplex/cplex_interface.cpp: const Options CplexInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"cplex",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to CPLEX"}},</td></tr>
  <tr><td>    {"qp_method",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Determines which CPLEX algorithm to use."}},</td></tr>
  <tr><td>    {"dump_to_file",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Dumps QP to file in CPLEX format."}},</td></tr>
  <tr><td>    {"dump_filename",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "The filename to dump to."}},</td></tr>
  <tr><td>    {"tol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Tolerance of solver"}},</td></tr>
  <tr><td>    {"dep_check",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Detect redundant constraints."}},</td></tr>
  <tr><td>    {"warm_start",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use warm start with simplex methods (affects only the simplex methods)."}},</td></tr>
  <tr><td>    {"mip_start",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Hot start integers with x0 [Default false]."}},</td></tr>
  <tr><td>    {"sos_groups",</td></tr>
  <tr><td>     {OT_INTVECTORVECTOR,</td></tr>
  <tr><td>      "Definition of SOS groups by indices."}},</td></tr>
  <tr><td>    {"sos_weights",</td></tr>
  <tr><td>     {OT_DOUBLEVECTORVECTOR,</td></tr>
  <tr><td>      "Weights corresponding to SOS entries."}},</td></tr>
  <tr><td>    {"sos_types",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Specify 1 or 2 for each SOS group."}},</td></tr>
  <tr><td>    {"version_suffix",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Specify version of cplex to load. "</td></tr>
  <tr><td>      "We will attempt to load libcplex<version_suffix>.[so|dll|dylib]. "</td></tr>
  <tr><td>      "Default value is taken from CPLEX_VERSION env variable."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/ampl/ampl_interface.cpp: const Options AmplInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"solver",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "AMPL solver binary"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/fatrop/fatrop_conic_interface.cpp: const Options FatropConicInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"N",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "OCP horizon"}},</td></tr>
  <tr><td>    {"nx",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Number of states, length N+1"}},</td></tr>
  <tr><td>    {"nu",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Number of controls, length N"}},</td></tr>
  <tr><td>    {"ng",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Number of non-dynamic constraints, length N+1"}},</td></tr>
  <tr><td>    {"fatrop",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to fatrop"}}}</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/qpoases/qpoases_interface.cpp: const Options QpoasesInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"sparse",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Formulate the QP using sparse matrices. [false]"}},</td></tr>
  <tr><td>    {"schur",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use Schur Complement Approach [false]"}},</td></tr>
  <tr><td>    {"hessian_type",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Type of Hessian - see qpOASES documentation "</td></tr>
  <tr><td>      "[UNKNOWN|posdef|semidef|indef|zero|identity]]"}},</td></tr>
  <tr><td>    {"max_schur",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximal number of Schur updates [75]"}},</td></tr>
  <tr><td>    {"linsol_plugin",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Linear solver plugin"}},</td></tr>
  <tr><td>    {"nWSR",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "The maximum number of working set recalculations to be performed during "</td></tr>
  <tr><td>      "the initial homotopy. Default is 5(nx + nc)"}},</td></tr>
  <tr><td>    {"CPUtime",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "The maximum allowed CPU time in seconds for the whole initialisation"</td></tr>
  <tr><td>      " (and the actually required one on output). Disabled if unset."}},</td></tr>
  <tr><td>    {"printLevel",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Defines the amount of text output during QP solution, see Section 5.7"}},</td></tr>
  <tr><td>    {"enableRamping",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Enables ramping."}},</td></tr>
  <tr><td>    {"enableFarBounds",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Enables the use of  far bounds."}},</td></tr>
  <tr><td>    {"enableFlippingBounds",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Enables the use of  flipping bounds."}},</td></tr>
  <tr><td>    {"enableRegularisation",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Enables automatic  Hessian regularisation."}},</td></tr>
  <tr><td>    {"enableFullLITests",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Enables condition-hardened  (but more expensive) LI test."}},</td></tr>
  <tr><td>    {"enableNZCTests",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Enables nonzero curvature  tests."}},</td></tr>
  <tr><td>    {"enableDriftCorrection",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Specifies the frequency of drift corrections: 0: turns them off."}},</td></tr>
  <tr><td>    {"enableCholeskyRefactorisation",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Specifies the frequency of a full re-factorisation of projected "</td></tr>
  <tr><td>      "Hessian matrix: 0: turns them off,  1: uses them at each iteration etc."}},</td></tr>
  <tr><td>    {"enableEqualities",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Specifies whether equalities should be treated  as always active "</td></tr>
  <tr><td>      "(True) or not (False)"}},</td></tr>
  <tr><td>    {"terminationTolerance",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Relative termination tolerance to stop homotopy."}},</td></tr>
  <tr><td>    {"boundTolerance",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "If upper and lower bounds differ less than this tolerance, they are regarded "</td></tr>
  <tr><td>      "equal, i.e. as  equality constraint."}},</td></tr>
  <tr><td>    {"boundRelaxation",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Initial relaxation of bounds to start homotopy  and initial value for far bounds."}},</td></tr>
  <tr><td>    {"epsNum",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Numerator tolerance for ratio tests."}},</td></tr>
  <tr><td>    {"epsDen",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Denominator tolerance for ratio tests."}},</td></tr>
  <tr><td>    {"maxPrimalJump",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Maximum allowed jump in primal variables in  nonzero curvature tests."}},</td></tr>
  <tr><td>    {"maxDualJump",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Maximum allowed jump in dual variables in  linear independence tests."}},</td></tr>
  <tr><td>    {"initialRamping",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Start value for ramping strategy."}},</td></tr>
  <tr><td>    {"finalRamping",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Final value for ramping strategy."}},</td></tr>
  <tr><td>    {"initialFarBounds",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Initial size for far bounds."}},</td></tr>
  <tr><td>    {"growFarBounds",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Factor to grow far bounds."}},</td></tr>
  <tr><td>    {"initialStatusBounds",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Initial status of bounds at first iteration."}},</td></tr>
  <tr><td>    {"epsFlipping",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Tolerance of squared Cholesky diagonal factor  which triggers flipping bound."}},</td></tr>
  <tr><td>    {"numRegularisationSteps",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of successive regularisation steps."}},</td></tr>
  <tr><td>    {"epsRegularisation",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Scaling factor of identity matrix used for  Hessian regularisation."}},</td></tr>
  <tr><td>    {"numRefinementSteps",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of iterative refinement steps."}},</td></tr>
  <tr><td>    {"epsIterRef",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Early termination tolerance for iterative  refinement."}},</td></tr>
  <tr><td>    {"epsLITests",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Tolerance for linear independence tests."}},</td></tr>
  <tr><td>    {"epsNZCTests",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Tolerance for nonzero curvature tests."}},</td></tr>
  <tr><td>    {"enableInertiaCorrection",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Should working set be repaired when negative curvature is discovered during hotstart."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/clp/clp_interface.cpp: const Options ClpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"clp",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to CLP. "</td></tr>
  <tr><td>      "A first set of options can be found in ClpParameters.hpp. eg. 'PrimalTolerance'. "</td></tr>
  <tr><td>      "There are other options in additions. "</td></tr>
  <tr><td>      "'AutomaticScaling' (bool) is recognised. "</td></tr>
  <tr><td>      "'initial_solve' (default off) activates the use of Clp's initialSolve. "</td></tr>
  <tr><td>      "'initial_solve_options' takes a dictionary with following keys (see ClpSolve.hpp): "</td></tr>
  <tr><td>      " SolveType (string), PresolveType (string), "</td></tr>
  <tr><td>      " NumberPasses, SpecialOptions (intvectorvector), IndependentOptions (intvectorvector)."</td></tr>
  <tr><td>      }}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/snopt/snopt_interface.cpp: const Options SnoptInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"snopt",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to SNOPT"}},</td></tr>
  <tr><td>    {"start",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Warm-start options for Worhp: cold|warm|hot"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/knitro/knitro_interface.cpp: const Options KnitroInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"knitro",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to KNITRO"}},</td></tr>
  <tr><td>    {"options_file",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Read options from file (solver specific)"}},</td></tr>
  <tr><td>    {"detect_linear_constraints",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Detect type of constraints"}},</td></tr>
  <tr><td>    {"contype",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Type of constraint"}},</td></tr>
  <tr><td>    {"complem_variables",</td></tr>
  <tr><td>     {OT_INTVECTORVECTOR,</td></tr>
  <tr><td>      "List of complementary constraints on simple bounds. "</td></tr>
  <tr><td>      "Pair (i, j) encodes complementarity between the bounds on variable i and variable j."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/ooqp/ooqp_interface.cpp: const Options OoqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"print_level",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Print level. OOQP listens to print_level 0, 10 and 100"}},</td></tr>
  <tr><td>    {"mutol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "tolerance as provided with setMuTol to OOQP"}},</td></tr>
  <tr><td>    {"artol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "tolerance as provided with setArTol to OOQP"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sleqp/sleqp_interface.cpp: const Options SLEQPInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"sleqp",</td></tr>
  <tr><td>      {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to SLEQP"}},</td></tr>
  <tr><td>     {"print_level",</td></tr>
  <tr><td>      {OT_INT,</td></tr>
  <tr><td>      "Print level of SLEQP (default: 2/SLEQP_LOG_WARN)"}},</td></tr>
  <tr><td>     {"max_iter",</td></tr>
  <tr><td>      {OT_INT,</td></tr>
  <tr><td>      "Maximum number of iterations"}},</td></tr>
  <tr><td>     {"max_wall_time",</td></tr>
  <tr><td>      {OT_DOUBLE,</td></tr>
  <tr><td>      "maximum wall time allowed"}}</td></tr>
  <tr><td>    }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/superscs/superscs_interface.cpp: const Options SuperscsInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"superscs",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to superscs."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/hpmpc/hpmpc_interface.cpp: const Options HpmpcInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"N",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "OCP horizon"}},</td></tr>
  <tr><td>    {"nx",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Number of states, length N+1"}},</td></tr>
  <tr><td>    {"nu",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Number of controls, length N"}},</td></tr>
  <tr><td>    {"ng",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Number of non-dynamic constraints, length N+1"}},</td></tr>
  <tr><td>    {"mu0",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Max element in cost function as estimate of max multiplier"}},</td></tr>
  <tr><td>    {"max_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Max number of iterations"}},</td></tr>
  <tr><td>    {"tol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Tolerance in the duality measure"}},</td></tr>
  <tr><td>    {"warm_start",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use warm-starting"}},</td></tr>
  <tr><td>    {"inf",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "HPMPC cannot handle infinities. Infinities will be replaced by this option's value."}},</td></tr>
  <tr><td>    {"print_level",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Amount of diagnostic printing [Default: 1]."}},</td></tr>
  <tr><td>    {"target",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "hpmpc target"}},</td></tr>
  <tr><td>    {"blasfeo_target",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "hpmpc target"}}}</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/slicot/slicot_dple.cpp: const Options Slicotrple</summary>
  <table><tr><th>= {{&Dple::options_},</td></th>
  <tr><td>   {{"linear_solver",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "User-defined linear solver class. Needed for sensitivities."}},</td></tr>
  <tr><td>    {"linear_solver_options",</td></tr>
  <tr><td>      {OT_DICT,</td></tr>
  <tr><td>       "Options to be passed to the linear solver."}},</td></tr>
  <tr><td>    {"psd_num_zero",</td></tr>
  <tr><td>      {OT_DOUBLE,</td></tr>
  <tr><td>        "Numerical zero used in Periodic Schur decomposition with slicot."</td></tr>
  <tr><td>        "This option is needed when your systems has Floquet multipliers"</td></tr>
  <tr><td>        "zero or close to zero"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/blocksqp/blocksqp.cpp: const Options Blocksqp</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"qpsol",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "The QP solver to be used by the SQP method"}},</td></tr>
  <tr><td>    {"qpsol_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to the QP solver"}},</td></tr>
  <tr><td>    {"linsol",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "The linear solver to be used by the QP method"}},</td></tr>
  <tr><td>    {"print_header",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print solver header at startup"}},</td></tr>
  <tr><td>    {"print_iteration",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print SQP iterations"}},</td></tr>
  <tr><td>    {"eps",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Values smaller than this are regarded as numerically zero"}},</td></tr>
  <tr><td>    {"opttol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Optimality tolerance"}},</td></tr>
  <tr><td>    {"nlinfeastol",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Nonlinear feasibility tolerance"}},</td></tr>
  <tr><td>    {"schur",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use qpOASES Schur compliment approach"}},</td></tr>
  <tr><td>    {"globalization",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Enable globalization"}},</td></tr>
  <tr><td>    {"restore_feas",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use feasibility restoration phase"}},</td></tr>
  <tr><td>    {"max_line_search",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of steps in line search"}},</td></tr>
  <tr><td>    {"max_consec_reduced_steps",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of consecutive reduced steps"}},</td></tr>
  <tr><td>    {"max_consec_skipped_updates",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of consecutive skipped updates"}},</td></tr>
  <tr><td>    {"max_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of SQP iterations"}},</td></tr>
  <tr><td>    {"warmstart",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use warmstarting"}},</td></tr>
  <tr><td>    {"qp_init",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use warmstarting"}},</td></tr>
  <tr><td>    {"max_it_qp",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of QP iterations per SQP iteration"}},</td></tr>
  <tr><td>    {"block_hess",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Blockwise Hessian approximation?"}},</td></tr>
  <tr><td>    {"hess_scaling",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Scaling strategy for Hessian approximation"}},</td></tr>
  <tr><td>    {"fallback_scaling",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "If indefinite update is used, the type of fallback strategy"}},</td></tr>
  <tr><td>    {"max_time_qp",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Maximum number of time in seconds per QP solve per SQP iteration"}},</td></tr>
  <tr><td>    {"ini_hess_diag",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Initial Hessian guess: diagonal matrix diag(iniHessDiag)"}},</td></tr>
  <tr><td>    {"col_eps",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Epsilon for COL scaling strategy"}},</td></tr>
  <tr><td>    {"col_tau1",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "tau1 for COL scaling strategy"}},</td></tr>
  <tr><td>    {"col_tau2",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "tau2 for COL scaling strategy"}},</td></tr>
  <tr><td>    {"hess_damp",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Activate Powell damping for BFGS"}},</td></tr>
  <tr><td>    {"hess_damp_fac",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Damping factor for BFGS Powell modification"}},</td></tr>
  <tr><td>    {"hess_update",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Type of Hessian approximation"}},</td></tr>
  <tr><td>    {"fallback_update",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "If indefinite update is used, the type of fallback strategy"}},</td></tr>
  <tr><td>    {"hess_lim_mem",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Full or limited memory"}},</td></tr>
  <tr><td>    {"hess_memsize",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Memory size for L-BFGS updates"}},</td></tr>
  <tr><td>    {"which_second_derv",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "For which block should second derivatives be provided by the user"}},</td></tr>
  <tr><td>    {"skip_first_globalization",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "No globalization strategy in first iteration"}},</td></tr>
  <tr><td>    {"conv_strategy",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Convexification strategy"}},</td></tr>
  <tr><td>    {"max_conv_qp",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "How many additional QPs may be solved for convexification per iteration?"}},</td></tr>
  <tr><td>    {"max_soc_iter",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Maximum number of SOC line search iterations"}},</td></tr>
  <tr><td>    {"gamma_theta",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"gamma_f",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"kappa_soc",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"kappa_f",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"theta_max",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"theta_min",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"delta",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"s_theta",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"s_f",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"kappa_minus",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"kappa_plus",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"kappa_plus_max",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"delta_h0",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"eta",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Filter line search parameter, cf. IPOPT paper"}},</td></tr>
  <tr><td>    {"obj_lo",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Lower bound on objective function [-inf]"}},</td></tr>
  <tr><td>    {"obj_up",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Upper bound on objective function [inf]"}},</td></tr>
  <tr><td>    {"rho",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Feasibility restoration phase parameter"}},</td></tr>
  <tr><td>    {"zeta",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Feasibility restoration phase parameter"}},</td></tr>
  <tr><td>    {"print_maxit_reached",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print error when maximum number of SQP iterations reached"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/ipopt/ipopt_interface.cpp: const Options IpoptInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"pass_nonlinear_variables",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Pass list of variables entering nonlinearly to IPOPT"}},</td></tr>
  <tr><td>    {"ipopt",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to IPOPT"}},</td></tr>
  <tr><td>    {"var_string_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "String metadata (a dictionary with lists of strings) "</td></tr>
  <tr><td>      "about variables to be passed to IPOPT"}},</td></tr>
  <tr><td>    {"var_integer_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Integer metadata (a dictionary with lists of integers) "</td></tr>
  <tr><td>      "about variables to be passed to IPOPT"}},</td></tr>
  <tr><td>    {"var_numeric_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Numeric metadata (a dictionary with lists of reals) about "</td></tr>
  <tr><td>      "variables to be passed to IPOPT"}},</td></tr>
  <tr><td>    {"con_string_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "String metadata (a dictionary with lists of strings) about "</td></tr>
  <tr><td>      "constraints to be passed to IPOPT"}},</td></tr>
  <tr><td>    {"con_integer_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Integer metadata (a dictionary with lists of integers) "</td></tr>
  <tr><td>      "about constraints to be passed to IPOPT"}},</td></tr>
  <tr><td>    {"con_numeric_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Numeric metadata (a dictionary with lists of reals) about "</td></tr>
  <tr><td>      "constraints to be passed to IPOPT"}},</td></tr>
  <tr><td>    {"hess_lag",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</td></tr>
  <tr><td>    {"jac_g",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the Jacobian of the constraints "</td></tr>
  <tr><td>      "(autogenerated by default)"}},</td></tr>
  <tr><td>    {"grad_f",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the gradient of the objective "</td></tr>
  <tr><td>      "(column, autogenerated by default)"}},</td></tr>
  <tr><td>    {"convexify_strategy",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "NONE|regularize|eigen-reflect|eigen-clip. "</td></tr>
  <tr><td>      "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</td></tr>
  <tr><td>    {"convexify_margin",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "When using a convexification strategy, make sure that "</td></tr>
  <tr><td>      "the smallest eigenvalue is at least this (default: 1e-7)."}},</td></tr>
  <tr><td>    {"max_iter_eig",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</td></tr>
  <tr><td>    {"clip_inactive_lam",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Explicitly set Lagrange multipliers to 0 when bound is deemed inactive "</td></tr>
  <tr><td>      "(default: false)."}},</td></tr>
  <tr><td>    {"inactive_lam_strategy",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Strategy to detect if a bound is inactive. "</td></tr>
  <tr><td>      "RELTOL: use solver-defined constraint tolerance * inactive_lam_value|"</td></tr>
  <tr><td>      "abstol: use inactive_lam_value"}},</td></tr>
  <tr><td>    {"inactive_lam_value",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Value used in inactive_lam_strategy (default: 10)."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/clang/clang_compiler.cpp: const Options ClangCompiler</summary>
  <table><tr><th>= {{&ImporterInternal::options_},</td></th>
  <tr><td>   {{"include_path",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Include paths for the JIT compiler. "</td></tr>
  <tr><td>      "The include directory shipped with CasADi will be automatically appended."}},</td></tr>
  <tr><td>    {"flags",</td></tr>
  <tr><td>     {OT_STRINGVECTOR,</td></tr>
  <tr><td>      "Compile flags for the JIT compiler. Default: None"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/proxqp/proxqp_interface.cpp: const Options ProxqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"proxqp",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "const proxqp options."}},</td></tr>
  <tr><td>    {"warm_start_primal",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use x input to warmstart [Default: true]."}},</td></tr>
  <tr><td>    {"warm_start_dual",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use y and z input to warmstart [Default: true]."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/bonmin/bonmin_interface.cpp: const Options BonminInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</td></th>
  <tr><td>   {{"pass_nonlinear_variables",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Pass list of variables entering nonlinearly to BONMIN"}},</td></tr>
  <tr><td>    {"pass_nonlinear_constraints",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Pass list of constraints entering nonlinearly to BONMIN"}},</td></tr>
  <tr><td>    {"bonmin",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to BONMIN"}},</td></tr>
  <tr><td>    {"var_string_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "String metadata (a dictionary with lists of strings) "</td></tr>
  <tr><td>      "about variables to be passed to BONMIN"}},</td></tr>
  <tr><td>    {"var_integer_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Integer metadata (a dictionary with lists of integers) "</td></tr>
  <tr><td>      "about variables to be passed to BONMIN"}},</td></tr>
  <tr><td>    {"var_numeric_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Numeric metadata (a dictionary with lists of reals) about "</td></tr>
  <tr><td>      "variables to be passed to BONMIN"}},</td></tr>
  <tr><td>    {"con_string_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "String metadata (a dictionary with lists of strings) about "</td></tr>
  <tr><td>      "constraints to be passed to BONMIN"}},</td></tr>
  <tr><td>    {"con_integer_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Integer metadata (a dictionary with lists of integers) "</td></tr>
  <tr><td>      "about constraints to be passed to BONMIN"}},</td></tr>
  <tr><td>    {"con_numeric_md",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Numeric metadata (a dictionary with lists of reals) about "</td></tr>
  <tr><td>      "constraints to be passed to BONMIN"}},</td></tr>
  <tr><td>    {"hess_lag",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</td></tr>
  <tr><td>    {"hess_lag_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options for the autogenerated Hessian of the Lagrangian."}},</td></tr>
  <tr><td>    {"jac_g",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the Jacobian of the constraints "</td></tr>
  <tr><td>      "(autogenerated by default)"}},</td></tr>
  <tr><td>    {"jac_g_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options for the autogenerated Jacobian of the constraints."}},</td></tr>
  <tr><td>    {"grad_f",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function for calculating the gradient of the objective "</td></tr>
  <tr><td>      "(column, autogenerated by default)"}},</td></tr>
  <tr><td>    {"grad_f_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options for the autogenerated gradient of the objective."}},</td></tr>
  <tr><td>    {"sos1_groups",</td></tr>
  <tr><td>     {OT_INTVECTORVECTOR,</td></tr>
  <tr><td>      "Options for the autogenerated gradient of the objective."}},</td></tr>
  <tr><td>    {"sos1_weights",</td></tr>
  <tr><td>     {OT_DOUBLEVECTORVECTOR,</td></tr>
  <tr><td>      "Options for the autogenerated gradient of the objective."}},</td></tr>
  <tr><td>    {"sos1_priorities",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Options for the autogenerated gradient of the objective."}},</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/cbc/cbc_interface.cpp: const Options CbcInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"cbc",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to CBC."</td></tr>
  <tr><td>      "Three sets of options are supported. "</td></tr>
  <tr><td>      "The first can be found in OsiSolverParameters.hpp. "</td></tr>
  <tr><td>      "The second can be found in CbcModel.hpp. "</td></tr>
  <tr><td>      "The third are options that can be passed to CbcMain1."</td></tr>
  <tr><td>      }},</td></tr>
  <tr><td>    {"sos_groups",</td></tr>
  <tr><td>     {OT_INTVECTORVECTOR,</td></tr>
  <tr><td>      "Definition of SOS groups by indices."}},</td></tr>
  <tr><td>    {"sos_weights",</td></tr>
  <tr><td>     {OT_DOUBLEVECTORVECTOR,</td></tr>
  <tr><td>      "Weights corresponding to SOS entries."}},</td></tr>
  <tr><td>    {"sos_types",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Specify 1 or 2 for each SOS group."}},</td></tr>
  <tr><td>    {"hot_start",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Hot start with x0 [Default false]."}},</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/osqp/osqp_interface.cpp: const Options OsqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</td></th>
  <tr><td>   {{"osqp",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "const Options to be passed to osqp."}},</td></tr>
  <tr><td>    {"warm_start_primal",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use x0 input to warmstart [Default: true]."}},</td></tr>
  <tr><td>    {"warm_start_dual",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use lam_a0 and lam_x0 input to warmstart [Default: truw]."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/sx_function.cpp: const Options SXFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"default_in",</td></tr>
  <tr><td>     {OT_DOUBLEVECTOR,</td></tr>
  <tr><td>      "Default input values"}},</td></tr>
  <tr><td>    {"just_in_time_sparsity",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Propagate sparsity patterns using just-in-time "</td></tr>
  <tr><td>      "compilation to a CPU or GPU using OpenCL"}},</td></tr>
  <tr><td>    {"just_in_time_opencl",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Just-in-time compilation for numeric evaluation using OpenCL (experimental)"}},</td></tr>
  <tr><td>    {"live_variables",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Reuse variables in the work vector"}},</td></tr>
  <tr><td>    {"cse",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Perform common subexpression elimination (complexity is N*log(N) in graph size)"}},</td></tr>
  <tr><td>    {"allow_free",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Allow construction with free variables (Default: false)"}},</td></tr>
  <tr><td>    {"allow_duplicate_io_names",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Allow construction with duplicate io names (Default: false)"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options Integrator</summary>
  <table><tr><th>= {{&OracleFunction::options_},</td></th>
  <tr><td>   {{"expand",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Replace MX with SX expressions in problem formulation [false]"}},</td></tr>
  <tr><td>  {"print_stats",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Print out statistics after integration"}},</td></tr>
  <tr><td>  {"nfwd",</td></tr>
  <tr><td>   {OT_INT,</td></tr>
  <tr><td>    "Number of forward sensitivities to be calculated [0]"}},</td></tr>
  <tr><td>  {"nadj",</td></tr>
  <tr><td>   {OT_INT,</td></tr>
  <tr><td>    "Number of adjoint sensitivities to be calculated [0]"}},</td></tr>
  <tr><td>  {"t0",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "[DEPRECATED] Beginning of the time horizon"}},</td></tr>
  <tr><td>  {"tf",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "[DEPRECATED] End of the time horizon"}},</td></tr>
  <tr><td>  {"grid",</td></tr>
  <tr><td>    {OT_DOUBLEVECTOR,</td></tr>
  <tr><td>    "[DEPRECATED] Time grid"}},</td></tr>
  <tr><td>  {"augmented_options",</td></tr>
  <tr><td>    {OT_DICT,</td></tr>
  <tr><td>    "Options to be passed down to the augmented integrator, if one is constructed."}},</td></tr>
  <tr><td>  {"output_t0",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "[DEPRECATED] Output the state at the initial time"}}</td></tr>
  <tr><td>  }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options FixedStepIntegrator</summary>
  <table><tr><th>= {{&Integrator::options_},</td></th>
  <tr><td>   {{"number_of_finite_elements",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Target number of finite elements. "</td></tr>
  <tr><td>    "The actual number may be higher to accommodate all output times"}},</td></tr>
  <tr><td>  {"simplify",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Implement as MX Function (codegeneratable/serializable) default: false"}},</td></tr>
  <tr><td>  {"simplify_options",</td></tr>
  <tr><td>    {OT_DICT,</td></tr>
  <tr><td>    "Any options to pass to simplified form Function constructor"}}</td></tr>
  <tr><td>  }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options ImplicitFixedStepIntegrator</summary>
  <table><tr><th>= {{&FixedStepIntegrator::options_},</td></th>
  <tr><td>   {{"rootfinder",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>    "An implicit function solver"}},</td></tr>
  <tr><td>  {"rootfinder_options",</td></tr>
  <tr><td>    {OT_DICT,</td></tr>
  <tr><td>    "Options to be passed to the NLP Solver"}}</td></tr>
  <tr><td>  }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/conic.cpp: const Options Conic</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"discrete",</td></tr>
  <tr><td>     {OT_BOOLVECTOR,</td></tr>
  <tr><td>      "Indicates which of the variables are discrete, i.e. integer-valued"}},</td></tr>
  <tr><td>    {"print_problem",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print a numeric description of the problem"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/rootfinder.cpp: const Options Rootfinder</summary>
  <table><tr><th>= {{&OracleFunction::options_},</td></th>
  <tr><td>   {{"linear_solver",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "User-defined linear solver class. Needed for sensitivities."}},</td></tr>
  <tr><td>    {"linear_solver_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to the linear solver."}},</td></tr>
  <tr><td>    {"constraints",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "Constrain the unknowns. 0 (default): no constraint on ui, "</td></tr>
  <tr><td>      "1: ui >= 0.0, -1: ui <= 0.0, 2: ui > 0.0, -2: ui < 0.0."}},</td></tr>
  <tr><td>    {"implicit_input",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Index of the input that corresponds to the actual root-finding"}},</td></tr>
  <tr><td>    {"implicit_output",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Index of the output that corresponds to the actual root-finding"}},</td></tr>
  <tr><td>    {"jacobian_function",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Function object for calculating the Jacobian (autogenerated by default)"}},</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/mx_function.cpp: const Options MXFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"default_in",</td></tr>
  <tr><td>     {OT_DOUBLEVECTOR,</td></tr>
  <tr><td>      "Default input values"}},</td></tr>
  <tr><td>    {"live_variables",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Reuse variables in the work vector"}},</td></tr>
  <tr><td>    {"print_instructions",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print each operation during evaluation"}},</td></tr>
  <tr><td>    {"cse",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Perform common subexpression elimination (complexity is N*log(N) in graph size)"}},</td></tr>
  <tr><td>    {"allow_free",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Allow construction with free variables (Default: false)"}},</td></tr>
  <tr><td>    {"allow_duplicate_io_names",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Allow construction with duplicate io names (Default: false)"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/expm.cpp: const Options Expm</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"const_A",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Assume A is constant. Default: false."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/finite_differences.cpp: const Options FiniteDiff</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"second_order_stepsize",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Second order perturbation size [default: 1e-3]"}},</td></tr>
  <tr><td>  {"h",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Step size [default: computed from abstol]"}},</td></tr>
  <tr><td>  {"h_max",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Maximum step size [default 0]"}},</td></tr>
  <tr><td>  {"h_min",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Minimum step size [default inf]"}},</td></tr>
  <tr><td>  {"smoothing",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Smoothing regularization [default: machine precision]"}},</td></tr>
  <tr><td>  {"reltol",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Accuracy of function inputs [default: query object]"}},</td></tr>
  <tr><td>  {"abstol",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Accuracy of function outputs [default: query object]"}},</td></tr>
  <tr><td>  {"u_aim",</td></tr>
  <tr><td>    {OT_DOUBLE,</td></tr>
  <tr><td>    "Target ratio of roundoff error to truncation error [default: 100.]"}},</td></tr>
  <tr><td>  {"h_iter",</td></tr>
  <tr><td>    {OT_INT,</td></tr>
  <tr><td>    "Number of iterations to improve on the step-size "</td></tr>
  <tr><td>    "[default: 1 if error estimate available, otherwise 0]"}},</td></tr>
  <tr><td>  }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/jit_function.cpp: const Options JitFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"buffered",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>      "Buffer the calls, user does not need to "}},</td></tr>
  <tr><td>     {"jac",</td></tr>
  <tr><td>    {OT_STRING,</td></tr>
  <tr><td>      "Function body for Jacobian"}},</td></tr>
  <tr><td>    {"hess",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Function body for Hessian"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/function_internal.cpp: const Options ProtoFunction</summary>
  <table><tr><th>= {{},</td></th>
  <tr><td>   {{"verbose",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Verbose evaluation -- for debugging"}},</td></tr>
  <tr><td>    {"print_time",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "print information about execution time. Implies record_time."}},</td></tr>
  <tr><td>    {"record_time",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "record information about execution time, for retrieval with stats()."}},</td></tr>
  <tr><td>    {"regularity_check",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Throw exceptions when NaN or Inf appears during evaluation"}},</td></tr>
  <tr><td>    {"error_on_fail",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Throw exceptions when function evaluation fails (default true)."}}</td></tr>
  <tr><td>    }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/function_internal.cpp: const Options FunctionInternal</summary>
  <table><tr><th>= {{&ProtoFunction::options_},</td></th>
  <tr><td>   {{"ad_weight",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Weighting factor for derivative calculation."</td></tr>
  <tr><td>      "When there is an option of either using forward or reverse mode "</td></tr>
  <tr><td>      "directional derivatives, the condition ad_weight*nf<=(1-ad_weight)*na "</td></tr>
  <tr><td>      "is used where nf and na are estimates of the number of forward/reverse "</td></tr>
  <tr><td>      "mode directional derivatives needed. By default, ad_weight is calculated "</td></tr>
  <tr><td>      "automatically, but this can be overridden by setting this option. "</td></tr>
  <tr><td>      "In particular, 0 means forcing forward mode and 1 forcing reverse mode. "</td></tr>
  <tr><td>      "Leave unset for (class specific) heuristics."}},</td></tr>
  <tr><td>    {"ad_weight_sp",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Weighting factor for sparsity pattern calculation calculation."</td></tr>
  <tr><td>      "Overrides default behavior. Set to 0 and 1 to force forward and "</td></tr>
  <tr><td>      "reverse mode respectively. Cf. option \"ad_weight\". "</td></tr>
  <tr><td>      "When set to -1, sparsity is completely ignored and dense matrices are used."}},</td></tr>
  <tr><td>    {"always_inline",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Force inlining."}},</td></tr>
  <tr><td>    {"never_inline",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Forbid inlining."}},</td></tr>
  <tr><td>    {"jac_penalty",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "When requested for a number of forward/reverse directions,   "</td></tr>
  <tr><td>      "it may be cheaper to compute first the full jacobian and then "</td></tr>
  <tr><td>      "multiply with seeds, rather than obtain the requested directions "</td></tr>
  <tr><td>      "in a straightforward manner. "</td></tr>
  <tr><td>      "Casadi uses a heuristic to decide which is cheaper. "</td></tr>
  <tr><td>      "A high value of 'jac_penalty' makes it less likely for the heurstic "</td></tr>
  <tr><td>      "to chose the full Jacobian strategy. "</td></tr>
  <tr><td>      "The special value -1 indicates never to use the full Jacobian strategy"}},</td></tr>
  <tr><td>    {"user_data",</td></tr>
  <tr><td>     {OT_VOIDPTR,</td></tr>
  <tr><td>      "A user-defined field that can be used to identify "</td></tr>
  <tr><td>      "the function or pass additional information"}},</td></tr>
  <tr><td>    {"inputs_check",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Throw exceptions when the numerical values of the inputs don't make sense"}},</td></tr>
  <tr><td>    {"gather_stats",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Deprecated option (ignored): Statistics are now always collected."}},</td></tr>
  <tr><td>    {"input_scheme",</td></tr>
  <tr><td>     {OT_STRINGVECTOR,</td></tr>
  <tr><td>      "Deprecated option (ignored)"}},</td></tr>
  <tr><td>    {"output_scheme",</td></tr>
  <tr><td>     {OT_STRINGVECTOR,</td></tr>
  <tr><td>      "Deprecated option (ignored)"}},</td></tr>
  <tr><td>    {"jit",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use just-in-time compiler to speed up the evaluation"}},</td></tr>
  <tr><td>    {"jit_cleanup",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Cleanup up the temporary source file that jit creates. Default: true"}},</td></tr>
  <tr><td>    {"jit_serialize",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Specify behaviour when serializing a jitted function: SOURCE|link|embed."}},</td></tr>
  <tr><td>    {"jit_name",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "The file name used to write out code. "</td></tr>
  <tr><td>      "The actual file names used depend on 'jit_temp_suffix' and include extensions. "</td></tr>
  <tr><td>      "Default: 'jit_tmp'"}},</td></tr>
  <tr><td>    {"jit_temp_suffix",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Use a temporary (seemingly random) filename suffix for generated code and libraries. "</td></tr>
  <tr><td>      "This is desired for thread-safety. "</td></tr>
  <tr><td>      "This behaviour may defeat caching compiler wrappers. "</td></tr>
  <tr><td>      "Default: true"}},</td></tr>
  <tr><td>    {"compiler",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Just-in-time compiler plugin to be used."}},</td></tr>
  <tr><td>    {"jit_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to the jit compiler."}},</td></tr>
  <tr><td>    {"derivative_of",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "The function is a derivative of another function. "</td></tr>
  <tr><td>      "The type of derivative (directional derivative, Jacobian) "</td></tr>
  <tr><td>      "is inferred from the function name."}},</td></tr>
  <tr><td>    {"max_num_dir",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Specify the maximum number of directions for derivative functions."</td></tr>
  <tr><td>      " Overrules the builtin optimized_num_dir."}},</td></tr>
  <tr><td>    {"enable_forward",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Enable derivative calculation using generated functions for"</td></tr>
  <tr><td>      " Jacobian-times-vector products - typically using forward mode AD"</td></tr>
  <tr><td>      " - if available. [default: true]"}},</td></tr>
  <tr><td>    {"enable_reverse",</td></tr>
  <tr><td>      {OT_BOOL,</td></tr>
  <tr><td>      "Enable derivative calculation using generated functions for"</td></tr>
  <tr><td>      " transposed Jacobian-times-vector products - typically using reverse mode AD"</td></tr>
  <tr><td>      " - if available. [default: true]"}},</td></tr>
  <tr><td>    {"enable_jacobian",</td></tr>
  <tr><td>      {OT_BOOL,</td></tr>
  <tr><td>      "Enable derivative calculation using generated functions for"</td></tr>
  <tr><td>      " Jacobians of all differentiable outputs with respect to all differentiable inputs"</td></tr>
  <tr><td>      " - if available. [default: true]"}},</td></tr>
  <tr><td>    {"enable_fd",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Enable derivative calculation by finite differencing. [default: false]]"}},</td></tr>
  <tr><td>    {"fd_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to the finite difference instance"}},</td></tr>
  <tr><td>    {"fd_method",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Method for finite differencing [default 'central']"}},</td></tr>
  <tr><td>    {"print_in",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print numerical values of inputs [default: false]"}},</td></tr>
  <tr><td>    {"print_out",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print numerical values of outputs [default: false]"}},</td></tr>
  <tr><td>    {"max_io",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Acceptable number of inputs and outputs. Warn if exceeded."}},</td></tr>
  <tr><td>    {"dump_in",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Dump numerical values of inputs to file (readable with DM.from_file) [default: false]"}},</td></tr>
  <tr><td>    {"dump_out",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Dump numerical values of outputs to file (readable with DM.from_file) [default: false]"}},</td></tr>
  <tr><td>    {"dump",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Dump function to file upon first evaluation. [false]"}},</td></tr>
  <tr><td>    {"dump_dir",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Directory to dump inputs/outputs to. Make sure the directory exists [.]"}},</td></tr>
  <tr><td>    {"dump_format",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Choose file format to dump matrices. See DM.from_file [mtx]"}},</td></tr>
  <tr><td>    {"forward_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to a forward mode constructor"}},</td></tr>
  <tr><td>    {"reverse_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to a reverse mode constructor"}},</td></tr>
  <tr><td>    {"jacobian_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to a Jacobian constructor"}},</td></tr>
  <tr><td>    {"der_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Default options to be used to populate forward_options, reverse_options, and "</td></tr>
  <tr><td>      "jacobian_options before those options are merged in."}},</td></tr>
  <tr><td>    {"custom_jacobian",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "Override CasADi's AD. Use together with 'jac_penalty': 0. "</td></tr>
  <tr><td>      "Note: Highly experimental. Syntax may break often."}},</td></tr>
  <tr><td>    {"is_diff_in",</td></tr>
  <tr><td>     {OT_BOOLVECTOR,</td></tr>
  <tr><td>      "Indicate for each input if it should be differentiable."}},</td></tr>
  <tr><td>    {"is_diff_out",</td></tr>
  <tr><td>     {OT_BOOLVECTOR,</td></tr>
  <tr><td>      "Indicate for each output if it should be differentiable."}},</td></tr>
  <tr><td>    {"post_expand",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "After construction, expand this Function. Default: False"}},</td></tr>
  <tr><td>    {"post_expand_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to post-construction expansion. Default: empty"}},</td></tr>
  <tr><td>    {"cache",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Prepopulate the function cache. Default: empty"}},</td></tr>
  <tr><td>    {"external_transform",</td></tr>
  <tr><td>     {OT_VECTORVECTOR,</td></tr>
  <tr><td>      "List of external_transform instruction arguments. Default: empty"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/oracle_function.cpp: const Options OracleFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"expand",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Replace MX with SX expressions in problem formulation [false]"}},</td></tr>
  <tr><td>  {"monitor",</td></tr>
  <tr><td>    {OT_STRINGVECTOR,</td></tr>
  <tr><td>    "Set of user problem functions to be monitored"}},</td></tr>
  <tr><td>  {"show_eval_warnings",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>    "Show warnings generated from function evaluations [true]"}},</td></tr>
  <tr><td>  {"common_options",</td></tr>
  <tr><td>    {OT_DICT,</td></tr>
  <tr><td>    "Options for auto-generated functions"}},</td></tr>
  <tr><td>  {"specific_options",</td></tr>
  <tr><td>    {OT_DICT,</td></tr>
  <tr><td>    "Options for specific auto-generated functions,"</td></tr>
  <tr><td>    " overwriting the defaults from common_options. Nested dictionary."}}</td></tr>
  <tr><td>}</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/importer_internal.cpp: const Options ImporterInternal</summary>
  <table><tr><th>= {{},</td></th>
  <tr><td>   {{"verbose",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Verbose evaluation -- for debugging"}}</td></tr>
  <tr><td>    }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/interpolant.cpp: const Options Interpolant</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"lookup_mode",</td></tr>
  <tr><td>     {OT_STRINGVECTOR,</td></tr>
  <tr><td>      "Specifies, for each grid dimension, the lookup algorithm used to find the correct index. "</td></tr>
  <tr><td>      "'linear' uses a for-loop + break; (default when #knots<=100), "</td></tr>
  <tr><td>      "'exact' uses floored division (only for uniform grids), "</td></tr>
  <tr><td>      "'binary' uses a binary search. (default when #knots>100)."}},</td></tr>
  <tr><td>    {"inline",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Implement the lookup table in MX primitives. "</td></tr>
  <tr><td>      "Useful when you need derivatives with respect to grid and/or coefficients. "</td></tr>
  <tr><td>      "Such derivatives are fundamentally dense, so use with caution."}},</td></tr>
  <tr><td>    {"batch_x",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Evaluate a batch of different inputs at once (default 1)."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/nlpsol.cpp: const Options Nlpsol</summary>
  <table><tr><th>= {{&OracleFunction::options_},</td></th>
  <tr><td>   {{"iteration_callback",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "A function that will be called at each iteration with the solver as input. "</td></tr>
  <tr><td>      "Check documentation of Callback."}},</td></tr>
  <tr><td>    {"iteration_callback_step",</td></tr>
  <tr><td>     {OT_INT,</td></tr>
  <tr><td>      "Only call the callback function every few iterations."}},</td></tr>
  <tr><td>    {"iteration_callback_ignore_errors",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "If set to true, errors thrown by iteration_callback will be ignored."}},</td></tr>
  <tr><td>    {"ignore_check_vec",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "If set to true, the input shape of F will not be checked."}},</td></tr>
  <tr><td>    {"warn_initial_bounds",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Warn if the initial guess does not satisfy LBX and UBX"}},</td></tr>
  <tr><td>    {"eval_errors_fatal",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "When errors occur during evaluation of f,g,...,"</td></tr>
  <tr><td>      "stop the iterations"}},</td></tr>
  <tr><td>    {"verbose_init",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Print out timing information about "</td></tr>
  <tr><td>      "the different stages of initialization"}},</td></tr>
  <tr><td>    {"discrete",</td></tr>
  <tr><td>     {OT_BOOLVECTOR,</td></tr>
  <tr><td>      "Indicates which of the variables are discrete, i.e. integer-valued"}},</td></tr>
  <tr><td>    {"calc_multipliers",</td></tr>
  <tr><td>    {OT_BOOL,</td></tr>
  <tr><td>     "Calculate Lagrange multipliers in the Nlpsol base class"}},</td></tr>
  <tr><td>    {"calc_lam_x",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Calculate 'lam_x' in the Nlpsol base class"}},</td></tr>
  <tr><td>    {"calc_lam_p",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Calculate 'lam_p' in the Nlpsol base class"}},</td></tr>
  <tr><td>    {"calc_f",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Calculate 'f' in the Nlpsol base class"}},</td></tr>
  <tr><td>    {"calc_g",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Calculate 'g' in the Nlpsol base class"}},</td></tr>
  <tr><td>    {"no_nlp_grad",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Prevent the creation of the 'nlp_grad' function"}},</td></tr>
  <tr><td>    {"bound_consistency",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Ensure that primal-dual solution is consistent with the bounds"}},</td></tr>
  <tr><td>    {"min_lam",</td></tr>
  <tr><td>     {OT_DOUBLE,</td></tr>
  <tr><td>      "Minimum allowed multiplier value"}},</td></tr>
  <tr><td>    {"oracle_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Options to be passed to the oracle function"}},</td></tr>
  <tr><td>    {"sens_linsol",</td></tr>
  <tr><td>     {OT_STRING,</td></tr>
  <tr><td>      "Linear solver used for parametric sensitivities (default 'qr')."}},</td></tr>
  <tr><td>    {"sens_linsol_options",</td></tr>
  <tr><td>     {OT_DICT,</td></tr>
  <tr><td>      "Linear solver options used for parametric sensitivities."}},</td></tr>
  <tr><td>    {"detect_simple_bounds",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Automatically detect simple bounds (lbx/ubx) (default false). "</td></tr>
  <tr><td>      "This is hopefully beneficial to speed and robustness but may also have adverse affects: "</td></tr>
  <tr><td>      "1) Subtleties in heuristics and stopping criteria may change the solution, "</td></tr>
  <tr><td>      "2) IPOPT may lie about multipliers of simple equality bounds unless "</td></tr>
  <tr><td>      "'fixed_variable_treatment' is set to 'relax_bounds'."}},</td></tr>
  <tr><td>    {"detect_simple_bounds_is_simple",</td></tr>
  <tr><td>     {OT_BOOLVECTOR,</td></tr>
  <tr><td>      "For internal use only."}},</td></tr>
  <tr><td>    {"detect_simple_bounds_parts",</td></tr>
  <tr><td>     {OT_FUNCTION,</td></tr>
  <tr><td>      "For internal use only."}},</td></tr>
  <tr><td>    {"detect_simple_bounds_target_x",</td></tr>
  <tr><td>     {OT_INTVECTOR,</td></tr>
  <tr><td>      "For internal use only."}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/fmu_function.cpp: const Options FmuFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"scheme_in",</td></tr>
  <tr><td>   {OT_STRINGVECTOR,</td></tr>
  <tr><td>    "Names of the inputs in the scheme"}},</td></tr>
  <tr><td>  {"scheme_out",</td></tr>
  <tr><td>   {OT_STRINGVECTOR,</td></tr>
  <tr><td>    "Names of the outputs in the scheme"}},</td></tr>
  <tr><td>  {"scheme",</td></tr>
  <tr><td>   {OT_DICT,</td></tr>
  <tr><td>    "Definitions of the scheme variables"}},</td></tr>
  <tr><td>  {"aux",</td></tr>
  <tr><td>   {OT_STRINGVECTOR,</td></tr>
  <tr><td>    "Auxilliary variables"}},</td></tr>
  <tr><td>  {"enable_ad",</td></tr>
  <tr><td>   {OT_BOOL,</td></tr>
  <tr><td>    "Calculate first order derivatives using FMU directional derivative support"}},</td></tr>
  <tr><td>  {"validate_ad",</td></tr>
  <tr><td>   {OT_BOOL,</td></tr>
  <tr><td>    "Compare analytic derivatives with finite differences for validation"}},</td></tr>
  <tr><td>  {"validate_ad_file",</td></tr>
  <tr><td>   {OT_STRING,</td></tr>
  <tr><td>    "Redirect results of Hessian validation to a file instead of generating a warning"}},</td></tr>
  <tr><td>  {"check_hessian",</td></tr>
  <tr><td>   {OT_BOOL,</td></tr>
  <tr><td>    "Symmetry check for Hessian"}},</td></tr>
  <tr><td>  {"make_symmetric",</td></tr>
  <tr><td>   {OT_BOOL,</td></tr>
  <tr><td>    "Ensure Hessian is symmetric"}},</td></tr>
  <tr><td>  {"step",</td></tr>
  <tr><td>   {OT_DOUBLE,</td></tr>
  <tr><td>    "Step size, scaled by nominal value"}},</td></tr>
  <tr><td>  {"abstol",</td></tr>
  <tr><td>   {OT_DOUBLE,</td></tr>
  <tr><td>    "Absolute error tolerance, scaled by nominal value"}},</td></tr>
  <tr><td>  {"reltol",</td></tr>
  <tr><td>   {OT_DOUBLE,</td></tr>
  <tr><td>    "Relative error tolerance"}},</td></tr>
  <tr><td>  {"parallelization",</td></tr>
  <tr><td>   {OT_STRING,</td></tr>
  <tr><td>    "Parallelization [SERIAL|openmp|thread]"}},</td></tr>
  <tr><td>  {"print_progress",</td></tr>
  <tr><td>   {OT_BOOL,</td></tr>
  <tr><td>    "Print progress during Jacobian/Hessian evaluation"}},</td></tr>
  <tr><td>  {"new_jacobian",</td></tr>
  <tr><td>   {OT_BOOL,</td></tr>
  <tr><td>    "Use Jacobian implementation in class"}},</td></tr>
  <tr><td>  {"new_hessian",</td></tr>
  <tr><td>   {OT_BOOL,</td></tr>
  <tr><td>    "Use Hessian implementation in class"}},</td></tr>
  <tr><td>  {"hessian_coloring",</td></tr>
  <tr><td>   {OT_BOOL,</td></tr>
  <tr><td>    "Enable the use of graph coloring (star coloring) for Hessian calculation. "</td></tr>
  <tr><td>    "Note that disabling the coloring can improve symmetry check diagnostics."}}</td></tr>
  <tr><td> }</td></tr>
  <tr><td>  };</td></tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/dple.cpp: const Options Dple</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</td></th>
  <tr><td>   {{"const_dim",</td></tr>
  <tr><td>     {OT_BOOL,</td></tr>
  <tr><td>      "Assume constant dimension of P"}},</td></tr>
  <tr><td>    {"pos_def",</td></tr>
  <tr><td>      {OT_BOOL,</td></tr>
  <tr><td>       "Assume P positive definite"}},</td></tr>
  <tr><td>    {"error_unstable",</td></tr>
  <tr><td>      {OT_BOOL,</td></tr>
  <tr><td>      "Throw an exception when it is detected that Product(A_i, i=N..1)"</td></tr>
  <tr><td>      "has eigenvalues greater than 1-eps_unstable"}},</td></tr>
  <tr><td>    {"eps_unstable",</td></tr>
  <tr><td>      {OT_DOUBLE,</td></tr>
  <tr><td>      "A margin for unstability detection"}}</td></tr>
  <tr><td>   }</td></tr>
  <tr><td>  };</td></tr>
</table></details>
