# MA24_simulink

<details>
 <summary>./casadi/casadi/solvers/linsol_ldl.cpp: const Options LinsolLdl</summary>
  <table><tr><th>= {{&ProtoFunction::options_},</th></tr>
  <tr>   {{"incomplete",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>     "Incomplete factorization, without any fill-in"}},</tr>
  <tr>    {"preordering",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>     "Approximate minimal degree (AMD) preordering"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/linear_interpolant.cpp: const Options LinearInterpolant</summary>
  <table><tr><th>= {{&Interpolant::options_},</th></tr>
  <tr>   {{"lookup_mode",</tr>
  <tr>     {OT_STRINGVECTOR,</tr>
  <tr>      "Sets, for each grid dimenion, the lookup algorithm used to find the correct index. "</tr>
  <tr>      "'linear' uses a for-loop + break; "</tr>
  <tr>      "'exact' uses floored division (only for uniform grids)."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/ipqp.cpp: const Options Ipqp</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"max_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of iterations [1000]."}},</tr>
  <tr>    {"constr_viol_tol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Constraint violation tolerance [1e-8]."}},</tr>
  <tr>    {"dual_inf_tol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Dual feasibility violation tolerance [1e-8]"}},</tr>
  <tr>    {"print_header",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print header [true]."}},</tr>
  <tr>    {"print_iter",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print iterations [true]."}},</tr>
  <tr>    {"print_info",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print info [true]."}},</tr>
  <tr>    {"linear_solver",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "A custom linear solver creator function [default: ldl]"}},</tr>
  <tr>    {"linear_solver_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to the linear solver"}},</tr>
  <tr>    {"min_lam",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Smallest multiplier treated as inactive for the initial active set [0]."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/qrsqp.cpp: const Options Qrsqp</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"qpsol",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "The QP solver to be used by the SQP method [qrqp]"}},</tr>
  <tr>    {"qpsol_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to the QP solver"}},</tr>
  <tr>    {"hessian_approximation",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "limited-memory|exact"}},</tr>
  <tr>    {"max_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of SQP iterations"}},</tr>
  <tr>    {"min_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Minimum number of SQP iterations"}},</tr>
  <tr>    {"max_iter_ls",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of linesearch iterations"}},</tr>
  <tr>    {"tol_pr",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion for primal infeasibility"}},</tr>
  <tr>    {"tol_du",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion for dual infeasability"}},</tr>
  <tr>    {"c1",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Armijo condition, coefficient of decrease in merit"}},</tr>
  <tr>    {"beta",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Line-search parameter, restoration factor of stepsize"}},</tr>
  <tr>    {"merit_memory",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Size of memory to store history of merit function values"}},</tr>
  <tr>    {"lbfgs_memory",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Size of L-BFGS memory."}},</tr>
  <tr>    {"regularize",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Automatic regularization of Lagrange Hessian."}},</tr>
  <tr>    {"print_header",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print the header with problem statistics"}},</tr>
  <tr>    {"print_iteration",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print the iterations"}},</tr>
  <tr>    {"min_step_size",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "The size (inf-norm) of the step size should not become smaller than this."}},</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/scpgen.cpp: const Options Scpgen</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"qpsol",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "The QP solver to be used by the SQP method"}},</tr>
  <tr>    {"qpsol_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to the QP solver"}},</tr>
  <tr>    {"hessian_approximation",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "gauss-newton|exact"}},</tr>
  <tr>    {"max_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of SQP iterations"}},</tr>
  <tr>    {"max_iter_ls",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of linesearch iterations"}},</tr>
  <tr>    {"tol_pr",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion for primal infeasibility"}},</tr>
  <tr>    {"tol_du",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion for dual infeasability"}},</tr>
  <tr>    {"tol_reg",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion for regularization"}},</tr>
  <tr>    {"tol_pr_step",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion for the step size"}},</tr>
  <tr>    {"c1",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Armijo condition, coefficient of decrease in merit"}},</tr>
  <tr>    {"beta",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Line-search parameter, restoration factor of stepsize"}},</tr>
  <tr>    {"merit_memsize",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Size of memory to store history of merit function values"}},</tr>
  <tr>    {"merit_start",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Lower bound for the merit function parameter"}},</tr>
  <tr>    {"lbfgs_memory",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Size of L-BFGS memory."}},</tr>
  <tr>    {"regularize",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Automatic regularization of Lagrange Hessian."}},</tr>
  <tr>    {"print_header",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print the header with problem statistics"}},</tr>
  <tr>    {"codegen",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "C-code generation"}},</tr>
  <tr>    {"reg_threshold",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Threshold for the regularization."}},</tr>
  <tr>    {"name_x",</tr>
  <tr>     {OT_STRINGVECTOR,</tr>
  <tr>      "Names of the variables."}},</tr>
  <tr>    {"print_x",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Which variables to print."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/linsol_qr.cpp: const Options LinsolQr</summary>
  <table><tr><th>= {{&LinsolInternal::options_},</th></tr>
  <tr>   {{"eps",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Minimum R entry before singularity is declared [1e-12]"}},</tr>
  <tr>    {"cache",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Amount of factorisations to remember (thread-local) [0]"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/newton.cpp: const Options Newton</summary>
  <table><tr><th>= {{&Rootfinder::options_},</th></tr>
  <tr>   {{"abstol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion tolerance on max(|F|)"}},</tr>
  <tr>    {"abstolStep",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion tolerance on step size"}},</tr>
  <tr>    {"max_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of Newton iterations to perform before returning."}},</tr>
  <tr>    {"print_iteration",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print information about each iteration"}},</tr>
  <tr>    {"line_search",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Enable line-search (default: true)"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/sqpmethod.cpp: const Options Sqpmethod</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"qpsol",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>    "The QP solver to be used by the SQP method [qpoases]"}},</tr>
  <tr>  {"qpsol_options",</tr>
  <tr>    {OT_DICT,</tr>
  <tr>    "Options to be passed to the QP solver"}},</tr>
  <tr>  {"hessian_approximation",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>    "limited-memory|exact"}},</tr>
  <tr>  {"max_iter",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Maximum number of SQP iterations"}},</tr>
  <tr>  {"min_iter",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Minimum number of SQP iterations"}},</tr>
  <tr>  {"max_iter_ls",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Maximum number of linesearch iterations"}},</tr>
  <tr>  {"tol_pr",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Stopping criterion for primal infeasibility"}},</tr>
  <tr>  {"tol_du",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Stopping criterion for dual infeasability"}},</tr>
  <tr>  {"c1",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Armijo condition, coefficient of decrease in merit"}},</tr>
  <tr>  {"beta",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Line-search parameter, restoration factor of stepsize"}},</tr>
  <tr>  {"merit_memory",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Size of memory to store history of merit function values"}},</tr>
  <tr>  {"lbfgs_memory",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Size of L-BFGS memory."}},</tr>
  <tr>  {"print_header",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Print the header with problem statistics"}},</tr>
  <tr>  {"print_iteration",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Print the iterations"}},</tr>
  <tr>  {"print_status",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Print a status message after solving"}},</tr>
  <tr>  {"min_step_size",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "The size (inf-norm) of the step size should not become smaller than this."}},</tr>
  <tr>  {"hess_lag",</tr>
  <tr>    {OT_FUNCTION,</tr>
  <tr>    "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</tr>
  <tr>  {"jac_fg",</tr>
  <tr>    {OT_FUNCTION,</tr>
  <tr>    "Function for calculating the gradient of the objective and Jacobian of the constraints "</tr>
  <tr>    "(autogenerated by default)"}},</tr>
  <tr>  {"convexify_strategy",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>    "NONE|regularize|eigen-reflect|eigen-clip. "</tr>
  <tr>    "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</tr>
  <tr>  {"convexify_margin",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "When using a convexification strategy, make sure that "</tr>
  <tr>    "the smallest eigenvalue is at least this (default: 1e-7)."}},</tr>
  <tr>  {"max_iter_eig",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</tr>
  <tr>  {"elastic_mode",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Enable the elastic mode which is used when the QP is infeasible (default: false)."}},</tr>
  <tr>  {"gamma_0",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Starting value for the penalty parameter of elastic mode (default: 1)."}},</tr>
  <tr>  {"gamma_max",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Maximum value for the penalty parameter of elastic mode (default: 1e20)."}},</tr>
  <tr>  {"gamma_1_min",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Minimum value for gamma_1 (default: 1e-5)."}},</tr>
  <tr>  {"second_order_corrections",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Enable second order corrections. "</tr>
  <tr>    "These are used when a step is considered bad by the merit function and constraint norm "</tr>
  <tr>    "(default: false)."}},</tr>
  <tr>  {"init_feasible",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Initialize the QP subproblems with a feasible initial value (default: false)."}}</tr>
  <tr>  }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/fast_newton.cpp: const Options FastNewton</summary>
  <table><tr><th>= {{&Rootfinder::options_},</th></tr>
  <tr>   {{"abstol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion tolerance on ||g||__inf)"}},</tr>
  <tr>    {"abstolStep",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion tolerance on step size"}},</tr>
  <tr>    {"max_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of Newton iterations to perform before returning."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/shell_compiler.cpp: const Options ShellCompiler</summary>
  <table><tr><th>= {{&ImporterInternal::options_},</th></tr>
  <tr>   {{"compiler",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Compiler command"}},</tr>
  <tr>    {"linker",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Linker command"}},</tr>
  <tr>    {"directory",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Directory to put temporary objects in. Must end with a file separator."}},</tr>
  <tr>    {"compiler_setup",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Compiler setup command. Intended to be fixed."</tr>
  <tr>      " The 'flag' option is the prefered way to set"</tr>
  <tr>      " custom flags."}},</tr>
  <tr>    {"linker_setup",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Linker setup command. Intended to be fixed."</tr>
  <tr>      " The 'flag' option is the prefered way to set"</tr>
  <tr>      " custom flags."}},</tr>
  <tr>    {"compiler_flags",</tr>
  <tr>     {OT_STRINGVECTOR,</tr>
  <tr>      "Alias for 'compiler_flags'"}},</tr>
  <tr>    {"flags",</tr>
  <tr>      {OT_STRINGVECTOR,</tr>
  <tr>      "Compile flags for the JIT compiler. Default: None"}},</tr>
  <tr>    {"linker_flags",</tr>
  <tr>     {OT_STRINGVECTOR,</tr>
  <tr>      "Linker flags for the JIT compiler. Default: None"}},</tr>
  <tr>    {"cleanup",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Cleanup temporary files when unloading. Default: true"}},</tr>
  <tr>    {"compiler_output_flag",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>     "Compiler flag to denote object output. Default: '-o '"}},</tr>
  <tr>    {"linker_output_flag",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>     "Linker flag to denote shared library output. Default: '-o '"}},</tr>
  <tr>    {"extra_suffixes",</tr>
  <tr>     {OT_STRINGVECTOR,</tr>
  <tr>     "List of suffixes for extra files that the compiler may generate. Default: None"}},</tr>
  <tr>    {"name",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "The file name used to write out compiled objects/libraries. "</tr>
  <tr>      "The actual file names used depend on 'temp_suffix' and include extensions. "</tr>
  <tr>      "Default: 'tmp_casadi_compiler_shell'"}},</tr>
  <tr>    {"temp_suffix",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use a temporary (seemingly random) filename suffix for file names. "</tr>
  <tr>      "This is desired for thread-safety. "</tr>
  <tr>      "This behaviour may defeat caching compiler wrappers. "</tr>
  <tr>      "Default: true"}},</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/implicit_to_nlp.cpp: const Options ImplicitToNlp</summary>
  <table><tr><th>= {{&Rootfinder::options_},</th></tr>
  <tr>   {{"nlpsol",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Name of solver."}},</tr>
  <tr>    {"nlpsol_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to solver."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/qrqp.cpp: const Options Qrqp</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"max_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of iterations [1000]."}},</tr>
  <tr>    {"constr_viol_tol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Constraint violation tolerance [1e-8]."}},</tr>
  <tr>    {"dual_inf_tol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Dual feasibility violation tolerance [1e-8]"}},</tr>
  <tr>    {"print_header",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print header [true]."}},</tr>
  <tr>    {"print_iter",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print iterations [true]."}},</tr>
  <tr>    {"print_info",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print info [true]."}},</tr>
  <tr>    {"print_lincomb",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print dependant linear combinations of constraints [false]. "</tr>
  <tr>      "Printed numbers are 0-based indices into the vector of [simple bounds;linear bounds]"}},</tr>
  <tr>    {"min_lam",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Smallest multiplier treated as inactive for the initial active set [0]."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/feasiblesqpmethod.cpp: const Options Feasiblesqpmethod</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"solve_type",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "The solver type: Either SQP or SLP. Defaults to SQP"}},</tr>
  <tr>    {"qpsol",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "The QP solver to be used by the SQP method [qpoases]"}},</tr>
  <tr>    {"qpsol_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to the QP solver"}},</tr>
  <tr>    {"hessian_approximation",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "limited-memory|exact"}},</tr>
  <tr>    {"max_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of SQP iterations"}},</tr>
  <tr>    {"min_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Minimum number of SQP iterations"}},</tr>
  <tr>    {"tol_pr",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion for primal infeasibility"}},</tr>
  <tr>    {"tol_du",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion for dual infeasability"}},</tr>
  <tr>    {"merit_memory",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Size of memory to store history of merit function values"}},</tr>
  <tr>    {"lbfgs_memory",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Size of L-BFGS memory."}},</tr>
  <tr>    {"print_header",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print the header with problem statistics"}},</tr>
  <tr>    {"print_iteration",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print the iterations"}},</tr>
  <tr>    {"print_status",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print a status message after solving"}},</tr>
  <tr>    {"f",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the objective function (autogenerated by default)"}},</tr>
  <tr>    {"g",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the constraints (autogenerated by default)"}},</tr>
  <tr>    {"grad_f",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the gradient of the objective (autogenerated by default)"}},</tr>
  <tr>    {"jac_g",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the Jacobian of the constraints (autogenerated by default)"}},</tr>
  <tr>    {"hess_lag",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</tr>
  <tr>    {"convexify_strategy",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "NONE|regularize|eigen-reflect|eigen-clip. "</tr>
  <tr>      "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</tr>
  <tr>    {"convexify_margin",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "When using a convexification strategy, make sure that "</tr>
  <tr>      "the smallest eigenvalue4 is at least this (default: 1e-7)."}},</tr>
  <tr>    {"max_iter_eig",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</tr>
  <tr>    {"init_feasible",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Initialize the QP subproblems with a feasible initial value (default: false)."}},</tr>
  <tr>    {"optim_tol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Optimality tolerance. Below this value an iterate is considered to be optimal."}},</tr>
  <tr>    {"feas_tol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Feasibility tolerance. Below this tolerance an iterate is considered to be feasible."}},</tr>
  <tr>    {"tr_rad0",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Initial trust-region radius."}},</tr>
  <tr>    {"tr_eta1",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Lower eta in trust-region acceptance criterion."}},</tr>
  <tr>    {"tr_eta2",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Upper eta in trust-region acceptance criterion."}},</tr>
  <tr>    {"tr_alpha1",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Lower alpha in trust-region size criterion."}},</tr>
  <tr>    {"tr_alpha2",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Upper alpha in trust-region size criterion."}},</tr>
  <tr>    {"tr_tol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Trust-region tolerance. "</tr>
  <tr>      "Below this value another scalar is equal to the trust region radius."}},</tr>
  <tr>    {"tr_acceptance",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Is the trust-region ratio above this value, the step is accepted."}},</tr>
  <tr>    {"tr_rad_min",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Minimum trust-region radius."}},</tr>
  <tr>    {"tr_rad_max",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Maximum trust-region radius."}},</tr>
  <tr>    {"tr_scale_vector",</tr>
  <tr>     {OT_DOUBLEVECTOR,</tr>
  <tr>      "Vector that tells where trust-region is applied."}},</tr>
  <tr>    {"contraction_acceptance_value",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "If the empirical contraction rate in the feasibility iterations "</tr>
  <tr>      "is above this value in the heuristics the iterations are aborted."}},</tr>
  <tr>    {"watchdog",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Number of watchdog iterations in feasibility iterations. "</tr>
  <tr>      "After this amount of iterations, it is checked with the contraction acceptance value, "</tr>
  <tr>      "if iterations are converging."}},</tr>
  <tr>    {"max_inner_iter",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Maximum number of inner iterations."}},</tr>
  <tr>    {"use_anderson",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use Anderson Acceleration. (default false)"}},</tr>
  <tr>    {"anderson_memory",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Anderson memory. If Anderson is used default is 1, else default is 0."}},</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/collocation.cpp: const Options Collocation</summary>
  <table><tr><th>= {{&ImplicitFixedStepIntegrator::options_},</th></tr>
  <tr>   {{"interpolation_order",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Order of the interpolating polynomials"}},</tr>
  <tr>    {"collocation_scheme",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Collocation scheme: radau|legendre"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/symbolic_qr.cpp: const Options SymbolicQr</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"fopts",</tr>
  <tr>    {OT_DICT,</tr>
  <tr>     "Options to be passed to generated function objects"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/bspline_interpolant.cpp: const Options BSplineInterpolant</summary>
  <table><tr><th>= {{&Interpolant::options_},</th></tr>
  <tr>   {{"degree",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Sets, for each grid dimension, the degree of the spline."}},</tr>
  <tr>     {"linear_solver",</tr>
  <tr>      {OT_STRING,</tr>
  <tr>       "Solver used for constructing the coefficient tensor."}},</tr>
  <tr>     {"linear_solver_options",</tr>
  <tr>      {OT_DICT,</tr>
  <tr>       "Options to be passed to the linear solver."}},</tr>
  <tr>     {"algorithm",</tr>
  <tr>      {OT_STRING,</tr>
  <tr>       "Algorithm used for fitting the data: 'not_a_knot' (default, same as Matlab),"</tr>
  <tr>      " 'smooth_linear'."}},</tr>
  <tr>     {"smooth_linear_frac",</tr>
  <tr>      {OT_DOUBLE,</tr>
  <tr>       "When 'smooth_linear' algorithm is active, determines sharpness between"</tr>
  <tr>       " 0 (sharp, as linear interpolation) and 0.5 (smooth)."</tr>
  <tr>       "Default value is 0.1."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/qp_to_nlp.cpp: const Options QpToNlp</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"nlpsol",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Name of solver."}},</tr>
  <tr>    {"nlpsol_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to solver."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/hpipm/hpipm_interface.cpp: const Options HpipmInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"N",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "OCP horizon"}},</tr>
  <tr>    {"nx",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Number of states, length N+1"}},</tr>
  <tr>    {"nu",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Number of controls, length N"}},</tr>
  <tr>    {"ng",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Number of non-dynamic constraints, length N+1"}},</tr>
  <tr>    {"inf",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Replace infinities by this amount [default: 1e8]"}},</tr>
  <tr>    {"hpipm",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to hpipm"}}}</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/lapack/lapack_lu.cpp: const Options LapackLu</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"equilibration",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Equilibrate the matrix"}},</tr>
  <tr>    {"allow_equilibration_failure",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Non-fatal error when equilibration fails"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/lapack/lapack_qr.cpp: const Options LapackQr</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"max_nrhs",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of right-hand-sides that get processed in a single pass [default:10]."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/highs/highs_interface.cpp: const Options HighsInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"highs",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to HiGHS."</tr>
  <tr>      }},</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/alpaqa/alpaqa_interface.cpp: const Options AlpaqaInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"alpaqa",</tr>
  <tr>      {OT_DICT,</tr>
  <tr>      "Options to be passed to Alpaqa"}}</tr>
  <tr>    }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/mumps/mumps_interface.cpp: const Options MumpsInterface</summary>
  <table><tr><th>= {{&ProtoFunction::options_},</th></tr>
  <tr>   {{"symmetric",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>     "Symmetric matrix"}},</tr>
  <tr>    {"posdef",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>     "Positive definite"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/daqp/daqp_interface.cpp: const Options DaqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"daqp",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to Daqp."</tr>
  <tr>      }},</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/worhp/worhp_interface.cpp: const Options WorhpInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"worhp",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to WORHP"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/cvodes_interface.cpp: const Options CvodesInterface</summary>
  <table><tr><th>= {{&SundialsInterface::options_},</th></tr>
  <tr>   {{"linear_multistep_method",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>    "Integrator scheme: BDF|adams"}},</tr>
  <tr>  {"nonlinear_solver_iteration",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>    "Nonlinear solver type: NEWTON|functional"}},</tr>
  <tr>  {"min_step_size",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Min step size [default: 0/0.0]"}},</tr>
  <tr>  {"fsens_all_at_once",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Calculate all right hand sides of the sensitivity equations at once"}},</tr>
  <tr>  {"always_recalculate_jacobian",</tr>
  <tr>   {OT_BOOL,</tr>
  <tr>    "Recalculate Jacobian before factorizations, even if Jacobian is current [default: true]"}}</tr>
  <tr>  }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/kinsol_interface.cpp: const Options KinsolInterface</summary>
  <table><tr><th>= {{&Rootfinder::options_},</th></tr>
  <tr>   {{"max_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of Newton iterations. Putting 0 sets the default value of KinSol."}},</tr>
  <tr>    {"print_level",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Verbosity level"}},</tr>
  <tr>    {"abstol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Stopping criterion tolerance"}},</tr>
  <tr>    {"linear_solver_type",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "dense|banded|iterative|user_defined"}},</tr>
  <tr>    {"upper_bandwidth",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Upper bandwidth for banded linear solvers"}},</tr>
  <tr>    {"lower_bandwidth",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Lower bandwidth for banded linear solvers"}},</tr>
  <tr>    {"max_krylov",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum Krylov space dimension"}},</tr>
  <tr>    {"exact_jacobian",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use exact Jacobian information"}},</tr>
  <tr>    {"iterative_solver",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "gmres|bcgstab|tfqmr"}},</tr>
  <tr>    {"f_scale",</tr>
  <tr>     {OT_DOUBLEVECTOR,</tr>
  <tr>      "Equation scaling factors"}},</tr>
  <tr>    {"u_scale",</tr>
  <tr>     {OT_DOUBLEVECTOR,</tr>
  <tr>      "Variable scaling factors"}},</tr>
  <tr>    {"pretype",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Type of preconditioner"}},</tr>
  <tr>    {"use_preconditioner",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Precondition an iterative solver"}},</tr>
  <tr>    {"strategy",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Globalization strategy"}},</tr>
  <tr>    {"disable_internal_warnings",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Disable KINSOL internal warning messages"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/idas_interface.cpp: const Options IdasInterface</summary>
  <table><tr><th>= {{&SundialsInterface::options_},</th></tr>
  <tr>   {{"suppress_algebraic",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Suppress algebraic variables in the error testing"}},</tr>
  <tr>  {"calc_ic",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Use IDACalcIC to get consistent initial conditions."}},</tr>
  <tr>  {"constraints",</tr>
  <tr>    {OT_INTVECTOR,</tr>
  <tr>    "Constrain the solution y=[x,z]. 0 (default): no constraint on yi, "</tr>
  <tr>    "1: yi >= 0.0, -1: yi <= 0.0, 2: yi > 0.0, -2: yi < 0.0."}},</tr>
  <tr>  {"calc_icB",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Use IDACalcIC to get consistent initial conditions for "</tr>
  <tr>    "backwards system [default: equal to calc_ic]."}},</tr>
  <tr>  {"abstolv",</tr>
  <tr>    {OT_DOUBLEVECTOR,</tr>
  <tr>    "Absolute tolerarance for each component"}},</tr>
  <tr>  {"max_step_size",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Maximim step size"}},</tr>
  <tr>  {"first_time",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "First requested time as a fraction of the time interval"}},</tr>
  <tr>  {"cj_scaling",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "IDAS scaling on cj for the user-defined linear solver module"}},</tr>
  <tr>  {"init_xdot",</tr>
  <tr>    {OT_DOUBLEVECTOR,</tr>
  <tr>    "Initial values for the state derivatives"}}</tr>
  <tr>  }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/sundials_interface.cpp: const Options SundialsInterface</summary>
  <table><tr><th>= {{&Integrator::options_},</th></tr>
  <tr>   {{"max_num_steps",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Maximum number of integrator steps"}},</tr>
  <tr>  {"reltol",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Relative tolerence for the IVP solution"}},</tr>
  <tr>  {"abstol",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Absolute tolerence for the IVP solution"}},</tr>
  <tr>  {"newton_scheme",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>    "Linear solver scheme in the Newton method: DIRECT|gmres|bcgstab|tfqmr"}},</tr>
  <tr>  {"max_krylov",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Maximum Krylov subspace size"}},</tr>
  <tr>  {"sensitivity_method",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>    "Sensitivity method: SIMULTANEOUS|staggered"}},</tr>
  <tr>  {"max_multistep_order",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Maximum order for the (variable-order) multistep method"}},</tr>
  <tr>  {"use_preconditioner",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Precondition the iterative solver [default: true]"}},</tr>
  <tr>  {"stop_at_end",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "[DEPRECATED] Stop the integrator at the end of the interval"}},</tr>
  <tr>  {"disable_internal_warnings",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Disable SUNDIALS internal warning messages"}},</tr>
  <tr>  {"quad_err_con",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Should the quadratures affect the step size control"}},</tr>
  <tr>  {"fsens_err_con",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "include the forward sensitivities in all error controls"}},</tr>
  <tr>  {"steps_per_checkpoint",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Number of steps between two consecutive checkpoints"}},</tr>
  <tr>  {"interpolation_type",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>    "Type of interpolation for the adjoint sensitivities"}},</tr>
  <tr>  {"linear_solver",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>    "A custom linear solver creator function [default: qr]"}},</tr>
  <tr>  {"linear_solver_options",</tr>
  <tr>    {OT_DICT,</tr>
  <tr>    "Options to be passed to the linear solver"}},</tr>
  <tr>  {"second_order_correction",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Second order correction in the augmented system Jacobian [true]"}},</tr>
  <tr>  {"step0",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "initial step size [default: 0/estimated]"}},</tr>
  <tr>  {"max_step_size",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Max step size [default: 0/inf]"}},</tr>
  <tr>  {"max_order",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Maximum order"}},</tr>
  <tr>  {"nonlin_conv_coeff",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Coefficient in the nonlinear convergence test"}},</tr>
  <tr>  {"scale_abstol",</tr>
  <tr>   {OT_BOOL,</tr>
  <tr>    "Scale absolute tolerance by nominal value"}}</tr>
  <tr>  }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/gurobi/gurobi_interface.cpp: const Options GurobiInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"vtype",</tr>
  <tr>     {OT_STRINGVECTOR,</tr>
  <tr>      "Type of variables: [CONTINUOUS|binary|integer|semicont|semiint]"}},</tr>
  <tr>    {"gurobi",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to gurobi."}},</tr>
  <tr>    {"sos_groups",</tr>
  <tr>     {OT_INTVECTORVECTOR,</tr>
  <tr>      "Definition of SOS groups by indices."}},</tr>
  <tr>    {"sos_weights",</tr>
  <tr>     {OT_DOUBLEVECTORVECTOR,</tr>
  <tr>      "Weights corresponding to SOS entries."}},</tr>
  <tr>    {"sos_types",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Specify 1 or 2 for each SOS group."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/cplex/cplex_interface.cpp: const Options CplexInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"cplex",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to CPLEX"}},</tr>
  <tr>    {"qp_method",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Determines which CPLEX algorithm to use."}},</tr>
  <tr>    {"dump_to_file",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Dumps QP to file in CPLEX format."}},</tr>
  <tr>    {"dump_filename",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "The filename to dump to."}},</tr>
  <tr>    {"tol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Tolerance of solver"}},</tr>
  <tr>    {"dep_check",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Detect redundant constraints."}},</tr>
  <tr>    {"warm_start",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use warm start with simplex methods (affects only the simplex methods)."}},</tr>
  <tr>    {"mip_start",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Hot start integers with x0 [Default false]."}},</tr>
  <tr>    {"sos_groups",</tr>
  <tr>     {OT_INTVECTORVECTOR,</tr>
  <tr>      "Definition of SOS groups by indices."}},</tr>
  <tr>    {"sos_weights",</tr>
  <tr>     {OT_DOUBLEVECTORVECTOR,</tr>
  <tr>      "Weights corresponding to SOS entries."}},</tr>
  <tr>    {"sos_types",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Specify 1 or 2 for each SOS group."}},</tr>
  <tr>    {"version_suffix",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Specify version of cplex to load. "</tr>
  <tr>      "We will attempt to load libcplex<version_suffix>.[so|dll|dylib]. "</tr>
  <tr>      "Default value is taken from CPLEX_VERSION env variable."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/ampl/ampl_interface.cpp: const Options AmplInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"solver",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "AMPL solver binary"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/fatrop/fatrop_conic_interface.cpp: const Options FatropConicInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"N",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "OCP horizon"}},</tr>
  <tr>    {"nx",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Number of states, length N+1"}},</tr>
  <tr>    {"nu",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Number of controls, length N"}},</tr>
  <tr>    {"ng",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Number of non-dynamic constraints, length N+1"}},</tr>
  <tr>    {"fatrop",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to fatrop"}}}</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/qpoases/qpoases_interface.cpp: const Options QpoasesInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"sparse",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Formulate the QP using sparse matrices. [false]"}},</tr>
  <tr>    {"schur",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use Schur Complement Approach [false]"}},</tr>
  <tr>    {"hessian_type",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Type of Hessian - see qpOASES documentation "</tr>
  <tr>      "[UNKNOWN|posdef|semidef|indef|zero|identity]]"}},</tr>
  <tr>    {"max_schur",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximal number of Schur updates [75]"}},</tr>
  <tr>    {"linsol_plugin",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Linear solver plugin"}},</tr>
  <tr>    {"nWSR",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "The maximum number of working set recalculations to be performed during "</tr>
  <tr>      "the initial homotopy. Default is 5(nx + nc)"}},</tr>
  <tr>    {"CPUtime",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "The maximum allowed CPU time in seconds for the whole initialisation"</tr>
  <tr>      " (and the actually required one on output). Disabled if unset."}},</tr>
  <tr>    {"printLevel",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Defines the amount of text output during QP solution, see Section 5.7"}},</tr>
  <tr>    {"enableRamping",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Enables ramping."}},</tr>
  <tr>    {"enableFarBounds",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Enables the use of  far bounds."}},</tr>
  <tr>    {"enableFlippingBounds",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Enables the use of  flipping bounds."}},</tr>
  <tr>    {"enableRegularisation",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Enables automatic  Hessian regularisation."}},</tr>
  <tr>    {"enableFullLITests",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Enables condition-hardened  (but more expensive) LI test."}},</tr>
  <tr>    {"enableNZCTests",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Enables nonzero curvature  tests."}},</tr>
  <tr>    {"enableDriftCorrection",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Specifies the frequency of drift corrections: 0: turns them off."}},</tr>
  <tr>    {"enableCholeskyRefactorisation",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Specifies the frequency of a full re-factorisation of projected "</tr>
  <tr>      "Hessian matrix: 0: turns them off,  1: uses them at each iteration etc."}},</tr>
  <tr>    {"enableEqualities",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Specifies whether equalities should be treated  as always active "</tr>
  <tr>      "(True) or not (False)"}},</tr>
  <tr>    {"terminationTolerance",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Relative termination tolerance to stop homotopy."}},</tr>
  <tr>    {"boundTolerance",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "If upper and lower bounds differ less than this tolerance, they are regarded "</tr>
  <tr>      "equal, i.e. as  equality constraint."}},</tr>
  <tr>    {"boundRelaxation",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Initial relaxation of bounds to start homotopy  and initial value for far bounds."}},</tr>
  <tr>    {"epsNum",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Numerator tolerance for ratio tests."}},</tr>
  <tr>    {"epsDen",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Denominator tolerance for ratio tests."}},</tr>
  <tr>    {"maxPrimalJump",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Maximum allowed jump in primal variables in  nonzero curvature tests."}},</tr>
  <tr>    {"maxDualJump",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Maximum allowed jump in dual variables in  linear independence tests."}},</tr>
  <tr>    {"initialRamping",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Start value for ramping strategy."}},</tr>
  <tr>    {"finalRamping",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Final value for ramping strategy."}},</tr>
  <tr>    {"initialFarBounds",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Initial size for far bounds."}},</tr>
  <tr>    {"growFarBounds",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Factor to grow far bounds."}},</tr>
  <tr>    {"initialStatusBounds",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Initial status of bounds at first iteration."}},</tr>
  <tr>    {"epsFlipping",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Tolerance of squared Cholesky diagonal factor  which triggers flipping bound."}},</tr>
  <tr>    {"numRegularisationSteps",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of successive regularisation steps."}},</tr>
  <tr>    {"epsRegularisation",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Scaling factor of identity matrix used for  Hessian regularisation."}},</tr>
  <tr>    {"numRefinementSteps",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of iterative refinement steps."}},</tr>
  <tr>    {"epsIterRef",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Early termination tolerance for iterative  refinement."}},</tr>
  <tr>    {"epsLITests",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Tolerance for linear independence tests."}},</tr>
  <tr>    {"epsNZCTests",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Tolerance for nonzero curvature tests."}},</tr>
  <tr>    {"enableInertiaCorrection",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Should working set be repaired when negative curvature is discovered during hotstart."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/clp/clp_interface.cpp: const Options ClpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"clp",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to CLP. "</tr>
  <tr>      "A first set of options can be found in ClpParameters.hpp. eg. 'PrimalTolerance'. "</tr>
  <tr>      "There are other options in additions. "</tr>
  <tr>      "'AutomaticScaling' (bool) is recognised. "</tr>
  <tr>      "'initial_solve' (default off) activates the use of Clp's initialSolve. "</tr>
  <tr>      "'initial_solve_options' takes a dictionary with following keys (see ClpSolve.hpp): "</tr>
  <tr>      " SolveType (string), PresolveType (string), "</tr>
  <tr>      " NumberPasses, SpecialOptions (intvectorvector), IndependentOptions (intvectorvector)."</tr>
  <tr>      }}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/snopt/snopt_interface.cpp: const Options SnoptInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"snopt",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to SNOPT"}},</tr>
  <tr>    {"start",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Warm-start options for Worhp: cold|warm|hot"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/knitro/knitro_interface.cpp: const Options KnitroInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"knitro",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to KNITRO"}},</tr>
  <tr>    {"options_file",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Read options from file (solver specific)"}},</tr>
  <tr>    {"detect_linear_constraints",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Detect type of constraints"}},</tr>
  <tr>    {"contype",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Type of constraint"}},</tr>
  <tr>    {"complem_variables",</tr>
  <tr>     {OT_INTVECTORVECTOR,</tr>
  <tr>      "List of complementary constraints on simple bounds. "</tr>
  <tr>      "Pair (i, j) encodes complementarity between the bounds on variable i and variable j."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/ooqp/ooqp_interface.cpp: const Options OoqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"print_level",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Print level. OOQP listens to print_level 0, 10 and 100"}},</tr>
  <tr>    {"mutol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "tolerance as provided with setMuTol to OOQP"}},</tr>
  <tr>    {"artol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "tolerance as provided with setArTol to OOQP"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sleqp/sleqp_interface.cpp: const Options SLEQPInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"sleqp",</tr>
  <tr>      {OT_DICT,</tr>
  <tr>      "Options to be passed to SLEQP"}},</tr>
  <tr>     {"print_level",</tr>
  <tr>      {OT_INT,</tr>
  <tr>      "Print level of SLEQP (default: 2/SLEQP_LOG_WARN)"}},</tr>
  <tr>     {"max_iter",</tr>
  <tr>      {OT_INT,</tr>
  <tr>      "Maximum number of iterations"}},</tr>
  <tr>     {"max_wall_time",</tr>
  <tr>      {OT_DOUBLE,</tr>
  <tr>      "maximum wall time allowed"}}</tr>
  <tr>    }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/superscs/superscs_interface.cpp: const Options SuperscsInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"superscs",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to superscs."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/hpmpc/hpmpc_interface.cpp: const Options HpmpcInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"N",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "OCP horizon"}},</tr>
  <tr>    {"nx",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Number of states, length N+1"}},</tr>
  <tr>    {"nu",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Number of controls, length N"}},</tr>
  <tr>    {"ng",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Number of non-dynamic constraints, length N+1"}},</tr>
  <tr>    {"mu0",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Max element in cost function as estimate of max multiplier"}},</tr>
  <tr>    {"max_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Max number of iterations"}},</tr>
  <tr>    {"tol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Tolerance in the duality measure"}},</tr>
  <tr>    {"warm_start",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use warm-starting"}},</tr>
  <tr>    {"inf",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "HPMPC cannot handle infinities. Infinities will be replaced by this option's value."}},</tr>
  <tr>    {"print_level",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Amount of diagnostic printing [Default: 1]."}},</tr>
  <tr>    {"target",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "hpmpc target"}},</tr>
  <tr>    {"blasfeo_target",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "hpmpc target"}}}</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/slicot/slicot_dple.cpp: const Options Slicotrple</summary>
  <table><tr><th>= {{&Dple::options_},</th></tr>
  <tr>   {{"linear_solver",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "User-defined linear solver class. Needed for sensitivities."}},</tr>
  <tr>    {"linear_solver_options",</tr>
  <tr>      {OT_DICT,</tr>
  <tr>       "Options to be passed to the linear solver."}},</tr>
  <tr>    {"psd_num_zero",</tr>
  <tr>      {OT_DOUBLE,</tr>
  <tr>        "Numerical zero used in Periodic Schur decomposition with slicot."</tr>
  <tr>        "This option is needed when your systems has Floquet multipliers"</tr>
  <tr>        "zero or close to zero"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/blocksqp/blocksqp.cpp: const Options Blocksqp</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"qpsol",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "The QP solver to be used by the SQP method"}},</tr>
  <tr>    {"qpsol_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to the QP solver"}},</tr>
  <tr>    {"linsol",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "The linear solver to be used by the QP method"}},</tr>
  <tr>    {"print_header",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print solver header at startup"}},</tr>
  <tr>    {"print_iteration",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print SQP iterations"}},</tr>
  <tr>    {"eps",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Values smaller than this are regarded as numerically zero"}},</tr>
  <tr>    {"opttol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Optimality tolerance"}},</tr>
  <tr>    {"nlinfeastol",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Nonlinear feasibility tolerance"}},</tr>
  <tr>    {"schur",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use qpOASES Schur compliment approach"}},</tr>
  <tr>    {"globalization",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Enable globalization"}},</tr>
  <tr>    {"restore_feas",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use feasibility restoration phase"}},</tr>
  <tr>    {"max_line_search",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of steps in line search"}},</tr>
  <tr>    {"max_consec_reduced_steps",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of consecutive reduced steps"}},</tr>
  <tr>    {"max_consec_skipped_updates",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of consecutive skipped updates"}},</tr>
  <tr>    {"max_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of SQP iterations"}},</tr>
  <tr>    {"warmstart",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use warmstarting"}},</tr>
  <tr>    {"qp_init",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use warmstarting"}},</tr>
  <tr>    {"max_it_qp",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of QP iterations per SQP iteration"}},</tr>
  <tr>    {"block_hess",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Blockwise Hessian approximation?"}},</tr>
  <tr>    {"hess_scaling",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Scaling strategy for Hessian approximation"}},</tr>
  <tr>    {"fallback_scaling",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "If indefinite update is used, the type of fallback strategy"}},</tr>
  <tr>    {"max_time_qp",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Maximum number of time in seconds per QP solve per SQP iteration"}},</tr>
  <tr>    {"ini_hess_diag",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Initial Hessian guess: diagonal matrix diag(iniHessDiag)"}},</tr>
  <tr>    {"col_eps",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Epsilon for COL scaling strategy"}},</tr>
  <tr>    {"col_tau1",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "tau1 for COL scaling strategy"}},</tr>
  <tr>    {"col_tau2",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "tau2 for COL scaling strategy"}},</tr>
  <tr>    {"hess_damp",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Activate Powell damping for BFGS"}},</tr>
  <tr>    {"hess_damp_fac",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Damping factor for BFGS Powell modification"}},</tr>
  <tr>    {"hess_update",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Type of Hessian approximation"}},</tr>
  <tr>    {"fallback_update",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "If indefinite update is used, the type of fallback strategy"}},</tr>
  <tr>    {"hess_lim_mem",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Full or limited memory"}},</tr>
  <tr>    {"hess_memsize",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Memory size for L-BFGS updates"}},</tr>
  <tr>    {"which_second_derv",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "For which block should second derivatives be provided by the user"}},</tr>
  <tr>    {"skip_first_globalization",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "No globalization strategy in first iteration"}},</tr>
  <tr>    {"conv_strategy",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Convexification strategy"}},</tr>
  <tr>    {"max_conv_qp",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "How many additional QPs may be solved for convexification per iteration?"}},</tr>
  <tr>    {"max_soc_iter",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Maximum number of SOC line search iterations"}},</tr>
  <tr>    {"gamma_theta",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"gamma_f",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"kappa_soc",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"kappa_f",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"theta_max",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"theta_min",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"delta",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"s_theta",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"s_f",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"kappa_minus",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"kappa_plus",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"kappa_plus_max",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"delta_h0",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"eta",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Filter line search parameter, cf. IPOPT paper"}},</tr>
  <tr>    {"obj_lo",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Lower bound on objective function [-inf]"}},</tr>
  <tr>    {"obj_up",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Upper bound on objective function [inf]"}},</tr>
  <tr>    {"rho",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Feasibility restoration phase parameter"}},</tr>
  <tr>    {"zeta",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Feasibility restoration phase parameter"}},</tr>
  <tr>    {"print_maxit_reached",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print error when maximum number of SQP iterations reached"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/ipopt/ipopt_interface.cpp: const Options IpoptInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"pass_nonlinear_variables",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Pass list of variables entering nonlinearly to IPOPT"}},</tr>
  <tr>    {"ipopt",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to IPOPT"}},</tr>
  <tr>    {"var_string_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "String metadata (a dictionary with lists of strings) "</tr>
  <tr>      "about variables to be passed to IPOPT"}},</tr>
  <tr>    {"var_integer_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Integer metadata (a dictionary with lists of integers) "</tr>
  <tr>      "about variables to be passed to IPOPT"}},</tr>
  <tr>    {"var_numeric_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Numeric metadata (a dictionary with lists of reals) about "</tr>
  <tr>      "variables to be passed to IPOPT"}},</tr>
  <tr>    {"con_string_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "String metadata (a dictionary with lists of strings) about "</tr>
  <tr>      "constraints to be passed to IPOPT"}},</tr>
  <tr>    {"con_integer_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Integer metadata (a dictionary with lists of integers) "</tr>
  <tr>      "about constraints to be passed to IPOPT"}},</tr>
  <tr>    {"con_numeric_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Numeric metadata (a dictionary with lists of reals) about "</tr>
  <tr>      "constraints to be passed to IPOPT"}},</tr>
  <tr>    {"hess_lag",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</tr>
  <tr>    {"jac_g",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the Jacobian of the constraints "</tr>
  <tr>      "(autogenerated by default)"}},</tr>
  <tr>    {"grad_f",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the gradient of the objective "</tr>
  <tr>      "(column, autogenerated by default)"}},</tr>
  <tr>    {"convexify_strategy",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "NONE|regularize|eigen-reflect|eigen-clip. "</tr>
  <tr>      "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</tr>
  <tr>    {"convexify_margin",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "When using a convexification strategy, make sure that "</tr>
  <tr>      "the smallest eigenvalue is at least this (default: 1e-7)."}},</tr>
  <tr>    {"max_iter_eig",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</tr>
  <tr>    {"clip_inactive_lam",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Explicitly set Lagrange multipliers to 0 when bound is deemed inactive "</tr>
  <tr>      "(default: false)."}},</tr>
  <tr>    {"inactive_lam_strategy",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Strategy to detect if a bound is inactive. "</tr>
  <tr>      "RELTOL: use solver-defined constraint tolerance * inactive_lam_value|"</tr>
  <tr>      "abstol: use inactive_lam_value"}},</tr>
  <tr>    {"inactive_lam_value",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Value used in inactive_lam_strategy (default: 10)."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/clang/clang_compiler.cpp: const Options ClangCompiler</summary>
  <table><tr><th>= {{&ImporterInternal::options_},</th></tr>
  <tr>   {{"include_path",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Include paths for the JIT compiler. "</tr>
  <tr>      "The include directory shipped with CasADi will be automatically appended."}},</tr>
  <tr>    {"flags",</tr>
  <tr>     {OT_STRINGVECTOR,</tr>
  <tr>      "Compile flags for the JIT compiler. Default: None"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/proxqp/proxqp_interface.cpp: const Options ProxqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"proxqp",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "const proxqp options."}},</tr>
  <tr>    {"warm_start_primal",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use x input to warmstart [Default: true]."}},</tr>
  <tr>    {"warm_start_dual",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use y and z input to warmstart [Default: true]."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/bonmin/bonmin_interface.cpp: const Options BonminInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <tr>   {{"pass_nonlinear_variables",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Pass list of variables entering nonlinearly to BONMIN"}},</tr>
  <tr>    {"pass_nonlinear_constraints",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Pass list of constraints entering nonlinearly to BONMIN"}},</tr>
  <tr>    {"bonmin",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to BONMIN"}},</tr>
  <tr>    {"var_string_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "String metadata (a dictionary with lists of strings) "</tr>
  <tr>      "about variables to be passed to BONMIN"}},</tr>
  <tr>    {"var_integer_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Integer metadata (a dictionary with lists of integers) "</tr>
  <tr>      "about variables to be passed to BONMIN"}},</tr>
  <tr>    {"var_numeric_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Numeric metadata (a dictionary with lists of reals) about "</tr>
  <tr>      "variables to be passed to BONMIN"}},</tr>
  <tr>    {"con_string_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "String metadata (a dictionary with lists of strings) about "</tr>
  <tr>      "constraints to be passed to BONMIN"}},</tr>
  <tr>    {"con_integer_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Integer metadata (a dictionary with lists of integers) "</tr>
  <tr>      "about constraints to be passed to BONMIN"}},</tr>
  <tr>    {"con_numeric_md",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Numeric metadata (a dictionary with lists of reals) about "</tr>
  <tr>      "constraints to be passed to BONMIN"}},</tr>
  <tr>    {"hess_lag",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</tr>
  <tr>    {"hess_lag_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options for the autogenerated Hessian of the Lagrangian."}},</tr>
  <tr>    {"jac_g",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the Jacobian of the constraints "</tr>
  <tr>      "(autogenerated by default)"}},</tr>
  <tr>    {"jac_g_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options for the autogenerated Jacobian of the constraints."}},</tr>
  <tr>    {"grad_f",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function for calculating the gradient of the objective "</tr>
  <tr>      "(column, autogenerated by default)"}},</tr>
  <tr>    {"grad_f_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options for the autogenerated gradient of the objective."}},</tr>
  <tr>    {"sos1_groups",</tr>
  <tr>     {OT_INTVECTORVECTOR,</tr>
  <tr>      "Options for the autogenerated gradient of the objective."}},</tr>
  <tr>    {"sos1_weights",</tr>
  <tr>     {OT_DOUBLEVECTORVECTOR,</tr>
  <tr>      "Options for the autogenerated gradient of the objective."}},</tr>
  <tr>    {"sos1_priorities",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Options for the autogenerated gradient of the objective."}},</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/cbc/cbc_interface.cpp: const Options CbcInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"cbc",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to CBC."</tr>
  <tr>      "Three sets of options are supported. "</tr>
  <tr>      "The first can be found in OsiSolverParameters.hpp. "</tr>
  <tr>      "The second can be found in CbcModel.hpp. "</tr>
  <tr>      "The third are options that can be passed to CbcMain1."</tr>
  <tr>      }},</tr>
  <tr>    {"sos_groups",</tr>
  <tr>     {OT_INTVECTORVECTOR,</tr>
  <tr>      "Definition of SOS groups by indices."}},</tr>
  <tr>    {"sos_weights",</tr>
  <tr>     {OT_DOUBLEVECTORVECTOR,</tr>
  <tr>      "Weights corresponding to SOS entries."}},</tr>
  <tr>    {"sos_types",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Specify 1 or 2 for each SOS group."}},</tr>
  <tr>    {"hot_start",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Hot start with x0 [Default false]."}},</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/osqp/osqp_interface.cpp: const Options OsqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <tr>   {{"osqp",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "const Options to be passed to osqp."}},</tr>
  <tr>    {"warm_start_primal",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use x0 input to warmstart [Default: true]."}},</tr>
  <tr>    {"warm_start_dual",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use lam_a0 and lam_x0 input to warmstart [Default: truw]."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/sx_function.cpp: const Options SXFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"default_in",</tr>
  <tr>     {OT_DOUBLEVECTOR,</tr>
  <tr>      "Default input values"}},</tr>
  <tr>    {"just_in_time_sparsity",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Propagate sparsity patterns using just-in-time "</tr>
  <tr>      "compilation to a CPU or GPU using OpenCL"}},</tr>
  <tr>    {"just_in_time_opencl",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Just-in-time compilation for numeric evaluation using OpenCL (experimental)"}},</tr>
  <tr>    {"live_variables",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Reuse variables in the work vector"}},</tr>
  <tr>    {"cse",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Perform common subexpression elimination (complexity is N*log(N) in graph size)"}},</tr>
  <tr>    {"allow_free",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Allow construction with free variables (Default: false)"}},</tr>
  <tr>    {"allow_duplicate_io_names",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Allow construction with duplicate io names (Default: false)"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options Integrator</summary>
  <table><tr><th>= {{&OracleFunction::options_},</th></tr>
  <tr>   {{"expand",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Replace MX with SX expressions in problem formulation [false]"}},</tr>
  <tr>  {"print_stats",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Print out statistics after integration"}},</tr>
  <tr>  {"nfwd",</tr>
  <tr>   {OT_INT,</tr>
  <tr>    "Number of forward sensitivities to be calculated [0]"}},</tr>
  <tr>  {"nadj",</tr>
  <tr>   {OT_INT,</tr>
  <tr>    "Number of adjoint sensitivities to be calculated [0]"}},</tr>
  <tr>  {"t0",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "[DEPRECATED] Beginning of the time horizon"}},</tr>
  <tr>  {"tf",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "[DEPRECATED] End of the time horizon"}},</tr>
  <tr>  {"grid",</tr>
  <tr>    {OT_DOUBLEVECTOR,</tr>
  <tr>    "[DEPRECATED] Time grid"}},</tr>
  <tr>  {"augmented_options",</tr>
  <tr>    {OT_DICT,</tr>
  <tr>    "Options to be passed down to the augmented integrator, if one is constructed."}},</tr>
  <tr>  {"output_t0",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "[DEPRECATED] Output the state at the initial time"}}</tr>
  <tr>  }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options FixedStepIntegrator</summary>
  <table><tr><th>= {{&Integrator::options_},</th></tr>
  <tr>   {{"number_of_finite_elements",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Target number of finite elements. "</tr>
  <tr>    "The actual number may be higher to accommodate all output times"}},</tr>
  <tr>  {"simplify",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Implement as MX Function (codegeneratable/serializable) default: false"}},</tr>
  <tr>  {"simplify_options",</tr>
  <tr>    {OT_DICT,</tr>
  <tr>    "Any options to pass to simplified form Function constructor"}}</tr>
  <tr>  }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options ImplicitFixedStepIntegrator</summary>
  <table><tr><th>= {{&FixedStepIntegrator::options_},</th></tr>
  <tr>   {{"rootfinder",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>    "An implicit function solver"}},</tr>
  <tr>  {"rootfinder_options",</tr>
  <tr>    {OT_DICT,</tr>
  <tr>    "Options to be passed to the NLP Solver"}}</tr>
  <tr>  }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/conic.cpp: const Options Conic</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"discrete",</tr>
  <tr>     {OT_BOOLVECTOR,</tr>
  <tr>      "Indicates which of the variables are discrete, i.e. integer-valued"}},</tr>
  <tr>    {"print_problem",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print a numeric description of the problem"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/rootfinder.cpp: const Options Rootfinder</summary>
  <table><tr><th>= {{&OracleFunction::options_},</th></tr>
  <tr>   {{"linear_solver",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "User-defined linear solver class. Needed for sensitivities."}},</tr>
  <tr>    {"linear_solver_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to the linear solver."}},</tr>
  <tr>    {"constraints",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "Constrain the unknowns. 0 (default): no constraint on ui, "</tr>
  <tr>      "1: ui >= 0.0, -1: ui <= 0.0, 2: ui > 0.0, -2: ui < 0.0."}},</tr>
  <tr>    {"implicit_input",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Index of the input that corresponds to the actual root-finding"}},</tr>
  <tr>    {"implicit_output",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Index of the output that corresponds to the actual root-finding"}},</tr>
  <tr>    {"jacobian_function",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Function object for calculating the Jacobian (autogenerated by default)"}},</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/mx_function.cpp: const Options MXFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"default_in",</tr>
  <tr>     {OT_DOUBLEVECTOR,</tr>
  <tr>      "Default input values"}},</tr>
  <tr>    {"live_variables",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Reuse variables in the work vector"}},</tr>
  <tr>    {"print_instructions",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print each operation during evaluation"}},</tr>
  <tr>    {"cse",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Perform common subexpression elimination (complexity is N*log(N) in graph size)"}},</tr>
  <tr>    {"allow_free",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Allow construction with free variables (Default: false)"}},</tr>
  <tr>    {"allow_duplicate_io_names",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Allow construction with duplicate io names (Default: false)"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/expm.cpp: const Options Expm</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"const_A",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Assume A is constant. Default: false."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/finite_differences.cpp: const Options FiniteDiff</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"second_order_stepsize",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Second order perturbation size [default: 1e-3]"}},</tr>
  <tr>  {"h",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Step size [default: computed from abstol]"}},</tr>
  <tr>  {"h_max",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Maximum step size [default 0]"}},</tr>
  <tr>  {"h_min",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Minimum step size [default inf]"}},</tr>
  <tr>  {"smoothing",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Smoothing regularization [default: machine precision]"}},</tr>
  <tr>  {"reltol",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Accuracy of function inputs [default: query object]"}},</tr>
  <tr>  {"abstol",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Accuracy of function outputs [default: query object]"}},</tr>
  <tr>  {"u_aim",</tr>
  <tr>    {OT_DOUBLE,</tr>
  <tr>    "Target ratio of roundoff error to truncation error [default: 100.]"}},</tr>
  <tr>  {"h_iter",</tr>
  <tr>    {OT_INT,</tr>
  <tr>    "Number of iterations to improve on the step-size "</tr>
  <tr>    "[default: 1 if error estimate available, otherwise 0]"}},</tr>
  <tr>  }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/jit_function.cpp: const Options JitFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"buffered",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>      "Buffer the calls, user does not need to "}},</tr>
  <tr>     {"jac",</tr>
  <tr>    {OT_STRING,</tr>
  <tr>      "Function body for Jacobian"}},</tr>
  <tr>    {"hess",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Function body for Hessian"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/function_internal.cpp: const Options ProtoFunction</summary>
  <table><tr><th>= {{},</th></tr>
  <tr>   {{"verbose",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Verbose evaluation -- for debugging"}},</tr>
  <tr>    {"print_time",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "print information about execution time. Implies record_time."}},</tr>
  <tr>    {"record_time",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "record information about execution time, for retrieval with stats()."}},</tr>
  <tr>    {"regularity_check",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Throw exceptions when NaN or Inf appears during evaluation"}},</tr>
  <tr>    {"error_on_fail",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Throw exceptions when function evaluation fails (default true)."}}</tr>
  <tr>    }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/function_internal.cpp: const Options FunctionInternal</summary>
  <table><tr><th>= {{&ProtoFunction::options_},</th></tr>
  <tr>   {{"ad_weight",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Weighting factor for derivative calculation."</tr>
  <tr>      "When there is an option of either using forward or reverse mode "</tr>
  <tr>      "directional derivatives, the condition ad_weight*nf<=(1-ad_weight)*na "</tr>
  <tr>      "is used where nf and na are estimates of the number of forward/reverse "</tr>
  <tr>      "mode directional derivatives needed. By default, ad_weight is calculated "</tr>
  <tr>      "automatically, but this can be overridden by setting this option. "</tr>
  <tr>      "In particular, 0 means forcing forward mode and 1 forcing reverse mode. "</tr>
  <tr>      "Leave unset for (class specific) heuristics."}},</tr>
  <tr>    {"ad_weight_sp",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Weighting factor for sparsity pattern calculation calculation."</tr>
  <tr>      "Overrides default behavior. Set to 0 and 1 to force forward and "</tr>
  <tr>      "reverse mode respectively. Cf. option \"ad_weight\". "</tr>
  <tr>      "When set to -1, sparsity is completely ignored and dense matrices are used."}},</tr>
  <tr>    {"always_inline",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Force inlining."}},</tr>
  <tr>    {"never_inline",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Forbid inlining."}},</tr>
  <tr>    {"jac_penalty",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "When requested for a number of forward/reverse directions,   "</tr>
  <tr>      "it may be cheaper to compute first the full jacobian and then "</tr>
  <tr>      "multiply with seeds, rather than obtain the requested directions "</tr>
  <tr>      "in a straightforward manner. "</tr>
  <tr>      "Casadi uses a heuristic to decide which is cheaper. "</tr>
  <tr>      "A high value of 'jac_penalty' makes it less likely for the heurstic "</tr>
  <tr>      "to chose the full Jacobian strategy. "</tr>
  <tr>      "The special value -1 indicates never to use the full Jacobian strategy"}},</tr>
  <tr>    {"user_data",</tr>
  <tr>     {OT_VOIDPTR,</tr>
  <tr>      "A user-defined field that can be used to identify "</tr>
  <tr>      "the function or pass additional information"}},</tr>
  <tr>    {"inputs_check",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Throw exceptions when the numerical values of the inputs don't make sense"}},</tr>
  <tr>    {"gather_stats",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Deprecated option (ignored): Statistics are now always collected."}},</tr>
  <tr>    {"input_scheme",</tr>
  <tr>     {OT_STRINGVECTOR,</tr>
  <tr>      "Deprecated option (ignored)"}},</tr>
  <tr>    {"output_scheme",</tr>
  <tr>     {OT_STRINGVECTOR,</tr>
  <tr>      "Deprecated option (ignored)"}},</tr>
  <tr>    {"jit",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use just-in-time compiler to speed up the evaluation"}},</tr>
  <tr>    {"jit_cleanup",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Cleanup up the temporary source file that jit creates. Default: true"}},</tr>
  <tr>    {"jit_serialize",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Specify behaviour when serializing a jitted function: SOURCE|link|embed."}},</tr>
  <tr>    {"jit_name",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "The file name used to write out code. "</tr>
  <tr>      "The actual file names used depend on 'jit_temp_suffix' and include extensions. "</tr>
  <tr>      "Default: 'jit_tmp'"}},</tr>
  <tr>    {"jit_temp_suffix",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Use a temporary (seemingly random) filename suffix for generated code and libraries. "</tr>
  <tr>      "This is desired for thread-safety. "</tr>
  <tr>      "This behaviour may defeat caching compiler wrappers. "</tr>
  <tr>      "Default: true"}},</tr>
  <tr>    {"compiler",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Just-in-time compiler plugin to be used."}},</tr>
  <tr>    {"jit_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to the jit compiler."}},</tr>
  <tr>    {"derivative_of",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "The function is a derivative of another function. "</tr>
  <tr>      "The type of derivative (directional derivative, Jacobian) "</tr>
  <tr>      "is inferred from the function name."}},</tr>
  <tr>    {"max_num_dir",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Specify the maximum number of directions for derivative functions."</tr>
  <tr>      " Overrules the builtin optimized_num_dir."}},</tr>
  <tr>    {"enable_forward",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Enable derivative calculation using generated functions for"</tr>
  <tr>      " Jacobian-times-vector products - typically using forward mode AD"</tr>
  <tr>      " - if available. [default: true]"}},</tr>
  <tr>    {"enable_reverse",</tr>
  <tr>      {OT_BOOL,</tr>
  <tr>      "Enable derivative calculation using generated functions for"</tr>
  <tr>      " transposed Jacobian-times-vector products - typically using reverse mode AD"</tr>
  <tr>      " - if available. [default: true]"}},</tr>
  <tr>    {"enable_jacobian",</tr>
  <tr>      {OT_BOOL,</tr>
  <tr>      "Enable derivative calculation using generated functions for"</tr>
  <tr>      " Jacobians of all differentiable outputs with respect to all differentiable inputs"</tr>
  <tr>      " - if available. [default: true]"}},</tr>
  <tr>    {"enable_fd",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Enable derivative calculation by finite differencing. [default: false]]"}},</tr>
  <tr>    {"fd_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to the finite difference instance"}},</tr>
  <tr>    {"fd_method",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Method for finite differencing [default 'central']"}},</tr>
  <tr>    {"print_in",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print numerical values of inputs [default: false]"}},</tr>
  <tr>    {"print_out",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print numerical values of outputs [default: false]"}},</tr>
  <tr>    {"max_io",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Acceptable number of inputs and outputs. Warn if exceeded."}},</tr>
  <tr>    {"dump_in",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Dump numerical values of inputs to file (readable with DM.from_file) [default: false]"}},</tr>
  <tr>    {"dump_out",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Dump numerical values of outputs to file (readable with DM.from_file) [default: false]"}},</tr>
  <tr>    {"dump",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Dump function to file upon first evaluation. [false]"}},</tr>
  <tr>    {"dump_dir",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Directory to dump inputs/outputs to. Make sure the directory exists [.]"}},</tr>
  <tr>    {"dump_format",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Choose file format to dump matrices. See DM.from_file [mtx]"}},</tr>
  <tr>    {"forward_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to a forward mode constructor"}},</tr>
  <tr>    {"reverse_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to a reverse mode constructor"}},</tr>
  <tr>    {"jacobian_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to a Jacobian constructor"}},</tr>
  <tr>    {"der_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Default options to be used to populate forward_options, reverse_options, and "</tr>
  <tr>      "jacobian_options before those options are merged in."}},</tr>
  <tr>    {"custom_jacobian",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "Override CasADi's AD. Use together with 'jac_penalty': 0. "</tr>
  <tr>      "Note: Highly experimental. Syntax may break often."}},</tr>
  <tr>    {"is_diff_in",</tr>
  <tr>     {OT_BOOLVECTOR,</tr>
  <tr>      "Indicate for each input if it should be differentiable."}},</tr>
  <tr>    {"is_diff_out",</tr>
  <tr>     {OT_BOOLVECTOR,</tr>
  <tr>      "Indicate for each output if it should be differentiable."}},</tr>
  <tr>    {"post_expand",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "After construction, expand this Function. Default: False"}},</tr>
  <tr>    {"post_expand_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to post-construction expansion. Default: empty"}},</tr>
  <tr>    {"cache",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Prepopulate the function cache. Default: empty"}},</tr>
  <tr>    {"external_transform",</tr>
  <tr>     {OT_VECTORVECTOR,</tr>
  <tr>      "List of external_transform instruction arguments. Default: empty"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/oracle_function.cpp: const Options OracleFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"expand",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Replace MX with SX expressions in problem formulation [false]"}},</tr>
  <tr>  {"monitor",</tr>
  <tr>    {OT_STRINGVECTOR,</tr>
  <tr>    "Set of user problem functions to be monitored"}},</tr>
  <tr>  {"show_eval_warnings",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>    "Show warnings generated from function evaluations [true]"}},</tr>
  <tr>  {"common_options",</tr>
  <tr>    {OT_DICT,</tr>
  <tr>    "Options for auto-generated functions"}},</tr>
  <tr>  {"specific_options",</tr>
  <tr>    {OT_DICT,</tr>
  <tr>    "Options for specific auto-generated functions,"</tr>
  <tr>    " overwriting the defaults from common_options. Nested dictionary."}}</tr>
  <tr>}</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/importer_internal.cpp: const Options ImporterInternal</summary>
  <table><tr><th>= {{},</th></tr>
  <tr>   {{"verbose",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Verbose evaluation -- for debugging"}}</tr>
  <tr>    }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/interpolant.cpp: const Options Interpolant</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"lookup_mode",</tr>
  <tr>     {OT_STRINGVECTOR,</tr>
  <tr>      "Specifies, for each grid dimension, the lookup algorithm used to find the correct index. "</tr>
  <tr>      "'linear' uses a for-loop + break; (default when #knots<=100), "</tr>
  <tr>      "'exact' uses floored division (only for uniform grids), "</tr>
  <tr>      "'binary' uses a binary search. (default when #knots>100)."}},</tr>
  <tr>    {"inline",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Implement the lookup table in MX primitives. "</tr>
  <tr>      "Useful when you need derivatives with respect to grid and/or coefficients. "</tr>
  <tr>      "Such derivatives are fundamentally dense, so use with caution."}},</tr>
  <tr>    {"batch_x",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Evaluate a batch of different inputs at once (default 1)."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/nlpsol.cpp: const Options Nlpsol</summary>
  <table><tr><th>= {{&OracleFunction::options_},</th></tr>
  <tr>   {{"iteration_callback",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "A function that will be called at each iteration with the solver as input. "</tr>
  <tr>      "Check documentation of Callback."}},</tr>
  <tr>    {"iteration_callback_step",</tr>
  <tr>     {OT_INT,</tr>
  <tr>      "Only call the callback function every few iterations."}},</tr>
  <tr>    {"iteration_callback_ignore_errors",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "If set to true, errors thrown by iteration_callback will be ignored."}},</tr>
  <tr>    {"ignore_check_vec",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "If set to true, the input shape of F will not be checked."}},</tr>
  <tr>    {"warn_initial_bounds",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Warn if the initial guess does not satisfy LBX and UBX"}},</tr>
  <tr>    {"eval_errors_fatal",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "When errors occur during evaluation of f,g,...,"</tr>
  <tr>      "stop the iterations"}},</tr>
  <tr>    {"verbose_init",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Print out timing information about "</tr>
  <tr>      "the different stages of initialization"}},</tr>
  <tr>    {"discrete",</tr>
  <tr>     {OT_BOOLVECTOR,</tr>
  <tr>      "Indicates which of the variables are discrete, i.e. integer-valued"}},</tr>
  <tr>    {"calc_multipliers",</tr>
  <tr>    {OT_BOOL,</tr>
  <tr>     "Calculate Lagrange multipliers in the Nlpsol base class"}},</tr>
  <tr>    {"calc_lam_x",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Calculate 'lam_x' in the Nlpsol base class"}},</tr>
  <tr>    {"calc_lam_p",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Calculate 'lam_p' in the Nlpsol base class"}},</tr>
  <tr>    {"calc_f",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Calculate 'f' in the Nlpsol base class"}},</tr>
  <tr>    {"calc_g",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Calculate 'g' in the Nlpsol base class"}},</tr>
  <tr>    {"no_nlp_grad",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Prevent the creation of the 'nlp_grad' function"}},</tr>
  <tr>    {"bound_consistency",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Ensure that primal-dual solution is consistent with the bounds"}},</tr>
  <tr>    {"min_lam",</tr>
  <tr>     {OT_DOUBLE,</tr>
  <tr>      "Minimum allowed multiplier value"}},</tr>
  <tr>    {"oracle_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Options to be passed to the oracle function"}},</tr>
  <tr>    {"sens_linsol",</tr>
  <tr>     {OT_STRING,</tr>
  <tr>      "Linear solver used for parametric sensitivities (default 'qr')."}},</tr>
  <tr>    {"sens_linsol_options",</tr>
  <tr>     {OT_DICT,</tr>
  <tr>      "Linear solver options used for parametric sensitivities."}},</tr>
  <tr>    {"detect_simple_bounds",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Automatically detect simple bounds (lbx/ubx) (default false). "</tr>
  <tr>      "This is hopefully beneficial to speed and robustness but may also have adverse affects: "</tr>
  <tr>      "1) Subtleties in heuristics and stopping criteria may change the solution, "</tr>
  <tr>      "2) IPOPT may lie about multipliers of simple equality bounds unless "</tr>
  <tr>      "'fixed_variable_treatment' is set to 'relax_bounds'."}},</tr>
  <tr>    {"detect_simple_bounds_is_simple",</tr>
  <tr>     {OT_BOOLVECTOR,</tr>
  <tr>      "For internal use only."}},</tr>
  <tr>    {"detect_simple_bounds_parts",</tr>
  <tr>     {OT_FUNCTION,</tr>
  <tr>      "For internal use only."}},</tr>
  <tr>    {"detect_simple_bounds_target_x",</tr>
  <tr>     {OT_INTVECTOR,</tr>
  <tr>      "For internal use only."}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/fmu_function.cpp: const Options FmuFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"scheme_in",</tr>
  <tr>   {OT_STRINGVECTOR,</tr>
  <tr>    "Names of the inputs in the scheme"}},</tr>
  <tr>  {"scheme_out",</tr>
  <tr>   {OT_STRINGVECTOR,</tr>
  <tr>    "Names of the outputs in the scheme"}},</tr>
  <tr>  {"scheme",</tr>
  <tr>   {OT_DICT,</tr>
  <tr>    "Definitions of the scheme variables"}},</tr>
  <tr>  {"aux",</tr>
  <tr>   {OT_STRINGVECTOR,</tr>
  <tr>    "Auxilliary variables"}},</tr>
  <tr>  {"enable_ad",</tr>
  <tr>   {OT_BOOL,</tr>
  <tr>    "Calculate first order derivatives using FMU directional derivative support"}},</tr>
  <tr>  {"validate_ad",</tr>
  <tr>   {OT_BOOL,</tr>
  <tr>    "Compare analytic derivatives with finite differences for validation"}},</tr>
  <tr>  {"validate_ad_file",</tr>
  <tr>   {OT_STRING,</tr>
  <tr>    "Redirect results of Hessian validation to a file instead of generating a warning"}},</tr>
  <tr>  {"check_hessian",</tr>
  <tr>   {OT_BOOL,</tr>
  <tr>    "Symmetry check for Hessian"}},</tr>
  <tr>  {"make_symmetric",</tr>
  <tr>   {OT_BOOL,</tr>
  <tr>    "Ensure Hessian is symmetric"}},</tr>
  <tr>  {"step",</tr>
  <tr>   {OT_DOUBLE,</tr>
  <tr>    "Step size, scaled by nominal value"}},</tr>
  <tr>  {"abstol",</tr>
  <tr>   {OT_DOUBLE,</tr>
  <tr>    "Absolute error tolerance, scaled by nominal value"}},</tr>
  <tr>  {"reltol",</tr>
  <tr>   {OT_DOUBLE,</tr>
  <tr>    "Relative error tolerance"}},</tr>
  <tr>  {"parallelization",</tr>
  <tr>   {OT_STRING,</tr>
  <tr>    "Parallelization [SERIAL|openmp|thread]"}},</tr>
  <tr>  {"print_progress",</tr>
  <tr>   {OT_BOOL,</tr>
  <tr>    "Print progress during Jacobian/Hessian evaluation"}},</tr>
  <tr>  {"new_jacobian",</tr>
  <tr>   {OT_BOOL,</tr>
  <tr>    "Use Jacobian implementation in class"}},</tr>
  <tr>  {"new_hessian",</tr>
  <tr>   {OT_BOOL,</tr>
  <tr>    "Use Hessian implementation in class"}},</tr>
  <tr>  {"hessian_coloring",</tr>
  <tr>   {OT_BOOL,</tr>
  <tr>    "Enable the use of graph coloring (star coloring) for Hessian calculation. "</tr>
  <tr>    "Note that disabling the coloring can improve symmetry check diagnostics."}}</tr>
  <tr> }</tr>
  <tr>  };</tr>
</table></details>

<details>
 <summary>./casadi/casadi/core/dple.cpp: const Options Dple</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <tr>   {{"const_dim",</tr>
  <tr>     {OT_BOOL,</tr>
  <tr>      "Assume constant dimension of P"}},</tr>
  <tr>    {"pos_def",</tr>
  <tr>      {OT_BOOL,</tr>
  <tr>       "Assume P positive definite"}},</tr>
  <tr>    {"error_unstable",</tr>
  <tr>      {OT_BOOL,</tr>
  <tr>      "Throw an exception when it is detected that Product(A_i, i=N..1)"</tr>
  <tr>      "has eigenvalues greater than 1-eps_unstable"}},</tr>
  <tr>    {"eps_unstable",</tr>
  <tr>      {OT_DOUBLE,</tr>
  <tr>      "A margin for unstability detection"}}</tr>
  <tr>   }</tr>
  <tr>  };</tr>
</table></details>
