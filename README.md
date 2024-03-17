# MA24_simulink

<details>
 <summary>./casadi/casadi/solvers/linsol_ldl.cpp: const Options LinsolLdl</summary>
  <table><tr><th>= {{&ProtoFunction::options_},</th></tr>
  <td>   {{"incomplete",</td>
  <td>    {OT_BOOL,</td>
  <td>     "Incomplete factorization, without any fill-in"}},</td>
  <td>    {"preordering",</td>
  <td>     {OT_BOOL,</td>
  <td>     "Approximate minimal degree (AMD) preordering"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/linear_interpolant.cpp: const Options LinearInterpolant</summary>
  <table><tr><th>= {{&Interpolant::options_},</th></tr>
  <td>   {{"lookup_mode",</td>
  <td>     {OT_STRINGVECTOR,</td>
  <td>      "Sets, for each grid dimenion, the lookup algorithm used to find the correct index. "</td>
  <td>      "'linear' uses a for-loop + break; "</td>
  <td>      "'exact' uses floored division (only for uniform grids)."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/ipqp.cpp: const Options Ipqp</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"max_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of iterations [1000]."}},</td>
  <td>    {"constr_viol_tol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Constraint violation tolerance [1e-8]."}},</td>
  <td>    {"dual_inf_tol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Dual feasibility violation tolerance [1e-8]"}},</td>
  <td>    {"print_header",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print header [true]."}},</td>
  <td>    {"print_iter",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print iterations [true]."}},</td>
  <td>    {"print_info",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print info [true]."}},</td>
  <td>    {"linear_solver",</td>
  <td>     {OT_STRING,</td>
  <td>      "A custom linear solver creator function [default: ldl]"}},</td>
  <td>    {"linear_solver_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to the linear solver"}},</td>
  <td>    {"min_lam",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Smallest multiplier treated as inactive for the initial active set [0]."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/qrsqp.cpp: const Options Qrsqp</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"qpsol",</td>
  <td>     {OT_STRING,</td>
  <td>      "The QP solver to be used by the SQP method [qrqp]"}},</td>
  <td>    {"qpsol_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to the QP solver"}},</td>
  <td>    {"hessian_approximation",</td>
  <td>     {OT_STRING,</td>
  <td>      "limited-memory|exact"}},</td>
  <td>    {"max_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of SQP iterations"}},</td>
  <td>    {"min_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Minimum number of SQP iterations"}},</td>
  <td>    {"max_iter_ls",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of linesearch iterations"}},</td>
  <td>    {"tol_pr",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion for primal infeasibility"}},</td>
  <td>    {"tol_du",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion for dual infeasability"}},</td>
  <td>    {"c1",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Armijo condition, coefficient of decrease in merit"}},</td>
  <td>    {"beta",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Line-search parameter, restoration factor of stepsize"}},</td>
  <td>    {"merit_memory",</td>
  <td>     {OT_INT,</td>
  <td>      "Size of memory to store history of merit function values"}},</td>
  <td>    {"lbfgs_memory",</td>
  <td>     {OT_INT,</td>
  <td>      "Size of L-BFGS memory."}},</td>
  <td>    {"regularize",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Automatic regularization of Lagrange Hessian."}},</td>
  <td>    {"print_header",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print the header with problem statistics"}},</td>
  <td>    {"print_iteration",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print the iterations"}},</td>
  <td>    {"min_step_size",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "The size (inf-norm) of the step size should not become smaller than this."}},</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/scpgen.cpp: const Options Scpgen</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"qpsol",</td>
  <td>     {OT_STRING,</td>
  <td>      "The QP solver to be used by the SQP method"}},</td>
  <td>    {"qpsol_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to the QP solver"}},</td>
  <td>    {"hessian_approximation",</td>
  <td>     {OT_STRING,</td>
  <td>      "gauss-newton|exact"}},</td>
  <td>    {"max_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of SQP iterations"}},</td>
  <td>    {"max_iter_ls",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of linesearch iterations"}},</td>
  <td>    {"tol_pr",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion for primal infeasibility"}},</td>
  <td>    {"tol_du",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion for dual infeasability"}},</td>
  <td>    {"tol_reg",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion for regularization"}},</td>
  <td>    {"tol_pr_step",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion for the step size"}},</td>
  <td>    {"c1",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Armijo condition, coefficient of decrease in merit"}},</td>
  <td>    {"beta",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Line-search parameter, restoration factor of stepsize"}},</td>
  <td>    {"merit_memsize",</td>
  <td>     {OT_INT,</td>
  <td>      "Size of memory to store history of merit function values"}},</td>
  <td>    {"merit_start",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Lower bound for the merit function parameter"}},</td>
  <td>    {"lbfgs_memory",</td>
  <td>     {OT_INT,</td>
  <td>      "Size of L-BFGS memory."}},</td>
  <td>    {"regularize",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Automatic regularization of Lagrange Hessian."}},</td>
  <td>    {"print_header",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print the header with problem statistics"}},</td>
  <td>    {"codegen",</td>
  <td>     {OT_BOOL,</td>
  <td>      "C-code generation"}},</td>
  <td>    {"reg_threshold",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Threshold for the regularization."}},</td>
  <td>    {"name_x",</td>
  <td>     {OT_STRINGVECTOR,</td>
  <td>      "Names of the variables."}},</td>
  <td>    {"print_x",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Which variables to print."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/linsol_qr.cpp: const Options LinsolQr</summary>
  <table><tr><th>= {{&LinsolInternal::options_},</th></tr>
  <td>   {{"eps",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Minimum R entry before singularity is declared [1e-12]"}},</td>
  <td>    {"cache",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Amount of factorisations to remember (thread-local) [0]"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/newton.cpp: const Options Newton</summary>
  <table><tr><th>= {{&Rootfinder::options_},</th></tr>
  <td>   {{"abstol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion tolerance on max(|F|)"}},</td>
  <td>    {"abstolStep",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion tolerance on step size"}},</td>
  <td>    {"max_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of Newton iterations to perform before returning."}},</td>
  <td>    {"print_iteration",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print information about each iteration"}},</td>
  <td>    {"line_search",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Enable line-search (default: true)"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/sqpmethod.cpp: const Options Sqpmethod</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"qpsol",</td>
  <td>    {OT_STRING,</td>
  <td>    "The QP solver to be used by the SQP method [qpoases]"}},</td>
  <td>  {"qpsol_options",</td>
  <td>    {OT_DICT,</td>
  <td>    "Options to be passed to the QP solver"}},</td>
  <td>  {"hessian_approximation",</td>
  <td>    {OT_STRING,</td>
  <td>    "limited-memory|exact"}},</td>
  <td>  {"max_iter",</td>
  <td>    {OT_INT,</td>
  <td>    "Maximum number of SQP iterations"}},</td>
  <td>  {"min_iter",</td>
  <td>    {OT_INT,</td>
  <td>    "Minimum number of SQP iterations"}},</td>
  <td>  {"max_iter_ls",</td>
  <td>    {OT_INT,</td>
  <td>    "Maximum number of linesearch iterations"}},</td>
  <td>  {"tol_pr",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Stopping criterion for primal infeasibility"}},</td>
  <td>  {"tol_du",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Stopping criterion for dual infeasability"}},</td>
  <td>  {"c1",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Armijo condition, coefficient of decrease in merit"}},</td>
  <td>  {"beta",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Line-search parameter, restoration factor of stepsize"}},</td>
  <td>  {"merit_memory",</td>
  <td>    {OT_INT,</td>
  <td>    "Size of memory to store history of merit function values"}},</td>
  <td>  {"lbfgs_memory",</td>
  <td>    {OT_INT,</td>
  <td>    "Size of L-BFGS memory."}},</td>
  <td>  {"print_header",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Print the header with problem statistics"}},</td>
  <td>  {"print_iteration",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Print the iterations"}},</td>
  <td>  {"print_status",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Print a status message after solving"}},</td>
  <td>  {"min_step_size",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "The size (inf-norm) of the step size should not become smaller than this."}},</td>
  <td>  {"hess_lag",</td>
  <td>    {OT_FUNCTION,</td>
  <td>    "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</td>
  <td>  {"jac_fg",</td>
  <td>    {OT_FUNCTION,</td>
  <td>    "Function for calculating the gradient of the objective and Jacobian of the constraints "</td>
  <td>    "(autogenerated by default)"}},</td>
  <td>  {"convexify_strategy",</td>
  <td>    {OT_STRING,</td>
  <td>    "NONE|regularize|eigen-reflect|eigen-clip. "</td>
  <td>    "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</td>
  <td>  {"convexify_margin",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "When using a convexification strategy, make sure that "</td>
  <td>    "the smallest eigenvalue is at least this (default: 1e-7)."}},</td>
  <td>  {"max_iter_eig",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</td>
  <td>  {"elastic_mode",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Enable the elastic mode which is used when the QP is infeasible (default: false)."}},</td>
  <td>  {"gamma_0",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Starting value for the penalty parameter of elastic mode (default: 1)."}},</td>
  <td>  {"gamma_max",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Maximum value for the penalty parameter of elastic mode (default: 1e20)."}},</td>
  <td>  {"gamma_1_min",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Minimum value for gamma_1 (default: 1e-5)."}},</td>
  <td>  {"second_order_corrections",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Enable second order corrections. "</td>
  <td>    "These are used when a step is considered bad by the merit function and constraint norm "</td>
  <td>    "(default: false)."}},</td>
  <td>  {"init_feasible",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Initialize the QP subproblems with a feasible initial value (default: false)."}}</td>
  <td>  }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/fast_newton.cpp: const Options FastNewton</summary>
  <table><tr><th>= {{&Rootfinder::options_},</th></tr>
  <td>   {{"abstol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion tolerance on ||g||__inf)"}},</td>
  <td>    {"abstolStep",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion tolerance on step size"}},</td>
  <td>    {"max_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of Newton iterations to perform before returning."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/shell_compiler.cpp: const Options ShellCompiler</summary>
  <table><tr><th>= {{&ImporterInternal::options_},</th></tr>
  <td>   {{"compiler",</td>
  <td>     {OT_STRING,</td>
  <td>      "Compiler command"}},</td>
  <td>    {"linker",</td>
  <td>     {OT_STRING,</td>
  <td>      "Linker command"}},</td>
  <td>    {"directory",</td>
  <td>     {OT_STRING,</td>
  <td>      "Directory to put temporary objects in. Must end with a file separator."}},</td>
  <td>    {"compiler_setup",</td>
  <td>     {OT_STRING,</td>
  <td>      "Compiler setup command. Intended to be fixed."</td>
  <td>      " The 'flag' option is the prefered way to set"</td>
  <td>      " custom flags."}},</td>
  <td>    {"linker_setup",</td>
  <td>     {OT_STRING,</td>
  <td>      "Linker setup command. Intended to be fixed."</td>
  <td>      " The 'flag' option is the prefered way to set"</td>
  <td>      " custom flags."}},</td>
  <td>    {"compiler_flags",</td>
  <td>     {OT_STRINGVECTOR,</td>
  <td>      "Alias for 'compiler_flags'"}},</td>
  <td>    {"flags",</td>
  <td>      {OT_STRINGVECTOR,</td>
  <td>      "Compile flags for the JIT compiler. Default: None"}},</td>
  <td>    {"linker_flags",</td>
  <td>     {OT_STRINGVECTOR,</td>
  <td>      "Linker flags for the JIT compiler. Default: None"}},</td>
  <td>    {"cleanup",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Cleanup temporary files when unloading. Default: true"}},</td>
  <td>    {"compiler_output_flag",</td>
  <td>     {OT_STRING,</td>
  <td>     "Compiler flag to denote object output. Default: '-o '"}},</td>
  <td>    {"linker_output_flag",</td>
  <td>     {OT_STRING,</td>
  <td>     "Linker flag to denote shared library output. Default: '-o '"}},</td>
  <td>    {"extra_suffixes",</td>
  <td>     {OT_STRINGVECTOR,</td>
  <td>     "List of suffixes for extra files that the compiler may generate. Default: None"}},</td>
  <td>    {"name",</td>
  <td>     {OT_STRING,</td>
  <td>      "The file name used to write out compiled objects/libraries. "</td>
  <td>      "The actual file names used depend on 'temp_suffix' and include extensions. "</td>
  <td>      "Default: 'tmp_casadi_compiler_shell'"}},</td>
  <td>    {"temp_suffix",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use a temporary (seemingly random) filename suffix for file names. "</td>
  <td>      "This is desired for thread-safety. "</td>
  <td>      "This behaviour may defeat caching compiler wrappers. "</td>
  <td>      "Default: true"}},</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/implicit_to_nlp.cpp: const Options ImplicitToNlp</summary>
  <table><tr><th>= {{&Rootfinder::options_},</th></tr>
  <td>   {{"nlpsol",</td>
  <td>     {OT_STRING,</td>
  <td>      "Name of solver."}},</td>
  <td>    {"nlpsol_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to solver."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/qrqp.cpp: const Options Qrqp</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"max_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of iterations [1000]."}},</td>
  <td>    {"constr_viol_tol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Constraint violation tolerance [1e-8]."}},</td>
  <td>    {"dual_inf_tol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Dual feasibility violation tolerance [1e-8]"}},</td>
  <td>    {"print_header",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print header [true]."}},</td>
  <td>    {"print_iter",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print iterations [true]."}},</td>
  <td>    {"print_info",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print info [true]."}},</td>
  <td>    {"print_lincomb",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print dependant linear combinations of constraints [false]. "</td>
  <td>      "Printed numbers are 0-based indices into the vector of [simple bounds;linear bounds]"}},</td>
  <td>    {"min_lam",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Smallest multiplier treated as inactive for the initial active set [0]."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/feasiblesqpmethod.cpp: const Options Feasiblesqpmethod</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"solve_type",</td>
  <td>     {OT_STRING,</td>
  <td>      "The solver type: Either SQP or SLP. Defaults to SQP"}},</td>
  <td>    {"qpsol",</td>
  <td>     {OT_STRING,</td>
  <td>      "The QP solver to be used by the SQP method [qpoases]"}},</td>
  <td>    {"qpsol_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to the QP solver"}},</td>
  <td>    {"hessian_approximation",</td>
  <td>     {OT_STRING,</td>
  <td>      "limited-memory|exact"}},</td>
  <td>    {"max_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of SQP iterations"}},</td>
  <td>    {"min_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Minimum number of SQP iterations"}},</td>
  <td>    {"tol_pr",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion for primal infeasibility"}},</td>
  <td>    {"tol_du",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion for dual infeasability"}},</td>
  <td>    {"merit_memory",</td>
  <td>     {OT_INT,</td>
  <td>      "Size of memory to store history of merit function values"}},</td>
  <td>    {"lbfgs_memory",</td>
  <td>     {OT_INT,</td>
  <td>      "Size of L-BFGS memory."}},</td>
  <td>    {"print_header",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print the header with problem statistics"}},</td>
  <td>    {"print_iteration",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print the iterations"}},</td>
  <td>    {"print_status",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print a status message after solving"}},</td>
  <td>    {"f",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the objective function (autogenerated by default)"}},</td>
  <td>    {"g",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the constraints (autogenerated by default)"}},</td>
  <td>    {"grad_f",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the gradient of the objective (autogenerated by default)"}},</td>
  <td>    {"jac_g",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the Jacobian of the constraints (autogenerated by default)"}},</td>
  <td>    {"hess_lag",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</td>
  <td>    {"convexify_strategy",</td>
  <td>     {OT_STRING,</td>
  <td>      "NONE|regularize|eigen-reflect|eigen-clip. "</td>
  <td>      "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</td>
  <td>    {"convexify_margin",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "When using a convexification strategy, make sure that "</td>
  <td>      "the smallest eigenvalue4 is at least this (default: 1e-7)."}},</td>
  <td>    {"max_iter_eig",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</td>
  <td>    {"init_feasible",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Initialize the QP subproblems with a feasible initial value (default: false)."}},</td>
  <td>    {"optim_tol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Optimality tolerance. Below this value an iterate is considered to be optimal."}},</td>
  <td>    {"feas_tol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Feasibility tolerance. Below this tolerance an iterate is considered to be feasible."}},</td>
  <td>    {"tr_rad0",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Initial trust-region radius."}},</td>
  <td>    {"tr_eta1",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Lower eta in trust-region acceptance criterion."}},</td>
  <td>    {"tr_eta2",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Upper eta in trust-region acceptance criterion."}},</td>
  <td>    {"tr_alpha1",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Lower alpha in trust-region size criterion."}},</td>
  <td>    {"tr_alpha2",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Upper alpha in trust-region size criterion."}},</td>
  <td>    {"tr_tol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Trust-region tolerance. "</td>
  <td>      "Below this value another scalar is equal to the trust region radius."}},</td>
  <td>    {"tr_acceptance",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Is the trust-region ratio above this value, the step is accepted."}},</td>
  <td>    {"tr_rad_min",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Minimum trust-region radius."}},</td>
  <td>    {"tr_rad_max",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Maximum trust-region radius."}},</td>
  <td>    {"tr_scale_vector",</td>
  <td>     {OT_DOUBLEVECTOR,</td>
  <td>      "Vector that tells where trust-region is applied."}},</td>
  <td>    {"contraction_acceptance_value",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "If the empirical contraction rate in the feasibility iterations "</td>
  <td>      "is above this value in the heuristics the iterations are aborted."}},</td>
  <td>    {"watchdog",</td>
  <td>     {OT_INT,</td>
  <td>      "Number of watchdog iterations in feasibility iterations. "</td>
  <td>      "After this amount of iterations, it is checked with the contraction acceptance value, "</td>
  <td>      "if iterations are converging."}},</td>
  <td>    {"max_inner_iter",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Maximum number of inner iterations."}},</td>
  <td>    {"use_anderson",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use Anderson Acceleration. (default false)"}},</td>
  <td>    {"anderson_memory",</td>
  <td>     {OT_INT,</td>
  <td>      "Anderson memory. If Anderson is used default is 1, else default is 0."}},</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/collocation.cpp: const Options Collocation</summary>
  <table><tr><th>= {{&ImplicitFixedStepIntegrator::options_},</th></tr>
  <td>   {{"interpolation_order",</td>
  <td>     {OT_INT,</td>
  <td>      "Order of the interpolating polynomials"}},</td>
  <td>    {"collocation_scheme",</td>
  <td>     {OT_STRING,</td>
  <td>      "Collocation scheme: radau|legendre"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/symbolic_qr.cpp: const Options SymbolicQr</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"fopts",</td>
  <td>    {OT_DICT,</td>
  <td>     "Options to be passed to generated function objects"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/bspline_interpolant.cpp: const Options BSplineInterpolant</summary>
  <table><tr><th>= {{&Interpolant::options_},</th></tr>
  <td>   {{"degree",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Sets, for each grid dimension, the degree of the spline."}},</td>
  <td>     {"linear_solver",</td>
  <td>      {OT_STRING,</td>
  <td>       "Solver used for constructing the coefficient tensor."}},</td>
  <td>     {"linear_solver_options",</td>
  <td>      {OT_DICT,</td>
  <td>       "Options to be passed to the linear solver."}},</td>
  <td>     {"algorithm",</td>
  <td>      {OT_STRING,</td>
  <td>       "Algorithm used for fitting the data: 'not_a_knot' (default, same as Matlab),"</td>
  <td>      " 'smooth_linear'."}},</td>
  <td>     {"smooth_linear_frac",</td>
  <td>      {OT_DOUBLE,</td>
  <td>       "When 'smooth_linear' algorithm is active, determines sharpness between"</td>
  <td>       " 0 (sharp, as linear interpolation) and 0.5 (smooth)."</td>
  <td>       "Default value is 0.1."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/solvers/qp_to_nlp.cpp: const Options QpToNlp</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"nlpsol",</td>
  <td>     {OT_STRING,</td>
  <td>      "Name of solver."}},</td>
  <td>    {"nlpsol_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to solver."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/hpipm/hpipm_interface.cpp: const Options HpipmInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"N",</td>
  <td>     {OT_INT,</td>
  <td>      "OCP horizon"}},</td>
  <td>    {"nx",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Number of states, length N+1"}},</td>
  <td>    {"nu",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Number of controls, length N"}},</td>
  <td>    {"ng",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Number of non-dynamic constraints, length N+1"}},</td>
  <td>    {"inf",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Replace infinities by this amount [default: 1e8]"}},</td>
  <td>    {"hpipm",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to hpipm"}}}</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/lapack/lapack_lu.cpp: const Options LapackLu</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"equilibration",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Equilibrate the matrix"}},</td>
  <td>    {"allow_equilibration_failure",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Non-fatal error when equilibration fails"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/lapack/lapack_qr.cpp: const Options LapackQr</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"max_nrhs",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of right-hand-sides that get processed in a single pass [default:10]."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/highs/highs_interface.cpp: const Options HighsInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"highs",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to HiGHS."</td>
  <td>      }},</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/alpaqa/alpaqa_interface.cpp: const Options AlpaqaInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"alpaqa",</td>
  <td>      {OT_DICT,</td>
  <td>      "Options to be passed to Alpaqa"}}</td>
  <td>    }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/mumps/mumps_interface.cpp: const Options MumpsInterface</summary>
  <table><tr><th>= {{&ProtoFunction::options_},</th></tr>
  <td>   {{"symmetric",</td>
  <td>    {OT_BOOL,</td>
  <td>     "Symmetric matrix"}},</td>
  <td>    {"posdef",</td>
  <td>     {OT_BOOL,</td>
  <td>     "Positive definite"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/daqp/daqp_interface.cpp: const Options DaqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"daqp",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to Daqp."</td>
  <td>      }},</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/worhp/worhp_interface.cpp: const Options WorhpInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"worhp",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to WORHP"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/cvodes_interface.cpp: const Options CvodesInterface</summary>
  <table><tr><th>= {{&SundialsInterface::options_},</th></tr>
  <td>   {{"linear_multistep_method",</td>
  <td>    {OT_STRING,</td>
  <td>    "Integrator scheme: BDF|adams"}},</td>
  <td>  {"nonlinear_solver_iteration",</td>
  <td>    {OT_STRING,</td>
  <td>    "Nonlinear solver type: NEWTON|functional"}},</td>
  <td>  {"min_step_size",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Min step size [default: 0/0.0]"}},</td>
  <td>  {"fsens_all_at_once",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Calculate all right hand sides of the sensitivity equations at once"}},</td>
  <td>  {"always_recalculate_jacobian",</td>
  <td>   {OT_BOOL,</td>
  <td>    "Recalculate Jacobian before factorizations, even if Jacobian is current [default: true]"}}</td>
  <td>  }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/kinsol_interface.cpp: const Options KinsolInterface</summary>
  <table><tr><th>= {{&Rootfinder::options_},</th></tr>
  <td>   {{"max_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of Newton iterations. Putting 0 sets the default value of KinSol."}},</td>
  <td>    {"print_level",</td>
  <td>     {OT_INT,</td>
  <td>      "Verbosity level"}},</td>
  <td>    {"abstol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Stopping criterion tolerance"}},</td>
  <td>    {"linear_solver_type",</td>
  <td>     {OT_STRING,</td>
  <td>      "dense|banded|iterative|user_defined"}},</td>
  <td>    {"upper_bandwidth",</td>
  <td>     {OT_INT,</td>
  <td>      "Upper bandwidth for banded linear solvers"}},</td>
  <td>    {"lower_bandwidth",</td>
  <td>     {OT_INT,</td>
  <td>      "Lower bandwidth for banded linear solvers"}},</td>
  <td>    {"max_krylov",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum Krylov space dimension"}},</td>
  <td>    {"exact_jacobian",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use exact Jacobian information"}},</td>
  <td>    {"iterative_solver",</td>
  <td>     {OT_STRING,</td>
  <td>      "gmres|bcgstab|tfqmr"}},</td>
  <td>    {"f_scale",</td>
  <td>     {OT_DOUBLEVECTOR,</td>
  <td>      "Equation scaling factors"}},</td>
  <td>    {"u_scale",</td>
  <td>     {OT_DOUBLEVECTOR,</td>
  <td>      "Variable scaling factors"}},</td>
  <td>    {"pretype",</td>
  <td>     {OT_STRING,</td>
  <td>      "Type of preconditioner"}},</td>
  <td>    {"use_preconditioner",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Precondition an iterative solver"}},</td>
  <td>    {"strategy",</td>
  <td>     {OT_STRING,</td>
  <td>      "Globalization strategy"}},</td>
  <td>    {"disable_internal_warnings",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Disable KINSOL internal warning messages"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/idas_interface.cpp: const Options IdasInterface</summary>
  <table><tr><th>= {{&SundialsInterface::options_},</th></tr>
  <td>   {{"suppress_algebraic",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Suppress algebraic variables in the error testing"}},</td>
  <td>  {"calc_ic",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Use IDACalcIC to get consistent initial conditions."}},</td>
  <td>  {"constraints",</td>
  <td>    {OT_INTVECTOR,</td>
  <td>    "Constrain the solution y=[x,z]. 0 (default): no constraint on yi, "</td>
  <td>    "1: yi >= 0.0, -1: yi <= 0.0, 2: yi > 0.0, -2: yi < 0.0."}},</td>
  <td>  {"calc_icB",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Use IDACalcIC to get consistent initial conditions for "</td>
  <td>    "backwards system [default: equal to calc_ic]."}},</td>
  <td>  {"abstolv",</td>
  <td>    {OT_DOUBLEVECTOR,</td>
  <td>    "Absolute tolerarance for each component"}},</td>
  <td>  {"max_step_size",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Maximim step size"}},</td>
  <td>  {"first_time",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "First requested time as a fraction of the time interval"}},</td>
  <td>  {"cj_scaling",</td>
  <td>    {OT_BOOL,</td>
  <td>    "IDAS scaling on cj for the user-defined linear solver module"}},</td>
  <td>  {"init_xdot",</td>
  <td>    {OT_DOUBLEVECTOR,</td>
  <td>    "Initial values for the state derivatives"}}</td>
  <td>  }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/sundials_interface.cpp: const Options SundialsInterface</summary>
  <table><tr><th>= {{&Integrator::options_},</th></tr>
  <td>   {{"max_num_steps",</td>
  <td>    {OT_INT,</td>
  <td>    "Maximum number of integrator steps"}},</td>
  <td>  {"reltol",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Relative tolerence for the IVP solution"}},</td>
  <td>  {"abstol",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Absolute tolerence for the IVP solution"}},</td>
  <td>  {"newton_scheme",</td>
  <td>    {OT_STRING,</td>
  <td>    "Linear solver scheme in the Newton method: DIRECT|gmres|bcgstab|tfqmr"}},</td>
  <td>  {"max_krylov",</td>
  <td>    {OT_INT,</td>
  <td>    "Maximum Krylov subspace size"}},</td>
  <td>  {"sensitivity_method",</td>
  <td>    {OT_STRING,</td>
  <td>    "Sensitivity method: SIMULTANEOUS|staggered"}},</td>
  <td>  {"max_multistep_order",</td>
  <td>    {OT_INT,</td>
  <td>    "Maximum order for the (variable-order) multistep method"}},</td>
  <td>  {"use_preconditioner",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Precondition the iterative solver [default: true]"}},</td>
  <td>  {"stop_at_end",</td>
  <td>    {OT_BOOL,</td>
  <td>    "[DEPRECATED] Stop the integrator at the end of the interval"}},</td>
  <td>  {"disable_internal_warnings",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Disable SUNDIALS internal warning messages"}},</td>
  <td>  {"quad_err_con",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Should the quadratures affect the step size control"}},</td>
  <td>  {"fsens_err_con",</td>
  <td>    {OT_BOOL,</td>
  <td>    "include the forward sensitivities in all error controls"}},</td>
  <td>  {"steps_per_checkpoint",</td>
  <td>    {OT_INT,</td>
  <td>    "Number of steps between two consecutive checkpoints"}},</td>
  <td>  {"interpolation_type",</td>
  <td>    {OT_STRING,</td>
  <td>    "Type of interpolation for the adjoint sensitivities"}},</td>
  <td>  {"linear_solver",</td>
  <td>    {OT_STRING,</td>
  <td>    "A custom linear solver creator function [default: qr]"}},</td>
  <td>  {"linear_solver_options",</td>
  <td>    {OT_DICT,</td>
  <td>    "Options to be passed to the linear solver"}},</td>
  <td>  {"second_order_correction",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Second order correction in the augmented system Jacobian [true]"}},</td>
  <td>  {"step0",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "initial step size [default: 0/estimated]"}},</td>
  <td>  {"max_step_size",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Max step size [default: 0/inf]"}},</td>
  <td>  {"max_order",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Maximum order"}},</td>
  <td>  {"nonlin_conv_coeff",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Coefficient in the nonlinear convergence test"}},</td>
  <td>  {"scale_abstol",</td>
  <td>   {OT_BOOL,</td>
  <td>    "Scale absolute tolerance by nominal value"}}</td>
  <td>  }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/gurobi/gurobi_interface.cpp: const Options GurobiInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"vtype",</td>
  <td>     {OT_STRINGVECTOR,</td>
  <td>      "Type of variables: [CONTINUOUS|binary|integer|semicont|semiint]"}},</td>
  <td>    {"gurobi",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to gurobi."}},</td>
  <td>    {"sos_groups",</td>
  <td>     {OT_INTVECTORVECTOR,</td>
  <td>      "Definition of SOS groups by indices."}},</td>
  <td>    {"sos_weights",</td>
  <td>     {OT_DOUBLEVECTORVECTOR,</td>
  <td>      "Weights corresponding to SOS entries."}},</td>
  <td>    {"sos_types",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Specify 1 or 2 for each SOS group."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/cplex/cplex_interface.cpp: const Options CplexInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"cplex",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to CPLEX"}},</td>
  <td>    {"qp_method",</td>
  <td>     {OT_INT,</td>
  <td>      "Determines which CPLEX algorithm to use."}},</td>
  <td>    {"dump_to_file",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Dumps QP to file in CPLEX format."}},</td>
  <td>    {"dump_filename",</td>
  <td>     {OT_STRING,</td>
  <td>      "The filename to dump to."}},</td>
  <td>    {"tol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Tolerance of solver"}},</td>
  <td>    {"dep_check",</td>
  <td>     {OT_INT,</td>
  <td>      "Detect redundant constraints."}},</td>
  <td>    {"warm_start",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use warm start with simplex methods (affects only the simplex methods)."}},</td>
  <td>    {"mip_start",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Hot start integers with x0 [Default false]."}},</td>
  <td>    {"sos_groups",</td>
  <td>     {OT_INTVECTORVECTOR,</td>
  <td>      "Definition of SOS groups by indices."}},</td>
  <td>    {"sos_weights",</td>
  <td>     {OT_DOUBLEVECTORVECTOR,</td>
  <td>      "Weights corresponding to SOS entries."}},</td>
  <td>    {"sos_types",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Specify 1 or 2 for each SOS group."}},</td>
  <td>    {"version_suffix",</td>
  <td>     {OT_STRING,</td>
  <td>      "Specify version of cplex to load. "</td>
  <td>      "We will attempt to load libcplex<version_suffix>.[so|dll|dylib]. "</td>
  <td>      "Default value is taken from CPLEX_VERSION env variable."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/ampl/ampl_interface.cpp: const Options AmplInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"solver",</td>
  <td>     {OT_STRING,</td>
  <td>      "AMPL solver binary"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/fatrop/fatrop_conic_interface.cpp: const Options FatropConicInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"N",</td>
  <td>     {OT_INT,</td>
  <td>      "OCP horizon"}},</td>
  <td>    {"nx",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Number of states, length N+1"}},</td>
  <td>    {"nu",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Number of controls, length N"}},</td>
  <td>    {"ng",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Number of non-dynamic constraints, length N+1"}},</td>
  <td>    {"fatrop",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to fatrop"}}}</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/qpoases/qpoases_interface.cpp: const Options QpoasesInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"sparse",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Formulate the QP using sparse matrices. [false]"}},</td>
  <td>    {"schur",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use Schur Complement Approach [false]"}},</td>
  <td>    {"hessian_type",</td>
  <td>     {OT_STRING,</td>
  <td>      "Type of Hessian - see qpOASES documentation "</td>
  <td>      "[UNKNOWN|posdef|semidef|indef|zero|identity]]"}},</td>
  <td>    {"max_schur",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximal number of Schur updates [75]"}},</td>
  <td>    {"linsol_plugin",</td>
  <td>     {OT_STRING,</td>
  <td>      "Linear solver plugin"}},</td>
  <td>    {"nWSR",</td>
  <td>     {OT_INT,</td>
  <td>      "The maximum number of working set recalculations to be performed during "</td>
  <td>      "the initial homotopy. Default is 5(nx + nc)"}},</td>
  <td>    {"CPUtime",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "The maximum allowed CPU time in seconds for the whole initialisation"</td>
  <td>      " (and the actually required one on output). Disabled if unset."}},</td>
  <td>    {"printLevel",</td>
  <td>     {OT_STRING,</td>
  <td>      "Defines the amount of text output during QP solution, see Section 5.7"}},</td>
  <td>    {"enableRamping",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Enables ramping."}},</td>
  <td>    {"enableFarBounds",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Enables the use of  far bounds."}},</td>
  <td>    {"enableFlippingBounds",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Enables the use of  flipping bounds."}},</td>
  <td>    {"enableRegularisation",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Enables automatic  Hessian regularisation."}},</td>
  <td>    {"enableFullLITests",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Enables condition-hardened  (but more expensive) LI test."}},</td>
  <td>    {"enableNZCTests",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Enables nonzero curvature  tests."}},</td>
  <td>    {"enableDriftCorrection",</td>
  <td>     {OT_INT,</td>
  <td>      "Specifies the frequency of drift corrections: 0: turns them off."}},</td>
  <td>    {"enableCholeskyRefactorisation",</td>
  <td>     {OT_INT,</td>
  <td>      "Specifies the frequency of a full re-factorisation of projected "</td>
  <td>      "Hessian matrix: 0: turns them off,  1: uses them at each iteration etc."}},</td>
  <td>    {"enableEqualities",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Specifies whether equalities should be treated  as always active "</td>
  <td>      "(True) or not (False)"}},</td>
  <td>    {"terminationTolerance",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Relative termination tolerance to stop homotopy."}},</td>
  <td>    {"boundTolerance",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "If upper and lower bounds differ less than this tolerance, they are regarded "</td>
  <td>      "equal, i.e. as  equality constraint."}},</td>
  <td>    {"boundRelaxation",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Initial relaxation of bounds to start homotopy  and initial value for far bounds."}},</td>
  <td>    {"epsNum",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Numerator tolerance for ratio tests."}},</td>
  <td>    {"epsDen",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Denominator tolerance for ratio tests."}},</td>
  <td>    {"maxPrimalJump",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Maximum allowed jump in primal variables in  nonzero curvature tests."}},</td>
  <td>    {"maxDualJump",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Maximum allowed jump in dual variables in  linear independence tests."}},</td>
  <td>    {"initialRamping",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Start value for ramping strategy."}},</td>
  <td>    {"finalRamping",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Final value for ramping strategy."}},</td>
  <td>    {"initialFarBounds",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Initial size for far bounds."}},</td>
  <td>    {"growFarBounds",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Factor to grow far bounds."}},</td>
  <td>    {"initialStatusBounds",</td>
  <td>     {OT_STRING,</td>
  <td>      "Initial status of bounds at first iteration."}},</td>
  <td>    {"epsFlipping",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Tolerance of squared Cholesky diagonal factor  which triggers flipping bound."}},</td>
  <td>    {"numRegularisationSteps",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of successive regularisation steps."}},</td>
  <td>    {"epsRegularisation",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Scaling factor of identity matrix used for  Hessian regularisation."}},</td>
  <td>    {"numRefinementSteps",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of iterative refinement steps."}},</td>
  <td>    {"epsIterRef",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Early termination tolerance for iterative  refinement."}},</td>
  <td>    {"epsLITests",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Tolerance for linear independence tests."}},</td>
  <td>    {"epsNZCTests",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Tolerance for nonzero curvature tests."}},</td>
  <td>    {"enableInertiaCorrection",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Should working set be repaired when negative curvature is discovered during hotstart."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/clp/clp_interface.cpp: const Options ClpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"clp",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to CLP. "</td>
  <td>      "A first set of options can be found in ClpParameters.hpp. eg. 'PrimalTolerance'. "</td>
  <td>      "There are other options in additions. "</td>
  <td>      "'AutomaticScaling' (bool) is recognised. "</td>
  <td>      "'initial_solve' (default off) activates the use of Clp's initialSolve. "</td>
  <td>      "'initial_solve_options' takes a dictionary with following keys (see ClpSolve.hpp): "</td>
  <td>      " SolveType (string), PresolveType (string), "</td>
  <td>      " NumberPasses, SpecialOptions (intvectorvector), IndependentOptions (intvectorvector)."</td>
  <td>      }}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/snopt/snopt_interface.cpp: const Options SnoptInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"snopt",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to SNOPT"}},</td>
  <td>    {"start",</td>
  <td>     {OT_STRING,</td>
  <td>      "Warm-start options for Worhp: cold|warm|hot"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/knitro/knitro_interface.cpp: const Options KnitroInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"knitro",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to KNITRO"}},</td>
  <td>    {"options_file",</td>
  <td>     {OT_STRING,</td>
  <td>      "Read options from file (solver specific)"}},</td>
  <td>    {"detect_linear_constraints",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Detect type of constraints"}},</td>
  <td>    {"contype",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Type of constraint"}},</td>
  <td>    {"complem_variables",</td>
  <td>     {OT_INTVECTORVECTOR,</td>
  <td>      "List of complementary constraints on simple bounds. "</td>
  <td>      "Pair (i, j) encodes complementarity between the bounds on variable i and variable j."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/ooqp/ooqp_interface.cpp: const Options OoqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"print_level",</td>
  <td>     {OT_INT,</td>
  <td>      "Print level. OOQP listens to print_level 0, 10 and 100"}},</td>
  <td>    {"mutol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "tolerance as provided with setMuTol to OOQP"}},</td>
  <td>    {"artol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "tolerance as provided with setArTol to OOQP"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/sleqp/sleqp_interface.cpp: const Options SLEQPInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"sleqp",</td>
  <td>      {OT_DICT,</td>
  <td>      "Options to be passed to SLEQP"}},</td>
  <td>     {"print_level",</td>
  <td>      {OT_INT,</td>
  <td>      "Print level of SLEQP (default: 2/SLEQP_LOG_WARN)"}},</td>
  <td>     {"max_iter",</td>
  <td>      {OT_INT,</td>
  <td>      "Maximum number of iterations"}},</td>
  <td>     {"max_wall_time",</td>
  <td>      {OT_DOUBLE,</td>
  <td>      "maximum wall time allowed"}}</td>
  <td>    }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/superscs/superscs_interface.cpp: const Options SuperscsInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"superscs",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to superscs."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/hpmpc/hpmpc_interface.cpp: const Options HpmpcInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"N",</td>
  <td>     {OT_INT,</td>
  <td>      "OCP horizon"}},</td>
  <td>    {"nx",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Number of states, length N+1"}},</td>
  <td>    {"nu",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Number of controls, length N"}},</td>
  <td>    {"ng",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Number of non-dynamic constraints, length N+1"}},</td>
  <td>    {"mu0",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Max element in cost function as estimate of max multiplier"}},</td>
  <td>    {"max_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Max number of iterations"}},</td>
  <td>    {"tol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Tolerance in the duality measure"}},</td>
  <td>    {"warm_start",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use warm-starting"}},</td>
  <td>    {"inf",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "HPMPC cannot handle infinities. Infinities will be replaced by this option's value."}},</td>
  <td>    {"print_level",</td>
  <td>     {OT_INT,</td>
  <td>      "Amount of diagnostic printing [Default: 1]."}},</td>
  <td>    {"target",</td>
  <td>     {OT_STRING,</td>
  <td>      "hpmpc target"}},</td>
  <td>    {"blasfeo_target",</td>
  <td>     {OT_STRING,</td>
  <td>      "hpmpc target"}}}</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/slicot/slicot_dple.cpp: const Options SlicotDple</summary>
  <table><tr><th>= {{&Dple::options_},</th></tr>
  <td>   {{"linear_solver",</td>
  <td>     {OT_STRING,</td>
  <td>      "User-defined linear solver class. Needed for sensitivities."}},</td>
  <td>    {"linear_solver_options",</td>
  <td>      {OT_DICT,</td>
  <td>       "Options to be passed to the linear solver."}},</td>
  <td>    {"psd_num_zero",</td>
  <td>      {OT_DOUBLE,</td>
  <td>        "Numerical zero used in Periodic Schur decomposition with slicot."</td>
  <td>        "This option is needed when your systems has Floquet multipliers"</td>
  <td>        "zero or close to zero"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/blocksqp/blocksqp.cpp: const Options Blocksqp</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"qpsol",</td>
  <td>     {OT_STRING,</td>
  <td>      "The QP solver to be used by the SQP method"}},</td>
  <td>    {"qpsol_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to the QP solver"}},</td>
  <td>    {"linsol",</td>
  <td>     {OT_STRING,</td>
  <td>      "The linear solver to be used by the QP method"}},</td>
  <td>    {"print_header",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print solver header at startup"}},</td>
  <td>    {"print_iteration",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print SQP iterations"}},</td>
  <td>    {"eps",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Values smaller than this are regarded as numerically zero"}},</td>
  <td>    {"opttol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Optimality tolerance"}},</td>
  <td>    {"nlinfeastol",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Nonlinear feasibility tolerance"}},</td>
  <td>    {"schur",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use qpOASES Schur compliment approach"}},</td>
  <td>    {"globalization",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Enable globalization"}},</td>
  <td>    {"restore_feas",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use feasibility restoration phase"}},</td>
  <td>    {"max_line_search",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of steps in line search"}},</td>
  <td>    {"max_consec_reduced_steps",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of consecutive reduced steps"}},</td>
  <td>    {"max_consec_skipped_updates",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of consecutive skipped updates"}},</td>
  <td>    {"max_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of SQP iterations"}},</td>
  <td>    {"warmstart",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use warmstarting"}},</td>
  <td>    {"qp_init",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use warmstarting"}},</td>
  <td>    {"max_it_qp",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of QP iterations per SQP iteration"}},</td>
  <td>    {"block_hess",</td>
  <td>     {OT_INT,</td>
  <td>      "Blockwise Hessian approximation?"}},</td>
  <td>    {"hess_scaling",</td>
  <td>     {OT_INT,</td>
  <td>      "Scaling strategy for Hessian approximation"}},</td>
  <td>    {"fallback_scaling",</td>
  <td>     {OT_INT,</td>
  <td>      "If indefinite update is used, the type of fallback strategy"}},</td>
  <td>    {"max_time_qp",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Maximum number of time in seconds per QP solve per SQP iteration"}},</td>
  <td>    {"ini_hess_diag",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Initial Hessian guess: diagonal matrix diag(iniHessDiag)"}},</td>
  <td>    {"col_eps",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Epsilon for COL scaling strategy"}},</td>
  <td>    {"col_tau1",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "tau1 for COL scaling strategy"}},</td>
  <td>    {"col_tau2",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "tau2 for COL scaling strategy"}},</td>
  <td>    {"hess_damp",</td>
  <td>     {OT_INT,</td>
  <td>      "Activate Powell damping for BFGS"}},</td>
  <td>    {"hess_damp_fac",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Damping factor for BFGS Powell modification"}},</td>
  <td>    {"hess_update",</td>
  <td>     {OT_INT,</td>
  <td>      "Type of Hessian approximation"}},</td>
  <td>    {"fallback_update",</td>
  <td>     {OT_INT,</td>
  <td>      "If indefinite update is used, the type of fallback strategy"}},</td>
  <td>    {"hess_lim_mem",</td>
  <td>     {OT_INT,</td>
  <td>      "Full or limited memory"}},</td>
  <td>    {"hess_memsize",</td>
  <td>     {OT_INT,</td>
  <td>      "Memory size for L-BFGS updates"}},</td>
  <td>    {"which_second_derv",</td>
  <td>     {OT_INT,</td>
  <td>      "For which block should second derivatives be provided by the user"}},</td>
  <td>    {"skip_first_globalization",</td>
  <td>     {OT_BOOL,</td>
  <td>      "No globalization strategy in first iteration"}},</td>
  <td>    {"conv_strategy",</td>
  <td>     {OT_INT,</td>
  <td>      "Convexification strategy"}},</td>
  <td>    {"max_conv_qp",</td>
  <td>     {OT_INT,</td>
  <td>      "How many additional QPs may be solved for convexification per iteration?"}},</td>
  <td>    {"max_soc_iter",</td>
  <td>     {OT_INT,</td>
  <td>      "Maximum number of SOC line search iterations"}},</td>
  <td>    {"gamma_theta",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"gamma_f",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"kappa_soc",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"kappa_f",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"theta_max",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"theta_min",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"delta",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"s_theta",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"s_f",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"kappa_minus",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"kappa_plus",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"kappa_plus_max",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"delta_h0",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"eta",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Filter line search parameter, cf. IPOPT paper"}},</td>
  <td>    {"obj_lo",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Lower bound on objective function [-inf]"}},</td>
  <td>    {"obj_up",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Upper bound on objective function [inf]"}},</td>
  <td>    {"rho",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Feasibility restoration phase parameter"}},</td>
  <td>    {"zeta",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Feasibility restoration phase parameter"}},</td>
  <td>    {"print_maxit_reached",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print error when maximum number of SQP iterations reached"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/ipopt/ipopt_interface.cpp: const Options IpoptInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"pass_nonlinear_variables",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Pass list of variables entering nonlinearly to IPOPT"}},</td>
  <td>    {"ipopt",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to IPOPT"}},</td>
  <td>    {"var_string_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "String metadata (a dictionary with lists of strings) "</td>
  <td>      "about variables to be passed to IPOPT"}},</td>
  <td>    {"var_integer_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "Integer metadata (a dictionary with lists of integers) "</td>
  <td>      "about variables to be passed to IPOPT"}},</td>
  <td>    {"var_numeric_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "Numeric metadata (a dictionary with lists of reals) about "</td>
  <td>      "variables to be passed to IPOPT"}},</td>
  <td>    {"con_string_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "String metadata (a dictionary with lists of strings) about "</td>
  <td>      "constraints to be passed to IPOPT"}},</td>
  <td>    {"con_integer_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "Integer metadata (a dictionary with lists of integers) "</td>
  <td>      "about constraints to be passed to IPOPT"}},</td>
  <td>    {"con_numeric_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "Numeric metadata (a dictionary with lists of reals) about "</td>
  <td>      "constraints to be passed to IPOPT"}},</td>
  <td>    {"hess_lag",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</td>
  <td>    {"jac_g",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the Jacobian of the constraints "</td>
  <td>      "(autogenerated by default)"}},</td>
  <td>    {"grad_f",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the gradient of the objective "</td>
  <td>      "(column, autogenerated by default)"}},</td>
  <td>    {"convexify_strategy",</td>
  <td>     {OT_STRING,</td>
  <td>      "NONE|regularize|eigen-reflect|eigen-clip. "</td>
  <td>      "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</td>
  <td>    {"convexify_margin",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "When using a convexification strategy, make sure that "</td>
  <td>      "the smallest eigenvalue is at least this (default: 1e-7)."}},</td>
  <td>    {"max_iter_eig",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</td>
  <td>    {"clip_inactive_lam",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Explicitly set Lagrange multipliers to 0 when bound is deemed inactive "</td>
  <td>      "(default: false)."}},</td>
  <td>    {"inactive_lam_strategy",</td>
  <td>     {OT_STRING,</td>
  <td>      "Strategy to detect if a bound is inactive. "</td>
  <td>      "RELTOL: use solver-defined constraint tolerance * inactive_lam_value|"</td>
  <td>      "abstol: use inactive_lam_value"}},</td>
  <td>    {"inactive_lam_value",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Value used in inactive_lam_strategy (default: 10)."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/clang/clang_compiler.cpp: const Options ClangCompiler</summary>
  <table><tr><th>= {{&ImporterInternal::options_},</th></tr>
  <td>   {{"include_path",</td>
  <td>     {OT_STRING,</td>
  <td>      "Include paths for the JIT compiler. "</td>
  <td>      "The include directory shipped with CasADi will be automatically appended."}},</td>
  <td>    {"flags",</td>
  <td>     {OT_STRINGVECTOR,</td>
  <td>      "Compile flags for the JIT compiler. Default: None"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/proxqp/proxqp_interface.cpp: const Options ProxqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"proxqp",</td>
  <td>     {OT_DICT,</td>
  <td>      "const proxqp options."}},</td>
  <td>    {"warm_start_primal",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use x input to warmstart [Default: true]."}},</td>
  <td>    {"warm_start_dual",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use y and z input to warmstart [Default: true]."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/bonmin/bonmin_interface.cpp: const Options BonminInterface</summary>
  <table><tr><th>= {{&Nlpsol::options_},</th></tr>
  <td>   {{"pass_nonlinear_variables",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Pass list of variables entering nonlinearly to BONMIN"}},</td>
  <td>    {"pass_nonlinear_constraints",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Pass list of constraints entering nonlinearly to BONMIN"}},</td>
  <td>    {"bonmin",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to BONMIN"}},</td>
  <td>    {"var_string_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "String metadata (a dictionary with lists of strings) "</td>
  <td>      "about variables to be passed to BONMIN"}},</td>
  <td>    {"var_integer_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "Integer metadata (a dictionary with lists of integers) "</td>
  <td>      "about variables to be passed to BONMIN"}},</td>
  <td>    {"var_numeric_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "Numeric metadata (a dictionary with lists of reals) about "</td>
  <td>      "variables to be passed to BONMIN"}},</td>
  <td>    {"con_string_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "String metadata (a dictionary with lists of strings) about "</td>
  <td>      "constraints to be passed to BONMIN"}},</td>
  <td>    {"con_integer_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "Integer metadata (a dictionary with lists of integers) "</td>
  <td>      "about constraints to be passed to BONMIN"}},</td>
  <td>    {"con_numeric_md",</td>
  <td>     {OT_DICT,</td>
  <td>      "Numeric metadata (a dictionary with lists of reals) about "</td>
  <td>      "constraints to be passed to BONMIN"}},</td>
  <td>    {"hess_lag",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</td>
  <td>    {"hess_lag_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options for the autogenerated Hessian of the Lagrangian."}},</td>
  <td>    {"jac_g",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the Jacobian of the constraints "</td>
  <td>      "(autogenerated by default)"}},</td>
  <td>    {"jac_g_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options for the autogenerated Jacobian of the constraints."}},</td>
  <td>    {"grad_f",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function for calculating the gradient of the objective "</td>
  <td>      "(column, autogenerated by default)"}},</td>
  <td>    {"grad_f_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options for the autogenerated gradient of the objective."}},</td>
  <td>    {"sos1_groups",</td>
  <td>     {OT_INTVECTORVECTOR,</td>
  <td>      "Options for the autogenerated gradient of the objective."}},</td>
  <td>    {"sos1_weights",</td>
  <td>     {OT_DOUBLEVECTORVECTOR,</td>
  <td>      "Options for the autogenerated gradient of the objective."}},</td>
  <td>    {"sos1_priorities",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Options for the autogenerated gradient of the objective."}},</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/cbc/cbc_interface.cpp: const Options CbcInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"cbc",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to CBC."</td>
  <td>      "Three sets of options are supported. "</td>
  <td>      "The first can be found in OsiSolverParameters.hpp. "</td>
  <td>      "The second can be found in CbcModel.hpp. "</td>
  <td>      "The third are options that can be passed to CbcMain1."</td>
  <td>      }},</td>
  <td>    {"sos_groups",</td>
  <td>     {OT_INTVECTORVECTOR,</td>
  <td>      "Definition of SOS groups by indices."}},</td>
  <td>    {"sos_weights",</td>
  <td>     {OT_DOUBLEVECTORVECTOR,</td>
  <td>      "Weights corresponding to SOS entries."}},</td>
  <td>    {"sos_types",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Specify 1 or 2 for each SOS group."}},</td>
  <td>    {"hot_start",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Hot start with x0 [Default false]."}},</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/interfaces/osqp/osqp_interface.cpp: const Options OsqpInterface</summary>
  <table><tr><th>= {{&Conic::options_},</th></tr>
  <td>   {{"osqp",</td>
  <td>     {OT_DICT,</td>
  <td>      "const Options to be passed to osqp."}},</td>
  <td>    {"warm_start_primal",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use x0 input to warmstart [Default: true]."}},</td>
  <td>    {"warm_start_dual",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use lam_a0 and lam_x0 input to warmstart [Default: truw]."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/sx_function.cpp: const Options SXFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"default_in",</td>
  <td>     {OT_DOUBLEVECTOR,</td>
  <td>      "Default input values"}},</td>
  <td>    {"just_in_time_sparsity",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Propagate sparsity patterns using just-in-time "</td>
  <td>      "compilation to a CPU or GPU using OpenCL"}},</td>
  <td>    {"just_in_time_opencl",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Just-in-time compilation for numeric evaluation using OpenCL (experimental)"}},</td>
  <td>    {"live_variables",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Reuse variables in the work vector"}},</td>
  <td>    {"cse",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Perform common subexpression elimination (complexity is N*log(N) in graph size)"}},</td>
  <td>    {"allow_free",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Allow construction with free variables (Default: false)"}},</td>
  <td>    {"allow_duplicate_io_names",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Allow construction with duplicate io names (Default: false)"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options Integrator</summary>
  <table><tr><th>= {{&OracleFunction::options_},</th></tr>
  <td>   {{"expand",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Replace MX with SX expressions in problem formulation [false]"}},</td>
  <td>  {"print_stats",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Print out statistics after integration"}},</td>
  <td>  {"nfwd",</td>
  <td>   {OT_INT,</td>
  <td>    "Number of forward sensitivities to be calculated [0]"}},</td>
  <td>  {"nadj",</td>
  <td>   {OT_INT,</td>
  <td>    "Number of adjoint sensitivities to be calculated [0]"}},</td>
  <td>  {"t0",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "[DEPRECATED] Beginning of the time horizon"}},</td>
  <td>  {"tf",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "[DEPRECATED] End of the time horizon"}},</td>
  <td>  {"grid",</td>
  <td>    {OT_DOUBLEVECTOR,</td>
  <td>    "[DEPRECATED] Time grid"}},</td>
  <td>  {"augmented_options",</td>
  <td>    {OT_DICT,</td>
  <td>    "Options to be passed down to the augmented integrator, if one is constructed."}},</td>
  <td>  {"output_t0",</td>
  <td>    {OT_BOOL,</td>
  <td>    "[DEPRECATED] Output the state at the initial time"}}</td>
  <td>  }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options FixedStepIntegrator</summary>
  <table><tr><th>= {{&Integrator::options_},</th></tr>
  <td>   {{"number_of_finite_elements",</td>
  <td>    {OT_INT,</td>
  <td>    "Target number of finite elements. "</td>
  <td>    "The actual number may be higher to accommodate all output times"}},</td>
  <td>  {"simplify",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Implement as MX Function (codegeneratable/serializable) default: false"}},</td>
  <td>  {"simplify_options",</td>
  <td>    {OT_DICT,</td>
  <td>    "Any options to pass to simplified form Function constructor"}}</td>
  <td>  }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options ImplicitFixedStepIntegrator</summary>
  <table><tr><th>= {{&FixedStepIntegrator::options_},</th></tr>
  <td>   {{"rootfinder",</td>
  <td>    {OT_STRING,</td>
  <td>    "An implicit function solver"}},</td>
  <td>  {"rootfinder_options",</td>
  <td>    {OT_DICT,</td>
  <td>    "Options to be passed to the NLP Solver"}}</td>
  <td>  }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/conic.cpp: const Options Conic</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"discrete",</td>
  <td>     {OT_BOOLVECTOR,</td>
  <td>      "Indicates which of the variables are discrete, i.e. integer-valued"}},</td>
  <td>    {"print_problem",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print a numeric description of the problem"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/rootfinder.cpp: const Options Rootfinder</summary>
  <table><tr><th>= {{&OracleFunction::options_},</th></tr>
  <td>   {{"linear_solver",</td>
  <td>     {OT_STRING,</td>
  <td>      "User-defined linear solver class. Needed for sensitivities."}},</td>
  <td>    {"linear_solver_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to the linear solver."}},</td>
  <td>    {"constraints",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "Constrain the unknowns. 0 (default): no constraint on ui, "</td>
  <td>      "1: ui >= 0.0, -1: ui <= 0.0, 2: ui > 0.0, -2: ui < 0.0."}},</td>
  <td>    {"implicit_input",</td>
  <td>     {OT_INT,</td>
  <td>      "Index of the input that corresponds to the actual root-finding"}},</td>
  <td>    {"implicit_output",</td>
  <td>     {OT_INT,</td>
  <td>      "Index of the output that corresponds to the actual root-finding"}},</td>
  <td>    {"jacobian_function",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Function object for calculating the Jacobian (autogenerated by default)"}},</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/mx_function.cpp: const Options MXFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"default_in",</td>
  <td>     {OT_DOUBLEVECTOR,</td>
  <td>      "Default input values"}},</td>
  <td>    {"live_variables",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Reuse variables in the work vector"}},</td>
  <td>    {"print_instructions",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print each operation during evaluation"}},</td>
  <td>    {"cse",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Perform common subexpression elimination (complexity is N*log(N) in graph size)"}},</td>
  <td>    {"allow_free",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Allow construction with free variables (Default: false)"}},</td>
  <td>    {"allow_duplicate_io_names",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Allow construction with duplicate io names (Default: false)"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/expm.cpp: const Options Expm</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"const_A",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Assume A is constant. Default: false."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/finite_differences.cpp: const Options FiniteDiff</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"second_order_stepsize",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Second order perturbation size [default: 1e-3]"}},</td>
  <td>  {"h",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Step size [default: computed from abstol]"}},</td>
  <td>  {"h_max",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Maximum step size [default 0]"}},</td>
  <td>  {"h_min",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Minimum step size [default inf]"}},</td>
  <td>  {"smoothing",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Smoothing regularization [default: machine precision]"}},</td>
  <td>  {"reltol",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Accuracy of function inputs [default: query object]"}},</td>
  <td>  {"abstol",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Accuracy of function outputs [default: query object]"}},</td>
  <td>  {"u_aim",</td>
  <td>    {OT_DOUBLE,</td>
  <td>    "Target ratio of roundoff error to truncation error [default: 100.]"}},</td>
  <td>  {"h_iter",</td>
  <td>    {OT_INT,</td>
  <td>    "Number of iterations to improve on the step-size "</td>
  <td>    "[default: 1 if error estimate available, otherwise 0]"}},</td>
  <td>  }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/jit_function.cpp: const Options JitFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"buffered",</td>
  <td>    {OT_BOOL,</td>
  <td>      "Buffer the calls, user does not need to "}},</td>
  <td>     {"jac",</td>
  <td>    {OT_STRING,</td>
  <td>      "Function body for Jacobian"}},</td>
  <td>    {"hess",</td>
  <td>     {OT_STRING,</td>
  <td>      "Function body for Hessian"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/function_internal.cpp: const Options ProtoFunction</summary>
  <table><tr><th>= {{},</th></tr>
  <td>   {{"verbose",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Verbose evaluation -- for debugging"}},</td>
  <td>    {"print_time",</td>
  <td>     {OT_BOOL,</td>
  <td>      "print information about execution time. Implies record_time."}},</td>
  <td>    {"record_time",</td>
  <td>     {OT_BOOL,</td>
  <td>      "record information about execution time, for retrieval with stats()."}},</td>
  <td>    {"regularity_check",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Throw exceptions when NaN or Inf appears during evaluation"}},</td>
  <td>    {"error_on_fail",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Throw exceptions when function evaluation fails (default true)."}}</td>
  <td>    }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/function_internal.cpp: const Options FunctionInternal</summary>
  <table><tr><th>= {{&ProtoFunction::options_},</th></tr>
  <td>   {{"ad_weight",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Weighting factor for derivative calculation."</td>
  <td>      "When there is an option of either using forward or reverse mode "</td>
  <td>      "directional derivatives, the condition ad_weight*nf<=(1-ad_weight)*na "</td>
  <td>      "is used where nf and na are estimates of the number of forward/reverse "</td>
  <td>      "mode directional derivatives needed. By default, ad_weight is calculated "</td>
  <td>      "automatically, but this can be overridden by setting this option. "</td>
  <td>      "In particular, 0 means forcing forward mode and 1 forcing reverse mode. "</td>
  <td>      "Leave unset for (class specific) heuristics."}},</td>
  <td>    {"ad_weight_sp",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Weighting factor for sparsity pattern calculation calculation."</td>
  <td>      "Overrides default behavior. Set to 0 and 1 to force forward and "</td>
  <td>      "reverse mode respectively. Cf. option \"ad_weight\". "</td>
  <td>      "When set to -1, sparsity is completely ignored and dense matrices are used."}},</td>
  <td>    {"always_inline",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Force inlining."}},</td>
  <td>    {"never_inline",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Forbid inlining."}},</td>
  <td>    {"jac_penalty",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "When requested for a number of forward/reverse directions,   "</td>
  <td>      "it may be cheaper to compute first the full jacobian and then "</td>
  <td>      "multiply with seeds, rather than obtain the requested directions "</td>
  <td>      "in a straightforward manner. "</td>
  <td>      "Casadi uses a heuristic to decide which is cheaper. "</td>
  <td>      "A high value of 'jac_penalty' makes it less likely for the heurstic "</td>
  <td>      "to chose the full Jacobian strategy. "</td>
  <td>      "The special value -1 indicates never to use the full Jacobian strategy"}},</td>
  <td>    {"user_data",</td>
  <td>     {OT_VOIDPTR,</td>
  <td>      "A user-defined field that can be used to identify "</td>
  <td>      "the function or pass additional information"}},</td>
  <td>    {"inputs_check",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Throw exceptions when the numerical values of the inputs don't make sense"}},</td>
  <td>    {"gather_stats",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Deprecated option (ignored): Statistics are now always collected."}},</td>
  <td>    {"input_scheme",</td>
  <td>     {OT_STRINGVECTOR,</td>
  <td>      "Deprecated option (ignored)"}},</td>
  <td>    {"output_scheme",</td>
  <td>     {OT_STRINGVECTOR,</td>
  <td>      "Deprecated option (ignored)"}},</td>
  <td>    {"jit",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use just-in-time compiler to speed up the evaluation"}},</td>
  <td>    {"jit_cleanup",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Cleanup up the temporary source file that jit creates. Default: true"}},</td>
  <td>    {"jit_serialize",</td>
  <td>     {OT_STRING,</td>
  <td>      "Specify behaviour when serializing a jitted function: SOURCE|link|embed."}},</td>
  <td>    {"jit_name",</td>
  <td>     {OT_STRING,</td>
  <td>      "The file name used to write out code. "</td>
  <td>      "The actual file names used depend on 'jit_temp_suffix' and include extensions. "</td>
  <td>      "Default: 'jit_tmp'"}},</td>
  <td>    {"jit_temp_suffix",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Use a temporary (seemingly random) filename suffix for generated code and libraries. "</td>
  <td>      "This is desired for thread-safety. "</td>
  <td>      "This behaviour may defeat caching compiler wrappers. "</td>
  <td>      "Default: true"}},</td>
  <td>    {"compiler",</td>
  <td>     {OT_STRING,</td>
  <td>      "Just-in-time compiler plugin to be used."}},</td>
  <td>    {"jit_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to the jit compiler."}},</td>
  <td>    {"derivative_of",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "The function is a derivative of another function. "</td>
  <td>      "The type of derivative (directional derivative, Jacobian) "</td>
  <td>      "is inferred from the function name."}},</td>
  <td>    {"max_num_dir",</td>
  <td>     {OT_INT,</td>
  <td>      "Specify the maximum number of directions for derivative functions."</td>
  <td>      " Overrules the builtin optimized_num_dir."}},</td>
  <td>    {"enable_forward",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Enable derivative calculation using generated functions for"</td>
  <td>      " Jacobian-times-vector products - typically using forward mode AD"</td>
  <td>      " - if available. [default: true]"}},</td>
  <td>    {"enable_reverse",</td>
  <td>      {OT_BOOL,</td>
  <td>      "Enable derivative calculation using generated functions for"</td>
  <td>      " transposed Jacobian-times-vector products - typically using reverse mode AD"</td>
  <td>      " - if available. [default: true]"}},</td>
  <td>    {"enable_jacobian",</td>
  <td>      {OT_BOOL,</td>
  <td>      "Enable derivative calculation using generated functions for"</td>
  <td>      " Jacobians of all differentiable outputs with respect to all differentiable inputs"</td>
  <td>      " - if available. [default: true]"}},</td>
  <td>    {"enable_fd",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Enable derivative calculation by finite differencing. [default: false]]"}},</td>
  <td>    {"fd_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to the finite difference instance"}},</td>
  <td>    {"fd_method",</td>
  <td>     {OT_STRING,</td>
  <td>      "Method for finite differencing [default 'central']"}},</td>
  <td>    {"print_in",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print numerical values of inputs [default: false]"}},</td>
  <td>    {"print_out",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print numerical values of outputs [default: false]"}},</td>
  <td>    {"max_io",</td>
  <td>     {OT_INT,</td>
  <td>      "Acceptable number of inputs and outputs. Warn if exceeded."}},</td>
  <td>    {"dump_in",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Dump numerical values of inputs to file (readable with DM.from_file) [default: false]"}},</td>
  <td>    {"dump_out",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Dump numerical values of outputs to file (readable with DM.from_file) [default: false]"}},</td>
  <td>    {"dump",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Dump function to file upon first evaluation. [false]"}},</td>
  <td>    {"dump_dir",</td>
  <td>     {OT_STRING,</td>
  <td>      "Directory to dump inputs/outputs to. Make sure the directory exists [.]"}},</td>
  <td>    {"dump_format",</td>
  <td>     {OT_STRING,</td>
  <td>      "Choose file format to dump matrices. See DM.from_file [mtx]"}},</td>
  <td>    {"forward_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to a forward mode constructor"}},</td>
  <td>    {"reverse_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to a reverse mode constructor"}},</td>
  <td>    {"jacobian_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to a Jacobian constructor"}},</td>
  <td>    {"der_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Default options to be used to populate forward_options, reverse_options, and "</td>
  <td>      "jacobian_options before those options are merged in."}},</td>
  <td>    {"custom_jacobian",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "Override CasADi's AD. Use together with 'jac_penalty': 0. "</td>
  <td>      "Note: Highly experimental. Syntax may break often."}},</td>
  <td>    {"is_diff_in",</td>
  <td>     {OT_BOOLVECTOR,</td>
  <td>      "Indicate for each input if it should be differentiable."}},</td>
  <td>    {"is_diff_out",</td>
  <td>     {OT_BOOLVECTOR,</td>
  <td>      "Indicate for each output if it should be differentiable."}},</td>
  <td>    {"post_expand",</td>
  <td>     {OT_BOOL,</td>
  <td>      "After construction, expand this Function. Default: False"}},</td>
  <td>    {"post_expand_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to post-construction expansion. Default: empty"}},</td>
  <td>    {"cache",</td>
  <td>     {OT_DICT,</td>
  <td>      "Prepopulate the function cache. Default: empty"}},</td>
  <td>    {"external_transform",</td>
  <td>     {OT_VECTORVECTOR,</td>
  <td>      "List of external_transform instruction arguments. Default: empty"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/oracle_function.cpp: const Options OracleFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"expand",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Replace MX with SX expressions in problem formulation [false]"}},</td>
  <td>  {"monitor",</td>
  <td>    {OT_STRINGVECTOR,</td>
  <td>    "Set of user problem functions to be monitored"}},</td>
  <td>  {"show_eval_warnings",</td>
  <td>    {OT_BOOL,</td>
  <td>    "Show warnings generated from function evaluations [true]"}},</td>
  <td>  {"common_options",</td>
  <td>    {OT_DICT,</td>
  <td>    "Options for auto-generated functions"}},</td>
  <td>  {"specific_options",</td>
  <td>    {OT_DICT,</td>
  <td>    "Options for specific auto-generated functions,"</td>
  <td>    " overwriting the defaults from common_options. Nested dictionary."}}</td>
  <td>}</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/importer_internal.cpp: const Options ImporterInternal</summary>
  <table><tr><th>= {{},</th></tr>
  <td>   {{"verbose",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Verbose evaluation -- for debugging"}}</td>
  <td>    }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/interpolant.cpp: const Options Interpolant</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"lookup_mode",</td>
  <td>     {OT_STRINGVECTOR,</td>
  <td>      "Specifies, for each grid dimension, the lookup algorithm used to find the correct index. "</td>
  <td>      "'linear' uses a for-loop + break; (default when #knots<=100), "</td>
  <td>      "'exact' uses floored division (only for uniform grids), "</td>
  <td>      "'binary' uses a binary search. (default when #knots>100)."}},</td>
  <td>    {"inline",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Implement the lookup table in MX primitives. "</td>
  <td>      "Useful when you need derivatives with respect to grid and/or coefficients. "</td>
  <td>      "Such derivatives are fundamentally dense, so use with caution."}},</td>
  <td>    {"batch_x",</td>
  <td>     {OT_INT,</td>
  <td>      "Evaluate a batch of different inputs at once (default 1)."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/nlpsol.cpp: const Options Nlpsol</summary>
  <table><tr><th>= {{&OracleFunction::options_},</th></tr>
  <td>   {{"iteration_callback",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "A function that will be called at each iteration with the solver as input. "</td>
  <td>      "Check documentation of Callback."}},</td>
  <td>    {"iteration_callback_step",</td>
  <td>     {OT_INT,</td>
  <td>      "Only call the callback function every few iterations."}},</td>
  <td>    {"iteration_callback_ignore_errors",</td>
  <td>     {OT_BOOL,</td>
  <td>      "If set to true, errors thrown by iteration_callback will be ignored."}},</td>
  <td>    {"ignore_check_vec",</td>
  <td>     {OT_BOOL,</td>
  <td>      "If set to true, the input shape of F will not be checked."}},</td>
  <td>    {"warn_initial_bounds",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Warn if the initial guess does not satisfy LBX and UBX"}},</td>
  <td>    {"eval_errors_fatal",</td>
  <td>     {OT_BOOL,</td>
  <td>      "When errors occur during evaluation of f,g,...,"</td>
  <td>      "stop the iterations"}},</td>
  <td>    {"verbose_init",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Print out timing information about "</td>
  <td>      "the different stages of initialization"}},</td>
  <td>    {"discrete",</td>
  <td>     {OT_BOOLVECTOR,</td>
  <td>      "Indicates which of the variables are discrete, i.e. integer-valued"}},</td>
  <td>    {"calc_multipliers",</td>
  <td>    {OT_BOOL,</td>
  <td>     "Calculate Lagrange multipliers in the Nlpsol base class"}},</td>
  <td>    {"calc_lam_x",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Calculate 'lam_x' in the Nlpsol base class"}},</td>
  <td>    {"calc_lam_p",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Calculate 'lam_p' in the Nlpsol base class"}},</td>
  <td>    {"calc_f",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Calculate 'f' in the Nlpsol base class"}},</td>
  <td>    {"calc_g",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Calculate 'g' in the Nlpsol base class"}},</td>
  <td>    {"no_nlp_grad",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Prevent the creation of the 'nlp_grad' function"}},</td>
  <td>    {"bound_consistency",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Ensure that primal-dual solution is consistent with the bounds"}},</td>
  <td>    {"min_lam",</td>
  <td>     {OT_DOUBLE,</td>
  <td>      "Minimum allowed multiplier value"}},</td>
  <td>    {"oracle_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Options to be passed to the oracle function"}},</td>
  <td>    {"sens_linsol",</td>
  <td>     {OT_STRING,</td>
  <td>      "Linear solver used for parametric sensitivities (default 'qr')."}},</td>
  <td>    {"sens_linsol_options",</td>
  <td>     {OT_DICT,</td>
  <td>      "Linear solver options used for parametric sensitivities."}},</td>
  <td>    {"detect_simple_bounds",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Automatically detect simple bounds (lbx/ubx) (default false). "</td>
  <td>      "This is hopefully beneficial to speed and robustness but may also have adverse affects: "</td>
  <td>      "1) Subtleties in heuristics and stopping criteria may change the solution, "</td>
  <td>      "2) IPOPT may lie about multipliers of simple equality bounds unless "</td>
  <td>      "'fixed_variable_treatment' is set to 'relax_bounds'."}},</td>
  <td>    {"detect_simple_bounds_is_simple",</td>
  <td>     {OT_BOOLVECTOR,</td>
  <td>      "For internal use only."}},</td>
  <td>    {"detect_simple_bounds_parts",</td>
  <td>     {OT_FUNCTION,</td>
  <td>      "For internal use only."}},</td>
  <td>    {"detect_simple_bounds_target_x",</td>
  <td>     {OT_INTVECTOR,</td>
  <td>      "For internal use only."}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/fmu_function.cpp: const Options FmuFunction</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"scheme_in",</td>
  <td>   {OT_STRINGVECTOR,</td>
  <td>    "Names of the inputs in the scheme"}},</td>
  <td>  {"scheme_out",</td>
  <td>   {OT_STRINGVECTOR,</td>
  <td>    "Names of the outputs in the scheme"}},</td>
  <td>  {"scheme",</td>
  <td>   {OT_DICT,</td>
  <td>    "Definitions of the scheme variables"}},</td>
  <td>  {"aux",</td>
  <td>   {OT_STRINGVECTOR,</td>
  <td>    "Auxilliary variables"}},</td>
  <td>  {"enable_ad",</td>
  <td>   {OT_BOOL,</td>
  <td>    "Calculate first order derivatives using FMU directional derivative support"}},</td>
  <td>  {"validate_ad",</td>
  <td>   {OT_BOOL,</td>
  <td>    "Compare analytic derivatives with finite differences for validation"}},</td>
  <td>  {"validate_ad_file",</td>
  <td>   {OT_STRING,</td>
  <td>    "Redirect results of Hessian validation to a file instead of generating a warning"}},</td>
  <td>  {"check_hessian",</td>
  <td>   {OT_BOOL,</td>
  <td>    "Symmetry check for Hessian"}},</td>
  <td>  {"make_symmetric",</td>
  <td>   {OT_BOOL,</td>
  <td>    "Ensure Hessian is symmetric"}},</td>
  <td>  {"step",</td>
  <td>   {OT_DOUBLE,</td>
  <td>    "Step size, scaled by nominal value"}},</td>
  <td>  {"abstol",</td>
  <td>   {OT_DOUBLE,</td>
  <td>    "Absolute error tolerance, scaled by nominal value"}},</td>
  <td>  {"reltol",</td>
  <td>   {OT_DOUBLE,</td>
  <td>    "Relative error tolerance"}},</td>
  <td>  {"parallelization",</td>
  <td>   {OT_STRING,</td>
  <td>    "Parallelization [SERIAL|openmp|thread]"}},</td>
  <td>  {"print_progress",</td>
  <td>   {OT_BOOL,</td>
  <td>    "Print progress during Jacobian/Hessian evaluation"}},</td>
  <td>  {"new_jacobian",</td>
  <td>   {OT_BOOL,</td>
  <td>    "Use Jacobian implementation in class"}},</td>
  <td>  {"new_hessian",</td>
  <td>   {OT_BOOL,</td>
  <td>    "Use Hessian implementation in class"}},</td>
  <td>  {"hessian_coloring",</td>
  <td>   {OT_BOOL,</td>
  <td>    "Enable the use of graph coloring (star coloring) for Hessian calculation. "</td>
  <td>    "Note that disabling the coloring can improve symmetry check diagnostics."}}</td>
  <td> }</td>
  <td>  };</td>
</table></details>

<details>
 <summary>./casadi/casadi/core/dple.cpp: const Options Dple</summary>
  <table><tr><th>= {{&FunctionInternal::options_},</th></tr>
  <td>   {{"const_dim",</td>
  <td>     {OT_BOOL,</td>
  <td>      "Assume constant dimension of P"}},</td>
  <td>    {"pos_def",</td>
  <td>      {OT_BOOL,</td>
  <td>       "Assume P positive definite"}},</td>
  <td>    {"error_unstable",</td>
  <td>      {OT_BOOL,</td>
  <td>      "Throw an exception when it is detected that Product(A_i, i=N..1)"</td>
  <td>      "has eigenvalues greater than 1-eps_unstable"}},</td>
  <td>    {"eps_unstable",</td>
  <td>      {OT_DOUBLE,</td>
  <td>      "A margin for unstability detection"}}</td>
  <td>   }</td>
  <td>  };</td>
</table></details>
