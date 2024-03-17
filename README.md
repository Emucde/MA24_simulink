# MA24_simulink

./casadi/casadi/solvers/linsol_ldl.cpp: const Options LinsolLdl</summary>
  </p>= {{&ProtoFunction::options_},</p>
  </p>   {{"incomplete",</p>
  </p>    {OT_BOOL,</p>
  </p>     "Incomplete factorization, without any fill-in"}},</p>
  </p>    {"preordering",</p>
  </p>     {OT_BOOL,</p>
  </p>     "Approximate minimal degree (AMD) preordering"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/linear_interpolant.cpp: const Options LinearInterpolant</summary>
  </p>= {{&Interpolant::options_},</p>
  </p>   {{"lookup_mode",</p>
  </p>     {OT_STRINGVECTOR,</p>
  </p>      "Sets, for each grid dimenion, the lookup algorithm used to find the correct index. "</p>
  </p>      "'linear' uses a for-loop + break; "</p>
  </p>      "'exact' uses floored division (only for uniform grids)."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/ipqp.cpp: const Options Ipqp</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"max_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of iterations [1000]."}},</p>
  </p>    {"constr_viol_tol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Constraint violation tolerance [1e-8]."}},</p>
  </p>    {"dual_inf_tol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Dual feasibility violation tolerance [1e-8]"}},</p>
  </p>    {"print_header",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print header [true]."}},</p>
  </p>    {"print_iter",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print iterations [true]."}},</p>
  </p>    {"print_info",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print info [true]."}},</p>
  </p>    {"linear_solver",</p>
  </p>     {OT_STRING,</p>
  </p>      "A custom linear solver creator function [default: ldl]"}},</p>
  </p>    {"linear_solver_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to the linear solver"}},</p>
  </p>    {"min_lam",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Smallest multiplier treated as inactive for the initial active set [0]."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/qrsqp.cpp: const Options Qrsqp</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>   {{"qpsol",</p>
  </p>     {OT_STRING,</p>
  </p>      "The QP solver to be used by the SQP method [qrqp]"}},</p>
  </p>    {"qpsol_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to the QP solver"}},</p>
  </p>    {"hessian_approximation",</p>
  </p>     {OT_STRING,</p>
  </p>      "limited-memory|exact"}},</p>
  </p>    {"max_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of SQP iterations"}},</p>
  </p>    {"min_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Minimum number of SQP iterations"}},</p>
  </p>    {"max_iter_ls",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of linesearch iterations"}},</p>
  </p>    {"tol_pr",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion for primal infeasibility"}},</p>
  </p>    {"tol_du",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion for dual infeasability"}},</p>
  </p>    {"c1",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Armijo condition, coefficient of decrease in merit"}},</p>
  </p>    {"beta",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Line-search parameter, restoration factor of stepsize"}},</p>
  </p>    {"merit_memory",</p>
  </p>     {OT_INT,</p>
  </p>      "Size of memory to store history of merit function values"}},</p>
  </p>    {"lbfgs_memory",</p>
  </p>     {OT_INT,</p>
  </p>      "Size of L-BFGS memory."}},</p>
  </p>    {"regularize",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Automatic regularization of Lagrange Hessian."}},</p>
  </p>    {"print_header",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print the header with problem statistics"}},</p>
  </p>    {"print_iteration",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print the iterations"}},</p>
  </p>    {"min_step_size",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "The size (inf-norm) of the step size should not become smaller than this."}},</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/scpgen.cpp: const Options Scpgen</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>   {{"qpsol",</p>
  </p>     {OT_STRING,</p>
  </p>      "The QP solver to be used by the SQP method"}},</p>
  </p>    {"qpsol_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to the QP solver"}},</p>
  </p>    {"hessian_approximation",</p>
  </p>     {OT_STRING,</p>
  </p>      "gauss-newton|exact"}},</p>
  </p>    {"max_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of SQP iterations"}},</p>
  </p>    {"max_iter_ls",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of linesearch iterations"}},</p>
  </p>    {"tol_pr",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion for primal infeasibility"}},</p>
  </p>    {"tol_du",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion for dual infeasability"}},</p>
  </p>    {"tol_reg",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion for regularization"}},</p>
  </p>    {"tol_pr_step",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion for the step size"}},</p>
  </p>    {"c1",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Armijo condition, coefficient of decrease in merit"}},</p>
  </p>    {"beta",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Line-search parameter, restoration factor of stepsize"}},</p>
  </p>    {"merit_memsize",</p>
  </p>     {OT_INT,</p>
  </p>      "Size of memory to store history of merit function values"}},</p>
  </p>    {"merit_start",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Lower bound for the merit function parameter"}},</p>
  </p>    {"lbfgs_memory",</p>
  </p>     {OT_INT,</p>
  </p>      "Size of L-BFGS memory."}},</p>
  </p>    {"regularize",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Automatic regularization of Lagrange Hessian."}},</p>
  </p>    {"print_header",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print the header with problem statistics"}},</p>
  </p>    {"codegen",</p>
  </p>     {OT_BOOL,</p>
  </p>      "C-code generation"}},</p>
  </p>    {"reg_threshold",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Threshold for the regularization."}},</p>
  </p>    {"name_x",</p>
  </p>     {OT_STRINGVECTOR,</p>
  </p>      "Names of the variables."}},</p>
  </p>    {"print_x",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Which variables to print."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/linsol_qr.cpp: const Options LinsolQr</summary>
  </p>= {{&LinsolInternal::options_},</p>
  </p>   {{"eps",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Minimum R entry before singularity is declared [1e-12]"}},</p>
  </p>    {"cache",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Amount of factorisations to remember (thread-local) [0]"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/newton.cpp: const Options Newton</summary>
  </p>= {{&Rootfinder::options_},</p>
  </p>   {{"abstol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion tolerance on max(|F|)"}},</p>
  </p>    {"abstolStep",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion tolerance on step size"}},</p>
  </p>    {"max_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of Newton iterations to perform before returning."}},</p>
  </p>    {"print_iteration",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print information about each iteration"}},</p>
  </p>    {"line_search",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Enable line-search (default: true)"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/sqpmethod.cpp: const Options Sqpmethod</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>  {{"qpsol",</p>
  </p>    {OT_STRING,</p>
  </p>    "The QP solver to be used by the SQP method [qpoases]"}},</p>
  </p>  {"qpsol_options",</p>
  </p>    {OT_DICT,</p>
  </p>    "Options to be passed to the QP solver"}},</p>
  </p>  {"hessian_approximation",</p>
  </p>    {OT_STRING,</p>
  </p>    "limited-memory|exact"}},</p>
  </p>  {"max_iter",</p>
  </p>    {OT_INT,</p>
  </p>    "Maximum number of SQP iterations"}},</p>
  </p>  {"min_iter",</p>
  </p>    {OT_INT,</p>
  </p>    "Minimum number of SQP iterations"}},</p>
  </p>  {"max_iter_ls",</p>
  </p>    {OT_INT,</p>
  </p>    "Maximum number of linesearch iterations"}},</p>
  </p>  {"tol_pr",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Stopping criterion for primal infeasibility"}},</p>
  </p>  {"tol_du",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Stopping criterion for dual infeasability"}},</p>
  </p>  {"c1",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Armijo condition, coefficient of decrease in merit"}},</p>
  </p>  {"beta",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Line-search parameter, restoration factor of stepsize"}},</p>
  </p>  {"merit_memory",</p>
  </p>    {OT_INT,</p>
  </p>    "Size of memory to store history of merit function values"}},</p>
  </p>  {"lbfgs_memory",</p>
  </p>    {OT_INT,</p>
  </p>    "Size of L-BFGS memory."}},</p>
  </p>  {"print_header",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Print the header with problem statistics"}},</p>
  </p>  {"print_iteration",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Print the iterations"}},</p>
  </p>  {"print_status",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Print a status message after solving"}},</p>
  </p>  {"min_step_size",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "The size (inf-norm) of the step size should not become smaller than this."}},</p>
  </p>  {"hess_lag",</p>
  </p>    {OT_FUNCTION,</p>
  </p>    "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</p>
  </p>  {"jac_fg",</p>
  </p>    {OT_FUNCTION,</p>
  </p>    "Function for calculating the gradient of the objective and Jacobian of the constraints "</p>
  </p>    "(autogenerated by default)"}},</p>
  </p>  {"convexify_strategy",</p>
  </p>    {OT_STRING,</p>
  </p>    "NONE|regularize|eigen-reflect|eigen-clip. "</p>
  </p>    "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</p>
  </p>  {"convexify_margin",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "When using a convexification strategy, make sure that "</p>
  </p>    "the smallest eigenvalue is at least this (default: 1e-7)."}},</p>
  </p>  {"max_iter_eig",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</p>
  </p>  {"elastic_mode",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Enable the elastic mode which is used when the QP is infeasible (default: false)."}},</p>
  </p>  {"gamma_0",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Starting value for the penalty parameter of elastic mode (default: 1)."}},</p>
  </p>  {"gamma_max",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Maximum value for the penalty parameter of elastic mode (default: 1e20)."}},</p>
  </p>  {"gamma_1_min",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Minimum value for gamma_1 (default: 1e-5)."}},</p>
  </p>  {"second_order_corrections",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Enable second order corrections. "</p>
  </p>    "These are used when a step is considered bad by the merit function and constraint norm "</p>
  </p>    "(default: false)."}},</p>
  </p>  {"init_feasible",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Initialize the QP subproblems with a feasible initial value (default: false)."}}</p>
  </p>  }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/fast_newton.cpp: const Options FastNewton</summary>
  </p>= {{&Rootfinder::options_},</p>
  </p>   {{"abstol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion tolerance on ||g||__inf)"}},</p>
  </p>    {"abstolStep",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion tolerance on step size"}},</p>
  </p>    {"max_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of Newton iterations to perform before returning."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/shell_compiler.cpp: const Options ShellCompiler</summary>
  </p>= {{&ImporterInternal::options_},</p>
  </p>   {{"compiler",</p>
  </p>     {OT_STRING,</p>
  </p>      "Compiler command"}},</p>
  </p>    {"linker",</p>
  </p>     {OT_STRING,</p>
  </p>      "Linker command"}},</p>
  </p>    {"directory",</p>
  </p>     {OT_STRING,</p>
  </p>      "Directory to put temporary objects in. Must end with a file separator."}},</p>
  </p>    {"compiler_setup",</p>
  </p>     {OT_STRING,</p>
  </p>      "Compiler setup command. Intended to be fixed."</p>
  </p>      " The 'flag' option is the prefered way to set"</p>
  </p>      " custom flags."}},</p>
  </p>    {"linker_setup",</p>
  </p>     {OT_STRING,</p>
  </p>      "Linker setup command. Intended to be fixed."</p>
  </p>      " The 'flag' option is the prefered way to set"</p>
  </p>      " custom flags."}},</p>
  </p>    {"compiler_flags",</p>
  </p>     {OT_STRINGVECTOR,</p>
  </p>      "Alias for 'compiler_flags'"}},</p>
  </p>    {"flags",</p>
  </p>      {OT_STRINGVECTOR,</p>
  </p>      "Compile flags for the JIT compiler. Default: None"}},</p>
  </p>    {"linker_flags",</p>
  </p>     {OT_STRINGVECTOR,</p>
  </p>      "Linker flags for the JIT compiler. Default: None"}},</p>
  </p>    {"cleanup",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Cleanup temporary files when unloading. Default: true"}},</p>
  </p>    {"compiler_output_flag",</p>
  </p>     {OT_STRING,</p>
  </p>     "Compiler flag to denote object output. Default: '-o '"}},</p>
  </p>    {"linker_output_flag",</p>
  </p>     {OT_STRING,</p>
  </p>     "Linker flag to denote shared library output. Default: '-o '"}},</p>
  </p>    {"extra_suffixes",</p>
  </p>     {OT_STRINGVECTOR,</p>
  </p>     "List of suffixes for extra files that the compiler may generate. Default: None"}},</p>
  </p>    {"name",</p>
  </p>     {OT_STRING,</p>
  </p>      "The file name used to write out compiled objects/libraries. "</p>
  </p>      "The actual file names used depend on 'temp_suffix' and include extensions. "</p>
  </p>      "Default: 'tmp_casadi_compiler_shell'"}},</p>
  </p>    {"temp_suffix",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use a temporary (seemingly random) filename suffix for file names. "</p>
  </p>      "This is desired for thread-safety. "</p>
  </p>      "This behaviour may defeat caching compiler wrappers. "</p>
  </p>      "Default: true"}},</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/implicit_to_nlp.cpp: const Options ImplicitToNlp</summary>
  </p>= {{&Rootfinder::options_},</p>
  </p>   {{"nlpsol",</p>
  </p>     {OT_STRING,</p>
  </p>      "Name of solver."}},</p>
  </p>    {"nlpsol_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to solver."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/qrqp.cpp: const Options Qrqp</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"max_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of iterations [1000]."}},</p>
  </p>    {"constr_viol_tol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Constraint violation tolerance [1e-8]."}},</p>
  </p>    {"dual_inf_tol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Dual feasibility violation tolerance [1e-8]"}},</p>
  </p>    {"print_header",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print header [true]."}},</p>
  </p>    {"print_iter",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print iterations [true]."}},</p>
  </p>    {"print_info",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print info [true]."}},</p>
  </p>    {"print_lincomb",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print dependant linear combinations of constraints [false]. "</p>
  </p>      "Printed numbers are 0-based indices into the vector of [simple bounds;linear bounds]"}},</p>
  </p>    {"min_lam",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Smallest multiplier treated as inactive for the initial active set [0]."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/feasiblesqpmethod.cpp: const Options Feasiblesqpmethod</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>   {{"solve_type",</p>
  </p>     {OT_STRING,</p>
  </p>      "The solver type: Either SQP or SLP. Defaults to SQP"}},</p>
  </p>    {"qpsol",</p>
  </p>     {OT_STRING,</p>
  </p>      "The QP solver to be used by the SQP method [qpoases]"}},</p>
  </p>    {"qpsol_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to the QP solver"}},</p>
  </p>    {"hessian_approximation",</p>
  </p>     {OT_STRING,</p>
  </p>      "limited-memory|exact"}},</p>
  </p>    {"max_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of SQP iterations"}},</p>
  </p>    {"min_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Minimum number of SQP iterations"}},</p>
  </p>    {"tol_pr",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion for primal infeasibility"}},</p>
  </p>    {"tol_du",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion for dual infeasability"}},</p>
  </p>    {"merit_memory",</p>
  </p>     {OT_INT,</p>
  </p>      "Size of memory to store history of merit function values"}},</p>
  </p>    {"lbfgs_memory",</p>
  </p>     {OT_INT,</p>
  </p>      "Size of L-BFGS memory."}},</p>
  </p>    {"print_header",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print the header with problem statistics"}},</p>
  </p>    {"print_iteration",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print the iterations"}},</p>
  </p>    {"print_status",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print a status message after solving"}},</p>
  </p>    {"f",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the objective function (autogenerated by default)"}},</p>
  </p>    {"g",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the constraints (autogenerated by default)"}},</p>
  </p>    {"grad_f",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the gradient of the objective (autogenerated by default)"}},</p>
  </p>    {"jac_g",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the Jacobian of the constraints (autogenerated by default)"}},</p>
  </p>    {"hess_lag",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</p>
  </p>    {"convexify_strategy",</p>
  </p>     {OT_STRING,</p>
  </p>      "NONE|regularize|eigen-reflect|eigen-clip. "</p>
  </p>      "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</p>
  </p>    {"convexify_margin",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "When using a convexification strategy, make sure that "</p>
  </p>      "the smallest eigenvalue4 is at least this (default: 1e-7)."}},</p>
  </p>    {"max_iter_eig",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</p>
  </p>    {"init_feasible",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Initialize the QP subproblems with a feasible initial value (default: false)."}},</p>
  </p>    {"optim_tol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Optimality tolerance. Below this value an iterate is considered to be optimal."}},</p>
  </p>    {"feas_tol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Feasibility tolerance. Below this tolerance an iterate is considered to be feasible."}},</p>
  </p>    {"tr_rad0",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Initial trust-region radius."}},</p>
  </p>    {"tr_eta1",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Lower eta in trust-region acceptance criterion."}},</p>
  </p>    {"tr_eta2",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Upper eta in trust-region acceptance criterion."}},</p>
  </p>    {"tr_alpha1",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Lower alpha in trust-region size criterion."}},</p>
  </p>    {"tr_alpha2",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Upper alpha in trust-region size criterion."}},</p>
  </p>    {"tr_tol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Trust-region tolerance. "</p>
  </p>      "Below this value another scalar is equal to the trust region radius."}},</p>
  </p>    {"tr_acceptance",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Is the trust-region ratio above this value, the step is accepted."}},</p>
  </p>    {"tr_rad_min",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Minimum trust-region radius."}},</p>
  </p>    {"tr_rad_max",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Maximum trust-region radius."}},</p>
  </p>    {"tr_scale_vector",</p>
  </p>     {OT_DOUBLEVECTOR,</p>
  </p>      "Vector that tells where trust-region is applied."}},</p>
  </p>    {"contraction_acceptance_value",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "If the empirical contraction rate in the feasibility iterations "</p>
  </p>      "is above this value in the heuristics the iterations are aborted."}},</p>
  </p>    {"watchdog",</p>
  </p>     {OT_INT,</p>
  </p>      "Number of watchdog iterations in feasibility iterations. "</p>
  </p>      "After this amount of iterations, it is checked with the contraction acceptance value, "</p>
  </p>      "if iterations are converging."}},</p>
  </p>    {"max_inner_iter",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Maximum number of inner iterations."}},</p>
  </p>    {"use_anderson",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use Anderson Acceleration. (default false)"}},</p>
  </p>    {"anderson_memory",</p>
  </p>     {OT_INT,</p>
  </p>      "Anderson memory. If Anderson is used default is 1, else default is 0."}},</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/collocation.cpp: const Options Collocation</summary>
  </p>= {{&ImplicitFixedStepIntegrator::options_},</p>
  </p>   {{"interpolation_order",</p>
  </p>     {OT_INT,</p>
  </p>      "Order of the interpolating polynomials"}},</p>
  </p>    {"collocation_scheme",</p>
  </p>     {OT_STRING,</p>
  </p>      "Collocation scheme: radau|legendre"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/symbolic_qr.cpp: const Options SymbolicQr</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>  {{"fopts",</p>
  </p>    {OT_DICT,</p>
  </p>     "Options to be passed to generated function objects"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/bspline_interpolant.cpp: const Options BSplineInterpolant</summary>
  </p>= {{&Interpolant::options_},</p>
  </p>    {{"degree",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Sets, for each grid dimension, the degree of the spline."}},</p>
  </p>     {"linear_solver",</p>
  </p>      {OT_STRING,</p>
  </p>       "Solver used for constructing the coefficient tensor."}},</p>
  </p>     {"linear_solver_options",</p>
  </p>      {OT_DICT,</p>
  </p>       "Options to be passed to the linear solver."}},</p>
  </p>     {"algorithm",</p>
  </p>      {OT_STRING,</p>
  </p>       "Algorithm used for fitting the data: 'not_a_knot' (default, same as Matlab),"</p>
  </p>      " 'smooth_linear'."}},</p>
  </p>     {"smooth_linear_frac",</p>
  </p>      {OT_DOUBLE,</p>
  </p>       "When 'smooth_linear' algorithm is active, determines sharpness between"</p>
  </p>       " 0 (sharp, as linear interpolation) and 0.5 (smooth)."</p>
  </p>       "Default value is 0.1."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/solvers/qp_to_nlp.cpp: const Options QpToNlp</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"nlpsol",</p>
  </p>     {OT_STRING,</p>
  </p>      "Name of solver."}},</p>
  </p>    {"nlpsol_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to solver."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/hpipm/hpipm_interface.cpp: const Options HpipmInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"N",</p>
  </p>     {OT_INT,</p>
  </p>      "OCP horizon"}},</p>
  </p>    {"nx",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Number of states, length N+1"}},</p>
  </p>    {"nu",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Number of controls, length N"}},</p>
  </p>    {"ng",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Number of non-dynamic constraints, length N+1"}},</p>
  </p>    {"inf",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Replace infinities by this amount [default: 1e8]"}},</p>
  </p>    {"hpipm",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to hpipm"}}}</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/lapack/lapack_lu.cpp: const Options LapackLu</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>   {{"equilibration",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Equilibrate the matrix"}},</p>
  </p>    {"allow_equilibration_failure",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Non-fatal error when equilibration fails"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/lapack/lapack_qr.cpp: const Options LapackQr</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>   {{"max_nrhs",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of right-hand-sides that get processed in a single pass [default:10]."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/highs/highs_interface.cpp: const Options HighsInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"highs",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to HiGHS."</p>
  </p>      }},</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/alpaqa/alpaqa_interface.cpp: const Options AlpaqaInterface</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>    {{"alpaqa",</p>
  </p>      {OT_DICT,</p>
  </p>      "Options to be passed to Alpaqa"}}</p>
  </p>    }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/mumps/mumps_interface.cpp: const Options MumpsInterface</summary>
  </p>= {{&ProtoFunction::options_},</p>
  </p>   {{"symmetric",</p>
  </p>    {OT_BOOL,</p>
  </p>     "Symmetric matrix"}},</p>
  </p>    {"posdef",</p>
  </p>     {OT_BOOL,</p>
  </p>     "Positive definite"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/daqp/daqp_interface.cpp: const Options DaqpInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"daqp",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to Daqp."</p>
  </p>      }},</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/worhp/worhp_interface.cpp: const Options WorhpInterface</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>   {{"worhp",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to WORHP"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/cvodes_interface.cpp: const Options CvodesInterface</summary>
  </p>= {{&SundialsInterface::options_},</p>
  </p>  {{"linear_multistep_method",</p>
  </p>    {OT_STRING,</p>
  </p>    "Integrator scheme: BDF|adams"}},</p>
  </p>  {"nonlinear_solver_iteration",</p>
  </p>    {OT_STRING,</p>
  </p>    "Nonlinear solver type: NEWTON|functional"}},</p>
  </p>  {"min_step_size",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Min step size [default: 0/0.0]"}},</p>
  </p>  {"fsens_all_at_once",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Calculate all right hand sides of the sensitivity equations at once"}},</p>
  </p>  {"always_recalculate_jacobian",</p>
  </p>   {OT_BOOL,</p>
  </p>    "Recalculate Jacobian before factorizations, even if Jacobian is current [default: true]"}}</p>
  </p>  }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/kinsol_interface.cpp: const Options KinsolInterface</summary>
  </p>= {{&Rootfinder::options_},</p>
  </p>   {{"max_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of Newton iterations. Putting 0 sets the default value of KinSol."}},</p>
  </p>    {"print_level",</p>
  </p>     {OT_INT,</p>
  </p>      "Verbosity level"}},</p>
  </p>    {"abstol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Stopping criterion tolerance"}},</p>
  </p>    {"linear_solver_type",</p>
  </p>     {OT_STRING,</p>
  </p>      "dense|banded|iterative|user_defined"}},</p>
  </p>    {"upper_bandwidth",</p>
  </p>     {OT_INT,</p>
  </p>      "Upper bandwidth for banded linear solvers"}},</p>
  </p>    {"lower_bandwidth",</p>
  </p>     {OT_INT,</p>
  </p>      "Lower bandwidth for banded linear solvers"}},</p>
  </p>    {"max_krylov",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum Krylov space dimension"}},</p>
  </p>    {"exact_jacobian",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use exact Jacobian information"}},</p>
  </p>    {"iterative_solver",</p>
  </p>     {OT_STRING,</p>
  </p>      "gmres|bcgstab|tfqmr"}},</p>
  </p>    {"f_scale",</p>
  </p>     {OT_DOUBLEVECTOR,</p>
  </p>      "Equation scaling factors"}},</p>
  </p>    {"u_scale",</p>
  </p>     {OT_DOUBLEVECTOR,</p>
  </p>      "Variable scaling factors"}},</p>
  </p>    {"pretype",</p>
  </p>     {OT_STRING,</p>
  </p>      "Type of preconditioner"}},</p>
  </p>    {"use_preconditioner",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Precondition an iterative solver"}},</p>
  </p>    {"strategy",</p>
  </p>     {OT_STRING,</p>
  </p>      "Globalization strategy"}},</p>
  </p>    {"disable_internal_warnings",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Disable KINSOL internal warning messages"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/idas_interface.cpp: const Options IdasInterface</summary>
  </p>= {{&SundialsInterface::options_},</p>
  </p>  {{"suppress_algebraic",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Suppress algebraic variables in the error testing"}},</p>
  </p>  {"calc_ic",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Use IDACalcIC to get consistent initial conditions."}},</p>
  </p>  {"constraints",</p>
  </p>    {OT_INTVECTOR,</p>
  </p>    "Constrain the solution y=[x,z]. 0 (default): no constraint on yi, "</p>
  </p>    "1: yi >= 0.0, -1: yi <= 0.0, 2: yi > 0.0, -2: yi < 0.0."}},</p>
  </p>  {"calc_icB",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Use IDACalcIC to get consistent initial conditions for "</p>
  </p>    "backwards system [default: equal to calc_ic]."}},</p>
  </p>  {"abstolv",</p>
  </p>    {OT_DOUBLEVECTOR,</p>
  </p>    "Absolute tolerarance for each component"}},</p>
  </p>  {"max_step_size",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Maximim step size"}},</p>
  </p>  {"first_time",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "First requested time as a fraction of the time interval"}},</p>
  </p>  {"cj_scaling",</p>
  </p>    {OT_BOOL,</p>
  </p>    "IDAS scaling on cj for the user-defined linear solver module"}},</p>
  </p>  {"init_xdot",</p>
  </p>    {OT_DOUBLEVECTOR,</p>
  </p>    "Initial values for the state derivatives"}}</p>
  </p>  }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/sundials/sundials_interface.cpp: const Options SundialsInterface</summary>
  </p>= {{&Integrator::options_},</p>
  </p>  {{"max_num_steps",</p>
  </p>    {OT_INT,</p>
  </p>    "Maximum number of integrator steps"}},</p>
  </p>  {"reltol",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Relative tolerence for the IVP solution"}},</p>
  </p>  {"abstol",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Absolute tolerence for the IVP solution"}},</p>
  </p>  {"newton_scheme",</p>
  </p>    {OT_STRING,</p>
  </p>    "Linear solver scheme in the Newton method: DIRECT|gmres|bcgstab|tfqmr"}},</p>
  </p>  {"max_krylov",</p>
  </p>    {OT_INT,</p>
  </p>    "Maximum Krylov subspace size"}},</p>
  </p>  {"sensitivity_method",</p>
  </p>    {OT_STRING,</p>
  </p>    "Sensitivity method: SIMULTANEOUS|staggered"}},</p>
  </p>  {"max_multistep_order",</p>
  </p>    {OT_INT,</p>
  </p>    "Maximum order for the (variable-order) multistep method"}},</p>
  </p>  {"use_preconditioner",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Precondition the iterative solver [default: true]"}},</p>
  </p>  {"stop_at_end",</p>
  </p>    {OT_BOOL,</p>
  </p>    "[DEPRECATED] Stop the integrator at the end of the interval"}},</p>
  </p>  {"disable_internal_warnings",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Disable SUNDIALS internal warning messages"}},</p>
  </p>  {"quad_err_con",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Should the quadratures affect the step size control"}},</p>
  </p>  {"fsens_err_con",</p>
  </p>    {OT_BOOL,</p>
  </p>    "include the forward sensitivities in all error controls"}},</p>
  </p>  {"steps_per_checkpoint",</p>
  </p>    {OT_INT,</p>
  </p>    "Number of steps between two consecutive checkpoints"}},</p>
  </p>  {"interpolation_type",</p>
  </p>    {OT_STRING,</p>
  </p>    "Type of interpolation for the adjoint sensitivities"}},</p>
  </p>  {"linear_solver",</p>
  </p>    {OT_STRING,</p>
  </p>    "A custom linear solver creator function [default: qr]"}},</p>
  </p>  {"linear_solver_options",</p>
  </p>    {OT_DICT,</p>
  </p>    "Options to be passed to the linear solver"}},</p>
  </p>  {"second_order_correction",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Second order correction in the augmented system Jacobian [true]"}},</p>
  </p>  {"step0",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "initial step size [default: 0/estimated]"}},</p>
  </p>  {"max_step_size",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Max step size [default: 0/inf]"}},</p>
  </p>  {"max_order",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Maximum order"}},</p>
  </p>  {"nonlin_conv_coeff",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Coefficient in the nonlinear convergence test"}},</p>
  </p>  {"scale_abstol",</p>
  </p>   {OT_BOOL,</p>
  </p>    "Scale absolute tolerance by nominal value"}}</p>
  </p>  }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/gurobi/gurobi_interface.cpp: const Options GurobiInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"vtype",</p>
  </p>     {OT_STRINGVECTOR,</p>
  </p>      "Type of variables: [CONTINUOUS|binary|integer|semicont|semiint]"}},</p>
  </p>    {"gurobi",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to gurobi."}},</p>
  </p>    {"sos_groups",</p>
  </p>     {OT_INTVECTORVECTOR,</p>
  </p>      "Definition of SOS groups by indices."}},</p>
  </p>    {"sos_weights",</p>
  </p>     {OT_DOUBLEVECTORVECTOR,</p>
  </p>      "Weights corresponding to SOS entries."}},</p>
  </p>    {"sos_types",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Specify 1 or 2 for each SOS group."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/cplex/cplex_interface.cpp: const Options CplexInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"cplex",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to CPLEX"}},</p>
  </p>    {"qp_method",</p>
  </p>     {OT_INT,</p>
  </p>      "Determines which CPLEX algorithm to use."}},</p>
  </p>    {"dump_to_file",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Dumps QP to file in CPLEX format."}},</p>
  </p>    {"dump_filename",</p>
  </p>     {OT_STRING,</p>
  </p>      "The filename to dump to."}},</p>
  </p>    {"tol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Tolerance of solver"}},</p>
  </p>    {"dep_check",</p>
  </p>     {OT_INT,</p>
  </p>      "Detect redundant constraints."}},</p>
  </p>    {"warm_start",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use warm start with simplex methods (affects only the simplex methods)."}},</p>
  </p>    {"mip_start",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Hot start integers with x0 [Default false]."}},</p>
  </p>    {"sos_groups",</p>
  </p>     {OT_INTVECTORVECTOR,</p>
  </p>      "Definition of SOS groups by indices."}},</p>
  </p>    {"sos_weights",</p>
  </p>     {OT_DOUBLEVECTORVECTOR,</p>
  </p>      "Weights corresponding to SOS entries."}},</p>
  </p>    {"sos_types",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Specify 1 or 2 for each SOS group."}},</p>
  </p>    {"version_suffix",</p>
  </p>     {OT_STRING,</p>
  </p>      "Specify version of cplex to load. "</p>
  </p>      "We will attempt to load libcplex<version_suffix>.[so|dll|dylib]. "</p>
  </p>      "Default value is taken from CPLEX_VERSION env variable."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/ampl/ampl_interface.cpp: const Options AmplInterface</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>   {{"solver",</p>
  </p>     {OT_STRING,</p>
  </p>      "AMPL solver binary"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/fatrop/fatrop_conic_interface.cpp: const Options FatropConicInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"N",</p>
  </p>     {OT_INT,</p>
  </p>      "OCP horizon"}},</p>
  </p>    {"nx",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Number of states, length N+1"}},</p>
  </p>    {"nu",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Number of controls, length N"}},</p>
  </p>    {"ng",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Number of non-dynamic constraints, length N+1"}},</p>
  </p>    {"fatrop",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to fatrop"}}}</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/qpoases/qpoases_interface.cpp: const Options QpoasesInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"sparse",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Formulate the QP using sparse matrices. [false]"}},</p>
  </p>    {"schur",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use Schur Complement Approach [false]"}},</p>
  </p>    {"hessian_type",</p>
  </p>     {OT_STRING,</p>
  </p>      "Type of Hessian - see qpOASES documentation "</p>
  </p>      "[UNKNOWN|posdef|semidef|indef|zero|identity]]"}},</p>
  </p>    {"max_schur",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximal number of Schur updates [75]"}},</p>
  </p>    {"linsol_plugin",</p>
  </p>     {OT_STRING,</p>
  </p>      "Linear solver plugin"}},</p>
  </p>    {"nWSR",</p>
  </p>     {OT_INT,</p>
  </p>      "The maximum number of working set recalculations to be performed during "</p>
  </p>      "the initial homotopy. Default is 5(nx + nc)"}},</p>
  </p>    {"CPUtime",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "The maximum allowed CPU time in seconds for the whole initialisation"</p>
  </p>      " (and the actually required one on output). Disabled if unset."}},</p>
  </p>    {"printLevel",</p>
  </p>     {OT_STRING,</p>
  </p>      "Defines the amount of text output during QP solution, see Section 5.7"}},</p>
  </p>    {"enableRamping",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Enables ramping."}},</p>
  </p>    {"enableFarBounds",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Enables the use of  far bounds."}},</p>
  </p>    {"enableFlippingBounds",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Enables the use of  flipping bounds."}},</p>
  </p>    {"enableRegularisation",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Enables automatic  Hessian regularisation."}},</p>
  </p>    {"enableFullLITests",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Enables condition-hardened  (but more expensive) LI test."}},</p>
  </p>    {"enableNZCTests",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Enables nonzero curvature  tests."}},</p>
  </p>    {"enableDriftCorrection",</p>
  </p>     {OT_INT,</p>
  </p>      "Specifies the frequency of drift corrections: 0: turns them off."}},</p>
  </p>    {"enableCholeskyRefactorisation",</p>
  </p>     {OT_INT,</p>
  </p>      "Specifies the frequency of a full re-factorisation of projected "</p>
  </p>      "Hessian matrix: 0: turns them off,  1: uses them at each iteration etc."}},</p>
  </p>    {"enableEqualities",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Specifies whether equalities should be treated  as always active "</p>
  </p>      "(True) or not (False)"}},</p>
  </p>    {"terminationTolerance",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Relative termination tolerance to stop homotopy."}},</p>
  </p>    {"boundTolerance",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "If upper and lower bounds differ less than this tolerance, they are regarded "</p>
  </p>      "equal, i.e. as  equality constraint."}},</p>
  </p>    {"boundRelaxation",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Initial relaxation of bounds to start homotopy  and initial value for far bounds."}},</p>
  </p>    {"epsNum",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Numerator tolerance for ratio tests."}},</p>
  </p>    {"epsDen",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Denominator tolerance for ratio tests."}},</p>
  </p>    {"maxPrimalJump",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Maximum allowed jump in primal variables in  nonzero curvature tests."}},</p>
  </p>    {"maxDualJump",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Maximum allowed jump in dual variables in  linear independence tests."}},</p>
  </p>    {"initialRamping",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Start value for ramping strategy."}},</p>
  </p>    {"finalRamping",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Final value for ramping strategy."}},</p>
  </p>    {"initialFarBounds",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Initial size for far bounds."}},</p>
  </p>    {"growFarBounds",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Factor to grow far bounds."}},</p>
  </p>    {"initialStatusBounds",</p>
  </p>     {OT_STRING,</p>
  </p>      "Initial status of bounds at first iteration."}},</p>
  </p>    {"epsFlipping",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Tolerance of squared Cholesky diagonal factor  which triggers flipping bound."}},</p>
  </p>    {"numRegularisationSteps",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of successive regularisation steps."}},</p>
  </p>    {"epsRegularisation",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Scaling factor of identity matrix used for  Hessian regularisation."}},</p>
  </p>    {"numRefinementSteps",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of iterative refinement steps."}},</p>
  </p>    {"epsIterRef",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Early termination tolerance for iterative  refinement."}},</p>
  </p>    {"epsLITests",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Tolerance for linear independence tests."}},</p>
  </p>    {"epsNZCTests",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Tolerance for nonzero curvature tests."}},</p>
  </p>    {"enableInertiaCorrection",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Should working set be repaired when negative curvature is discovered during hotstart."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/clp/clp_interface.cpp: const Options ClpInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"clp",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to CLP. "</p>
  </p>      "A first set of options can be found in ClpParameters.hpp. eg. 'PrimalTolerance'. "</p>
  </p>      "There are other options in additions. "</p>
  </p>      "'AutomaticScaling' (bool) is recognised. "</p>
  </p>      "'initial_solve' (default off) activates the use of Clp's initialSolve. "</p>
  </p>      "'initial_solve_options' takes a dictionary with following keys (see ClpSolve.hpp): "</p>
  </p>      " SolveType (string), PresolveType (string), "</p>
  </p>      " NumberPasses, SpecialOptions (intvectorvector), IndependentOptions (intvectorvector)."</p>
  </p>      }}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/snopt/snopt_interface.cpp: const Options SnoptInterface</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>   {{"snopt",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to SNOPT"}},</p>
  </p>    {"start",</p>
  </p>     {OT_STRING,</p>
  </p>      "Warm-start options for Worhp: cold|warm|hot"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/knitro/knitro_interface.cpp: const Options KnitroInterface</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>   {{"knitro",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to KNITRO"}},</p>
  </p>    {"options_file",</p>
  </p>     {OT_STRING,</p>
  </p>      "Read options from file (solver specific)"}},</p>
  </p>    {"detect_linear_constraints",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Detect type of constraints"}},</p>
  </p>    {"contype",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Type of constraint"}},</p>
  </p>    {"complem_variables",</p>
  </p>     {OT_INTVECTORVECTOR,</p>
  </p>      "List of complementary constraints on simple bounds. "</p>
  </p>      "Pair (i, j) encodes complementarity between the bounds on variable i and variable j."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/ooqp/ooqp_interface.cpp: const Options OoqpInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"print_level",</p>
  </p>     {OT_INT,</p>
  </p>      "Print level. OOQP listens to print_level 0, 10 and 100"}},</p>
  </p>    {"mutol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "tolerance as provided with setMuTol to OOQP"}},</p>
  </p>    {"artol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "tolerance as provided with setArTol to OOQP"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/sleqp/sleqp_interface.cpp: const Options SLEQPInterface</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>    {{"sleqp",</p>
  </p>      {OT_DICT,</p>
  </p>      "Options to be passed to SLEQP"}},</p>
  </p>     {"print_level",</p>
  </p>      {OT_INT,</p>
  </p>      "Print level of SLEQP (default: 2/SLEQP_LOG_WARN)"}},</p>
  </p>     {"max_iter",</p>
  </p>      {OT_INT,</p>
  </p>      "Maximum number of iterations"}},</p>
  </p>     {"max_wall_time",</p>
  </p>      {OT_DOUBLE,</p>
  </p>      "maximum wall time allowed"}}</p>
  </p>    }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/superscs/superscs_interface.cpp: const Options SuperscsInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"superscs",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to superscs."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/hpmpc/hpmpc_interface.cpp: const Options HpmpcInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"N",</p>
  </p>     {OT_INT,</p>
  </p>      "OCP horizon"}},</p>
  </p>    {"nx",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Number of states, length N+1"}},</p>
  </p>    {"nu",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Number of controls, length N"}},</p>
  </p>    {"ng",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Number of non-dynamic constraints, length N+1"}},</p>
  </p>    {"mu0",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Max element in cost function as estimate of max multiplier"}},</p>
  </p>    {"max_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Max number of iterations"}},</p>
  </p>    {"tol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Tolerance in the duality measure"}},</p>
  </p>    {"warm_start",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use warm-starting"}},</p>
  </p>    {"inf",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "HPMPC cannot handle infinities. Infinities will be replaced by this option's value."}},</p>
  </p>    {"print_level",</p>
  </p>     {OT_INT,</p>
  </p>      "Amount of diagnostic printing [Default: 1]."}},</p>
  </p>    {"target",</p>
  </p>     {OT_STRING,</p>
  </p>      "hpmpc target"}},</p>
  </p>    {"blasfeo_target",</p>
  </p>     {OT_STRING,</p>
  </p>      "hpmpc target"}}}</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/slicot/slicot_dple.cpp: const Options SlicotDple</summary>
  </p>= {{&Dple::options_},</p>
  </p>   {{"linear_solver",</p>
  </p>     {OT_STRING,</p>
  </p>      "User-defined linear solver class. Needed for sensitivities."}},</p>
  </p>    {"linear_solver_options",</p>
  </p>      {OT_DICT,</p>
  </p>       "Options to be passed to the linear solver."}},</p>
  </p>    {"psd_num_zero",</p>
  </p>      {OT_DOUBLE,</p>
  </p>        "Numerical zero used in Periodic Schur decomposition with slicot."</p>
  </p>        "This option is needed when your systems has Floquet multipliers"</p>
  </p>        "zero or close to zero"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/blocksqp/blocksqp.cpp: const Options Blocksqp</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>   {{"qpsol",</p>
  </p>     {OT_STRING,</p>
  </p>      "The QP solver to be used by the SQP method"}},</p>
  </p>    {"qpsol_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to the QP solver"}},</p>
  </p>    {"linsol",</p>
  </p>     {OT_STRING,</p>
  </p>      "The linear solver to be used by the QP method"}},</p>
  </p>    {"print_header",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print solver header at startup"}},</p>
  </p>    {"print_iteration",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print SQP iterations"}},</p>
  </p>    {"eps",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Values smaller than this are regarded as numerically zero"}},</p>
  </p>    {"opttol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Optimality tolerance"}},</p>
  </p>    {"nlinfeastol",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Nonlinear feasibility tolerance"}},</p>
  </p>    {"schur",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use qpOASES Schur compliment approach"}},</p>
  </p>    {"globalization",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Enable globalization"}},</p>
  </p>    {"restore_feas",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use feasibility restoration phase"}},</p>
  </p>    {"max_line_search",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of steps in line search"}},</p>
  </p>    {"max_consec_reduced_steps",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of consecutive reduced steps"}},</p>
  </p>    {"max_consec_skipped_updates",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of consecutive skipped updates"}},</p>
  </p>    {"max_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of SQP iterations"}},</p>
  </p>    {"warmstart",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use warmstarting"}},</p>
  </p>    {"qp_init",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use warmstarting"}},</p>
  </p>    {"max_it_qp",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of QP iterations per SQP iteration"}},</p>
  </p>    {"block_hess",</p>
  </p>     {OT_INT,</p>
  </p>      "Blockwise Hessian approximation?"}},</p>
  </p>    {"hess_scaling",</p>
  </p>     {OT_INT,</p>
  </p>      "Scaling strategy for Hessian approximation"}},</p>
  </p>    {"fallback_scaling",</p>
  </p>     {OT_INT,</p>
  </p>      "If indefinite update is used, the type of fallback strategy"}},</p>
  </p>    {"max_time_qp",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Maximum number of time in seconds per QP solve per SQP iteration"}},</p>
  </p>    {"ini_hess_diag",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Initial Hessian guess: diagonal matrix diag(iniHessDiag)"}},</p>
  </p>    {"col_eps",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Epsilon for COL scaling strategy"}},</p>
  </p>    {"col_tau1",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "tau1 for COL scaling strategy"}},</p>
  </p>    {"col_tau2",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "tau2 for COL scaling strategy"}},</p>
  </p>    {"hess_damp",</p>
  </p>     {OT_INT,</p>
  </p>      "Activate Powell damping for BFGS"}},</p>
  </p>    {"hess_damp_fac",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Damping factor for BFGS Powell modification"}},</p>
  </p>    {"hess_update",</p>
  </p>     {OT_INT,</p>
  </p>      "Type of Hessian approximation"}},</p>
  </p>    {"fallback_update",</p>
  </p>     {OT_INT,</p>
  </p>      "If indefinite update is used, the type of fallback strategy"}},</p>
  </p>    {"hess_lim_mem",</p>
  </p>     {OT_INT,</p>
  </p>      "Full or limited memory"}},</p>
  </p>    {"hess_memsize",</p>
  </p>     {OT_INT,</p>
  </p>      "Memory size for L-BFGS updates"}},</p>
  </p>    {"which_second_derv",</p>
  </p>     {OT_INT,</p>
  </p>      "For which block should second derivatives be provided by the user"}},</p>
  </p>    {"skip_first_globalization",</p>
  </p>     {OT_BOOL,</p>
  </p>      "No globalization strategy in first iteration"}},</p>
  </p>    {"conv_strategy",</p>
  </p>     {OT_INT,</p>
  </p>      "Convexification strategy"}},</p>
  </p>    {"max_conv_qp",</p>
  </p>     {OT_INT,</p>
  </p>      "How many additional QPs may be solved for convexification per iteration?"}},</p>
  </p>    {"max_soc_iter",</p>
  </p>     {OT_INT,</p>
  </p>      "Maximum number of SOC line search iterations"}},</p>
  </p>    {"gamma_theta",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"gamma_f",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"kappa_soc",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"kappa_f",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"theta_max",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"theta_min",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"delta",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"s_theta",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"s_f",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"kappa_minus",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"kappa_plus",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"kappa_plus_max",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"delta_h0",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"eta",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Filter line search parameter, cf. IPOPT paper"}},</p>
  </p>    {"obj_lo",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Lower bound on objective function [-inf]"}},</p>
  </p>    {"obj_up",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Upper bound on objective function [inf]"}},</p>
  </p>    {"rho",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Feasibility restoration phase parameter"}},</p>
  </p>    {"zeta",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Feasibility restoration phase parameter"}},</p>
  </p>    {"print_maxit_reached",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print error when maximum number of SQP iterations reached"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/ipopt/ipopt_interface.cpp: const Options IpoptInterface</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>   {{"pass_nonlinear_variables",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Pass list of variables entering nonlinearly to IPOPT"}},</p>
  </p>    {"ipopt",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to IPOPT"}},</p>
  </p>    {"var_string_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "String metadata (a dictionary with lists of strings) "</p>
  </p>      "about variables to be passed to IPOPT"}},</p>
  </p>    {"var_integer_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "Integer metadata (a dictionary with lists of integers) "</p>
  </p>      "about variables to be passed to IPOPT"}},</p>
  </p>    {"var_numeric_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "Numeric metadata (a dictionary with lists of reals) about "</p>
  </p>      "variables to be passed to IPOPT"}},</p>
  </p>    {"con_string_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "String metadata (a dictionary with lists of strings) about "</p>
  </p>      "constraints to be passed to IPOPT"}},</p>
  </p>    {"con_integer_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "Integer metadata (a dictionary with lists of integers) "</p>
  </p>      "about constraints to be passed to IPOPT"}},</p>
  </p>    {"con_numeric_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "Numeric metadata (a dictionary with lists of reals) about "</p>
  </p>      "constraints to be passed to IPOPT"}},</p>
  </p>    {"hess_lag",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</p>
  </p>    {"jac_g",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the Jacobian of the constraints "</p>
  </p>      "(autogenerated by default)"}},</p>
  </p>    {"grad_f",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the gradient of the objective "</p>
  </p>      "(column, autogenerated by default)"}},</p>
  </p>    {"convexify_strategy",</p>
  </p>     {OT_STRING,</p>
  </p>      "NONE|regularize|eigen-reflect|eigen-clip. "</p>
  </p>      "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},</p>
  </p>    {"convexify_margin",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "When using a convexification strategy, make sure that "</p>
  </p>      "the smallest eigenvalue is at least this (default: 1e-7)."}},</p>
  </p>    {"max_iter_eig",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},</p>
  </p>    {"clip_inactive_lam",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Explicitly set Lagrange multipliers to 0 when bound is deemed inactive "</p>
  </p>      "(default: false)."}},</p>
  </p>    {"inactive_lam_strategy",</p>
  </p>     {OT_STRING,</p>
  </p>      "Strategy to detect if a bound is inactive. "</p>
  </p>      "RELTOL: use solver-defined constraint tolerance * inactive_lam_value|"</p>
  </p>      "abstol: use inactive_lam_value"}},</p>
  </p>    {"inactive_lam_value",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Value used in inactive_lam_strategy (default: 10)."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/clang/clang_compiler.cpp: const Options ClangCompiler</summary>
  </p>= {{&ImporterInternal::options_},</p>
  </p>   {{"include_path",</p>
  </p>     {OT_STRING,</p>
  </p>      "Include paths for the JIT compiler. "</p>
  </p>      "The include directory shipped with CasADi will be automatically appended."}},</p>
  </p>    {"flags",</p>
  </p>     {OT_STRINGVECTOR,</p>
  </p>      "Compile flags for the JIT compiler. Default: None"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/proxqp/proxqp_interface.cpp: const Options ProxqpInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"proxqp",</p>
  </p>     {OT_DICT,</p>
  </p>      "const proxqp options."}},</p>
  </p>    {"warm_start_primal",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use x input to warmstart [Default: true]."}},</p>
  </p>    {"warm_start_dual",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use y and z input to warmstart [Default: true]."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/bonmin/bonmin_interface.cpp: const Options BonminInterface</summary>
  </p>= {{&Nlpsol::options_},</p>
  </p>   {{"pass_nonlinear_variables",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Pass list of variables entering nonlinearly to BONMIN"}},</p>
  </p>    {"pass_nonlinear_constraints",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Pass list of constraints entering nonlinearly to BONMIN"}},</p>
  </p>    {"bonmin",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to BONMIN"}},</p>
  </p>    {"var_string_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "String metadata (a dictionary with lists of strings) "</p>
  </p>      "about variables to be passed to BONMIN"}},</p>
  </p>    {"var_integer_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "Integer metadata (a dictionary with lists of integers) "</p>
  </p>      "about variables to be passed to BONMIN"}},</p>
  </p>    {"var_numeric_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "Numeric metadata (a dictionary with lists of reals) about "</p>
  </p>      "variables to be passed to BONMIN"}},</p>
  </p>    {"con_string_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "String metadata (a dictionary with lists of strings) about "</p>
  </p>      "constraints to be passed to BONMIN"}},</p>
  </p>    {"con_integer_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "Integer metadata (a dictionary with lists of integers) "</p>
  </p>      "about constraints to be passed to BONMIN"}},</p>
  </p>    {"con_numeric_md",</p>
  </p>     {OT_DICT,</p>
  </p>      "Numeric metadata (a dictionary with lists of reals) about "</p>
  </p>      "constraints to be passed to BONMIN"}},</p>
  </p>    {"hess_lag",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},</p>
  </p>    {"hess_lag_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options for the autogenerated Hessian of the Lagrangian."}},</p>
  </p>    {"jac_g",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the Jacobian of the constraints "</p>
  </p>      "(autogenerated by default)"}},</p>
  </p>    {"jac_g_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options for the autogenerated Jacobian of the constraints."}},</p>
  </p>    {"grad_f",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function for calculating the gradient of the objective "</p>
  </p>      "(column, autogenerated by default)"}},</p>
  </p>    {"grad_f_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options for the autogenerated gradient of the objective."}},</p>
  </p>    {"sos1_groups",</p>
  </p>     {OT_INTVECTORVECTOR,</p>
  </p>      "Options for the autogenerated gradient of the objective."}},</p>
  </p>    {"sos1_weights",</p>
  </p>     {OT_DOUBLEVECTORVECTOR,</p>
  </p>      "Options for the autogenerated gradient of the objective."}},</p>
  </p>    {"sos1_priorities",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Options for the autogenerated gradient of the objective."}},</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/cbc/cbc_interface.cpp: const Options CbcInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"cbc",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to CBC."</p>
  </p>      "Three sets of options are supported. "</p>
  </p>      "The first can be found in OsiSolverParameters.hpp. "</p>
  </p>      "The second can be found in CbcModel.hpp. "</p>
  </p>      "The third are options that can be passed to CbcMain1."</p>
  </p>      }},</p>
  </p>    {"sos_groups",</p>
  </p>     {OT_INTVECTORVECTOR,</p>
  </p>      "Definition of SOS groups by indices."}},</p>
  </p>    {"sos_weights",</p>
  </p>     {OT_DOUBLEVECTORVECTOR,</p>
  </p>      "Weights corresponding to SOS entries."}},</p>
  </p>    {"sos_types",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Specify 1 or 2 for each SOS group."}},</p>
  </p>    {"hot_start",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Hot start with x0 [Default false]."}},</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/interfaces/osqp/osqp_interface.cpp: const Options OsqpInterface</summary>
  </p>= {{&Conic::options_},</p>
  </p>   {{"osqp",</p>
  </p>     {OT_DICT,</p>
  </p>      "const Options to be passed to osqp."}},</p>
  </p>    {"warm_start_primal",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use x0 input to warmstart [Default: true]."}},</p>
  </p>    {"warm_start_dual",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use lam_a0 and lam_x0 input to warmstart [Default: truw]."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/sx_function.cpp: const Options SXFunction</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>   {{"default_in",</p>
  </p>     {OT_DOUBLEVECTOR,</p>
  </p>      "Default input values"}},</p>
  </p>    {"just_in_time_sparsity",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Propagate sparsity patterns using just-in-time "</p>
  </p>      "compilation to a CPU or GPU using OpenCL"}},</p>
  </p>    {"just_in_time_opencl",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Just-in-time compilation for numeric evaluation using OpenCL (experimental)"}},</p>
  </p>    {"live_variables",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Reuse variables in the work vector"}},</p>
  </p>    {"cse",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Perform common subexpression elimination (complexity is N*log(N) in graph size)"}},</p>
  </p>    {"allow_free",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Allow construction with free variables (Default: false)"}},</p>
  </p>    {"allow_duplicate_io_names",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Allow construction with duplicate io names (Default: false)"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options Integrator</summary>
  </p>= {{&OracleFunction::options_},</p>
  </p>  {{"expand",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Replace MX with SX expressions in problem formulation [false]"}},</p>
  </p>  {"print_stats",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Print out statistics after integration"}},</p>
  </p>  {"nfwd",</p>
  </p>   {OT_INT,</p>
  </p>    "Number of forward sensitivities to be calculated [0]"}},</p>
  </p>  {"nadj",</p>
  </p>   {OT_INT,</p>
  </p>    "Number of adjoint sensitivities to be calculated [0]"}},</p>
  </p>  {"t0",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "[DEPRECATED] Beginning of the time horizon"}},</p>
  </p>  {"tf",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "[DEPRECATED] End of the time horizon"}},</p>
  </p>  {"grid",</p>
  </p>    {OT_DOUBLEVECTOR,</p>
  </p>    "[DEPRECATED] Time grid"}},</p>
  </p>  {"augmented_options",</p>
  </p>    {OT_DICT,</p>
  </p>    "Options to be passed down to the augmented integrator, if one is constructed."}},</p>
  </p>  {"output_t0",</p>
  </p>    {OT_BOOL,</p>
  </p>    "[DEPRECATED] Output the state at the initial time"}}</p>
  </p>  }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options FixedStepIntegrator</summary>
  </p>= {{&Integrator::options_},</p>
  </p>  {{"number_of_finite_elements",</p>
  </p>    {OT_INT,</p>
  </p>    "Target number of finite elements. "</p>
  </p>    "The actual number may be higher to accommodate all output times"}},</p>
  </p>  {"simplify",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Implement as MX Function (codegeneratable/serializable) default: false"}},</p>
  </p>  {"simplify_options",</p>
  </p>    {OT_DICT,</p>
  </p>    "Any options to pass to simplified form Function constructor"}}</p>
  </p>  }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/integrator.cpp: const Options ImplicitFixedStepIntegrator</summary>
  </p>= {{&FixedStepIntegrator::options_},</p>
  </p>  {{"rootfinder",</p>
  </p>    {OT_STRING,</p>
  </p>    "An implicit function solver"}},</p>
  </p>  {"rootfinder_options",</p>
  </p>    {OT_DICT,</p>
  </p>    "Options to be passed to the NLP Solver"}}</p>
  </p>  }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/conic.cpp: const Options Conic</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>   {{"discrete",</p>
  </p>     {OT_BOOLVECTOR,</p>
  </p>      "Indicates which of the variables are discrete, i.e. integer-valued"}},</p>
  </p>    {"print_problem",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print a numeric description of the problem"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/rootfinder.cpp: const Options Rootfinder</summary>
  </p>= {{&OracleFunction::options_},</p>
  </p>   {{"linear_solver",</p>
  </p>     {OT_STRING,</p>
  </p>      "User-defined linear solver class. Needed for sensitivities."}},</p>
  </p>    {"linear_solver_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to the linear solver."}},</p>
  </p>    {"constraints",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "Constrain the unknowns. 0 (default): no constraint on ui, "</p>
  </p>      "1: ui >= 0.0, -1: ui <= 0.0, 2: ui > 0.0, -2: ui < 0.0."}},</p>
  </p>    {"implicit_input",</p>
  </p>     {OT_INT,</p>
  </p>      "Index of the input that corresponds to the actual root-finding"}},</p>
  </p>    {"implicit_output",</p>
  </p>     {OT_INT,</p>
  </p>      "Index of the output that corresponds to the actual root-finding"}},</p>
  </p>    {"jacobian_function",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Function object for calculating the Jacobian (autogenerated by default)"}},</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/mx_function.cpp: const Options MXFunction</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>   {{"default_in",</p>
  </p>     {OT_DOUBLEVECTOR,</p>
  </p>      "Default input values"}},</p>
  </p>    {"live_variables",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Reuse variables in the work vector"}},</p>
  </p>    {"print_instructions",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print each operation during evaluation"}},</p>
  </p>    {"cse",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Perform common subexpression elimination (complexity is N*log(N) in graph size)"}},</p>
  </p>    {"allow_free",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Allow construction with free variables (Default: false)"}},</p>
  </p>    {"allow_duplicate_io_names",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Allow construction with duplicate io names (Default: false)"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/expm.cpp: const Options Expm</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>   {{"const_A",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Assume A is constant. Default: false."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/finite_differences.cpp: const Options FiniteDiff</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>  {{"second_order_stepsize",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Second order perturbation size [default: 1e-3]"}},</p>
  </p>  {"h",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Step size [default: computed from abstol]"}},</p>
  </p>  {"h_max",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Maximum step size [default 0]"}},</p>
  </p>  {"h_min",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Minimum step size [default inf]"}},</p>
  </p>  {"smoothing",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Smoothing regularization [default: machine precision]"}},</p>
  </p>  {"reltol",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Accuracy of function inputs [default: query object]"}},</p>
  </p>  {"abstol",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Accuracy of function outputs [default: query object]"}},</p>
  </p>  {"u_aim",</p>
  </p>    {OT_DOUBLE,</p>
  </p>    "Target ratio of roundoff error to truncation error [default: 100.]"}},</p>
  </p>  {"h_iter",</p>
  </p>    {OT_INT,</p>
  </p>    "Number of iterations to improve on the step-size "</p>
  </p>    "[default: 1 if error estimate available, otherwise 0]"}},</p>
  </p>  }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/jit_function.cpp: const Options JitFunction</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>   {{"buffered",</p>
  </p>    {OT_BOOL,</p>
  </p>      "Buffer the calls, user does not need to "}},</p>
  </p>     {"jac",</p>
  </p>    {OT_STRING,</p>
  </p>      "Function body for Jacobian"}},</p>
  </p>    {"hess",</p>
  </p>     {OT_STRING,</p>
  </p>      "Function body for Hessian"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/function_internal.cpp: const Options ProtoFunction</summary>
  </p>= {{},</p>
  </p>   {{"verbose",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Verbose evaluation -- for debugging"}},</p>
  </p>    {"print_time",</p>
  </p>     {OT_BOOL,</p>
  </p>      "print information about execution time. Implies record_time."}},</p>
  </p>    {"record_time",</p>
  </p>     {OT_BOOL,</p>
  </p>      "record information about execution time, for retrieval with stats()."}},</p>
  </p>    {"regularity_check",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Throw exceptions when NaN or Inf appears during evaluation"}},</p>
  </p>    {"error_on_fail",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Throw exceptions when function evaluation fails (default true)."}}</p>
  </p>    }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/function_internal.cpp: const Options FunctionInternal</summary>
  </p>= {{&ProtoFunction::options_},</p>
  </p>    {{"ad_weight",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Weighting factor for derivative calculation."</p>
  </p>      "When there is an option of either using forward or reverse mode "</p>
  </p>      "directional derivatives, the condition ad_weight*nf<=(1-ad_weight)*na "</p>
  </p>      "is used where nf and na are estimates of the number of forward/reverse "</p>
  </p>      "mode directional derivatives needed. By default, ad_weight is calculated "</p>
  </p>      "automatically, but this can be overridden by setting this option. "</p>
  </p>      "In particular, 0 means forcing forward mode and 1 forcing reverse mode. "</p>
  </p>      "Leave unset for (class specific) heuristics."}},</p>
  </p>    {"ad_weight_sp",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Weighting factor for sparsity pattern calculation calculation."</p>
  </p>      "Overrides default behavior. Set to 0 and 1 to force forward and "</p>
  </p>      "reverse mode respectively. Cf. option \"ad_weight\". "</p>
  </p>      "When set to -1, sparsity is completely ignored and dense matrices are used."}},</p>
  </p>    {"always_inline",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Force inlining."}},</p>
  </p>    {"never_inline",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Forbid inlining."}},</p>
  </p>    {"jac_penalty",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "When requested for a number of forward/reverse directions,   "</p>
  </p>      "it may be cheaper to compute first the full jacobian and then "</p>
  </p>      "multiply with seeds, rather than obtain the requested directions "</p>
  </p>      "in a straightforward manner. "</p>
  </p>      "Casadi uses a heuristic to decide which is cheaper. "</p>
  </p>      "A high value of 'jac_penalty' makes it less likely for the heurstic "</p>
  </p>      "to chose the full Jacobian strategy. "</p>
  </p>      "The special value -1 indicates never to use the full Jacobian strategy"}},</p>
  </p>    {"user_data",</p>
  </p>     {OT_VOIDPTR,</p>
  </p>      "A user-defined field that can be used to identify "</p>
  </p>      "the function or pass additional information"}},</p>
  </p>    {"inputs_check",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Throw exceptions when the numerical values of the inputs don't make sense"}},</p>
  </p>    {"gather_stats",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Deprecated option (ignored): Statistics are now always collected."}},</p>
  </p>    {"input_scheme",</p>
  </p>     {OT_STRINGVECTOR,</p>
  </p>      "Deprecated option (ignored)"}},</p>
  </p>    {"output_scheme",</p>
  </p>     {OT_STRINGVECTOR,</p>
  </p>      "Deprecated option (ignored)"}},</p>
  </p>    {"jit",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use just-in-time compiler to speed up the evaluation"}},</p>
  </p>    {"jit_cleanup",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Cleanup up the temporary source file that jit creates. Default: true"}},</p>
  </p>    {"jit_serialize",</p>
  </p>     {OT_STRING,</p>
  </p>      "Specify behaviour when serializing a jitted function: SOURCE|link|embed."}},</p>
  </p>    {"jit_name",</p>
  </p>     {OT_STRING,</p>
  </p>      "The file name used to write out code. "</p>
  </p>      "The actual file names used depend on 'jit_temp_suffix' and include extensions. "</p>
  </p>      "Default: 'jit_tmp'"}},</p>
  </p>    {"jit_temp_suffix",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Use a temporary (seemingly random) filename suffix for generated code and libraries. "</p>
  </p>      "This is desired for thread-safety. "</p>
  </p>      "This behaviour may defeat caching compiler wrappers. "</p>
  </p>      "Default: true"}},</p>
  </p>    {"compiler",</p>
  </p>     {OT_STRING,</p>
  </p>      "Just-in-time compiler plugin to be used."}},</p>
  </p>    {"jit_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to the jit compiler."}},</p>
  </p>    {"derivative_of",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "The function is a derivative of another function. "</p>
  </p>      "The type of derivative (directional derivative, Jacobian) "</p>
  </p>      "is inferred from the function name."}},</p>
  </p>    {"max_num_dir",</p>
  </p>     {OT_INT,</p>
  </p>      "Specify the maximum number of directions for derivative functions."</p>
  </p>      " Overrules the builtin optimized_num_dir."}},</p>
  </p>    {"enable_forward",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Enable derivative calculation using generated functions for"</p>
  </p>      " Jacobian-times-vector products - typically using forward mode AD"</p>
  </p>      " - if available. [default: true]"}},</p>
  </p>    {"enable_reverse",</p>
  </p>      {OT_BOOL,</p>
  </p>      "Enable derivative calculation using generated functions for"</p>
  </p>      " transposed Jacobian-times-vector products - typically using reverse mode AD"</p>
  </p>      " - if available. [default: true]"}},</p>
  </p>    {"enable_jacobian",</p>
  </p>      {OT_BOOL,</p>
  </p>      "Enable derivative calculation using generated functions for"</p>
  </p>      " Jacobians of all differentiable outputs with respect to all differentiable inputs"</p>
  </p>      " - if available. [default: true]"}},</p>
  </p>    {"enable_fd",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Enable derivative calculation by finite differencing. [default: false]]"}},</p>
  </p>    {"fd_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to the finite difference instance"}},</p>
  </p>    {"fd_method",</p>
  </p>     {OT_STRING,</p>
  </p>      "Method for finite differencing [default 'central']"}},</p>
  </p>    {"print_in",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print numerical values of inputs [default: false]"}},</p>
  </p>    {"print_out",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print numerical values of outputs [default: false]"}},</p>
  </p>    {"max_io",</p>
  </p>     {OT_INT,</p>
  </p>      "Acceptable number of inputs and outputs. Warn if exceeded."}},</p>
  </p>    {"dump_in",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Dump numerical values of inputs to file (readable with DM.from_file) [default: false]"}},</p>
  </p>    {"dump_out",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Dump numerical values of outputs to file (readable with DM.from_file) [default: false]"}},</p>
  </p>    {"dump",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Dump function to file upon first evaluation. [false]"}},</p>
  </p>    {"dump_dir",</p>
  </p>     {OT_STRING,</p>
  </p>      "Directory to dump inputs/outputs to. Make sure the directory exists [.]"}},</p>
  </p>    {"dump_format",</p>
  </p>     {OT_STRING,</p>
  </p>      "Choose file format to dump matrices. See DM.from_file [mtx]"}},</p>
  </p>    {"forward_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to a forward mode constructor"}},</p>
  </p>    {"reverse_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to a reverse mode constructor"}},</p>
  </p>    {"jacobian_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to a Jacobian constructor"}},</p>
  </p>    {"der_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Default options to be used to populate forward_options, reverse_options, and "</p>
  </p>      "jacobian_options before those options are merged in."}},</p>
  </p>    {"custom_jacobian",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "Override CasADi's AD. Use together with 'jac_penalty': 0. "</p>
  </p>      "Note: Highly experimental. Syntax may break often."}},</p>
  </p>    {"is_diff_in",</p>
  </p>     {OT_BOOLVECTOR,</p>
  </p>      "Indicate for each input if it should be differentiable."}},</p>
  </p>    {"is_diff_out",</p>
  </p>     {OT_BOOLVECTOR,</p>
  </p>      "Indicate for each output if it should be differentiable."}},</p>
  </p>    {"post_expand",</p>
  </p>     {OT_BOOL,</p>
  </p>      "After construction, expand this Function. Default: False"}},</p>
  </p>    {"post_expand_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to post-construction expansion. Default: empty"}},</p>
  </p>    {"cache",</p>
  </p>     {OT_DICT,</p>
  </p>      "Prepopulate the function cache. Default: empty"}},</p>
  </p>    {"external_transform",</p>
  </p>     {OT_VECTORVECTOR,</p>
  </p>      "List of external_transform instruction arguments. Default: empty"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/oracle_function.cpp: const Options OracleFunction</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>  {{"expand",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Replace MX with SX expressions in problem formulation [false]"}},</p>
  </p>  {"monitor",</p>
  </p>    {OT_STRINGVECTOR,</p>
  </p>    "Set of user problem functions to be monitored"}},</p>
  </p>  {"show_eval_warnings",</p>
  </p>    {OT_BOOL,</p>
  </p>    "Show warnings generated from function evaluations [true]"}},</p>
  </p>  {"common_options",</p>
  </p>    {OT_DICT,</p>
  </p>    "Options for auto-generated functions"}},</p>
  </p>  {"specific_options",</p>
  </p>    {OT_DICT,</p>
  </p>    "Options for specific auto-generated functions,"</p>
  </p>    " overwriting the defaults from common_options. Nested dictionary."}}</p>
  </p>}</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/importer_internal.cpp: const Options ImporterInternal</summary>
  </p>= {{},</p>
  </p>   {{"verbose",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Verbose evaluation -- for debugging"}}</p>
  </p>    }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/interpolant.cpp: const Options Interpolant</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>   {{"lookup_mode",</p>
  </p>     {OT_STRINGVECTOR,</p>
  </p>      "Specifies, for each grid dimension, the lookup algorithm used to find the correct index. "</p>
  </p>      "'linear' uses a for-loop + break; (default when #knots<=100), "</p>
  </p>      "'exact' uses floored division (only for uniform grids), "</p>
  </p>      "'binary' uses a binary search. (default when #knots>100)."}},</p>
  </p>    {"inline",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Implement the lookup table in MX primitives. "</p>
  </p>      "Useful when you need derivatives with respect to grid and/or coefficients. "</p>
  </p>      "Such derivatives are fundamentally dense, so use with caution."}},</p>
  </p>    {"batch_x",</p>
  </p>     {OT_INT,</p>
  </p>      "Evaluate a batch of different inputs at once (default 1)."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/nlpsol.cpp: const Options Nlpsol</summary>
  </p>= {{&OracleFunction::options_},</p>
  </p>   {{"iteration_callback",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "A function that will be called at each iteration with the solver as input. "</p>
  </p>      "Check documentation of Callback."}},</p>
  </p>    {"iteration_callback_step",</p>
  </p>     {OT_INT,</p>
  </p>      "Only call the callback function every few iterations."}},</p>
  </p>    {"iteration_callback_ignore_errors",</p>
  </p>     {OT_BOOL,</p>
  </p>      "If set to true, errors thrown by iteration_callback will be ignored."}},</p>
  </p>    {"ignore_check_vec",</p>
  </p>     {OT_BOOL,</p>
  </p>      "If set to true, the input shape of F will not be checked."}},</p>
  </p>    {"warn_initial_bounds",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Warn if the initial guess does not satisfy LBX and UBX"}},</p>
  </p>    {"eval_errors_fatal",</p>
  </p>     {OT_BOOL,</p>
  </p>      "When errors occur during evaluation of f,g,...,"</p>
  </p>      "stop the iterations"}},</p>
  </p>    {"verbose_init",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Print out timing information about "</p>
  </p>      "the different stages of initialization"}},</p>
  </p>    {"discrete",</p>
  </p>     {OT_BOOLVECTOR,</p>
  </p>      "Indicates which of the variables are discrete, i.e. integer-valued"}},</p>
  </p>    {"calc_multipliers",</p>
  </p>    {OT_BOOL,</p>
  </p>     "Calculate Lagrange multipliers in the Nlpsol base class"}},</p>
  </p>    {"calc_lam_x",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Calculate 'lam_x' in the Nlpsol base class"}},</p>
  </p>    {"calc_lam_p",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Calculate 'lam_p' in the Nlpsol base class"}},</p>
  </p>    {"calc_f",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Calculate 'f' in the Nlpsol base class"}},</p>
  </p>    {"calc_g",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Calculate 'g' in the Nlpsol base class"}},</p>
  </p>    {"no_nlp_grad",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Prevent the creation of the 'nlp_grad' function"}},</p>
  </p>    {"bound_consistency",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Ensure that primal-dual solution is consistent with the bounds"}},</p>
  </p>    {"min_lam",</p>
  </p>     {OT_DOUBLE,</p>
  </p>      "Minimum allowed multiplier value"}},</p>
  </p>    {"oracle_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Options to be passed to the oracle function"}},</p>
  </p>    {"sens_linsol",</p>
  </p>     {OT_STRING,</p>
  </p>      "Linear solver used for parametric sensitivities (default 'qr')."}},</p>
  </p>    {"sens_linsol_options",</p>
  </p>     {OT_DICT,</p>
  </p>      "Linear solver options used for parametric sensitivities."}},</p>
  </p>    {"detect_simple_bounds",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Automatically detect simple bounds (lbx/ubx) (default false). "</p>
  </p>      "This is hopefully beneficial to speed and robustness but may also have adverse affects: "</p>
  </p>      "1) Subtleties in heuristics and stopping criteria may change the solution, "</p>
  </p>      "2) IPOPT may lie about multipliers of simple equality bounds unless "</p>
  </p>      "'fixed_variable_treatment' is set to 'relax_bounds'."}},</p>
  </p>    {"detect_simple_bounds_is_simple",</p>
  </p>     {OT_BOOLVECTOR,</p>
  </p>      "For internal use only."}},</p>
  </p>    {"detect_simple_bounds_parts",</p>
  </p>     {OT_FUNCTION,</p>
  </p>      "For internal use only."}},</p>
  </p>    {"detect_simple_bounds_target_x",</p>
  </p>     {OT_INTVECTOR,</p>
  </p>      "For internal use only."}}</p>
  </p>   }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/fmu_function.cpp: const Options FmuFunction</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p> {{"scheme_in",</p>
  </p>   {OT_STRINGVECTOR,</p>
  </p>    "Names of the inputs in the scheme"}},</p>
  </p>  {"scheme_out",</p>
  </p>   {OT_STRINGVECTOR,</p>
  </p>    "Names of the outputs in the scheme"}},</p>
  </p>  {"scheme",</p>
  </p>   {OT_DICT,</p>
  </p>    "Definitions of the scheme variables"}},</p>
  </p>  {"aux",</p>
  </p>   {OT_STRINGVECTOR,</p>
  </p>    "Auxilliary variables"}},</p>
  </p>  {"enable_ad",</p>
  </p>   {OT_BOOL,</p>
  </p>    "Calculate first order derivatives using FMU directional derivative support"}},</p>
  </p>  {"validate_ad",</p>
  </p>   {OT_BOOL,</p>
  </p>    "Compare analytic derivatives with finite differences for validation"}},</p>
  </p>  {"validate_ad_file",</p>
  </p>   {OT_STRING,</p>
  </p>    "Redirect results of Hessian validation to a file instead of generating a warning"}},</p>
  </p>  {"check_hessian",</p>
  </p>   {OT_BOOL,</p>
  </p>    "Symmetry check for Hessian"}},</p>
  </p>  {"make_symmetric",</p>
  </p>   {OT_BOOL,</p>
  </p>    "Ensure Hessian is symmetric"}},</p>
  </p>  {"step",</p>
  </p>   {OT_DOUBLE,</p>
  </p>    "Step size, scaled by nominal value"}},</p>
  </p>  {"abstol",</p>
  </p>   {OT_DOUBLE,</p>
  </p>    "Absolute error tolerance, scaled by nominal value"}},</p>
  </p>  {"reltol",</p>
  </p>   {OT_DOUBLE,</p>
  </p>    "Relative error tolerance"}},</p>
  </p>  {"parallelization",</p>
  </p>   {OT_STRING,</p>
  </p>    "Parallelization [SERIAL|openmp|thread]"}},</p>
  </p>  {"print_progress",</p>
  </p>   {OT_BOOL,</p>
  </p>    "Print progress during Jacobian/Hessian evaluation"}},</p>
  </p>  {"new_jacobian",</p>
  </p>   {OT_BOOL,</p>
  </p>    "Use Jacobian implementation in class"}},</p>
  </p>  {"new_hessian",</p>
  </p>   {OT_BOOL,</p>
  </p>    "Use Hessian implementation in class"}},</p>
  </p>  {"hessian_coloring",</p>
  </p>   {OT_BOOL,</p>
  </p>    "Enable the use of graph coloring (star coloring) for Hessian calculation. "</p>
  </p>    "Note that disabling the coloring can improve symmetry check diagnostics."}}</p>
  </p> }</p>
  </p>  };</p>
</details>

<details>
 <summary>./casadi/casadi/core/dple.cpp: const Options Dple</summary>
  </p>= {{&FunctionInternal::options_},</p>
  </p>   {{"const_dim",</p>
  </p>     {OT_BOOL,</p>
  </p>      "Assume constant dimension of P"}},</p>
  </p>    {"pos_def",</p>
  </p>      {OT_BOOL,</p>
  </p>       "Assume P positive definite"}},</p>
  </p>    {"error_unstable",</p>
  </p>      {OT_BOOL,</p>
  </p>      "Throw an exception when it is detected that Product(A_i, i=N..1)"</p>
  </p>      "has eigenvalues greater than 1-eps_unstable"}},</p>
  </p>    {"eps_unstable",</p>
  </p>      {OT_DOUBLE,</p>
  </p>      "A margin for unstability detection"}}</p>
  </p>   }</p>
  </p>  };</p>
</details>
