# MA24_simulink

./casadi/casadi/solvers/linsol_ldl.cpp: const Options LinsolLdl</summary>
  = {{&ProtoFunction::options_},
     {{"incomplete",
      {OT_BOOL,
       "Incomplete factorization, without any fill-in"}},
      {"preordering",
       {OT_BOOL,
       "Approximate minimal degree (AMD) preordering"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/linear_interpolant.cpp: const Options LinearInterpolant</summary>
  = {{&Interpolant::options_},
     {{"lookup_mode",
       {OT_STRINGVECTOR,
        "Sets, for each grid dimenion, the lookup algorithm used to find the correct index. "
        "'linear' uses a for-loop + break; "
        "'exact' uses floored division (only for uniform grids)."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/ipqp.cpp: const Options Ipqp</summary>
  = {{&Conic::options_},
     {{"max_iter",
       {OT_INT,
        "Maximum number of iterations [1000]."}},
      {"constr_viol_tol",
       {OT_DOUBLE,
        "Constraint violation tolerance [1e-8]."}},
      {"dual_inf_tol",
       {OT_DOUBLE,
        "Dual feasibility violation tolerance [1e-8]"}},
      {"print_header",
       {OT_BOOL,
        "Print header [true]."}},
      {"print_iter",
       {OT_BOOL,
        "Print iterations [true]."}},
      {"print_info",
       {OT_BOOL,
        "Print info [true]."}},
      {"linear_solver",
       {OT_STRING,
        "A custom linear solver creator function [default: ldl]"}},
      {"linear_solver_options",
       {OT_DICT,
        "Options to be passed to the linear solver"}},
      {"min_lam",
       {OT_DOUBLE,
        "Smallest multiplier treated as inactive for the initial active set [0]."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/qrsqp.cpp: const Options Qrsqp</summary>
  = {{&Nlpsol::options_},
     {{"qpsol",
       {OT_STRING,
        "The QP solver to be used by the SQP method [qrqp]"}},
      {"qpsol_options",
       {OT_DICT,
        "Options to be passed to the QP solver"}},
      {"hessian_approximation",
       {OT_STRING,
        "limited-memory|exact"}},
      {"max_iter",
       {OT_INT,
        "Maximum number of SQP iterations"}},
      {"min_iter",
       {OT_INT,
        "Minimum number of SQP iterations"}},
      {"max_iter_ls",
       {OT_INT,
        "Maximum number of linesearch iterations"}},
      {"tol_pr",
       {OT_DOUBLE,
        "Stopping criterion for primal infeasibility"}},
      {"tol_du",
       {OT_DOUBLE,
        "Stopping criterion for dual infeasability"}},
      {"c1",
       {OT_DOUBLE,
        "Armijo condition, coefficient of decrease in merit"}},
      {"beta",
       {OT_DOUBLE,
        "Line-search parameter, restoration factor of stepsize"}},
      {"merit_memory",
       {OT_INT,
        "Size of memory to store history of merit function values"}},
      {"lbfgs_memory",
       {OT_INT,
        "Size of L-BFGS memory."}},
      {"regularize",
       {OT_BOOL,
        "Automatic regularization of Lagrange Hessian."}},
      {"print_header",
       {OT_BOOL,
        "Print the header with problem statistics"}},
      {"print_iteration",
       {OT_BOOL,
        "Print the iterations"}},
      {"min_step_size",
       {OT_DOUBLE,
        "The size (inf-norm) of the step size should not become smaller than this."}},
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/scpgen.cpp: const Options Scpgen</summary>
  = {{&Nlpsol::options_},
     {{"qpsol",
       {OT_STRING,
        "The QP solver to be used by the SQP method"}},
      {"qpsol_options",
       {OT_DICT,
        "Options to be passed to the QP solver"}},
      {"hessian_approximation",
       {OT_STRING,
        "gauss-newton|exact"}},
      {"max_iter",
       {OT_INT,
        "Maximum number of SQP iterations"}},
      {"max_iter_ls",
       {OT_INT,
        "Maximum number of linesearch iterations"}},
      {"tol_pr",
       {OT_DOUBLE,
        "Stopping criterion for primal infeasibility"}},
      {"tol_du",
       {OT_DOUBLE,
        "Stopping criterion for dual infeasability"}},
      {"tol_reg",
       {OT_DOUBLE,
        "Stopping criterion for regularization"}},
      {"tol_pr_step",
       {OT_DOUBLE,
        "Stopping criterion for the step size"}},
      {"c1",
       {OT_DOUBLE,
        "Armijo condition, coefficient of decrease in merit"}},
      {"beta",
       {OT_DOUBLE,
        "Line-search parameter, restoration factor of stepsize"}},
      {"merit_memsize",
       {OT_INT,
        "Size of memory to store history of merit function values"}},
      {"merit_start",
       {OT_DOUBLE,
        "Lower bound for the merit function parameter"}},
      {"lbfgs_memory",
       {OT_INT,
        "Size of L-BFGS memory."}},
      {"regularize",
       {OT_BOOL,
        "Automatic regularization of Lagrange Hessian."}},
      {"print_header",
       {OT_BOOL,
        "Print the header with problem statistics"}},
      {"codegen",
       {OT_BOOL,
        "C-code generation"}},
      {"reg_threshold",
       {OT_DOUBLE,
        "Threshold for the regularization."}},
      {"name_x",
       {OT_STRINGVECTOR,
        "Names of the variables."}},
      {"print_x",
       {OT_INTVECTOR,
        "Which variables to print."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/linsol_qr.cpp: const Options LinsolQr</summary>
  = {{&LinsolInternal::options_},
     {{"eps",
       {OT_DOUBLE,
        "Minimum R entry before singularity is declared [1e-12]"}},
      {"cache",
       {OT_DOUBLE,
        "Amount of factorisations to remember (thread-local) [0]"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/newton.cpp: const Options Newton</summary>
  = {{&Rootfinder::options_},
     {{"abstol",
       {OT_DOUBLE,
        "Stopping criterion tolerance on max(|F|)"}},
      {"abstolStep",
       {OT_DOUBLE,
        "Stopping criterion tolerance on step size"}},
      {"max_iter",
       {OT_INT,
        "Maximum number of Newton iterations to perform before returning."}},
      {"print_iteration",
       {OT_BOOL,
        "Print information about each iteration"}},
      {"line_search",
       {OT_BOOL,
        "Enable line-search (default: true)"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/sqpmethod.cpp: const Options Sqpmethod</summary>
  = {{&Nlpsol::options_},
    {{"qpsol",
      {OT_STRING,
      "The QP solver to be used by the SQP method [qpoases]"}},
    {"qpsol_options",
      {OT_DICT,
      "Options to be passed to the QP solver"}},
    {"hessian_approximation",
      {OT_STRING,
      "limited-memory|exact"}},
    {"max_iter",
      {OT_INT,
      "Maximum number of SQP iterations"}},
    {"min_iter",
      {OT_INT,
      "Minimum number of SQP iterations"}},
    {"max_iter_ls",
      {OT_INT,
      "Maximum number of linesearch iterations"}},
    {"tol_pr",
      {OT_DOUBLE,
      "Stopping criterion for primal infeasibility"}},
    {"tol_du",
      {OT_DOUBLE,
      "Stopping criterion for dual infeasability"}},
    {"c1",
      {OT_DOUBLE,
      "Armijo condition, coefficient of decrease in merit"}},
    {"beta",
      {OT_DOUBLE,
      "Line-search parameter, restoration factor of stepsize"}},
    {"merit_memory",
      {OT_INT,
      "Size of memory to store history of merit function values"}},
    {"lbfgs_memory",
      {OT_INT,
      "Size of L-BFGS memory."}},
    {"print_header",
      {OT_BOOL,
      "Print the header with problem statistics"}},
    {"print_iteration",
      {OT_BOOL,
      "Print the iterations"}},
    {"print_status",
      {OT_BOOL,
      "Print a status message after solving"}},
    {"min_step_size",
      {OT_DOUBLE,
      "The size (inf-norm) of the step size should not become smaller than this."}},
    {"hess_lag",
      {OT_FUNCTION,
      "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},
    {"jac_fg",
      {OT_FUNCTION,
      "Function for calculating the gradient of the objective and Jacobian of the constraints "
      "(autogenerated by default)"}},
    {"convexify_strategy",
      {OT_STRING,
      "NONE|regularize|eigen-reflect|eigen-clip. "
      "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},
    {"convexify_margin",
      {OT_DOUBLE,
      "When using a convexification strategy, make sure that "
      "the smallest eigenvalue is at least this (default: 1e-7)."}},
    {"max_iter_eig",
      {OT_DOUBLE,
      "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},
    {"elastic_mode",
      {OT_BOOL,
      "Enable the elastic mode which is used when the QP is infeasible (default: false)."}},
    {"gamma_0",
      {OT_DOUBLE,
      "Starting value for the penalty parameter of elastic mode (default: 1)."}},
    {"gamma_max",
      {OT_DOUBLE,
      "Maximum value for the penalty parameter of elastic mode (default: 1e20)."}},
    {"gamma_1_min",
      {OT_DOUBLE,
      "Minimum value for gamma_1 (default: 1e-5)."}},
    {"second_order_corrections",
      {OT_BOOL,
      "Enable second order corrections. "
      "These are used when a step is considered bad by the merit function and constraint norm "
      "(default: false)."}},
    {"init_feasible",
      {OT_BOOL,
      "Initialize the QP subproblems with a feasible initial value (default: false)."}}
    }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/fast_newton.cpp: const Options FastNewton</summary>
  = {{&Rootfinder::options_},
     {{"abstol",
       {OT_DOUBLE,
        "Stopping criterion tolerance on ||g||__inf)"}},
      {"abstolStep",
       {OT_DOUBLE,
        "Stopping criterion tolerance on step size"}},
      {"max_iter",
       {OT_INT,
        "Maximum number of Newton iterations to perform before returning."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/shell_compiler.cpp: const Options ShellCompiler</summary>
  = {{&ImporterInternal::options_},
     {{"compiler",
       {OT_STRING,
        "Compiler command"}},
      {"linker",
       {OT_STRING,
        "Linker command"}},
      {"directory",
       {OT_STRING,
        "Directory to put temporary objects in. Must end with a file separator."}},
      {"compiler_setup",
       {OT_STRING,
        "Compiler setup command. Intended to be fixed."
        " The 'flag' option is the prefered way to set"
        " custom flags."}},
      {"linker_setup",
       {OT_STRING,
        "Linker setup command. Intended to be fixed."
        " The 'flag' option is the prefered way to set"
        " custom flags."}},
      {"compiler_flags",
       {OT_STRINGVECTOR,
        "Alias for 'compiler_flags'"}},
      {"flags",
        {OT_STRINGVECTOR,
        "Compile flags for the JIT compiler. Default: None"}},
      {"linker_flags",
       {OT_STRINGVECTOR,
        "Linker flags for the JIT compiler. Default: None"}},
      {"cleanup",
       {OT_BOOL,
        "Cleanup temporary files when unloading. Default: true"}},
      {"compiler_output_flag",
       {OT_STRING,
       "Compiler flag to denote object output. Default: '-o '"}},
      {"linker_output_flag",
       {OT_STRING,
       "Linker flag to denote shared library output. Default: '-o '"}},
      {"extra_suffixes",
       {OT_STRINGVECTOR,
       "List of suffixes for extra files that the compiler may generate. Default: None"}},
      {"name",
       {OT_STRING,
        "The file name used to write out compiled objects/libraries. "
        "The actual file names used depend on 'temp_suffix' and include extensions. "
        "Default: 'tmp_casadi_compiler_shell'"}},
      {"temp_suffix",
       {OT_BOOL,
        "Use a temporary (seemingly random) filename suffix for file names. "
        "This is desired for thread-safety. "
        "This behaviour may defeat caching compiler wrappers. "
        "Default: true"}},
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/implicit_to_nlp.cpp: const Options ImplicitToNlp</summary>
  = {{&Rootfinder::options_},
     {{"nlpsol",
       {OT_STRING,
        "Name of solver."}},
      {"nlpsol_options",
       {OT_DICT,
        "Options to be passed to solver."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/qrqp.cpp: const Options Qrqp</summary>
  = {{&Conic::options_},
     {{"max_iter",
       {OT_INT,
        "Maximum number of iterations [1000]."}},
      {"constr_viol_tol",
       {OT_DOUBLE,
        "Constraint violation tolerance [1e-8]."}},
      {"dual_inf_tol",
       {OT_DOUBLE,
        "Dual feasibility violation tolerance [1e-8]"}},
      {"print_header",
       {OT_BOOL,
        "Print header [true]."}},
      {"print_iter",
       {OT_BOOL,
        "Print iterations [true]."}},
      {"print_info",
       {OT_BOOL,
        "Print info [true]."}},
      {"print_lincomb",
       {OT_BOOL,
        "Print dependant linear combinations of constraints [false]. "
        "Printed numbers are 0-based indices into the vector of [simple bounds;linear bounds]"}},
      {"min_lam",
       {OT_DOUBLE,
        "Smallest multiplier treated as inactive for the initial active set [0]."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/feasiblesqpmethod.cpp: const Options Feasiblesqpmethod</summary>
  = {{&Nlpsol::options_},
     {{"solve_type",
       {OT_STRING,
        "The solver type: Either SQP or SLP. Defaults to SQP"}},
      {"qpsol",
       {OT_STRING,
        "The QP solver to be used by the SQP method [qpoases]"}},
      {"qpsol_options",
       {OT_DICT,
        "Options to be passed to the QP solver"}},
      {"hessian_approximation",
       {OT_STRING,
        "limited-memory|exact"}},
      {"max_iter",
       {OT_INT,
        "Maximum number of SQP iterations"}},
      {"min_iter",
       {OT_INT,
        "Minimum number of SQP iterations"}},
      {"tol_pr",
       {OT_DOUBLE,
        "Stopping criterion for primal infeasibility"}},
      {"tol_du",
       {OT_DOUBLE,
        "Stopping criterion for dual infeasability"}},
      {"merit_memory",
       {OT_INT,
        "Size of memory to store history of merit function values"}},
      {"lbfgs_memory",
       {OT_INT,
        "Size of L-BFGS memory."}},
      {"print_header",
       {OT_BOOL,
        "Print the header with problem statistics"}},
      {"print_iteration",
       {OT_BOOL,
        "Print the iterations"}},
      {"print_status",
       {OT_BOOL,
        "Print a status message after solving"}},
      {"f",
       {OT_FUNCTION,
        "Function for calculating the objective function (autogenerated by default)"}},
      {"g",
       {OT_FUNCTION,
        "Function for calculating the constraints (autogenerated by default)"}},
      {"grad_f",
       {OT_FUNCTION,
        "Function for calculating the gradient of the objective (autogenerated by default)"}},
      {"jac_g",
       {OT_FUNCTION,
        "Function for calculating the Jacobian of the constraints (autogenerated by default)"}},
      {"hess_lag",
       {OT_FUNCTION,
        "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},
      {"convexify_strategy",
       {OT_STRING,
        "NONE|regularize|eigen-reflect|eigen-clip. "
        "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},
      {"convexify_margin",
       {OT_DOUBLE,
        "When using a convexification strategy, make sure that "
        "the smallest eigenvalue4 is at least this (default: 1e-7)."}},
      {"max_iter_eig",
       {OT_DOUBLE,
        "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},
      {"init_feasible",
       {OT_BOOL,
        "Initialize the QP subproblems with a feasible initial value (default: false)."}},
      {"optim_tol",
       {OT_DOUBLE,
        "Optimality tolerance. Below this value an iterate is considered to be optimal."}},
      {"feas_tol",
       {OT_DOUBLE,
        "Feasibility tolerance. Below this tolerance an iterate is considered to be feasible."}},
      {"tr_rad0",
       {OT_DOUBLE,
        "Initial trust-region radius."}},
      {"tr_eta1",
       {OT_DOUBLE,
        "Lower eta in trust-region acceptance criterion."}},
      {"tr_eta2",
       {OT_DOUBLE,
        "Upper eta in trust-region acceptance criterion."}},
      {"tr_alpha1",
       {OT_DOUBLE,
        "Lower alpha in trust-region size criterion."}},
      {"tr_alpha2",
       {OT_DOUBLE,
        "Upper alpha in trust-region size criterion."}},
      {"tr_tol",
       {OT_DOUBLE,
        "Trust-region tolerance. "
        "Below this value another scalar is equal to the trust region radius."}},
      {"tr_acceptance",
       {OT_DOUBLE,
        "Is the trust-region ratio above this value, the step is accepted."}},
      {"tr_rad_min",
       {OT_DOUBLE,
        "Minimum trust-region radius."}},
      {"tr_rad_max",
       {OT_DOUBLE,
        "Maximum trust-region radius."}},
      {"tr_scale_vector",
       {OT_DOUBLEVECTOR,
        "Vector that tells where trust-region is applied."}},
      {"contraction_acceptance_value",
       {OT_DOUBLE,
        "If the empirical contraction rate in the feasibility iterations "
        "is above this value in the heuristics the iterations are aborted."}},
      {"watchdog",
       {OT_INT,
        "Number of watchdog iterations in feasibility iterations. "
        "After this amount of iterations, it is checked with the contraction acceptance value, "
        "if iterations are converging."}},
      {"max_inner_iter",
       {OT_DOUBLE,
        "Maximum number of inner iterations."}},
      {"use_anderson",
       {OT_BOOL,
        "Use Anderson Acceleration. (default false)"}},
      {"anderson_memory",
       {OT_INT,
        "Anderson memory. If Anderson is used default is 1, else default is 0."}},
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/collocation.cpp: const Options Collocation</summary>
  = {{&ImplicitFixedStepIntegrator::options_},
     {{"interpolation_order",
       {OT_INT,
        "Order of the interpolating polynomials"}},
      {"collocation_scheme",
       {OT_STRING,
        "Collocation scheme: radau|legendre"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/symbolic_qr.cpp: const Options SymbolicQr</summary>
  = {{&FunctionInternal::options_},
    {{"fopts",
      {OT_DICT,
       "Options to be passed to generated function objects"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/bspline_interpolant.cpp: const Options BSplineInterpolant</summary>
  = {{&Interpolant::options_},
      {{"degree",
       {OT_INTVECTOR,
        "Sets, for each grid dimension, the degree of the spline."}},
       {"linear_solver",
        {OT_STRING,
         "Solver used for constructing the coefficient tensor."}},
       {"linear_solver_options",
        {OT_DICT,
         "Options to be passed to the linear solver."}},
       {"algorithm",
        {OT_STRING,
         "Algorithm used for fitting the data: 'not_a_knot' (default, same as Matlab),"
        " 'smooth_linear'."}},
       {"smooth_linear_frac",
        {OT_DOUBLE,
         "When 'smooth_linear' algorithm is active, determines sharpness between"
         " 0 (sharp, as linear interpolation) and 0.5 (smooth)."
         "Default value is 0.1."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/solvers/qp_to_nlp.cpp: const Options QpToNlp</summary>
  = {{&Conic::options_},
     {{"nlpsol",
       {OT_STRING,
        "Name of solver."}},
      {"nlpsol_options",
       {OT_DICT,
        "Options to be passed to solver."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/hpipm/hpipm_interface.cpp: const Options HpipmInterface</summary>
  = {{&Conic::options_},
     {{"N",
       {OT_INT,
        "OCP horizon"}},
      {"nx",
       {OT_INTVECTOR,
        "Number of states, length N+1"}},
      {"nu",
       {OT_INTVECTOR,
        "Number of controls, length N"}},
      {"ng",
       {OT_INTVECTOR,
        "Number of non-dynamic constraints, length N+1"}},
      {"inf",
       {OT_DOUBLE,
        "Replace infinities by this amount [default: 1e8]"}},
      {"hpipm",
       {OT_DICT,
        "Options to be passed to hpipm"}}}
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/lapack/lapack_lu.cpp: const Options LapackLu</summary>
  = {{&FunctionInternal::options_},
     {{"equilibration",
       {OT_BOOL,
        "Equilibrate the matrix"}},
      {"allow_equilibration_failure",
       {OT_BOOL,
        "Non-fatal error when equilibration fails"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/lapack/lapack_qr.cpp: const Options LapackQr</summary>
  = {{&FunctionInternal::options_},
     {{"max_nrhs",
       {OT_INT,
        "Maximum number of right-hand-sides that get processed in a single pass [default:10]."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/highs/highs_interface.cpp: const Options HighsInterface</summary>
  = {{&Conic::options_},
     {{"highs",
       {OT_DICT,
        "Options to be passed to HiGHS."
        }},
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/alpaqa/alpaqa_interface.cpp: const Options AlpaqaInterface</summary>
  = {{&Nlpsol::options_},
      {{"alpaqa",
        {OT_DICT,
        "Options to be passed to Alpaqa"}}
      }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/mumps/mumps_interface.cpp: const Options MumpsInterface</summary>
  = {{&ProtoFunction::options_},
     {{"symmetric",
      {OT_BOOL,
       "Symmetric matrix"}},
      {"posdef",
       {OT_BOOL,
       "Positive definite"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/daqp/daqp_interface.cpp: const Options DaqpInterface</summary>
  = {{&Conic::options_},
     {{"daqp",
       {OT_DICT,
        "Options to be passed to Daqp."
        }},
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/worhp/worhp_interface.cpp: const Options WorhpInterface</summary>
  = {{&Nlpsol::options_},
     {{"worhp",
       {OT_DICT,
        "Options to be passed to WORHP"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/sundials/cvodes_interface.cpp: const Options CvodesInterface</summary>
  = {{&SundialsInterface::options_},
    {{"linear_multistep_method",
      {OT_STRING,
      "Integrator scheme: BDF|adams"}},
    {"nonlinear_solver_iteration",
      {OT_STRING,
      "Nonlinear solver type: NEWTON|functional"}},
    {"min_step_size",
      {OT_DOUBLE,
      "Min step size [default: 0/0.0]"}},
    {"fsens_all_at_once",
      {OT_BOOL,
      "Calculate all right hand sides of the sensitivity equations at once"}},
    {"always_recalculate_jacobian",
     {OT_BOOL,
      "Recalculate Jacobian before factorizations, even if Jacobian is current [default: true]"}}
    }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/sundials/kinsol_interface.cpp: const Options KinsolInterface</summary>
  = {{&Rootfinder::options_},
     {{"max_iter",
       {OT_INT,
        "Maximum number of Newton iterations. Putting 0 sets the default value of KinSol."}},
      {"print_level",
       {OT_INT,
        "Verbosity level"}},
      {"abstol",
       {OT_DOUBLE,
        "Stopping criterion tolerance"}},
      {"linear_solver_type",
       {OT_STRING,
        "dense|banded|iterative|user_defined"}},
      {"upper_bandwidth",
       {OT_INT,
        "Upper bandwidth for banded linear solvers"}},
      {"lower_bandwidth",
       {OT_INT,
        "Lower bandwidth for banded linear solvers"}},
      {"max_krylov",
       {OT_INT,
        "Maximum Krylov space dimension"}},
      {"exact_jacobian",
       {OT_BOOL,
        "Use exact Jacobian information"}},
      {"iterative_solver",
       {OT_STRING,
        "gmres|bcgstab|tfqmr"}},
      {"f_scale",
       {OT_DOUBLEVECTOR,
        "Equation scaling factors"}},
      {"u_scale",
       {OT_DOUBLEVECTOR,
        "Variable scaling factors"}},
      {"pretype",
       {OT_STRING,
        "Type of preconditioner"}},
      {"use_preconditioner",
       {OT_BOOL,
        "Precondition an iterative solver"}},
      {"strategy",
       {OT_STRING,
        "Globalization strategy"}},
      {"disable_internal_warnings",
       {OT_BOOL,
        "Disable KINSOL internal warning messages"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/sundials/idas_interface.cpp: const Options IdasInterface</summary>
  = {{&SundialsInterface::options_},
    {{"suppress_algebraic",
      {OT_BOOL,
      "Suppress algebraic variables in the error testing"}},
    {"calc_ic",
      {OT_BOOL,
      "Use IDACalcIC to get consistent initial conditions."}},
    {"constraints",
      {OT_INTVECTOR,
      "Constrain the solution y=[x,z]. 0 (default): no constraint on yi, "
      "1: yi >= 0.0, -1: yi <= 0.0, 2: yi > 0.0, -2: yi < 0.0."}},
    {"calc_icB",
      {OT_BOOL,
      "Use IDACalcIC to get consistent initial conditions for "
      "backwards system [default: equal to calc_ic]."}},
    {"abstolv",
      {OT_DOUBLEVECTOR,
      "Absolute tolerarance for each component"}},
    {"max_step_size",
      {OT_DOUBLE,
      "Maximim step size"}},
    {"first_time",
      {OT_DOUBLE,
      "First requested time as a fraction of the time interval"}},
    {"cj_scaling",
      {OT_BOOL,
      "IDAS scaling on cj for the user-defined linear solver module"}},
    {"init_xdot",
      {OT_DOUBLEVECTOR,
      "Initial values for the state derivatives"}}
    }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/sundials/sundials_interface.cpp: const Options SundialsInterface</summary>
  = {{&Integrator::options_},
    {{"max_num_steps",
      {OT_INT,
      "Maximum number of integrator steps"}},
    {"reltol",
      {OT_DOUBLE,
      "Relative tolerence for the IVP solution"}},
    {"abstol",
      {OT_DOUBLE,
      "Absolute tolerence for the IVP solution"}},
    {"newton_scheme",
      {OT_STRING,
      "Linear solver scheme in the Newton method: DIRECT|gmres|bcgstab|tfqmr"}},
    {"max_krylov",
      {OT_INT,
      "Maximum Krylov subspace size"}},
    {"sensitivity_method",
      {OT_STRING,
      "Sensitivity method: SIMULTANEOUS|staggered"}},
    {"max_multistep_order",
      {OT_INT,
      "Maximum order for the (variable-order) multistep method"}},
    {"use_preconditioner",
      {OT_BOOL,
      "Precondition the iterative solver [default: true]"}},
    {"stop_at_end",
      {OT_BOOL,
      "[DEPRECATED] Stop the integrator at the end of the interval"}},
    {"disable_internal_warnings",
      {OT_BOOL,
      "Disable SUNDIALS internal warning messages"}},
    {"quad_err_con",
      {OT_BOOL,
      "Should the quadratures affect the step size control"}},
    {"fsens_err_con",
      {OT_BOOL,
      "include the forward sensitivities in all error controls"}},
    {"steps_per_checkpoint",
      {OT_INT,
      "Number of steps between two consecutive checkpoints"}},
    {"interpolation_type",
      {OT_STRING,
      "Type of interpolation for the adjoint sensitivities"}},
    {"linear_solver",
      {OT_STRING,
      "A custom linear solver creator function [default: qr]"}},
    {"linear_solver_options",
      {OT_DICT,
      "Options to be passed to the linear solver"}},
    {"second_order_correction",
      {OT_BOOL,
      "Second order correction in the augmented system Jacobian [true]"}},
    {"step0",
      {OT_DOUBLE,
      "initial step size [default: 0/estimated]"}},
    {"max_step_size",
      {OT_DOUBLE,
      "Max step size [default: 0/inf]"}},
    {"max_order",
      {OT_DOUBLE,
      "Maximum order"}},
    {"nonlin_conv_coeff",
      {OT_DOUBLE,
      "Coefficient in the nonlinear convergence test"}},
    {"scale_abstol",
     {OT_BOOL,
      "Scale absolute tolerance by nominal value"}}
    }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/gurobi/gurobi_interface.cpp: const Options GurobiInterface</summary>
  = {{&Conic::options_},
     {{"vtype",
       {OT_STRINGVECTOR,
        "Type of variables: [CONTINUOUS|binary|integer|semicont|semiint]"}},
      {"gurobi",
       {OT_DICT,
        "Options to be passed to gurobi."}},
      {"sos_groups",
       {OT_INTVECTORVECTOR,
        "Definition of SOS groups by indices."}},
      {"sos_weights",
       {OT_DOUBLEVECTORVECTOR,
        "Weights corresponding to SOS entries."}},
      {"sos_types",
       {OT_INTVECTOR,
        "Specify 1 or 2 for each SOS group."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/cplex/cplex_interface.cpp: const Options CplexInterface</summary>
  = {{&Conic::options_},
     {{"cplex",
       {OT_DICT,
        "Options to be passed to CPLEX"}},
      {"qp_method",
       {OT_INT,
        "Determines which CPLEX algorithm to use."}},
      {"dump_to_file",
       {OT_BOOL,
        "Dumps QP to file in CPLEX format."}},
      {"dump_filename",
       {OT_STRING,
        "The filename to dump to."}},
      {"tol",
       {OT_DOUBLE,
        "Tolerance of solver"}},
      {"dep_check",
       {OT_INT,
        "Detect redundant constraints."}},
      {"warm_start",
       {OT_BOOL,
        "Use warm start with simplex methods (affects only the simplex methods)."}},
      {"mip_start",
       {OT_BOOL,
        "Hot start integers with x0 [Default false]."}},
      {"sos_groups",
       {OT_INTVECTORVECTOR,
        "Definition of SOS groups by indices."}},
      {"sos_weights",
       {OT_DOUBLEVECTORVECTOR,
        "Weights corresponding to SOS entries."}},
      {"sos_types",
       {OT_INTVECTOR,
        "Specify 1 or 2 for each SOS group."}},
      {"version_suffix",
       {OT_STRING,
        "Specify version of cplex to load. "
        "We will attempt to load libcplex<version_suffix>.[so|dll|dylib]. "
        "Default value is taken from CPLEX_VERSION env variable."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/ampl/ampl_interface.cpp: const Options AmplInterface</summary>
  = {{&Nlpsol::options_},
     {{"solver",
       {OT_STRING,
        "AMPL solver binary"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/fatrop/fatrop_conic_interface.cpp: const Options FatropConicInterface</summary>
  = {{&Conic::options_},
     {{"N",
       {OT_INT,
        "OCP horizon"}},
      {"nx",
       {OT_INTVECTOR,
        "Number of states, length N+1"}},
      {"nu",
       {OT_INTVECTOR,
        "Number of controls, length N"}},
      {"ng",
       {OT_INTVECTOR,
        "Number of non-dynamic constraints, length N+1"}},
      {"fatrop",
       {OT_DICT,
        "Options to be passed to fatrop"}}}
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/qpoases/qpoases_interface.cpp: const Options QpoasesInterface</summary>
  = {{&Conic::options_},
     {{"sparse",
       {OT_BOOL,
        "Formulate the QP using sparse matrices. [false]"}},
      {"schur",
       {OT_BOOL,
        "Use Schur Complement Approach [false]"}},
      {"hessian_type",
       {OT_STRING,
        "Type of Hessian - see qpOASES documentation "
        "[UNKNOWN|posdef|semidef|indef|zero|identity]]"}},
      {"max_schur",
       {OT_INT,
        "Maximal number of Schur updates [75]"}},
      {"linsol_plugin",
       {OT_STRING,
        "Linear solver plugin"}},
      {"nWSR",
       {OT_INT,
        "The maximum number of working set recalculations to be performed during "
        "the initial homotopy. Default is 5(nx + nc)"}},
      {"CPUtime",
       {OT_DOUBLE,
        "The maximum allowed CPU time in seconds for the whole initialisation"
        " (and the actually required one on output). Disabled if unset."}},
      {"printLevel",
       {OT_STRING,
        "Defines the amount of text output during QP solution, see Section 5.7"}},
      {"enableRamping",
       {OT_BOOL,
        "Enables ramping."}},
      {"enableFarBounds",
       {OT_BOOL,
        "Enables the use of  far bounds."}},
      {"enableFlippingBounds",
       {OT_BOOL,
        "Enables the use of  flipping bounds."}},
      {"enableRegularisation",
       {OT_BOOL,
        "Enables automatic  Hessian regularisation."}},
      {"enableFullLITests",
       {OT_BOOL,
        "Enables condition-hardened  (but more expensive) LI test."}},
      {"enableNZCTests",
       {OT_BOOL,
        "Enables nonzero curvature  tests."}},
      {"enableDriftCorrection",
       {OT_INT,
        "Specifies the frequency of drift corrections: 0: turns them off."}},
      {"enableCholeskyRefactorisation",
       {OT_INT,
        "Specifies the frequency of a full re-factorisation of projected "
        "Hessian matrix: 0: turns them off,  1: uses them at each iteration etc."}},
      {"enableEqualities",
       {OT_BOOL,
        "Specifies whether equalities should be treated  as always active "
        "(True) or not (False)"}},
      {"terminationTolerance",
       {OT_DOUBLE,
        "Relative termination tolerance to stop homotopy."}},
      {"boundTolerance",
       {OT_DOUBLE,
        "If upper and lower bounds differ less than this tolerance, they are regarded "
        "equal, i.e. as  equality constraint."}},
      {"boundRelaxation",
       {OT_DOUBLE,
        "Initial relaxation of bounds to start homotopy  and initial value for far bounds."}},
      {"epsNum",
       {OT_DOUBLE,
        "Numerator tolerance for ratio tests."}},
      {"epsDen",
       {OT_DOUBLE,
        "Denominator tolerance for ratio tests."}},
      {"maxPrimalJump",
       {OT_DOUBLE,
        "Maximum allowed jump in primal variables in  nonzero curvature tests."}},
      {"maxDualJump",
       {OT_DOUBLE,
        "Maximum allowed jump in dual variables in  linear independence tests."}},
      {"initialRamping",
       {OT_DOUBLE,
        "Start value for ramping strategy."}},
      {"finalRamping",
       {OT_DOUBLE,
        "Final value for ramping strategy."}},
      {"initialFarBounds",
       {OT_DOUBLE,
        "Initial size for far bounds."}},
      {"growFarBounds",
       {OT_DOUBLE,
        "Factor to grow far bounds."}},
      {"initialStatusBounds",
       {OT_STRING,
        "Initial status of bounds at first iteration."}},
      {"epsFlipping",
       {OT_DOUBLE,
        "Tolerance of squared Cholesky diagonal factor  which triggers flipping bound."}},
      {"numRegularisationSteps",
       {OT_INT,
        "Maximum number of successive regularisation steps."}},
      {"epsRegularisation",
       {OT_DOUBLE,
        "Scaling factor of identity matrix used for  Hessian regularisation."}},
      {"numRefinementSteps",
       {OT_INT,
        "Maximum number of iterative refinement steps."}},
      {"epsIterRef",
       {OT_DOUBLE,
        "Early termination tolerance for iterative  refinement."}},
      {"epsLITests",
       {OT_DOUBLE,
        "Tolerance for linear independence tests."}},
      {"epsNZCTests",
       {OT_DOUBLE,
        "Tolerance for nonzero curvature tests."}},
      {"enableInertiaCorrection",
       {OT_BOOL,
        "Should working set be repaired when negative curvature is discovered during hotstart."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/clp/clp_interface.cpp: const Options ClpInterface</summary>
  = {{&Conic::options_},
     {{"clp",
       {OT_DICT,
        "Options to be passed to CLP. "
        "A first set of options can be found in ClpParameters.hpp. eg. 'PrimalTolerance'. "
        "There are other options in additions. "
        "'AutomaticScaling' (bool) is recognised. "
        "'initial_solve' (default off) activates the use of Clp's initialSolve. "
        "'initial_solve_options' takes a dictionary with following keys (see ClpSolve.hpp): "
        " SolveType (string), PresolveType (string), "
        " NumberPasses, SpecialOptions (intvectorvector), IndependentOptions (intvectorvector)."
        }}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/snopt/snopt_interface.cpp: const Options SnoptInterface</summary>
  = {{&Nlpsol::options_},
     {{"snopt",
       {OT_DICT,
        "Options to be passed to SNOPT"}},
      {"start",
       {OT_STRING,
        "Warm-start options for Worhp: cold|warm|hot"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/knitro/knitro_interface.cpp: const Options KnitroInterface</summary>
  = {{&Nlpsol::options_},
     {{"knitro",
       {OT_DICT,
        "Options to be passed to KNITRO"}},
      {"options_file",
       {OT_STRING,
        "Read options from file (solver specific)"}},
      {"detect_linear_constraints",
       {OT_BOOL,
        "Detect type of constraints"}},
      {"contype",
       {OT_INTVECTOR,
        "Type of constraint"}},
      {"complem_variables",
       {OT_INTVECTORVECTOR,
        "List of complementary constraints on simple bounds. "
        "Pair (i, j) encodes complementarity between the bounds on variable i and variable j."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/ooqp/ooqp_interface.cpp: const Options OoqpInterface</summary>
  = {{&Conic::options_},
     {{"print_level",
       {OT_INT,
        "Print level. OOQP listens to print_level 0, 10 and 100"}},
      {"mutol",
       {OT_DOUBLE,
        "tolerance as provided with setMuTol to OOQP"}},
      {"artol",
       {OT_DOUBLE,
        "tolerance as provided with setArTol to OOQP"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/sleqp/sleqp_interface.cpp: const Options SLEQPInterface</summary>
  = {{&Nlpsol::options_},
      {{"sleqp",
        {OT_DICT,
        "Options to be passed to SLEQP"}},
       {"print_level",
        {OT_INT,
        "Print level of SLEQP (default: 2/SLEQP_LOG_WARN)"}},
       {"max_iter",
        {OT_INT,
        "Maximum number of iterations"}},
       {"max_wall_time",
        {OT_DOUBLE,
        "maximum wall time allowed"}}
      }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/superscs/superscs_interface.cpp: const Options SuperscsInterface</summary>
  = {{&Conic::options_},
     {{"superscs",
       {OT_DICT,
        "Options to be passed to superscs."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/hpmpc/hpmpc_interface.cpp: const Options HpmpcInterface</summary>
  = {{&Conic::options_},
     {{"N",
       {OT_INT,
        "OCP horizon"}},
      {"nx",
       {OT_INTVECTOR,
        "Number of states, length N+1"}},
      {"nu",
       {OT_INTVECTOR,
        "Number of controls, length N"}},
      {"ng",
       {OT_INTVECTOR,
        "Number of non-dynamic constraints, length N+1"}},
      {"mu0",
       {OT_DOUBLE,
        "Max element in cost function as estimate of max multiplier"}},
      {"max_iter",
       {OT_INT,
        "Max number of iterations"}},
      {"tol",
       {OT_DOUBLE,
        "Tolerance in the duality measure"}},
      {"warm_start",
       {OT_BOOL,
        "Use warm-starting"}},
      {"inf",
       {OT_DOUBLE,
        "HPMPC cannot handle infinities. Infinities will be replaced by this option's value."}},
      {"print_level",
       {OT_INT,
        "Amount of diagnostic printing [Default: 1]."}},
      {"target",
       {OT_STRING,
        "hpmpc target"}},
      {"blasfeo_target",
       {OT_STRING,
        "hpmpc target"}}}
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/slicot/slicot_dple.cpp: const Options SlicotDple</summary>
  = {{&Dple::options_},
     {{"linear_solver",
       {OT_STRING,
        "User-defined linear solver class. Needed for sensitivities."}},
      {"linear_solver_options",
        {OT_DICT,
         "Options to be passed to the linear solver."}},
      {"psd_num_zero",
        {OT_DOUBLE,
          "Numerical zero used in Periodic Schur decomposition with slicot."
          "This option is needed when your systems has Floquet multipliers"
          "zero or close to zero"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/blocksqp/blocksqp.cpp: const Options Blocksqp</summary>
  = {{&Nlpsol::options_},
     {{"qpsol",
       {OT_STRING,
        "The QP solver to be used by the SQP method"}},
      {"qpsol_options",
       {OT_DICT,
        "Options to be passed to the QP solver"}},
      {"linsol",
       {OT_STRING,
        "The linear solver to be used by the QP method"}},
      {"print_header",
       {OT_BOOL,
        "Print solver header at startup"}},
      {"print_iteration",
       {OT_BOOL,
        "Print SQP iterations"}},
      {"eps",
       {OT_DOUBLE,
        "Values smaller than this are regarded as numerically zero"}},
      {"opttol",
       {OT_DOUBLE,
        "Optimality tolerance"}},
      {"nlinfeastol",
       {OT_DOUBLE,
        "Nonlinear feasibility tolerance"}},
      {"schur",
       {OT_BOOL,
        "Use qpOASES Schur compliment approach"}},
      {"globalization",
       {OT_BOOL,
        "Enable globalization"}},
      {"restore_feas",
       {OT_BOOL,
        "Use feasibility restoration phase"}},
      {"max_line_search",
       {OT_INT,
        "Maximum number of steps in line search"}},
      {"max_consec_reduced_steps",
       {OT_INT,
        "Maximum number of consecutive reduced steps"}},
      {"max_consec_skipped_updates",
       {OT_INT,
        "Maximum number of consecutive skipped updates"}},
      {"max_iter",
       {OT_INT,
        "Maximum number of SQP iterations"}},
      {"warmstart",
       {OT_BOOL,
        "Use warmstarting"}},
      {"qp_init",
       {OT_BOOL,
        "Use warmstarting"}},
      {"max_it_qp",
       {OT_INT,
        "Maximum number of QP iterations per SQP iteration"}},
      {"block_hess",
       {OT_INT,
        "Blockwise Hessian approximation?"}},
      {"hess_scaling",
       {OT_INT,
        "Scaling strategy for Hessian approximation"}},
      {"fallback_scaling",
       {OT_INT,
        "If indefinite update is used, the type of fallback strategy"}},
      {"max_time_qp",
       {OT_DOUBLE,
        "Maximum number of time in seconds per QP solve per SQP iteration"}},
      {"ini_hess_diag",
       {OT_DOUBLE,
        "Initial Hessian guess: diagonal matrix diag(iniHessDiag)"}},
      {"col_eps",
       {OT_DOUBLE,
        "Epsilon for COL scaling strategy"}},
      {"col_tau1",
       {OT_DOUBLE,
        "tau1 for COL scaling strategy"}},
      {"col_tau2",
       {OT_DOUBLE,
        "tau2 for COL scaling strategy"}},
      {"hess_damp",
       {OT_INT,
        "Activate Powell damping for BFGS"}},
      {"hess_damp_fac",
       {OT_DOUBLE,
        "Damping factor for BFGS Powell modification"}},
      {"hess_update",
       {OT_INT,
        "Type of Hessian approximation"}},
      {"fallback_update",
       {OT_INT,
        "If indefinite update is used, the type of fallback strategy"}},
      {"hess_lim_mem",
       {OT_INT,
        "Full or limited memory"}},
      {"hess_memsize",
       {OT_INT,
        "Memory size for L-BFGS updates"}},
      {"which_second_derv",
       {OT_INT,
        "For which block should second derivatives be provided by the user"}},
      {"skip_first_globalization",
       {OT_BOOL,
        "No globalization strategy in first iteration"}},
      {"conv_strategy",
       {OT_INT,
        "Convexification strategy"}},
      {"max_conv_qp",
       {OT_INT,
        "How many additional QPs may be solved for convexification per iteration?"}},
      {"max_soc_iter",
       {OT_INT,
        "Maximum number of SOC line search iterations"}},
      {"gamma_theta",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"gamma_f",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"kappa_soc",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"kappa_f",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"theta_max",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"theta_min",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"delta",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"s_theta",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"s_f",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"kappa_minus",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"kappa_plus",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"kappa_plus_max",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"delta_h0",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"eta",
       {OT_DOUBLE,
        "Filter line search parameter, cf. IPOPT paper"}},
      {"obj_lo",
       {OT_DOUBLE,
        "Lower bound on objective function [-inf]"}},
      {"obj_up",
       {OT_DOUBLE,
        "Upper bound on objective function [inf]"}},
      {"rho",
       {OT_DOUBLE,
        "Feasibility restoration phase parameter"}},
      {"zeta",
       {OT_DOUBLE,
        "Feasibility restoration phase parameter"}},
      {"print_maxit_reached",
       {OT_BOOL,
        "Print error when maximum number of SQP iterations reached"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/ipopt/ipopt_interface.cpp: const Options IpoptInterface</summary>
  = {{&Nlpsol::options_},
     {{"pass_nonlinear_variables",
       {OT_BOOL,
        "Pass list of variables entering nonlinearly to IPOPT"}},
      {"ipopt",
       {OT_DICT,
        "Options to be passed to IPOPT"}},
      {"var_string_md",
       {OT_DICT,
        "String metadata (a dictionary with lists of strings) "
        "about variables to be passed to IPOPT"}},
      {"var_integer_md",
       {OT_DICT,
        "Integer metadata (a dictionary with lists of integers) "
        "about variables to be passed to IPOPT"}},
      {"var_numeric_md",
       {OT_DICT,
        "Numeric metadata (a dictionary with lists of reals) about "
        "variables to be passed to IPOPT"}},
      {"con_string_md",
       {OT_DICT,
        "String metadata (a dictionary with lists of strings) about "
        "constraints to be passed to IPOPT"}},
      {"con_integer_md",
       {OT_DICT,
        "Integer metadata (a dictionary with lists of integers) "
        "about constraints to be passed to IPOPT"}},
      {"con_numeric_md",
       {OT_DICT,
        "Numeric metadata (a dictionary with lists of reals) about "
        "constraints to be passed to IPOPT"}},
      {"hess_lag",
       {OT_FUNCTION,
        "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},
      {"jac_g",
       {OT_FUNCTION,
        "Function for calculating the Jacobian of the constraints "
        "(autogenerated by default)"}},
      {"grad_f",
       {OT_FUNCTION,
        "Function for calculating the gradient of the objective "
        "(column, autogenerated by default)"}},
      {"convexify_strategy",
       {OT_STRING,
        "NONE|regularize|eigen-reflect|eigen-clip. "
        "Strategy to convexify the Lagrange Hessian before passing it to the solver."}},
      {"convexify_margin",
       {OT_DOUBLE,
        "When using a convexification strategy, make sure that "
        "the smallest eigenvalue is at least this (default: 1e-7)."}},
      {"max_iter_eig",
       {OT_DOUBLE,
        "Maximum number of iterations to compute an eigenvalue decomposition (default: 50)."}},
      {"clip_inactive_lam",
       {OT_BOOL,
        "Explicitly set Lagrange multipliers to 0 when bound is deemed inactive "
        "(default: false)."}},
      {"inactive_lam_strategy",
       {OT_STRING,
        "Strategy to detect if a bound is inactive. "
        "RELTOL: use solver-defined constraint tolerance * inactive_lam_value|"
        "abstol: use inactive_lam_value"}},
      {"inactive_lam_value",
       {OT_DOUBLE,
        "Value used in inactive_lam_strategy (default: 10)."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/clang/clang_compiler.cpp: const Options ClangCompiler</summary>
  = {{&ImporterInternal::options_},
     {{"include_path",
       {OT_STRING,
        "Include paths for the JIT compiler. "
        "The include directory shipped with CasADi will be automatically appended."}},
      {"flags",
       {OT_STRINGVECTOR,
        "Compile flags for the JIT compiler. Default: None"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/proxqp/proxqp_interface.cpp: const Options ProxqpInterface</summary>
  = {{&Conic::options_},
     {{"proxqp",
       {OT_DICT,
        "const proxqp options."}},
      {"warm_start_primal",
       {OT_BOOL,
        "Use x input to warmstart [Default: true]."}},
      {"warm_start_dual",
       {OT_BOOL,
        "Use y and z input to warmstart [Default: true]."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/bonmin/bonmin_interface.cpp: const Options BonminInterface</summary>
  = {{&Nlpsol::options_},
     {{"pass_nonlinear_variables",
       {OT_BOOL,
        "Pass list of variables entering nonlinearly to BONMIN"}},
      {"pass_nonlinear_constraints",
       {OT_BOOL,
        "Pass list of constraints entering nonlinearly to BONMIN"}},
      {"bonmin",
       {OT_DICT,
        "Options to be passed to BONMIN"}},
      {"var_string_md",
       {OT_DICT,
        "String metadata (a dictionary with lists of strings) "
        "about variables to be passed to BONMIN"}},
      {"var_integer_md",
       {OT_DICT,
        "Integer metadata (a dictionary with lists of integers) "
        "about variables to be passed to BONMIN"}},
      {"var_numeric_md",
       {OT_DICT,
        "Numeric metadata (a dictionary with lists of reals) about "
        "variables to be passed to BONMIN"}},
      {"con_string_md",
       {OT_DICT,
        "String metadata (a dictionary with lists of strings) about "
        "constraints to be passed to BONMIN"}},
      {"con_integer_md",
       {OT_DICT,
        "Integer metadata (a dictionary with lists of integers) "
        "about constraints to be passed to BONMIN"}},
      {"con_numeric_md",
       {OT_DICT,
        "Numeric metadata (a dictionary with lists of reals) about "
        "constraints to be passed to BONMIN"}},
      {"hess_lag",
       {OT_FUNCTION,
        "Function for calculating the Hessian of the Lagrangian (autogenerated by default)"}},
      {"hess_lag_options",
       {OT_DICT,
        "Options for the autogenerated Hessian of the Lagrangian."}},
      {"jac_g",
       {OT_FUNCTION,
        "Function for calculating the Jacobian of the constraints "
        "(autogenerated by default)"}},
      {"jac_g_options",
       {OT_DICT,
        "Options for the autogenerated Jacobian of the constraints."}},
      {"grad_f",
       {OT_FUNCTION,
        "Function for calculating the gradient of the objective "
        "(column, autogenerated by default)"}},
      {"grad_f_options",
       {OT_DICT,
        "Options for the autogenerated gradient of the objective."}},
      {"sos1_groups",
       {OT_INTVECTORVECTOR,
        "Options for the autogenerated gradient of the objective."}},
      {"sos1_weights",
       {OT_DOUBLEVECTORVECTOR,
        "Options for the autogenerated gradient of the objective."}},
      {"sos1_priorities",
       {OT_INTVECTOR,
        "Options for the autogenerated gradient of the objective."}},
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/cbc/cbc_interface.cpp: const Options CbcInterface</summary>
  = {{&Conic::options_},
     {{"cbc",
       {OT_DICT,
        "Options to be passed to CBC."
        "Three sets of options are supported. "
        "The first can be found in OsiSolverParameters.hpp. "
        "The second can be found in CbcModel.hpp. "
        "The third are options that can be passed to CbcMain1."
        }},
      {"sos_groups",
       {OT_INTVECTORVECTOR,
        "Definition of SOS groups by indices."}},
      {"sos_weights",
       {OT_DOUBLEVECTORVECTOR,
        "Weights corresponding to SOS entries."}},
      {"sos_types",
       {OT_INTVECTOR,
        "Specify 1 or 2 for each SOS group."}},
      {"hot_start",
       {OT_BOOL,
        "Hot start with x0 [Default false]."}},
     }
    };
</details>

<details>
  <summary>./casadi/casadi/interfaces/osqp/osqp_interface.cpp: const Options OsqpInterface</summary>
  = {{&Conic::options_},
     {{"osqp",
       {OT_DICT,
        "const Options to be passed to osqp."}},
      {"warm_start_primal",
       {OT_BOOL,
        "Use x0 input to warmstart [Default: true]."}},
      {"warm_start_dual",
       {OT_BOOL,
        "Use lam_a0 and lam_x0 input to warmstart [Default: truw]."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/core/sx_function.cpp: const Options SXFunction</summary>
  = {{&FunctionInternal::options_},
     {{"default_in",
       {OT_DOUBLEVECTOR,
        "Default input values"}},
      {"just_in_time_sparsity",
       {OT_BOOL,
        "Propagate sparsity patterns using just-in-time "
        "compilation to a CPU or GPU using OpenCL"}},
      {"just_in_time_opencl",
       {OT_BOOL,
        "Just-in-time compilation for numeric evaluation using OpenCL (experimental)"}},
      {"live_variables",
       {OT_BOOL,
        "Reuse variables in the work vector"}},
      {"cse",
       {OT_BOOL,
        "Perform common subexpression elimination (complexity is N*log(N) in graph size)"}},
      {"allow_free",
       {OT_BOOL,
        "Allow construction with free variables (Default: false)"}},
      {"allow_duplicate_io_names",
       {OT_BOOL,
        "Allow construction with duplicate io names (Default: false)"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/core/integrator.cpp: const Options Integrator</summary>
  = {{&OracleFunction::options_},
    {{"expand",
      {OT_BOOL,
      "Replace MX with SX expressions in problem formulation [false]"}},
    {"print_stats",
      {OT_BOOL,
      "Print out statistics after integration"}},
    {"nfwd",
     {OT_INT,
      "Number of forward sensitivities to be calculated [0]"}},
    {"nadj",
     {OT_INT,
      "Number of adjoint sensitivities to be calculated [0]"}},
    {"t0",
      {OT_DOUBLE,
      "[DEPRECATED] Beginning of the time horizon"}},
    {"tf",
      {OT_DOUBLE,
      "[DEPRECATED] End of the time horizon"}},
    {"grid",
      {OT_DOUBLEVECTOR,
      "[DEPRECATED] Time grid"}},
    {"augmented_options",
      {OT_DICT,
      "Options to be passed down to the augmented integrator, if one is constructed."}},
    {"output_t0",
      {OT_BOOL,
      "[DEPRECATED] Output the state at the initial time"}}
    }
    };
</details>

<details>
  <summary>./casadi/casadi/core/integrator.cpp: const Options FixedStepIntegrator</summary>
  = {{&Integrator::options_},
    {{"number_of_finite_elements",
      {OT_INT,
      "Target number of finite elements. "
      "The actual number may be higher to accommodate all output times"}},
    {"simplify",
      {OT_BOOL,
      "Implement as MX Function (codegeneratable/serializable) default: false"}},
    {"simplify_options",
      {OT_DICT,
      "Any options to pass to simplified form Function constructor"}}
    }
    };
</details>

<details>
  <summary>./casadi/casadi/core/integrator.cpp: const Options ImplicitFixedStepIntegrator</summary>
  = {{&FixedStepIntegrator::options_},
    {{"rootfinder",
      {OT_STRING,
      "An implicit function solver"}},
    {"rootfinder_options",
      {OT_DICT,
      "Options to be passed to the NLP Solver"}}
    }
    };
</details>

<details>
  <summary>./casadi/casadi/core/conic.cpp: const Options Conic</summary>
  = {{&FunctionInternal::options_},
     {{"discrete",
       {OT_BOOLVECTOR,
        "Indicates which of the variables are discrete, i.e. integer-valued"}},
      {"print_problem",
       {OT_BOOL,
        "Print a numeric description of the problem"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/core/rootfinder.cpp: const Options Rootfinder</summary>
  = {{&OracleFunction::options_},
     {{"linear_solver",
       {OT_STRING,
        "User-defined linear solver class. Needed for sensitivities."}},
      {"linear_solver_options",
       {OT_DICT,
        "Options to be passed to the linear solver."}},
      {"constraints",
       {OT_INTVECTOR,
        "Constrain the unknowns. 0 (default): no constraint on ui, "
        "1: ui >= 0.0, -1: ui <= 0.0, 2: ui > 0.0, -2: ui < 0.0."}},
      {"implicit_input",
       {OT_INT,
        "Index of the input that corresponds to the actual root-finding"}},
      {"implicit_output",
       {OT_INT,
        "Index of the output that corresponds to the actual root-finding"}},
      {"jacobian_function",
       {OT_FUNCTION,
        "Function object for calculating the Jacobian (autogenerated by default)"}},
     }
    };
</details>

<details>
  <summary>./casadi/casadi/core/mx_function.cpp: const Options MXFunction</summary>
  = {{&FunctionInternal::options_},
     {{"default_in",
       {OT_DOUBLEVECTOR,
        "Default input values"}},
      {"live_variables",
       {OT_BOOL,
        "Reuse variables in the work vector"}},
      {"print_instructions",
       {OT_BOOL,
        "Print each operation during evaluation"}},
      {"cse",
       {OT_BOOL,
        "Perform common subexpression elimination (complexity is N*log(N) in graph size)"}},
      {"allow_free",
       {OT_BOOL,
        "Allow construction with free variables (Default: false)"}},
      {"allow_duplicate_io_names",
       {OT_BOOL,
        "Allow construction with duplicate io names (Default: false)"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/core/expm.cpp: const Options Expm</summary>
  = {{&FunctionInternal::options_},
     {{"const_A",
       {OT_BOOL,
        "Assume A is constant. Default: false."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/core/finite_differences.cpp: const Options FiniteDiff</summary>
  = {{&FunctionInternal::options_},
    {{"second_order_stepsize",
      {OT_DOUBLE,
      "Second order perturbation size [default: 1e-3]"}},
    {"h",
      {OT_DOUBLE,
      "Step size [default: computed from abstol]"}},
    {"h_max",
      {OT_DOUBLE,
      "Maximum step size [default 0]"}},
    {"h_min",
      {OT_DOUBLE,
      "Minimum step size [default inf]"}},
    {"smoothing",
      {OT_DOUBLE,
      "Smoothing regularization [default: machine precision]"}},
    {"reltol",
      {OT_DOUBLE,
      "Accuracy of function inputs [default: query object]"}},
    {"abstol",
      {OT_DOUBLE,
      "Accuracy of function outputs [default: query object]"}},
    {"u_aim",
      {OT_DOUBLE,
      "Target ratio of roundoff error to truncation error [default: 100.]"}},
    {"h_iter",
      {OT_INT,
      "Number of iterations to improve on the step-size "
      "[default: 1 if error estimate available, otherwise 0]"}},
    }
    };
</details>

<details>
  <summary>./casadi/casadi/core/jit_function.cpp: const Options JitFunction</summary>
  = {{&FunctionInternal::options_},
     {{"buffered",
      {OT_BOOL,
        "Buffer the calls, user does not need to "}},
       {"jac",
      {OT_STRING,
        "Function body for Jacobian"}},
      {"hess",
       {OT_STRING,
        "Function body for Hessian"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/core/function_internal.cpp: const Options ProtoFunction</summary>
  = {{},
     {{"verbose",
       {OT_BOOL,
        "Verbose evaluation -- for debugging"}},
      {"print_time",
       {OT_BOOL,
        "print information about execution time. Implies record_time."}},
      {"record_time",
       {OT_BOOL,
        "record information about execution time, for retrieval with stats()."}},
      {"regularity_check",
       {OT_BOOL,
        "Throw exceptions when NaN or Inf appears during evaluation"}},
      {"error_on_fail",
       {OT_BOOL,
        "Throw exceptions when function evaluation fails (default true)."}}
      }
    };
</details>

<details>
  <summary>./casadi/casadi/core/function_internal.cpp: const Options FunctionInternal</summary>
  = {{&ProtoFunction::options_},
      {{"ad_weight",
       {OT_DOUBLE,
        "Weighting factor for derivative calculation."
        "When there is an option of either using forward or reverse mode "
        "directional derivatives, the condition ad_weight*nf<=(1-ad_weight)*na "
        "is used where nf and na are estimates of the number of forward/reverse "
        "mode directional derivatives needed. By default, ad_weight is calculated "
        "automatically, but this can be overridden by setting this option. "
        "In particular, 0 means forcing forward mode and 1 forcing reverse mode. "
        "Leave unset for (class specific) heuristics."}},
      {"ad_weight_sp",
       {OT_DOUBLE,
        "Weighting factor for sparsity pattern calculation calculation."
        "Overrides default behavior. Set to 0 and 1 to force forward and "
        "reverse mode respectively. Cf. option \"ad_weight\". "
        "When set to -1, sparsity is completely ignored and dense matrices are used."}},
      {"always_inline",
       {OT_BOOL,
        "Force inlining."}},
      {"never_inline",
       {OT_BOOL,
        "Forbid inlining."}},
      {"jac_penalty",
       {OT_DOUBLE,
        "When requested for a number of forward/reverse directions,   "
        "it may be cheaper to compute first the full jacobian and then "
        "multiply with seeds, rather than obtain the requested directions "
        "in a straightforward manner. "
        "Casadi uses a heuristic to decide which is cheaper. "
        "A high value of 'jac_penalty' makes it less likely for the heurstic "
        "to chose the full Jacobian strategy. "
        "The special value -1 indicates never to use the full Jacobian strategy"}},
      {"user_data",
       {OT_VOIDPTR,
        "A user-defined field that can be used to identify "
        "the function or pass additional information"}},
      {"inputs_check",
       {OT_BOOL,
        "Throw exceptions when the numerical values of the inputs don't make sense"}},
      {"gather_stats",
       {OT_BOOL,
        "Deprecated option (ignored): Statistics are now always collected."}},
      {"input_scheme",
       {OT_STRINGVECTOR,
        "Deprecated option (ignored)"}},
      {"output_scheme",
       {OT_STRINGVECTOR,
        "Deprecated option (ignored)"}},
      {"jit",
       {OT_BOOL,
        "Use just-in-time compiler to speed up the evaluation"}},
      {"jit_cleanup",
       {OT_BOOL,
        "Cleanup up the temporary source file that jit creates. Default: true"}},
      {"jit_serialize",
       {OT_STRING,
        "Specify behaviour when serializing a jitted function: SOURCE|link|embed."}},
      {"jit_name",
       {OT_STRING,
        "The file name used to write out code. "
        "The actual file names used depend on 'jit_temp_suffix' and include extensions. "
        "Default: 'jit_tmp'"}},
      {"jit_temp_suffix",
       {OT_BOOL,
        "Use a temporary (seemingly random) filename suffix for generated code and libraries. "
        "This is desired for thread-safety. "
        "This behaviour may defeat caching compiler wrappers. "
        "Default: true"}},
      {"compiler",
       {OT_STRING,
        "Just-in-time compiler plugin to be used."}},
      {"jit_options",
       {OT_DICT,
        "Options to be passed to the jit compiler."}},
      {"derivative_of",
       {OT_FUNCTION,
        "The function is a derivative of another function. "
        "The type of derivative (directional derivative, Jacobian) "
        "is inferred from the function name."}},
      {"max_num_dir",
       {OT_INT,
        "Specify the maximum number of directions for derivative functions."
        " Overrules the builtin optimized_num_dir."}},
      {"enable_forward",
       {OT_BOOL,
        "Enable derivative calculation using generated functions for"
        " Jacobian-times-vector products - typically using forward mode AD"
        " - if available. [default: true]"}},
      {"enable_reverse",
        {OT_BOOL,
        "Enable derivative calculation using generated functions for"
        " transposed Jacobian-times-vector products - typically using reverse mode AD"
        " - if available. [default: true]"}},
      {"enable_jacobian",
        {OT_BOOL,
        "Enable derivative calculation using generated functions for"
        " Jacobians of all differentiable outputs with respect to all differentiable inputs"
        " - if available. [default: true]"}},
      {"enable_fd",
       {OT_BOOL,
        "Enable derivative calculation by finite differencing. [default: false]]"}},
      {"fd_options",
       {OT_DICT,
        "Options to be passed to the finite difference instance"}},
      {"fd_method",
       {OT_STRING,
        "Method for finite differencing [default 'central']"}},
      {"print_in",
       {OT_BOOL,
        "Print numerical values of inputs [default: false]"}},
      {"print_out",
       {OT_BOOL,
        "Print numerical values of outputs [default: false]"}},
      {"max_io",
       {OT_INT,
        "Acceptable number of inputs and outputs. Warn if exceeded."}},
      {"dump_in",
       {OT_BOOL,
        "Dump numerical values of inputs to file (readable with DM.from_file) [default: false]"}},
      {"dump_out",
       {OT_BOOL,
        "Dump numerical values of outputs to file (readable with DM.from_file) [default: false]"}},
      {"dump",
       {OT_BOOL,
        "Dump function to file upon first evaluation. [false]"}},
      {"dump_dir",
       {OT_STRING,
        "Directory to dump inputs/outputs to. Make sure the directory exists [.]"}},
      {"dump_format",
       {OT_STRING,
        "Choose file format to dump matrices. See DM.from_file [mtx]"}},
      {"forward_options",
       {OT_DICT,
        "Options to be passed to a forward mode constructor"}},
      {"reverse_options",
       {OT_DICT,
        "Options to be passed to a reverse mode constructor"}},
      {"jacobian_options",
       {OT_DICT,
        "Options to be passed to a Jacobian constructor"}},
      {"der_options",
       {OT_DICT,
        "Default options to be used to populate forward_options, reverse_options, and "
        "jacobian_options before those options are merged in."}},
      {"custom_jacobian",
       {OT_FUNCTION,
        "Override CasADi's AD. Use together with 'jac_penalty': 0. "
        "Note: Highly experimental. Syntax may break often."}},
      {"is_diff_in",
       {OT_BOOLVECTOR,
        "Indicate for each input if it should be differentiable."}},
      {"is_diff_out",
       {OT_BOOLVECTOR,
        "Indicate for each output if it should be differentiable."}},
      {"post_expand",
       {OT_BOOL,
        "After construction, expand this Function. Default: False"}},
      {"post_expand_options",
       {OT_DICT,
        "Options to be passed to post-construction expansion. Default: empty"}},
      {"cache",
       {OT_DICT,
        "Prepopulate the function cache. Default: empty"}},
      {"external_transform",
       {OT_VECTORVECTOR,
        "List of external_transform instruction arguments. Default: empty"}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/core/oracle_function.cpp: const Options OracleFunction</summary>
  = {{&FunctionInternal::options_},
    {{"expand",
      {OT_BOOL,
      "Replace MX with SX expressions in problem formulation [false]"}},
    {"monitor",
      {OT_STRINGVECTOR,
      "Set of user problem functions to be monitored"}},
    {"show_eval_warnings",
      {OT_BOOL,
      "Show warnings generated from function evaluations [true]"}},
    {"common_options",
      {OT_DICT,
      "Options for auto-generated functions"}},
    {"specific_options",
      {OT_DICT,
      "Options for specific auto-generated functions,"
      " overwriting the defaults from common_options. Nested dictionary."}}
  }
    };
</details>

<details>
  <summary>./casadi/casadi/core/importer_internal.cpp: const Options ImporterInternal</summary>
  = {{},
     {{"verbose",
       {OT_BOOL,
        "Verbose evaluation -- for debugging"}}
      }
    };
</details>

<details>
  <summary>./casadi/casadi/core/interpolant.cpp: const Options Interpolant</summary>
  = {{&FunctionInternal::options_},
     {{"lookup_mode",
       {OT_STRINGVECTOR,
        "Specifies, for each grid dimension, the lookup algorithm used to find the correct index. "
        "'linear' uses a for-loop + break; (default when #knots<=100), "
        "'exact' uses floored division (only for uniform grids), "
        "'binary' uses a binary search. (default when #knots>100)."}},
      {"inline",
       {OT_BOOL,
        "Implement the lookup table in MX primitives. "
        "Useful when you need derivatives with respect to grid and/or coefficients. "
        "Such derivatives are fundamentally dense, so use with caution."}},
      {"batch_x",
       {OT_INT,
        "Evaluate a batch of different inputs at once (default 1)."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/core/nlpsol.cpp: const Options Nlpsol</summary>
  = {{&OracleFunction::options_},
     {{"iteration_callback",
       {OT_FUNCTION,
        "A function that will be called at each iteration with the solver as input. "
        "Check documentation of Callback."}},
      {"iteration_callback_step",
       {OT_INT,
        "Only call the callback function every few iterations."}},
      {"iteration_callback_ignore_errors",
       {OT_BOOL,
        "If set to true, errors thrown by iteration_callback will be ignored."}},
      {"ignore_check_vec",
       {OT_BOOL,
        "If set to true, the input shape of F will not be checked."}},
      {"warn_initial_bounds",
       {OT_BOOL,
        "Warn if the initial guess does not satisfy LBX and UBX"}},
      {"eval_errors_fatal",
       {OT_BOOL,
        "When errors occur during evaluation of f,g,...,"
        "stop the iterations"}},
      {"verbose_init",
       {OT_BOOL,
        "Print out timing information about "
        "the different stages of initialization"}},
      {"discrete",
       {OT_BOOLVECTOR,
        "Indicates which of the variables are discrete, i.e. integer-valued"}},
      {"calc_multipliers",
      {OT_BOOL,
       "Calculate Lagrange multipliers in the Nlpsol base class"}},
      {"calc_lam_x",
       {OT_BOOL,
        "Calculate 'lam_x' in the Nlpsol base class"}},
      {"calc_lam_p",
       {OT_BOOL,
        "Calculate 'lam_p' in the Nlpsol base class"}},
      {"calc_f",
       {OT_BOOL,
        "Calculate 'f' in the Nlpsol base class"}},
      {"calc_g",
       {OT_BOOL,
        "Calculate 'g' in the Nlpsol base class"}},
      {"no_nlp_grad",
       {OT_BOOL,
        "Prevent the creation of the 'nlp_grad' function"}},
      {"bound_consistency",
       {OT_BOOL,
        "Ensure that primal-dual solution is consistent with the bounds"}},
      {"min_lam",
       {OT_DOUBLE,
        "Minimum allowed multiplier value"}},
      {"oracle_options",
       {OT_DICT,
        "Options to be passed to the oracle function"}},
      {"sens_linsol",
       {OT_STRING,
        "Linear solver used for parametric sensitivities (default 'qr')."}},
      {"sens_linsol_options",
       {OT_DICT,
        "Linear solver options used for parametric sensitivities."}},
      {"detect_simple_bounds",
       {OT_BOOL,
        "Automatically detect simple bounds (lbx/ubx) (default false). "
        "This is hopefully beneficial to speed and robustness but may also have adverse affects: "
        "1) Subtleties in heuristics and stopping criteria may change the solution, "
        "2) IPOPT may lie about multipliers of simple equality bounds unless "
        "'fixed_variable_treatment' is set to 'relax_bounds'."}},
      {"detect_simple_bounds_is_simple",
       {OT_BOOLVECTOR,
        "For internal use only."}},
      {"detect_simple_bounds_parts",
       {OT_FUNCTION,
        "For internal use only."}},
      {"detect_simple_bounds_target_x",
       {OT_INTVECTOR,
        "For internal use only."}}
     }
    };
</details>

<details>
  <summary>./casadi/casadi/core/fmu_function.cpp: const Options FmuFunction</summary>
  = {{&FunctionInternal::options_},
   {{"scheme_in",
     {OT_STRINGVECTOR,
      "Names of the inputs in the scheme"}},
    {"scheme_out",
     {OT_STRINGVECTOR,
      "Names of the outputs in the scheme"}},
    {"scheme",
     {OT_DICT,
      "Definitions of the scheme variables"}},
    {"aux",
     {OT_STRINGVECTOR,
      "Auxilliary variables"}},
    {"enable_ad",
     {OT_BOOL,
      "Calculate first order derivatives using FMU directional derivative support"}},
    {"validate_ad",
     {OT_BOOL,
      "Compare analytic derivatives with finite differences for validation"}},
    {"validate_ad_file",
     {OT_STRING,
      "Redirect results of Hessian validation to a file instead of generating a warning"}},
    {"check_hessian",
     {OT_BOOL,
      "Symmetry check for Hessian"}},
    {"make_symmetric",
     {OT_BOOL,
      "Ensure Hessian is symmetric"}},
    {"step",
     {OT_DOUBLE,
      "Step size, scaled by nominal value"}},
    {"abstol",
     {OT_DOUBLE,
      "Absolute error tolerance, scaled by nominal value"}},
    {"reltol",
     {OT_DOUBLE,
      "Relative error tolerance"}},
    {"parallelization",
     {OT_STRING,
      "Parallelization [SERIAL|openmp|thread]"}},
    {"print_progress",
     {OT_BOOL,
      "Print progress during Jacobian/Hessian evaluation"}},
    {"new_jacobian",
     {OT_BOOL,
      "Use Jacobian implementation in class"}},
    {"new_hessian",
     {OT_BOOL,
      "Use Hessian implementation in class"}},
    {"hessian_coloring",
     {OT_BOOL,
      "Enable the use of graph coloring (star coloring) for Hessian calculation. "
      "Note that disabling the coloring can improve symmetry check diagnostics."}}
   }
    };
</details>

<details>
  <summary>./casadi/casadi/core/dple.cpp: const Options Dple</summary>
  = {{&FunctionInternal::options_},
     {{"const_dim",
       {OT_BOOL,
        "Assume constant dimension of P"}},
      {"pos_def",
        {OT_BOOL,
         "Assume P positive definite"}},
      {"error_unstable",
        {OT_BOOL,
        "Throw an exception when it is detected that Product(A_i, i=N..1)"
        "has eigenvalues greater than 1-eps_unstable"}},
      {"eps_unstable",
        {OT_DOUBLE,
        "A margin for unstability detection"}}
     }
    };
</details>

<details>
  <summary>
