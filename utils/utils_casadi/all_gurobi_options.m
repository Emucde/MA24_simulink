% Documentation: https://docs.gurobi.com/projects/optimizer/en/current/reference/parameters.html#cloudaccessid

% AggFill
% Presolve aggregation fill level
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% Controls the amount of fill allowed during presolve aggregation. Larger values generally lead to presolved models with fewer rows and columns, but with more constraint matrix non-zeros. The default value chooses automatically, and usually works well.
opts.qpsol_options.gurobi.AggFill = -1; % [Default: -1], [Allowed: {-1, 0, 10}]

% Aggregate
% Presolve aggregation
% Type: int
% Default value: 1
% Minimum value: 0
% Maximum value: 2
% Controls the aggregation level in presolve. The options are off (0), moderate (1), or aggressive (2). In rare instances, aggregation can lead to an accumulation of numerical errors. Turning it off can sometimes improve solution accuracy.
opts.qpsol_options.gurobi.Aggregate = 1; % [Default: 1], [Allowed: {0, 1, 2}]

% BarConvTol
% Barrier convergence tolerance
% Type: double
% Default value: 1e-8
% Minimum value: 0.0
% Maximum value: 1.0
% The barrier solver terminates when the relative difference between the primal and dual objective values is less than the specified tolerance. Tightening this tolerance often produces a more accurate solution, which can sometimes reduce the time spent in crossover.
opts.qpsol_options.gurobi.BarConvTol = 1e-8;  % [Default: 1e-8], [Allowed: {0.0, 1.0}]

% BarCorrectors
% Barrier central corrections
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% Limits the number of central corrections performed in each barrier iteration. The default value chooses automatically, depending on problem characteristics.
opts.qpsol_options.gurobi.BarCorrectors = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]

% BarHomogeneous
% Barrier homogeneous algorithm
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 1
% Determines whether to use the homogeneous barrier algorithm. At the default setting (-1), it is only used when barrier solves a node relaxation for a MIP model.
opts.qpsol_options.gurobi.BarHomogeneous = -1; % [Default: -1], [Allowed: {-1, 0, 1}]

% BarIterLimit
% Barrier iteration limit
% Type: int
% Default value: 1000
% Minimum value: 0
% Maximum value: MAXINT
% Limits the number of barrier iterations performed. This parameter is rarely used. If you would like barrier to terminate early, you should use the BarConvTol parameter instead.
opts.qpsol_options.gurobi.BarIterLimit = 1000; % [Default: 1000], [Allowed: {0, 2000000000}]

% BarOrder
% Barrier ordering algorithm
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 1
% Chooses the barrier sparse matrix fill-reducing algorithm. A value of 0 chooses Approximate Minimum Degree ordering, while a value of 1 chooses Nested Dissection ordering.
opts.qpsol_options.gurobi.BarOrder = -1; % [Default: -1], [Allowed: {-1, 0, 1}]

% BarQCPConvTol
% Barrier convergence tolerance for QCP models
% Type: double
% Default value: 1e-6
% Minimum value: 0.0
% Maximum value: 1.0
% When solving a QCP model, the barrier solver terminates when the relative difference between the primal and dual objective values is less than the specified tolerance.
opts.qpsol_options.gurobi.BarQCPConvTol = 1e-6; % [Default: 1e-6], [Allowed: {0.0, 1.0}]

% BestBdStop
% Objective bound to stop optimization
% Type: double
% Default value: Inf
% Minimum value: -Inf
% Maximum value: Inf
% Terminates as soon as the engine determines that the best bound on the objective value is at least as good as the specified value.
opts.qpsol_options.gurobi.BestBdStop = 2147483648; % [Default: Inf], [Allowed: {-Inf, Inf}]

% BestObjStop
% Objective value to stop optimization
% Type: double
% Default value: -Inf
% Minimum value: -Inf
% Maximum value: Inf
% Terminate as soon as the engine finds a feasible solution whose objective value is at least as good as the specified value.
opts.qpsol_options.gurobi.BestObjStop = -Inf; % [Default: -Inf], [Allowed: {-Inf, Inf}]

% BQPCuts
% BQP cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls Boolean Quadric Polytope (BQP) cut generation. Use 0 to disable these cuts, 1 for moderate cut generation, or 2 for aggressive cut generation.
opts.qpsol_options.gurobi.BQPCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% BranchDir
% Preferred branch direction
% Type: int
% Default value: 0
% Minimum value: -1
% Maximum value: 1
% Determines which child node is explored first in the branch-and-cut search. The default value chooses automatically.
opts.qpsol_options.gurobi.BranchDir = 0; % [Default: 0], [Allowed: {-1, 0, 1}]

% CliqueCuts
% Clique cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls clique cut generation. Use 0 to disable these cuts, 1 for moderate cut generation, or 2 for aggressive cut generation.
opts.qpsol_options.gurobi.CliqueCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% CloudAccessID
% Access ID for Gurobi Instant Cloud
% Type: string
% Default value: ''
% Set this parameter to the Access ID for your Instant Cloud license when launching a new instance.
opts.qpsol_options.gurobi.CloudAccessID = ''; % [Default: '']

% CloudHost
% Host for the Gurobi Cloud entry point
% Type: string
% Default value: ''
% Set this parameter to the host name of the Gurobi Cloud entry point. Currently cloud.gurobi.com.
opts.qpsol_options.gurobi.CloudHost = ''; % [Default: '']

% CloudSecretKey
% Secret Key for Gurobi Instant Cloud
% Type: string
% Default value: ''
% Set this parameter to the Secret Key for your Instant Cloud license when launching a new instance.
opts.qpsol_options.gurobi.CloudSecretKey = ''; % [Default: '']

% CloudPool
% Cloud pool to use for Gurobi Instant Cloud instance
% Type: string
% Default value: ''
% Set this parameter to the name of the cloud pool you would like to use for your new Instant Cloud instance.
opts.qpsol_options.gurobi.CloudPool = ''; % [Default: '']

% ComputeServer
% Name of a node in the Remote Services cluster
% Type: string
% Default value: ''
% Set this parameter to the name of a node in the Remote Services cluster where you’d like your Compute Server job to run.
opts.qpsol_options.gurobi.ComputeServer = ''; % [Default: '']

% ConcurrentJobs
% Distributed concurrent optimizer job count
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: MAXINT
% Enables distributed concurrent optimization
opts.qpsol_options.gurobi.ConcurrentJobs = 0; % [Default: 0], [Allowed: {0, 2000000000}]

% ConcurrentMethod
% Controls the methods used by the concurrent continuous solver
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% This parameter controls which methods are run concurrently by the concurrent solver.
opts.qpsol_options.gurobi.ConcurrentMethod = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% ConcurrentMIP
% Enables the concurrent MIP solver
% Type: int
% Default value: 1
% Minimum value: 1
% Maximum value: 64
% This parameter enables the concurrent MIP solver.
opts.qpsol_options.gurobi.ConcurrentMIP = 1; % [Default: 1], [Allowed: {1, 0}]

% ConcurrentSettings
% Create concurrent environments from a list of .prm files
% Type: string
% Default value: ''
% Command-line only parameter to set parameter files for different instances in concurrent MIP runs.
% opts.qpsol_options.gurobi.ConcurrentSettings = ''; % [Default: '']

% CoverCuts
% Cover cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls cover cut generation.
opts.qpsol_options.gurobi.CoverCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% Crossover
% Barrier crossover strategy
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 4
% Determines the crossover strategy used to transform the interior solution produced by barrier into a basic solution.
opts.qpsol_options.gurobi.Crossover = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3, 4}]

% CrossoverBasis
% Crossover basis construction strategy
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 1
% Determines the initial basis construction strategy for crossover.
opts.qpsol_options.gurobi.CrossoverBasis = -1; % [Default: -1], [Allowed: {-1, 0, 1}]

% CSAPIAccessID
% Access ID for Gurobi Cluster Manager
% Type: string
% Default value: ''
% A unique identifier used to authenticate an application on a Gurobi Cluster Manager.
opts.qpsol_options.gurobi.CSAPIAccessID = ''; % [Default: '']

% CSAPISecret
% Secret key for Gurobi Cluster Manager
% Type: string
% Default value: ''
% The secret password associated with an API access ID.
opts.qpsol_options.gurobi.CSAPISecret = ''; % [Default: '']

% CSAppName
% Application name of the batches or jobs
% Type: string
% Default value: ''
% The application name which will be sent to the server to track which application is submitting the batches or jobs.
opts.qpsol_options.gurobi.CSAppName = ''; % [Default: '']

% CSAuthToken
% JSON Web Token for accessing the Cluster Manager
% Type: string
% Default value: ''
% When a client authenticates with a Cluster Manager using a username and password, a signed token is returned.
opts.qpsol_options.gurobi.CSAuthToken = ''; % [Default: '']

% CSBatchMode
% Controls Batch-Mode optimization
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% When set to 1, enable the local creation of models, and later submit batch-optimization jobs to the Cluster Manager.
opts.qpsol_options.gurobi.CSBatchMode = 0; % [Default: 0], [Allowed: {0, 1}]

% CSClientLog
% Turns logging on or off
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 3
% Turns logging on or off for Compute Server and the Web License Service (WLS).
opts.qpsol_options.gurobi.CSClientLog = 0; % [Default: 0], [Allowed: {0, 1, 2, 3}]

% CSGroup
% Group placement request for cluster
% Type: string
% Default value: ''
% Specifies one or more groups of cluster nodes to control the placement of the job.
opts.qpsol_options.gurobi.CSGroup = ''; % [Default: '']

% CSIdleTimeout
% Idle time before Compute Server kills a job
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% This parameter allows you to set a limit on how long a Compute Server job can sit idle before the server kills the job.
opts.qpsol_options.gurobi.CSIdleTimeout = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]

% CSManager
% URL of the Cluster Manager for the Remote Services cluster
% Type: string
% Default value: ''
% URL of the Cluster Manager for the Remote Services cluster.
opts.qpsol_options.gurobi.CSManager = ''; % [Default: '']

% CSPriority
% Job priority for Remote Services job
% Type: int
% Default value: 0
% Minimum value: -100
% Maximum value: 100
% The priority of the Compute Server job.
opts.qpsol_options.gurobi.CSPriority = 0; % [Default: 0], [Allowed: {-100, 0, 100}]

% CSQueueTimeout
% Queue timeout for new jobs
% Type: double
% Default value: -1
% Minimum value: -1
% Maximum value: Infinity
% This parameter allows you to set a limit on how long a new Compute Server job will wait in queue.
opts.qpsol_options.gurobi.CSQueueTimeout = -1; % [Default: -1], [Allowed: {-1, 0, Inf}]

% CSRouter
% Router node for Remote Services cluster
% Type: string
% Default value: ''
% The router node for a Remote Services cluster.
opts.qpsol_options.gurobi.CSRouter = ''; % [Default: '']

% CSTLSInsecure
% Use insecure mode in Transport Layer Security (TLS)
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% Indicates whether the Remote Services cluster is using insecure mode in the TLS.
opts.qpsol_options.gurobi.CSTLSInsecure = 0; % [Default: 0], [Allowed: {0, 1}]

% CutAggPasses
% Constraint aggregation passes in cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% A non-negative value indicates the maximum number of constraint aggregation passes performed during cut generation.
opts.qpsol_options.gurobi.CutAggPasses = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]

% Cutoff
% Objective cutoff
% Type: double
% Default value: Infinity
% Minimum value: -Infinity
% Maximum value: Infinity
% Indicates that you aren’t interested in solutions whose objective values are worse than the specified value.
opts.qpsol_options.gurobi.Cutoff = 2147483648; % [Default: Inf], [Allowed: {-Inf, Inf}]

% CutPasses
% Cutting plane passes
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% A non-negative value indicates the maximum number of cutting plane passes performed during root cut generation.
opts.qpsol_options.gurobi.CutPasses = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]

% Cuts
% Global cut control
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% Global cut aggressiveness setting.
opts.qpsol_options.gurobi.Cuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% DegenMoves
% Degenerate simplex moves
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% Limits degenerate simplex moves performed to improve the integrality of the current relaxation solution.
opts.qpsol_options.gurobi.DegenMoves = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]

% Disconnected
% Disconnected component strategy
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls how aggressively the nature of disconnected components in a model is handled.
opts.qpsol_options.gurobi.Disconnected = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}].

% DisplayInterval
% Frequency of log lines
% Type: int
% Default value: 5
% Minimum value: 1
% Maximum value: MAXINT
% Determines the frequency at which log lines are printed (in seconds).
opts.qpsol_options.gurobi.DisplayInterval = 5; % [Default: 5], [Allowed: {1, 2000000000}]

% DistributedMIPJobs
% Distributed MIP job count
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: MAXINT
% Enables distributed MIP. A value of n causes the MIP solver to divide the work of solving a MIP model among n machines.
opts.qpsol_options.gurobi.DistributedMIPJobs = 0; % [Default: 0], [Allowed: {0, 2000000000}]

% DualImpliedCuts
% Dual implied bound cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls dual implied bound cut generation.
opts.qpsol_options.gurobi.DualImpliedCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% DualReductions
% Controls dual reductions
% Type: int
% Default value: 1
% Minimum value: 0
% Maximum value: 1
% Determines whether dual reductions are performed during the optimization process.
opts.qpsol_options.gurobi.DualReductions = 1; % [Default: 1], [Allowed: {0, 1}]

% FeasibilityTol
% Primal feasibility tolerance
% Type: double
% Default value: 1e-6
% Minimum value: 1e-9
% Maximum value: 1e-2
% All constraints must be satisfied to a tolerance of FeasibilityTol.
opts.qpsol_options.gurobi.FeasibilityTol = 1e-6; % [Default: 1e-6], [Allowed: {1e-9, 1e-2}]

% FeasRelaxBigM
% Big-M value for feasibility relaxations
% Type: double
% Default value: 1e6
% Minimum value: 0
% Maximum value: Infinity
% When relaxing a constraint in a feasibility relaxation, it is sometimes necessary to introduce a big-M value. This parameter determines the default magnitude of that value.
opts.qpsol_options.gurobi.FeasRelaxBigM = 1e6; % [Default: 1e6], [Allowed: {0, Inf}]

% FlowCoverCuts
% Flow cover cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls flow cover cut generation.
opts.qpsol_options.gurobi.FlowCoverCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% FlowPathCuts
% Flow path cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls flow path cut generation.
opts.qpsol_options.gurobi.FlowPathCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% FuncPieceError
% Error allowed for PWL translation of function constraints
% Type: double
% Default value: 1e-3
% Minimum value: 1e-6
% Maximum value: 1e+6
% Provides the maximum allowed error in the piecewise-linear approximation.
opts.qpsol_options.gurobi.FuncPieceError = 1e-3; % [Default: 1e-3], [Allowed: {1e-6, 1e6}]

% FuncPieceLength
% Piece length for PWL translation of function constraints
% Type: double
% Default value: 1e-2
% Minimum value: 1e-5
% Maximum value: 1e+6
% The length of each piece of the piecewise-linear approximation.
opts.qpsol_options.gurobi.FuncPieceLength = 1e-2; % [Default: 1e-2], [Allowed: {1e-5, 1e6}]

% FuncPieceRatio
% Control whether to under- or over-estimate function values in PWL approximation
% Type: double
% Default value: -1
% Minimum value: -1
% Maximum value: 1
% Controls whether the piecewise-linear approximation of a function constraint is an underestimate, overestimate, or interpolated.
opts.qpsol_options.gurobi.FuncPieceRatio = -1; % [Default: -1], [Allowed: {-1, 1}]

% FuncPieces
% Sets strategy for PWL function approximation
% Type: int
% Default value: 0
% Minimum value: -2
% Maximum value: 2e+8
% This parameter sets the strategy used for performing a piecewise-linear approximation of a function constraint.
opts.qpsol_options.gurobi.FuncPieces = 0; % [Default: 0], [Allowed: {-2, 0, 2000000000}]

% FuncMaxVal
% Maximum allowed value for x and y variables in function constraints with piecewise-linear approximation
% Type: double
% Default value: 1e+6
% Minimum value: 1e-2
% Maximum value: Infinity
% This parameter limits the bounds on the variables that participate in function constraints approximated by a piecewise-linear function.
opts.qpsol_options.gurobi.FuncMaxVal = 1e+6; % [Default: 1e+6], [Allowed: {1e-2, Inf}]

% FuncNonlinear
% Chooses the approximation approach used to handle function constraints
% Type: int
% Default value: 1
% Minimum value: 0
% Maximum value: 1
% This parameter controls whether general function constraints are replaced with a static piecewise-linear approximation or handled using a dynamic outer-approximation approach.
opts.qpsol_options.gurobi.FuncNonlinear = 1; % [Default: 1], [Allowed: {0, 1}]

% GomoryPasses
% Gomory cut passes
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% A non-negative value indicates the maximum number of Gomory cut passes performed.
opts.qpsol_options.gurobi.GomoryPasses = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]

% GUBCoverCuts
% GUB cover cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls GUB cover cut generation.
opts.qpsol_options.gurobi.GUBCoverCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% Heuristics
% Time spent in feasibility heuristics
% Type: double
% Default value: 0.05
% Minimum value: 0
% Maximum value: 1
% Determines the amount of time spent in MIP heuristics.
opts.qpsol_options.gurobi.Heuristics = 0.05; % [Default: 0.05], [Allowed: {0, 1}]

% IgnoreNames
% Indicates whether to ignore names provided by users
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% If set to 1, Gurobi will ignore any names provided by the user for subsequent variables or constraints.
opts.qpsol_options.gurobi.IgnoreNames = 0; % [Default: 0], [Allowed: {0, 1}]

% IISMethod
% Selects method used to compute IIS
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% Chooses the IIS method to use.
opts.qpsol_options.gurobi.IISMethod = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% ImpliedCuts
% Implied bound cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls implied bound cut generation.
opts.qpsol_options.gurobi.ImpliedCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% ImproveStartGap
% Solution improvement strategy control
% Type: double
% Default value: 0.0
% Minimum value: 0.0
% Maximum value: Infinity
% Specifies an optimality gap at which the MIP solver switches to a solution improvement strategy.
opts.qpsol_options.gurobi.ImproveStartGap = 0.0; % [Default: 0.0], [Allowed: {0.0, Inf}]

% ImproveStartNodes
% Solution improvement strategy control
% Type: double
% Default value: Infinity
% Minimum value: 0.0
% Maximum value: Infinity
% Specifies the node count at which the MIP solver switches to a solution improvement strategy.
opts.qpsol_options.gurobi.ImproveStartNodes = 2147483648; % [Default: Inf], [Allowed: {0.0, Inf}]

% ImproveStartTime
% Solution improvement strategy control
% Type: double
% Default value: Infinity
% Minimum value: 0.0
% Maximum value: Infinity
% Specifies the time when the MIP solver switches to a solution improvement strategy.
opts.qpsol_options.gurobi.ImproveStartTime = 2147483648; % [Default: Inf], [Allowed: {0.0, Inf}]

% InfProofCuts
% Infeasibility proof cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls infeasibility proof cut generation.
opts.qpsol_options.gurobi.InfProofCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% InfUnbdInfo
% Additional info for infeasible/unbounded models
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% Set this parameter if you want to query the unbounded ray or the infeasibility proof for infeasible models.
opts.qpsol_options.gurobi.InfUnbdInfo = 0; % [Default: 0], [Allowed: {0, 1}]

% InputFile
% Import data into a model before beginning optimization
% Type: string
% Default value: ''
% Specifies the name of a file that will be read before beginning a command-line optimization run.
% opts.qpsol_options.gurobi.InputFile = ''; % [Default: '']

% IntegralityFocus
% Integrality focus
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% Requests that the solver work harder to try to avoid solutions that exploit integrality tolerances.
opts.qpsol_options.gurobi.IntegralityFocus = 0; % [Default: 0], [Allowed: {0, 1}]

% IntFeasTol
% Integer feasibility tolerance
% Type: double
% Default value: 1e-5
% Minimum value: 1e-9
% Maximum value: 1e-1
% An integrality restriction on a variable is considered satisfied when the variable’s value is less than IntFeasTol from the nearest integer value.
opts.qpsol_options.gurobi.IntFeasTol = 1e-5; % [Default: 1e-5], [Allowed: {1e-9, 1e-1}]

% IterationLimit
% Simplex iteration limit
% Type: double
% Default value: Infinity
% Minimum value: 0
% Maximum value: Infinity
% Limits the number of simplex iterations performed.
opts.qpsol_options.gurobi.IterationLimit = 2147483648; % [Default: Inf], [Allowed: {0, Inf}]

% JobID
% Compute Server Job ID
% Type: string
% Default value: ''
% Provides the Compute Server Job ID for the current job.
opts.qpsol_options.gurobi.JobID = ''; % [Default: '']

% JSONSolDetail
% Level of detail in JSON solution format
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% Controls the amount of detail included in a JSON solution.
opts.qpsol_options.gurobi.JSONSolDetail = 0; % [Default: 0], [Allowed: {0, 1}]

% LazyConstraints
% Programs that use lazy constraints must set this parameter
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% Programs that add lazy constraints through a callback must set this parameter to value 1.
opts.qpsol_options.gurobi.LazyConstraints = 0; % [Default: 0], [Allowed: {0, 1}]

% LicenseID
% License ID
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: MAXINT
% When using a WLS license, set this parameter to the license ID.
% opts.qpsol_options.gurobi.LicenseID = 0; % [Default: 0], [Allowed: {0, 2000000000}]

% LiftProjectCuts
% Lift-and-project cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls lift-and-project cut generation.
opts.qpsol_options.gurobi.LiftProjectCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% LPWarmStart
% Controls whether and how to warm-start LP optimization
% Type: int
% Default value: 1
% Minimum value: 0
% Maximum value: 2
% Controls whether Gurobi uses warm start information for LP optimization.
opts.qpsol_options.gurobi.LPWarmStart = 1; % [Default: 1], [Allowed: {0, 1, 2}]

% LogFile
% Name for Gurobi log file
% Type: string
% Default value: ''
% Determines the name of the Gurobi log file.
opts.qpsol_options.gurobi.LogFile = ''; % [Default: '']

% LogToConsole
% Control console logging
% Type: int
% Default value: 1
% Minimum value: 0
% Maximum value: 1
% Enables or disables console logging.
opts.qpsol_options.gurobi.LogToConsole = 1; % [Default: 1], [Allowed: {0, 1}]

% MarkowitzTol
% Threshold pivoting tolerance
% Type: double
% Default value: 0.0078125
% Minimum value: 1e-4
% Maximum value: 0.999
% The Markowitz tolerance is used to limit numerical error in the simplex algorithm.
opts.qpsol_options.gurobi.MarkowitzTol = 0.0078125; % [Default: 0.0078125], [Allowed: {1e-4, 0.999}]

% MemLimit
% Memory limit
% Type: double
% Default value: Infinity
% Minimum value: 0
% Maximum value: Infinity
% Limits the total amount of memory available to Gurobi.
opts.qpsol_options.gurobi.MemLimit = 2147483648; % [Default: Inf], [Allowed: {0, Inf}]

% Method
% Algorithm used to solve continuous models
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 5
% Algorithm used to solve continuous models or the initial root relaxation of a MIP model.
opts.qpsol_options.gurobi.Method = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3, 4, 5}]

% MinRelNodes
% Minimum relaxation heuristic
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% Number of nodes to explore in the minimum relaxation heuristic.
opts.qpsol_options.gurobi.MinRelNodes = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]

% MIPFocus
% MIP solver focus
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 3
% Allows you to modify your high-level solution strategy, depending on your goals.
opts.qpsol_options.gurobi.MIPFocus = 0; % [Default: 0], [Allowed: {0, 1, 2, 3}]

% MIPGap
% Relative MIP optimality gap
% Type: double
% Default value: 1e-4
% Minimum value: 0
% Maximum value: Infinity
% The MIP solver will terminate (with an optimal result) when the gap between the lower and upper objective bound is less than MIPGap times the absolute value of the incumbent objective value.
opts.qpsol_options.gurobi.MIPGap = 1e-4; % [Default: 1e-4], [Allowed: {0, Inf}]

% MIPGapAbs
% Absolute MIP optimality gap
% Type: double
% Default value: 1e-10
% Minimum value: 0
% Maximum value: Infinity
% The MIP solver will terminate (with an optimal result) when the gap between the lower and upper objective bound is less than MIPGapAbs.
opts.qpsol_options.gurobi.MIPGapAbs = 1e-10; % [Default: 1e-10], [Allowed: {0, Inf}]

% MIPSepCuts
% MIP separation cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls MIP separation cut generation.
opts.qpsol_options.gurobi.MIPSepCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% MIQCPMethod
% Method used to solve MIQCP models
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 1
% Controls the method used to solve MIQCP models.
opts.qpsol_options.gurobi.MIQCPMethod = -1; % [Default: -1], [Allowed: {-1, 0, 1}]

% MIRCuts
% MIR cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls Mixed Integer Rounding (MIR) cut generation.
opts.qpsol_options.gurobi.MIRCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% MixingCuts
% Mixing cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls Mixing cut generation.
opts.qpsol_options.gurobi.MixingCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% ModKCuts
% Mod-k cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls mod-k cut generation.
opts.qpsol_options.gurobi.ModKCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% MultiObjMethod
% Method used for multi-objective solves
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls the method used for solving multi-objective models.
opts.qpsol_options.gurobi.MultiObjMethod = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% MultiObjPre
% Initial presolve level on multi-objective models
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls the initial presolve level used for multi-objective models.
opts.qpsol_options.gurobi.MultiObjPre = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% MultiObjSettings
% Create multi-objective settings from a list of .prm files
% Type: string
% Default value: ''
% Command-line only parameter to set parameter files for different solves in a multi-objective model.
% opts.qpsol_options.gurobi.MultiObjSettings = ''; % [Default: '']

% NetworkAlg
% Network simplex algorithm
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 1
% Controls whether to use network simplex.
opts.qpsol_options.gurobi.NetworkAlg = -1; % [Default: -1], [Allowed: {-1, 0, 1}]

% NetworkCuts
% Network cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls network cut generation.
opts.qpsol_options.gurobi.NetworkCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% NLPHeur
% Controls the NLP heuristic
% Type: int
% Default value: 1
% Minimum value: 0
% Maximum value: 1
% The NLP heuristic uses a non-linear barrier solver to find feasible solutions.
opts.qpsol_options.gurobi.NLPHeur = 1; % [Default: 1], [Allowed: {0, 1}]

% NodefileDir
% Directory for node files
% Type: string
% Default value: '.'
% Determines the directory into which nodes are written when node memory usage exceeds the specified NodefileStart value.
opts.qpsol_options.gurobi.NodefileDir = '.'; % [Default: '.']

% NodefileStart
% Write MIP nodes to disk
% Type: double
% Default value: Inf
% Minimum value: 0
% Maximum value: Inf
% Limits the memory usage when storing MIP nodes.
opts.qpsol_options.gurobi.NodefileStart = 2147483648; % [Default: Inf], [Allowed: {0, Inf}]

% NodeLimit
% MIP node limit
% Type: double
% Default value: Inf
% Minimum value: 0
% Maximum value: Inf
% Limits the number of MIP nodes explored.
opts.qpsol_options.gurobi.NodeLimit = 2147483648; % [Default: Inf], [Allowed: {0, Inf}]

% NodeMethod
% Method used to solve MIP node relaxations
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Algorithm used for MIP node relaxations.
opts.qpsol_options.gurobi.NodeMethod = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% NonConvex
% Strategy for handling non-convex quadratic programs
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Sets the strategy for handling non-convex quadratic objectives or non-convex quadratic constraints.
opts.qpsol_options.gurobi.NonConvex = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% NoRelHeurTime
% Limits the amount of time spent in the NoRel heuristic
% Type: double
% Default value: 0
% Minimum value: 0
% Maximum value: Infinity
% Limits the amount of time spent in the NoRel heuristic.
opts.qpsol_options.gurobi.NoRelHeurTime = 0; % [Default: 0], [Allowed: {0, Inf}]

% NoRelHeurWork
% Limits the amount of work spent in the NoRel heuristic
% Type: double
% Default value: 0
% Minimum value: 0
% Maximum value: Infinity
% Limits the amount of work spent in the NoRel heuristic.
opts.qpsol_options.gurobi.NoRelHeurWork = 0; % [Default: 0], [Allowed: {0, Inf}]

% NormAdjust
% Choose simplex pricing norm
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% Chooses among various simplex pricing norms for the pricing algorithm.
opts.qpsol_options.gurobi.NormAdjust = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% NumericFocus
% Numerical focus
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 3
% Controls the degree to which the code attempts to detect and manage numerical issues.
opts.qpsol_options.gurobi.NumericFocus = 0; % [Default: 0], [Allowed: {0, 1, 2, 3}]

% OBBT
% Controls aggressiveness of optimality-based bound tightening
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% Control the aggressiveness of bound tightening based on optimality.
opts.qpsol_options.gurobi.OBBT = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% ObjNumber
% Selects objective index of multi-objectives
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: MAXINT
% Selects the index of the objective you want to work with in multi-objective models.
opts.qpsol_options.gurobi.ObjNumber = 0; % [Default: 0], [Allowed: {0, 2000000000}]

% ObjScale
% Objective scaling
% Type: double
% Default value: 0.0
% Minimum value: -1
% Maximum value: Infinity
% Divides the model objective by the specified value to avoid numerical issues.
opts.qpsol_options.gurobi.ObjScale = 0.0; % [Default: 0.0], [Allowed: {-1, Inf}]

% OptimalityTol
% Dual feasibility tolerance
% Type: double
% Default value: 1e-6
% Minimum value: 1e-9
% Maximum value: 1e-2
% Reduced costs must be smaller than this value in the improving direction.
opts.qpsol_options.gurobi.OptimalityTol = 1e-6; % [Default: 1e-6], [Allowed: {1e-9, 1e-2}]

% OutputFlag
% Controls Gurobi output
% Type: int
% Default value: 1
% Minimum value: 0
% Maximum value: 1
% Enables or disables solver output to the console/log.
opts.qpsol_options.gurobi.OutputFlag = 1; % [Default: 1], [Allowed: {0, 1}]

% PartitionPlace
% Controls where the partition heuristic runs
% Type: int
% Default value: 15
% Minimum value: 0
% Maximum value: 31
% Determines the positions of the partitioning heuristic in the optimization process.
opts.qpsol_options.gurobi.PartitionPlace = 15; % [Default: 15], [Allowed: {0, 31}]

% PerturbValue
% Simplex perturbation
% Type: double
% Default value: 0.0002
% Minimum value: 0
% Maximum value: Infinity
% Magnitude of the simplex perturbation that is applied when progress has stalled.
opts.qpsol_options.gurobi.PerturbValue = 0.0002; % [Default: 0.0002], [Allowed: {0, Inf}]

% PoolGap
% Maximum relative gap for stored solutions
% Type: double
% Default value: Infinity
% Minimum value: 0
% Maximum value: Infinity
% Determines how large a (relative) gap to tolerate in stored solutions.
opts.qpsol_options.gurobi.PoolGap = 2147483648; % [Default: Inf], [Allowed: {0, Inf}]

% PoolGapAbs
% Maximum absolute gap for stored solutions
% Type: double
% Default value: Infinity
% Minimum value: 0
% Maximum value: Infinity
% Determines how large a (absolute) gap to tolerate in stored solutions.
opts.qpsol_options.gurobi.PoolGapAbs = 2147483648; % [Default: Inf], [Allowed: {0, Inf}]

% PoolSearchMode
% Selects different modes for exploring the MIP search tree
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 2
% Selects different modes for exploring the MIP search tree.
opts.qpsol_options.gurobi.PoolSearchMode = 0; % [Default: 0], [Allowed: {0, 1, 2}]

% PoolSolutions
% Number of MIP solutions to store
% Type: int
% Default value: 10
% Minimum value: 1
% Maximum value: MAXINT
% Sets the limit for how many MIP solutions are stored.
opts.qpsol_options.gurobi.PoolSolutions = 10; % [Default: 10], [Allowed: {1, 2000000000}]

% PreCrush
% Controls presolve reductions that affect user cuts
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% Shuts off certain reductions that may affect the application of user cuts.
opts.qpsol_options.gurobi.PreCrush = 0; % [Default: 0], [Allowed: {0, 1}]

% PreDepRow
% Controls presolve dependent row reduction
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 1
% Enables or disables the reduction of dependent rows during presolve.
opts.qpsol_options.gurobi.PreDepRow = -1; % [Default: -1], [Allowed: {-1, 0, 1}]

% PreDual
% Controls presolve model dualization
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls whether presolve forms the dual of continuous models or not.
opts.qpsol_options.gurobi.PreDual = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% PreMIQCPForm
% Format of presolved MIQCP model
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Determines the format of the presolved version of an MIQCP model.
opts.qpsol_options.gurobi.PreMIQCPForm = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% PrePasses
% Presolve pass limit
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% Limits the number of passes performed by presolve.
opts.qpsol_options.gurobi.PrePasses = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]

% PreQLinearize
% Presolve quadratic linearization
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls quadratic expressions linearization during presolve.
opts.qpsol_options.gurobi.PreQLinearize = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% Presolve
% Controls the presolve level
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Determines how aggressive presolve should be.
opts.qpsol_options.gurobi.Presolve = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% PreSOS1BigM
% Threshold for SOS1-to-binary reformulation
% Type: double
% Default value: -1
% Minimum value: -1
% Maximum value: 1e10
% Controls the automatic reformulation of SOS1 constraints into binary form.
opts.qpsol_options.gurobi.PreSOS1BigM = -1; % [Default: -1], [Allowed: {-1, 0, 1e10}]

% PreSOS1Encoding
% Encoding used for SOS1 reformulation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% Controls the automatic reformulation of SOS1 constraints.
opts.qpsol_options.gurobi.PreSOS1Encoding = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% PreSOS2BigM
% Threshold for SOS2-to-binary reformulation
% Type: double
% Default value: -1
% Minimum value: -1
% Maximum value: 1e10
% Controls the automatic reformulation of SOS2 constraints into binary form.
opts.qpsol_options.gurobi.PreSOS2BigM = -1; % [Default: -1], [Allowed: {-1, 0, 1e10}]

% PreSOS2Encoding
% Encoding used for SOS2 reformulation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% Controls the automatic reformulation of SOS2 constraints.
opts.qpsol_options.gurobi.PreSOS2Encoding = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% PreSparsify
% Controls the presolve sparsify reduction
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Determines whether or not the sparsify reduction will be applied during presolve.
opts.qpsol_options.gurobi.PreSparsify = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% ProjImpliedCuts
% Projected implied bound cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls projected implied bound cut generation.
opts.qpsol_options.gurobi.ProjImpliedCuts = -1; % [Default: -1

; % [Allowed: {-1, 0, 1, 2}]

% PSDCuts
% PSD cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls PSD cut generation. Use 0 to disable these cuts, 1 for moderate cut generation, or 2 for aggressive cut generation.
opts.qpsol_options.gurobi.PSDCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% PSDTol
% Positive semi-definite tolerance
% Type: double
% Default value: 1e-6
% Minimum value: 0
% Maximum value: Infinity
% Sets a limit on the amount of diagonal perturbation that the optimizer is allowed to perform on a Q matrix to correct minor PSD violations.
opts.qpsol_options.gurobi.PSDTol = 1e-6; % [Default: 1e-6], [Allowed: {0, Inf}]

% PumpPasses
% Passes of the feasibility pump heuristic
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% Number of passes of the feasibility pump heuristic.
opts.qpsol_options.gurobi.PumpPasses = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]

% QCPDual
% Dual variables for QCP models
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% Determines whether dual variable values are computed for QCP models.
opts.qpsol_options.gurobi.QCPDual = 0; % [Default: 0], [Allowed: {0, 1}]

% Quad
% Controls quad precision in simplex
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 1
% Enables or disables quad precision computation in simplex.
opts.qpsol_options.gurobi.Quad = -1; % [Default: -1], [Allowed: {-1, 0, 1}]

% Record
% Enables API call recording
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% Enables API call recording. Gurobi will write files capturing the sequence of commands issued.
opts.qpsol_options.gurobi.Record = 0; % [Default: 0], [Allowed: {0, 1}]

% ResultFile
% Write a result file upon completion of optimization
% Type: string
% Default value: ''
% Specifies the name of the result file to be written upon completion of optimization.
opts.qpsol_options.gurobi.ResultFile = ''; % [Default: '']

% RINS
% Relaxation Induced Neighborhood Search (RINS) heuristic frequency
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% Frequency of the RINS heuristic, which applies RINS at every n-th node of the MIP search tree.
opts.qpsol_options.gurobi.RINS = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]

% RelaxLiftCuts
% Relax-and-lift cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls relax-and-lift cut generation.
opts.qpsol_options.gurobi.RelaxLiftCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% RLTCuts
% RLT cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls Relaxation Linearization Technique (RLT) cut generation.
opts.qpsol_options.gurobi.RLTCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% ScaleFlag
% Model scaling
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% Controls model scaling to improve numerical properties of the constraint matrix.
opts.qpsol_options.gurobi.ScaleFlag = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% ScenarioNumber
% Selects scenario index of multi-scenario models
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: MAXINT
% When working with multiple scenarios, this parameter selects the index of the scenario you want to work with.
opts.qpsol_options.gurobi.ScenarioNumber = 0; % [Default: 0], [Allowed: {0, 2000000000}]

% Seed
% Random number seed
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: MAXINT
% Modifies the random number seed, leading to different solution paths.
opts.qpsol_options.gurobi.Seed = 0; % [Default: 0], [Allowed: {0, 2000000000}]

% ServerPassword
% Client password for Remote Services cluster or token server
% Type: string
% Default value: ''
% The password for connecting to the server (Compute Server or token server).
% opts.qpsol_options.gurobi.ServerPassword = ''; % [Default: '']

% ServerTimeout
% Network timeout
% Type: int
% Default value: 60
% Minimum value: 1
% Maximum value: MAXINT
% Network time-out for Compute Server and token server.
% opts.qpsol_options.gurobi.ServerTimeout = 60; % [Default: 60], [Allowed: {1, 2000000000}]

% Sifting
% Controls sifting within dual simplex
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Enables or disables sifting within dual simplex, which can be useful for LP models with many more variables than constraints.
opts.qpsol_options.gurobi.Sifting = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% SiftMethod
% LP method used to solve sifting sub-problems
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% LP method used to solve sub-problems during sifting.
opts.qpsol_options.gurobi.SiftMethod = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% SimplexPricing
% Simplex pricing strategy
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% Determines the simplex variable pricing strategy.
opts.qpsol_options.gurobi.SimplexPricing = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% SoftMemLimit
% Soft memory limit
% Type: double
% Default value: Infinity
% Minimum value: 0
% Maximum value: Infinity
% Limits the total amount of memory available to Gurobi. 
opts.qpsol_options.gurobi.SoftMemLimit = 2147483648; % [Default: Inf], [Allowed: {0, Inf}]

% SolutionLimit
% MIP solution limit
% Type: int
% Default value: MAXINT
% Minimum value: 1
% Maximum value: MAXINT
% Limits the number of feasible MIP solutions found.
opts.qpsol_options.gurobi.SolutionLimit = 2000000000; % [Default: 2000000000], [Allowed: {1, MAXINT}]

% SolutionTarget
% Solution Target for LP
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 1
% Specifies the solution target for linear programs (LP).
opts.qpsol_options.gurobi.SolutionTarget = -1; % [Default: -1], [Allowed: {-1, 0, 1}]

% SolFiles
% Location to store intermediate solution files
% Type: string
% Default value: ''
% During the MIP solution process, this parameter specifies where to store intermediate solution files.
opts.qpsol_options.gurobi.SolFiles = ''; % [Default: '']

% SolutionNumber
% Select a sub-optimal MIP solution
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: MAXINT
% When retrieving a solution, selects which one to return if multiple solutions are stored.
opts.qpsol_options.gurobi.SolutionNumber = 0; % [Default: 0], [Allowed: {0, 2000000000}]

% StartNodeLimit
% Limit MIP start sub-MIP nodes
% Type: int
% Default value: -1
% Minimum value: -3
% Maximum value: MAXINT
% This parameter limits the number of branch-and-bound nodes explored when completing a partial MIP start.
opts.qpsol_options.gurobi.StartNodeLimit = -1; % [Default: -1], [Allowed: {-3, -2, -1, 0, 2000000000}]

% StartNumber
% Selects MIP start index
% Type: int
% Default value: 0
% Minimum value: -1
% Maximum value: MAXINT
% Selects the index of the MIP start you want to work with.
opts.qpsol_options.gurobi.StartNumber = 0; % [Default: 0], [Allowed: {-1, 0, 2000000000}]

% StrongCGCuts
% Strong-CG cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls Strong Chvátal-Gomory (Strong-CG) cut generation.
opts.qpsol_options.gurobi.StrongCGCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% SubMIPCuts
% Sub-MIP cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls sub-MIP cut generation.
opts.qpsol_options.gurobi.SubMIPCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% SubMIPNodes
% Nodes explored in sub-MIP heuristics
% Type: int
% Default value: 500
% Minimum value: 0
% Maximum value: MAXINT
% Limits the number of nodes explored by MIP-based heuristics (such as RINS).
opts.qpsol_options.gurobi.SubMIPNodes = 500; % [Default: 500], [Allowed: {0, 2000000000}]

% Symmetry
% Symmetry detection
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls symmetry detection.
opts.qpsol_options.gurobi.Symmetry = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% ThreadLimit
% Thread limit
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1024
% The ThreadLimit parameter is a configuration parameter for an environment that can be used to limit the number of threads used.
opts.qpsol_options.gurobi.ThreadLimit = 0; % [Default: 0], [Allowed: {0, 1024}]

% Threads
% Thread count
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1024
% Controls the number of threads to apply to parallel algorithms.
opts.qpsol_options.gurobi.Threads = 0; % [Default: 0], [Allowed: {0, 1024}]

% TimeLimit
% Time limit
% Type: double
% Default value: Infinity
% Minimum value: 0
% Maximum value: Infinity
% Limits the total time expended (in seconds) for solving the model.
opts.qpsol_options.gurobi.TimeLimit = 2147483648; % [Default: Inf], [Allowed: {0, Inf}]

% TokenServer
% Name of your token server
% Type: string
% Default value: ''
% When using a token license, set this parameter to the name of the token server.
% opts.qpsol_options.gurobi.TokenServer = ''; % [Default: '']

% TSPort
% Port for token server
% Type: int
% Default value: 41954
% Minimum value: 0
% Maximum value: 65536
% Port to use when connecting to the Gurobi token server.
% opts.qpsol_options.gurobi.TSPort = 41954; % [Default: 41954], [Allowed: {0, 65536}]

% TuneBaseSettings
% Comma-separated list of base parameter settings
% Type: string
% Default value: ''
% A list of parameter files that define settings that should be tried first during the tuning process.
% opts.qpsol_options.gurobi.TuneBaseSettings = ''; % [Default: '']

% TuneCleanup
% Enables a tuning cleanup phase
% Type: double
% Default value: 0.0
% Minimum value: 0.0
% Maximum value: 1.0
% Enables a cleanup phase at the end of tuning to reduce the number of parameter changes required.
% opts.qpsol_options.gurobi.TuneCleanup = 0.0; % [Default: 0.0], [Allowed: {0.0, 1.0}]

% TuneCriterion
% Tuning criterion
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% Modifies the tuning criterion for the tuning tool.
% opts.qpsol_options.gurobi.TuneCriterion = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% TuneDynamicJobs
% Dynamic distributed tuning job count
% Type: int
% Default value: 0
% Minimum value: -1
% Maximum value: MAXINT
% Enables distributed parallel tuning to increase performance of the tuning tool.
% opts.qpsol_options.gurobi.TuneDynamicJobs = 0; % [Default: 0], [Allowed: {-1, 0, 2000000000}]

% TuneJobs
% Permanent distributed tuning job count
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: MAXINT
% Enables distributed parallel tuning using a static set of workers.
% opts.qpsol_options.gurobi.TuneJobs = 0; % [Default: 0], [Allowed: {0, 2000000000}]

% TuneMetric
% Method for aggregating tuning results
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 1
% Controls how timed results are aggregated during tuning.
% opts.qpsol_options.gurobi.TuneMetric = -1; % [Default: -1], [Allowed: {-1, 0, 1}]

% TuneOutput
% Tuning output level
% Type: int
% Default value: 2
% Minimum value: 0
% Maximum value: 3
% Controls the amount of output produced by the tuning tool.
% opts.qpsol_options.gurobi.TuneOutput = 2; % [Default: 2], [Allowed: {0, 1, 2, 3}]

% TuneParams
% Comma-separated list of JSON files containing parameters with their properties the tuner should consider.
% Type: string
% Default value: ''
% A list of JSON files defining parameters for the tuning process.
% opts.qpsol_options.gurobi.TuneParams = ''; % [Default: '']

% TuneResults
% Number of improved parameter sets returned
% Type: int
% Default value: -1
% Minimum value: -2
% Maximum value: MAXINT
% Controls how many parameter sets should be retained when tuning is complete.
% opts.qpsol_options.gurobi.TuneResults = -1; % [Default: -1], [Allowed: {-2, -1, 2000000000}]

% TuneTargetMIPGap
% A target gap to be reached
% Type: double
% Default value: 0
% Minimum value: 0
% Maximum value: Infinity
% A target gap the tuning tool will try to achieve.
% opts.qpsol_options.gurobi.TuneTargetMIPGap = 0; % [Default: 0], [Allowed: {0, Inf}]

% TuneTargetTime
% A target runtime in seconds to be reached
% Type: double
% Default value: 0.005
% Minimum value: 0
% Maximum value: Infinity
% A target runtime for tuning, the tuner stops when this is reached.
% opts.qpsol_options.gurobi.TuneTargetTime = 0.005; % [Default: 0.005], [Allowed: {0, Inf}]

% TuneTimeLimit
% Tuning tool time limit
% Type: double
% Default value: Infinity
% Minimum value: 0
% Maximum value: Infinity
% Limits total tuning runtime (in seconds).
% opts.qpsol_options.gurobi.TuneTimeLimit = 2147483648; % [Default: Inf], [Allowed: {0, Inf}]

% TuneTrials
% Perform multiple runs on each parameter set to limit the effect of random noise
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: MAXINT
% Allows performing multiple solves for each parameter set to reduce randomness influence.
% opts.qpsol_options.gurobi.TuneTrials = 0; % [Default: 0], [Allowed: {0, 2000000000}]

% TuneUseFilename
% Use model file names as model names
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: 1
% When set to 1, tuning will refer to the model using the name of the model file instead.
% opts.qpsol_options.gurobi.TuneUseFilename = 0; % [Default: 0], [Allowed: {0, 1}]

% UpdateMode
% Changes the behavior of lazy updates
% Type: int
% Default value: 1
% Minimum value: 0
% Maximum value: 1
% Determines how newly added variables and linear constraints are handled.
opts.qpsol_options.gurobi.UpdateMode = 1; % [Default: 1], [Allowed: {0, 1}]

% Username
% User name for Remote Services
% Type: string
% Default value: ''
% Identify the user connecting to the Remote Services Manager.
% opts.qpsol_options.gurobi.Username = ''; % [Default: '']

% VarBranch
% Branch variable selection strategy
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 3
% Controls the branch variable selection strategy.
opts.qpsol_options.gurobi.VarBranch = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2, 3}]

% WLSAccessID
% Web License Service access ID
% Type: string
% Default value: ''
% When using a WLS license, set this parameter to the access ID for your license.
% opts.qpsol_options.gurobi.WLSAccessID = ''; % [Default: '']

% WLSConfig
% Web License Service configuration
% Type: string
% Default value: ''
% When using a WLS On Demand license, specifies which configuration to use.
% opts.qpsol_options.gurobi.WLSConfig = ''; % [Default: '']

% WLSProxy
% Web License Service proxy
% Type: string
% Default value: ''
% Comma-separated list of addresses of the WLS proxies to connect to.
% opts.qpsol_options.gurobi.WLSProxy = ''; % [Default: '']

% WLSSecret
% Web License Service secret
% Type: string
% Default value: ''
% When using a WLS license, set this parameter to the secret key for your license.
% opts.qpsol_options.gurobi.WLSSecret = ''; % [Default: '']

% WLSToken
% Web License Service token
% Type: string
% Default value: ''
% If you retrieved your own token through the WLS REST API, use this parameter to pass that token to the library.
% opts.qpsol_options.gurobi.WLSToken = ''; % [Default: '']

% WLSTokenDuration
% Web License Service token duration
% Type: int
% Default value: 0
% Minimum value: 0
% Maximum value: MAXINT
% This parameter can be used to adjust the lifespan of a token.
% opts.qpsol_options.gurobi.WLSTokenDuration = 0; % [Default: 0], [Allowed: {0, 2000000000}]

% WLSTokenRefresh
% Web License Service token refresh interval
% Type: double
% Default value: 0.9
% Minimum value: 0.0
% Maximum value: 1.0
% Specifies the fraction of token duration after which a token refresh is triggered.
% opts.qpsol_options.gurobi.WLSTokenRefresh = 0.9; % [Default: 0.9], [Allowed: {0.0, 1.0}]

% WorkerPassword
% Distributed worker password
% Type: string
% Default value: ''
% When using a distributed algorithm, this parameter allows you to specify the password for the distributed worker cluster.
% opts.qpsol_options.gurobi.WorkerPassword = ''; % [Default: '']

% WorkerPool
% Distributed worker cluster (for distributed algorithms)
% Type: string
% Default value: ''
% Allows you to specify a Remote Services cluster that will provide distributed workers.
opts.qpsol_options.gurobi.WorkerPool = ''; % [Default: '']

% WorkLimit
% Work limit
% Type: double
% Default value: Infinity
% Minimum value: 0
% Maximum value: Infinity
% Limits the total work expended (in work units).
opts.qpsol_options.gurobi.WorkLimit = 2147483648; % [Default: Inf], [Allowed: {0, Inf}]

% ZeroHalfCuts
% Zero-half cut generation
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: 2
% Controls zero-half cut generation.
opts.qpsol_options.gurobi.ZeroHalfCuts = -1; % [Default: -1], [Allowed: {-1, 0, 1, 2}]

% ZeroObjNodes
% Zero-objective heuristic
% Type: int
% Default value: -1
% Minimum value: -1
% Maximum value: MAXINT
% Number of nodes to explore in the zero objective heuristic.
opts.qpsol_options.gurobi.ZeroObjNodes = -1; % [Default: -1], [Allowed: {-1, 0, 2000000000}]