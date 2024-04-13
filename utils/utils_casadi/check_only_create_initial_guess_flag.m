plot_init_guess                 = false; % plot init guess
plot_null_simu                  = false; % plot system simulation for x0=0, u0 = ID(x0)
convert_maple_to_casadi         = false; % convert maple functions into casadi functions
fullsimu                        = false; % make full mpc simulation and plot results
weights_and_limits_as_parameter = true; % otherwise minimal set of inputs and parameter is used. Leads to faster run time and compile time.
compile_sfunction               = false; % needed for simulink s-function, filename: "s_function_"+casadi_func_name
compile_matlab_sfunction        = false; % only needed for matlab MPC simu, filename: "casadi_func_name