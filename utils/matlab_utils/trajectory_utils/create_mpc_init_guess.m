import casadi.*;

% Calculate initial guess for alle trajectories and mpcs
compile_sfun                    = false;
weights_and_limits_as_parameter = true;
plot_null_simu                  = false;
print_init_guess_cost_functions = false;

tic
for name={files.name}
    name_mat_file    = name{1};
    param_MPC_name   = name_mat_file(1:end-4);
    param_MPC_struct = eval(param_MPC_name);

    param_MPC_init_guess_name = [param_MPC_name, '_init_guess'];
    param_MPC_init_guess_mat_file = [s_fun_path, '/initial_guess/', param_MPC_init_guess_name,'.mat'];

    fprintf('\n');
    disp(['Calculating init guess for: ', param_MPC_name]);

    casadi_func_name = param_MPC_struct.name;
    MPC_variant      = param_MPC_struct.variant;
    MPC_solver       = param_MPC_struct.solver;
    Ts_MPC           = param_MPC_struct.Ts     ;
    rk_iter          = param_MPC_struct.rk_iter;
    N_MPC            = param_MPC_struct.N  ;
    T_horizon_MPC    = param_MPC_struct.T_horizon;                   
    N_step_MPC       = param_MPC_struct.N_step;
    MPC_version      = param_MPC_struct.version;
    int_method       = param_MPC_struct.int_method;
    fixed_parameter  = param_MPC_struct.fixed_parameter;

    weights_and_limits_as_parameter = ~fixed_parameter;
    DT = Ts_MPC;
    M = rk_iter;

    output_dir = [s_fun_path,'/'];
    input_dir = [s_fun_path,'/casadi_functions/'];

    f_opt = Function.load([s_fun_path, '/', casadi_func_name, '.casadi']);

    param_weight_init = param_weight.(casadi_func_name);
    param_weight_init_cell = merge_cell_arrays(struct2cell(param_weight_init), 'vector');

    init_guess_cell = cell(1, param_traj.N_traj);            
    for ii=1:param_traj.N_traj
        param_trajectory = param_traj_data_fun(traj_settings, 'get', ii, param_traj_data);
        traj_select_mpc = param_traj.traj_type(ii);

        % Achtung, hier drin werden for loops mit der Variable i verwendet, daher darf die äußere
        % Schleife nicht die Variable i verwenden!!!!
        nlpsol_generate_opt_problem;
        
        % Hint: If the init_guess is already the global minima, the solver
        % will get an Evaluation Failed because it get's NaN values due to
        % the perfect zero gradients. Therefore add eps to init_guess to
        % avoid this problem.
        init_guess_0 = eps + init_guess_0;
        if(weights_and_limits_as_parameter)
            % TODO - why +1e-15 necessary??
            [~, xx_full_opt_sol, ~] = f_opt(mpc_init_reference_values, init_guess_0, param_weight_init_cell);
        else
            [~, xx_full_opt_sol] = f_opt(mpc_init_reference_values, init_guess_0);
        end
        init_guess = full(xx_full_opt_sol);

        init_guess_cell{ii} = init_guess;
    end

    init_guess_arr = vertcat(init_guess_cell{:});

    param_MPC = struct('init_guess', init_guess_arr);
    
    eval([param_MPC_init_guess_name, ' = param_MPC;']);
    save(param_MPC_init_guess_mat_file, param_MPC_init_guess_name);
end
disp(['parameter.m: Execution Time for Init guess Calculation: ', sprintf('%f', toc), 's']);

init_MPC_weights; % why necessary?