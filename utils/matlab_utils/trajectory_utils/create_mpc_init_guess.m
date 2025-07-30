import casadi.*;
% Create initial guess for MPC optimization problem
% This script calculates the initial guess for the MPC optimization problem
% based on the provided trajectory settings and parameters.

if(overwrite_init_guess)

    % Calculate initial guess for alle trajectories and mpcs
    weights_and_limits_as_parameter = true;
    plot_null_simu                  = false;
    print_init_guess_cost_functions = false;

    fprintf('\n\nStart execution of ''create_mpc_init_guess.m''\n');

    % create init_guess
    try

        tic
        for name={files.name}
            name_mat_file    = name{1};
            param_MPC_name   = name_mat_file(1:end-4);
            param_MPC_struct = eval(param_MPC_name);

            param_MPC_init_guess_name = [param_MPC_name, '_init_guess'];
            param_MPC_init_guess_mat_file = [s_fun_path, '/initial_guess/', param_MPC_init_guess_name,'.mat'];
            param_MPC_init_guess_bin_file = [s_fun_path, '/initial_guess/', param_MPC_init_guess_name,'.bin'];

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
            
            traj_names = fieldnames(traj_settings.traj_mode);

            init_guess_cell = cell(1, param_traj.N_traj);            
            for ii=1:param_traj.N_traj
                fprintf('--------------------------------------------------------------------\n\n');
                fprintf(['Creating initial guess for trajectory ',num2str(ii),' (', traj_names{param_traj.traj_type(ii)},'):\n\n']);
                param_trajectory = param_traj_data_fun(traj_settings, 'get', ii, param_traj_data);
                traj_select_mpc = ii;

                % Achtung, hier drin werden for loops mit der Variable i verwendet, daher darf die äußere
                % Schleife nicht die Variable i verwenden!!!!
                nlpsol_generate_opt_problem;
                
                if(warm_start)
                    if(weights_and_limits_as_parameter)
                        [~, xx_full_opt_sol, ~] = f_opt(mpc_init_reference_values, init_guess_0, param_weight_init_cell);
                    else
                        [~, xx_full_opt_sol] = f_opt(mpc_init_reference_values, init_guess_0);
                    end
                
                    init_guess = full(xx_full_opt_sol);
                else
                    init_guess = full(init_guess_0');
                end

                init_guess_cell{ii} = init_guess;
            end

            init_guess_arr = vertcat(init_guess_cell{:});

            param_MPC = struct('init_guess', init_guess_arr);
            
            eval([param_MPC_init_guess_name, ' = param_MPC;']);

            % TODO: sauberes first-start init
            % create init_guess folder if not exist
            if ~exist([s_fun_path, '/initial_guess'], 'dir')
                mkdir([s_fun_path, '/initial_guess']);
            end

            save(param_MPC_init_guess_mat_file, param_MPC_init_guess_name);
            save_init_guess_as_binary(init_guess_arr, param_MPC_init_guess_bin_file);
        end
        fprintf('--------------------------------------------------------------------\n\n');
        fprintf(['parameter.m: Execution Time for Init guess Calculation: ', sprintf('%f', toc), 's\n\n']);
        fprintf('Execution of ''create_mpc_init_guess.m'' finished\n');
        fprintf('--------------------------------------------------------------------\n\n');
    catch ME
        disp('Cannot create trajectory, please compile new')
        fprintf(2, 'Fehler: %s\n', getReport(ME));
    end
else
    init_guess_files = dir([s_fun_path, '/initial_guess/*.mat']);
    file_list = cellfun(@(file) [s_fun_path, '/initial_guess/', file], {init_guess_files.name}, 'UniformOutput', false);
    cellfun(@load, file_list);
end