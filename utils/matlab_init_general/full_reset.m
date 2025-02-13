if(full_reset_flag)
    % get name of starting script
    S = dbstack('-completenames');
    if ~isempty(S)
        [~, filename, ext] = fileparts(S(end).file);
        % disp(['The first script started was: ' filename ext]);
    end

    if(~strcmp(filename, 'mpc_casadi_main') && ~reset_started_flag) % otherwise ignore flag
        try

        % get user prompt for checking whether sure
        disp('Full reset of all files and folders in ./s_functions/robot_name/ is about to be done.');
        disp('Are you sure you want to proceed? (y/n)');
        user_response = input('','s');
        
            if(strcmp(user_response, 'y'))
                reset_started_flag = true;
                dont_clear = true;

                % delete all files and folder in ./s_functions/robot_name/
                if exist(s_fun_path, 'dir')
                    removeDir([s_fun_path, '/']);
                end
                % create folder ./s_functions/robot_name/
                
                mkdir(s_fun_path);

                % create necessary folders:
                % ./s_functions/robot_name/casadi_functions
                % ./s_functions/robot_name/initial_guess
                % ./s_functions/robot_name/matlab_functions
                % ./s_functions/robot_name/mpc_c_sourcefiles
                % ./s_functions/robot_name/mpc_settings
                % ./s_functions/robot_name/trajectory_data

                folder_names = {'casadi_functions', 'initial_guess', 'matlab_functions', 'mpc_c_sourcefiles', 'mpc_settings', 'trajectory_data'};
                for i = 1:length(folder_names)
                    mkdir([s_fun_path, '/', folder_names{i}]);
                end
                
                % remove checksum file './utils/utils_casadi/casadi_mpc_definitions/checksums.txt' if exists
                if exist(['./utils/utils_casadi/casadi_mpc_definitions/checksums.txt'], 'file')
                    delete(['./utils/utils_casadi/casadi_mpc_definitions/checksums.txt']);
                end

                % regenerate casadi functions: run './urdf_creation/7dof_sys_pinocchio3_to_casadi.py'
                disp('Please run the file ./urdf_creation/7dof_sys_pinocchio3_to_casadi.py to regenerate the casadi functions.');
                %{
                    % search current anaconda version from https://repo.anaconda.com/archive/
                    wget https://repo.anaconda.com/archive/Anaconda3-2024.10-1-Linux-x86_64.sh
                    bash Anaconda3-2024.10-1-Linux-x86_64.sh
                    conda create -n mpc python=3.12
                    conda config --add channels conda-forge
                    conda config --set channel_priority strict
                    conda install pinocchio crocoddyl nodejs casadi numpy
                %}
                
                disp('If done, press any key to continue...');
                input('');

                % compile casadi functions to simulink s-functions and matlab functions
                compile_py_cfun_to_sfun;
                
                % regenerate paths (otherwise matlab casadi functions are not detected)
                restoredefaultpath;
                parameter_str = "parameters_7dof";
                simulink_main_model_name = 'sim_discrete_7dof';
                run('./utils/matlab_init_general/add_default_matlab_paths'); % because it is not on path per default
                get_robot_name; % set robot_name to active robot from simulink (or default value when simulink is closed)
                s_fun_path = ['./s_functions/', robot_name];

                % regenerate trajectory
                overwrite_offline_traj_forced_extern = true;
                %run(parameter_str);
                parameters_7dof;
                overwrite_offline_traj_forced_extern = false;

                % compile s_functions and generate sources for mpcs
                print_init_guess_cost_functions = true;
                plot_init_guess                 = false; % plot init guess
                plot_null_simu                  = false; % plot system simulation for x0 = 0, u0 = ID(x0)
                convert_maple_to_casadi         = false; % convert maple functions into casadi functions
                fullsimu                        = false; % make full mpc simulation and plot results
                traj_select_mpc                 = 1; % (1: equilibrium, 2: 5th order diff filt, 3: 5th order poly, 4: smooth sinus)
                create_init_guess_for_all_traj  = true; % create init guess for all trajectories
                create_test_solve               = true; % create init guess for all trajectories
                compile_sfun                    = true; % needed for simulink s-function, filename: "s_function_"+casadi_func_name
                compile_matlab_sfunction        = false; % only needed for matlab MPC simu, filename: "casadi_func_name
                iterate_all_mpc_sfunctions      = true;
                mpc_source_selection            = 4; % (1: all MPCs, 2: only dynamic MPCs, 3: only kinematic MPCs, 4: only selected MPC)
                coptimflags                     = '-Ofast -march=native -flto'; % Optimization flag for compilation
                use_jit                         = false; % use jit for compilation (precompiles before each RUN!!!
                generate_realtime_udp_c_fun     = true; % create SOURCEFILES
                reload_parameters_m             = true; % reload parameters.m at the end (clears all variables!)
                remove_sourcefiles              = false; % remove source files after compilation
                
                use_extern_flags = true;
                mpc_casadi_main;
                use_extern_flags = false;

                clear('dont_clear');

                reset_started_flag = false;
            end
        catch
            fprintf(2, 'Failed to reset. This happens sometimes, because matlab has problems with paths. running ''restoredefaultpath'' should do it. Please simply try it again.\n');
            disp('Do you want to run it again? (y/n)');
            user_response = input('','s');
            if(strcmp(user_response, 'y'))
                clc;
                clear('dont_clear');
                restoredefaultpath;
                run('./main_matlab/parameters_7dof');
            end
        end
    end

end

function removeDir(dirPath)
    for attempt = 1:5
        try
            rmpath(dirPath);
            rmdir(dirPath, 's');
            return;  % Success, exit the function
        catch
            pause(1);  % Wait for 1 second before retrying
        end
    end
    fprintf(2, 'Failed to remove directory after 5 attempts. This happens sometimes, because matlab has problems with paths. running ''restoredefaultpath'' should do it. Please simply try it again.\n');
    disp('Do you want to run it again? (y/n)');
    user_response = input('','s');
    if(strcmp(user_response, 'y'))
        clc;
        clear('dont_clear');
        restoredefaultpath;
        run('./main_matlab/parameters_7dof');
    end
end