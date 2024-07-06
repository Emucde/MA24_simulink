function solve_mpc_notfound_bug(simulink_main_model_name, mode)
arguments
    simulink_main_model_name char = 'sim_discrete_7dof'
    mode char = 'both' % 'both', 'comment', 'uncomment'
end

% Trick:
% 1. Comment all MPCs out
% 2. Reload all MPCs
% 3. Compile Simulink
% 4. Comment all MPCs in
% Bug should be fixed.

if(bdIsLoaded(simulink_main_model_name))
    % reload MPC's if they cannot be found:
    % 1. get list of all blocks in controller subsystem
    controller_blocklist = get_param('sim_discrete_7dof/Simulation models/Controller Subsystem/', 'Blocks');
    
    % get Names of MPC subsystems
    mpc_subsys_list = controller_blocklist(cellfun(@(x) contains(x, 'MPC'), controller_blocklist));
    
    if(strcmp(mode, 'comment') || strcmp(mode, 'both'))
            
        % reload all MPCs:
        for i=1:length(mpc_subsys_list)
            mpc_sfun_string = [simulink_main_model_name, '/Simulation models/Controller Subsystem/', mpc_subsys_list{i}, '/S-Function'];
            sfun_mpc_modules = get_param(mpc_sfun_string, 'SFunctionModules');

            % unsetting the module enforces reloading of the mpc!
            get_param(mpc_sfun_string, 'SFunctionModules');
            set_param(mpc_sfun_string, 'SFunctionModules', '''''');
            
            % Setting the same parameter again enforces loading of the mpc!
            set_param(mpc_sfun_string, 'SFunctionModules', sfun_mpc_modules);

            % Comment MPC out
            set_param(mpc_sfun_string, 'Commented', 'on');
        end

        % save system
        save_system(simulink_main_model_name, 'SaveDirtyReferencedModels','on');

        % close all subsystems
        closeAllSimulinkModels('.', simulink_main_model_name);

        % save system
        save_system(simulink_main_model_name, 'SaveDirtyReferencedModels','on');
    end

    if(strcmp(mode, 'both'))
        % compile simulink
        set_param(simulink_main_model_name, 'SimulationCommand', 'update');

        % save system
        save_system(simulink_main_model_name, 'SaveDirtyReferencedModels','on');
    end

    if(strcmp(mode, 'uncomment') || strcmp(mode, 'both'))
        % Due to the init scripts all variables defined here were deleted!

        % get all blocks
        controller_blocklist = get_param('sim_discrete_7dof/Simulation models/Controller Subsystem/', 'Blocks');
        
        % get Names of MPC subsystems
        mpc_subsys_list = controller_blocklist(cellfun(@(x) contains(x, 'MPC'), controller_blocklist));

        % uncomment all MPCs
        for i=1:length(mpc_subsys_list)
            mpc_sfun_string = [simulink_main_model_name, '/Simulation models/Controller Subsystem/', mpc_subsys_list{i}, '/S-Function'];

            % Comment MPC in
            set_param(mpc_sfun_string, 'Commented', 'off');
        end

        % save system
        save_system(simulink_main_model_name, 'SaveDirtyReferencedModels','on');
    end
end