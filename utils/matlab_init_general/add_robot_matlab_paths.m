% Add and remove robot dependent paths
robot_folder_cell = cell(2,1);

try
    load('./utils/matlab_init_general/old_robot_name.mat');
catch
    robot_name_old = '';
end

if(~strcmp(robot_name, robot_name_old))
    disp('Robot change detected, changing paths and closing all Simulink models...')
    warning('off', 'MATLAB:rmpath:DirNotFound')
    rmpath(genpath('./s_functions'));
    rmpath(genpath('./utils/simulink_utils'));
    % closeAllSimulinkModels('./utils/simulink_utils');
    % closeAllSimulinkModels('./s_functions') % closeAllSimulinkModels('./s_functions')

    % close all scripts due to path changes
    closeAllSimulinkModels('.');
    warning('on', 'MATLAB:rmpath:DirNotFound')

    if(bdIsLoaded(simulink_main_model_name))
        % reload MPC's if they cannot be found:
        % 1. get list of all blocks in controller subsystem
        controller_blocklist = get_param('sim_discrete_7dof/Simulation models/Controller Subsystem/', 'Blocks');
        
        % get Names of MPC subsystems
        mpc_subsys_list = controller_blocklist(cellfun(@(x) contains(x, 'MPC'), controller_blocklist));
        
        % reload all MPCs:
        for i=1:length(mpc_subsys_list)
            mpc_sfun_string = [simulink_main_model_name, '/Simulation models/Controller Subsystem/', mpc_subsys_list{i}, '/S-Function'];
            sfun_mpc_parameters = get_param(mpc_sfun_string, 'Parameters');
            % Setting the same parameter again enforces loading of the mpc!
            set_param(mpc_sfun_string, 'Parameters', sfun_mpc_parameters);
        end
    end
end

if(contains(robot_name, '6dof'))
    robot_folder_cell(1) = {'utils/simulink_utils/utils_6dof'};
elseif(contains(robot_name, '7dof'))
    robot_folder_cell(1) = {'utils/simulink_utils/utils_7dof'};
else
    error('Invalid robot name: %s', robot_name, 'Valid robot names: fr3_7dof, fr3_6dof, ur5e_6dof');
end

if(strcmp(robot_name, 'fr3_7dof') || strcmp(robot_name, 'fr3_6dof'))
    robot_folder_cell(2) = {'utils/simulink_utils/fr3_visual'};
elseif(strcmp(robot_name, 'ur5e_6dof'))
    robot_folder_cell(2) = {'utils/simulink_utils/ur5e_visual'};
else
    error('Only robot_name ( fr3_7dof | fr3_6dof | ur5e_6dof ) implemented!')
end

folder_add_to_path = [s_fun_path(3:end); robot_folder_cell];

% Add to path 2/2
for i = 1:length(folder_add_to_path)
    if ~contains(path, folder_add_to_path{i})
        addpath(genpath(['./', folder_add_to_path{i}]));
    end
end

robot_name_old = robot_name;
save('./utils/matlab_init_general/old_robot_name.mat', 'robot_name_old');