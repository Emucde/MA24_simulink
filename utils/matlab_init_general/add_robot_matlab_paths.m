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
    rmpath(genpath('./utils/simulink_utils/utils_6dof'));
    rmpath(genpath('./utils/simulink_utils/utils_7dof'));
    rmpath(genpath('./utils/simulink_utils/fr3_visual'));
    rmpath(genpath('./utils/simulink_utils/fr3_no_hand_visual'));
    rmpath(genpath('./utils/simulink_utils/ur5e_visual'));
    % closeAllSimulinkModels('./utils/simulink_utils');
    % closeAllSimulinkModels('./s_functions') % closeAllSimulinkModels('./s_functions')

    % close all scripts due to path changes
    closeAllSimulinkModels('.', simulink_main_model_name);

    % change s function paths
    rename_s_functions_parameter;

    warning('on', 'MATLAB:rmpath:DirNotFound')
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
elseif(strcmp(robot_name, 'fr3_no_hand_6dof'))
    robot_folder_cell(2) = {'utils/simulink_utils/fr3_no_hand_visual'};
elseif(strcmp(robot_name, 'ur5e_6dof'))
    robot_folder_cell(2) = {'utils/simulink_utils/ur5e_visual'};
else
    error('Only robot_name ( fr3_7dof | fr3_6dof | fr3_no_hand_6dof | ur5e_6dof ) implemented!')
end

folder_add_to_path = [s_fun_path(3:end); robot_folder_cell];

% Add to path 2/2
for i = 1:length(folder_add_to_path)
    if ~contains(path, folder_add_to_path{i})
        addpath(genpath(['./', folder_add_to_path{i}]));
    end
end

if(~strcmp(robot_name, robot_name_old))
    robot_name_old = robot_name;
    save('./utils/matlab_init_general/old_robot_name.mat', 'robot_name_old');
    disp('Paths changed successfully!')
end