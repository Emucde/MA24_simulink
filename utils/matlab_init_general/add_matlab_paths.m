%restoredefaultpath

% Default robot independent paths
folder_add_to_path = {
    '../../main_matlab'
    '../../utils/matlab_utils'
    '../../utils/matlab_init_general'
    '../../utils/utils_casadi'
    '../../maple/maple_generated/fr3_7dof'  % yes this too, because I need T matrix from it for trajectory generation
    '../../urdf_creation'
    '../../main_simulink'
};

% Add to path 1/2
for i = 1:length(folder_add_to_path)
    if ~contains(path, folder_add_to_path{i})
        addpath(genpath(folder_add_to_path{i}));
    end
end

robot_folder_cell = cell(2,1);

try
    load('../../utils/matlab_init_general/old_robot_name.mat');
catch
    robot_name_old = '';
end

if(~strcmp(robot_name, robot_name_old))
    warning('off', 'MATLAB:rmpath:DirNotFound')
    rmpath(genpath('../../s_functions'));
    rmpath(genpath('../../utils/simulink_utils'));
    closeAllSimulinkModels('../../utils/simulink_utils');
    closeAllSimulinkModels('../../s_functions') % closeAllSimulinkModels('./s_functions')
    warning('on', 'MATLAB:rmpath:DirNotFound')
end

if(contains(robot_name, '6dof'))
    robot_folder_cell(1) = {'../..//utils/simulink_utils/utils_6dof/'};
elseif(contains(robot_name, '7dof'))
    robot_folder_cell(1) = {'../..//utils/simulink_utils/utils_7dof/'};
else
    error('Invalid robot name: %s', robot_name, 'Valid robot names: fr3_7dof, fr3_6dof, ur5e_6dof');
end

if(strcmp(robot_name, 'fr3_7dof') || strcmp(robot_name, 'fr3_6dof'))
    robot_folder_cell(2) = {'../..//utils/simulink_utils/fr3_visual/'};
elseif(strcmp(robot_name, 'ur5e_6dof'))
    robot_folder_cell(2) = {'../..//utils/simulink_utils/ur5e_visual/'};
else
    error('Only robot_name ( fr3_7dof | fr3_6dof | ur5e_6dof ) implemented!')
end

folder_add_to_path = [['../../', s_fun_path]; robot_folder_cell];

% Add to path 2/2
for i = 1:length(folder_add_to_path)
    if ~contains(path, folder_add_to_path{i})
        addpath(genpath(folder_add_to_path{i}));
    end
end

robot_name_old = robot_name;
save('../../utils/matlab_init_general/old_robot_name.mat', 'robot_name_old');