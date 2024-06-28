load('./utils/matlab_init_general/old_robot_name.mat');

if(~strcmp(robot_name, robot_name_old))
    warning('off', 'MATLAB:rmpath:DirNotFound')
    rmpath(genpath('./s_functions'));
    rmpath(genpath('./utils/simulink_utils'));
    closeAllSimulinkModels('./utils/simulink_utils');
    closeAllSimulinkModels('./s_functions')
    warning('on', 'MATLAB:rmpath:DirNotFound')
end

if(contains(robot_name, '6dof'))
    addpath(genpath('./utils/simulink_utils/utils_6dof/'));
elseif(contains(robot_name, '7dof'))
    addpath(genpath('./utils/simulink_utils/utils_7dof/'));
else
    error('Invalid robot name: %s', robot_name, 'Valid robot names: fr3_7dof, fr3_6dof, ur5e_6dof');
end

if(strcmp(robot_name, 'fr3_7dof') || strcmp(robot_name, 'fr3_6dof'))
    addpath(genpath('./utils/simulink_utils/fr3_visual/'));
elseif(strcmp(robot_name, 'ur5e_6dof'))
    addpath(genpath('./utils/simulink_utils/ur5e_visual/'));
else
    error('Only robot_name ( fr3_7dof | fr3_6dof | ur5e_6dof ) implemented!')
end

robot_name_old = robot_name;
save('./utils/matlab_init_general/old_robot_name.mat', 'robot_name_old');