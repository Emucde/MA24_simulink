load('./utils/matlab_init_general/old_robot_name.mat');

if(~strcmp(robot_name, robot_name_old))
    % delete old robot files from path (otherwise conflicts due to same file names)
    rmpath(genpath(['./s_functions/', robot_name_old]));
    robot_name_old = robot_name;
    save('./utils/matlab_init_general/old_robot_name.mat', 'robot_name_old');
end

