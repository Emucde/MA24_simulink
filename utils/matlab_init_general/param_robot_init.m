% Load robot parameters

if(strcmp(robot_name, 'fr3_7dof'))
    n = 7;
    m = 6;
    param_robot_fr3_init;
    param_robot = fr3.param;
elseif(strcmp(robot_name, 'fr3_6dof'))
    n = 6;
    m = 6;
    param_robot_fr3_init;
    param_robot = fr3.param;
elseif(strcmp(robot_name, 'ur5e_6dof'))
    n = 6;
    m = 6;
    param_robot_ur5e_init;
    param_robot = ur5e.param;
else
    error('Only robot_name ( fr3_7dof | fr3_6dof | ur5e_6dof ) implemented!');
end

n_indices = param_robot.n_indices;
n_indices_fixed = setdiff(1:n, n_indices);
disp(['Fixed joint: ', num2str(n_indices_fixed'), ', nDOF = ', num2str(n), ', see param_robot_init.m']);