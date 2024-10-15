% Load robot parameters
% Don't forget to recompile all MPC's if you make changes here!

if(strcmp(robot_name, 'fr3_7dof'))
    n = 7;
    m = 6;

    param_robot_fr3_init;

    n_red = 7;
    fr3.param.n_red = n_red;

    param_robot = fr3.param;
elseif(strcmp(robot_name, 'fr3_6dof') || strcmp(robot_name, 'fr3_no_hand_6dof'))
    n = 7; % 7dof model is used for all calculations with fixed joint 3 value (should be zero but must not)
    m = 6;

    param_robot_fr3_init;

    % it is possible to control more pose dimensions than the robot has joints - in scope of an MPC

    fr3.param.yt_indices = [1 3]; % default [1 2 3]: use x, y and z position
    fr3.param.yr_indices = []; % default [1 2 3] use x, y, z error of quaternion

    fr3.param.n_indices_fixed = [1 3 5 6 7]; % joint 3 fixed to zero
    % fr3.param.n_indices_fixed = 3; % joint 3 fixed to zero
    fr3.param.n_indices = setdiff(1:n, fr3.param.n_indices_fixed); % joint 3 fixed to zero
    
    n_red = length(fr3.param.n_indices);
    fr3.param.n_red = n_red;

    param_robot = fr3.param;
    disp('Hint: All model calculations are done with the 7dof model, the 6dof mode is ensured due to a jointspace controller fixing joint 3 to zero.');
elseif(strcmp(robot_name, 'ur5e_6dof'))
    n = 6;
    m = 6;

    param_robot_ur5e_init;

    n_red = 6;
    ur5e.param.n_red = n_red;

    param_robot = ur5e.param;
else
    error('Only robot_name ( fr3_7dof | fr3_6dof | fr3_no_hand_6dof | ur5e_6dof ) implemented!');
end

n_indices = param_robot.n_indices;
n_indices_fixed = setdiff(1:n, n_indices);

if(max(n_indices_fixed) > n)
    error('Fixed joint index exceeds number of joints!');
elseif(min(n_indices_fixed) < 1)
    error('Fixed joint index is smaller than 1!');
elseif(min(param_robot.yt_indices) < 1)
    error('yt_indices is smaller than 1! [1 2 3] means use x, y and z (default value)!');
elseif(max(param_robot.yt_indices) > 3)
    error('yt_indices exceeds 3! [1 2 3] means use x, y and z (default value)!');
elseif(min(param_robot.yr_indices) < 1)
    error('yr_indices is smaller than 1! [1 2 3] means use x, y and z quaternion (default value)!');
elseif(max(param_robot.yr_indices) > 3)
    error('yr_indices exceeds 3! [1 2 3] means use x, y and z quaternion (default value)!');
end

disp(['Fixed joint: ', num2str(n_indices_fixed), ', nDOF = ', num2str(n), ', see param_robot_init.m']);