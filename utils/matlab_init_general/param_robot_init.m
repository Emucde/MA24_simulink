% Load robot parameters

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

disp(['Fixed joint: ', num2str(n_indices_fixed), ', nDOF = ', num2str(n), ', see param_robot_init.m']);