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
    % TODO after nDOF change:
    % Matlab:
    % 1. unfix or fix joints in fr3.urdf
    % 2. generate and recompile python Pincchio 3xx casadi functions,
    %    adapt q_0 for that in py files.
    % Simulink:
    % 3. Controller: comment in or uncomment nullspace control in CT Controller Subsystem
    % 4. Debug Subsystem: comment in or comment out code in "manipulability and collinearity" matlab function
    % 5. Debug Subsystem: update bus selector block
elseif(strcmp(robot_name, 'ur5e_6dof'))
    n = 6;
    m = 6;
    param_robot_ur5e_init;
    param_robot = ur5e.param;
else
    error('Only robot_name ( fr3_7dof | fr3_6dof | ur5e_6dof ) implemented!');
end