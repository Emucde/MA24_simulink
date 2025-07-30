% init_MPC_weights.m
% Initialize MPC weights based on the robot name.
if(strcmp(robot_name, 'fr3_7dof'))
    init_mpc_weights_fr3_7dof;
elseif(strcmp(robot_name, 'fr3_6dof'))
    init_mpc_weights_fr3_6dof;
elseif(strcmp(robot_name, 'fr3_no_hand_6dof'))
    [param_weight_init, param_weight] = init_mpc_weights_fr3_no_hand_6dof();
elseif(strcmp(robot_name, 'ur5e_6dof'))
    init_mpc_weights_ur5e_6dof;
else
    error('Only robot_name ( fr3_7dof | fr3_6dof | fr3_no_hand_6dof | ur5e_6dof ) implemented!');
end