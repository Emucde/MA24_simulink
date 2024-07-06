%% Trajectory selection
traj_mode.equilibrium = 1;
traj_mode.differential_filter = 2;
traj_mode.polynomial = 3;
traj_mode.sinus = 4;

if(strcmp(robot_name, 'fr3_7dof'))
    param_traj_fr3_7dof;
elseif(strcmp(robot_name, 'fr3_6dof'))
    param_traj_fr3_6dof;
elseif(strcmp(robot_name, 'ur5e_6dof'))
    param_traj_ur5e_6dof;
else
    error('Only robot_name ( fr3_7dof | fr3_6dof | ur5e_6dof ) implemented!');
end

traj_struct_combined = combine_trajectories(traj_cell, param_global, param_robot);
param_traj = traj_struct_combined;