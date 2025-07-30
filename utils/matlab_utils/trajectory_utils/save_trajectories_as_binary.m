function save_trajectories_as_binary(param_traj_data, param_traj, traj_file, q0_init_file)
% save_trajectories_as_binary - Saves trajectory data and initial conditions as binary files.
% Syntax:
%   save_trajectories_as_binary(param_traj_data, param_traj, traj_file, q0_init_file)
% Inputs:
%   param_traj_data - A structure containing trajectory data including positions, velocities, and orientations.
%   param_traj - A structure containing initial conditions for the trajectory.
%   traj_file - The name of the file where the trajectory data will be saved.
%   q0_init_file - The name of the file where the initial conditions will be saved.
% Outputs:
%   Two binary files containing the trajectory data and initial conditions. Used for C++ in ROS2.

    % Extract sizes of the trajectory data.
    traj_size = size(param_traj_data.p_d);
    
    if(length(traj_size) < 3)
        traj_amount = 1;
    else
        traj_amount = traj_size(3);
    end
    
    % Prepare the data structures for writing
    % Combine the trajectory data into a single array
    yy_d = [
        param_traj_data.p_d; 
        param_traj_data.p_d_p; 
        param_traj_data.p_d_pp; 
        param_traj_data.q_d; 
        param_traj_data.omega_d; 
        param_traj_data.omega_d_p;
        param_traj_data.Phi_d;
        param_traj_data.Phi_d_p;
        param_traj_data.Phi_d_pp;
    ];
    
    yy_d_size = size(yy_d);
    
    % Reshape the data as needed
    y_d = reshape(yy_d, yy_d_size(1), yy_d_size(2) * traj_amount, 1);
    
    % Open the binary file for writing
    fileID = fopen(traj_file, 'w');
    
    rows = yy_d_size(1);
    cols = yy_d_size(2);
    
    % Write the dimensions to the file
    fwrite(fileID, [rows, cols, traj_amount], 'uint32');
    
    % Write the trajectory data to the file
    fwrite(fileID, y_d(:), 'double');
    
    % Close the binary trajectory file
    fclose(fileID);

    % Create q0_init.bin file
    fileID = fopen(q0_init_file, 'w');

    x0_arr = [param_traj.q_0; param_traj.q_0_p];
    x0_arr_size = size(x0_arr);
    
    rows = x0_arr_size(1);
    cols = x0_arr_size(2);
    
    fwrite(fileID, [rows, cols], 'uint32');
    fwrite(fileID, x0_arr(:), 'double');
    
    % Close the q0_init.bin file
    fclose(fileID);
end