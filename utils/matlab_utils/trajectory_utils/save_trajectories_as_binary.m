function save_trajectories_as_binary(param_traj_data, param_traj, traj_file, q0_init_file)

    traj_size = size(param_traj_data.p_d);

    if(length(traj_size) < 3)
        traj_amount = 1;
    else
        traj_amount = traj_size(3);
    end

    yy_d = [param_traj_data.p_d; param_traj_data.q_d];
    yy_d_size = size(yy_d);

    y_d = reshape(yy_d, yy_d_size(1), yy_d_size(2)*traj_amount, 1);

    % Open the binary file for writing
    fileID = fopen(traj_file, 'w');

    rows = yy_d_size(1);
    cols = yy_d_size(2);
    
    % Write the dimensions of y_d to the file
    fwrite(fileID, [rows, cols, traj_amount], 'uint32');
    
    % Write the trajectory data to the file
    fwrite(fileID, y_d(:), 'double');
    
    % Close the file
    fclose(fileID);

    % create q0_init.bin file:
    fileID = fopen(q0_init_file, 'w');

    x0_arr = [param_traj.q_0; param_traj.q_0_p];
    x0_arr_size = size(x0_arr);

    rows = x0_arr_size(1);
    cols = x0_arr_size(2);

    fwrite(fileID, [rows, cols], 'uint32');

    fwrite(fileID, x0_arr(:), 'double');
    
    % Close the file
    fclose(fileID);
end