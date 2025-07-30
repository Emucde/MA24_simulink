function save_init_guess_as_binary(init_guess_arr, file_name)
% save_init_guess_as_binary - Saves an initial guess trajectory as a binary file.
% Syntax:
%   save_init_guess_as_binary(init_guess_arr, file_name)
% Inputs:
%   init_guess_arr - A 2D array representing the initial guess trajectory.
%   file_name - The name of the file where the trajectory will be saved.
% Outputs:
%   A binary file containing the trajectory data. Used for C++ in ROS2.

    data_size = size(init_guess_arr);
    if(length(data_size) > 2)
        error(['init_guess_arr should be a 3d array: ', num2str(data_size)]);
    end

    if(data_size(1) < data_size(2)) % init_guess data should be in rows
        init_guess_arr = init_guess_arr';
        data_size = size(init_guess_arr);
    end

    % Open the binary file for writing
    fileID = fopen(file_name, 'w');

    rows = data_size(1);
    cols = data_size(2);
    
    % Write the dimensions of y_d to the file
    %rows_per_traj = 
    fwrite(fileID, [rows, cols], 'uint32');
    
    % Write the trajectory data to the file
    fwrite(fileID, init_guess_arr(:), 'double');
    
    % Close the file
    fclose(fileID);
end