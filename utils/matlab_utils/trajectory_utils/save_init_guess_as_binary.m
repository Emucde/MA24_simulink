function save_init_guess_as_binary(init_guess_arr, file_name)
    % init guess should be a 2d array!

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