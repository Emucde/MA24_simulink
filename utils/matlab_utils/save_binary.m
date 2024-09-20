function save_binary(y_d, file_name)
% e.g. save_binary(init_guess_0, mpc8_init_guess.bin')
    % Open the binary file for writing
    fileID = fopen(file_name, 'w');
    
    % Write the dimensions of y_d to the file
    [rows, cols] = size(y_d);
    fwrite(fileID, [rows, cols], 'uint32');
    
    % Write the trajectory data to the file
    fwrite(fileID, y_d, 'double');
    
    % Close the file
    fclose(fileID);
end