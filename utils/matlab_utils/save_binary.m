function save_binary(y_d)
    % Open the binary file for writing
    fileID = fopen('traj.bin', 'w');
    
    % Write the dimensions of y_d to the file
    [rows, cols] = size(y_d);
    fwrite(fileID, [rows, cols], 'uint32');
    
    % Write the trajectory data to the file
    fwrite(fileID, y_d, 'double');
    
    % Close the file
    fclose(fileID);
end