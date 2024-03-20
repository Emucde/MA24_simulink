function replace_s_function_string(file_path, s_fun_name_old, s_fun_name_new)
    % Open the file for reading
    fileID = fopen(file_path, 'r');

    if(fileID == -1)
        error(['file= "', file_path, '", casadi_func_name = "', replacement_string, '"']);
    end
    
    % Read the file contents
    file_contents = fscanf(fileID, '%c');
    
    % Close the file
    fclose(fileID);
    
    file_contents = strrep(file_contents, s_fun_name_old, s_fun_name_new);

    % Open the file for writing
    fileID = fopen(file_path, 'w');
    
    % Write the modified contents to the file
    fprintf(fileID, '%c', file_contents);
    
    % Close the file
    fclose(fileID);
end