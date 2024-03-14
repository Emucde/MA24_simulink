function replace_strings_in_casadi_file(file_path, replacement_string)
    % Open the file for reading
    fileID = fopen(file_path, 'r');

    if(fileID == -1)
        error(['file= "', file_path, '", casadi_func_name = "', replacement_string, '"']);
    end
    
    % Read the file contents
    file_contents = fscanf(fileID, '%c');
    
    % Close the file
    fclose(fileID);

    replacement_string_old = replacement_string;
    replacement_string = strcat(replacement_string, '_');
    
    file_contents = strrep(file_contents, 's_function', strcat('s_function_', replacement_string(1:end-1)));
    file_contents = strrep(file_contents, 'f_n_in', strrep('f_n_in', 'f_', replacement_string));
    file_contents = strrep(file_contents, 'f_n_out', strrep('f_n_out', 'f_', replacement_string));
    file_contents = strrep(file_contents, 'f_work', strrep('f_work', 'f_', replacement_string));
    file_contents = strrep(file_contents, 'f_checkout', strrep('f_checkout', 'f_', replacement_string));
    file_contents = strrep(file_contents, 'f_release', strrep('f_release', 'f_', replacement_string));
    file_contents = strrep(file_contents, 'f_sparsity_in', strrep('f_sparsity_in', 'f_', replacement_string));
    file_contents = strrep(file_contents, 'f_sparsity_out', strrep('f_sparsity_out', 'f_', replacement_string));
    file_contents = strrep(file_contents, 'f_incref', strrep('f_incref', 'f_', replacement_string));
    file_contents = strrep(file_contents, 'f_decref', strrep('f_decref', 'f_', replacement_string));
    file_contents = strrep(file_contents, 'f.h', strrep('f.h', 'f', replacement_string_old));
    file_contents = strrep(file_contents, 'f(arg,res,iw,w,mem)', strrep('f(arg,res,iw,w,mem)', 'f', replacement_string_old));

    % Open the file for writing
    fileID = fopen(file_path, 'w');
    
    % Write the modified contents to the file
    fprintf(fileID, '%c', file_contents);
    
    % Close the file
    fclose(fileID);
end