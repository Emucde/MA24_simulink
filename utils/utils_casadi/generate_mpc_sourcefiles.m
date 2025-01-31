function [files_changed_out] = generate_mpc_sourcefiles(casadi_fun, casadi_opt_problem_paths, current_mpc_mfile, s_fun_path, files_changed)
    % Generate C code (compare "compile_casadi_sfunction.m")
    % Disable Mex Compile and create only .h and .c files for including in main.c with udp communication (gcc)

    % All this changed checks are very important, because cmake use only timestamps to check if a file has changed

    mpc_sourcefile_path = [s_fun_path, '/mpc_c_sourcefiles/'];

    casadi_fun_name = casadi_fun.name;

    casadi_fun_c_header_name = [casadi_fun_name, '.c'];
    casadi_fun_h_header_name = [casadi_fun_name, '.h'];

    checksum_opt_prob_file_name = [casadi_opt_problem_paths, 'checksums.txt'];
    if(~exist(checksum_opt_prob_file_name, 'file'))
        calculate_checksums_with_md5sum(casadi_opt_problem_paths, checksum_opt_prob_file_name);
    end

    checksum_opt_prob_data = readtable(checksum_opt_prob_file_name, 'Delimiter', '  ', 'ReadVariableNames', true, 'FileType', 'text');
    % fileID = fopen(checksum_opt_prob_file_name, 'r');
    % checksum_opt_prob_data = textscan(fileID, '%s  %s', 'Delimiter', '  ');
    % checksum_opt_prob_data = reshape(checksum_opt_prob_data{1}(3:end), 2, [])';
    % checksum_opt_prob_data = struct2table(struct( ...
    %     'FileName', {checksum_opt_prob_data(:,2)}, ...
    %     'Checksum', {checksum_opt_prob_data(:,1)} ...
    % ));
    % fclose(fileID);
    current_casadi_func_c_file = [mpc_sourcefile_path, casadi_fun_c_header_name];
    current_casadi_func_h_file = [mpc_sourcefile_path, casadi_fun_h_header_name];
    exists_current_mpc_c_file = exist(current_casadi_func_c_file, 'file');
    exists_current_mpc_h_file = exist(current_casadi_func_h_file, 'file');

    [~, current_checksum] = linux_md5sum(current_mpc_mfile);
    index = contains(checksum_opt_prob_data.FileName, current_mpc_mfile);

    % Checksum for mpc_casadi_main.m change and nlp_solver_settings.m change
    checksum_settings_change_file = [s_fun_path, '/mpc_c_sourcefiles/checksum_settings.txt'];
    checksum_settings_filelist = {'./main_matlab/mpc_casadi_main.m', './utils/utils_casadi/nlp_solver_settings.m'};
    if(~exist(checksum_settings_change_file, 'file'))
        calculate_filelist_checksums_with_md5sum(checksum_settings_filelist, checksum_settings_change_file);
    end

    checksum_settings_data = readtable(checksum_settings_change_file, 'Delimiter', '  ', 'ReadVariableNames', true, 'FileType', 'text');
    
    if(any(files_changed))
        settings_change_flags = files_changed;
    else
        settings_change_flags = ~arrayfun(@(i) compare_checksum_table(checksum_settings_data, checksum_settings_filelist{i}), 1:length(checksum_settings_filelist));
    end

    
    settings_changed = any(settings_change_flags);



    if strcmp(checksum_opt_prob_data.Checksum{index}, current_checksum) && exists_current_mpc_c_file && exists_current_mpc_h_file && ~settings_changed
        fprintf('%s did not changed. Skipping creating casadi c sources.\n', current_mpc_mfile);
    else
        for i = 1:length(settings_change_flags)
            if settings_change_flags(i)
                fprintf(2, '%s changed. Recreating casadi c sources.\n', checksum_settings_filelist{i});
                files_changed(i) = true;
            end
        end
        cg_options = struct;
        cg_options.with_header = true;
        cg = casadi.CodeGenerator(casadi_fun_name, cg_options);
        cg.add(casadi_fun);
        cg.generate();

        if(exists_current_mpc_c_file)
            checksum_c_same = compare_checksums(casadi_fun_c_header_name, current_casadi_func_c_file);
        else
            checksum_c_same = false;
        end

        if(exists_current_mpc_h_file)
            checksum_h_same = compare_checksums(casadi_fun_h_header_name, current_casadi_func_h_file);
        else
            checksum_h_same = false;
        end

        checksum_same = checksum_c_same && checksum_h_same;
        nlp_solver_changed = settings_change_flags(2);

        if checksum_same && ~nlp_solver_changed
            delete(casadi_fun_c_header_name);
            delete(casadi_fun_h_header_name);
            fprintf(['Header and source files ', casadi_fun_h_header_name, ' and ', casadi_fun_c_header_name, ' did not change.\nSkipping creating casadi c sources.\n']);
        else
            if(~checksum_h_same || nlp_solver_changed)
                fprintf(2, ['Header file ', casadi_fun_h_header_name, ' changed.\n']);
                movefile(casadi_fun_h_header_name, mpc_sourcefile_path);
            else
                delete(casadi_fun_h_header_name);
            end
            if(~checksum_c_same || nlp_solver_changed)
                fprintf(2, ['Source file ', casadi_fun_c_header_name, ' changed.\n']);
                movefile(casadi_fun_c_header_name, mpc_sourcefile_path);
            else
                delete(casadi_fun_c_header_name);
            end
        end

        % Update checksums
        checksum_opt_prob_data.Checksum{index} = current_checksum;
        update_checksumfile(checksum_opt_prob_data, checksum_opt_prob_file_name);

        for i = 1:length(settings_change_flags)
            if settings_change_flags(i)
                [~, checksum_settings_data.Checksum{i}] = linux_md5sum(checksum_settings_filelist{i});
            end
        end
        update_checksumfile(checksum_settings_data, checksum_settings_change_file);
    end

    files_changed_out = files_changed;
end


function calculate_checksums_with_md5sum(folder_path, output_file)
    % Check if the output file already exists and remove it
    if exist(output_file, 'file')
        delete(output_file);
    end

    % Get a list of all files in the folder
    files = dir(fullfile(folder_path, '*.m'));  % Read all files
    files = files(~[files.isdir]);  % Filter out directories

    % Open the output file for writing
    fid = fopen(output_file, 'w');
    if fid == -1
        error('Cannot open output file: %s', output_file);
    end

    % create header
    fprintf(fid, 'Checksum  FileName\n');

    % Loop over all files and compute their checksums
    for i = 1:length(files)
        file_path = fullfile(folder_path, files(i).name);  % Full path to the file
        % Execute the md5sum command and capture the output
        [status, cmdout] = linux_md5sum(file_path, 'cmdout');

        if status == 0  % Command executed successfully
            % Write the output to the file
            fprintf(fid, '%s', cmdout);
        else
            fprintf(fid, 'Error processing file: %s\n', files(i).name);
        end
    end

    fclose(fid);  % Close the output file
end

function [status, checksum] = linux_md5sum(file_path, cmdout_str)
    arguments
        file_path (1, :) char
        cmdout_str (1, :) char = ''
    end
    % Execute the md5sum command and capture the output
    [status, cmdout] = system(['md5sum ', file_path]);

    if status ~= 0
        error('Error executing md5sum command: %s', cmdout);
    end
    if length(cmdout) < 32
        error('Error processing file: %s, cmdout=%s', file_path, cmdout);
    end
    if(strcmp(cmdout_str, 'cmdout'))
        checksum = cmdout;
    else
        checksum = cmdout(1:32);  % Extract the checksum from the output
    end
end

function update_checksumfile(checksumData, checksum_file_name)
    % Write the updated checksums to the output file
    writetable(checksumData, checksum_file_name, 'Delimiter', ' ', 'WriteVariableNames', true);

    fileContent = fileread(checksum_file_name);

    % Replace single spaces with double spaces (due to md5hash)
    modifiedContent = strrep(fileContent, ' ', '  ');

    % Write the modified content back to the file
    fid = fopen(checksum_file_name, 'w');
    fprintf(fid, '%s', modifiedContent);
    fclose(fid);
end

function calculate_filelist_checksums_with_md5sum(file_list, output_file)
    % Check if the output file already exists and remove it
    if exist(output_file, 'file')
        delete(output_file);
    end

    % Open the output file for writing
    fid = fopen(output_file, 'w');
    if fid == -1
        error('Cannot open output file: %s', output_file);
    end

    % create header
    fprintf(fid, 'Checksum  FileName\n');

    % Loop over all files and compute their checksums
    for i = 1:length(file_list)
        file_path = fullfile(file_list{i});  % Full path to the file
        % Execute the md5sum command and capture the output
        [status, cmdout] = linux_md5sum(file_path, 'cmdout');

        if status == 0  % Command executed successfully
            % Write the output to the file
            fprintf(fid, '%s', cmdout);
        else
            fprintf(fid, 'Error processing file: %s\n', file_list{i});
        end
    end

    fclose(fid);  % Close the output file
end

function [is_equal] = compare_checksum_table(checksumData, file_path)
    index = find(contains(checksumData.FileName, file_path));
    if index > 0
        [~, current_checksum] = linux_md5sum(file_path);
        is_equal = strcmp(checksumData.Checksum{index}, current_checksum);
    else
        is_equal = false;
    end
end

function [is_equal] = compare_checksums(file_path1, file_path2)
    [~, checksum1] = linux_md5sum(file_path1);
    [~, checksum2] = linux_md5sum(file_path2);
    is_equal = strcmp(checksum1, checksum2);
end