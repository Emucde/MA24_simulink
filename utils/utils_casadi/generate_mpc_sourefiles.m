function generate_mpc_sourefiles(casadi_fun, casadi_opt_problem_paths, current_mpc_mfile, mpc_c_sourcefile_path)
    % Generate C code (compare "compile_casadi_sfunction.m")
    % Disable Mex Compile and create only .h and .c files for including in main.c with udp communication (gcc)

    casadi_fun_name = casadi_fun.name;

    casadi_fun_c_header_name = [casadi_fun_name, '.c'];
    casadi_fun_h_header_name = [casadi_fun_name, '.h'];

    checksum_file_name = [casadi_opt_problem_paths, 'checksums.txt'];
    if(~exist(checksum_file_name, 'file'))
        calculate_checksums_with_md5sum(casadi_opt_problem_paths, checksum_file_name);
    end

    checksumData = readtable(checksum_file_name, 'Delimiter', '  ', 'ReadVariableNames', true);

    [~, cmdout] = linux_md5sum(current_mpc_mfile);
    current_checksum = cmdout(1:32); % always 32 hex chars

    index = contains(checksumData.FileName, current_mpc_mfile);
    
    if strcmp(checksumData.Checksum{index}, current_checksum)
        fprintf('%s did not changed. Skipping creating casadi c sources.\n', current_mpc_mfile);
    else
        fprintf('Checksum of %s changed. Regenerating header and source files.\n', current_mpc_mfile);
        cg_options = struct;
        cg_options.with_header = true;
        cg = casadi.CodeGenerator(casadi_fun_name, cg_options);
        cg.add(casadi_fun);
        cg.generate();

        % double check: change checksum of both source files
        [~, checksum_str_new] = linux_md5sum(casadi_fun_c_header_name);
        [~, checksum_str_old] = linux_md5sum([mpc_c_sourcefile_path, casadi_fun_c_header_name]);

        if strcmp(checksum_str_new(1:32), checksum_str_old(1:32))
            delete(casadi_fun_c_header_name);
            delete(casadi_fun_h_header_name);
            fprintf(['Header and source files ', casadi_fun_h_header_name, ' and ', casadi_fun_c_header_name, ' did not change.\nSkipping creating casadi c sources.\n']);
        else
            movefile(casadi_fun_c_header_name, mpc_c_sourcefile_path);
            movefile(casadi_fun_h_header_name, mpc_c_sourcefile_path);
            fprintf(['New Header and source files ', casadi_fun_h_header_name, ' and ', casadi_fun_c_header_name, ' created.\n']);
        end

        checksumData.Checksum{index} = current_checksum;
        update_checksumfile(checksumData, checksum_file_name);
    end
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
        [status, cmdout] = linux_md5sum(file_path);

        if status == 0  % Command executed successfully
            % Write the output to the file
            fprintf(fid, '%s', cmdout);
        else
            fprintf(fid, 'Error processing file: %s\n', files(i).name);
        end
    end

    fclose(fid);  % Close the output file
end

function [status, cmdout] = linux_md5sum(file_path)
    % Execute the md5sum command and capture the output
    [status, cmdout] = system(['md5sum ', file_path]);

    if status ~= 0
        error('Error executing md5sum command: %s', cmdout);
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