function generate_mpc_param_realtime_udp_c_fun(param_weight, param_MPC, func_name, output_dir, s_fun_path)
    % Open the header file for writing
    param_weight_header_name = [func_name, '_param.h'];
    fid = fopen([output_dir, param_weight_header_name], 'w');

    % Write the header guard
    fprintf(fid, ['#ifndef ', func_name, '_PARAM_H\n']);
    fprintf(fid, ['#define ', func_name, '_PARAM_H\n\n']);

    % Include necessary headers
    fprintf(fid, '#include <stdint.h>\n');
    fprintf(fid, '#include <math.h>\n\n');

    % Add Inf define
    fprintf(fid, '#ifndef Inf\n');
    fprintf(fid, '#define Inf INFINITY\n');
    fprintf(fid, '#endif\n\n');

    % Add path to init_guess and trajectory
    param_MPC_traj_data_bin_file = ['.', s_fun_path, 'trajectory_data/param_traj_data.bin'];
    param_MPC_init_guess_bin_file = ['.', s_fun_path, 'initial_guess/param_', func_name,'_init_guess.bin'];
    param_MPC_x0_init_bin_file = ['.', s_fun_path, 'trajectory_data/param_x0_init.bin'];

    fprintf(fid, '#ifndef TRAJ_DATA_PATH\n');
    fprintf(fid, '#define TRAJ_DATA_PATH "%s"\n', param_MPC_traj_data_bin_file);
    fprintf(fid, '#endif\n\n');

    fprintf(fid, '#ifndef X0_INIT_PATH\n');
    fprintf(fid, '#define X0_INIT_PATH "%s"\n', param_MPC_x0_init_bin_file);
    fprintf(fid, '#endif\n\n');

    fprintf(fid, '#define %s_INIT_GUESS_PATH "%s"\n', func_name, param_MPC_init_guess_bin_file);

    fprintf(fid, '\n');

    % Add MPC Param

    fprintf(fid, '//MPC_SETTINGS:\n');

    % Get field names
    field_names = fieldnames(param_MPC);
    for i = 1:length(field_names)
        field = field_names{i};
        field_data = param_MPC.(field);
        if( isscalar(field_data) && (isnumeric(field_data) || islogical(field_data)) )
            fprintf(fid, '#define %s_%s %g\n', func_name, field, field_data);
        elseif(ischar(field_data)) % then it must be string
            fprintf(fid, '#define %s_%s "%s"\n', func_name, field, field_data);
        elseif(ismatrix(field_data))
            if(strcmp(field, 'traj_indices'))
                field_data = field_data-1;
                if(field_data(1) ~= 0)
                    error(['First index not 0: field_data could be corrupted: ', num2str(field_data)]);
                end
            end
            fprintf(fid, 'static const uint32_t %s_%s[] = {', func_name, field);
            fprintf(fid, '%d,', field_data(1:end-1));
            fprintf(fid, '%d};\n', field_data(end));
        else
            error(['field data is nether numeric, integer or matrix: ', field_data])
        end
    end

    fprintf(fid, '\n');
    
    fprintf(fid, '//MPC_WEIGHTS:\n');

    %{
    %% OLD CODE FOR parameter struct.
    % But for C it is better to have it as an 1D array.

    % Define the struct
    fprintf(fid, 'typedef struct {\n');
    
    % Get field names
    field_names = fieldnames(param_weight.(func_name));
    % Write struct members
    for i = 1:length(field_names)
        field = field_names{i};
        field_size = size(param_weight.(func_name).(field));
        total_elements = prod(field_size);
        
        if length(field_size) == 2 && field_size(2) == 1
            % It's a vector
            fprintf(fid, '    double %s[%d];\n', field, field_size(1));
        else
            % It's a matrix or higher dimensional array
            fprintf(fid, '    double %s[%d];\n', field, total_elements);
        end
    end
    
    fprintf(fid,  ['} ', func_name, '_ParamWeight_t;\n\n']);

    % Define the constant struct
    fprintf(fid, ['const ',func_name, '_ParamWeight_t %s_param_weight = {\n'], func_name);

    % Write the data for each field
    for i = 1:length(field_names)
        field = field_names{i};
        field_data = param_weight.(func_name).(field);
        field_size = size(field_data);
        
        fprintf(fid, '    .%s = {\n', field);
        
        if length(field_size) == 2 && all(field_size > 1)
            % It's a matrix
            fprintf(fid, '        /* %dx%d matrix values */\n', field_size(1), field_size(2));
            for row = 1:field_size(1)
                fprintf(fid, '        ');
                fprintf(fid, '%g, ', field_data(row, :));
                fprintf(fid, '\n');
            end
        else
            % It's a vector or higher dimensional array
            fprintf(fid, '        /* %s array values */\n', mat2str(field_size));
            fprintf(fid, '        ');
            fprintf(fid, '%g, ', field_data(:)');
            fprintf(fid, '\n');
        end
        
        fprintf(fid, '    },\n');
    end

    fprintf(fid, '};\n');
    %}

    % Get field names
    field_names = fieldnames(param_weight.(func_name));
    
    % Calculate total size of the array
    total_size = 0;
    for i = 1:length(field_names)
        field = field_names{i};
        total_size = total_size + numel(param_weight.(func_name).(field));
    end
    
    % Start writing the array
    fprintf(fid, 'const casadi_real %s_param_weight[%d] = {\n', func_name, total_size);
    
    % Write the data for each field
    for i = 1:length(field_names)
        field = field_names{i};
        field_data = param_weight.(func_name).(field);
        field_size = size(field_data);
        
        % Add comment to indicate the start of a new field
        fprintf(fid, '    /* %s : ', field);
        
        if length(field_size) == 2 && all(field_size > 1)
            % It's a matrix
            fprintf(fid, '%dx%d matrix values */\n', field_size(1), field_size(2));
            for row = 1:field_size(1)
                fprintf(fid, '    ');
                fprintf(fid, '%g, ', field_data(row, :));
                fprintf(fid, '\n');
            end
        else
            % It's a vector or higher dimensional array
            fprintf(fid, '%s array values */\n', mat2str(field_size));
            fprintf(fid, '    ');
            fprintf(fid, '%g, ', field_data(:)');
            fprintf(fid, '\n');
        end
        
        % Add a blank line between fields for better readability
        if i < length(field_names)
            fprintf(fid, '\n');
        end
    end
    
    % Close the array
    fprintf(fid, '};\n');

    fprintf(fid, '\n\n#endif');

    % Close the file
    fclose(fid);

    disp(['Header file ', param_weight_header_name, ' created.']);
end