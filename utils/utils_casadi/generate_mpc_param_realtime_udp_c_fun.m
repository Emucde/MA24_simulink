function generate_mpc_param_realtime_udp_c_fun(param_weight, func_name, output_dir)
    % Get field names
    field_names = fieldnames(param_weight.(func_name));

    % Open the header file for writing
    param_weight_header_name = [func_name, '_param_weight.h'];
    fid = fopen([output_dir, param_weight_header_name], 'w');

    % Write the header guard
    fprintf(fid, ['#ifndef ', func_name, '_PARAM_WEIGHT_H\n']);
    fprintf(fid, ['#define ', func_name, '_PARAM_WEIGHT_H\n\n']);

    % Include necessary headers
    fprintf(fid, '#include <stdint.h>\n\n');

    % Add Inf define
    fprintf(fid, '#ifndef Inf\n');
    fprintf(fid, '#define Inf INFINITY\n');
    fprintf(fid, '#endif\n\n');

    % Define the struct
    fprintf(fid, 'typedef struct {\n');
    
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
    fprintf(fid, '\n\n#endif');

    % Close the file
    fclose(fid);

    disp(['Header file ', param_weight_header_name, ' created.']);
end