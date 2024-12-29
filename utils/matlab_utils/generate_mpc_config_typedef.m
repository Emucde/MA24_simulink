function generate_mpc_config_typedef(filename, structName)
    % Default filename if not provided
    if nargin < 1 || isempty(filename)
        filename = 'config.h';
    end
    
    % Default struct name if not provided
    if nargin < 2 || isempty(structName)
        structName = 'ConfigStruct';
    end

    % Open the file for writing
    fid = fopen(filename, 'w');

    % Write the header guard
    fprintf(fid, '#ifndef %s_H\n', upper(structName));
    fprintf(fid, '#define %s_H\n\n', upper(structName));

    % Write necessary includes
    fprintf(fid, '#include <stdint.h>\n\n');

    % Write the struct definition with typedef
    fprintf(fid, 'typedef struct {\n');
    fprintf(fid, '    const char* x0_init_path;\n');
    fprintf(fid, '    const char* init_guess_path;\n');
    fprintf(fid, '    const char* traj_data_path;\n');
    fprintf(fid, '    const uint32_t traj_data_per_horizon;\n');
    fprintf(fid, '    uint32_t y_d_len;\n');
    fprintf(fid, '    uint32_t init_guess_len;\n');
    fprintf(fid, '    uint32_t x_k_addr;\n');
    fprintf(fid, '    uint32_t y_d_addr;\n');
    fprintf(fid, '    uint32_t in_init_guess_addr;\n');
    fprintf(fid, '    uint32_t in_param_weight_addr;\n');
    fprintf(fid, '    const casadi_real* param_weight;\n');
    fprintf(fid, '} %s_t;\n\n', structName);

    % Close the header guard
    fprintf(fid, '#endif // %s_H\n', upper(structName));

    % Close the file
    fclose(fid);

    fprintf('Header file "%s" has been created successfully.\n', filename);
end