function create_only_source(casadi_fun, source_dir, casadi_fun_name)
    %CREATE_ONLY_SOURCE Creates only the source files for the S-function
    %
    %   This function creates the source files for the S-function without compiling them.
    %
    %   Inputs:
    %       casadi_fun - CasADi function to be compiled
    %       source_dir - Directory where the source files will be saved
    %       casadi_fun_name - Name of the CasADi function (default: 'fr3_ekf')
    
    arguments
        casadi_fun
        source_dir
        casadi_fun_name = 'fr3_ekf'
    end
    
    import casadi.*;
    
    % Create the source files
    cg_options = struct;
    cg_options.casadi_real = 'double';
    cg_options.real_min = num2str(eps);
    cg_options.casadi_int = 'int';
    cg_options.with_header = true;
    cg_options.main = false;
    cg = CodeGenerator(casadi_fun_name, cg_options);
    cg.add(casadi_fun);
    cg.generate();
    
    % move file to location
    movefile([casadi_fun_name, '.c'], source_dir);
    movefile([casadi_fun_name, '.h'], source_dir);
    
    calculate_simple_casadi_addresses(casadi_fun, casadi_fun_name, source_dir);
end