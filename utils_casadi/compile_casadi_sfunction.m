function compile_casadi_sfunction(casadi_fun, s_fun_name, output_dir, MPC_solver, opt_flag, mode)
% COMPILE_CASADI_SFUNCTION Compiles a CasADi S-function for use in Simulink.
%
% Inputs:
%   casadi_fun      : CasADi function to be compiled.
%   s_fun_name      : Name of the s_function (can be 's_function_nlpsol.c' (nlpsol) or 's_function.c' (opti))
%   MPC_solver      : Name of the MPC solver to be used (optional, for display only).
%   mode            : Compile Mode with different s_function file
%                   :   mode = 1: use default nlpsol s_function "s_function_nlpsol.c" [from [1]]
%                   :   mode = 2: use default opti s_function "s_function.c" from [2, 3]
%   opt_flag        : gcc compiler flag. Can be -O0, ..., -O0, -Os and -Ofast.
%                     See Table below
%
% Level  | Execution Time             | Code Size | Memory Usage | Compile Time
%--------|----------------------------|-----------|--------------|--------------
% -O0    | Default (no opt) (slow)    |           |              |      (+0%   )
% -O1    | Balanced (size/time)       | ++        | ++           | ++   (+340% )
% -O2    | More emphasis (size/time)  | ++        | +++          | +++  (+540% )
% -O3    | Most emphasis (size/time)  | ---       | +++          | ++++ (+1860%)
% -Os    | Optimize for code size     | ++        | --           | ++   (+400% )
% -Ofast | Similar to -O3 (fast math) | +++       | +++          | ++++ (+2180%)
%
% Dependencies:
%   - Assumes a custom function 'replace_strings_in_casadi_file' is available.
% 
% Sources:
% [1] https://web.casadi.org/blog/mpc-simulink2/
% [2] https://web.casadi.org/blog/s-function/
% [3] https://github.com/casadi/casadi/discussions/3337

    import casadi.*;

    casadi_fun_name = casadi_fun.name;
    casadi_fun_c_header_name = [casadi_fun_name, '.c'];
    casadi_fun_h_header_name = [casadi_fun_name, '.h'];
    casadi_fun_c_header_path = [output_dir, casadi_fun_c_header_name];
    casadi_fun_h_header_path = [output_dir, casadi_fun_h_header_name];

    if(mode == 1) % classic nlpsol s_function
        s_func_name_origin       = [output_dir, s_fun_name];
        s_func_name_new          = [s_fun_name(1:end-2), '_', casadi_fun_c_header_name]; % final name for Simulink s-function (.c)
        s_fun_c_file_path        = [output_dir, s_func_name_new];

        copyfile(s_func_name_origin, s_fun_c_file_path, 'f');
        replace_strings_in_casadi_file(s_fun_c_file_path, ['nlpsol_', casadi_fun_name]);
        
        % Save the CasADi function
        casadi_fun.save([output_dir, casadi_fun_name, '.casadi']);
        
        % Get paths to CasADi libraries and headers
        lib_path = GlobalOptions.getCasadiPath();
        inc_path = GlobalOptions.getCasadiIncludePath();
        
        % Compile the S-function
        disp("Compiling Simulink " + s_func_name_new + " (nlpsol, solver="+MPC_solver+") ");
        mex(['-I' inc_path],['-L' lib_path],'-lcasadi', s_fun_c_file_path, '-largeArrayDims', ['COPTIMFLAGS="',opt_flag,'"'], '-outdir', output_dir);
        %file_name = 'f_opt.casadi';
        
        delete(s_fun_c_file_path);

        pathname = split(output_dir, '/');
        pathname = pathname{2};
        fprintf('\n');
        disp(['S-function name:', s_fun_name(1:end-2), '_', casadi_fun_name]);
        disp("S-function parameters:'" + pathname + "/" + casadi_fun_name + ".casadi', '" + casadi_fun_name + "'");
        disp("S-function modules:" + "'" + casadi_fun_name + "'");
        fprintf('\n');
    elseif(mode == 2) % modified combination of [2] and [3]
        s_func_name_origin = [output_dir, s_fun_name];
        s_func_name_new = [output_dir, s_fun_name(1:end-2), '_', casadi_fun_c_header_name];  % final name for Simulink s-function (.c)
        
        opts = struct('main', true, ...
            'mex', true);
        casadi_fun.generate(casadi_fun_name, opts);
        
        cg_options = struct;
        cg_options.casadi_real = 'real_T';
        cg_options.real_min = num2str(eps);
        cg_options.casadi_int = 'int_T';
        cg_options.with_header = true;
        cg = CodeGenerator(casadi_fun_name, cg_options);
        cg.add_include('simstruc.h');
        cg.add(casadi_fun);
        cg.generate();
        
        movefile(casadi_fun_c_header_name, output_dir);
        movefile(casadi_fun_h_header_name, output_dir);
        
        copyfile(s_func_name_origin, s_func_name_new, 'f');
        replace_strings_in_casadi_file(s_func_name_new, casadi_fun_name);
        replace_s_function_string(s_func_name_new, ['s_function_', casadi_fun_name], ['s_function_opti_', casadi_fun_name])
        
        %mex(s_func_matlab_file, casadi_fun_c_header_path, '-outdir', s_fun_path);
        disp(['Compiling s-function ', s_func_name_new, ' with ', opt_flag, ' (nlpsol-opti method)'])
        mex(s_func_name_new, casadi_fun_c_header_path, '-largeArrayDims', ['COPTIMFLAGS="',opt_flag,'"'], '-outdir', output_dir)
        
        delete(s_func_name_new);
        delete(casadi_fun_c_header_path);
        delete(casadi_fun_h_header_path);
        %if(exist([output_dir, casadi_fun_name, '.casadi'], 'file') == 2)
        %    delete([output_dir, casadi_fun_name, '.casadi']);
        %end
        
        % display s-function in simulink settings
        fprintf('\n');
        disp(['S-function name:', s_fun_name(1:end-2), '_', casadi_fun_name]);
        disp("S-function parameters:");
        disp("S-function modules:" + "'" + casadi_fun_name + "'");
        fprintf('\n');
    else
        error(['Error! mode shoud bei 1 (nlpsol fast compile - slow exec) or 2 (nlpsol opti slow compile - fast exec. mode is ', sprintf("%d", mode)]);
    end
end