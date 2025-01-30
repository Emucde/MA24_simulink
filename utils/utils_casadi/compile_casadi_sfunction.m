function compile_casadi_sfunction(casadi_fun, s_fun_path, output_dir, MPC_solver, opt_flag, mode, remove_sourcefiles, use_jit)
% COMPILE_CASADI_SFUNCTION Compiles a CasADi S-function for use in Simulink.
%
% Inputs:
%   casadi_fun      : CasADi function to be compiled.
%   s_fun_path      : Path of the s_function (can be './s_function_nlpsol.c' (nlpsol) or './s_function.c' (opti))
%   output_dir      : Directory where the compiled S-function will be saved.
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
%
% Warning: Method 2 does not support ipopt solver, only Method 1 (nlpsol)
arguments
    casadi_fun casadi.Function
    s_fun_path char
    output_dir char
    MPC_solver char
    opt_flag char = '-O2'
    mode (1,1) {mustBeInteger, mustBeInRange(mode, 1, 2)} = 1
    remove_sourcefiles logical = true
    use_jit logical = false
end


    import casadi.*;

    casadi_fun_name = casadi_fun.name;
    casadi_fun_c_header_name = [casadi_fun_name, '.c'];
    casadi_fun_h_header_name = [casadi_fun_name, '.h'];
    casadi_fun_c_header_path = [output_dir, '/', casadi_fun_c_header_name];
    casadi_fun_h_header_path = [output_dir, '/', casadi_fun_h_header_name];

    % Save the CasADi function (also in opti case because it is needed for
    % init guess calculations)
    casadi_fun.save([output_dir, '/', casadi_fun_name, '.casadi']);

    default_sfun_path = './utils/simulink_utils/default_sfunction/';

    if(mode == 1) % classic nlpsol s_function
        s_fun_file     = [default_sfun_path, 's_function_nlpsol.c'];
        s_fun_name_new = ['s_function_nlpsol_', casadi_fun_name];
        s_fun_file_new = [output_dir, '/', s_fun_name_new, '.c'];

        copyfile(s_fun_file, s_fun_file_new, 'f');
        replace_strings_in_casadi_file(s_fun_file_new, ['nlpsol_', casadi_fun_name]);
        
        % Get paths to CasADi libraries and headers
        lib_path = GlobalOptions.getCasadiPath();
        inc_path = GlobalOptions.getCasadiIncludePath();
        
        % Compile the S-function
        disp("Compiling Simulink " + s_fun_file_new + " (nlpsol, solver="+MPC_solver+") ");
        mex(['-I' inc_path],['-L' lib_path],'-lcasadi', s_fun_file_new, '-largeArrayDims', ['COPTIMFLAGS="',opt_flag,'"'], '-outdir', output_dir);
        %file_name = 'f_opt.casadi';
        
        if(remove_sourcefiles)
            delete(s_fun_file_new);
        end

        pathname = output_dir;
        fprintf('\n');
        disp(['S-function name: ', s_fun_name_new]);
        disp("S-function parameters:'" + pathname + "/" + casadi_fun_name + ".casadi', '" + casadi_fun_name + "'");
        disp("S-function modules:" + "'" + casadi_fun_name + "'");
        fprintf('\n');
    elseif(mode == 2) % modified combination of [2] and [3]
        s_fun_file = [default_sfun_path, 's_function_opti.c']; % TODO: create file if not exists!
        s_fun_name_new = ['s_function_opti_', casadi_fun_name];
        s_fun_file_new = [output_dir, '/', s_fun_name_new, '.c']; % TODO: create file if not exists!

        % Enable Mex Compile
        opts = struct('main', true, ...
            'mex', true);
        casadi_fun.generate(casadi_fun_name, opts); %TODO: wird das nicht ohnehin von cg.generate() ueberschrieben?
        % Ja es wird vom unteren cg.generate() überschrieben, es ist aber spannend sich diese files anzusehen,
        % da hier das main vorhanden ist, vgl. "calc_upd_cfun_addresses.m2

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
        
        copyfile(s_fun_file, s_fun_file_new, 'f');
        replace_strings_in_casadi_file(s_fun_file_new, casadi_fun_name);
        replace_s_function_string(s_fun_file_new, ['s_function_', casadi_fun_name], ['s_function_opti_', casadi_fun_name])

        if(use_jit)
            jit_library_path = ['-L', pwd, output_dir(2:end), '/mpc_c_sourcefiles'];
            jit_library = ['-l', casadi_fun_name, '_jit'];
        else
            jit_library_path = '';
            jit_library = '';
        end
        
        %mex(s_func_matlab_file, casadi_fun_c_header_path, '-outdir', s_fun_path);
        disp(['Compiling s-function ', s_fun_file_new, ' with ', opt_flag, ' (nlpsol-opti method)'])

        if((strcmp(MPC_solver, 'ipopt')))
            % for codegenerating ipopt install
            %{
            sudo apt-get install gcc g++ gfortran git patch wget pkg-config
            sudo apt-get install cmake libeigen3-dev
            sudo apt-get install liblapack-dev libmetis-dev libblas-dev
            git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
            cd ThirdParty-Mumps/
            ./get.Mumps
            ./configure
            make
            sudo make install
            git clone https://github.com/coin-or/Ipopt.git
            cd Ipopt/
            ./configure 
            make
            make test # ignore the errors, if it solves it works
            sudo make install
            % PS: it does not run in simulink, crashes instant.
            %}
            mex(s_fun_file_new, casadi_fun_c_header_path, '-lipopt', '-largeArrayDims', jit_library_path, jit_library, ['COPTIMFLAGS="',opt_flag,'"'], '-outdir', output_dir)
        elseif((strcmp(MPC_solver, 'fatrop')))
            % for codegenerating fatrop install (not tested in simulink but might not work)
            %{
            # Dependency: blasfeo
            git clone https://github.com/giaf/blasfeo.git
            cd blasfeo
            make shared_library -j $(nproc)
            sudo make install_shared

            git clone https://github.com/meco-group/fatrop.git --recursive
            cd fatrop
            export CMAKE_ARGS="-DBLASFEO_TARGET=X64_AUTOMATIC -DENABLE_MULTITHREADING=OFF"
            mkdir build
            cd build/
            cmake -DBLASFEO_TARGET=X64_AUTOMATIC ..
            make -j
            sudo make install
            %}
            mex(s_fun_file_new, casadi_fun_c_header_path, '-I/opt/blasfeo/include/', '-lblasfeo', '-lfatrop', jit_library_path, jit_library, '-largeArrayDims', ['COPTIMFLAGS="',opt_flag,'"'], '-outdir', output_dir)
        else
            mex(s_fun_file_new, casadi_fun_c_header_path, '-largeArrayDims', jit_library_path, jit_library, ['COPTIMFLAGS="',opt_flag,'"'], '-outdir', output_dir)
        end

        if(remove_sourcefiles)
            delete(s_fun_file_new);
            delete(casadi_fun_c_header_path);
            delete(casadi_fun_h_header_path);
            if(exist([output_dir, '/', casadi_fun_name, '.casadi'], 'file') == 2)
                delete([output_dir, '/', casadi_fun_name, '.casadi']);
            end
        end
        
        % display s-function in simulink settings
        fprintf('\n');
        disp(['S-function name: ', s_fun_name_new]);
        disp("S-function parameters:");
        disp("S-function modules:" + "'" + casadi_fun_name + "'");
        fprintf('\n');
    else
        error(['Error! mode shoud bei 1 (nlpsol fast compile - slow exec) or 2 (nlpsol opti slow compile - fast exec. mode is ', sprintf("%d", mode)]);
    end
end