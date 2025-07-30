function mex_name = casadi_fun_to_mex(casadi_fun, source_dir, output_dir, mex_name, opt_flag, solver_name, use_own_solver)
% casadi_fun_to_mex - Compile a CasADi function to a MATLAB mex file.
    % src: https://github.com/casadi/casadi/discussions/3337
    % Optimization options for MATLAB compilation
    
    % Level  | Execution Time             | Code Size | Memory Usage | Compile Time
    %--------|----------------------------|-----------|--------------|--------------
    % -O0    | Default (no opt) (slow)    |           |              |      (+0%   )
    % -O1    | Balanced (size/time)       | ++        | ++           | ++   (+340% )
    % -O2    | More emphasis (size/time)  | ++        | +++          | +++  (+540% )
    % -O3    | Most emphasis (size/time)  | ---       | +++          | ++++ (+1860%)
    % -Os    | Optimize for code size     | ++        | --           | ++   (+400% )
    % -Ofast | Similar to -O3 (fast math) | +++       | +++          | ++++ (+2180%)
    arguments
        casadi_fun (1,1) casadi.Function
        source_dir char
        output_dir char
        mex_name = casadi_fun.name;
        opt_flag = '-O3';
        solver_name = '';
        use_own_solver = false;
    end

    disp(['Compiling Matlab s-function ', casadi_fun.name, ' with ',opt_flag]);
    init_casadi;
    
    full_name = [source_dir, '/', mex_name, '.c'];
    opts = struct('main', true, ...
        'mex', true);
    casadi_fun.generate(mex_name, opts);
    if ~strcmp(source_dir,pwd)
        movefile([mex_name,'.c'], source_dir);
    end

    if((strcmp(solver_name, 'ipopt')))
        % fix bug
        % append to c file:
        % #ifndef CASADI_INFINITY
        % #define casadi_inf INFINITY
        % #endif

        file_content = fileread(full_name);
        % insert string at the begin of the file:
        insert_string = sprintf('#ifndef CASADI_INFINITY\n#define casadi_inf INFINITY\n#endif\n');
        file_content = [insert_string, file_content];
        fid = fopen(full_name, 'w');
        fwrite(fid, file_content);
        fclose(fid);     
        if(use_own_solver)
            % uncomment #export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/media/daten/Anwendungen/casadi-3.6.7-linux64-matlab2018b
            % add export LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6
            %   in ~/.bashrc
            ipopt_include_dirs='/usr/local/include/coin-or';
            ipopt_library_dirs='/usr/local/lib/';
            
            mex(full_name, ...
                ['LDFLAGS="-Wl,-rpath,', ipopt_library_dirs, ' -Wl,-rpath,',lib_path, '"'], ...  % Combine LDFLAGS
                ['-I' inc_path], ['-L' lib_path], '-lcasadi', ...
                ['-I' ipopt_include_dirs], ...
                ['-L' ipopt_library_dirs], '-lipopt', ...
                '-largeArrayDims', ...
                ['COPTIMFLAGS="', opt_flag, '"'], ...
                '-outdir', output_dir);
        else
            % mex(full_name, ...
            %     ['LDFLAGS="$LDFLAGS -Wl,-rpath,', lib_path, '"'], ...  % Combine LDFLAGS
            %     ['-I' inc_path], ['-L' lib_path], '-lcasadi', '-lipopt', ...
            %     '-largeArrayDims', ...
            %     ['COPTIMFLAGS="', opt_flag, '"'], ...
            %     '-outdir', output_dir);
            mex(full_name, ['-I' inc_path],['-L' lib_path],'-lcasadi', '-lipopt', '-largeArrayDims', ['COPTIMFLAGS="',opt_flag,'"'], '-outdir', output_dir);
        end
    elseif((strcmp(solver_name, 'fatrop')))
        if(use_own_solver)           
            % THIS ONLY WORKS IF CASADI IS NOT ON LD_LIBRARY_PATH !!!!! (restart matlab after setting LD_LIBRARY_PATH)
            % Otherwise -I and -L is ignored and the libraries from casadi are used!
            % uncomment #export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/media/daten/Anwendungen/casadi-3.6.7-linux64-matlab2018b
            % add export LD_PRELOAD=/lib/x86_64-linux-gnu/libstdc++.so.6
            %   in ~/.bashrc
            fatrop_include_dirs='/usr/local/include/fatrop/';
            fatrop_library_dirs='/usr/local/lib/';
            blasfeo_include_dirs='/opt/blasfeo/include/';
            blasfeo_library_dirs='/opt/blasfeo/lib/';
            
            mex(full_name, ...
                ['LDFLAGS="-Wl,-rpath,', fatrop_library_dirs, ' -Wl,-rpath,', blasfeo_library_dirs, '"'], ...  % Combine LDFLAGS
                ['-I' fatrop_include_dirs], ...
                ['-L' fatrop_library_dirs], [fatrop_library_dirs, 'libfatrop.so'], ...
                ['-I' blasfeo_include_dirs], ...
                ['-L' blasfeo_library_dirs], [blasfeo_library_dirs, 'libblasfeo.so'], ...
                '-largeArrayDims', ...
                ['COPTIMFLAGS="', opt_flag, '"'], ...
                '-outdir', output_dir);
        else
            mex(full_name, ['-I' inc_path],['-L' lib_path],'-lcasadi', '-lblasfeo', '-lfatrop', '-largeArrayDims', ['COPTIMFLAGS="',opt_flag,'"'], '-outdir', output_dir);
        end
    else
        mex(full_name, '-largeArrayDims', ['COPTIMFLAGS="',opt_flag,'"'], '-outdir', output_dir);
    end
    delete(full_name)