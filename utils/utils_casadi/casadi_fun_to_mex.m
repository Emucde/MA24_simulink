function mex_name = casadi_fun_to_mex(casadi_fun, source_dir, output_dir, mex_name, opt_flag)
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
        opt_flag = '-O3'
    end

    disp(['Compiling Matlab s-function ', casadi_fun.name, ' with ',opt_flag])
    
    full_name = [source_dir, '/', mex_name, '.c'];
    opts = struct('main', true, ...
        'mex', true);
    casadi_fun.generate(mex_name, opts);
    if ~strcmp(source_dir,pwd)
        movefile([mex_name,'.c'], source_dir);
    end
    mex(full_name, '-largeArrayDims', ['COPTIMFLAGS="',opt_flag,'"']);
    
    if ~strcmp(source_dir,pwd)
        movefile([mex_name,'.mexa64'], output_dir);
    end
    delete(full_name)
end