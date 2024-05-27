% Alternative: Standalone
% if(~isdeployed)
%     cd(fileparts(which(mfilename)));
% end

% init_casadi;
% import casadi.*;
% output_dir = './s_functions/s_functions_7dof/';

import casadi.*;

if(~exist('parameter_str', 'var'))
    parameters_7dof;
end

fun_arr = { ...
    'sys_fun_qpp_py', ...
    'sys_fun_x_py', ...
    'compute_tau_py', ...
    'robot_model_bus_fun_py', ...
    'inertia_matrix_py', ...
    'n_q_coriols_qp_plus_g_py', ...
    'gravitational_forces_py', ...
    'hom_transform_endeffector_py', ...
    'quat_endeffector_py', ...
    'geo_jacobian_endeffector_py', ...
    'geo_jacobian_endeffector_p_py' ...
};

% fun_arr = {'sys_fun_qpp_py'};

compile_mode = 2;
output_dir = './s_functions/s_functions_7dof/';

try

    for i = 1:length(fun_arr)
        fun_name = fun_arr{i};
        
        sys_fun = Function.load([output_dir, fun_name '.casadi']);
        if(compile_mode == 1)
            tic;
            s_fun_name = 's_function_nlpsol.c';
            compile_casadi_sfunction(sys_fun, s_fun_name, output_dir, fun_name, '-O3', compile_mode); % default nlpsol s-function
            disp(['Compile time for casadi s-function (nlpsol): ', num2str(toc), ' s']);
        elseif(compile_mode == 2)
            tic;
            s_fun_name = 's_function_opti.c';
            compile_casadi_sfunction(sys_fun, s_fun_name, output_dir, fun_name, '-O2', compile_mode); % default opti s-function
            disp(['Compile time for casadi s-function (opti for nlpsol): ', num2str(toc), ' s']);
        end
    end
catch ME
    disp('Error in compile_py_cfun_to_sfun.m')
    fprintf(2, 'Fehler: %s\n', getReport(ME));
end