% Compiles casadi functions from python to s-functions for simulink

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

% valid robot_names: fr3_7dof, fr3_6dof, ur5e
% robot_name = 'fr3_7dof';
% robot_name = 'fr3_6dof';
robot_name = 'fr3_no_hand_6dof';
% robot_name = 'ur5e_6dof';

compile_mode = 2; % Compile mode: 1 - nlpsol, 2 - opti
output_dir = ['./s_functions/', robot_name, '/'];
input_dir = [output_dir, 'casadi_functions/'];

% s functions for matlab only use (not intrinsic functions)
fun_arr_matlab = { ...
    'sys_fun_qpp_aba_py', ...
    'sys_fun_qpp_sol_py', ...
    'sys_fun_x_aba_py', ...
    'sys_fun_x_sol_py', ...
    'compute_tau_py', ...
    'robot_model_bus_fun_py', ...
    'inertia_matrix_py', ...
    'n_q_coriols_qp_plus_g_py', ...
    'gravitational_forces_py', ...
    'hom_transform_endeffector_py', ...
    'quat_endeffector_py', ...
    'geo_jacobian_endeffector_py', ...
    'geo_jacobian_endeffector_p_py' ...
    'inverse_inertia_matrix_py', ...
};

% s functions for simulink
fun_arr_sfun = { ...
    'sys_fun_qpp_aba_py', ...
    'sys_fun_qpp_sol_py', ...
    'robot_model_bus_fun_py' ...
};

try

    for i = 1:length(fun_arr_sfun)
        fun_name = fun_arr_sfun{i};

        casadi_fun = Function.load([input_dir, fun_name '.casadi']);
        if(compile_mode == 1)
            tic;
            s_fun_name = 's_function_nlpsol.c';
            compile_casadi_sfunction(casadi_fun, s_fun_name, output_dir, fun_name, '-O3', compile_mode); % default nlpsol s-function
            disp(['Compile time for casadi s-function (nlpsol): ', num2str(toc), ' s']);
        elseif(compile_mode == 2)
            tic;
            s_fun_name = 's_function_opti.c';
            compile_casadi_sfunction(casadi_fun, s_fun_name, output_dir, fun_name, '-O2', compile_mode); % default opti s-function
            disp(['Compile time for casadi s-function (opti for nlpsol): ', num2str(toc), ' s']);
        end
    end
catch ME
    disp('Error in compile_py_cfun_to_sfun.m')
    fprintf(2, 'Error: %s\n', getReport(ME));
end

output_dir = ['./s_functions/', robot_name, '/matlab_functions/'];
try
    for i = 1:length(fun_arr_matlab)
        fun_name = fun_arr_matlab{i};
        
        casadi_fun = Function.load([input_dir, fun_name '.casadi']);

        casadi_fun_to_mex(casadi_fun, output_dir, fun_name, '-O2');
        disp(['Compile time for matlab s-function: ', num2str(toc), ' s']);
    end
catch ME
    disp('Error in casadi_fun_to_mex.m')
    fprintf(2, 'Error: %s\n', getReport(ME));
end