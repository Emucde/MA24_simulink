% Compiles casadi functions from python to s-functions for simulink
% This function does the following:
% 1. Compile the casadi functions to s-functions for simulink (without sources)
% 2. Compile the casadi functions to matlab functions (mex files)
% 3. Compile the casadi functions to s-functions for realtime simulink (with sources)


% Alternative: Standalone
% if(~isdeployed)
%     cd(fileparts(which(mfilename)));
% end

% init_casadi;
% import casadi.*;
% output_dir = './s_functions/s_functions_7dof/';

import casadi.*;

if(~exist('parameter_str', 'var'))
    run('main_matlab/parameters_7dof.m');
end

compile_matlab_sfun = true;
create_some_sources = true;

% valid robot_names: fr3_7dof, fr3_6dof, ur5e
% robot_name = 'fr3_7dof';
% robot_name = 'fr3_6dof';
robot_name = 'fr3_no_hand_6dof';
% robot_name = 'ur5e_6dof';

compile_mode = 2; % Compile mode: 1 - nlpsol, 2 - opti
% coptimflags = '-O3'; % Optimization flag for compilation
coptimflags = '-Ofast -march=native -flto'; % Optimization flag for compilation

output_dir = ['./s_functions/', robot_name, '/'];
input_dir = [output_dir, 'casadi_functions/'];
s_fun_path = output_dir;

output_dir_realtime = './main_franka/Controller/s_functions/';

% s functions for simulink
fun_arr_sfun = { ...
    'sys_fun_qpp_aba_red_py', ...
    'sys_fun_qpp_aba_py', ...
    'sys_fun_qpp_sol_py', ...
    'robot_model_bus_fun_py', ...
    'robot_model_bus_fun_red_py', ...
    'ekf_fun_py' ...
};

fun_arr_sfun_realtime = { ...
    'sys_fun_qpp_aba_py', ...
    'robot_model_bus_fun_py' ...
};

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
    'hom_transform_endeffector_red_py', ...
    'hom_transform_joint_1_py', ...
    'hom_transform_joint_2_py', ...
    'hom_transform_joint_3_py', ...
    'hom_transform_joint_4_py', ...
    'hom_transform_joint_5_py', ...
    'hom_transform_joint_6_py', ...
    'hom_transform_joint_7_py', ...
    'quat_endeffector_py', ...
    'quat_R_endeffector_py', ...
    'geo_jacobian_endeffector_py', ...
    'geo_jacobian_endeffector_p_py' ...
};

fun_arr_only_source = { ...
    'ekf_fun_no_gravity_py' ...
};

if create_some_sources
    disp('Create only source files for s-functions:');
    for i = 1:length(fun_arr_only_source)
        fun_name = fun_arr_only_source{i};
        
        casadi_fun = Function.load([input_dir, fun_name '.casadi']);
        
        create_only_source(casadi_fun, [output_dir, 'mpc_c_sourcefiles/']);
        % return
    end
    fprintf('\n--------------------------------------------------------------------\n\n');
end

%% 1. Compile casadi functions to s-functions for simulink (without sources)

try
    disp('Compile casadi functions to s-functions for simulink:');
    compile_multiple_cfun(fun_arr_sfun, s_fun_path, input_dir, output_dir, coptimflags, compile_mode, false);
    fprintf('\n--------------------------------------------------------------------\n\n');

    disp('Compile casadi functions to s-functions for realtime simulink:');
    compile_multiple_cfun(fun_arr_sfun_realtime, s_fun_path, input_dir, output_dir_realtime, coptimflags, compile_mode, false);
    fprintf('\n--------------------------------------------------------------------\n\n');
catch ME
    disp('Error in compile_py_cfun_to_sfun.m')
    % fprintf(2, 'Error: %s\n', getReport(ME));
    error('Error: %s\n', getReport(ME));
end

%% 2. Compile casadi functions to matlab functions (mex files)

if compile_matlab_sfun
    output_dir = ['./s_functions/', robot_name, '/matlab_functions/'];
    try
        for i = 1:length(fun_arr_matlab)
            fun_name = fun_arr_matlab{i};
            
            casadi_fun = Function.load([input_dir, fun_name '.casadi']);

            disp('--------------------------------------------------------------------');
            disp(['Compile casadi functions to matlab functions (mex files): ', fun_name]);
            
            tic;
            casadi_fun_to_mex(casadi_fun, output_dir, [s_fun_path, '/matlab_functions'], fun_name, coptimflags);
            disp(['Compile time for matlab s-function: ', num2str(toc), ' s']);
        end
        fprintf('\n--------------------------------------------------------------------\n\n');
    catch ME
        disp('Error in casadi_fun_to_mex.m')
        % fprintf(2, 'Error: %s\n', getReport(ME));
        error('Error: %s\n', getReport(ME));
    end
end

%% 3. Compile casadi functions to s-functions for simulink (with sources)

function compile_multiple_cfun(cfun_arr, s_fun_path, input_dir, output_dir, coptimflags, compile_mode, remove_sourcefiles)
%COMPILE_MULTIPLE_CFUN Compiles multiple CasADi functions to S-functions
%
%   This function compiles a list of CasADi functions to S-functions for use in Simulink.
%
%   Inputs:
%       cfun_arr - Cell array of function names to be compiled
%       s_fun_path - Path to the S-function template
%       input_dir - Directory containing the .casadi files
%       output_dir - Directory where compiled S-functions will be saved
%       coptimflags - Optimization flag for compilation (e.g., '-O2', '-O3')
%       compile_mode - Compilation mode (1 for nlpsol, 2 for opti)
%       remove_sourcefiles - Boolean flag to remove source files after compilation
%
%   The function loads each CasADi function, compiles it to an S-function,
%   and displays the compilation time for each function.

    for i = 1:length(cfun_arr)
        fun_name = cfun_arr{i};

        casadi_fun = casadi.Function.load([input_dir, fun_name '.casadi']);
        if(remove_sourcefiles)
            add_str = ' with sources';
        else
            add_str = '';
        end

        if(compile_mode == 1)
            out_string = ['Compile time for casadi s-function (nlpsol', add_str, '): '];
        elseif(compile_mode == 2)
            out_string = ['Compile time for casadi s-function (opti for nlpsol', add_str, '): '];
        end

        tic;
        compile_casadi_sfunction(casadi_fun, s_fun_path, output_dir, fun_name, coptimflags, compile_mode, remove_sourcefiles);
        disp([out_string, num2str(toc), ' s']);
    end
end