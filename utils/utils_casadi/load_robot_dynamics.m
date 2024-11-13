function [f, compute_tau_fun, gravity_fun, hom_transform_endeffector_py_fun, quat_endeffector_py_fun] = load_robot_dynamics(input_dir, n, no_gravity, use_aba)
    % LOAD_ROBOT_DYNAMICS Loads and prepares robot dynamics functions
    %
    % This function loads various dynamics-related functions for a robot model,
    % including forward dynamics, inverse dynamics, gravitational forces,
    % and end-effector transformations.
    %
    % Inputs:
    %   input_dir - String, directory containing the .casadi function files
    %   n - Integer, number of degrees of freedom of the robot
    %   no_gravity - Boolean, true if gravity should be ignored, false otherwise
    %   use_aba - Boolean, true to use Articulated Body Algorithm, false otherwise
    %
    % Outputs:
    %   f - CasADi Function, forward dynamics function
    %   compute_tau_fun - CasADi Function, inverse dynamics function
    %   gravity_fun - CasADi Function, gravitational forces function
    %   hom_transform_endeffector_py_fun - CasADi Function, homogeneous transform of end-effector
    %   quat_endeffector_py_fun - CasADi Function, quaternion of end-effector
    %
    % Note: This function assumes that the necessary .casadi files are present in the input_dir.
    import casadi.*;

        % Determine which system function to load based on gravity and ABA settings
        if use_aba
            if no_gravity
                sys_fun_str = 'sys_fun_x_sol_nogravity_py.casadi';
            else
                sys_fun_str = 'sys_fun_x_aba_py.casadi';
            end
        else
            if no_gravity
                sys_fun_str = 'sys_fun_x_sol_nogravity_py.casadi';
            else
                sys_fun_str = 'sys_fun_x_sol_py.casadi';
            end
        end
    
        % Determine which tau computation function to load and set up gravity function
        if no_gravity
            tau_fun_str = 'compute_tau_nogravity_py.casadi';
            gravity_fun = Function('g', {SX.sym('q', n, 1)}, {SX(n, 1)}); % always zero
        else
            tau_fun_str = 'compute_tau_py.casadi';
            gravity_fun = Function.load([input_dir, 'gravitational_forces_py.casadi']); % Inverse Dynamics (ID)
        end
    
        % Load the dynamics functions
        f = Function.load([input_dir, sys_fun_str]); % forward dynamics (FD), d/dt x = f(x, u), x = [q; dq]
        compute_tau_fun = Function.load([input_dir, tau_fun_str]); % Inverse Dynamics (ID)
    
        % Load end-effector transformation functions
        hom_transform_endeffector_py_fun = Function.load([input_dir, 'hom_transform_endeffector_py.casadi']);
        quat_endeffector_py_fun = Function.load([input_dir, 'quat_endeffector_py.casadi']);
    end    