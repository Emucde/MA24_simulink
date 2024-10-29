function w = calc_manipulability_reduced(q_reduced, param_robot)
% CALCULATE_REDUCED_MANIPULABILITY Computes the reduced manipulability measure
%
% Inputs:
%   param_robot - Structure containing robot parameters
%     .q_0_ref  - Reference joint configuration vector
%     .n_indices - Indices of joints to be considered for reduced manipulability
%   q_reduced   - Reduced joint configuration vector
%
% Outputs:
%   w           - Reduced manipulability measure
%
% This function calculates the reduced manipulability measure for a robot
% given a set of joint indices and their corresponding joint values.

    % Initialize full joint configuration with reference values
    q = param_robot.q_0_ref;
    
    % Update joint configuration with reduced set
    q(param_robot.n_indices) = q_reduced;
    
    % Calculate the geometric Jacobian for the end-effector
    J = geo_jacobian_endeffector_py(q);
    
    % Extract the reduced Jacobian using specified indices
    J_red = J(:, param_robot.n_indices);
    
    % Calculate the manipulability measure
    % Note: abs() is used to ensure a non-negative value inside the sqrt
    w = sqrt(abs(det(J_red * J_red')));
    
    % Note: If the determinant is expected to be always non-negative,
    % the abs() function can be removed for slightly better performance.
end
