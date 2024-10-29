function y_W_E = kin_fun_reduced(q_reduced, param_robot)
% CALCULATE_END_EFFECTOR_POSE Computes the end-effector pose given a reduced joint configuration
%
% Inputs:
%   param_robot - Structure containing robot parameters
%     .q_0_ref  - Reference joint configuration vector
%     .n_indices - Indices of joints to be updated
%   q_reduced   - Reduced joint configuration vector
%
% Outputs:
%   y_W_E       - End-effector pose [x; y; z; qw; qx; qy; qz]
%                 where [x; y; z] is the position and [qw; qx; qy; qz] is the orientation in quaternion form
%
% This function calculates the end-effector pose (position and orientation)
% given a reduced set of joint angles. The pose is represented as a 7x1 vector
% with the first three elements being the position and the last four being
% the orientation in quaternion form.

    % Update joint configuration
    q = param_robot.q_0_ref;
    q(param_robot.n_indices) = q_reduced;
    
    % Calculate forward kinematics
    H = hom_transform_endeffector_py(q);

    % Extract position (first 3 elements of 4th column)
    position = H(1:3, 4);
    
    % Extract rotation matrix and convert to quaternion
    quaternion = rotm2quat_v4(H(1:3, 1:3));
    
    % Combine position and orientation into a single vector
    y_W_E = [position; quaternion];
end
