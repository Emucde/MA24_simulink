function y_W_E_err = kin_fun_err_reduced(xe, q_reduced, param_robot)
% CALCULATE_ERROR Computes the position and orientation error between the end-effector and desired pose
%
% Inputs:
%   param_robot.q_0_ref - Initial joint configuration vector
%   q_reduced   - Reduced joint configuration vector
%   param_robot.n_indices - Indices of joints to be updated (non fixed indices)
%   xe          - Desired end-effector pose [x; y; z; qw; qx; qy; qz]
%
% Outputs:
%   y_W_E_err       - Error vector [position_error; orientation_error]
%
% This function calculates the position and orientation error between
% the current end-effector pose and the desired pose in world frame.


    % Update joint configuration
    q = param_robot.q_0_ref;
    q(param_robot.n_indices) = q_reduced;
    
    % Calculate forward kinematics
    H = hom_transform_endeffector_py(q);
    
    % Calculate rotation error
    %RR = H(1:3,1:3)*quat2rotm_v2(xe(4:7))';
    
    % Calculate position error
    p_err = H(1:3,4) - xe(1:3);
    
    % Calculate orientation error
    %q_err = rotm2quat_v4(RR);

    q_err = quat_mult(rotm2quat_v4(H(1:3,1:3)), quat_inv(xe(4:7)));
    %r_err = q_err(2:4);

    % Combine position and orientation errors
    y_W_E_err = [p_err; q_err];
end
