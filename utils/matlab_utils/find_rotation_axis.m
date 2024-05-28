function [rot_ax_o, rot_alpha_scale_o] = find_rotation_axis(R_init, R_target)
%FIND_ROTATION_AXIS Computes the rotation axis and angle between two rotation matrices
%
% Inputs:
%   R_init (3x3 matrix) - Initial rotation matrix
%   R_target (3x3 matrix) - Target rotation matrix
%
% Outputs:
%   rot_ax_o (3x1 vector) - Optimized rotation axis
%   rot_alpha_scale_o (scalar) - Optimized rotation angle in radians
%
% Description:
%   This function computes the rotation axis and angle between two rotation matrices R_init and R_target.
%   It calculates the difference rotation matrix RR = R_init'*R_target.
%   The function then extracts the rotation angle from the trace of RR.
%   The rotation axis is computed from the off-diagonal elements of RR.
%   To ensure the rotation axis and angle corresponds to the shortest
%   rotation distance only the angle with the smaller magnitude is returned 
%   with its corresponding rotation axis

    % Difference of rotation: R_2_0 = R_2_1*R_1_0 => R_1_0 = R_2_1^(-1) * R_2_0 = R_2_1' * R_2_0 = R_1_2 * R_2_0
    % R_target = RR * R_init => RR = R_target*R_init'
    % Compute the difference rotation matrix
    %RR = R_init'*R_target;
    RR = R_target*R_init';

    % Extract the rotation angle from the trace of RR
    rot_alpha_scale = acos((trace(RR) - 1) / 2);
    
    % Compute the rotation axis from the off-diagonal elements of RR
    rot_ax = [RR(3,2) - RR(2,3); RR(1,3) - RR(3,1); RR(2,1) - RR(1,2)];
    rot_ax = rot_ax / (2 * sin(rot_alpha_scale));
    
    % Ensure the rotation axis points in the positive mathematical direction
    if rot_alpha_scale > pi
        rot_alpha_scale_o = 2*pi - rot_alpha_scale;
        rot_ax_o = -rot_ax;
    else
        rot_alpha_scale_o = rot_alpha_scale;
        rot_ax_o = rot_ax;
    end

end