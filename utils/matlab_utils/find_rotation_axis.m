function [rot_ax_o, rot_alpha_scale_o] = find_rotation_axis(R_init, R_target, mode)
arguments
    R_init (3,3) double {mustBeReal, mustBeFinite}
    R_target (3,3) double {mustBeReal, mustBeFinite}
    mode char {mustBeMember(mode, ['fast_from_rotm |', ' slow_from_quat'])} = 'fast_from_rotm'
end
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

    % Nachmultiplikation: R_2_0 = R_1_0*R_2_1 => R_2_1 = R_1_0^(-1) * R_2_0 = R_1_0' * R_2_0 = R_0_1 * R_2_0
    %   mit R_2_0 = R_target_0 und R_1_0 = R_init_0
    %   Drehung um körperfestes KOS, d. h. im Bezug auf R_init
    % Vormultiplikation: R_2_0 = R_1_0*R_2_1 => R_2_1 = R_1_0^(-1) * R_2_0 = R_1_0' * R_2_0 = R_0_1 * R_2_0
    %   mit R_2_0 = R_target_0 und R_2_1 = R_init_0, da die
    %   Vormultipklikationsvarianteverwendet wird. D. h. wir möchten
    %   Drehungen um ein Bezugsfestes Inertialsystem ausführen und das ist
    %   eben dann möglich, wenn man die Drehoperation zur
    %   Anfangsrotationsmatrix R_init von links multipliziert. Damit ist die
    %   gesuchte Fehlerrotationsmatrix RR = R_1_0.
    %   D.h. es gilt
    %   R_1_0 = R_2_0 * R_2_1' = R_target * R_init'

    % Compute the difference rotation matrix
    RR = R_init'*R_target; % Im Fall der Nachmulitplikation (drehung um bezugsfestes KOS) korrekt.
    %RR = R_target * R_init'; % Im Fall der Vormultiplikation korrekt.

    if mode == "fast_from_rotm"
        % Have the problem that not always wa rotation axis can be found.
        % Extract the rotation angle from the trace of RR
        rot_alpha_scale = acos((trace(RR) - 1) / 2);
        
        if(rot_alpha_scale == 0)
            rot_ax = [0; 0; 0];
        else
            % Compute the rotation axis from the off-diagonal elements of RR
            rot_ax = [RR(3,2) - RR(2,3); RR(1,3) - RR(3,1); RR(2,1) - RR(1,2)];
            rot_ax = rot_ax / (2 * sin(rot_alpha_scale));
        end
    elseif mode == "slow_from_quat"
        quat = rotm2quat_v4(RR);
        quat = quat/norm(quat); % avoids numerical errors
        rot_alpha_scale = 2*acos(quat(1));
        if(rot_alpha_scale == 0)
            rot_ax = [0; 0; 0];
        else
            rot_ax = quat(2:4) / sin(rot_alpha_scale/2);
        end
    end
        
    % Ensure the rotation axis points in the positive mathematical direction
    if rot_alpha_scale > pi
        rot_alpha_scale_o = 2*pi - rot_alpha_scale;
        rot_ax_o = -rot_ax;
    else
        rot_alpha_scale_o = rot_alpha_scale;
        rot_ax_o = rot_ax;
    end

    if(~isreal(rot_alpha_scale_o))
        error('Rotation angle is not real');
    end
end