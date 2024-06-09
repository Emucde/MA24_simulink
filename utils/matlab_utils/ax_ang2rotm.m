function rotation_matrix = ax_ang2rotm(rot_ax, ang)
%AX_ANG2ROTM Converts an axis-angle representation to a rotation matrix.
%   rotation_matrix = ax_ang2rotm(rot_ax, ang) calculates the rotation matrix
%   using the Rodrigues formula for a given rotation axis and angle.
%
%   Inputs:
%       - rot_ax: 3-element vector representing the rotation axis.
%       - ang: Angle of rotation in radians.
%
%   Output:
%       - rotation_matrix: 3x3 rotation matrix.

    % Calculate the skew symmetric matrix
    skew_ew = skew(rot_ax);

    % Calculate the rotation matrix
    rotation_matrix = eye(3) + sin(ang) * skew_ew + (1 - cos(ang)) * skew_ew^2;
end