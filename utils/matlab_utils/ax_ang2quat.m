function quat = ax_ang2quat(rot_ax, ang)
% AX_ANG2QUAT Converts an axis-angle representation to a quaternion.
%
%   INPUTS:
%   - rot_ax: 3x1 vector representing the rotation axis.
%   - ang: Scalar value representing the rotation angle in radians.
%
%   OUTPUT:
%   - quat: 4x1 vector representing the quaternion [qw, qx, qy, qz].
%
%   Note: The rotation axis should be a unit vector.

    % Calculate the quaternion
    quat = [cos(ang/2); rot_ax*sin(ang/2)];

end