function euler_angles = rotm2eul_casadi(R)
% rotm2eul_casadi - Converts a rotation matrix to Euler angles using CasADi.
% The rotation matrix R should be a 3x3 matrix.
% The output euler_angles will be a 3x1 vector containing the angles in radians
    import casadi.*

    % Extract Euler angles 'ZYZ' from the rotation matrix (0 < theta < pi)
    phi = atan2(R(2,3), R(1,3));
    theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    psi = atan2(R(3,2), -R(3,1));

    euler_angles = [phi; theta; psi];
end