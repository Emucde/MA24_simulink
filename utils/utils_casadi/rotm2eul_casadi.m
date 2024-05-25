function euler_angles = rotm2eul_casadi(R)
    import casadi.*

    % Extract Euler angles 'ZYZ' from the rotation matrix (0 < theta < pi)
    phi = atan2(R(2,3), R(1,3));
    theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    psi = atan2(R(3,2), -R(3,1));

    euler_angles = [phi; theta; psi];
end