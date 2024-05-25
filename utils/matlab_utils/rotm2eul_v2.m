function Phi = rotm2eul_v2(R)
    % Convert the rotation matrix to Euler angles using the ZYZ convention
    %Phi = rotm2eul(rotation_matrix, 'ZYZ')';
    %import casadi.*

    %R = MX(rot_matrix); % Convert rotation matrix to CasADi MX type

    % Extract Euler angles 'ZYZ' from the rotation matrix ( 0 < theta < pi)
    phi = atan2(R(2,3), R(1,3));
    theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    psi = atan2(R(3,2), -R(3,1));

    % Ensure Euler angles are within the valid range
    %phi = wrapToPi(phi);
    %theta = wrapToPi(theta);
    %psi = wrapToPi(psi);

    Phi = [phi; theta; psi];
end