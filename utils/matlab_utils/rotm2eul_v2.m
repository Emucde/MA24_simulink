function Phi = rotm2eul_v2(R)
    % Convert the rotation matrix to Euler angles using the ZYZ convention
    %Phi = rotm2eul(rotation_matrix, 'ZYZ')';

    %R = MX(rot_matrix); % Convert rotation matrix to CasADi MX type

    % Extract Euler angles 'ZYZ' from the rotation matrix ( 0 < theta < pi)
    phi = atan2(R(2,3), R(1,3));
    theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
    psi = atan2(R(3,2), -R(3,1));

    % Ensure Euler angles are within the valid range
    %phi = wrapToPi(phi);
    %theta = wrapToPi(theta);
    %psi = wrapToPi(psi);

    % r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
    % r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
    % r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);
    % 
    % % Calculate theta
    % theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3)); %theta = asin(r23);
    % 
    % % Check special cases
    % if abs(r13) < 1e-10 && abs(r23) < 1e-10 % r13 = 0 and r23 = 0
    %     % Ambiguous case, set phi = 0
    %     phi = 0;
    %     psi = atan2(r21, r22);
    % elseif abs(r13) < 1e-10 % r13 = 0
    %     phi = atan2(-r21, r11);
    %     psi = atan2(r32, r33);
    % elseif abs(r23) < 1e-10 % r23 = 0
    %     phi = atan2(r12, r22);
    %     psi = atan2(-r31, r33);
    % else
    %     % General case
    %     phi = atan2(r13, -r33);
    %     psi = atan2(r23, r22);
    % end

    Phi = [phi; theta; psi];
end