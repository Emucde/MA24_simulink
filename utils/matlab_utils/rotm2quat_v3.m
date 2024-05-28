function q = rotm2quat_v3(R)
    % Converts a rotation matrix R into a quaternion q

    % Extract the elements of the rotation matrix
    r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
    r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
    r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);
    
    % Calculate the quaternion components
    q0 = sqrt(abs(1 + r11 + r22 + r33)) / 2;
    q1 = sign(r32 - r23) * sqrt(abs(r11 - r22 - r33 + 1)) / 2;
    q2 = sign(r13 - r31) * sqrt(abs(r22 - r33 - r11 + 1)) / 2;
    q3 = sign(r21 - r12) * sqrt(abs(r33 - r11 - r22 + 1)) / 2;
    
    % Assemble the quaternion
    q = [q0; q1; q2; q3];
    warning('rotm2quat_v3 is not robust to numerical errors for numbers < 2.2204e-16 = eps. Use rotation2quaternion instead.');
    % ACHTUNG IST NICHT ROBUST AUF RAUSCHEN NAHE 0! Wenn es zu vorzeichenwechseln kommt, dann ist das Ergebnis falsch.
    % passiert wenn einträge < eps = 2.2204e-16 sind
    %q=q/norm(q);
end