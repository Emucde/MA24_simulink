function q = rotm2quat_v2(R)
    % Converts a rotation matrix R into a quaternion q
    % without using any if-else constructs.
    
    % Extract the elements of the rotation matrix
    r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
    r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
    r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);
    
    % Calculate the quaternion components
    q0 = sqrt(1 + r11 + r22 + r33) / 2;
    q1 = (r32 - r23) / (4 * q0);
    q2 = (r13 - r31) / (4 * q0);
    q3 = (r21 - r12) / (4 * q0);
    
    % Assemble the quaternion
    q = [q0; q1; q2; q3];
end