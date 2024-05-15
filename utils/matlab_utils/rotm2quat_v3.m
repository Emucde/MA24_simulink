function q = rotm2quat_v3(R)
    % Converts a rotation matrix R into a quaternion q
    % without using any if-else constructs.

    %my_sign = @(x) sign(x) + (x == 0);
    %my_sign = @(x) 2*(x >= 0) - 1;
    
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
end