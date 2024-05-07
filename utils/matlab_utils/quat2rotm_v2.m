function R = quat2rotm_v2(q)
    % Converts a quaternion q into a rotation matrix R
    % without using any if-else constructs.
    
    % Extract the quaternion components
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
    
    % Calculate the rotation matrix elements
    r11 = q0^2 + q1^2 - q2^2 - q3^2;
    r12 = 2 * (q1 * q2 - q0 * q3);
    r13 = 2 * (q1 * q3 + q0 * q2);
    
    r21 = 2 * (q1 * q2 + q0 * q3);
    r22 = q0^2 - q1^2 + q2^2 - q3^2;
    r23 = 2 * (q2 * q3 - q0 * q1);
    
    r31 = 2 * (q1 * q3 - q0 * q2);
    r32 = 2 * (q2 * q3 + q0 * q1);
    r33 = q0^2 - q1^2 - q2^2 + q3^2;
    
    % Assemble the rotation matrix
    R = [r11, r12, r13;
         r21, r22, r23;
         r31, r32, r33];
end