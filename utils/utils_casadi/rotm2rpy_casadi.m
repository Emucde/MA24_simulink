function rpy = rotm2rpy_casadi(R)
    % Converts a rotation matrix to roll, pitch, and yaw angles
    % from extrinsic ZYX RPY angles, i. e. the roll, pitch and yaw
    % action should be interpreted as a rotation about the new axes

    import casadi.*

    % Extract elements of the rotation matrix
    r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
    r21 = R(2,1); r22 = R(2,2); r23 = R(2,3); 
    r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);


    % ZYX
    % Calculate the roll, pitch, and yaw angles (Craig, S. 47f)
    % alpha = yaw, beta = pitch, gamma = roll
    %beta = atan2(-r31, sqrt(r32^2 + r33^2));
    %cos_beta = cos(beta);
%
    %gamma = if_else(beta == pi/2, atan2(r12, r13), if_else(beta == -pi/2, atan2(-r12, -r13), atan2(r32/cos_beta, r33/cos_beta)));
    %alpha = if_else(beta == pi/2, 0,               if_else(beta == -pi/2, 0,                 atan2(r21/cos_beta, r11/cos_beta)));

    % ZYX
    % beta = if_else(r31 == 1, -pi/2, if_else(r31 == -1, pi/2, -asin(r31))); cos_beta = cos(beta);
    % alpha = if_else(r31 == 1, 0, if_else(r31 == -1, 0, atan2(r21/cos_beta, r11/cos_beta)));
    % gamma = if_else(r31 == 1, atan2(-r12, -r13), if_else(r31 == -1, atan2(r12, r13), atan2(r32/cos_beta, r33/cos_beta)));

    % see rotm2rpy.m
    beta = atan2(-r31, sqrt(r32^2 + r33^2));
    cos_beta = cos(beta);

    delta = 1e-13;
    
    gamma = if_else(abs(beta - pi/2) < delta, atan2(r12, r13), if_else(abs(beta +pi/2) < delta, atan2(-r12, -r13), atan2(r32/cos_beta, r33/cos_beta)));
    alpha = if_else(abs(beta - pi/2) < delta, 0, if_else(abs(beta + pi/2) < delta, 0, atan2(r21/cos_beta, r11/cos_beta)));

    rpy = vertcat(alpha, beta, gamma);

    % Alernative implementation of ZYX
    % % Calculate Pitch (Beta)
    % pitch = atan2(-r31, sqrt(r32^2 + r33^2));

    % % Check for gimbal lock condition
    % is_pitch_gimbal_lock = if_else(abs(r31) == 1, 1, 0);
    
    % % roll (Gamma) calculation
    % roll = if_else(is_pitch_gimbal_lock == 1, ...  % Pitch is +/- 90 degrees
    %                 0, ...                        % Set roll to 0 in gimbal lock
    %                 atan2(r32/cos(pitch), r33/cos(pitch)));
    
    % % Yaw (Alpha) calculation
    % yaw = if_else(is_pitch_gimbal_lock == 1, ...
    %                 0, ...                        % Set yaw to 0 in gimbal lock
    %                 atan2(r21/cos(pitch), r11/cos(pitch)));

    % % Concatenate the angles in the order ZYX: [Yaw, Pitch, roll]
    % rpy = vertcat(yaw, pitch, roll);
end