function rpy = rotm2rpy_casadi(R)
    % Converts a rotation matrix to roll, pitch, and yaw angles
    % from extrinsic XYZ RPY angles, i. e. the roll, pitch and yaw
    % action should be interpreted as a rotation about the new axes
    % i. e.
    %    1. roll around x
    %    2. roll around new y
    %    3. roll around new z
    import casadi.*

    % Extract elements of the rotation matrix
    r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
    r21 = R(2,1); r22 = R(2,2); r23 = R(2,3); 
    r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);

    % Calculate the roll, pitch, and yaw angles (Craig, S. 47f)
    % alpha = yaw, beta = pitch, gamma = roll
    beta = atan2(-r31, sqrt(r32^2 + r33^2));
    cos_beta = cos(beta);

    % [TODO if_else]
    gamma = if_else(beta == pi/2, atan2(r12, r13), if_else(beta == -pi/2, atan2(-r12, -r13), atan2(r32/cos_beta, r33/cos_beta)));
    alpha = if_else(beta == pi/2, 0,               if_else(beta == -pi/2, 0,                 atan2(r21/cos_beta, r11/cos_beta)));
    %if(beta == pi/2)
    %    gamma = atan2(r12, r13);
    %    alpha = 0;
    %elseif(beta == -pi/2)
    %    gamma = atan2(-r12, -r13);
    %    alpha = 0;
    %else
    %    gamma = atan2(r32/cos_beta, r33/cos_beta);
    %    alpha = atan2(r21/cos_beta, r11/cos_beta);
    %end

    % Create a CasADi symbolic vector for the rpy angles
    yaw = alpha; pitch = beta; roll = gamma;
    rpy = vertcat(yaw, pitch, roll);
end