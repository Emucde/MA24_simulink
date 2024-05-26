function rpy = rotm2rpy(R)
    % Convert the rotation matrix to Euler angles using the XYZ convention
    %Phi = rotm2eul(R, 'XYZ')';

    % Extract elements of the rotation matrix
    r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
    r21 = R(2,1); r22 = R(2,2); r23 = R(2,3); 
    r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);

    % Calculate the roll, pitch, and yaw angles (Craig, S. 47f)
    % alpha = yaw, beta = pitch, gamma = roll
    beta = atan2(-r31, sqrt(r32^2 + r33^2));
    cos_beta = cos(beta);

    if(beta == pi/2)
        gamma = atan2(r12, r13);
        alpha = 0;
    elseif(beta == -pi/2)
        gamma = atan2(-r12, -r13);
        alpha = 0;
    else
        gamma = atan2(r32/cos_beta, r33/cos_beta);
        alpha = atan2(r21/cos_beta, r11/cos_beta);
    end

    % Create a CasADi symbolic vector for the rpy angles
    yaw = alpha; pitch = beta; roll = gamma;
    rpy = vertcat(yaw, pitch, roll);
end