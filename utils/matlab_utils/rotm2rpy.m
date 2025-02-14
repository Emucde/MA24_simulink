function rpy = rotm2rpy(R)
    % Convert the rotation matrix to Euler angles using the XYZ convention
    %Phi = rotm2eul(R, 'ZYX')';


    % Extract elements of the rotation matrix
    r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
    r21 = R(2,1); r22 = R(2,2); r23 = R(2,3); 
    r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);
    
    %%{
    % Calculate the roll, pitch, and yaw angles (Craig, S. 47f)
    % alpha = yaw, beta = pitch, gamma = roll
    beta = atan2(-r31, sqrt(r32^2 + r33^2));
    cos_beta = cos(beta);

    if(abs(beta - pi/2) < eps)
        gamma = atan2(r12, r13);
        alpha = 0;
    elseif(abs(beta + pi/2) < eps)
        gamma = atan2(-r12, -r13);
        alpha = 0;
    else
        gamma = atan2(r32/cos_beta, r33/cos_beta);
        alpha = atan2(r21/cos_beta, r11/cos_beta);
    end
    %}

    %{
    if(r31 == 1)
        alpha1 = 0;
        beta1 = -pi/2;
        gamma1 = atan2(-r12, -r13);

        alpha2 = alpha1;
        beta2 = beta1;
        gamma2 = gamma1;
    elseif(r31 == -1)
        alpha1 = 0;
        beta1 = pi/2;
        gamma1 = atan2(r12, r13);

        alpha2 = alpha1;
        beta2 = beta1;
        gamma2 = gamma1;
    else
        beta1 = -asin(r31);
        beta2 = pi - beta1;

        c_beta1 = cos(beta1);
        c_beta2 = cos(beta2);

        alpha1 = atan2(r32/c_beta1, r33/c_beta1);
        alpha2 = atan2(r32/c_beta2, r33/c_beta2);

        gamma1 = atan2(r21/c_beta1, r11/c_beta1);
        gamma2 = atan2(r21/c_beta2, r11/c_beta2);
    end

    % use only first solution
    alpha = alpha1;
    beta = beta1;
    gamma = gamma1;
    %}
    
    rpy = vertcat(alpha, beta, gamma);
end