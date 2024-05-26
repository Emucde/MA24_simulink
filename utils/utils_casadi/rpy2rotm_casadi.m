function R = rpy2rotm_casadi(phi)
    import casadi.*;
    % Extract roll, pitch, and yaw angles
    alpha = phi(1); % yaw
    beta = phi(2); % pitch
    gamma = phi(3); % roll
    
    % Define rotational matrices
    %R_x
    R_alpha = [1, 0, 0;
               0, cos(alpha), -sin(alpha);
               0, sin(alpha), cos(alpha)];
    
    %R_y
    R_beta = [cos(beta), 0, sin(beta);
              0, 1, 0;
              -sin(beta), 0, cos(beta)];
    
    %R_z
    R_gamma = [cos(gamma), -sin(gamma), 0;
               sin(gamma), cos(gamma), 0;
               0, 0, 1];
    
    % Compute the overall rotational matrix
    R = R_gamma * R_beta * R_alpha; %R_x R_y R_z
end