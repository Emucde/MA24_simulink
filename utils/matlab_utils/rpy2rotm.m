function R = rpy2rotm(Phi)
    % Konvertiert RPY-Winkel XYZ in eine Rotationsmatrix
    %R = eul2rotm(Phi, 'ZYX');

    alpha = Phi(1); % yaw
    beta  = Phi(2); % pitch
    gamma = Phi(3); % roll
    
    % Define rotational matrices
    %R_z
    R_z = [cos(alpha), -sin(alpha), 0; ...
           sin(alpha), cos(alpha), 0; ...
           0, 0, 1];

    %R_y
    R_y = [cos(beta), 0, sin(beta); ...
           0, 1, 0; ...
          -sin(beta), 0, cos(beta)];

    %R_x
    R_x = [ 1, 0, 0; ...
            0, cos(gamma), -sin(gamma); ...
            0, sin(gamma), cos(gamma)];
    
    % Compute the overall rotational matrix
    R = R_z * R_y * R_x; %R_z R_y R_x
end