function R = eul2rotm_casadi(Phi)
    % Konvertiert Euler-Winkel ZYZ in eine Rotationsmatrix
    % Eingaben:
    %   phi, theta, psi: Euler-Winkel vom Typ SX
    % Ausgabe:
    %   R: Rotationsmatrix vom Typ SX (3x3)

    import casadi.*;
    
    phi = Phi(1);
    theta = Phi(2);
    psi = Phi(3);
    
    % Rotationsmatrix für Z-Achse
    Rz_phi = [cos(phi), -sin(phi), 0;
          sin(phi),  cos(phi), 0;
          0,               0,               1];
    
    % Rotationsmatrix für Y-Achse  
    Ry_theta = [cos(theta), 0, sin(theta);
          0,                1, 0;
          -sin(theta), 0, cos(theta)];
    
    % Rotationsmatrix für Z-Achse
    Rz_psi = [cos(psi), -sin(psi), 0;
           sin(psi),  cos(psi), 0;
           0,                0,               1];
    
    % Gesamtrotationsmatrix
    R = Rz_phi * Ry_theta * Rz_psi;
end