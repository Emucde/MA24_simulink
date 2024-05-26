function R = rpy2rotm(Phi)
    % Konvertiert RPY-Winkel XYZ in eine Rotationsmatrix
    R = eul2rotm(Phi, 'XYZ');
end