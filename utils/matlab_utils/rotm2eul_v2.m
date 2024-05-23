function Phi = rotm2eul_v2(rotation_matrix)
    % Convert the rotation matrix to Euler angles using the ZYZ convention
    Phi = rotm2eul(rotation_matrix, 'ZYZ')';
end
