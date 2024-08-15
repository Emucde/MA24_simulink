function rotation = spline_rotmat(rot_matrices, theta_j)
% spline_rotmat Calculates various path parameters based on a series of rotational matrices.
% 
% Input:
%   - rot_matrices: An array of rotational matrices where each column represents a rotational matrix.
%                   The first column represents R0, the second represents R1, and so on.
%   - theta_j : paramter t_tilde_j from spline calculation. At this
%               parameter the spline has the value of the desired point QQ
% 
% Output:
%   - path_param:   A struct containing the calculated path parameters.
%                   It includes the following fields:
%                   - R_init: Initial rotational matrix (R1_W_E)
%                   - skew_ew: Skew symmetric matrix obtained from the quaternion representation of RR
%                   - skew_ew_square: Square of skew_ew
%                   - alpha: Angle of rotation obtained from quaternion representation of RR
%                   - theta_j: theta parameter on that a point QQ is
%                              present.
%

% Example usage:
% R1_W_E = [1 0 0; 0 1 0; 0 0 1];
% R2_W_E = [-1 0 0; 0 0 1; 0 1 0];
% rot_matrices = [R1_W_E, R2_W_E];
% path_param = calculatePathParameters(rot_matrices);

N = size(rot_matrices,3);
R_init = zeros(3,3,N-1);
skew_ew = zeros(3,3,N-1);
skew_ew_square = zeros(3,3,N-1);
alpha = zeros(1,N-1);

for i = 1:N-1
    % Calculate RR by multiplying the transpose of R_i with R_{i+1}
    RR = rot_matrices(:,:,i)' * rot_matrices(:,:,i+1);
    
    % Convert RR to quaternion representation
    quat_R2 = rotation2quaternion(RR);
    
    % Extract components of the quaternion
    qq1 = quat_R2(1);
    qq1_alpha = 2 * acos(qq1);

    if(sin(qq1_alpha / 2) == 0)
        qq1_r = [0 0 0];
    else
        qq1_r = quat_R2(2:4) / sin(qq1_alpha / 2);
    end

    R_init(:,:,i) = rot_matrices(:,:,i);
    skew_ew(:,:,i) = skew(qq1_r);
    skew_ew_square(:,:,i) = skew(qq1_r)^2;
    alpha(i) = qq1_alpha/(theta_j(i+1)-theta_j(i));
end

% Define path parameters
rotation.R_init = R_init;
rotation.skew_ew = skew_ew;
rotation.skew_ew_square = skew_ew_square;
rotation.alpha = alpha;
rotation.theta_j = theta_j;

end