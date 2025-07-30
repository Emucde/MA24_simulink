function epsilon = rotm2quatvec(R)
%ROT2QUATVEC Convert rotation matrix to quaternion vector.
%   epsilon = rotm2quatvec(R) converts a rotation matrix R to a quaternion
%   vector epsilon, where epsilon = [w; x; y; z] and w is the scalar part.
%   The quaternion is normalized such that ||epsilon|| = 1.
%   Input:
%       R - 3x3 rotation matrix
%   Output:
%       epsilon - 4x1 quaternion vector [w; x; y; z]
    q = rotm2quat_v4(R);
    epsilon = q(2:4);
end