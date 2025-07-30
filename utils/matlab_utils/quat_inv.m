function s = quat_inv(r)
%QUAT_INV Inverse of a quaternion.
%   s = quat_inv(r) computes the inverse of a quaternion r.
r = reshape(r,[4 1]);
s = [r(1); -r(2:4)];

end

