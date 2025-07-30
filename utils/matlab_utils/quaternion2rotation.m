function R = quaternion2rotation(q)
% quaternion2rotation converts a quaternion to a rotation matrix.
%   R = quaternion2rotation(q) converts a quaternion q to a 3x3 rotation
%   matrix R. The quaternion q is expected to be in the format [s, x, y, z],
%   where s is the scalar part and (x, y, z) are the vector part.
    q = double(q);
    s = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    R = [   1-2*(y^2+z^2)   2*(x*y-s*z) 2*(x*z+s*y)
        2*(x*y+s*z) 1-2*(x^2+z^2)   2*(y*z-s*x)
        2*(x*z-s*y) 2*(y*z+s*x) 1-2*(x^2+y^2)   ];
end
