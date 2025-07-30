function s = quat_mult_vec(qq1, qq2)
%QUAT_MULT_VEC Multiply a quaternion by a vector.
%   s = quat_mult_vec(qq1, qq2) multiplies quaternion qq1 by vector qq2.
%   qq1 is a 4xN matrix where each column is a quaternion (w, x, y, z),
%   and qq2 is a 4xN matrix where each column is a vector (w, x, y, z).
%   The result s is a 3xN matrix where each column is the resulting vector (x, y, z).
    q1q2 = quat_mult(qq1, qq2);
        
    s = q1q2(2:4,1);
end
    
    