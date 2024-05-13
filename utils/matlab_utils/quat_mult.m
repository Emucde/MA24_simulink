function s = quat_mult(q,r)
% q,r vector of length 4, interpreted as quaternions
% return s = q*r quaternion product

% Tims Version

% make shure, q and r are interpreted as columns
% q = reshape(q,[4 1]);
% r = reshape(r,[4 1]);

% q_eta = q(1);
% q_eps = q(2:4);
% r_eta = r(1);
% r_eps = r(2:4);
% 
% s = [q_eta*r_eta - q_eps'*r_eps; q_eta*r_eps + r_eta*q_eps + cross(q_eps,r_eps)]';

% Matlab (glaube ist unnötig)
%q = reshape(q,[4 1]);
%r = reshape(r,[4 1]);

% Calculate vector portion of quaternion product
% vec = s1*v2 + s2*v1 + cross(v1,v2)
vec = [ q(1)*r(2); q(1)*r(3); q(1)*r(4) ] + ...
      [ r(1)*q(2); r(1)*q(3); r(1)*q(4) ] + ...
      [ q(3)*r(4)-q(4)*r(3); 
        q(4)*r(2)-q(2)*r(4); ...
        q(2)*r(3)-q(3)*r(2)             ];

% Calculate scalar portion of quaternion product
% scalar = s1*s2 - dot(v1,v2)
scalar = q(1)*r(1) - q(2)*r(2) - q(3)*r(3) - q(4)*r(4);
    
s = [scalar; vec];
end

