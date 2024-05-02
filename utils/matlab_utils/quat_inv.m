function s = quat_inv(r)
r = reshape(r,[4 1]);
s = [r(1); -r(2:4)];

end

