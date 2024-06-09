function S = skew(v)
  % Calculate the skew symmetric matrix from a 3D vector
  %   v: 3-element vector
  %   S: 3x3 skew symmetric matrix

  S = [ 0,    -v(3),  v(2); ...
        v(3),  0,    -v(1); ...
       -v(2),  v(1),  0];

end