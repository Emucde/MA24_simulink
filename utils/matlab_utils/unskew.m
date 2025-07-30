function v=unskew(S)
  % UNSKEW - Converts a skew-symmetric matrix to a vector.
  % Usage: v=unskew(S)
  tmp=S+S';
  tmp_norm=norm(tmp);
  if(length(S)~=3)
    error('unskew(S): Invalid matrix size S');
  elseif(tmp_norm>1e-9) %eps(16)
    tmp_norm
    S
    error('unskew(S): Non-skew-symmetric matrix S');
  else
    v = zeros(3,1); % it takes only positive values form skew matrix
    v(1) = S(3,2); % s1
    v(2) = S(1,3); % s2
    v(3) = S(2,1); % s3

    % Skew matrix with elements s1, s2, s3:
    %     [  0  -s3  s2   ]   [  S(1,1)  S(1,2)  S(1,3)  ]
    % S = [  s3   0  -s1  ] = [  S(2,1)  S(2,2)  S(2,3)  ]
    %     [ -s2  s1   0   ]   [  S(3,1)  S(3,2)  S(3,3)  ]
  end
end
