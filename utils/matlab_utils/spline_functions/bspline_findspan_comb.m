function i = bspline_findspan_comb(theta,knot,degree)
  % Find B-Spline span matching to current position theta using binary
  % search.
  % Algorithm A2.1 from "The NURBS Book" (p. 72f) used
  % Indices are adapted for the use in MATLAB.
  U=knot;
  m=length(U)-1;
  p=degree;
  n=m-p-1;
  u=theta;
  if(theta>=U(n+1+1))
    i=n;
  else
    low=p;
    high=n+1;
    mid=floor((low+high)/2);
    while(u<U(mid+1) || u>=U(mid+1+1))
      if(u<U(mid+1))
        high=mid;
      else
        low=mid;
      end
      mid=floor((low+high)/2);
      %Break condition, if the theta is out of range
      if(low==mid && mid==high)
        break;
      end
    end
    i=mid;
  end
end
