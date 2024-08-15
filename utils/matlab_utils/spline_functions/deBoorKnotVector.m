function knots = deBoorKnotVector(t_tilde, p)
% Generates the knot vector u according to the de Boor algorithm
% Inputs:
%   i: Knot number
%   n: Number of control points
%   p: Order of the B-spline basis function
% Output:
%   knots: The knot vector generated based on the de Boor algorithm

    % Implement the generation of the knot vector u based on the de Boor algorithm
    n = length(t_tilde)-1;
    uu = zeros(1, n + p + 2); % m=n+p+1 aber m fängt bei 0 an!
    uu(1+(0):1+(p)) = 0;
    uu(1+(n+1):end) = 1;
    for i = 1:n-p
        uu(1+i+p) = 1/p * sum(t_tilde(1+(i):1+(i+p-1)), 'all');
    end
    knots = uu;
end