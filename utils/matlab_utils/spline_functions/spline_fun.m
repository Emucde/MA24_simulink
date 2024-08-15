function [CC] = spline_fun(theta, k, bspline)
% spline_fun calculates the value of a B-spline curve at a given point theta.
% Inputs:
%   theta: scalar value representing the point at which to evaluate the curve.
%   k: scalar value representing the deviations that should be calculated
%   bspline: struct containing the control points and knot vector of the B-spline curve.
%
% Outputs:
%   CC: column vector of size (m x 1), C = [C(theta), C'(theta), ..., C'k(theta)]
% the value of the B-spline curve at theta, where m is the number of dimensions of the control points.

    %i = bspline_findspan(theta, bspline);

    N = sum(bspline.knot==0);
    knots = bspline.knot(1+N:length(bspline.knot)-N);
    i = sum(knots<=theta)+N-1;

    p = bspline.degree;
    control_points = bspline.control_points;
    CC = control_points(1+(i-p):1+(i),:)' * bspline_basisfunction(theta,i,k,bspline)';
end