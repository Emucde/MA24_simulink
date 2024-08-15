function [CC] = spline_fun_comb(theta, k, control_points, knots, degree)
% spline_fun calculates the value of a combined B-spline curve at a given point theta.
% Inputs:
%   theta: scalar value representing the point at which to evaluate the curve.
%   k: scalar value representing the deviations that should be calculated
%       bspline_comb: Struct containing the combined information with fields
%           'degree', 'knot', 'control_points', 'start_knot',
%           'end_knot', 'start_control_point', 'end_control_point'.
%       see combine_bsplines.m for more infos.
%
% Outputs:
%   CC: column vector of size (m x 1), C = [C(theta), C'(theta), ..., C'k(theta)]
% the value of the B-spline curve at theta, where m is the number of dimensions of the control points.

    i = bspline_findspan_comb(theta, knots, degree);
    CC = control_points(1+(i-degree):1+(i),:)' * bspline_basisfunction_comb(theta,i,k,knots, degree)';
end