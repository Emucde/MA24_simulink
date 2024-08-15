function [N, dN] = bsplineBasis(i, p, u, knots)
% Function to compute the B-spline basis function N_{i,p}(u) of order p
% Inputs:
%   i: Control point index (0,...,n)
%   p: Order of the basis function
%   u: Interpolation parameter
%   knots: Knot vector (e.g., [u0, u1, ..., um])
% Output:
%   N: The B-spline basis function N_{i,p}(u)

% Implement the B-spline basis function computation here
% Refer to the provided formulas (2.43) for Ni,p (u) and Ni,0 (u)
% from Ebmer 2021 (in this work are typos inside!!!) and after Algorithm
% 2.4 from thenurbs book 2nd ed.

% This recursive algorithm is very ineffective in run time. Please use
% bspline_basisfunction and bspline_findspan instead of this, they are
% much, much faster. This algorithm is only used to caclulate the desired
% control points P_i.

% legend
% u_i = u_{i}
% u_ip1 = u_{i+1}
% u_ipp = u_{i+p}
% u_ippp1 = u_{i+p+1}
%knots = [u_0, u_1, ..., u_m] = [knots(1+(0)), knots(1+(1)), ..., knots(1+(m))]

    u_i = knots(1+(i));
    u_ip1 = knots(1+(i+1));
    u_ipp = knots(1+(i+p));
    u_ippp1 = knots(1+(i+p+1));

    if(p <= 0)
        if(u == u_ip1 && u_ip1 == knots(end))
            % Special case: if u == u_ip1 == knots(end) the equation in the
            % else branch would always lead to 0. But in this special case
            % the result should be N=1, see ALGORITHM A2.4 from TheNurbs
            % Book in special cases. The second special case
            % u == knots(0) && i == 0 is implicit fulfilled in the else
            % expression due to the u >= u_i
            N = 1;
        else
            % Typo in ebmer work: there is no u <= u_ip1 case!
            N = double(u >= u_i && u < u_ip1);
        end
    else
        if(u_ipp == u_i)
            % if u_ipp == u_i then N_i_pm1 = 0
            w1 = 0;
        else
            N_i_pm1 = bsplineBasis(i,p-1,u,knots);
            w1 = N_i_pm1*(u-u_i)/(u_ipp-u_i);
        end

        if(u_ippp1 == u_ip1)
            % if u_ippp1 == u_ip1 then N_ip1_pm1 = 0
            w2 = 0;
        else
            N_ip1_pm1 = bsplineBasis(i+1,p-1,u,knots);
            w2 = N_ip1_pm1*(u_ippp1-u)/(u_ippp1 - u_ip1);
        end

        N = w1 + w2;
    end

    if(u_ipp == u_i)
        w3 = 0;
    else
        N_i_pm1 = bsplineBasis(i,p-1,u,knots);
        w3 = N_i_pm1/(u_ipp - u_i);
    end
    if(u_ippp1 == u_ip1)
        w4 = 0;
    else
        N_ip1_pm1 = bsplineBasis(i+1,p-1,u,knots);
        w4 = N_ip1_pm1/(u_ippp1 - u_ip1);
    end
    dN = p*(w3 - w4);
end