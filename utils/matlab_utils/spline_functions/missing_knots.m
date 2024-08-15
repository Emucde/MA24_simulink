function [a, b] = missing_knots(t, p, Ind, Ind_deriv)
% Calculates the missing knots u_0, ..., u_m with m=n+k+p+2 knots
% Inputs:
%   t0: starting parameter
%   tn: ending parameter
%   p: degree
%   Ind_0: index of the starting direction
%   Ind_k: index of the ending direction
% Outputs:
%   u: missing knots
% 
% Example usage:
%   u = calculate_missing_knots(0, 1, 2, 1, 3);

%% Compute the knots without tangents preset

n = length(t)-1; % Es gilt t = [t_0, t_1, ..., t_n]
k = length(Ind)-1; % Es gilt Ind = [Ind_0, Ind_1, ..., Ind_k]

if Ind(1+(0)) == 0
    is = 0;
else
    is = 1;
end

if Ind(1+(k)) == n
    ie = n-p+1;
else
    ie = n-p;
end

a(1+(0)) = t(1+(0));
r = 0;

for i=is:ie
    sum = 0;
    r=r+1;
    for j= i:i+p-1
        sum = sum+t(1+(j));
    end
    a(1+(r)) = sum/p;
end
r=r+1;
a(1+(r)) = t(1+(n));

%% Add extra knots where derivatives are present

s = -1;
for i=0:r-1
    c(1+(0)) = a(1+(i)); % = c_0
    indices = find(t >= a(1+(i)) & t < a(1+(i+1))); % = I_1,...,I_{l-1} (indexes of parameters) Important: starting with 1 in matlab *1
    l = length(indices) + 1;
    c( (1+(1)) : (1+(l-1)) ) = t(indices); % parameters c_1,...c_{l-1} inside [a_i, a_{i+1})
    c(1+(l)) = a(1+(i+1)); % = c_l

    for j = 0:l-2
        if any(indices(j+1)-1 == Ind_deriv) % *1: Ind_deriv starting with 0 as index! 
            s=s+1;
            b(1+(s)) = (c(1+(j)) + c(1+(j+1)) + c(1+(j+2)))/3;
        end
    end
end

% special case: Please be aware, that D0 and Dr have to be defined but
% their indices should not be in Ind_deriv!
if isempty(Ind_deriv)
    b = [];
end