function [t_tilde, d] = chordLengthParametrization(Q)
% Chord length parametrization
% Input:
%   Q: Vector of path points Q_j
% Output:
%   t_tilde: Parametrization vector t_tilde_j

[row, n] = size(Q);
n = n-1;
t_tilde = zeros(1, n+1); %t_0 = 0

Q_j   = Q(:,1+(1):1+(n));
Q_jm1 = Q(:,1+(0):1+(n-1));

if(row >= 2)
    d = sum(vecnorm(Q_j-Q_jm1,2));
else
    error('dots Q_j should be at least 2 dimensional!')
end

% TYPO IN Curve interpolation with directional constraints for
% engineering design (2008) und Robotische Assemblierung von 
% Betonringsegmenten im Tunnelbau (2021): Norm wurde vergessen bei chord
% length method, vgl. mit
% Curve Interpolation with Arbitrary End Derivatives (2000)

% Nichtiterative Veriante.
%{
for k = 1:n-1
    t_temp = 0;
    for i=1:k
        t_temp = t_temp + norm(Q(:,1+(i)) - Q(:,1+(i-1)), 2);
    end
    t_tilde(1+(k)) = t_temp/d;
end
t_tilde(1+(n)) = 1;
%}

% iterative Variante
for i = 1:n-1
    t_tilde(1+(i)) = t_tilde(1+(i-1)) + ...
                     + norm(Q(:,1+(i)) - Q(:,1+(i-1)), 2)/d;
end
t_tilde(1+(n)) = 1;

end