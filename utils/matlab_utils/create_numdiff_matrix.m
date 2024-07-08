function S_v = create_numdiff_matrix(T_a, n, N)
    % Create the matrix S_v
    E = eye(n);
    S_v = zeros(n * N, n * N);
    S_v(1:n, 1:2*n) = [-2*E 2*E];

    for i = 1:N-2
        S_v(1+n*i:n*(i+1), 1+n*(i-1):n*(i+2)) = [-E zeros(n) E];
    end
    S_v(1+end-n:end, 1+end-2*n:end) = [-2*E 2*E];

    S_v = S_v / (2 * T_a);
end