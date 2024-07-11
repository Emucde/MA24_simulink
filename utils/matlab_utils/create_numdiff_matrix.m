function D = create_numdiff_matrix(T_a, n, N, variant)
arguments
    T_a (1,1) double
    n (1,1) double
    N (1,1) double
    variant char {mustBeMember(variant, {'fwdbwdcentral', 'bwd', 'savgol'})} = 'fwdbwdcentral'
end

    if(strcmp(variant, 'fwdbwdcentral'))
        % Create the matrix S_v
        E = eye(n);
        S_v = zeros(n * N, n * N);
        S_v(1:n, 1:2*n) = [-2*E 2*E];

        for i = 1:N-2
            S_v(1+n*i:n*(i+1), 1+n*(i-1):n*(i+2)) = [-E zeros(n) E];
        end
        S_v(1+end-n:end, 1+end-2*n:end) = [-2*E 2*E];

        S_v = S_v / (2 * T_a);
        D = S_v;
    elseif(strcmp(variant, 'bwd'))
        % Create the matrix S_v
        E = eye(n);
        S_v = zeros(n * N, n * N);

        for i = 0:N-2
            S_v(1+n*i:n*(i+1), 1+n*i:n*(i+2)) = [-E E];
        end
        S_v(1+end-n:end, 1+end-2*n:end) = [-E E];

        S_v = S_v / T_a;
        D = S_v;
    elseif(strcmp(variant, 'savgol'))
        Nq = 1;
        d = 2;
        DD = create_savgol_deviation_matrices(T_a, Nq, d, N);
        % idea: S_v is first created for one scalar data and then for a n
        % dimensional data it is necessary to use this data multiple time
        DD_vec = cell(1, d);
        for l=1:1:d+1
            S0_v = DD{l};
            S_v = zeros(n*N, n*N);
            for i=1:1:N
                for j=1:1:N
                    S_v((i-1)*n + [1:n], (j-1)*n + [1:n]) = eye(n) * S0_v(i, j);
                end
            end
            DD_vec{l} = S_v;
        end
        D = DD_vec;
    else
        error(['Unknown variant: ' variant, ': choose from ''fwdbwdcentral'', ''savgol''']);
end