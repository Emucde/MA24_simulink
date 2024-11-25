function [DD] = create_savgol_deviation_matrices_not_equidist(Ta, Nq, d, N, sample_indices)
    % Deviation Matrix Calculation with Sarvitzky-Golay Filter
    % Inputs:
    %   Ta: Time step
    %   Nq: Number of previous samples
    %   d: Order of polynomial
    %   N: Size of matrix
    %   sample_dist_vector: distance of samples, can be equidstant or not
    % Outputs:
    %   DD: Matrix for deviation calculation
    %
    % Formula:
    % $\hat{\mathbf y}^{(l)} [k] = 
    % \begin{bmatrix} 
    %   \mathbf 0_{(2N_q+1)\times {l}} &\mathbf s_0&\mathbf s_1&\mathbf s_2&\cdots &\mathbf s_{d-l}
    % \end{bmatrix}
    % \text{diag}
    % \bigg(
    %   \begin{bmatrix}
    %     \mathbf 0_{1\times l} & l! & (l+1)! & \frac{(l+2)!}{2!} & \cdots & \frac{(d)!}{d-l!}
    %   \end{bmatrix}
    % \bigg)
    % \mathbf H^\mathrm T \mathbf y[k]$
    %e
    % ŷ⁽ˡ⁾[k] = [ 𝟎₍2Nₓ+1₎ₓₗ  s₀ s₁ s₂ ⋯ s₍d-l₎ ] diag( [ 𝟎₁ₓₗ  l! (l+1)! (l+2)!/2! ⋯ d!/(d-l)! ] ) Hᵀ y[k]

    % Compute the transpose of the inverse of the product of S and its transpose

    %L = chol(S' * S);
    %H_T = L\(L'\S');
    %H = H_T';

    fact = @(n) factorial(n);
    
    DD = cell(1, d+1);
    for l = 0:d
        ii = l:d;
        D = zeros(N, N);
        H = create_SH_matrices(Nq, d, Ta, sample_indices, 1);
        for m = -Nq:-1
            h_m_i = sum(((m*Ta*sample_indices(1+Nq+m)).^(ii-l) .* fact(ii) ./ fact(ii - l)) .* H(:, ii+1), 2);
            D(m+Nq+1, :) = [h_m_i', zeros(1, N-2*Nq-1)];
        end
        for i = 0:N-2*Nq-1
            H = create_SH_matrices(Nq, d, Ta, sample_indices, 1+i);
            D(i+Nq+1, :) = circshift([fact(l) * H(:, l+1)', zeros(1, N-2*Nq-1)], i);
        end
        H = create_SH_matrices(Nq, d, Ta, sample_indices, N-2*Nq);
        for m = 1:Nq
            h_m_i = sum(((m*Ta*sample_indices(1+Nq+m)).^(ii-l) .* fact(ii) ./ fact(ii - l)) .* H(:, ii+1), 2);
            D(N - Nq + m, :) = [zeros(1, N-2*Nq-1), h_m_i'];
        end
        DD{l+1} = D;
    end
end

function H = create_SH_matrices(Nq, d, Ta, sample_indices, i)
    m = -Nq:Nq; % window
    ii = 0:d;

    S = (Ta * sample_indices(i:i+2*Nq) .* m)'.^ii;

    % Fast way to calculate H_T = inv(S' * S) * S';
    H_T = (S' * S) \ S';
    H = H_T';
end