function [DD] = create_savgol_deviation_matrices(Ta, Nq, d, N)
    % Deviation Matrix Calculation with Sarvitzky-Golay Filter
    % Inputs:
    %   Ta: Time step
    %   Nq: Number of previous samples
    %   d: Order of polynomial
    %   N: Size of matrix
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

    % Compute the transpose of the inverse of the product of S and its transpose

    m = -Nq:Nq; % window
    ii = 0:d;

    S = (Ta * m)'.^ii;

    % Fast way to calculate H_T = inv(S' * S) * S';
    H_T = (S' * S) \ S';
    H = H_T';

    %L = chol(S' * S);
    %H_T = L\(L'\S');
    %H = H_T';

    fact = @(n) factorial(n);
    
    DD = cell(1, d+1);
    for l = 0:d
        ii = l:d;
        D = zeros(N, N);
        for m = -Nq:-1
            h_m_i = sum(((m*Ta).^(ii-l) .* fact(ii) ./ fact(ii - l)) .* H(:, ii+1), 2);
            D(m+Nq+1, :) = [h_m_i', zeros(1, N-2*Nq-1)];
        end
        for i = 0:N-2*Nq-1
            D(i+Nq+1, :) = circshift([fact(l) * H(:, l+1)', zeros(1, N-2*Nq-1)], i);
        end
        for m = 1:Nq
            h_m_i = sum(((m*Ta).^(ii-l) .* fact(ii) ./ fact(ii - l)) .* H(:, ii+1), 2);
            D(N - Nq + m, :) = [zeros(1, N-2*Nq-1), h_m_i'];
        end
        DD{l+1} = D;
    end
end