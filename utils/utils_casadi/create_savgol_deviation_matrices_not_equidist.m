function [DD] = create_savgol_deviation_matrices_not_equidist(Ta, Nq, d, N, sample_indices)
    % Deviation Matrix Calculation with Sarvitzky-Golay Filter
    % Inputs:
    %   Ta: Time step
    %   Nq: Number of previous samples
    %   d: Order of polynomial
    %   N: Size of matrix
    %   sample_indices: Time indices of data. If sample_indices is multiplied by 
    %                   the sampling time Ta, you get the times at which the samples were sampled
    %
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

    fact = @(n) factorial(n);

    % Skizze: Beispiel für Fenster im Fall Nq=1:
    %
    % Ta    Ta_MPC-Ta        Ta_MPC             Ta_MPC                   Ta_MPC          Ta_MPC
    % <--><------------><----------------><---------------->  ...  <---------------><--------------->
    % |---|-------------|-----------------|-----------------| ...  |----------------|----------------|--------> t
    % 0  Ta           Ta_MPC         2Ta_MPC           3Ta_MPC   (N-3)Ta_MPC    (N-2)Ta_MPC    (N-1)Ta_MPC        
    % q0  q1            q2                q3                q4    qN-2              qN              qN
    %     |             |                 |                 |      |                |                |
    % ------------------|                 |                 |      |                |                |
    % [   Fenster 1    ]|                 |                 |      |                |                |
    % [-1,0,          4]|                 |                 |      |                |                |
    % ------------------|                 |                 |      |                |                |
    %     |             |                 |                 |      |                |                |
    %     |-------------------------------|                 |      |                |                |
    %     |[         Fenster 2           ]|                 |      |                |                |
    %     |[-4,         0,              5]|                 |      |                |                |
    %     |-------------------------------|                 |      |                |                |
    %     |             |                 |                 |      |                |                |
    %     |             |-----------------------------------|      |                |                |
    %     |             |[            Fenster 3            ]|      |                |                |
    %     |             |[-5,             0,              5]|      |                |                |
    %     |             |-----------------------------------|      |                |                |
    %     |             |                 |                 |      |                |                |
    %     |             |                 |                ...    ...              ...              ...
    %     |             |                 |                 |      |                |                |
    %     |             |                 |                 |      |---------------------------------|
    %     |             |                 |                 |      |[           Fenster N           ]|
    %     |             |                 |                 |      |[-5,            0,             5]|
    %     |             |                 |                 |      |---------------------------------|
    %
    % Ta ... Abtastzeit des CT Reglers und Sensoren
    % Ta_MPC ... Interne Abtastzeit der MPC: So grob wird Trajektorie abgetastet
    % 
    % Beim Sarvitsky Golay filter wird bekanntlich in jedem Fenster ein eigenes Polynom approximiert.
    % Indem dieses abgeleitet wird, kann auch ein Ableitungsfilter berechnet werden.
    % Besondere Vorsicht aber auf die Polynomordnung: Ist diese 2 oder kleiner, ist die zweite Ableitung
    % konstant. Möchte man so wie hier Randpunkte approximieren, sind diese sehr ungenau, da sie an den
    % dann auch an den Rändern konstant gehalten werden. Das ist insbesonders bei großen Nq ein riesiges Problem!
    % Daher ist es dann genauer die Randpunkte über numerisches Differenzieren zu berechnen.
    %
    % Beispiel für Fenster im Fall von Ta_MPC = 5 * Ta:
    % für Nq=1:
    % windows_Nq_is_1 = {[-1,0,4], [-4,0,5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5], [-5, 0, 5]};
    % für Nq=2
    windows_Nq_is_2 = {[-5, -4, 0, 5, 10], [-9, -5, 0, 5, 10], [-10, -5, 0, 5, 10], [-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10],[-10, -5, 0, 5, 10]};
    %
    % das ist eben der grundlegende Unterschied zur äquidistanten Variante, dort wird ja immer m = -Nq:Nq verwendet und damit wird immer der Bereich
    % [-Nq, -Nq+1, -Nq+2, ..., 0, Nq, Nq+1, Nq+2, ...] verwendet und anschließend mit Ta multiplziert. Das beudetet, man hätte immer die Fenster
    % Nq = 1:
    % windows_Nq_is_1 = {[-1,0,1], [1,0,1], [1,0,1], [1,0,1], [1,0,1], [1,0,1], ...}
    % oder
    % Nq = 2:
    % windows_Nq_is_2 = {[-2,-1,0,1,2], [-2,-1,0,1,2], [-2,-1,0,1,2], [-2,-1,0,1,2], [-2,-1,0,1,2], ...}
    %
    % im Endeffekt nix anders als https://dsp.stackexchange.com/questions/1676/savitzky-golay-smoothing-filter-for-not-equally-spaced-data
    % vgl. die S Berechnung unten, nur dass ich halt noch die Grenzpunkte links und rechts approximiere
    % https://pubs.acs.org/doi/abs/10.1021/ac00005a031

    % Initialize windows cell array
    window_arr = cell(1, length(sample_indices)-2*Nq);

    % Calculate windows
    m = -Nq:Nq;
    for i = 1:length(window_arr)
        window_arr{i} = sample_indices(i+Nq+m) - sample_indices(Nq+i);
    end

    DD = cell(1, d+1);
    for l = 0:d
        ii = l:d;
        D = zeros(N, N);
        window = window_arr{1};
        H = create_SH_matrices(d, Ta, window);
        for m = -Nq:-1
            h_m_i = sum(((Ta*window(1+Nq+m)).^(ii-l) .* fact(ii) ./ fact(ii - l)) .* H(:, ii+1), 2);
            D(m+Nq+1, :) = [h_m_i', zeros(1, N-2*Nq-1)];
        end
        for i = 0:N-2*Nq-1
            window = window_arr{1+i};
            H = create_SH_matrices(d, Ta, window);
            D(i+Nq+1, :) = circshift([fact(l) * H(:, l+1)', zeros(1, N-2*Nq-1)], i);
        end
        window = window_arr{end};
        H = create_SH_matrices(d, Ta, window);
        for m = 1:Nq
            h_m_i = sum(((Ta*window(1+Nq+m)).^(ii-l) .* fact(ii) ./ fact(ii - l)) .* H(:, ii+1), 2);
            D(N - Nq + m, :) = [zeros(1, N-2*Nq-1), h_m_i'];
        end
        DD{l+1} = D;
    end
end

function H = create_SH_matrices(d, Ta, sample_indices)
    ii = 0:d;

    S = (Ta * sample_indices)'.^ii;

    % Fast way to calculate H_T = inv(S' * S) * S';
    H_T = (S' * S) \ S';
    H = H_T';

    %L = chol(S' * S);
    %H_T = L\(L'\S');
    %H = H_T';

end