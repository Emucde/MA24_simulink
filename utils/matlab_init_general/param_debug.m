% Sarvitsky Golay Filter for mean of data.

N_data = 301; % should be odd
N_window = floor((N_data-1)/2);
DD = create_savgol_deviation_matrices(param_global.Ta, N_window, 2, N_data);
QQ = DD{1};
param_savgol.bT = QQ(1+N_window, :); % only use middlepoint filter
%param_savgol.bT = QQ(1, :); % only use middlepoint filter
param_savgol.N = N_data;