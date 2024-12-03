%{
figure(1);
data = readmatrix('daten_mpc_Ts_in_1ms_erhoeht.csv');
data = data(:, 2:end);
[num_rows, num_cols] = size(data);

% Anzahl der Subplots bestimmen
num_subplots = num_cols;

for i = 1:num_subplots
    subplot(2, num_cols/2, i);
    plot(2+[1:length(data)], data(:, i));
    title(['Spalte ', num2str(i)]);
    xlabel('Zeilenindex');
    ylabel('Wert');
    xlim([3, 40]);
    grid on;

    % Ermitteln des Schnittpunkts mit der x-Achse (y=0)
    [val, idx] = min(abs(data(:,i)));
    x_zero = idx+2;
    hold on;
    plot([x_zero x_zero], [min(data(:, i)) max(data(:, i))], 'r--');
    hold off;
end
%}

figure(1);
data = readmatrix('daten_mpc_N_in_1ms_erhoehen.csv');
data = data(:, 2:end);
[num_rows, num_cols] = size(data);

title_str = {'T solver (s)', 'T total (s)', 'sum err in x (m)', 'sum err in y (m)', 'sum err in z (m)', 'sum err in rx (m)', 'sum err in ry (m)', 'sum err in rz (m)'};
y_str = {'(s)', '(s)', '(m)', '(m)', '(m)', '(1)', '(1)', '(1)'};
% Anzahl der Subplots bestimmen
num_subplots = num_cols;

for i = 1:num_subplots
    subplot(2, num_cols/2, i);
    semilogy(2+[1:length(data)], data(:, i));
    title(title_str{i});
    xlabel('T horizon (ms) bzw. N MPC Steps (1)');
    ylabel(y_str{i});
    %xlim([3, 40]);
    grid on;

    % Ermitteln des Schnittpunkts mit der x-Achse (y=0)
    [val, idx] = min(abs(data(:,i)));
    x_zero = (2+idx);
    hold on;
    plot([x_zero x_zero], [min(data(:, i)) max(data(:, i))], 'r--');
    hold off;
end

figure(2);
data = readmatrix('daten_mpc_Ts_in_1ms_erhoeht.csv');
data = data(:, 2:end);
[num_rows, num_cols] = size(data);

title_str = {'T solver (s)', 'T total (s)', 'sum err in x (m)', 'sum err in y (m)', 'sum err in z (m)', 'sum err in rx (m)', 'sum err in ry (m)', 'sum err in rz (m)'};
y_str = {'(s)', '(s)', '(m)', '(m)', '(m)', '(1)', '(1)', '(1)'};
% Anzahl der Subplots bestimmen
num_subplots = num_cols;

for i = 1:num_subplots
    subplot(2, num_cols/2, i);
    plot((5+[1:length(data)])*1e-3, data(:, i));
    title(title_str{i});
    xlabel('T horizon (s)');
    ylabel(y_str{i});
    %xlim([3, 40]);
    grid on;

    % Ermitteln des Schnittpunkts mit der x-Achse (y=0)
    [val, idx] = min(abs(data(:,i)));
    x_zero = (5+idx)*1e-3;
    hold on;
    plot([x_zero x_zero], [min(data(:, i)) max(data(:, i))], 'r--');
    hold off;
end