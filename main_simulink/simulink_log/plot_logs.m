% Open the file for reading
%fid = fopen(getlatestfile('./main_simulink/simulink_log'), 'r');
fid = fopen('240604_20_07_MPC_v3_5th_order_polynomial_log.txt', 'r');
%fid = fopen('240605_18_08_MPC_v1_5th_order_polynomial_log.txt', 'r');

file_contents_orig = fscanf(fid, '%c');

fclose(fid);

file_contents = file_contents_orig;

% 1. Alle "|" löschen
file_contents = strrep(file_contents, '|', '');

% 2. Alle ":" löschen
file_contents = strrep(file_contents, ':', '');

% 3. Alle "(" löschen
file_contents = strrep(file_contents, '(', '');

% 4. Alle "s" löschen
file_contents = strrep(file_contents, ' s)', '');
file_contents = strrep(file_contents, ' s ', ' ');

% 5. Alle ")" löschen
file_contents = strrep(file_contents, ')', '');

% 6. Alle "ms" in 1e-3 umwandeln
file_contents = strrep(file_contents, 'ms', 'e-3');

% 7. Alle "us" in 1e-6 umwandeln
file_contents = strrep(file_contents, 'us', 'e-6');

% 8. Alle "ns" in 1e-9 umwandeln
file_contents = strrep(file_contents, 'ns', 'e-9');

% 9. Alle Stellen mit mehr als einem space durch ein einzelnes space ersetzen

% Replace newline characters with temporary markers
file_contents = regexprep(file_contents, '\n+', '~~');

% Replace multiple spaces with a single space using regular expressions
file_contents = regexprep(file_contents, '\s+', ' ');

% Restore newline characters
file_contents = regexprep(file_contents, '~~', '\n');
%disp(file_contents);

split_str = "solver t_proc avg t_wall avg n_eval\n";
split_str_char = char(split_str);

data_mean_split = strsplit(file_contents, 'mean x_err');
data_without_mean_split = strsplit(data_mean_split{2}, split_str);

err_data = ['mean x_err', data_without_mean_split{1}];


after_err_data = strjoin([""+split_str_char(1:end-2), data_without_mean_split{2}], '\n');

file_contents = strjoin([data_mean_split{1}, after_err_data]);

% Daten für jeden Step extrahieren
data_blocks = strsplit(file_contents, split_str);

% 1. Block verwerfen, da hier noch keine Daten vorkommen
data = data_blocks(2:end);

% Zeilenheader erhalten
for i=1:length(data)
    data_init = textscan(data{i}, '%s %s %s %s %s %s');
    data_init = [data_init{1}];
    if(length(data_init) == 7)
        break;
    end
end

% Da im letzten Block oft eine Fehlermeldung ausgegeben wird, werden von
% Diesem nur die ersten 7 Zeilen verwendet, da hier sicher daten stehen.
end_data = data{end};
end_data_split = strsplit(end_data, '\n');
for i=1:length(end_data_split)
    tval = 0;
    for j=1:7
        tval = tval + contains(end_data_split{i}, data_init{j});
    end
    if(tval == 0)
        break;
    end
end

data{end} = strjoin(end_data_split(1:i-1), '\n');


data_init_arr = char(data_init); % 7xX String array

% Create struct
data_log = struct;
for i = 1:7
    data_log.(data_init{i})            = struct;
    data_log.(data_init{i}).t_proc     = zeros(length(data), 1);
    data_log.(data_init{i}).t_proc_avg = zeros(length(data), 1);
    data_log.(data_init{i}).t_wall     = zeros(length(data), 1);
    data_log.(data_init{i}).t_wall_avg = zeros(length(data), 1);
    data_log.(data_init{i}).n_eval     = zeros(length(data), 1);
end


for i = 1:length(data)
    data_i = textscan(data{i}, '%s %s %s %s %s %s');
    data_arr = [data_i{:}];
    for j = 1:size(data_arr, 1)
        idx = find(cellfun(@(x) strcmp(x, data_arr{j}), data_init));

        data_log.(data_init{idx}).t_proc(i)     = str2double(data_arr{j, 2});
        data_log.(data_init{idx}).t_proc_avg(i) = str2double(data_arr{j, 3});
        data_log.(data_init{idx}).t_wall(i)     = str2double(data_arr{j, 4});
        data_log.(data_init{idx}).t_wall_avg(i) = str2double(data_arr{j, 5});
        data_log.(data_init{idx}).n_eval(i)     = str2double(data_arr{j, 6});
    end
end

Ta = 1e-3;
t_log = (0:Ta:(length(data)-1)*Ta)';

figure(1);clf;
sgtitle('t_proc and t_proc_avg for different solvers', 'Interpreter', 'none');
for i = 1:2:14
    idx = round(i/2);
    subplot(7, 2, i);
    plot(t_log, data_log.(data_init{idx}).t_proc);
    title([data_init{idx}, ': t_proc'], 'Interpreter', 'none');
    ylabel('t (s)');

    subplot(7, 2, i+1);
    plot(t_log, data_log.(data_init{idx}).t_proc_avg);
    title([data_init{idx}, ': t_proc_avg'], 'Interpreter', 'none');
    ylabel('t (s)');
end
xlabel('Sim time t (s)');

figure(2);clf;
sgtitle('t_wall and t_wall_avg for different solvers', 'Interpreter', 'none');
for i = 1:2:14
    idx = round(i/2);
    subplot(7, 2, i);
    plot(t_log, data_log.(data_init{idx}).t_wall);
    title([data_init{idx}, ': t_wall'], 'Interpreter', 'none');
    ylabel('t (s)');

    subplot(7, 2, i+1);
    plot(t_log, data_log.(data_init{idx}).t_wall_avg);
    title([data_init{idx}, ': t_wall_avg'], 'Interpreter', 'none');
    ylabel('t (s)');
end
xlabel('Sim time t (s)');

figure(3);clf;
sgtitle('n_eval for different solvers', 'Interpreter', 'none');
for i = 1:7
    subplot(7, 1, i);
    plot(t_log, data_log.(data_init{i}).n_eval);
    title([data_init{i}, ': n_eval'], 'Interpreter', 'none');
    ylabel('n');
end
xlabel('Sim time t (s)');

% Weitere Plots: Zeiten Pro Step und Zeit

% Die Daten scheinen obwohl sie pro step verfügbar sind bei den Zeiten nicht sehr genau zu sein.
% D. h. die Zeit wird scheinbar nicht jede param_globa.Ta Abtastzeit aktualisiert und es gibt
% Plateaus. Daher interpoliere ich die Daten an diesen Stellen.

% Interpoliere die Daten
t_log_interp = (0:Ta/10:(length(data)-1)*Ta)';

for i = 1:7
    data_log.(data_init{i}).t_proc_interp = interp1(t_log, data_log.(data_init{i}).t_proc, t_log_interp);
    data_log.(data_init{i}).t_proc_avg_interp = interp1(t_log, data_log.(data_init{i}).t_proc_avg, t_log_interp);
    data_log.(data_init{i}).t_wall_interp = interp1(t_log, data_log.(data_init{i}).t_wall, t_log_interp);
    data_log.(data_init{i}).t_wall_avg_interp = interp1(t_log, data_log.(data_init{i}).t_wall_avg, t_log_interp);
    data_log.(data_init{i}).n_eval_interp = interp1(t_log, data_log.(data_init{i}).n_eval, t_log_interp);
end

figure(4); clf;
sgtitle('t_proc/(sampling step) for different solvers', 'Interpreter', 'none');
for i = 1:2
    subplot(4, 2, i);
    plot(t_log, data_log.(data_init{i}).t_proc);
    title([data_init{i}, ': t_proc/(sampling step)'], 'Interpreter', 'none');
    ylabel('(s)');
end
for i = 3:6
    subplot(4, 2, i);
    t_proc_intp = intp_data( data_log.(data_init{i}).t_proc );

    dt = t_proc_intp(2:end) - t_proc_intp(1:end-1);

    dt_circ = circshift(dt,1); % shifte linke nach rechts
    dt(dt < 0) = dt_circ(dt < 0); % nimm den linken

    Ndt = length(dt);
    plot(t_log(1:Ndt), dt);
    title([data_init{i}, ': t_proc/(sampling step)'], 'Interpreter', 'none');
    ylabel('(s)');
    xlabel('Sim time t (s)');
end
i=7;
subplot(4, 2, i);
plot(t_log, data_log.(data_init{i}).t_proc);
title([data_init{i}, ': t_proc/(sampling step)'], 'Interpreter', 'none');
ylabel('(s)');
xlabel('Sim time t (s)');
%%
figure(5); clf;
sgtitle('t_wall/(sampling step) for different solvers', 'Interpreter', 'none');
for i = 1:2
    subplot(4, 2, i);
    plot(t_log, data_log.(data_init{i}).t_wall);
    title([data_init{i}, ': t_wall/(sampling step)'], 'Interpreter', 'none');
    ylabel('(s)');
end
for i = 3:6
    subplot(4, 2, i);
    t_wall_intp = intp_data( data_log.(data_init{i}).t_wall );

    dt = t_wall_intp(2:end) - t_wall_intp(1:end-1);

    dt_circ = circshift(dt,1); % shifte linke nach rechts
    dt(dt < 0) = dt_circ(dt < 0); % nimm den linken

    Ndt = length(dt);
    plot(t_log(1:Ndt), dt);
    title([data_init{i}, ': t_wall/(sampling step)'], 'Interpreter', 'none');
    ylabel('(s)');
    xlabel('Sim time t (s)');
end
i=7;
subplot(4, 2, i);
plot(t_log, data_log.(data_init{i}).t_wall);
title([data_init{i}, ': t_wall/(sampling step)'], 'Interpreter', 'none');
ylabel('(s)');
xlabel('Sim time t (s)');
%%
NN = length(t_log)-1;
n_total = zeros(NN, 1);
figure(6); clf;
sgtitle('n_eval/(sampling step) for different solvers', 'Interpreter', 'none');
for i = 1:2
    subplot(4, 2, i);
    n_eval_data = data_log.(data_init{i}).n_eval;
    for j=2:length(n_eval_data)
        if(n_eval_data(j) == 0)
           n_eval_data(j) = n_eval_data(j-1);
        end
    end
    plot(t_log, n_eval_data);
    title([data_init{i}, ': n_eval/(sampling step)'], 'Interpreter', 'none');

    n_total(1:NN) = n_total(1:NN) + n_eval_data(1:NN);
end
for i = 3:6
    subplot(4, 2, i);
    if( i ~= 4 && i~=6)
        n_eval_intp = intp_data( data_log.(data_init{i}).n_eval );
    else
        n_eval_intp = data_log.(data_init{i}).n_eval;
    end
    
    dt = n_eval_intp(2:end) - n_eval_intp(1:end-1);
    dt = ceil(dt); % steps have to be integer

    dt_circ = circshift(dt,1); % shifte linke nach rechts
    dt(dt < 0) = dt_circ(dt < 0); % nimm den linken

    Ndt = length(dt);
    plot(t_log(1:Ndt), dt);
    title([data_init{i}, ': n_eval/(sampling step)'], 'Interpreter', 'none');
    n_total(1:Ndt) = n_total(1:Ndt) + dt(1:Ndt);
end
xlabel('Sim time t (s)');

i=7; % total nsteps ist einfach falsch. Daher einfach alles aufaddieren
subplot(4, 2, i);
plot(t_log(2:end), n_total);
title([data_init{i}, ': n_eval/(sampling step)'], 'Interpreter', 'none');
xlabel('Sim time t (s)');




function vec_out = intp_data(vec_in)
    N = length(vec_in)-1;
    vec_out = zeros(N, 1);
    i=1;
    while(i < N)
        idx = find(vec_in(i) < vec_in(i+1 : end), 1);
        if(isempty(idx))
            i = i-1;
            idx = N-i+1;
            vec_out(i:i+idx) = linspace(vec_out(i), vec_in(i+idx), idx+1);
            break
        elseif(idx > 1)
            vec_out(i:i+idx) = linspace(vec_in(i), vec_in(i+idx), idx+1);
            i = i + idx;
        else
            vec_out(i) = vec_in(i);
            i=i+1;
        end
    end
end