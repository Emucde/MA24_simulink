% Simulink: convert plots to figure:

fig = gcf;
%dataObjs = findobj(fig,'-property','YData'); % unsorted reading
%datalen = length(dataObjs);

fig_subplots = flip(fig.Children.Children);
subplot_number = length(fig_subplots);

signals = repmat({0}, 1, subplot_number)';
N_dec = 1; % read only each 100th sample
for i=1:subplot_number
    subplot_data = flip(findobj(fig_subplots(i),'-property','YData'));
    signal_number = length(subplot_data);
    signals{i} = repmat({""; 0; 0; ""; [0 0 0]}, 1, signal_number)';
    for j=1:signal_number
        signals{i}{j,1} = subplot_data(j).DisplayName;
        signals{i}{j,2} = subplot_data(j).XData(1:N_dec:end);
        signals{i}{j,3} = subplot_data(j).YData(1:N_dec:end);
        signals{i}{j,4} = subplot_data(j).LineStyle;
        signals{i}{j,5} = subplot_data(j).Color;
    end
end

save_path = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240904_meeting/kin_mpc_dev';
%file_name = '240826_messung4_traj1_ct.mat';
%file_name = '240826_messung12_traj6_mpc1_dyn_qpp_weight_with_limits.mat';
file_name = '240904_sim1_traj2_kin_mpc_dev.mat';
save([save_path, '/', file_name], 'signals');