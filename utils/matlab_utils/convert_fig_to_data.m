% Convert a figure to a data structure
% This script reads the data from a figure and saves it in a structured format.
% The data structure contains the signals from each subplot, including their
% names, X and Y data, line styles, and colors.

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

save_path = '/home/rslstudent/Students/Emanuel/crocoddyl_html_files/';
% save_path = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/241012_meeting/mpc_v1_dyn/traj3_T_horizon_5ms';
%file_name = '240904_sim1_traj2_kin_mpc_int_40ms.mat';
file_name = '241218_traj4_ct_control_Ta0_1ms.mat';
save([save_path, '/', file_name], 'signals');