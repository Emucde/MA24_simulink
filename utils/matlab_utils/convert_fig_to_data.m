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

%save_path = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/traj3_poly_ellbow1/CT_mit_singreg';
%save_path = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/traj3_poly_ellbow1/MPC_v1_dyn';
save_path = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/traj3_poly_ellbow1/MPC_v3_kin_int';
%save_path = '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/traj3_poly_ellbow1/MPC_v3_kin_int_laenger_praed';
%file_name = '240826_messung7_traj3_ct.mat';
%file_name = '240826_messung8_traj3_mpc1_dyn.mat';
file_name = '240826_messung9_traj3_mpc8_kin_int.mat';
%file_name = '240826_messung10_traj3_mpc8_kin_int_laenger_praed.mat';
save([save_path, '/', file_name], 'signals');