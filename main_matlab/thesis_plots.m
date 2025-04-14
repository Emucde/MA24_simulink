if(~exist('parameter_str', 'var'))
    parameters_7dof;
end

% Extract the data for each trajectory
traj_indices = [1, 2, 4, 5];
for traj_idx = traj_indices
    % Extract the data
    time_data = param_traj_data.t(1:10000)';
    p_d_data = param_traj_data.p_d(:,1:10000,traj_idx)';
    q_d_data = param_traj_data.q_d(:,1:10000,traj_idx)';
    
    % Create tables with appropriate column names
    p_d_table = array2table([time_data, p_d_data], 'VariableNames', {'time', 'x', 'y', 'z'});
    q_d_table = array2table([time_data, q_d_data], 'VariableNames', {'time', 'quat1', 'quat2', 'quat3', 'quat4'});
    
    % Save the data as CSV files
    writetable(p_d_table, sprintf('/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter4/p_d_data_traj%d.csv', traj_idx));
    writetable(q_d_table, sprintf('/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter4/q_d_data_traj%d.csv', traj_idx));
end