if(~exist('parameter_str', 'var'))
    parameters_7dof;
end

steps = 1000;

% Extract the data for each trajectory
traj_indices = [1, 2, 4, 5];
for traj_idx = traj_indices
    % Extract the data
    time_data = param_traj_data.t(1:steps:10000)';
    p_d_data = param_traj_data.p_d(:,1:steps:10000,traj_idx)';
    q_d_data = param_traj_data.q_d(:,1:steps:10000,traj_idx)';
    
    % Create tables with appropriate column names
    p_d_table = array2table([time_data, p_d_data], 'VariableNames', {'time', 'x', 'y', 'z'});
    q_d_table = array2table([time_data, q_d_data], 'VariableNames', {'time', 'quat1', 'quat2', 'quat3', 'quat4'});
    
    % Save the data as CSV files
    writetable(p_d_table, sprintf('/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter4/p_d_data_traj%d.csv', traj_idx));
    writetable(q_d_table, sprintf('/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter4/q_d_data_traj%d.csv', traj_idx));
end

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250309/Traj1/ID_damping/robot_plots_20250415_094713.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj1_ID_damping.csv', ...
    steps ...
    );

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250309/Traj1/ID_ThresholdSmallSingularValues/robot_plots_20250415_101140.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj1_ID_ThresholdSmallSingularValues.csv', ...
    steps ...
);

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250309/Traj1/Crocoddyl_NMPC_20_N_step_1/robot_plots_20250421_113019.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj1_Crocoddyl_NMPC_20_N_step_1.csv', ...
    steps ...
);

function export_selected_columns(input_csv, output_csv, steps)
    data = readtable(input_csv, 'VariableNamingRule', 'preserve');
    exportTable = table();
    exportTable.time = data.("t (s)")(1:steps:10000);
    exportTable.e_x = 1e3*data.("e_x (m)")(2001:steps:end);
    exportTable.e_y = 1e3*data.("e_y (m)")(2001:steps:end);
    exportTable.e_z = 1e3*data.("e_z (m)")(2001:steps:end);
    exportTable.quat_err_2 = data.("quat_err_2")(2001:steps:end);
    exportTable.quat_err_3 = data.("quat_err_3")(2001:steps:end);
    exportTable.quat_err_4 = data.("quat_err_4")(2001:steps:end);
    exportTable.manip = data.("Manip. w = √( det( JJ⸆ ) )")(2001:steps:end);
    exportTable.qd_1 = data.("q̇_1")(2001:steps:end);
    exportTable.qd_2 = data.("q̇_2")(2001:steps:end);
    exportTable.qd_3 = data.("q̇_3")(2001:steps:end);
    exportTable.qd_4 = data.("q̇_4")(2001:steps:end);
    exportTable.qd_5 = data.("q̇_5")(2001:steps:end);
    exportTable.qd_6 = data.("q̇_6")(2001:steps:end);
    exportTable.qd_7 = data.("q̇_7")(2001:steps:end);
    exportTable.tau_1 = data.("tau_1")(2001:steps:end);
    exportTable.tau_2 = data.("tau_2")(2001:steps:end);
    exportTable.tau_3 = data.("tau_3")(2001:steps:end);
    exportTable.tau_4 = data.("tau_4")(2001:steps:end);
    exportTable.tau_5 = data.("tau_5")(2001:steps:end);
    exportTable.tau_6 = data.("tau_6")(2001:steps:end);
    exportTable.tau_7 = data.("tau_7")(2001:steps:end);
    writetable(exportTable, output_csv);
end