if(~exist('parameter_str', 'var'))
    parameters_7dof;
end

steps = 10;

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
    writetable(p_d_table, sprintf('/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter3/p_d_data_traj%d.csv', traj_idx));
    writetable(q_d_table, sprintf('/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter3/q_d_data_traj%d.csv', traj_idx));
end

% TRAJ 1 ID Damping
disp('Exporting Traj 1 ID Damping data...');

export_selected_columns_ID( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250309/Traj1/ID_damping/robot_plots_20250415_094713.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj1_ID_damping.csv', ...
    steps ...
    );

export_selected_columns_ID( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250309/Traj1/ID_damping_Simulation/robot_plots_20250422_110845.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj1_ID_damping_simulation.csv', ...
    steps ...
    );

% TRAJ 1 ID ThresholdSmallSingularValues

export_selected_columns_ID( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250309/Traj1/ID_ThresholdSmallSingularValues/robot_plots_20250415_101140.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj1_ID_ThresholdSmallSingularValues.csv', ...
    steps ...
);

export_selected_columns_ID( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250309/Traj1/ID_ThresholdSmallSingularValues_Simulation/robot_plots_20250422_110939.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj1_ID_ThresholdSmallSingularValues_simulation.csv', ...
    steps ...
);

% TRAJ 1 Crocoddyl NMPC 20 N step 1

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj1/Crocoddyl_N_step_1_N_MPC_20_yr1e5_v3_best/robot_plots_20250503_154144.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj1_Crocoddyl_NMPC_20_N_step_1.csv', ...
    steps ...
);

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj1/Crocoddyl_N_step_1_N_MPC_20_yr1e5_simulation/robot_plots_20250503_154329.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj1_Crocoddyl_NMPC_20_N_step_1_simulation.csv', ...
    steps ...
);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TRAJ 2 ID Damping

disp('Exporting Traj 2 ID Damping data...');

export_selected_columns_ID( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250309/Traj2/ID_damping/robot_plots_20250425_103626.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj2_ID_damping.csv', ...
    steps ...
    );

export_selected_columns_ID( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250309/Traj2/ID_damping_Simulation/robot_plots_20250422_112103.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj2_ID_damping_simulation.csv', ...
    steps ...
    );

% TRAJ 2 ID ThresholdSmallSingularValues

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250716/traj2/ID_threshholding_1/robot_plots_20250718_140117.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj2_ID_ThresholdSmallSingularValues.csv', ...
    steps ...
);

export_selected_columns_ID( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250309/Traj2/ID_ThresholdSmallSingularValues_Simulation/robot_plots_20250422_112143.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj2_ID_ThresholdSmallSingularValues_simulation.csv', ...
    steps ...
);

% TRAJ 2 Crocoddyl NMPC 20 N step 1

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj2/Crocoddyl_N_step_1_N_MPC_20_yr1e5_q_x_2e0_v4_best/robot_plots_20250503_154642.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj2_Crocoddyl_NMPC_20_N_step_1.csv', ...
    steps ...
);

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj2/Crocoddyl_N_step_1_N_MPC_20_yr1e5_q_x_2e0_simulation/robot_plots_20250503_154733.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj2_Crocoddyl_NMPC_20_N_step_1_simulation.csv', ...
    steps ...
);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TRAJ 4 ID Damping

disp('Exporting Traj 4 ID Damping data...');

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250716/traj3/Damping_3_best/robot_plots_20250718_144134.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj4_ID_damping.csv', ...
    steps ...
    );

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250716/traj3/Damping_simulation/robot_plots_20250718_161659.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj4_ID_damping_simulation.csv', ...
    steps ...
    );

% TRAJ 4 ID ThresholdSmallSingularValues

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250716/traj3/Thresholding_eps_1_v2/robot_plots_20250718_144354.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj4_ID_ThresholdSmallSingularValues.csv', ...
    steps ...
);

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250716/traj3/Thresholding_eps_1_simulation/robot_plots_20250718_161759.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj4_ID_ThresholdSmallSingularValues_simulation.csv', ...
    steps ...
);

% TRAJ 4 Crocoddyl NMPC 20 N step 1

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj4/Crocoddyl_N_step_1_N_MPC_20_yr1e5_v5_best/robot_plots_20250505_123517.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj4_Crocoddyl_NMPC_20_N_step_1.csv', ...
    steps ...
);

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj4/Crocoddyl_N_step_1_N_MPC_20_yr1e5_simulation/robot_plots_20250503_160938.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj4_Crocoddyl_NMPC_20_N_step_1_simulation.csv', ...
    steps ...
);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TRAJ 5 ID Damping

disp('Exporting Traj 5 ID Damping data...');

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250716/traj4/Damping_101010/robot_plots_20250720_132008.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj5_ID_damping.csv', ...
    steps ...
    );

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250716/traj4/Damping_101010_simulation/robot_plots_20250720_131943.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj5_ID_damping_simulation.csv', ...
    steps ...
    );

% TRAJ 5 ID ThresholdSmallSingularValues

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250716/traj4/Threshholding_eps_1_101010_best/robot_plots_20250720_132045.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj5_ID_ThresholdSmallSingularValues.csv', ...
    steps ...
);

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250716/traj4/Threshholding_eps_1_101010_simulation/robot_plots_20250720_132031.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj5_ID_ThresholdSmallSingularValues_simulation.csv', ...
    steps ...
);

% TRAJ 5 Crocoddyl NMPC 20 N step 1

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj5/Crocoddyl_N_step_1_N_MPC_20_yr1e3_v7_best/robot_plots_20250505_161447.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj5_Crocoddyl_NMPC_20_N_step_1.csv', ...
    steps ...
);

export_selected_columns( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250503/traj5/Crocoddyl_N_step_1_N_MPC_20_yr1e3_simulation/robot_plots_20250503_161425.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter5/Traj5_Crocoddyl_NMPC_20_N_step_1_simulation.csv', ...
    steps ...
);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATIONEN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MPC1_dyn_classic_N_MPC_5 - Casadi
export_selected_columns_simus( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250602/traj1/MPC1_dyn_classic_N_MPC_5/robot_plots_20250603_110116.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter4/MPC1_dyn_classic_N_MPC_5.csv', ...
    steps ...
);

% Crocoddyl_N_step_1_N_MPC_5 - Crocoddyl
export_selected_columns_simus( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250528/traj1/Crocoddyl_N_step_1_N_MPC_5/robot_plots_20250530_134355.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter4/Crocoddyl_N_step_1_N_MPC_5.csv', ...
    steps ...
);

% MPC2_dyn_parametric_N_MPC_5 - Casadi
export_selected_columns_simus( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250602/traj1/MPC2_dyn_parametric_N_MPC_5/robot_plots_20250603_110316.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter4/MPC2_dyn_parametric_N_MPC_5.csv', ...
    steps ...
);

% MPC3_kin_int_thelenberg_N_MPC_5 - Casadi
export_selected_columns_simus( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250602/traj1/MPC3_kin_int_thelenberg_N_MPC_5/robot_plots_20250603_110403.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter4/MPC3_kin_int_thelenberg_N_MPC_5.csv', ...
    steps ...
);

% MPC3_kin_int_parametric_N_MPC_5 - Casadi
export_selected_columns_simus( ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/messungen/250602/traj1/MPC3_kin_int_parametric_N_MPC_5/robot_plots_20250603_110607.csv', ...
    '/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/thesis_latex/Master-Thesis-2024-25/doc/tex/figures/chapter4/MPC3_kin_int_parametric_N_MPC_5.csv', ...
    steps ...
);

disp('Done!');


function export_selected_columns(input_csv, output_csv, steps)
    data = readtable(input_csv, 'VariableNamingRule', 'preserve');
    exportTable = table();
    exportTable.time = data.("t (s)")(1:steps:end);
    exportTable.e_x = 1e3*data.("e_x (m)")(1:steps:end);
    exportTable.e_y = 1e3*data.("e_y (m)")(1:steps:end);
    exportTable.e_z = 1e3*data.("e_z (m)")(1:steps:end);
    exportTable.quat_err_2 = data.("quat_err_2")(1:steps:end);
    exportTable.quat_err_3 = data.("quat_err_3")(1:steps:end);
    exportTable.quat_err_4 = data.("quat_err_4")(1:steps:end);
    exportTable.manip = data.("Manip. w = √( det( JJ⸆ ) )")(1:steps:end);
    exportTable.qd_1 = data.("q̇_1")(1:steps:end);
    exportTable.qd_2 = data.("q̇_2")(1:steps:end);
    exportTable.qd_3 = data.("q̇_3")(1:steps:end);
    exportTable.qd_4 = data.("q̇_4")(1:steps:end);
    exportTable.qd_5 = data.("q̇_5")(1:steps:end);
    exportTable.qd_6 = data.("q̇_6")(1:steps:end);
    exportTable.qd_7 = data.("q̇_7")(1:steps:end);
    exportTable.tau_1 = data.("tau_1")(1:steps:end);
    exportTable.tau_2 = data.("tau_2")(1:steps:end);
    exportTable.tau_3 = data.("tau_3")(1:steps:end);
    exportTable.tau_4 = data.("tau_4")(1:steps:end);
    exportTable.tau_5 = data.("tau_5")(1:steps:end);
    exportTable.tau_6 = data.("tau_6")(1:steps:end);
    exportTable.tau_7 = data.("tau_7")(1:steps:end);
    writetable(exportTable, output_csv);
end

function export_selected_columns_ID(input_csv, output_csv, steps)
    data = readtable(input_csv, 'VariableNamingRule', 'preserve');
    exportTable = table();
    exportTable.time = data.("t (s)")(1:steps:end-1999);
    exportTable.e_x = 1e3*data.("e_x (m)")(2000:steps:end);
    exportTable.e_y = 1e3*data.("e_y (m)")(2000:steps:end);
    exportTable.e_z = 1e3*data.("e_z (m)")(2000:steps:end);
    exportTable.quat_err_2 = data.("quat_err_2")(2000:steps:end);
    exportTable.quat_err_3 = data.("quat_err_3")(2000:steps:end);
    exportTable.quat_err_4 = data.("quat_err_4")(2000:steps:end);
    exportTable.manip = data.("Manip. w = √( det( JJ⸆ ) )")(2000:steps:end);
    exportTable.qd_1 = data.("q̇_1")(2000:steps:end);
    exportTable.qd_2 = data.("q̇_2")(2000:steps:end);
    exportTable.qd_3 = data.("q̇_3")(2000:steps:end);
    exportTable.qd_4 = data.("q̇_4")(2000:steps:end);
    exportTable.qd_5 = data.("q̇_5")(2000:steps:end);
    exportTable.qd_6 = data.("q̇_6")(2000:steps:end);
    exportTable.qd_7 = data.("q̇_7")(2000:steps:end);
    exportTable.tau_1 = data.("tau_1")(2000:steps:end);
    exportTable.tau_2 = data.("tau_2")(2000:steps:end);
    exportTable.tau_3 = data.("tau_3")(2000:steps:end);
    exportTable.tau_4 = data.("tau_4")(2000:steps:end);
    exportTable.tau_5 = data.("tau_5")(2000:steps:end);
    exportTable.tau_6 = data.("tau_6")(2000:steps:end);
    exportTable.tau_7 = data.("tau_7")(2000:steps:end);
    writetable(exportTable, output_csv);
end

function export_selected_columns_ID_crash(input_csv, output_csv, steps, zero_index)
    data = readtable(input_csv, 'VariableNamingRule', 'preserve');
    exportTable = table();

    mult = zeros(10001,1);
    mult(1:zero_index) = 1;
    mult = mult(1:steps:end);
    exportTable.time = data.("t (s)")(1:steps:end-1999);
    exportTable.e_x = 1e3*mult.*data.("e_x (m)")(2000:steps:end);
    exportTable.e_y = 1e3*mult.*data.("e_y (m)")(2000:steps:end);
    exportTable.e_z = 1e3*mult.*data.("e_z (m)")(2000:steps:end);
    exportTable.quat_err_2 = mult.*data.("quat_err_2")(2000:steps:end);
    exportTable.quat_err_3 = mult.*data.("quat_err_3")(2000:steps:end);
    exportTable.quat_err_4 = mult.*data.("quat_err_4")(2000:steps:end);
    exportTable.manip = mult.*data.("Manip. w = √( det( JJ⸆ ) )")(2000:steps:end);
    exportTable.qd_1 = mult.*data.("q̇_1")(2000:steps:end);
    exportTable.qd_2 = mult.*data.("q̇_2")(2000:steps:end);
    exportTable.qd_3 = mult.*data.("q̇_3")(2000:steps:end);
    exportTable.qd_4 = mult.*data.("q̇_4")(2000:steps:end);
    exportTable.qd_5 = mult.*data.("q̇_5")(2000:steps:end);
    exportTable.qd_6 = mult.*data.("q̇_6")(2000:steps:end);
    exportTable.qd_7 = mult.*data.("q̇_7")(2000:steps:end);
    exportTable.tau_1 = mult.*data.("tau_1")(2000:steps:end);
    exportTable.tau_2 = mult.*data.("tau_2")(2000:steps:end);
    exportTable.tau_3 = mult.*data.("tau_3")(2000:steps:end);
    exportTable.tau_4 = mult.*data.("tau_4")(2000:steps:end);
    exportTable.tau_5 = mult.*data.("tau_5")(2000:steps:end);
    exportTable.tau_6 = mult.*data.("tau_6")(2000:steps:end);
    exportTable.tau_7 = mult.*data.("tau_7")(2000:steps:end);
    writetable(exportTable, output_csv);
end

function export_selected_columns_crash(input_csv, output_csv, steps, zero_index)
    data = readtable(input_csv, 'VariableNamingRule', 'preserve');
    exportTable = table();

    mult = zeros(10000,1);
    mult(1:zero_index) = 1;
    mult = mult(1:steps:end);
    exportTable.time = data.("t (s)")(1:steps:end);
    exportTable.e_x = 1e3*mult.*data.("e_x (m)")(1:steps:end);
    exportTable.e_y = 1e3*mult.*data.("e_y (m)")(1:steps:end);
    exportTable.e_z = 1e3*mult.*data.("e_z (m)")(1:steps:end);
    exportTable.quat_err_2 = mult.*data.("quat_err_2")(1:steps:end);
    exportTable.quat_err_3 = mult.*data.("quat_err_3")(1:steps:end);
    exportTable.quat_err_4 = mult.*data.("quat_err_4")(1:steps:end);
    exportTable.manip = mult.*data.("Manip. w = √( det( JJ⸆ ) )")(1:steps:end);
    exportTable.qd_1 = mult.*data.("q̇_1")(1:steps:end);
    exportTable.qd_2 = mult.*data.("q̇_2")(1:steps:end);
    exportTable.qd_3 = mult.*data.("q̇_3")(1:steps:end);
    exportTable.qd_4 = mult.*data.("q̇_4")(1:steps:end);
    exportTable.qd_5 = mult.*data.("q̇_5")(1:steps:end);
    exportTable.qd_6 = mult.*data.("q̇_6")(1:steps:end);
    exportTable.qd_7 = mult.*data.("q̇_7")(1:steps:end);
    exportTable.tau_1 = mult.*data.("tau_1")(1:steps:end);
    exportTable.tau_2 = mult.*data.("tau_2")(1:steps:end);
    exportTable.tau_3 = mult.*data.("tau_3")(1:steps:end);
    exportTable.tau_4 = mult.*data.("tau_4")(1:steps:end);
    exportTable.tau_5 = mult.*data.("tau_5")(1:steps:end);
    exportTable.tau_6 = mult.*data.("tau_6")(1:steps:end);
    exportTable.tau_7 = mult.*data.("tau_7")(1:steps:end);
    writetable(exportTable, output_csv);
end

function export_selected_columns_simus(input_csv, output_csv, steps)
    data = readtable(input_csv, 'VariableNamingRule', 'preserve');
    exportTable = table();
    exportTable.time = data.("t (s)")(1:steps:end);
    exportTable.e_x = 1e6*data.("e_x (m)")(1:steps:end);
    exportTable.e_y = 1e6*data.("e_y (m)")(1:steps:end);
    exportTable.e_z = 1e6*data.("e_z (m)")(1:steps:end);
    exportTable.quat_err_2 = data.("quat_err_2")(1:steps:end);
    exportTable.quat_err_3 = data.("quat_err_3")(1:steps:end);
    exportTable.quat_err_4 = data.("quat_err_4")(1:steps:end);
    exportTable.manip = data.("Manip. w = √( det( JJ⸆ ) )")(1:steps:end);
    exportTable.qd_1 = data.("q̇_1")(1:steps:end);
    exportTable.qd_2 = data.("q̇_2")(1:steps:end);
    exportTable.qd_3 = data.("q̇_3")(1:steps:end);
    exportTable.qd_4 = data.("q̇_4")(1:steps:end);
    exportTable.qd_5 = data.("q̇_5")(1:steps:end);
    exportTable.qd_6 = data.("q̇_6")(1:steps:end);
    exportTable.qd_7 = data.("q̇_7")(1:steps:end);
    exportTable.tau_1 = data.("tau_1")(1:steps:end);
    exportTable.tau_2 = data.("tau_2")(1:steps:end);
    exportTable.tau_3 = data.("tau_3")(1:steps:end);
    exportTable.tau_4 = data.("tau_4")(1:steps:end);
    exportTable.tau_5 = data.("tau_5")(1:steps:end);
    exportTable.tau_6 = data.("tau_6")(1:steps:end);
    exportTable.tau_7 = data.("tau_7")(1:steps:end);
    exportTable.freq_per_Ta_step = data.("freq_per_Ta_step")(1:steps:end);
    writetable(exportTable, output_csv);
end