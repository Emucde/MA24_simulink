% activate_simulink_logs

% get_param(blk_name, 'ObjectParameters');
if(bdIsLoaded(simulink_main_model_name))
    blk_name = [simulink_main_model_name, '/trajectory combo box'];
    combo_states = get_param(blk_name, 'States');
    combo_states_name = {combo_states.Label};
    current_value = str2double(get_param(blk_name, 'Value'));
    idx = find(current_value == [combo_states.Value]);
    selected_box_label = combo_states_name{idx};
    split_label = strsplit(selected_box_label, ' ');
    fin_label_traj = strjoin([split_label(2:end)], '_');

    blk_name = [simulink_main_model_name, '/controller combo box'];
    combo_states = get_param(blk_name, 'States');
    combo_states_name = {combo_states.Label};
    current_value = str2double(get_param(blk_name, 'Value'));
    idx = find(current_value == [combo_states.Value]);
    selected_box_label = combo_states_name{idx};
    split_label = strsplit(selected_box_label, ' ');
    fin_label_ctrl = strjoin(split_label, '_');

    % clear diagnostic window from simulink
    save_system(simulink_main_model_name)
    sldiagviewer.diary('off'); % wichtig, sonst loggt er auch in die vorherigen files

    sldiagviewer.diary(['./main_simulink/simulink_log/', char(datetime('now', 'Format', 'yyMMdd_HH_mm')), '_', fin_label_ctrl, '_', fin_label_traj, '_log.txt']); % enable logger for simulink
    disp('______________________________________________________________________')
    disp(['Selected controller: ', fin_label_ctrl, ', Selected trajectory: ', fin_label_traj]);
end