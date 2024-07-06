if(bdIsLoaded(simulink_main_model_name))
    robot_blk_name = [simulink_main_model_name, '/robot name combo box'];
    % get all possible robot names e. g. fr3_7dof, fr3_6dof, ur5e_6dof from Labels of combo box,:
    combo_states = get_param(robot_blk_name, 'States');
    combo_states_name = {combo_states.Label};
    % get current selected value (a number)
    current_value = str2double(get_param(robot_blk_name, 'Value'));
    % find name to selected value
    idx = find(current_value == [combo_states.Value]);
    robot_name = combo_states_name{idx};
else % default value (e. g. for mpc compiling when simulink is closed)
    % use simulink robot name or use default name here defined:
    load('./utils/matlab_init_general/old_robot_name.mat');

    robot_name = robot_name_old;
    % robot_name = 'fr3_7dof';
    % robot_name = 'fr3_6dof';
    % robot_name = 'ur5e_6dof';
end

disp(['Selected robot: ', robot_name]);