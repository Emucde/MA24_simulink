% Change the combo box of the simulink model to the desired trajectory
% simulink file "sim_discrete_7dof.slx" must be loaded and opened
% get_param(simulink_main_model_name, 'ObjectParameters') % show all

if(bdIsLoaded(simulink_main_model_name))

    traj_blk_name = [simulink_main_model_name, '/trajectory combo box'];

    % Change name of Simulation Block
    % If robots are added this should be extended!
    robot_area_names = {'fr3_7dof Robot Simulation', 'fr3_6dof Robot Simulation', 'ur5e_6dof Robot Simulation'};
    for old_robot_number = 1:length(robot_area_names)
        try
            get_param(['sim_discrete_7dof/', robot_area_names{old_robot_number}], 'Name');
            break
        catch
            if(old_robot_number == length(robot_area_names))
                error('No robot simulation block found!')
            end
        end
    end

    % get trajectory names
    N_traj = length(param_traj_cell);
    traj_state_cell = cell(1, N_traj);

    for i=1:1:N_traj
        traj_name = param_traj_cell{i}.name;
        traj_state_cell{i} = struct('Value', i, 'Label', [num2str(i), ': ', traj_name]);
    end

    current_traj_value = str2double(get_param(traj_blk_name, 'Value'));
    if(current_traj_value > N_traj)
        set_param(traj_blk_name, 'Value', 1);
    end

    if(strcmp(robot_name, 'fr3_7dof'))
        %traj_state_cell = cell(1, 4);
        %traj_state_cell{1} = struct('Value', 1, 'Label', '1: stabilize equilibrium');
        %traj_state_cell{2} = struct('Value', 2, 'Label', '2: 5th order differential filter');
        %traj_state_cell{3} = struct('Value', 3, 'Label', '3: 5th order polynomial');
        %traj_state_cell{4} = struct('Value', 4, 'Label', '4: smooth sinus');
        set_param(['sim_discrete_7dof/', robot_area_names{old_robot_number}], 'Name', 'fr3_7dof Robot Simulation');
    elseif(strcmp(robot_name, 'fr3_6dof'))
        %traj_state_cell = cell(1, 4);
        %traj_state_cell{1} = struct('Value', 1, 'Label', '1: stabilize equilibrium');
        %traj_state_cell{2} = struct('Value', 2, 'Label', '2: 5th order differential filter');
        %traj_state_cell{3} = struct('Value', 3, 'Label', '3: 5th order polynomial');
        %traj_state_cell{4} = struct('Value', 4, 'Label', '4: smooth sinus');
        set_param(['sim_discrete_7dof/', robot_area_names{old_robot_number}], 'Name', 'fr3_6dof Robot Simulation');
    elseif(strcmp(robot_name, 'ur5e_6dof'))     
        set_param(['sim_discrete_7dof/', robot_area_names{old_robot_number}], 'Name', 'ur5e_6dof Robot Simulation');
       
    else
        error('Only robot_name ( fr3_7dof | fr3_6dof | ur5e_6dof ) implemented!');
    end

    % set new names when different
    traj_combo_states_old = get_param(traj_blk_name, 'States');
    traj_combo_states_new = [traj_state_cell{:}]';

    if(~isequal(traj_combo_states_old, traj_combo_states_new))
        set_param(traj_blk_name, 'States', traj_combo_states_new);
    end

    save_pending_state = get_param(simulink_main_model_name, 'Dirty');
    if(strcmp(save_pending_state, 'on'))
        save_system(simulink_main_model_name, 'SaveDirtyReferencedModels','on');
    end

end