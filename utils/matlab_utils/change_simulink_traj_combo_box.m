% Change the combo box of the simulink model to the desired trajectory
% simulink file "sim_discrete_7dof.slx" must be loaded and opened

if(bdIsLoaded(simulink_main_model_name))

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

    if(strcmp(robot_name, 'fr3_7dof'))
        traj_state_cell = cell(1, 4);
        traj_state_cell{1} = struct('Value', 1, 'Label', '1: stabilize equilibrium');
        traj_state_cell{2} = struct('Value', 2, 'Label', '2: 5th order differential filter');
        traj_state_cell{3} = struct('Value', 3, 'Label', '3: 5th order polynomial');
        traj_state_cell{4} = struct('Value', 4, 'Label', '4: smooth sinus');
        set_param(['sim_discrete_7dof/', robot_area_names{old_robot_number}], 'Name', 'fr3_7dof Robot Simulation');
    elseif(strcmp(robot_name, 'fr3_6dof'))
        traj_state_cell = cell(1, 4);
        traj_state_cell{1} = struct('Value', 1, 'Label', '1: stabilize equilibrium');
        traj_state_cell{2} = struct('Value', 2, 'Label', '2: 5th order differential filter');
        traj_state_cell{3} = struct('Value', 3, 'Label', '3: 5th order polynomial');
        traj_state_cell{4} = struct('Value', 4, 'Label', '4: smooth sinus');
        set_param(['sim_discrete_7dof/', robot_area_names{old_robot_number}], 'Name', 'fr3_6dof Robot Simulation');
    elseif(strcmp(robot_name, 'ur5e_6dof'))
        traj_state_cell = cell(1, 4);
        traj_state_cell{1} = struct('Value', 1, 'Label', '1: stabilize equilibrium');
        traj_state_cell{2} = struct('Value', 2, 'Label', '2: 5th order differential filter');
        traj_state_cell{3} = struct('Value', 3, 'Label', '3: 5th order polynomial');
        traj_state_cell{4} = struct('Value', 4, 'Label', '4: smooth sinus');
        set_param(['sim_discrete_7dof/', robot_area_names{old_robot_number}], 'Name', 'ur5e_6dof Robot Simulation');
    else
        error('Only robot_name ( fr3_7dof | fr3_6dof | ur5e_6dof ) implemented!');
    end

    % set new names when different
    traj_blk_name = [simulink_main_model_name, '/trajectory combo box'];
    traj_combo_states_old = get_param(traj_blk_name, 'States');
    traj_combo_states_new = [traj_state_cell{:}]';

    if(~isequal(traj_combo_states_old, traj_combo_states_new))
        set_param(traj_blk_name, 'States', traj_combo_states_new);
    end

end