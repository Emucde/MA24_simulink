function param_traj_data_out = param_traj_data_fun(traj_settings, method, data_index, param_traj_data_in, set_data)
    arguments
        traj_settings struct
        method string {mustBeMember(method, {'init', 'set', 'get'})}
        data_index double {mustBeInteger} = 1
        param_traj_data_in struct = struct
        set_data struct = struct
    end

    % important: fields should not contain t
    fields = fieldnames(traj_settings.x_d);
    if(strcmp(method, 'init'))
        % Create high dimensional struct from the trajectory bus
        
        bus_temp_struct = struct;
        bus_temp_struct.N = traj_settings.N_data_real;
        bus_temp_struct.t = traj_settings.t;
        for i = 1:numel(fields)
            val = traj_settings.x_d.(fields{i});
            if(size(val, 2) == 1)
                bus_temp_struct.(fields{i}) = repmat(val, 1, traj_settings.N_data, traj_settings.N_traj);
            else
                bus_temp_struct.(fields{i}) = repmat(val, 1, 1, traj_settings.N_data, traj_settings.N_traj);
            end
        end
        param_traj_data_out = bus_temp_struct;
    elseif(strcmp(method, 'set'))
        % Set the data of the trajectory
        param_traj_data_out = param_traj_data_in;
        for i = 1:numel(fields)
            val = param_traj_data_in.(fields{i});
            if(size(val, 4) == 1)
                val(:, :, data_index) = set_data.(fields{i});
            else
                val(:, :, :, data_index) = set_data.(fields{i});
            end
            param_traj_data_out.(fields{i}) = val;
        end
    elseif(strcmp(method, 'get'))
        % Outputs a single trajectory
        param_traj_data_out = struct;
        for i = 1:numel(fields)
            val = param_traj_data_in.(fields{i});
            if(size(val, 4) == 1)
                param_traj_data_out.(fields{i}) = val(:, :, data_index);
            else
                param_traj_data_out.(fields{i}) = val(:, :, :, data_index);
            end
        end
    else
        error('method have to be init, set or get');
    end
end