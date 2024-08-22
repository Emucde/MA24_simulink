if(bdIsLoaded(simulink_main_model_name))

    %% Kosmetische Einstellungen (Path Coords)
    path_point_size = 1;
    path_point_color = [0.5 0.5 0.5];
    path_point_opacity = 1.0;

    % path coords len
    path_coord_xaxis_len = 2; %in cm
    path_coord_yaxis_len = 2; %in cm
    path_coord_zaxis_len = 2; %in cm

    % path coords style
    path_coord_xaxis_color = [1 0.0 0.0]; % red
    path_coord_xaxis_opacity = 1/3;
    path_coord_yaxis_color = [0 1 0.0]; % green
    path_coord_yaxis_opacity = 1/3;
    path_coord_zaxis_color = [0 0.0 1]; % blue
    path_coord_zaxis_opacity = 1/3;

    % path frame coord len
    path_frame_coord_xaxis_len = 2; %in cm
    path_frame_coord_yaxis_len = 2; %in cm
    path_frame_coord_zaxis_len = 2; %in cm

    % path frame coord style
    path_frame_coord_xaxis_color = [0.7 0 0]; % dark red
    path_frame_coord_xaxis_opacity = 1;
    path_frame_coord_yaxis_color = [0 0.7 0]; % dark green
    path_frame_coord_yaxis_opacity = 1;
    path_frame_coord_zaxis_color = [0 0 0.7]; % dark blue
    path_frame_coord_zaxis_opacity = 1;

    path_display_update_time = param_global.Ta;

    % Kugeln an path coords
    path_frame_sphere_radius = 2.5; %in cm
    path_frame_sphere_color = [1.0 1.0 0.0]; % yellow
    path_frame_sphere_opacity = 0.2;


    % current edge points
    if(param_traj.traj_type(current_traj_value) == traj_mode.polynomial_jointspace || param_traj.traj_type(current_traj_value) == traj_mode.differential_filter_jointspace)
        joint_points = param_traj.joint_points(:,param_traj.start_index(current_traj_value):param_traj.stop_index(current_traj_value));
        edge_points = zeros(3, size(joint_points, 2));
        for i=1:size(joint_points, 2)
            H = hom_transform_endeffector_py(joint_points(:,i));
            edge_points(:, i) = H(1:3,4);
        end
    else
        edge_points = param_traj.pose(1:3,param_traj.start_index(current_traj_value):param_traj.stop_index(current_traj_value));
    end

    % koordinate system for trajectory
    N_timesteps = 20;
    N_total = length(param_traj_data.t);

    traj_indices = round(linspace(1,N_total,N_timesteps));

    % current_traj_value is defined in change_simulink_traj_combo_box.m
    paths_set1 = zeros(N_timesteps,7,1);
    paths_set1(:,1:3,1) = param_traj_data.p_d(:, traj_indices, current_traj_value)';
    paths_set1(:,4:7,1) = param_traj_data.q_d(:, traj_indices, current_traj_value)';

    param_path_set_points.paths_set1 = paths_set1;

    % Wichtig: Die Punkte müssen ein zwei Kommastellen eindeutig sein!
    point_dist = 0.01; % in m
    start_point = param_traj_data.p_d(:, 1, current_traj_value);
    current_traj_data = zeros(size(param_traj_data.p_d(:, :, current_traj_value)));
    current_t = zeros(size(param_traj_data.t));
    current_traj_data(:, 1) = start_point;
    cnt=2;
    for i=2:N_total
        next_point = param_traj_data.p_d(:, i, current_traj_value);
        next_t = param_traj_data.t(i);
        dist = norm(next_point - start_point, 2);
        if(dist > point_dist)
            current_traj_data(:, cnt) = next_point;
            current_t(cnt) = next_t;
            start_point = next_point;
            cnt=cnt+1;
        end
    end

    cnt=cnt-1;

    edge_points_time = param_traj.time(param_traj.start_index(current_traj_value):param_traj.stop_index(current_traj_value));
    current_traj_data(:, 1) = edge_points(:, 1);
    N_total_new = length(current_traj_data);
    for i=1:length(edge_points)
        [~,idx] = min(abs(current_t - edge_points_time(i)));

        current_traj_data(:, idx) = edge_points(:, i);
    end

    current_traj_data = current_traj_data(:, 1:cnt)';

    % remove duplicates
    N_curr_points = size(current_traj_data, 1);
    current_traj_data_fin = zeros(size(current_traj_data));
    current_traj_data_fin(1,:) = current_traj_data(1,:);
    start_point = current_traj_data_fin(1,:);
    cnt = 2;
    for i=2:N_curr_points
        next_point = current_traj_data(i,:);
        dist = norm(next_point - start_point, 2);
        if(dist > 1e-6)
            current_traj_data_fin(cnt,:) = next_point;
            cnt=cnt+1;
        end
        start_point = next_point;
    end
    current_traj_data = current_traj_data_fin(1:cnt-1,:);

    %{
    N_total_new = length(current_traj_data);
    cnt = 2;
    old_dist = norm(current_traj_data(1,:) - edge_points(cnt));
    dist_smaller_flag = 0;
    for i=1:N_total_new
        next_point = current_traj_data(i,:);
        dist = norm(next_point - edge_points(cnt));
        if(dist < old_dist)
            dist_smaller_flag = 1;
        end

        if(dist > old_dist && dist_smaller_flag == 1)
            current_traj_data(i,:) = edge_points(cnt);
            cnt = cnt+1;
            old_dist = dist;
        end

        if(cnt > size(edge_points, 2))
            break;
        end
    end
    %}
end