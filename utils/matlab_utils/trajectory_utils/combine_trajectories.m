function traj_struct_combined = combine_trajectories(traj_cell)
%COMBINE_TRAJECTORIES Combines multiple trajectories into a single struct
%
% This function takes a cell array of trajectory structs and combines them
% into a single struct, ensuring that the dimensions of the pose,
% rotation, and time arrays match for each trajectory.
%
% Input:
%   traj_cell - A cell array of trajectory structs, where each struct
%               contains the following fields:
%                   - pose: A 7xN array of pose
%                   - rotation: A 3x3xN array of rotation
%                   - time: A 1xN array of time pose
%                   - traj_type: The type of trajectory
%                   - N: The number of pose in the trajectory
%
% Output:
%   traj_struct_combined - A struct containing the combined trajectories,
%                          with the following fields:
%                   - start_index: A 1xN_traj array of start indices for each trajectory
%                   - stop_index: A 1xN_traj array of stop indices for each trajectory
%                   - pose: A 7xN_total array of combined pose
%                   - rotation: A 3x3xN_total array of combined rotation
%                   - time: A 1xN_total array of combined time pose
%                   - trajectory_type: A 1xN_traj array of trajectory types
%                   - N_traj: The number of trajectories
%
% where N_total is the total number of pose across all trajectories.
%
% The function checks that the dimensions of the pose, rotation, and time
% arrays match for each trajectory, and throws an error if they don't.
%

    % Check each trajectory and calculate total number of pose
    N_total = 0;
    N_traj = length(traj_cell);
    for i = 1 : N_traj
        traj_struct = traj_cell{i};
        N = traj_struct.N;

        if(size(traj_struct.pose, 2) ~= N)
            error('Number of poses does not match number of poses');
        elseif(size(traj_struct.rotation, 3) ~= N)
            error('Number of rotation does not match number of poses');
        elseif(length(traj_struct.time) ~= N)
            error('Number of times does not match number of poses');
        end
        N_total = N_total + N;
    end

    % Initialize arrays
    traj_struct_combined = struct;
    traj_struct_combined.start_index = zeros(1, N_traj);
    traj_struct_combined.stop_index  = zeros(1, N_traj);
    traj_struct_combined.pose        = zeros(7, N_total);
    traj_struct_combined.rotation    = zeros(3, 3, N_total);
    traj_struct_combined.rot_ax      = zeros(3, N_total);
    traj_struct_combined.alpha       = zeros(1, N_total);
    traj_struct_combined.time        = zeros(1, N_total);
    traj_struct_combined.traj_type   = zeros(1, N_traj);
    traj_struct_combined.N_traj      = N_traj;

    % Combine all trajectories
    start_index = 1;
    for i = 1 : N_traj
        traj_struct = traj_cell{i};
        N = traj_struct.N;

        stop_index = (start_index - 1) + N;

        traj_struct_combined.start_index(i) = start_index;
        traj_struct_combined.stop_index(i) = stop_index;
        traj_struct_combined.pose(:, start_index:stop_index) = traj_struct.pose;
        traj_struct_combined.rotation(:, :, start_index:stop_index) = traj_struct.rotation;
        traj_struct_combined.time(start_index:stop_index) = traj_struct.time;
        traj_struct_combined.traj_type(i) = traj_struct.traj_type;

        for j = 1:N
            t_val = traj_struct.time;
            index = (start_index - 1) + j;
            if(j == 1)
                rot_ax = [0; 0; 0];
                alpha = 0;
            else
                rotation = traj_struct.rotation;
                [rot_ax, rot_alpha_scale] = find_rotation_axis(rotation(:, :, j-1), rotation(:, :, j), "slow_from_quat");
                alpha = rot_alpha_scale + alpha;
            end
            traj_struct_combined.rot_ax(:, index) = rot_ax;
            traj_struct_combined.alpha( 1, index) = alpha;
        end

        start_index = stop_index + 1;
    end
end