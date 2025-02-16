double_sing_pose = csvread('./main_c/double_pose_singularities2.csv');

qq1 = double_sing_pose(:, 1:7);
qq2 = double_sing_pose(:, 8:14);

qq_all = [qq1; qq2];

% write back to file but 7 dim:
csvwrite('./main_c/double_pose_singularities_7dim.csv', qq_all);

% calculate pose distances
sorted_pose = zeros(size(qq_all, 1), 7);
pose_distances = zeros(size(qq_all, 1), 1);
sorted_pose(1, :) = qq_all(1, :);
for i=1:size(qq_all, 1)
    pose_i = sorted_pose(i, :);
    pose_distances_temp = zeros(size(qq_all, 1)-i, 1);
    cnt = 1;
    for j=1:size(qq_all, 1)
        pose_j = qq_all(j, :);
        pose_distances_temp(cnt) = norm(pose_i - pose_j, 2);
        cnt = cnt+1;
    end
    [pose_distances(i+1), min_idx] = min(pose_distances_temp);;
    % remove min_idx from qq_all
    sorted_pose(i+1, :) = qq_all(min_idx, :);
    qq_all = qq_all([1:min_idx-1, min_idx+1:end], :);

    if mod(i, 100) == 0
        disp(['Iteration: ', num2str(i)]);
        fprintf('Pose %d, pose_distances: %g\n', i, pose_distances(i));
    end
end